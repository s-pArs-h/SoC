# Design notes

Why the SoC is built the way it is, what each choice costs, and how it is
verified. The two reused blocks (the RV32I pipeline and the K-means core)
have their own design notes in their repositories.

## 1. Division of labour

Lloyd's K-means alternates two steps:

1. **Assignment**: every point goes to its nearest centroid. This is N x K
   squared distances per iteration: almost all of the arithmetic, perfectly
   regular, and a natural fit for a pipeline.
2. **Update**: each centroid moves to the mean of its points. This is K
   divisions per coordinate per iteration, irregular control (empty
   clusters, convergence test), and cheap compared with step 1.

The accelerator does step 1 and also accumulates the per-cluster sums and
counts as results stream out, so the CPU never touches the points during an
iteration. The CPU does step 2, the convergence test and all communication.

## 2. Bus: single-cycle slaves, no wait states

The interconnect reuses the pipelined core's data port unchanged: in the
cycle a load or store is in EX the core drives the address, read strobe or
byte write mask and write data, and expects read data in the next cycle.

Every slave is built to answer within that cycle: read data is registered
(like a block RAM), writes take effect on the clock edge. The interconnect
is then only:

* a decoder on address bits 31:28 producing one select per slave, and
* a read-data multiplexer steered by the region registered with the request.

**Trade-off.** This is the simplest possible bus and adds no cycles, but it
only works because every slave is fast. A slave that needed several cycles
(external DRAM, a slow peripheral) would need a ready/stall signal back to
the pipeline, which the core does not have. The path to a standard bus is a
bridge to AXI4-Lite with a stall on the core's memory stage; the slaves'
register maps would not change.

**Side-effect reads.** Reading UART RXDATA removes the byte from the FIFO.
That is only safe because this core never issues a load speculatively and
never re-issues one: loads issue from EX, and a trap or redirect can only
squash younger instructions that have not reached EX yet.

## 3. Accelerator integration: local point memory, not DMA

Two ways to feed the accelerator were considered:

| | Local point memory (chosen) | DMA from main RAM |
|---|---|---|
| Bus traffic per iteration | none (data loaded once) | N words |
| Points per cycle | 1, every iteration | at most 1, competing with the CPU for the RAM port |
| Extra memory | 16 KiB point + 1 KiB label memory | none |
| Complexity | streamer + FIFO | DMA engine, bus arbitration |

K-means re-reads the same data every iteration, so keeping it next to the
accelerator wins: after one load over the bus, each iteration streams at one
point per clock with no contention. The cost is block RAM, which the FPGA
has plenty of.

## 4. The streamer: credit-based flow control around memory latency

The point memory has a one-cycle read latency, and the K-means core can
stall (its valid/ready input). A naive "read when the core is ready" design
either wastes a cycle per point or loses data when the core stalls after a
read was issued.

The streamer issues a read only when the FIFO is guaranteed to have room for
the data when it arrives:

```
issue = points_left && (fifo_count + read_in_flight < FIFO_DEPTH)
```

Reads in flight are "credits" already spent. With a depth of 4 the FIFO sits
at about one entry in steady state and passes one point per cycle; when the
core stalls, issuing stops by itself and no read is ever dropped.

This is proven: k-induction shows `fifo_count + read_in_flight <= depth`
holds forever for any bus activity, which implies the FIFO can always accept
the returning data. Removing the credit check makes the proof fail with a
counterexample.

## 5. UART and the host protocol

* **UART**: 8N1, divider-based baud rate (115200 at 50 MHz: divider 434,
  0.006% error). The receiver synchronises the asynchronous input with two
  flip-flops, re-checks the start bit half a bit later (glitch filter) and
  samples each bit in its middle. 16-byte FIFOs on both sides.
* **Single-load polling**: RXDATA returns the byte and an "empty" flag in one
  word, so polling costs one load per byte, not two.
* **Protocol**: framed, length-prefixed, CRC-16/CCITT protected, one response
  per request. The CRC uses a 512-byte constant table, so checking a byte
  costs a handful of instructions and the firmware keeps up with the line.
* **Large payloads**: LOAD streams straight into the accelerator's point
  memory as bytes arrive, so a 16 KiB data set never needs to fit in the
  16 KiB RAM. The points are written before the CRC is known, so the
  firmware marks the data set empty first and only accepts it once the CRC
  matches; a corrupted load leaves zero points, never a half-written set.

### A bug the system test found: start-up versus the first request

The first version built the CRC table at start-up, and the first long
request after a reset timed out: its bytes arrived while the firmware was
still busy, and the 16-byte receive FIFO overflowed. Making the table a
constant shortened start-up but did not fix it: clearing `.bss` (4.6 KiB)
still takes about 6,000 cycles. The real fix is in the protocol: a session
starts with INFO, which is short enough to wait in the receive FIFO, and
data is only sent after the board has answered. Both the host program and
the test bench do this, with retries.

## 6. Firmware

* No C library: start-up code clears `.bss` and calls `main`; `memset` and
  `memcpy` are provided; libgcc supplies multiply and divide (RV32I has
  neither instruction).
* **No initialised writable data** (the linker script enforces it): the RAM
  image is loaded when the FPGA is configured, so after a reset button press
  any initialised variable would still hold its last value. Everything is
  set in code.
* The software assignment step is exact for the full 16-bit range: absolute
  differences fit 16 bits, their squares 32 bits unsigned, the sum 33 bits.
  Ties go to the lowest cluster index, exactly like the accelerator's
  arg-min tree, so the two modes give identical labels.
* Rounding: new centroid = mean rounded to nearest, halves away from zero.
  The Python reference model uses the same rule, so results are compared
  exactly, not with a tolerance.

## 7. Clocking and reset on the board

The 50 MHz system clock comes from the MMCM through a global buffer. Reset
is asserted asynchronously from the button or while the MMCM is not locked,
and released synchronously by a two-flop synchroniser. The UART input and
switches are synchronised inside the design and declared as false paths in
the constraints. All 37 pin assignments are generated from Digilent's
master XDC by `tools/gen_xdc.py`.

## 8. Verification strategy

| Layer | Purpose |
|---|---|
| Reused blocks | verified in their own repositories (riscv-formal, riscv-tests, random lockstep; cocotb, exhaustive datapath, formal) |
| FIFO proof | data integrity for every word, by k-induction: a solver-chosen position and value is tracked from write to read |
| Accelerator proof | streamer credit invariant and run bounds for any bus activity, plus the K-means core's own invariants |
| Bounded check | every run starts drained; this property links the streamer to the core's internal count, so it is checked for every run sequence within 30 cycles rather than proven |
| System test | firmware on the RTL, driven over the UART pins with the same protocol code as the PC, compared bit-exactly with a reference model; every error path; coverage closure |
| Host tests | the host program end to end against a software stand-in, so it is tested before it ever meets the board |
| Mutation checks | each layer was shown to catch a deliberately injected bug |

The system test is deliberately "black box": it only uses the UART, exactly
like the PC. A failure there means the board would fail too.

## 9. Where the time goes now

Per iteration with 64 points (measured in simulation):

| Part | Cycles | Share |
|---|---|---|
| Accelerator busy (64 points + 8 fill) | 72 | 4% |
| CPU: program centroids, start, poll, read back results | 123 | 7% |
| CPU: update step (8 software divisions) and bookkeeping | 1,657 | 89% |

The accelerator made the assignment step 497 times faster, and the update
step now dominates: Amdahl's law. The options, in increasing cost:

1. **RV32M** (multiply/divide instructions) in the core: helps the update
   and the software baseline alike.
2. **A small sequential divider** in the accelerator: it already has the
   sums and counts, so it could output the new centroids directly and
   test convergence itself; the CPU would only start runs.
3. **Overlap**: double-buffer the centroid registers so the next pass starts
   while the CPU still reads the previous results.

## 10. Questions to be ready for

* Walk through one Lloyd iteration: which block does what, and what crosses
  the bus?
* Why does the bus need no wait states? What would you change to add a slow
  peripheral?
* Why is reading RXDATA with a side effect safe on this core? When would it
  not be?
* Why a local point memory instead of DMA? What does it cost?
* Explain the streamer's credit check. What breaks without it, and how do
  you know it is correct?
* Why can the "starts drained" property not be proven with the same
  k-induction, and what would make it provable?
* How does the firmware stay correct after a reset when the RAM is not
  reloaded?
* What did the start-up bug teach you about protocol design?
* The whole run is 53x faster but the assignment step 497x: why, and what
  would you do next?
* How does the system test know the answer is right?
