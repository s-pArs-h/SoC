# K-means SoC on an FPGA

A small system-on-chip that runs the complete K-means clustering algorithm
on a Digilent Nexys A7 (Artix-7) FPGA:

* a **5-stage pipelined RV32I RISC-V core** (from [RISC_V](https://github.com/s-pArs-h/RISC_V))
  runs C firmware that implements Lloyd's algorithm;
* the **K-means accelerator** (from [KMeans](https://github.com/s-pArs-h/KMeans)),
  wrapped with its own point and label memories, does the assignment step at
  one point per clock;
* a **UART** connects to a PC, where a Python program loads a data set,
  starts runs, reads the results back and checks them;
* system control provides LEDs, switches and a cycle counter.

Everything sits on one memory-mapped bus. The same algorithm can run with
the assignment step on the accelerator or entirely in software on the same
core, so the speed-up is measured on identical work. Design rationale and
trade-offs: [docs/DESIGN.md](docs/DESIGN.md).

## Results

Cycle-accurate RTL simulation of the whole SoC with the real firmware,
64 points, K = 4, 9 iterations to convergence (`make sim`, `tb/speedup.txt`):

| | Accelerator | Software on the RV32I core | Speed-up |
|---|---|---|---|
| Assignment step, all iterations | 1,755 cycles | 872,520 cycles | **497x** |
| Whole run (assignment + update + control) | 16,672 cycles | 887,432 cycles | **53x** |

Both modes produce bit-identical centroids, counts, SSE and per-point labels,
equal to the Python reference model.

* The accelerator itself is busy 72 cycles per pass (64 points + 8 cycles of
  pipeline fill); the remaining ~120 cycles per pass are the CPU programming
  the centroids, polling for completion and reading back the sums, counts
  and SSE.
* RV32I has no multiply instruction, so each squared distance in software is
  two library calls; that is a large part of the 497x.
* **Amdahl's law in practice**: once the assignment step is accelerated, the
  software update step (8 divisions per iteration, no divide instruction)
  takes 90% of the run. That bounds the whole-run speed-up at 53x and is the
  next thing to move into hardware (see [docs/DESIGN.md](docs/DESIGN.md#9-where-the-time-goes-now)).

## Architecture

```
                       +------------------------------+
   USB-UART  <-------> | UART  (TX/RX FIFOs, 115200)  |  0x2000_0000
   (PC)                +------------------------------+
                                     ^
  +--------------------+             |
  | RV32I 5-stage      |  data bus   |      +---------------------------------+
  | pipeline           |-------------+----->| K-means accelerator             |  0x1000_0000
  | (riscv_pipeline)   |             |      |  registers, centroids           |
  +--------------------+             |      |  point memory --> streamer      |
     | instruction fetch             |      |     FIFO --> kmeans_core -------+--> per-cluster
     v                               |      |  label memory <-- results       |    sums, counts, SSE
  +--------------------+             |      +---------------------------------+
  | RAM 16 KiB         |<------------+
  | dual port (BRAM)   |  0x0000_0000 |      +---------------------------------+
  +--------------------+             +----->| system: LEDs, switches,         |  0x3000_0000
                                            | 64-bit cycle counter, CLK_HZ    |
                                            +---------------------------------+
```

* **Bus**: the core's own data port. Every slave answers in one cycle with
  registered read data, so the interconnect is just an address decoder and a
  read-data multiplexer, with no wait states.
* **Accelerator wrapper**: the data set is loaded once into a local point
  memory. For each run a streamer reads it through the second memory port
  into a small FIFO with credit-based flow control, the core consumes one
  point per cycle, and every result is written to a label memory.
* **Firmware**: no C library, 8 KiB of code, interrupt-free polling loop.
  It answers framed, CRC-protected requests from the PC.

## Memory map

| Address | Block | Contents |
|---|---|---|
| `0x0000_0000` | RAM | 16 KiB: code, data, stack |
| `0x1000_0000` | K-means | `+0x000` INFO, `+0x004` CTRL, `+0x008` STATUS, `+0x00C` NUM_POINTS, `+0x010` CYCLES, `+0x014/018` SSE, `+0x100 + 4k` CENTROID[k], `+0x200 + 16k` SUM_X / SUM_Y / COUNT[k] |
| `0x1001_0000` | K-means | point memory, one word per point `{y, x}`, 4096 points |
| `0x1002_0000` | K-means | label memory, one byte per point |
| `0x2000_0000` | UART | TXDATA, RXDATA (data + empty flag in one load), STATUS, DIV |
| `0x3000_0000` | system | LED, SW, CYCLE_LO / CYCLE_HI, CLK_HZ |

Register details are in the header comment of each RTL file and in
[sw/soc.h](sw/soc.h).

## Host protocol

Requests and responses are frames with a sync byte, command, length,
payload and CRC-16/CCITT ([sw/protocol.h](sw/protocol.h),
[host/protocol.py](host/protocol.py)):

| Command | Request | Response |
|---|---|---|
| INFO | | K, data width, point capacity, RAM size, clock frequency |
| LOAD | up to 4096 points (int16 x, y) | points stored |
| RUN | mode (accelerator / software), max iterations, K starting centroids | iterations, converged, centroids, counts, SSE, cycle counts |
| LABELS | offset, count | the cluster of each point after the last run |

Every request gets exactly one response, with a status: OK, CRC error, bad
length, unknown command, bad parameter, or timeout (a frame that stops
arriving part way).

## Verification

| Check | What it shows | Result |
|---|---|---|
| **System test** (cocotb + Icarus, `make sim`) | The real firmware runs on the whole RTL; the test talks to it over the UART pins with the same protocol code the PC uses, and compares every centroid, count, SSE value and label with a bit-exact Python model, in both modes | 6 tests pass; 18/18 coverage bins (incl. every error status, an empty cluster, distances above 2^32, a full point memory, a load with a bad CRC) |
| **Formal, FIFO** (SymbiYosys + Yices) | For any write position and value, the word stays queued unchanged and comes out at its turn; flags match the count; the count moves by exactly one per push or pop | unbounded proof, depth 4 and 16 |
| **Formal, accelerator** | With the bus driven arbitrarily: the streamer never overfills its FIFO or reads past the end of a run; the K-means core's control invariants hold | unbounded proof |
| | Every run starts with the streamer drained (nothing left from the previous run) | bounded, 30 cycles; a back-to-back run is reachable at cycle 13 |
| **Host tests** (pytest) | Protocol, CRC check value, rounding, the host program end to end against a software stand-in for the board | 8 tests pass |
| **Lint** | Verilator `-Wall`, every define combination, and the board top | clean |
| **Mutation checks** | FIFO overwriting when full: FIFO proof fails. Streamer without its credit check: accelerator proof fails. Labels written one slot off: system test fails. | caught |

The reused blocks are verified in their own repositories: the core with
riscv-formal (44/44 checks), riscv-tests and random lockstep simulation;
the K-means core with cocotb, an exhaustive datapath test and formal proofs.

## Resources

Yosys `synth_xilinx` estimate for Artix-7 (`make synth`); Vivado numbers
and timing will be added after implementation.

| Block | LUTs | FFs | LUTRAM | BRAM36 | DSP48E1 |
|---|---|---|---|---|---|
| RV32I 5-stage pipeline | 1428 | 333 | 12 | 0 | 0 |
| K-means accelerator (core, streamer, memories) | 971 | 940 | 6 | 12 | 8 |
| UART with FIFOs | 199 | 120 | 4 | 0 | 0 |
| System control | 51 | 176 | 0 | 0 | 0 |
| **Whole board design** | **2599** | **1575** | 22 | 16 | 8 |

About 4% of the xc7a100t's LUTs.

## Running it on the Nexys A7

```
make firmware                                   # build/firmware.hex
vivado -mode batch -source fpga/build.tcl       # bitstream + utilization/timing reports
vivado -mode batch -source fpga/program.tcl     # load it over USB
pip install pyserial matplotlib
python3 host/kmeans_host.py --port /dev/ttyUSB1 --points 4000 --plot result.png
```

On Windows the port is `COM<n>` (Device Manager, "USB Serial Port"); on
Linux it is usually the second of the board's two ttyUSB devices. The host
program generates clustered data (or reads `--csv`), chooses k-means++
starting centroids, runs both modes, checks them against the reference model
and prints the cycle counts. `--fake` runs it against a software stand-in
for the board.

LEDs: LED15 toggles on every request, LED8 shows the mode of the last run
and LED7-0 its iteration count; the red LED16 means the CPU has stopped.

## Running the tests

```
make lint        # Verilator -Wall
make sim         # system test (about 3 minutes)
make host-test   # host program tests
make formal      # SymbiYosys proofs
make synth       # Yosys resource estimate
```

Requires Icarus Verilog 12, Verilator 5, Yosys, SymbiYosys with Yices,
`riscv64-unknown-elf-gcc`, and Python 3 with `cocotb >= 2.0` and pytest.
CI runs all of it on every push.

## Repository layout

```
ip/riscv/          RV32I pipeline (vendored, see ip/README.md)
ip/kmeans/         K-means core (vendored)
rtl/               soc.sv (bus), kmeans_accel.sv, soc_uart.sv, uart_tx.sv, uart_rx.sv,
                   soc_sysctl.sv, soc_ram.sv, sync_fifo.sv
fpga/              Nexys A7 top level, constraints, Vivado build and program scripts
sw/                firmware: main.c (protocol), kmeans.c, crt0.S, link.ld
host/              kmeans_host.py, protocol.py, kmeans_ref.py (reference), fake_board.py, tests
tb/                cocotb system test and UART model
formal/            FIFO and accelerator proofs
synth/             Yosys resource estimate
tools/             hex conversion, CRC table and XDC generators
docs/DESIGN.md     design rationale
```
