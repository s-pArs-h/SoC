"""System tests: the firmware runs on the RTL, and the test talks to it
over the UART pins exactly as the host PC does on the board, using the same
protocol code (host/protocol.py). Results are checked against the
bit-exact reference model (host/kmeans_ref.py)."""
from __future__ import annotations

import os
import random
import sys

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "host"))
import kmeans_ref as ref                                            # noqa: E402
import protocol as p                                                # noqa: E402
from uart_model import Uart                                         # noqa: E402

CLK_NS = 10
CLK_HZ = int(os.environ.get("SIM_CLK_HZ", "1000000"))
BAUD = int(os.environ.get("SIM_BAUD", "125000"))
PTS_AW = int(os.environ.get("PTS_AW", "8"))
SEED = int(os.environ.get("SEED", "1"))
DIV = CLK_HZ // BAUD
MAX_POINTS = 1 << PTS_AW
K = 4

COVERAGE = {name: 0 for name in [
    "info", "load", "run_hw", "run_sw", "labels", "converged", "not_converged",
    "empty_cluster", "dist_ge_2_32", "st_crc", "st_len", "st_cmd", "st_param",
    "st_timeout", "garbage_before_sync", "load_max_points", "load_zero_points",
    "load_crc_invalidates",
]}


def hit(name):
    COVERAGE[name] += 1


class Host:
    """The PC side, in simulation."""

    def __init__(self, dut):
        self.dut = dut
        self.uart = Uart(dut.uart_rxd, dut.uart_txd, bit_ns=DIV * CLK_NS)

    async def transact(self, frame: bytes, timeout_ms: int = 200) -> p.Response:
        await self.uart.send(frame)
        hdr = await self.uart.recv(p.RESP_HEADER_LEN, timeout_ms * 1_000_000)
        n = p.parse_header(hdr)
        rest = await self.uart.recv(n + 2, timeout_ms * 1_000_000)
        return p.parse_response(hdr + rest)

    async def cmd(self, cmd, payload=b"", **kw) -> p.Response:
        return await self.transact(p.request(cmd, payload), **kw)

    async def info(self) -> p.Info:
        return p.parse_info((await self.cmd(p.CMD_INFO)).check(p.CMD_INFO))

    async def load(self, pts):
        r = (await self.cmd(p.CMD_LOAD, p.load_payload(pts))).check(p.CMD_LOAD)
        assert int.from_bytes(r, "little") == len(pts)
        hit("load")

    async def run(self, mode, max_iter, init) -> p.RunResult:
        r = await self.cmd(p.CMD_RUN, p.run_payload(mode, max_iter, init), timeout_ms=2000)
        return p.parse_run(r.check(p.CMD_RUN), K)

    async def labels(self, n, chunk=200):
        out = []
        for off in range(0, n, chunk):
            c = min(chunk, n - off)
            r = await self.cmd(p.CMD_LABELS, p.labels_payload(off, c))
            out += list(r.check(p.CMD_LABELS))
        hit("labels")
        return out


async def boot(dut) -> Host:
    cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())
    dut.sw.value = 0
    dut.rst_n.value = 0
    host = Host(dut)
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    # Handshake, as the host program does: the firmware only listens once
    # its start-up code has run, so a session begins with INFO (short
    # enough to wait in the receive FIFO) and data follows the answer.
    await host.info()
    return host


async def check_run(host, pts, init, max_iter, mode, label_check=True):
    """Runs K-means on the SoC and compares everything with the model."""
    exp = ref.lloyd(pts, init, max_iter)
    got = await host.run(mode, max_iter, init)
    tag = "hw" if mode == p.MODE_HW else "sw"
    assert got.iterations == exp["iterations"], (tag, got.iterations, exp["iterations"])
    assert got.converged == exp["converged"], tag
    assert got.centroids == exp["centroids"], (tag, got.centroids, exp["centroids"])
    assert got.counts == exp["counts"], (tag, got.counts, exp["counts"])
    assert got.sse == exp["sse"], (tag, got.sse, exp["sse"])
    if label_check and pts:
        assert await host.labels(len(pts)) == exp["labels"], tag
    hit("run_" + tag)
    hit("converged" if got.converged else "not_converged")
    if 0 in got.counts:
        hit("empty_cluster")
    if mode == p.MODE_HW:
        # the accelerator streams one point per clock
        assert got.accel_cycles <= got.iterations * (len(pts) + 16), got.accel_cycles
    return got


def forgy(pts, rng):
    return rng.sample(pts, K)


# ----------------------------------------------------------------------
@cocotb.test()
async def test_info(dut):
    host = await boot(dut)
    info = await host.info()
    assert (info.version, info.k, info.data_w) == (1, K, 16), info
    assert info.max_points == MAX_POINTS and info.ram_kib == 16 and info.clk_hz == CLK_HZ, info
    hit("info")


@cocotb.test()
async def test_kmeans_hw(dut):
    """Several data sets on the accelerator, every output compared."""
    host = await boot(dut)
    rng = random.Random(SEED)
    for _ in range(3):
        n = rng.choice([MAX_POINTS, rng.randint(K, MAX_POINTS)])
        pts = ref.blobs(n, K, spread=rng.choice([300, 3000]), rng=rng)
        await host.load(pts)
        if n == MAX_POINTS:
            hit("load_max_points")
        await check_run(host, pts, forgy(pts, rng), max_iter=50, mode=p.MODE_HW)


@cocotb.test()
async def test_hw_matches_sw(dut):
    """The same problem in both modes: identical results, and the speedup."""
    host = await boot(dut)
    rng = random.Random(SEED + 1)
    pts = ref.blobs(64, K, spread=2000, rng=rng)
    init = forgy(pts, rng)
    await host.load(pts)
    hw = await check_run(host, pts, init, 50, p.MODE_HW)
    sw = await check_run(host, pts, init, 50, p.MODE_SW)
    dut._log.info("n=%d, %d iterations: assignment %d vs %d cycles (%.1fx), whole run %d vs %d "
                  "cycles (%.1fx)", len(pts), hw.iterations, sw.assign_cycles, hw.assign_cycles,
                  sw.assign_cycles / hw.assign_cycles, sw.total_cycles, hw.total_cycles,
                  sw.total_cycles / hw.total_cycles)
    with open(os.environ.get("SPEEDUP_REPORT", "speedup.txt"), "w") as f:
        f.write(f"points {len(pts)} iterations {hw.iterations}\n"
                f"assign_cycles hw {hw.assign_cycles} sw {sw.assign_cycles}\n"
                f"total_cycles hw {hw.total_cycles} sw {sw.total_cycles}\n"
                f"accel_cycles {hw.accel_cycles}\n")
    # the iteration cap
    await check_run(host, pts, init, 1, p.MODE_HW)
    await check_run(host, pts, init, 1, p.MODE_SW)


@cocotb.test()
async def test_extremes(dut):
    """Corner points and far centroids: squared distances above 2^32."""
    host = await boot(dut)
    lo, hi = -32768, 32767
    pts = [(lo, lo), (hi, hi), (lo, hi), (hi, lo), (0, 0), (hi, 0), (lo, 0), (0, lo)] * 2
    for init in ([(hi, hi), (hi, hi - 1), (hi - 1, hi), (hi - 1, hi - 1)],     # ties, far points
                 [(lo, lo), (hi, hi), (lo, hi), (hi, lo)],
                 [(0, 0), (0, 0), (0, 0), (0, 0)]):                          # all tied
        await host.load(pts)
        for mode in (p.MODE_HW, p.MODE_SW):
            await check_run(host, pts, init, 20, mode)
        nearest = [min((x - cx) ** 2 + (y - cy) ** 2 for cx, cy in init) for x, y in pts]
        if max(nearest) >= 2 ** 32:
            hit("dist_ge_2_32")


@cocotb.test()
async def test_protocol_errors(dut):
    """Every error status, and the SoC keeps working afterwards."""
    host = await boot(dut)
    rng = random.Random(SEED + 2)

    # garbage before a frame is skipped
    await host.uart.send(bytes([0x00, 0xFF, 0x13, 0x5A]))
    assert (await host.info()).k == K
    hit("garbage_before_sync")

    # corrupted payload byte
    frame = bytearray(p.request(p.CMD_INFO, b"\x01\x02"))
    frame[4] ^= 0x40
    r = await host.transact(bytes(frame))
    assert r.status == p.ST_CRC, r
    hit("st_crc")

    r = await host.cmd(0x7E)
    assert (r.cmd, r.status) == (0x7E, p.ST_CMD), r
    hit("st_cmd")

    # lengths and parameters
    assert (await host.cmd(p.CMD_LOAD, bytes(6))).status == p.ST_LEN
    assert (await host.cmd(p.CMD_RUN, bytes(5))).status == p.ST_LEN
    assert (await host.cmd(p.CMD_LABELS, bytes(3))).status == p.ST_LEN
    hit("st_len")

    pts = ref.blobs(40, K, 500, rng)
    await host.load(pts)
    init = forgy(pts, rng)
    assert (await host.cmd(p.CMD_LABELS, p.labels_payload(0, 1))).status == p.ST_PARAM  # no run yet
    assert (await host.cmd(p.CMD_RUN, p.run_payload(2, 5, init))).status == p.ST_PARAM  # mode
    assert (await host.cmd(p.CMD_RUN, p.run_payload(0, 0, init))).status == p.ST_PARAM  # max_iter
    too_many = [(0, 0)] * (MAX_POINTS + 1)
    assert (await host.cmd(p.CMD_LOAD, p.load_payload(too_many))).status == p.ST_PARAM
    # a refused load leaves the previous data in place
    await check_run(host, pts, init, 30, p.MODE_HW)
    assert (await host.cmd(p.CMD_LABELS, p.labels_payload(30, 11))).status == p.ST_PARAM
    hit("st_param")

    # a frame that stops part way: the firmware gives up after 50 ms
    await host.uart.send(p.request(p.CMD_INFO, bytes(8))[:6])
    hdr = await host.uart.recv(p.RESP_HEADER_LEN, timeout_ns=CLK_NS * CLK_HZ)
    rest = await host.uart.recv(p.parse_header(hdr) + 2, timeout_ns=1_000_000)
    assert p.parse_response(hdr + rest).status == p.ST_TIMEOUT
    hit("st_timeout")

    # a load whose crc fails leaves no points
    frame = bytearray(p.request(p.CMD_LOAD, p.load_payload(pts)))
    frame[-1] ^= 1
    assert (await host.transact(bytes(frame))).status == p.ST_CRC
    got = await host.run(p.MODE_HW, 5, init)
    assert got.counts == [0] * K and got.iterations == 1 and got.converged
    hit("load_crc_invalidates")

    # zero points is a valid (empty) data set
    await host.load([])
    got = await host.run(p.MODE_SW, 5, init)
    assert got.counts == [0] * K and got.centroids == [tuple(c) for c in init]
    hit("load_zero_points")

    # and everything still works
    await host.load(pts)
    await check_run(host, pts, init, 30, p.MODE_SW)
    assert host.uart.frame_errors == 0


@cocotb.test()
async def test_coverage_closure(dut):
    """Fails unless every coverage bin was hit by the tests above."""
    await Timer(1, "ns")
    lines = [f"{name:24s} {count}" for name, count in COVERAGE.items()]
    with open(os.environ.get("COVERAGE_REPORT", "coverage.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    missing = [n for n, c in COVERAGE.items() if c == 0]
    assert not missing, f"coverage holes: {missing}"
