#!/usr/bin/env python3
"""Runs K-means on the SoC from a PC over the board's USB-UART.

    python3 host/kmeans_host.py --port /dev/ttyUSB1              # Linux
    python3 host/kmeans_host.py --port COM5 --points 4000         # Windows
    python3 host/kmeans_host.py --port /dev/ttyUSB1 --csv data.csv --plot out.png
    python3 host/kmeans_host.py --fake                             # no board needed

The program generates (or reads) a 2-D data set, loads it into the
accelerator's point memory, runs K-means with the assignment step on the
accelerator and then entirely in software on the same RISC-V core, checks
both against a bit-exact Python model, and reports cycle counts.

Requires pyserial for a real board (pip install pyserial); matplotlib for --plot.
"""
from __future__ import annotations

import argparse
import csv
import random
import sys
import time

import kmeans_ref as ref
import protocol as p


# ----------------------------------------------------------------------
# Transport
# ----------------------------------------------------------------------
class SerialLink:
    def __init__(self, port: str, baud: int):
        try:
            import serial
        except ImportError:
            sys.exit("pyserial is needed for a real board: pip install pyserial")
        self.s = serial.Serial(port, baud, timeout=0.05)

    def write(self, data: bytes):
        self.s.write(data)

    def read(self, n: int, timeout_s: float) -> bytes:
        out = bytearray()
        deadline = time.monotonic() + timeout_s
        while len(out) < n and time.monotonic() < deadline:
            out += self.s.read(n - len(out))
        return bytes(out)

    def flush_input(self):
        self.s.reset_input_buffer()


class Board:
    """Request/response over a link (SerialLink or fake_board.FakeBoard)."""

    def __init__(self, link):
        self.link = link
        self.info: p.Info | None = None

    def transact(self, cmd: int, payload: bytes = b"", timeout_s: float = 2.0) -> p.Response:
        self.link.write(p.request(cmd, payload))
        deadline = time.monotonic() + timeout_s
        while True:                                   # skip anything before a frame
            b = self.link.read(1, max(0.0, deadline - time.monotonic()))
            if not b:
                raise p.ProtocolError(f"no response to command {cmd:#x}")
            if b[0] == p.SYNC_RESP:
                break
        hdr = b + self.link.read(p.RESP_HEADER_LEN - 1, timeout_s)
        n = p.parse_header(hdr)
        rest = self.link.read(n + 2, timeout_s + n / 1000)
        return p.parse_response(hdr + rest)

    def connect(self, attempts: int = 5) -> p.Info:
        """INFO handshake: the firmware listens once its start-up code has run."""
        for i in range(attempts):
            try:
                self.info = p.parse_info(self.transact(p.CMD_INFO, timeout_s=0.5).check(p.CMD_INFO))
                return self.info
            except p.ProtocolError:
                if i == attempts - 1:
                    raise
                if hasattr(self.link, "flush_input"):
                    self.link.flush_input()
        raise AssertionError("unreachable")

    def load(self, pts):
        timeout = 2.0 + len(pts) * 4 * 10 / 9600            # generous for any baud >= 9600
        r = self.transact(p.CMD_LOAD, p.load_payload(pts), timeout_s=timeout).check(p.CMD_LOAD)
        if int.from_bytes(r, "little") != len(pts):
            raise p.ProtocolError("board stored a different number of points")

    def run(self, mode: int, max_iter: int, init) -> p.RunResult:
        r = self.transact(p.CMD_RUN, p.run_payload(mode, max_iter, init), timeout_s=120.0)
        return p.parse_run(r.check(p.CMD_RUN), self.info.k)

    def labels(self, n: int, chunk: int = 1024):
        out = []
        for off in range(0, n, chunk):
            c = min(chunk, n - off)
            out += list(self.transact(p.CMD_LABELS, p.labels_payload(off, c)).check(p.CMD_LABELS))
        return out


# ----------------------------------------------------------------------
# Data
# ----------------------------------------------------------------------
def read_csv(path: str):
    """First two numeric columns. Scaled to the int16 range if they don't fit
    as integers (aspect ratio kept)."""
    rows = []
    with open(path, newline="") as f:
        for row in csv.reader(f):
            try:
                rows.append((float(row[0]), float(row[1])))
            except (ValueError, IndexError):
                continue                               # header or blank line
    if not rows:
        sys.exit(f"{path}: no numeric rows")
    if all(x.is_integer() and y.is_integer() and -32768 <= x <= 32767 and -32768 <= y <= 32767
           for x, y in rows):
        return [(int(x), int(y)) for x, y in rows], None
    cx = (min(r[0] for r in rows) + max(r[0] for r in rows)) / 2
    cy = (min(r[1] for r in rows) + max(r[1] for r in rows)) / 2
    span = max(max(abs(x - cx), abs(y - cy)) for x, y in rows) or 1.0
    scale = 30000 / span
    return [(round((x - cx) * scale), round((y - cy) * scale)) for x, y in rows], (cx, cy, scale)


def plot(path, pts, labels, centroids, title):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed: skipping the plot")
        return
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.scatter([x for x, _ in pts], [y for _, y in pts], c=labels, s=6, cmap="tab10")
    ax.scatter([x for x, _ in centroids], [y for _, y in centroids], c="black", marker="x", s=80)
    ax.set_title(title)
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print(f"plot written to {path}")


# ----------------------------------------------------------------------
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port of the board, e.g. /dev/ttyUSB1 or COM5")
    src.add_argument("--fake", action="store_true", help="simulated board (no hardware)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--csv", help="data set: first two columns of a CSV file")
    ap.add_argument("--points", type=int, default=2000, help="generated data set size")
    ap.add_argument("--spread", type=int, default=2500, help="generated cluster spread")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--max-iter", type=int, default=50)
    ap.add_argument("--init", choices=["kmeans++", "random"], default="kmeans++",
                    help="starting centroids (computed here, sent with the run)")
    ap.add_argument("--mode", choices=["hw", "sw", "both"], default="both")
    ap.add_argument("--plot", help="write a scatter plot of the result (PNG)")
    ap.add_argument("--no-verify", action="store_true", help="skip the Python reference check")
    args = ap.parse_args(argv)

    if args.fake:
        from fake_board import FakeBoard
        link = FakeBoard()
    else:
        link = SerialLink(args.port, args.baud)
    board = Board(link)
    info = board.connect()
    print(f"board: K = {info.k}, up to {info.max_points} points, {info.clk_hz / 1e6:g} MHz, "
          f"{info.ram_kib} KiB RAM{' (fake)' if args.fake else ''}")

    rng = random.Random(args.seed)
    if args.csv:
        pts, scaled = read_csv(args.csv)
        if scaled:
            print(f"{args.csv}: values scaled by {scaled[2]:.4g} to fit 16-bit integers")
    else:
        pts = ref.blobs(args.points, info.k, args.spread, rng)
    if len(pts) > info.max_points:
        sys.exit(f"{len(pts)} points, the board holds {info.max_points}")
    if len(pts) < info.k:
        sys.exit(f"need at least {info.k} points")
    init = (ref.kmeans_pp(pts, info.k, rng) if args.init == "kmeans++"
            else rng.sample(pts, info.k))

    t = time.monotonic()
    board.load(pts)
    print(f"loaded {len(pts)} points in {time.monotonic() - t:.2f} s")

    expected = None if args.no_verify else ref.lloyd(pts, init, args.max_iter)
    modes = {"hw": [p.MODE_HW], "sw": [p.MODE_SW], "both": [p.MODE_HW, p.MODE_SW]}[args.mode]
    results, ok = {}, True
    for mode in modes:
        name = "accelerator" if mode == p.MODE_HW else "software"
        r = board.run(mode, args.max_iter, init)
        labels = board.labels(len(pts))
        results[mode] = (r, labels)
        if expected is not None:
            match = (r.iterations == expected["iterations"] and r.centroids == expected["centroids"]
                     and r.counts == expected["counts"] and r.sse == expected["sse"]
                     and labels == expected["labels"])
            ok &= match
            verdict = "matches the reference" if match else "DOES NOT MATCH the reference"
        else:
            verdict = "not checked"
        print(f"\n{name}: {r.iterations} iterations, "
              f"{'converged' if r.converged else 'stopped at --max-iter'}, SSE {r.sse}, {verdict}")
        for k, (c, n) in enumerate(zip(r.centroids, r.counts)):
            print(f"  cluster {k}: centroid ({c[0]:6d}, {c[1]:6d}), {n} points")
        if r.total_cycles:
            ms = 1e3 / info.clk_hz
            print(f"  cycles: assignment {r.assign_cycles:,} ({r.assign_cycles * ms:.2f} ms), "
                  f"whole run {r.total_cycles:,} ({r.total_cycles * ms:.2f} ms)"
                  + (f", accelerator busy {r.accel_cycles:,}" if mode == p.MODE_HW else ""))

    if len(results) == 2:
        hw, sw = results[p.MODE_HW][0], results[p.MODE_SW][0]
        if hw.assign_cycles and hw.total_cycles:
            print(f"\nspeed-up with the accelerator: assignment step "
                  f"{sw.assign_cycles / hw.assign_cycles:.0f}x, whole run "
                  f"{sw.total_cycles / hw.total_cycles:.1f}x")

    if args.plot:
        r, labels = results[modes[0]]
        plot(args.plot, pts, labels, r.centroids,
             f"K-means on the SoC: {len(pts)} points, {r.iterations} iterations")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
