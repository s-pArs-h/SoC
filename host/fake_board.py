"""A stand-in for the board: answers the UART protocol in Python, using the
reference model for the computation. It lets kmeans_host.py be tried and
tested without hardware. It reports zero cycle counts (there is no clock to
count), and it is not a model of the RTL: the RTL is tested in tb/.
"""
from __future__ import annotations

import struct

import kmeans_ref as ref
import protocol as p


class FakeBoard:
    K, MAX_POINTS, RAM_KIB, CLK_HZ = 4, 4096, 16, 50_000_000

    def __init__(self):
        self.rx = bytearray()       # bytes from the host, not yet handled
        self.tx = bytearray()       # bytes for the host
        self.points = []
        self.labels = None

    # link interface used by kmeans_host.Board
    def write(self, data: bytes):
        self.rx += data
        self._process()

    def read(self, n: int, timeout_s: float = 0.0) -> bytes:
        out, self.tx = bytes(self.tx[:n]), self.tx[n:]
        return out

    def flush_input(self):
        self.tx.clear()

    # ------------------------------------------------------------------
    def _respond(self, cmd, status, payload=b""):
        body = struct.pack("<BBH", cmd, status, len(payload)) + payload
        self.tx += bytes([p.SYNC_RESP]) + body + struct.pack("<H", p.crc16(body))

    def _process(self):
        while True:
            while self.rx and self.rx[0] != p.SYNC_REQ:
                del self.rx[0]
            if len(self.rx) < 4:
                return
            cmd, n = struct.unpack_from("<BH", self.rx, 1)
            if len(self.rx) < 4 + n + 2:
                return
            body = bytes(self.rx[1:4 + n])
            (crc,) = struct.unpack_from("<H", self.rx, 4 + n)
            del self.rx[:4 + n + 2]
            if crc != p.crc16(body):
                self._respond(cmd, p.ST_CRC)
                continue
            self._handle(cmd, body[3:])

    def _handle(self, cmd, pl):
        if cmd == p.CMD_INFO:
            self._respond(cmd, p.ST_OK, struct.pack("<BBBBHHI", p.PROTO_VERSION, self.K, 16, 0,
                                                    self.MAX_POINTS, self.RAM_KIB, self.CLK_HZ))
        elif cmd == p.CMD_LOAD:
            if len(pl) % 4:
                self._respond(cmd, p.ST_LEN)
            elif len(pl) // 4 > self.MAX_POINTS:
                self._respond(cmd, p.ST_PARAM)
            else:
                self.points = [struct.unpack_from("<hh", pl, 4 * i) for i in range(len(pl) // 4)]
                self.labels = None
                self._respond(cmd, p.ST_OK, struct.pack("<H", len(self.points)))
        elif cmd == p.CMD_RUN:
            if len(pl) != 2 + 4 * self.K:
                self._respond(cmd, p.ST_LEN)
                return
            mode, max_iter = pl[0], pl[1]
            if mode > p.MODE_SW or max_iter == 0:
                self._respond(cmd, p.ST_PARAM)
                return
            init = [struct.unpack_from("<hh", pl, 2 + 4 * k) for k in range(self.K)]
            r = ref.lloyd(self.points, init, max_iter)
            self.labels = r["labels"]
            out = struct.pack("<BBBB", r["iterations"], int(r["converged"]), mode, self.K)
            out += b"".join(struct.pack("<hh", *c) for c in r["centroids"])
            out += struct.pack(f"<{self.K}I", *r["counts"])
            out += struct.pack("<4Q", r["sse"], 0, 0, 0)
            self._respond(cmd, p.ST_OK, out)
        elif cmd == p.CMD_LABELS:
            if len(pl) != 4:
                self._respond(cmd, p.ST_LEN)
                return
            off, cnt = struct.unpack("<HH", pl)
            if self.labels is None or off + cnt > len(self.points):
                self._respond(cmd, p.ST_PARAM)
            else:
                self._respond(cmd, p.ST_OK, bytes(self.labels[off:off + cnt]))
        else:
            self._respond(cmd, p.ST_CMD)
