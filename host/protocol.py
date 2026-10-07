"""Host side of the SoC's UART protocol (the firmware side is sw/protocol.h).

Frames, all fields little-endian:
    request   0xA5  cmd  len[2]  payload[len]  crc[2]
    response  0x5A  cmd  status  len[2]  payload[len]  crc[2]
crc: CRC-16/CCITT-FALSE over everything between the sync byte and the crc.

This module only builds and parses bytes, so the same code drives the real
board (kmeans_host.py, over pyserial) and the simulation (tb/test_soc.py).
"""
from __future__ import annotations

import binascii
import struct
from dataclasses import dataclass

SYNC_REQ, SYNC_RESP = 0xA5, 0x5A
PROTO_VERSION = 1

CMD_INFO, CMD_LOAD, CMD_RUN, CMD_LABELS = 0x01, 0x02, 0x03, 0x04
MODE_HW, MODE_SW = 0, 1

ST_OK, ST_CRC, ST_LEN, ST_CMD, ST_PARAM, ST_TIMEOUT = range(6)
STATUS_NAMES = {ST_OK: "ok", ST_CRC: "crc error", ST_LEN: "bad length",
                ST_CMD: "unknown command", ST_PARAM: "bad parameter",
                ST_TIMEOUT: "timeout"}

RESP_HEADER_LEN = 5          # sync, cmd, status, len[2]


class ProtocolError(Exception):
    pass


def crc16(data: bytes) -> int:
    return binascii.crc_hqx(data, 0xFFFF)


def request(cmd: int, payload: bytes = b"") -> bytes:
    body = struct.pack("<BH", cmd, len(payload)) + payload
    return bytes([SYNC_REQ]) + body + struct.pack("<H", crc16(body))


# ----------------------------------------------------------------------
# Request payloads
# ----------------------------------------------------------------------
def load_payload(points) -> bytes:
    return b"".join(struct.pack("<hh", x, y) for x, y in points)


def run_payload(mode: int, max_iter: int, centroids) -> bytes:
    return struct.pack("<BB", mode, max_iter) + load_payload(centroids)


def labels_payload(offset: int, count: int) -> bytes:
    return struct.pack("<HH", offset, count)


# ----------------------------------------------------------------------
# Responses
# ----------------------------------------------------------------------
@dataclass
class Response:
    cmd: int
    status: int
    payload: bytes

    def check(self, cmd: int) -> bytes:
        if self.cmd != cmd:
            raise ProtocolError(f"response to command {self.cmd:#x}, expected {cmd:#x}")
        if self.status != ST_OK:
            raise ProtocolError(f"command {cmd:#x} failed: "
                                f"{STATUS_NAMES.get(self.status, self.status)}")
        return self.payload


def parse_header(hdr: bytes) -> int:
    """Checks a 5-byte response header; returns the payload length."""
    if len(hdr) != RESP_HEADER_LEN or hdr[0] != SYNC_RESP:
        raise ProtocolError(f"bad response header {hdr.hex()}")
    return struct.unpack_from("<H", hdr, 3)[0]


def parse_response(frame: bytes) -> Response:
    n = parse_header(frame[:RESP_HEADER_LEN])
    if len(frame) != RESP_HEADER_LEN + n + 2:
        raise ProtocolError("truncated response")
    body = frame[1:RESP_HEADER_LEN + n]
    (crc,) = struct.unpack_from("<H", frame, RESP_HEADER_LEN + n)
    if crc != crc16(body):
        raise ProtocolError("response crc mismatch")
    return Response(cmd=frame[1], status=frame[2],
                    payload=frame[RESP_HEADER_LEN:RESP_HEADER_LEN + n])


@dataclass
class Info:
    version: int
    k: int
    data_w: int
    max_points: int
    ram_kib: int
    clk_hz: int


def parse_info(p: bytes) -> Info:
    version, k, data_w, _, max_points, ram_kib, clk_hz = struct.unpack("<BBBBHHI", p)
    return Info(version, k, data_w, max_points, ram_kib, clk_hz)


@dataclass
class RunResult:
    iterations: int
    converged: bool
    mode: int
    centroids: list
    counts: list
    sse: int
    total_cycles: int
    assign_cycles: int
    accel_cycles: int


def parse_run(p: bytes, k: int) -> RunResult:
    it, conv, mode, kk = struct.unpack_from("<BBBB", p, 0)
    if kk != k or len(p) != 4 + 8 * k + 32:
        raise ProtocolError("unexpected run result size")
    cents = [tuple(struct.unpack_from("<hh", p, 4 + 4 * i)) for i in range(k)]
    counts = list(struct.unpack_from(f"<{k}I", p, 4 + 4 * k))
    sse, total, assign, accel = struct.unpack_from("<4Q", p, 4 + 8 * k)
    return RunResult(it, bool(conv), mode, cents, counts, sse, total, assign, accel)
