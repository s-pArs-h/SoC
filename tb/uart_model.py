"""Bit-level UART (8N1) for cocotb: drives the SoC's receive pin and
decodes its transmit pin. Timing uses Timer rather than counting clock
edges, which keeps the simulation fast."""
from __future__ import annotations

import cocotb
from cocotb.queue import Queue
from cocotb.triggers import FallingEdge, Timer, with_timeout


class UartFrameError(Exception):
    pass


class Uart:
    def __init__(self, rxd, txd, bit_ns: int):
        assert bit_ns % 2 == 0
        self.rxd, self.txd, self.bit_ns = rxd, txd, bit_ns
        self.received: Queue[int] = Queue()
        self.frame_errors = 0
        self.rxd.value = 1
        cocotb.start_soon(self._monitor())

    async def send(self, data: bytes, stop_bits: int = 1):
        for b in data:
            self.rxd.value = 0
            await Timer(self.bit_ns, "ns")
            for i in range(8):
                self.rxd.value = (b >> i) & 1
                await Timer(self.bit_ns, "ns")
            self.rxd.value = 1
            await Timer(self.bit_ns * stop_bits, "ns")

    async def recv(self, n: int, timeout_ns: int) -> bytes:
        out = bytearray()
        for _ in range(n):
            out.append(await with_timeout(self.received.get(), timeout_ns, "ns"))
        return bytes(out)

    async def _monitor(self):
        while True:
            await FallingEdge(self.txd)
            await Timer(self.bit_ns // 2, "ns")            # middle of the start bit
            if int(self.txd.value) != 0:
                continue                                    # glitch
            b = 0
            for i in range(8):
                await Timer(self.bit_ns, "ns")
                b |= int(self.txd.value) << i
            await Timer(self.bit_ns, "ns")
            if int(self.txd.value) != 1:
                self.frame_errors += 1
                raise UartFrameError("stop bit low on the SoC's transmit line")
            self.received.put_nowait(b)
