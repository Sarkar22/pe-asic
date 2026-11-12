# SPDX-License-Identifier: Apache-2.0
"""Tests at the pins, from reset, over the real SPI host link."""

import os
import random
import sys
import zlib

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, Timer
from cocotb.utils import get_sim_time

CS, SCK, MOSI = 0, 1, 2     # bits of ui_in
MISO = 3                    # bit of uo_out
HALF = 3                    # cycles per half SCK period: the fastest the link allows, clk/6

ISA_VERSION = 0x01
THREADS = 4
DEPTH = 512
LOG2_DEPTH = 9
ROUNDS = 3                  # resets in test_netlist_matches_rtl
MIN_TOGGLES = 2000
SIM_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'sim')
LOCK_IMAGES = 3             # random images in test_matches_simulator
LOCK_TRANSACTIONS = 250     # host transactions on each

# The UART transmitter: pin uo0, cycles per bit from mailbox word 7, bytes from mailbox word 0.
#   start: pcfg uo0, PP / put uo0, 1 / now / recv R2, 7
#   byte:  recv R0, 0 / put uo0, 0 / now / ldi R1, 8
#   bit:   shr8 R0, -, uo0 / tick R2 / djnz R1, bit
#          put uo0, 1 / tick R2 / tick R2 / goto byte
UART_TX = [0x90E8, 0x9020, 0x0100, 0xA5E8, 0xA500, 0x9000, 0x0100, 0x3208,
           0xF800, 0xE322, 0x2208, 0x9020, 0xE322, 0xE322, 0x1004]

# ldi R1, 5 / ldi R2, 6 / halt
THREE_STEPS = [0x3205, 0x3406, 0x0080]

# ldi R1, 100 / l: djnz R1, l / halt, at address 20: halts about 410 cycles after it starts
COUNTDOWN = [0x3264, 0x2215, 0x0080]

# ldi R1, 0xEF / lui R1, 0xBE / halt
BEEF = [0x32EF, 0x33BE, 0x0080]

# pcfg io0, PP / put io0, 1 / now / l: goto l, at address 0
IO_HIGH = [0x98E8, 0x9820, 0x0100, 0x1003]

# pcfg uo1, PP / put uo1, 1 / ldi R5, 0x5A / now / l: goto l, at address 60
OWNER = [0x91E8, 0x9120, 0x3A5A, 0x0100, 0x1040]

# put uo1, 1 / now / halt: does nothing unless the thread owns uo1
PUT_UO1 = [0x9120, 0x0100, 0x0080]

# At 80, stepped one word at a time:
#   pcfg uo4, PP / put uo4, 1 / now / tick 1 / put uo4, 0 / (an illegal word) / idle / tick 4000 / idle
STEPS = [0x94E8, 0x9420, 0x0100, 0xC001, 0x9400, 0x0001, 0x0000, 0xCFA0, 0x0000]

# until ui5, 1 / halt, at 100
WAITER = [0xB2C0, 0x0080]


def limit(k):
    """rdtb R1 / wrd R1 / li R2, k / edge -, 0, R2 / halt: the edge, 16 cycles after rdtb, sets a deadline k - 16
    cycles ahead. D moves only at the edge, so it is never half a ring from TB and STALE stays clear."""
    return [0x0690, 0x0610, 0x3400 | k & 255, 0x3500 | k >> 8, 0xE322, 0x0080]


# At 112: li R1, 0x100 / cmpi R1, 0 / bs R2, 0 / ldi R5, 0 / cmp R1, R5 / bs R2, 1 / li R3, 0x101
#         djnz R3, next / ldi R4, 0x55 / next: halt
WIDE = [0x3200, 0x3301, 0x5200, 0x8414, 0x3A00, 0x7364, 0x8434, 0x3601, 0x3701, 0x267B, 0x3855, 0x0080]

# pcfg BYP on uo0, uo4, uo5, uo6, uo7 / now / l: goto l, at 0
BYPASS = [0x90F8, 0x94F8, 0x95F8, 0x96F8, 0x97F8, 0x0100, 0x1006]

# Command codes the specification lists. Any other code is ignored for the rest of the transaction.
LISTED = {0x00, 0x01, 0x02, 0x10, 0x11, 0x12, 0x13, 0x14, 0x3F} | set(range(0x20, 0x30))

# l: goto l, at address 0
SPIN = [0x1000]

# Moves every free pin from a shift register, at address 16.
#   walk:  pcfg PP on uo1, uo2, uo4..uo7, io0..io7 / li R1, 0xACE1 / li R2, 0xB400 / now
#   loop:  crcr R1, R2 / put8 R1, uo / swap R1 / put8 R1, io / oe8 R1 / swap R1 / edge -, 0, 31 / goto loop
WALK = [
    0x91E8, 0x92E8, 0x94E8, 0x95E8, 0x96E8, 0x97E8, 0x98E8, 0x99E8,
    0x9AE8, 0x9BE8, 0x9CE8, 0x9DE8, 0x9EE8, 0x9FE8, 0x32E1, 0x33AC,
    0x3400, 0x35B4, 0x0100, 0x729C, 0x0710, 0x0410, 0x0711, 0x0791,
    0x0410, 0xE31F, 0x1023,
]

# Legal words in no useful order, so a thread goes through most instructions.
SCATTER = [
    0x73A4, 0x9120, 0xA604, 0x7644, 0xFC1E, 0x0200, 0x7210, 0x6979,
    0x3DF5, 0x9D20, 0x8D34, 0x7000, 0xFD7F, 0xAC98, 0x86B4, 0x9560,
    0x0741, 0x7208, 0x91E8, 0x7798, 0x0740, 0x421E, 0xB701, 0x0510,
    0xA6C4, 0x05D0, 0x0100, 0x7D44, 0x9600, 0xAB08, 0x05A0, 0xBE21,
    0x5770, 0xFE82, 0xA4D0, 0xC0DE, 0x9BC0, 0x03A0, 0x0200, 0x03D0,
    0x0600, 0x0420, 0x453D, 0xAAA8, 0x729C, 0x6B8F, 0x88A8, 0x36B3,
    0xA200, 0x0300, 0x0300, 0x7F1C, 0xA7D0, 0xB821, 0xBA81, 0x71E0,
    0xB228, 0x51D9, 0xBDBF, 0x57CC, 0x1784, 0x04E0, 0xFBDA, 0x8700,
    0x7190, 0x7B5C, 0x9CC0, 0x65B9, 0x9080, 0x0200, 0x6C8C, 0x07E1,
    0x7444, 0x06A0, 0xF1C0, 0x9800, 0xFF5F, 0xF04B, 0x0570, 0x15A7,
    0x1215, 0xB1C1, 0x4FBA, 0xBCA0, 0xB074, 0x7A58, 0xAC48, 0xA488,
    0x8C8C, 0xA578, 0xB180, 0x72CC, 0x9EC8, 0x7FDC, 0xB178, 0xA260,
    0x4257, 0x77DC, 0x7E10, 0x99F0, 0x72DC, 0x0100, 0x173F, 0x14B0,
    0x6E80, 0x0280, 0x12E5, 0x9C60, 0x7B9C, 0x8D40, 0x74A8, 0x9D00,
]

# Bypass lanes, open drain and the rows the other programs never run, at address 64.
#   lanes: tid R1 / ldi R3, 3 / li R2, 0x5A3C / li R6, 500 / pcfg io1, IN / now
#   again: pcfg BYP on uo0, uo4..uo7 / pcfg OD on io0, io2..io7 / shl16 R2, ui3, uo4 / xor R2, R1 / put8 R2, io
#          oe8 R2 / put uo6, T / put uo7, ^!T / edge uo5, T, 7 / getn ui4 / aget io1 / bo R2, 5 / clrt
#          ld R4, R6 / stp R2, R6 / tick 9 / pcfg PP on uo0, uo4..uo7, io0, io2..io7 / djnz R3, again / halt
LANES = [
    0x0810, 0x3603, 0x343C, 0x355A, 0x3CF4, 0x3D01, 0x99E0, 0x0100,
    0x90F8, 0x94F8, 0x95F8, 0x96F8, 0x97F8, 0x98F0, 0x9AF0, 0x9BF0,
    0x9CF0, 0x9DF0, 0x9EF0, 0x9FF0, 0xF634, 0x7454, 0x0721, 0x07A1,
    0x9640, 0x97A0, 0xE587, 0xA110, 0xA270, 0x84B0, 0x0180, 0xA930,
    0xAEB0, 0xC009, 0x90E8, 0x94E8, 0x95E8, 0x96E8, 0x97E8, 0x98E8,
    0x9AE8, 0x9BE8, 0x9CE8, 0x9DE8, 0x9EE8, 0x9FE8, 0x2648, 0x0080,
]


def pin(dut, n):
    return (int(dut.uo_out.value) >> n) & 1


class Host:
    """The host at the pads: SPI mode 0, MSB first. Inputs change on the falling clock edge."""

    def __init__(self, dut):
        self.dut = dut
        self.other = 0              # levels of the pins that are not part of the link
        self.link = 1 << CS

    def drive(self, cs, sck, mosi):
        self.link = cs << CS | sck << SCK | mosi << MOSI
        self.dut.ui_in.value = (self.other & 0xF8) | self.link

    async def wait(self, cycles):
        await ClockCycles(self.dut.clk, cycles, rising=False)

    async def shift(self, data, cut=None):
        """Sends data, or only its first `cut` bits. Returns the whole bytes read back."""
        reply = []
        n = 0
        for byte in data:
            got = 0
            for i in range(7, -1, -1):
                if n == cut:
                    return reply
                n += 1
                bit = (byte >> i) & 1
                self.drive(0, 0, bit)
                await self.wait(HALF)
                got = got << 1 | pin(self.dut, MISO)    # sampled as SCK rises
                self.drive(0, 1, bit)
                await self.wait(HALF)
            reply.append(got)
        return reply

    async def xfer(self, data, cut=None):
        await FallingEdge(self.dut.clk)
        self.drive(0, 0, 0)
        await self.wait(HALF)
        reply = await self.shift(data, cut)
        self.drive(0, 0, 0)
        await self.wait(HALF)
        self.drive(1, 0, 0)
        await self.wait(4)
        return reply

    async def caps(self):
        return (await self.xfer([0x00, 0, 0]))[1:]

    async def wrmem(self, addr, words):
        data = [x for w in words for x in (w >> 8, w & 255)]
        await self.xfer([0x01, addr >> 8, addr & 255] + data)

    async def rdmem(self, addr, n):
        r = await self.xfer([0x02, addr >> 8, addr & 255, 0, 0] + [0] * (2 * n))
        return [r[5 + 2 * i] << 8 | r[6 + 2 * i] for i in range(n)]

    async def run(self, stop, start):
        await self.xfer([0x10, stop, start])

    async def setpc(self, thread, pc):
        await self.xfer([0x11, thread, pc >> 8, pc & 255])

    async def status(self):
        return (await self.xfer([0x12, 0, 0, 0, 0]))[1:]

    async def step(self, thread):
        await self.xfer([0x13, thread])

    async def rdreg(self, thread, reg):
        r = await self.xfer([0x14, thread, reg, 0, 0, 0])
        return r[4] << 8 | r[5]

    async def post(self, n, data):
        return (await self.xfer([0x20 + n, data >> 8, data & 255, 0]))[3]

    async def take(self, n):
        return (await self.xfer([0x28 + n, 0, 0, 0]))[1:]

    async def abort(self):
        await self.xfer([0x3F])


async def start(dut):
    """Clock, reset and an idle link."""
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    host = Host(dut)
    dut.ena.value = 1
    dut.uio_in.value = 0
    host.drive(1, 0, 0)
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 8)
    return host


async def load(host, thread, words, addr=0, entry=None):
    await host.wrmem(addr, words)
    await host.setpc(thread, addr if entry is None else entry)


async def receive(dut, bit, limit=4000):
    """One 8N1 frame at uo0, in cycles per bit. Every edge must land on the bit grid. Fails if no start bit
    comes within `limit` cycles, so a stuck transmitter fails the test instead of hanging it."""
    for _ in range(limit):
        if not pin(dut, 0):
            break
        await FallingEdge(dut.clk)
    assert not pin(dut, 0), f"no start bit within {limit} cycles"
    levels = [0]
    for _ in range(10 * bit - 1):
        await FallingEdge(dut.clk)
        levels.append(pin(dut, 0))
    for k in range(1, len(levels)):
        if levels[k] != levels[k - 1]:
            assert k % bit == 0, f"edge {k} cycles into the frame, off the {bit}-cycle grid"
    assert levels[bit // 2] == 0, "start bit did not last"
    assert levels[9 * bit + bit // 2] == 1, "stop bit low"
    return sum(levels[(i + 1) * bit + bit // 2] << i for i in range(8))


@cocotb.test()
async def test_reset_state(dut):
    await start(dut)
    assert int(dut.uo_out.value) == 0, "every uo pin resets low"
    assert int(dut.uio_oe.value) == 0, "every uio pin resets undriven"


@cocotb.test()
async def test_caps(dut):
    host = await start(dut)
    assert await host.caps() == [ISA_VERSION, THREADS << 4 | LOG2_DEPTH]


@cocotb.test()
async def test_miso_moves_three_cycles_after_sck(dut):
    host = await start(dut)
    await FallingEdge(dut.clk)
    host.drive(0, 0, 0)
    await host.wait(HALF)
    await host.shift([0x00])            # CAPS: the first reply byte is 0x01
    changes = []
    for i in range(7, -1, -1):
        host.drive(0, 0, 0)
        await host.wait(HALF)
        before = pin(dut, MISO)
        host.drive(0, 1, 0)             # SCK rises at the pad, seen by the next rising clock edge
        seen = []
        for _ in range(HALF):
            await FallingEdge(dut.clk)
            seen.append(pin(dut, MISO))
        changes.append((before, seen))
    host.drive(0, 0, 0)                 # SCK falls, and CS rises only after it (9.1)
    await host.wait(HALF)
    host.drive(1, 0, 0)
    await host.wait(4)
    assert changes[-1][0] == 1, "bit 0 of the version is 1"
    assert changes[-1][1] == [1, 1, 0], f"MISO moved at the wrong cycle: {changes[-1]}"


@cocotb.test()
async def test_memory(dut):
    host = await start(dut)
    words = [0x1234, 0xBEEF, 0x0001, 0xFFFF, 0x8000]
    await host.wrmem(100, words)
    assert await host.rdmem(100, len(words)) == words, "read back"
    assert await host.rdmem(103, 1) == [0xFFFF], "read from the middle"
    await host.wrmem(0xFFFF, [0xCAFE, 0xF00D])
    assert await host.rdmem(DEPTH - 1, 1) == [0xCAFE], "addresses wrap at DEPTH"
    assert await host.rdmem(0, 1) == [0xF00D], "auto-increment wraps at DEPTH"


@cocotb.test()
async def test_uart_frame_at_the_pad(dut):
    host = await start(dut)
    await load(host, 0, UART_TX)
    await host.run(0, 1)
    await host.wait(60)
    assert pin(dut, 0) == 1, "the line idles high before the bit time arrives"
    bit = 40
    assert await host.post(7, bit) == 0
    for byte in (0x55, 0xA3, 0x00, 0xFF):
        receiver = cocotb.start_soon(receive(dut, bit))
        assert await host.post(0, byte) == 0
        assert await receiver == byte
        await host.wait(bit)
    status = await host.status()
    assert status == [0x10, 0x00, 0x00, 0x00], f"thread 0 running, nothing late: {status}"


@cocotb.test()
async def test_run_stop_step_rdreg(dut):
    host = await start(dut)
    await load(host, 2, THREE_STEPS, 40)
    assert await host.rdreg(2, 10) == 40, "PC set by SETPC"
    await host.step(2)
    await host.wait(16)
    assert (await host.rdreg(2, 1), await host.rdreg(2, 2), await host.rdreg(2, 10)) == (5, 0, 41), "one instruction"
    await host.step(2)
    await host.wait(16)
    assert (await host.rdreg(2, 2), await host.rdreg(2, 10)) == (6, 42), "the next one"
    assert (await host.status())[0] >> 4 == 0, "stopped after each STEP"
    await host.run(0, 1 << 2)
    await host.wait(16)
    assert await host.rdreg(2, 10) == 43, "halt leaves PC on the next word"
    assert (await host.status())[0] >> 4 == 0, "halt stops the thread"
    assert await host.rdreg(2, 32) == 0, "unknown reg8 reads 0"
    assert await host.rdreg(2 | 0xFC, 2) == 6, "only the low 2 bits of thread8 count"

    # A stop lands at the thread's next slot.
    await load(host, 1, SPIN)
    await host.run(0, 1 << 1)
    await host.wait(16)
    assert (await host.status())[0] >> 4 == 1 << 1
    await host.run(1 << 1, 1 << 1)
    await host.wait(16)
    assert (await host.status())[0] >> 4 == 0, "a thread named in both masks is only stopped"


@cocotb.test()
async def test_rdreg_state(dut):
    host = await start(dut)
    # now / tick 20 / clrt / sett / halt: D moves, T set, nothing late.
    words = [0x0100, 0xC014, 0x0180, 0x0200, 0x0080]
    await load(host, 3, words)
    host.other = 0xA0
    await host.run(0, 1 << 3)
    await host.wait(100)
    d = await host.rdreg(3, 8)
    assert (await host.rdreg(3, 11)) & 1 == 1, "T in bit 0 of flags"
    assert (await host.rdreg(3, 9)) & 0xF8 == 0xA0, "S holds the inputs at the last event"
    assert d != 0, "D reads back"


@cocotb.test()
async def test_mailbox(dut):
    host = await start(dut)
    assert await host.take(5) == [0, 0, 0], "an empty word reads as 0"
    assert await host.post(5, 0x1234) == 0, "taken"
    assert await host.post(5, 0x5678) == 1, "refused while full"
    status = await host.status()
    assert status[2] == 1 << 5, f"full flag: {status}"
    assert await host.take(5) == [1, 0x12, 0x34]
    assert await host.take(5) == [0, 0, 0], "emptied by the take"

    # recv R3, 6 / send R3, 2 / halt, with the data through the mailbox both ways
    await load(host, 0, [0xA5CC, 0xA74C, 0x0080], 40)
    await host.run(0, 1)
    assert await host.post(6, 0xABCD) == 0
    await host.wait(40)
    assert await host.take(2) == [1, 0xAB, 0xCD]


@cocotb.test()
async def test_busy(dut):
    host = await start(dut)
    await host.wrmem(0, SPIN)
    await host.wrmem(10, [0x2222])
    await host.setpc(0, 0)
    await host.run(0, 1)
    await host.setpc(0, 7)
    assert (await host.status())[3] == 1, "SETPC of a running thread sets BUSY"
    assert (await host.status())[3] == 0, "STATUS clears it"
    await host.step(0)
    assert (await host.status())[3] == 1, "STEP of a running thread sets BUSY"
    assert await host.rdreg(0, 1) == 0, "RDREG of a running thread reads 0"
    assert (await host.status())[3] == 1, "and sets BUSY"
    await host.setpc(1, 0)
    await host.setpc(2, 0)
    await host.setpc(3, 0)
    await host.run(0, 0xF)
    await host.wait(16)
    assert (await host.status())[0] >> 4 == 0xF
    await host.wrmem(10, [0x1111])
    assert (await host.status())[3] == 1, "no stopped thread: WRMEM sets BUSY"
    assert await host.rdmem(0, 1) == [0], "and RDMEM reads 0"
    assert (await host.status())[3] == 1, "and sets BUSY too"
    await host.run(0xF, 0)
    await host.wait(16)
    assert await host.rdmem(10, 1) == [0x2222], "the refused write did nothing"


@cocotb.test()
async def test_abort(dut):
    host = await start(dut)
    # pcfg io0, PP / put io0, 1 / now / l: goto l
    await load(host, 0, [0x98E8, 0x9820, 0x0100, 0x1003])
    await host.run(0, 1)
    await host.wait(40)
    assert int(dut.uio_oe.value) & 1 == 1 and int(dut.uio_out.value) & 1 == 1, "io0 driven high"
    await host.abort()
    await host.wait(10)
    assert int(dut.uio_oe.value) == 0, "ABORT leaves every uio pin undriven"
    assert (await host.status())[0] >> 4 == 0, "and stops the thread"


@cocotb.test()
async def test_transaction_rules(dut):
    host = await start(dut)
    await host.xfer([0x77, 0x10, 0x00, 0x01])
    assert (await host.status())[0] >> 4 == 0, "an unknown code ignores the rest of the transaction"

    # A rising CS discards a partial byte.
    await FallingEdge(dut.clk)
    host.drive(0, 0, 0)
    await host.wait(HALF)
    for i in range(5):
        host.drive(0, 0, 1)
        await host.wait(HALF)
        host.drive(0, 1, 1)
        await host.wait(HALF)
    host.drive(0, 0, 0)
    await host.wait(HALF)
    host.drive(1, 0, 0)
    await host.wait(4)
    assert await host.caps() == [ISA_VERSION, THREADS << 4 | LOG2_DEPTH], "a clean start after the partial byte"


@cocotb.test()
async def test_cut_commands(dut):
    """9.1: a command cut before its last argument bit does nothing, even after whole argument bytes."""
    host = await start(dut)
    await host.wrmem(0, SPIN)
    await host.wrmem(100, [0x1111, 0x2222, 0x3333])
    await host.setpc(0, 0)
    await host.xfer([0x10, 0x00, 0x01], cut=16)           # RUN with stop8 whole, start8 missing
    await host.xfer([0x10, 0x00, 0x01], cut=23)           # RUN without its last bit
    assert (await host.status())[0] >> 4 == 0, "a cut RUN starts nothing"
    await host.xfer([0x11, 0x00, 0x00, 0x05], cut=24)     # SETPC with pc16 half sent
    await host.xfer([0x11, 0x00, 0x00, 0x05], cut=31)
    assert await host.rdreg(0, 10) == 0, "a cut SETPC leaves PC"
    await host.xfer([0x13, 0x00], cut=15)                 # STEP without its last bit
    assert (await host.rdreg(0, 10), (await host.status())[0]) == (0, 0), "a cut STEP runs nothing"
    await host.xfer([0x25, 0x12, 0x34, 0x00], cut=16)     # POST with one data byte
    await host.xfer([0x25, 0x12, 0x34, 0x00], cut=23)
    assert await host.take(5) == [0, 0, 0], "a cut POST fills nothing"
    await host.xfer([0x01, 0x00, 100, 0xAA, 0xAA, 0xBB, 0xBB], cut=55)     # WRMEM: one whole word, then all but a bit
    await host.xfer([0x01, 0x00, 101, 0xCC, 0xCC], cut=32)                 # one whole data byte
    await host.xfer([0x01, 0x00, 102, 0xDD, 0xDD], cut=39)
    assert await host.rdmem(100, 3) == [0xAAAA, 0x2222, 0x3333], "only whole words are written"

    # 9.2: BUSY clears at the last bit of the fourth dummy byte, so a STATUS cut short leaves it set.
    await host.run(0, 1)
    await host.setpc(0, 7)
    await host.xfer([0x12, 0, 0, 0, 0], cut=32)
    await host.xfer([0x12, 0, 0, 0, 0], cut=39)
    assert (await host.status())[3] == 1, "a cut STATUS leaves BUSY set"
    assert (await host.status())[3] == 0, "a whole one clears it"


@cocotb.test()
async def test_dead_until_cs(dut):
    """9.4: a memory command that finds all four threads running sets BUSY and does nothing more until CS rises,
    even when a thread stops before its next word."""
    host = await start(dut)
    await host.wrmem(0, SPIN)
    await host.wrmem(20, COUNTDOWN)
    old = [0x5000 + i for i in range(8)]
    await host.wrmem(200, old)
    for t, pc in ((0, 0), (1, 0), (2, 0), (3, 20)):
        await host.setpc(t, pc)
    await host.run(0, 0xF)
    # The first data word lands about 255 cycles after RUN, with all four running. Thread 3 halts about 160
    # cycles later, two words in, so the later words find a stopped thread and must still not be written.
    await host.wrmem(200, [0xD000 + i for i in range(8)])
    status = await host.status()
    assert status[0] >> 4 == 0x7 and status[3] == 1, f"thread 3 stopped during the write, BUSY set: {status}"
    await host.run(0xF, 0)
    await host.wait(16)
    assert await host.rdmem(200, 8) == old, "no word of the refused write lands"
    await host.wrmem(200, [0xD000])
    assert await host.rdmem(200, 1) == [0xD000], "the next transaction works"


@cocotb.test()
async def test_unlisted_codes(dut):
    """9.2: every code the table does not list is ignored for the rest of its transaction, whatever follows."""
    host = await start(dut)
    await load(host, 0, IO_HIGH)
    await host.wrmem(0x10F, [0x1234])
    await host.run(0, 1)
    await host.wait(40)
    tail = [0x0F, 0x0F, 0x0F, 0x0F]                    # as RUN it would stop all, as WRMEM write 0x10F, as POST fill a word
    for n, code in enumerate(c for c in range(256) if c not in LISTED):
        reply = await host.xfer([code] + tail)
        assert reply == [0] * 5, f"code {code:#04x} replied {reply}"
        assert int(dut.uio_oe.value) & 1 and int(dut.uio_out.value) & 1, f"code {code:#04x} released io0"
        if n % 33 == 32:
            status = await host.status()          # STALE may set as TB passes half the ring from D: not checked
            assert (status[0], status[1] >> 4, status[2], status[3]) == (0x10, 0, 0, 0), f"after code {code:#04x}: {status}"
    status = await host.status()
    assert (status[0], status[1] >> 4, status[2], status[3]) == (0x10, 0, 0, 0), f"at the end: {status}"
    assert await host.rdmem(0x10F, 1) == [0x1234], "no unlisted code wrote memory"
    assert await host.rdreg(3, 10) == 0, "no unlisted code moved a stopped thread"


@cocotb.test()
async def test_bytes_after_reply(dut):
    """9.1: MISO reads 0 except during the reply bytes listed, however long CS stays low, and bytes after a command
    do nothing."""
    host = await start(dut)
    await load(host, 3, BEEF, 48)
    await host.step(3)
    await host.wait(16)
    await host.step(3)
    await host.wait(16)
    r = await host.xfer([0x14, 3, 1, 0, 0, 0] + [0] * 12)
    assert r == [0, 0, 0, 0, 0xBE, 0xEF] + [0] * 12, f"RDREG then 12 more bytes: {r}"
    r = await host.xfer([0x00, 0, 0] + [0x10, 0x00, 0x08] * 3)
    assert r == [0, ISA_VERSION, THREADS << 4 | LOG2_DEPTH] + [0] * 9, f"CAPS then 9 more bytes: {r}"
    assert (await host.status())[0] >> 4 == 0, "a RUN after CAPS in the same transaction does nothing"
    assert await host.post(6, 0xCAFE) == 0
    r = await host.xfer([0x2E, 0, 0, 0] + [0x26, 0x12, 0x34, 0] * 2)
    assert r == [0, 1, 0xCA, 0xFE] + [0] * 8, f"TAKE then 8 more bytes: {r}"
    assert await host.take(6) == [0, 0, 0], "the POST after it did nothing"
    r = await host.xfer([0x12, 0, 0, 0, 0] + [0xFF] * 7)
    assert r[5:] == [0] * 7, f"STATUS then 7 more bytes: {r}"


@cocotb.test()
async def test_reset_in_mid_run(dut):
    """Section 8: a reset in the middle of everything puts every flop back. Memory keeps its words."""
    host = await start(dut)
    await load(host, 1, OWNER, 60)
    await host.wrmem(70, PUT_UO1)
    host.other = 0xF8
    dut.uio_in.value = 0xFF
    await host.run(0, 1 << 1)
    await host.wait(40)
    assert pin(dut, 1) == 1, "thread 1 drives uo1"
    assert await host.post(4, 0x1234) == 0
    await host.setpc(1, 0)                              # BUSY
    await host.run(1 << 1, 0)
    await host.wait(16)
    assert await host.rdreg(1, 5) == 0x5A and await host.rdreg(1, 9) != 0, "R5 and S hold something"

    # Reset partway into a byte, with CS low and SCK low, as 9.1 asks while rst_n is low. CS stays low through the
    # reset and after it.
    await FallingEdge(dut.clk)
    host.drive(0, 0, 0)
    await host.wait(HALF)
    for _ in range(3):
        host.drive(0, 0, 1)
        await host.wait(HALF)
        host.drive(0, 1, 1)
        await host.wait(HALF)
    host.drive(0, 0, 0)
    await host.wait(HALF)
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 3, rising=False)
    assert int(dut.uo_out.value) == 0 and int(dut.uio_oe.value) == 0, "outputs clear in reset"
    await ClockCycles(dut.clk, 3, rising=False)
    dut.rst_n.value = 1
    host.other = 0
    dut.uio_in.value = 0
    await host.wait(8)
    reply = await host.shift([0x00, 0, 0])              # the link starts clean in cycle 0, with no CS edge
    host.drive(0, 0, 0)
    await host.wait(HALF)
    host.drive(1, 0, 0)
    await host.wait(4)
    assert reply == [0, ISA_VERSION, THREADS << 4 | LOG2_DEPTH], f"CAPS with CS held low through reset: {reply}"

    assert await host.status() == [0, 0, 0, 0], "no thread runs, nothing late, no word full, BUSY clear"
    for n in range(8):
        assert await host.take(n) == [0, 0, 0], f"mailbox word {n} empty"
    for t in range(THREADS):
        for reg in range(12):
            assert await host.rdreg(t, reg) == 0, f"thread {t} reg8 {reg} not reset"
    assert await host.rdmem(60, len(OWNER)) == OWNER, "memory keeps its words"
    # Thread 0 owns every pin after reset, so thread 1 can no longer move uo1.
    await host.setpc(1, 70)
    await host.run(0, 1 << 1)
    await host.wait(40)
    assert pin(dut, 1) == 0, "pin owners reset to thread 0"


@cocotb.test()
async def test_step_keeps_state(dut):
    """9.3: STEP runs one instruction and touches nothing else: not D, ILL, LATE or a pending event. SETPC leaves the
    event too. An event that fires on a stopped thread still commits its pins and snapshots the inputs."""
    host = await start(dut)
    await host.wrmem(80, STEPS)
    await host.setpc(2, 80)
    host.other = 0xA8
    for _ in range(3):                                  # pcfg, put uo4 1, now
        await host.step(2)
        await host.wait(16)
    assert pin(dut, 4) == 1, "the stepped now committed uo4"
    d0 = await host.rdreg(2, 8)
    await host.step(2)                                  # tick 1: D + 1 has long gone, so the event is late
    await host.wait(16)
    await host.step(2)                                  # put uo4, 0
    await host.wait(16)
    assert await host.rdreg(2, 8) == (d0 + 1) & 0xFFFF, "a STEP does not write D"
    await host.step(2)                                  # the illegal word
    await host.wait(16)
    await host.setpc(2, 86)
    await host.step(2)                                  # idle
    await host.wait(16)
    status = await host.status()
    assert (status[0], status[1] >> 4) == (1 << 2, 1 << 2), f"ILL and LATE stay set through a STEP: {status}"
    await host.step(2)                                  # tick 4000: pending when the step ends
    await host.wait(16)
    assert await host.rdreg(2, 8) == (d0 + 4001) & 0xFFFF
    host.other = 0x50
    await host.step(2)                                  # idle, with the event pending
    await host.wait(16)
    await host.setpc(2, 80)
    assert pin(dut, 4) == 1, "the event has not fired yet"
    await host.wait(2500)
    assert pin(dut, 4) == 0, "the event fired on the stopped thread and committed uo4"
    assert await host.rdreg(2, 9) == 0x0051, "and took a snapshot of the inputs"
    status = await host.status()
    assert (status[0], status[1] >> 4) == (1 << 2, 1 << 2), f"only a start clears ILL and LATE: {status}"


@cocotb.test()
async def test_stop_keeps_waiting(dut):
    """9.3: a thread stopped in a wait keeps WAITING, and a further stop leaves it."""
    host = await start(dut)
    await load(host, 3, WAITER, 100)
    await host.run(0, 1 << 3)
    await host.wait(40)
    for _ in range(2):
        await host.run(1 << 3, 0)
        await host.wait(16)
        assert await host.rdreg(3, 11) & 8, "WAITING in flags bit 3"
    assert (await host.status())[0] >> 4 == 0


@cocotb.test()
async def test_deadline_limits(dut):
    """3.5: a deadline must be 1 to 32,767 cycles after the cycle that arms it, so 32,767 ahead waits and 32,768
    ahead is late."""
    host = await start(dut)
    for k, late in ((16 + 32768, 1), (16 + 32767, 0)):
        await load(host, 1, limit(k), 104)
        await host.run(0, 1 << 1)
        await host.wait(80)
        status = await host.status()
        assert (status[1] >> 5) & 1 == late, f"deadline {k - 16} ahead: LATE {(status[1] >> 5) & 1}"
        await host.run(1 << 1, 0)
        await host.wait(16)


@cocotb.test()
async def test_wide_compare(dut):
    """6.3: cmpi, cmp and djnz look at all 16 bits."""
    host = await start(dut)
    await load(host, 0, WIDE, 112)
    await host.run(0, 1)
    await host.wait(100)
    assert (await host.status())[0] >> 4 == 0, "halted"
    assert await host.rdreg(0, 2) == 0, "0x100 is not 0 to cmpi or cmp"
    assert (await host.rdreg(0, 3), await host.rdreg(0, 4)) == (0x100, 0), "djnz on 0x101 jumps"


@cocotb.test()
async def test_memory_with_one_stopped(dut):
    """9.4: memory commands work while any thread is stopped."""
    host = await start(dut)
    await host.wrmem(0, SPIN)
    await host.run(0, 0x7)
    await host.wait(16)
    await host.wrmem(10, [0x4321])
    assert await host.rdmem(10, 1) == [0x4321]
    assert (await host.status())[3] == 0, "no BUSY with thread 3 stopped"


@cocotb.test()
async def test_bypass_lanes(dut):
    """5.8: a lane in bypass shows its input in the same cycle: uo0 shows ui3, and uo4 to uo7 show ui4 to ui7."""
    host = await start(dut)
    await load(host, 0, BYPASS)
    await host.run(0, 1)
    await host.wait(40)
    for v in (0x00, 0xF8, 0xA8, 0x50, 0x08, 0x80):
        host.other = v
        host.drive(1, 0, 0)
        await host.wait(1)
        got = int(dut.uo_out.value)
        want = (v >> 3 & 1) | (v & 0xF0)
        assert got & 0xF1 == want, f"ui {v:#04x}: uo {got:#04x}"


# ---- the shift units (section 5.9) ----------------------------------------------------------------------------------
# Each program is assembly source, assembled by the published assembler in ../sim/, which reads the instruction table
# from ../docs/ISA.md. The receivers store what they take into memory, and the host reads it back with RDMEM: a TAKE
# over the link takes about 200 cycles, longer than a word on most of these lines.

# UART 8N1 through thread 0's unit on uo0: each byte is one 10-bit word, the start bit in bit 0, the data in bits 8:1
# and the stop bit in bit 9, sent LSB first, one symbol per bit, so the unit times every bit and the thread only feeds
# words. Mailbox 7: cycles per bit. Mailbox 0: the bytes, bit 8 set on the last of a burst. Mailbox 4: after each
# burst, every status bit seen while it drained (bit 7: an underrun). uo2 toggles at the commit that starts a burst.
UNIT_UART = """
.equ TX uo0
start:  sstop txrx           ; a unit isn't reset when its thread starts
        pcfg  TX, PP
        put   TX, 1          ; the line idles high
        pcfg  uo2, PP
        now
        recv  R2, 7          ; cycles per bit
        scfg  R2, TXPER
        ldi   R0, 0x20
        scfg  R0, TXMODE     ; SYMS 0, LSB first, output A takes the value, IDLEA 1
        ldi   R0, 0
        scfg  R0, TXMAP      ; output A's code for symbol 0 is the bit itself
        scfg  R0, TXTAIL     ; no tail, no fraction
        scfg  R0, CODING     ; no CRC, no stuffing
        sstop tx             ; output A shows its idle level, 1
        smap  TX, A
        li    R3, 0x200      ; the stop bit
        now                  ; the switch commits: TX follows output A from here
burst:  ldi   R5, 0
        recv  R0, 0          ; the first byte of a burst
        not   R6
        bt    R6, 0
        put   uo2, T         ; uo2 toggles at the commit that starts the burst
        call  push           ; T = it was the last byte
        sgo   tx
        now                  ; the commit at E starts the send: the start bit and uo2 are on the pad from E + 1
        jt    drain
more:   recv  R0, 0
        call  push
        jf    more
drain:  sstat R4             ; bit 5: the send half is armed, sending or in its tail
        or    R5, R4         ; sstat clears TXUR once read, so keep every bit seen
        bt    R4, 5
        jt    drain
        send  R5, 4
        goto  burst
push:   cp    R1, R0
        andi  R1, 0xFF
        add   R1, R1         ; the data in bits 8:1
        or    R1, R3         ; the stop bit in bit 9
        bt    R0, 8          ; T = the last byte of the burst
        jt    pushl
        txw   R1, 10         ; blocks while the buffer is full
        ret
pushl:  txw   R1, 10, last
        ret
"""

# Thread 1's unit receives a UART on io0 from thread 0's unit (UNIT_UART with TX io0): timed samples, a hunt for each
# start bit's fall, the first sample 1.5 bits after it, 8 bits per reception, so each byte is one reception and one
# word. A uio pin reads its own drive, so no outside wire is needed. Mailbox 6: the receiver's cycles per bit.
UNIT_UART_RX = """
.equ BUF 400
rx:     sstop txrx
        recv  R2, 6          ; cycles per bit
        scfg  R2, RXPER
        cp    R1, R2
        clrt
        shr16 R1, -, -       ; R1 = half a bit
        add   R1, R2
        scfg  R1, RXFIRST    ; 1.5 bits after the fall: the middle of data bit 0
        li    R0, 0x0820
        scfg  R0, RXMODE     ; timed samples, LSB first, HUNT for a fall to 0, RPIN io0
        ldi   R0, 8
        scfg  R0, RXCNT      ; 8 bits end a reception; no sync pattern
        ldi   R0, 0
        scfg  R0, RXEND
        scfg  R0, RXOPT
        scfg  R0, CODING
        li    R5, BUF
        sgo   rx
        now                  ; this commit starts the hunt, which then goes on by itself
rloop:  rxw   R0             ; T = 1: each word is its reception's last
        stp   R0, R5
        goto  rloop
"""

# A 60-byte frame with its CRC-32 appended by thread 0's unit and checked by thread 1's: 802.3 Manchester, 5 cycles
# per bit (half bits of 2 and 3 cycles: TPER 2, TFRAC 128), output A on io2 and its complement, output B, on io3.
# Thread 1's Manchester decoder hunts for io2's rise, waits for the SFD as its sync pattern, and checks the residue.
# Mailbox 0: 0 sends the frame with the unit's FCS (kind fcs); 1 sends its words with no append, then 0x00000000 as
# plain data, a wrong FCS. Mailbox 4: the sender's status after each frame. Mailbox 5: the address after the
# reception's last word in BUF. Mailbox 6: the status word then (bits 4:0, the bits in that word). Mailbox 7: the end
# flags (srd end): bit 0 set when the check passed.
UNIT_CRC32 = """
.equ TDP io2
.equ TDN io3
.equ BUF 400
.equ FRMW 30
start:  sstop txrx
        pcfg  TDP, PP
        pcfg  TDN, PP
        put   TDP, 0
        put   TDN, 0
        now                  ; both lines driven low
        ldi   R0, 0x81
        scfg  R0, TXMODE     ; 2 symbols per bit, LSB first, idle A = B = 0, tail A = 1, B = 0
        li    R0, 0x0401
        scfg  R0, TXMAP      ; A: 1 - b, then b (a 1 is low then high). B: the complement
        ldi   R0, 2
        scfg  R0, TXPER
        li    R0, 0x800F
        scfg  R0, TXTAIL     ; TP_IDL, 15 cycles. TFRAC 128
        li    R0, 0x04C1
        scfg  R0, POLYHI     ; CRC-32
        li    R0, 0x1DB7
        scfg  R0, POLYLO
        li    R0, 0x02FF
        scfg  R0, CODING     ; the send half's CRC: 32 bits, all ones, complemented
        sstop tx
        smap  TDP, A
        smap  TDN, B
        now
frame:  recv  R6, 0          ; which frame
        li    R1, FRM
        ldi   R2, FRMW - 1   ; the words before the last
        li    R0, 0x5555
        txw   R0, 16         ; preamble bytes 1 and 2: no kind, not in the CRC
        txw   R0, 16         ; bytes 3 and 4
        sgo   tx
        now                  ; the commit starts the frame
        txw   R0, 16         ; bytes 5 and 6
        li    R0, 0xD555
        txw   R0, 16         ; byte 7, then the SFD
word:   ldp   R0, R1
        txw   R0, 16, crc
        djnz  R2, word
        ldp   R0, R1
        cmpi  R6, 0
        jf    plain
        txw   R0, 16, fcs, last  ; the last word, then the unit's FCS
        goto  wait
plain:  txw   R0, 16, crc    ; covered, but nothing appended
        ldi   R0, 0
        txw   R0, 16         ; a wrong FCS as two plain words
        txw   R0, 16, last
wait:   sstat R4
        bt    R4, 5
        jt    wait
        send  R4, 4
        goto  frame
FRM:    .data 0xFFFF
        .data 0xFFFF
        .data 0xFFFF
        .data 0x0002
        .data 0x105E
        .data 0x0100
        .data 0x0608
        .data 0x0100
        .data 0x0008
        .data 0x0406
        .data 0x0100
        .data 0x0002
        .data 0x105E
        .data 0x0100
        .data 0xA8C0
        .data 0x0200
        .data 0x0000
        .data 0x0000
        .data 0x0000
        .data 0xA8C0
        .data 0x0100
        .data 0x0000
        .data 0x0000
        .data 0x0000
        .data 0x0000
        .data 0x0000
        .data 0x0000
        .data 0x0000
        .data 0x0000
        .data 0x0000
rx:     sstop txrx
        li    R0, 0x0A61
        scfg  R0, RXMODE     ; Manchester decoder, hunt for a rise to 1, LSB first, RPIN io2
        ldi   R0, 5
        scfg  R0, RXPER      ; 5 cycles per bit
        li    R0, 0x0C00
        scfg  R0, RXOPT      ; RGAIN 2, RACQ for the preamble
        ldi   R0, 0
        scfg  R0, RXEND      ; the first bit with no clock edge ends the frame
        li    R0, 0x8000
        scfg  R0, RXCNT      ; a 16-bit sync pattern, no bit limit
        li    R0, 0xD555
        scfg  R0, RXSYNC     ; the last preamble byte, then the SFD
        li    R0, 0x04C1
        scfg  R0, POLYHI
        li    R0, 0x1DB7
        scfg  R0, POLYLO
        li    R0, 0xC704
        scfg  R0, CHKHI      ; the residue a good frame leaves
        li    R0, 0xDD7B
        scfg  R0, CHKLO
        ldi   R0, 0xFF
        scfg  R0, CODING     ; the receive half's CRC: 32 bits, all ones, checked against CHK
        sgo   rx
        now                  ; the hunt starts at this commit
next:   li    R5, BUF
rloop:  rxw   R0             ; T = 1 on a reception's last word
        stp   R0, R5
        jf    rloop
        sstat R3             ; R3[4:0] = the bits in the last word
        srd   R4, end        ; R4 bit 0 = the check passed
        send  R5, 5
        send  R3, 6
        send  R4, 7
        goto  next
"""

# USB low-speed style data packets from thread 0's unit to thread 1's, 33.33 cycles per bit (TPER 33, TFRAC 85): a
# NRZI pair from idle J, D+ on io4 (output A) and D- on io5 (output B). SYNC and PID go as words of kind stuff, the
# data as kind crc and its last word as fcs and last, so the unit stuffs from the SYNC on and appends CRC-16. The EOP is
# the tail: both lines low for 67 cycles, then J. Thread 1 samples D+ (timed, NRZI, RESYNC on every edge), takes the
# SYNC as its sync pattern, destuffs, leaves the PID out of the CRC (CSKIP 8), checks the CRC-16 residue, and ends at
# the SE0 (REOP with RCLK io5). Mailbox 0: a packet's address: its PID, its number of data bytes (bit 15: send them
# with nothing appended and then 0x0000, a wrong CRC), then the data, two bytes per word, low byte first. Mailboxes 4
# to 7 as in UNIT_CRC32.
UNIT_USB = """
.equ DP io4
.equ DM io5
.equ BUF 400
start:  sstop txrx
        pcfg  DP, PP
        pcfg  DM, PP
        put   DP, 0          ; idle J at low speed: D+ low, D- high
        put   DM, 1
        now
        ldi   R0, 0x58
        scfg  R0, TXMODE     ; SYMS 0, LSB first, TGLA, TGLB, IDLEA 0, IDLEB 1, tail A = B = 0
        li    R0, 0x0101
        scfg  R0, TXMAP      ; both outputs 1 - b, toggling: a 0 changes the line
        ldi   R0, 33
        scfg  R0, TXPER
        li    R0, 0x5543
        scfg  R0, TXTAIL     ; EOP: SE0 for 67 cycles. TFRAC 85
        li    R0, 0x8005
        scfg  R0, POLYHI     ; CRC-16
        ldi   R0, 0
        scfg  R0, POLYLO
        li    R0, 0xE6EF
        scfg  R0, CODING     ; 16 bits, all ones, complemented, the send half; ones stuffed after 6; REOP
        sstop tx             ; the outputs show idle J
        smap  DP, A
        smap  DM, B
        now
pkt:    recv  R1, 0          ; the packet's address
        ldp   R0, R1         ; its PID
        ldi   R2, 0x80
        txw   R2, 8, stuff   ; SYNC
        txw   R0, 8, stuff   ; PID: stuffed, not in the CRC
        sgo   tx
        now                  ; the commit starts the packet
        ldp   R2, R1         ; the number of data bytes
        bt    R2, 15
        jt    wrong
dloop:  ldp   R0, R1         ; the next two
        cmpi  R2, 1
        jt    one
        cmpi  R2, 2
        jt    two
        txw   R0, 16, crc
        subi  R2, 2
        goto  dloop
one:    txw   R0, 8, fcs, last
        goto  wait
two:    txw   R0, 16, fcs, last
        goto  wait
wrong:  li    R3, 0x7FFF
        and   R2, R3
wloop:  ldp   R0, R1
        txw   R0, 16, crc    ; covered, nothing appended
        subi  R2, 2
        cmpi  R2, 0
        jf    wloop
        ldi   R0, 0
        txw   R0, 16, stuff, last    ; a wrong CRC
wait:   sstat R4
        bt    R4, 5
        jt    wait
        send  R4, 4
        goto  pkt

P1:     .data 0x00C3   ; DATA0: 00 01 02 03
        .data 4
        .data 0x0100
        .data 0x0302
P2:     .data 0x004B   ; DATA1: FF FF FF 3F 7E 01 80 00, runs of ones
        .data 8
        .data 0xFFFF
        .data 0x3FFF
        .data 0x017E
        .data 0x0080
P3:     .data 0x00C3   ; DATA0: FF
        .data 1
        .data 0x00FF
P4:     .data 0x004B   ; DATA1: 7F FE 00
        .data 3
        .data 0xFE7F
        .data 0x0000
P5:     .data 0x00C3   ; DATA0: 11 22 33 44, with a wrong CRC
        .data 0x8004
        .data 0x2211
        .data 0x4433

rx:     sstop txrx
        li    R0, 0xDC70
        scfg  R0, RXMODE     ; timed, NRZI (RDIFF), HUNT for D+ rising, RPIN io4, RCLK io5
        ldi   R0, 33
        scfg  R0, RXPER
        li    R0, 0x8155
        scfg  R0, RXOPT      ; RFRAC 85, RESYNC, RGAIN 0; CSKIP 8
        ldi   R0, 16
        scfg  R0, RXFIRST    ; half a bit after an edge
        ldi   R0, 0
        scfg  R0, RXEND
        li    R0, 0x4000
        scfg  R0, RXCNT      ; an 8-bit sync pattern, no bit limit
        li    R0, 0x8000
        scfg  R0, RXSYNC     ; the SYNC field, laid out as a received word
        li    R0, 0x8005
        scfg  R0, POLYHI
        ldi   R0, 0
        scfg  R0, POLYLO
        li    R0, 0x800D
        scfg  R0, CHKHI      ; the residue of a good CRC-16
        ldi   R0, 0
        scfg  R0, CHKLO
        li    R0, 0xE4EF
        scfg  R0, CODING     ; the receive half's CRC-16; the destuffer, ones after 6; REOP
        sgo   rx
        now
next:   li    R5, BUF
rloop:  rxw   R0
        stp   R0, R5
        jf    rloop
        sstat R3
        srd   R4, end
        send  R5, 5
        send  R3, 6
        send  R4, 7
        goto  next
"""

# Thread 0 puts io0 and uo1 on shift output A, with io0 driven low as OUT, and starts a long send; ABORT must release
# io0 at once and take it off the shift output, and leave uo1 on output A, now at its idle level. After the ABORT,
# thread 0 commits io0's enable alone at "after": io0 must then drive OUT, 0, and not output A, since ABORT cleared
# SHSEL too.
UNIT_ABORT = """
start:  sstop txrx
        pcfg  io0, PP
        pcfg  uo1, PP
        put   io0, 0
        put   uo1, 0
        ldi   R0, 0x20
        scfg  R0, TXMODE     ; IDLEA 1
        ldi   R0, 0
        scfg  R0, TXMAP
        scfg  R0, TXTAIL
        scfg  R0, CODING
        ldi   R0, 50
        scfg  R0, TXPER      ; 50 cycles per bit
        sstop tx
        smap  io0, A
        smap  uo1, A
        li    R1, 0x5555
        txw   R1, 16
        txw   R1, 16
        sgo   tx
        now                  ; io0 and uo1 follow output A from here, and the send starts
spin:   goto  spin
after:  oe    io0, 1
        now                  ; commits OUT, OE and the output switch of io0
spin2:  goto  spin2
"""

# Output switches. Thread 0 owns every pin, as after reset. Thread 1 runs smap on uo2 first, which it doesn't own, so
# nothing happens: thread 0's commit leaves uo2 on OUT. (No pcfg of uo2 here: as a claim, it would clear the switch.) Thread 0 puts uo1 and io0 (push-pull, OUT 0) on output A, at its idle level 1, and io1 (open drain)
# on output B, at its idle level 0, which pulls io1 low. At the host's word in mailbox 7 it claims uo1 again with pcfg,
# which takes uo1 to OUT at once; at the next word it commits, which leaves uo1 there.
UNIT_SWITCH = """
start:  sstop txrx
        ldi   R0, 0x20
        scfg  R0, TXMODE     ; IDLEA 1, IDLEB 0
        sstop tx             ; LA 1, LB 0
        pcfg  uo1, PP
        put   uo1, 0
        pcfg  io0, PP
        put   io0, 0
        pcfg  io1, OD        ; released
        smap  uo1, A
        smap  io0, A
        smap  io1, B
        now                  ; uo1 and io0 show LA, 1; io1 shows LB, 0: pulled low
        recv  R0, 7
        pcfg  uo1, PP        ; takes uo1 off output A at once
        recv  R0, 7
        now                  ; its switch's shadow is clear too, so the commit leaves it on OUT
spin:   goto  spin
other:  smap  uo2, A         ; thread 1 doesn't own uo2
        halt
"""

# sstat's sticky bits and its T. The receive half samples ui4 every cycle from its commit, with no end, and nothing
# takes the words, so RXOVR sets; sstop rx then stops it, which leaves RXOVR. sstat reads RXOVR with TXUR clear, and T
# must be 1 (R5 := 1); a second sstat reads both clear, T 0 (R6 := 1). Then sgo tx with nothing to send underruns at
# its commit, and sstat reads TXUR (R7 := 1).
UNIT_SSTAT = """
start:  sstop txrx
        ldi   R0, 1
        scfg  R0, RXPER      ; a sample every cycle
        scfg  R0, RXFIRST
        li    R0, 0x0400
        scfg  R0, RXMODE     ; timed samples of ui4, no hunt: the reception starts at the commit
        ldi   R0, 0
        scfg  R0, RXCNT      ; no sync pattern, no bit limit
        scfg  R0, RXEND      ; no timeout
        scfg  R0, CODING
        sgo   rx
        now
        tick  100            ; a word every 16 cycles, and none taken
        sstop rx             ; waits for the tick, then stops the reception; RXOVR stays
        sstat R1
        jf    no1
        ldi   R5, 1
no1:    sstat R2
        jt    no2
        ldi   R6, 1
no2:    sgo   tx
        now                  ; nothing to send: the start underruns
        idle
        sstat R3
        jf    no3
        ldi   R7, 1
no3:    halt
"""

# World instructions wait for a pending event (section 3.6). After the host's word in mailbox 7: sgo after a tick waits
# for it, so the tick's commit doesn't start the send: the commit after sgo does, which also raises uo2. During the
# send, scfg of TXMAP after a second tick waits for that tick too, then turns output A to a constant 1, and uo4 rises
# at the commit after it. Had scfg run at once, the line would have stopped toggling 28 cycles earlier.
UNIT_WORLD = """
start:  sstop txrx
        pcfg  uo1, PP
        pcfg  uo2, PP
        pcfg  uo4, PP
        ldi   R0, 0x20
        scfg  R0, TXMODE     ; IDLEA 1
        ldi   R0, 0
        scfg  R0, TXMAP      ; output A: the bit
        scfg  R0, TXTAIL
        scfg  R0, CODING
        ldi   R0, 4
        scfg  R0, TXPER      ; 4 cycles per bit
        ldi   R3, 3
        sstop tx
        smap  uo1, A
        now                  ; uo1 follows output A, idle 1
        recv  R0, 7
        now                  ; D from now on, so the tick below is 60 cycles ahead, not late
        li    R1, 0xAAAA
        txw   R1, 16         ; 0, 1, 0, 1, ...
        tick  60
        sgo   tx             ; waits for the tick
        put   uo2, 1
        now                  ; this commit starts the send and raises uo2
        txw   R1, 16
        tick  40
        scfg  R3, TXMAP      ; waits for the tick, then output A is a constant 1
        put   uo4, 1
        now
spin:   goto  spin
"""

# Every slot written with a value that has bits in reserved places, then one word with its CRC-16 appended, which
# leaves the CRC register at its initial value: the top 16 bits all ones. Then halt.
UNIT_SLOTS = (0x8061, 0x0401, 0x0003, 0x1200, 0xA5C3, 0x1234, 0x0005, 0x00C8, 0x4010, 0x8000, 0x0C55, 0x8005,
              0x0F0F, 0x800D, 0xF0F0, 0x02AF)
SLOT_NAMES = ('TXMODE', 'TXMAP', 'TXPER', 'TXTAIL', 'RXMODE', 'RXPER', 'RXFIRST', 'RXEND', 'RXCNT', 'RXSYNC', 'RXOPT',
              'POLYHI', 'POLYLO', 'CHKHI', 'CHKLO', 'CODING')
UNIT_RDREG = """
start:  sstop txrx
""" + ''.join('        li    R0, %#06x\n        scfg  R0, %s\n' % (v, n) for v, n in zip(UNIT_SLOTS, SLOT_NAMES)) + """
        ldi   R1, 0x5A
        txw   R1, 8, fcs, last
        sgo   tx
        now
wait:   sstat R4
        bt    R4, 5
        jt    wait
        halt
        .org  500
under:  sgo   tx             ; nothing to send: the start underruns at the commit
        now
        idle
        idle
        halt
"""


def assemble(src, expect=()):
    """The words and labels of a program, from the published assembler. Any warning must be one the test expects."""
    sys.path.insert(0, SIM_DIR)
    sys.dont_write_bytecode = True      # leave no cache in sim/
    import asm
    img = asm.assemble(src, base=0, depth=DEPTH)
    assert all(any(e in w for e in expect) for w in img.warnings), img.warnings
    return img


async def load_image(host, img):
    """WRMEM of each run of consecutive words."""
    addrs = sorted(img.words)
    run = [addrs[0]]
    for a in addrs[1:] + [None]:
        if a is not None and a == run[-1] + 1:
            run.append(a)
            continue
        await host.wrmem(run[0], [img.words[x] for x in run])
        if a is not None:
            run = [a]


async def take_wait(host, n, limit=200):
    """The next word that a thread sends to mailbox word n."""
    for _ in range(limit):
        full, hi, lo = await host.take(n)
        if full:
            return hi << 8 | lo
        await host.wait(50)
    raise AssertionError(f"nothing in mailbox word {n}")


async def post_wait(host, n, value, limit=200):
    """POST value to mailbox word n, again while the word is still full."""
    for _ in range(limit):
        if await host.post(n, value) == 0:
            return
        await host.wait(20)
    raise AssertionError(f"mailbox word {n} never emptied")


class Board:
    """The uio pads as on the demo board: a pin the chip drives reads its own drive (section 5.2), one it doesn't reads
    1 through a pull-up. Records every pad, uo in bits 7:0 and io in bits 15:8, once per cycle. uo3 is MISO, the host
    link's, and records as 0: after an RDMEM's last word it shows the word after it, which may never have been written."""

    def __init__(self, dut):
        self.dut = dut
        self.levels = []

    async def run(self):
        while True:
            await FallingEdge(self.dut.clk)
            en = int(self.dut.uio_oe.value)
            io = (int(self.dut.uio_out.value) & en) | (~en & 0xFF)
            self.dut.uio_in.value = io
            uo = str(self.dut.uo_out.value)[::-1]        # bit 0 first
            assert all(c in '01' for i, c in enumerate(uo) if i != MISO), f"uo_out {uo[::-1]}"
            self.levels.append(sum(1 << i for i, c in enumerate(uo) if c == '1' and i != MISO) | io << 8)

    def pin(self, p, c):
        return (self.levels[c] >> p) & 1


def uart_frames(board, p, bit, since=0):
    """8N1 frames at bit cycles per bit on pad p from cycle since: (start cycle, byte, stop bit) for each fall from
    high, each bit read in its middle. Every edge from the first start bit to the last stop bit must be on the bit
    grid of that start bit, as when one commit times the whole burst."""
    out, c, first = [], max(since, 1), None
    while c + 10 * bit < len(board.levels):
        if board.pin(p, c - 1) and not board.pin(p, c):
            first = c if first is None else first
            assert (c - first) % bit == 0, f"start bit {c - first} cycles after the first, off the {bit}-cycle grid"
            byte = sum(board.pin(p, c + bit // 2 + (i + 1) * bit) << i for i in range(8))
            out.append((c, byte, board.pin(p, c + bit // 2 + 9 * bit)))
            for k in range(c + 1, c + 10 * bit):
                if board.pin(p, k) != board.pin(p, k - 1):
                    assert (k - c) % bit == 0, f"edge {k - c} cycles into the frame, off the {bit}-cycle grid"
            c += 10 * bit - 1
        c += 1
    return out


@cocotb.test()
async def test_unit_uart_tx(dut):
    """5.9.5: a unit sends UART bursts on a uo pin. A burst's start bit leaves in the cycle the commit that starts it
    moves an ordinary pin, every edge of the burst is on its grid, the line idles high between bursts, and a burst fed
    in time never underruns."""
    host = await start(dut)
    board = Board(dut)
    cocotb.start_soon(board.run())
    img = assemble(UNIT_UART)
    await load_image(host, img)
    await host.setpc(0, img.labels['start'])
    await host.run(0, 1)
    await host.wait(60)
    assert pin(dut, 0) == 1, "the line idles high"
    bit = 40
    await post_wait(host, 7, bit)
    for burst in ([0x55, 0xA3, 0x00, 0xFF], [0x81], [0x0F, 0xF0]):
        since = len(board.levels)
        for i, b in enumerate(burst):
            await post_wait(host, 0, b | (0x100 if i == len(burst) - 1 else 0))
        seen = await take_wait(host, 4)
        await host.wait(2 * bit)
        frames = uart_frames(board, 0, bit, since)
        assert [b for _, b, _ in frames] == burst, f"burst {burst}: decoded {frames}"
        marks = [c for c in range(since + 1, len(board.levels)) if board.pin(2, c) != board.pin(2, c - 1)]
        assert marks == [frames[0][0]], f"the start bit leaves with the commit's uo2 edge: {frames[0][0]}, uo2 {marks}"
        assert all(stop for _, _, stop in frames), "every stop bit high"
        assert not seen & 0x80, f"no underrun: status bits seen {seen:#06x}"
        assert pin(dut, 0) == 1, "idle high after the burst"


@cocotb.test()
async def test_unit_uio_to_unit(dut):
    """5.9.5 and 5.9.6: thread 0's unit sends a UART on io0; thread 1's unit, hunting on the same pin, which reads its
    own drive, receives every byte as one word. The pad decodes to the same bytes."""
    host = await start(dut)
    board = Board(dut)
    cocotb.start_soon(board.run())
    img = assemble(UNIT_UART.replace('.equ TX uo0', '.equ TX io0') + UNIT_UART_RX)
    await load_image(host, img)
    await host.setpc(0, img.labels['start'])
    await host.setpc(1, img.labels['rx'])
    bit = 24
    await host.post(6, bit)
    await host.run(0, 3)
    await host.wait(100)
    await post_wait(host, 7, bit)
    data = [0x55, 0x00, 0xFF, 0xC3, 0x3C, 0x81, 0x7E, 0x01]
    since = len(board.levels)
    for i, b in enumerate(data):
        await post_wait(host, 0, b | (0x100 if i == len(data) - 1 else 0))
    seen = await take_wait(host, 4)
    await host.wait(4 * bit)
    assert not seen & 0x80, f"no underrun: {seen:#06x}"
    assert [b for _, b, _ in uart_frames(board, 8, bit, since)] == data, "the io0 pad"
    assert await host.rdmem(400, len(data)) == data, "the words thread 1's unit received"


@cocotb.test()
async def test_unit_crc32_frame(dut):
    """5.9.7: a frame with the CRC-32 that one unit appends, checked by another unit's receive half: the words received
    are the frame and the FCS that zlib computes, sent low byte first, and the check passes; with a wrong FCS it fails;
    and the next good frame passes again, since an append's end reloads the initial value. TD- is the complement of
    TD+ on every cycle of every frame."""
    host = await start(dut)
    board = Board(dut)
    cocotb.start_soon(board.run())
    img = assemble(UNIT_CRC32)
    await load_image(host, img)
    await host.setpc(0, img.labels['start'])
    await host.setpc(1, img.labels['rx'])
    await host.run(0, 3)
    await host.wait(600)
    frm = [img.words[img.labels['FRM'] + i] for i in range(30)]
    fcs = zlib.crc32(b''.join(bytes((w & 255, w >> 8)) for w in frm))
    for kind in (0, 1, 0):
        since = len(board.levels)
        await post_wait(host, 0, kind)
        sent = await take_wait(host, 4)
        end = await take_wait(host, 5)
        last = await take_wait(host, 6)
        flags = await take_wait(host, 7)
        words = await host.rdmem(400, end - 400)
        want = frm + ([fcs & 0xFFFF, fcs >> 16] if kind == 0 else [0, 0])
        assert len(words) == 33 and (last & 31) == 0, f"32 full words and a last word of 0 bits: {len(words)}, {last:#x}"
        assert words[:32] == want, f"frame {kind}: {[hex(x) for x in words[28:33]]}, FCS {fcs:08x}"
        assert (flags & 1) == (kind == 0), f"end flag 0 {'set' if kind == 0 else 'clear'}: {flags:#x}"
        assert not sent & 0x80 and not last & (1 << 10), "no underrun, no overrun"
        span = [c for c in range(since + 1, len(board.levels)) if board.pin(10, c) != board.pin(10, c - 1)]
        assert len(span) > 500, "TD+ toggles through the frame"
        assert all(board.pin(11, c) != board.pin(10, c) for c in range(span[0], span[-1])), "TD- is TD+ inverted"


def crc16_usb(data):
    """CRC-16/USB from its catalog definition: poly 0x8005 reflected (0xA001), init 0xFFFF, xorout 0xFFFF."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc ^ 0xFFFF


def usb_packet(board, dp, dm, since):
    """One packet on the D+/D- pads after cycle since: NRZI from idle J (D+ low), a bit every 33 + 85/256 cycles from
    the first edge, a stuff bit after six ones dropped, up to the SE0. Returns (bits, stuff bits, SE0 cycles)."""
    t0 = next(c for c in range(since + 1, len(board.levels)) if board.pin(dp, c) and not board.pin(dp, c - 1))
    per = 33 + 85 / 256
    prev, ones, bits, stuffed, k = 0, 0, [], [], 0
    while True:
        c = int(t0 + (k + 0.5) * per)
        a, b = board.pin(dp, c), board.pin(dm, c)
        if a == 0 and b == 0:
            se0 = 0
            while not board.pin(dp, c + se0) and not board.pin(dm, c + se0):
                se0 += 1
            return bits, stuffed, se0 + int(0.5 * per)
        bit = 1 if a == prev else 0
        prev = a
        k += 1
        if ones == 6:
            stuffed.append(bit)
            ones = 0
            continue
        ones = ones + 1 if bit else 0
        bits.append(bit)


@cocotb.test()
async def test_unit_usb(dut):
    """5.9.5 to 5.9.7: USB low-speed style packets, NRZI with bit stuffing and CRC-16 appended by thread 0's unit. The
    pads decode to SYNC, PID, data and the CRC-16 of the catalog definition, every stuffed bit is a 0 after six ones,
    and the EOP is an SE0 of about 2 bits. Thread 1's unit receives PID, data and CRC destuffed, and its check passes,
    except on the packet sent with a wrong CRC."""
    assert crc16_usb(b'123456789') == 0xB4C8
    host = await start(dut)
    board = Board(dut)
    cocotb.start_soon(board.run())
    img = assemble(UNIT_USB)
    await load_image(host, img)
    await host.setpc(0, img.labels['start'])
    await host.setpc(1, img.labels['rx'])
    await host.run(0, 3)
    await host.wait(600)
    stuffs = 0
    for lab in ('P1', 'P2', 'P3', 'P4', 'P5'):
        a = img.labels[lab]
        pid, n = img.words[a], img.words[a + 1]
        wrong, n = n >> 15, n & 0x7FFF
        data = b''.join(bytes((w & 255, w >> 8)) for w in (img.words[a + 2 + i] for i in range((n + 1) // 2)))[:n]
        crc = crc16_usb(data)
        want = [0x80, pid] + list(data) + ([0, 0] if wrong else [crc & 255, crc >> 8])
        since = len(board.levels)
        await post_wait(host, 0, a)
        sent = await take_wait(host, 4)
        end = await take_wait(host, 5)
        last = await take_wait(host, 6)
        flags = await take_wait(host, 7)
        bits, stuffed, se0 = usb_packet(board, 12, 13, since)
        got = [sum(bits[8 * j + k] << k for k in range(8)) for j in range(len(bits) // 8)]
        assert got == want and len(bits) % 8 == 0, f"{lab} at the pads: {[hex(x) for x in got]}, want {[hex(x) for x in want]}"
        assert all(b == 0 for b in stuffed), f"{lab}: a stuffed bit is a 0"
        assert 60 <= se0 <= 80, f"{lab}: SE0 for {se0} cycles"
        stuffs += len(stuffed)
        words = await host.rdmem(400, end - 400)
        rbits = []
        for j, w in enumerate(words):
            rbits += [(w >> x) & 1 for x in range(16 if j < len(words) - 1 else last & 31)]
        rby = [sum(rbits[8 * j + k] << k for k in range(8)) for j in range(len(rbits) // 8)]
        assert rby == want[1:] and len(rbits) == 8 * len(want[1:]), f"{lab} received: {[hex(x) for x in rby]}"
        assert (flags & 1) == (not wrong), f"{lab}: end flag 0 {'clear' if wrong else 'set'}: {flags:#x}"
        assert not sent & 0x80, f"{lab}: no underrun"
    assert stuffs >= 3, "the runs of ones were stuffed"


@cocotb.test()
async def test_unit_abort(dut):
    """9.3: ABORT stops every unit and takes every uio pin off its shift output at once, and SHSEL too, so the next
    commit shows OUT there. A uo pin keeps its switch and shows the output's idle level."""
    host = await start(dut)
    img = assemble(UNIT_ABORT)
    await load_image(host, img)
    await host.setpc(0, img.labels['start'])
    await host.run(0, 1)
    await host.wait(200)
    assert (int(dut.uio_oe.value) & 1) == 1, "io0 driven"
    levels = set()
    for _ in range(400):
        await FallingEdge(dut.clk)
        levels.add((int(dut.uio_out.value) & 1, pin(dut, 1)))
    assert levels == {(0, 0), (1, 1)}, f"io0 and uo1 both follow output A through the send: {levels}"
    await host.abort()
    assert int(dut.uio_oe.value) == 0, "ABORT releases io0"
    assert pin(dut, 1) == 1, "uo1 stays on output A, at its idle level"
    for _ in range(200):
        await FallingEdge(dut.clk)
        assert pin(dut, 1) == 1, "the send stopped"
    assert ((await host.rdreg(0, 12)) & 0x2020) == 0x2000, "the send half is idle, LA 1"
    await host.setpc(0, img.labels['after'])
    await host.run(0, 1)
    await host.wait(60)
    assert (int(dut.uio_oe.value) & 1) == 1 and (int(dut.uio_out.value) & 1) == 0, "io0 shows OUT, 0, not output A"


@cocotb.test()
async def test_unit_output_switch(dut):
    """5.3, 5.9.4 and 9.3: smap writes only a pin its thread owns; pcfg takes a pin off its shift output at once, and
    clears the switch's shadow, so the next commit leaves it on OUT; ABORT takes every uio pin off its shift output,
    so an open-drain pin that a shift output holds low is released, and a push-pull one shows OUT again."""
    host = await start(dut)
    img = assemble(UNIT_SWITCH, expect=('uo2 written before any pcfg',))     # thread 1's smap, the point here
    await load_image(host, img)
    await host.setpc(1, img.labels['other'])
    await host.run(0, 1 << 1)
    await host.wait(40)
    await host.setpc(0, img.labels['start'])
    await host.run(0, 1)
    await host.wait(200)
    uo, out, oe = int(dut.uo_out.value), int(dut.uio_out.value), int(dut.uio_oe.value)
    assert (uo >> 1) & 1 == 1, "uo1 follows output A at its idle level"
    assert (uo >> 2) & 1 == 0, "another thread's smap did not switch uo2"
    assert oe & 3 == 3 and out & 3 == 1, f"io0 drives output A's 1, io1 is pulled low by output B: oe {oe:#x} out {out:#x}"
    await post_wait(host, 7, 1)
    await host.wait(60)
    assert pin(dut, 1) == 0, "pcfg took uo1 back to OUT at once"
    await post_wait(host, 7, 2)
    await host.wait(60)
    assert pin(dut, 1) == 0, "and the commit after it left uo1 there"
    await host.abort()
    out, oe = int(dut.uio_out.value), int(dut.uio_oe.value)
    assert oe == 0, f"ABORT releases every uio pin, io1 too: oe {oe:#x}"
    assert out & 1 == 0, "io0 is back on OUT, 0"


@cocotb.test()
async def test_unit_sstat(dut):
    """5.9.4: sstat clears the sticky bits it reads as 1, and its T is TXUR or RXOVR: an overrun alone sets T, a
    second sstat reads nothing sticky and T 0, and an underrun sets TXUR, which RDREG 12 shows until sstat reads it."""
    host = await start(dut)
    img = assemble(UNIT_SSTAT)
    await load_image(host, img)
    await host.setpc(1, img.labels['start'])
    await host.run(0, 1 << 1)
    await host.wait(400)
    assert (await host.status())[0] >> 4 == 0, "halted"
    r1, r2, r3 = (await host.rdreg(1, 1), await host.rdreg(1, 2), await host.rdreg(1, 3))
    assert r1 & 0x1400 == 0x1400 and not r1 & 0x80, f"the first sstat: RXOVR and RXSEEN, not TXUR: {r1:#06x}"
    assert await host.rdreg(1, 5) == 1, "and T 1, from RXOVR alone"
    assert r2 & 0x1480 == 0, f"the second: the sticky bits are clear: {r2:#06x}"
    assert await host.rdreg(1, 6) == 1, "and T 0"
    assert r3 & 0x80 and await host.rdreg(1, 7) == 1, f"the underrun: TXUR and T: {r3:#06x}"
    assert (await host.rdreg(1, 12)) & 0x1480 == 0, "sstat cleared TXUR too"


@cocotb.test()
async def test_unit_world_stall(dut):
    """3.6 and 5.9.4: scfg and sgo are world instructions, so with an event pending they wait for it. A send armed
    after a tick starts at the next commit, not at the tick's; a slot written after a tick changes the line only after
    that tick has fired."""
    host = await start(dut)
    img = assemble(UNIT_WORLD)
    await load_image(host, img)
    await host.setpc(0, img.labels['start'])
    await host.run(0, 1)
    await host.wait(300)
    assert pin(dut, 1) == 1 and pin(dut, 2) == 0 and pin(dut, 4) == 0, "set up: uo1 on output A at 1"
    board = Board(dut)
    cocotb.start_soon(board.run())
    await post_wait(host, 7, 1)
    await host.wait(500)
    edges = lambda p: [c for c in range(1, len(board.levels)) if board.pin(p, c) != board.pin(p, c - 1)]  # noqa: E731
    u1, u2, u4 = edges(1), edges(2), edges(4)
    assert len(u2) == 1 and len(u4) == 1, f"one edge on each marker: uo2 {u2}, uo4 {u4}"
    assert u1 and u1[0] == u2[0], f"the send starts at the commit after sgo: uo1 {u1[:3]}, uo2 {u2}"
    assert all(b - a == 4 for a, b in zip(u1, u1[1:])), f"4 cycles per bit up to the scfg: {u1}"
    assert u4[0] - u1[-1] <= 16, f"output A stopped toggling just before uo4 rose: last edge {u1[-1]}, uo4 {u4[0]}"


@cocotb.test()
async def test_unit_rdreg(dut):
    """9.2: RDREG 16 + k is slot k without its reserved bits, 12 the status word with nothing cleared, 13 and 14 the
    CRC register's halves, 15 RXPE, and 32 up read 0. A unit's registers read 0 after reset."""
    host = await start(dut)
    for t in range(THREADS):
        for reg in (12, 13, 14, 15, 16, 21, 31):
            assert await host.rdreg(t, reg) == 0, f"thread {t} reg8 {reg} after reset"
    img = assemble(UNIT_RDREG)
    await load_image(host, img)
    await host.setpc(2, img.labels['start'])
    await host.run(0, 1 << 2)
    await host.wait(400)
    assert (await host.status())[0] >> 4 == 0, "halted after the send"
    keep = [0x7FFF] + [0xFFFF] * 15
    for k, v in enumerate(UNIT_SLOTS):
        assert await host.rdreg(2, 16 + k) == v & keep[k], f"slot {k}"
    status = await host.rdreg(2, 12)
    assert status == 0x6000, f"idle, nothing sticky, LA and LB at their idle levels 1: {status:#06x}"
    assert (await host.rdreg(2, 13), await host.rdreg(2, 14)) == (0x0000, 0xFFFF), "CRC-16 at its initial value"
    assert await host.rdreg(2, 15) == 0, "RXPE"
    for reg in (32, 47, 63, 128, 255):
        assert await host.rdreg(2, reg) == 0, f"reg8 {reg} reads 0"
    assert await host.rdreg(3, 16) == 0, "another thread's slots are its own"
    assert await host.rdreg(3, 12) == 0, "and its unit took nothing from thread 2's txw"
    # sgo tx with nothing to send underruns at its commit, and TXUR stays set however often RDREG reads it.
    await host.setpc(2, img.labels['under'])
    await host.run(0, 1 << 2)
    await host.wait(60)
    for _ in range(2):
        assert (await host.rdreg(2, 12)) & 0x80, "TXUR, read by RDREG, stays set"
    await host.setpc(3, img.labels['under'])                            # thread 3's unit: nothing was pushed into it
    await host.run(0, 1 << 3)
    await host.wait(60)
    assert (await host.rdreg(3, 12)) & 0x80, "thread 3's start underruns too: thread 2's txw went to thread 2's unit"


# ---- the bypass override: programs/advanced/uart_bypass.s ----------------------------------------------------------

PROGRAMS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'programs')


class Line:
    """Drives ui4 with 8N1 frames, LSB first, on an exact grid of cycles, and records once per cycle, 1 ns after the
    falling edge where the inputs change, ui4, uo4 and the lane's switch: what the pad equation of 5.2 shows in that
    cycle. Cycles count from the line's start."""

    def __init__(self, dut, host, byp):
        self.dut, self.host, self.byp = dut, host, byp
        self.c, self.sched, self.rec = 0, [], []

    def send(self, lead, data, bit, gaps=()):
        """Frames from `lead` cycles from now. Returns the start cycle of each frame and the cycle after the last."""
        t, starts = self.c + lead, []
        for k, v in enumerate(data):
            starts.append(t)
            self.sched += [(t + j * bit, b) for j, b in enumerate([0] + [(v >> i) & 1 for i in range(8)] + [1])]
            t += 10 * bit + (gaps[k] if k < len(gaps) else 0)
        return starts, t

    async def run(self):
        level, i = 1, 0
        while True:
            await FallingEdge(self.dut.clk)
            while i < len(self.sched) and self.sched[i][0] <= self.c:
                level = self.sched[i][1]
                i += 1
            self.host.other = (self.host.other & ~0x10) | level << 4
            self.dut.ui_in.value = (self.host.other & 0xF8) | self.host.link
            await Timer(1, unit="ns")
            uo = str(self.dut.uo_out.value)[::-1]        # bit 0 first; MISO, bit 3, is not read
            self.rec.append((level, int(uo[4]), (int(self.byp.value) >> 4) & 1))
            self.c += 1

    async def until(self, c):
        while self.c < c:
            await FallingEdge(self.dut.clk)


@cocotb.test()
async def test_bypass_override(dut):
    """5.2 and 5.8, through programs/advanced/uart_bypass.s: a UART line passes from ui4 to uo4 on the lane, unchanged
    while no rule is armed. With a rule, the chosen frame's data bits leave as the host's byte, each on the incoming
    frame's own bit boundary, to the cycle, the lane switches only where the program's header says (3 cycles into
    the start bit, 9 into the stop bit), and every other cycle shows ui4. At 16 cycles per bit with a frame rule and
    at 50 with a byte rule, on two threads. At gate level the lane is the netlist's gates: the pins are checked the
    same way, the switch is read from the RTL core that runs beside the netlist, and the two must agree on every pin
    in every cycle."""
    gates = os.environ.get("GATES") == "yes"
    host = await start(dut)
    if gates:
        dut.compare.value = 1
    core = dut.rtl_core if gates else dut.user_project.core
    with open(os.path.join(PROGRAMS_DIR, 'advanced', 'uart_bypass.s')) as fh:
        img = assemble(fh.read())
    await load_image(host, img)
    line = Line(dut, host, core.pins.byp_q)
    cocotb.start_soon(line.run())
    runs = ((1, 16, 2, 0xA5, [0x31, 0x7E, 0x00, 0xFF, 0x0F, 0xC3], ()),
            (2, 50, 0x100 | 0x7E, 0x5A, [0x10, 0x7E, 0x81, 0x55, 0x01], (0, 7, 0, 60, 0)))
    for n, (th, bit, rule, new, data, gaps) in enumerate(runs):
        if n:
            await host.run(1 << runs[n - 1][0], 0)
        await host.setpc(th, img.labels['start'])
        await host.run(0, 1 << th)
        await post_wait(host, 7, bit)
        lo = line.c
        _, end = line.send(40, [0x55, 0xC3], bit)               # no rule armed yet: these pass as they are
        await line.until(end + 2 * bit)
        await post_wait(host, 1, new)
        await post_wait(host, 0, rule)
        ack = await take_wait(host, 4)
        starts, end = line.send(40, data, bit, gaps)
        out = await take_wait(host, 5)
        await line.until(end + 3 * bit)
        k = rule if rule < 0x100 else data.index(rule & 255) + 1
        s = starts[k]
        assert ack == rule and out == data[k], \
            f"thread {th}: MACK {ack:#x}, MOUT {out:#x}; rule {rule:#x}, byte {data[k]:#x}"
        rec = line.rec[lo:]
        want = [u for u, _, _ in rec]
        for j in range(8):
            want[s + (j + 1) * bit - lo:s + (j + 2) * bit - lo] = [(new >> j) & 1] * bit
        diff = [lo + i for i, (got, w) in enumerate(zip((o for _, o, _ in rec), want)) if got != w]
        assert not diff, f"thread {th}, {bit} cycles per bit: uo4 differs from the stimulus in {len(diff)} cycles, " \
                         f"the first {diff[0]}; frame {k} starts at {s}"
        flips = [(lo + i, b) for i, (_, _, b) in enumerate(rec) if i and b != rec[i - 1][2]]
        assert flips == [(s + 3, 0), (s + 9 * bit + 9, 1)], f"thread {th}: lane switches {flips}, frame {k} at {s}"
    if gates:
        assert int(dut.mismatches.value) == 0, f"the netlist and the RTL differ in {int(dut.mismatches.value)} cycles"


# ---- test_matches_simulator: the RTL beside the published simulator ------------------------------------

# Section 9.1 at its limits: SCK period 6 or more (clk/6), each SCK low and high 2 or more, CS high 3 or more, CS moving
# only while SCK is low. DUTY is (low, high) cycles of SCK; SETUP is the cycles from CS falling to the first SCK rise,
# LAST the cycles SCK stays high on the last bit, HOLD the cycles from the last SCK fall to CS rising, and GAP the
# cycles CS stays high between transactions. The first of each is the fastest the section allows.
DUTY = ((2, 4), (4, 2), (3, 3), (3, 3), (2, 4), (4, 2), (3, 3), (4, 4))
SETUP = (1, 2, 3, 6)
LAST = (2, 3)
HOLD = (1, 2, 3)
GAP = (3, 3, 4, 5, 9, 17)
CLAIMS = ((0, 1, 8, 9), (2, 4, 10, 11), (5, 6, 12, 13), (7, 14, 15))     # pins each thread claims first: BYP on a
LANE_PINS = (0, 4, 5, 6, 7)                                             # lane, PP on any other pin
FIXED = ('caps', 'run', 'setpc', 'status', 'step', 'rdreg', 'post', 'take', 'abort')
STOPPERS = ('halt', 'recv', 'send', 'rxw', 'txw')   # rows that stop or can block a thread for long


def status_bytes(sim):
    def nib(name):
        return sum(getattr(t, name) << t.TID for t in sim.th)
    return [nib('RUN') << 4 | nib('ILL'), nib('LATE') << 4 | nib('STALE'), sum(f << i for i, f in enumerate(sim.FULL)), sim.BUSY]


class Txn:
    """One host transaction and what it should do. on_byte runs in cycle h of each finished byte, after the
    simulator has run that cycle, which is where the simulator serves a command."""

    def __init__(self, rng, kind, unlisted, word=None):
        self.kind = kind
        self.want = {}                  # reply byte index -> value
        self.dead = False
        self.reads = 0
        self.rdreg = None               # candidate RDREG values, filled from h + 2 to h + 5
        if kind == 'caps':
            self.mosi = [0x00, 0, 0]
            self.want = {1: ISA_VERSION, 2: THREADS << 4 | LOG2_DEPTH}
        elif kind == 'wrmem':
            self.addr = rng.randrange(DEPTH + 40)
            self.data = [word(rng) if word else rng.getrandbits(16) for _ in range(rng.randint(1, 4))]
            if (self.addr - 12) % DEPTH < 40:
                self.addr = 64                  # leave the pin claims at 16 alone
            self.mosi = [0x01, self.addr >> 8, self.addr & 255] + [x for w in self.data for x in (w >> 8, w & 255)]
        elif kind == 'rdmem':
            self.addr = rng.randrange(DEPTH + 40)
            self.count = rng.randint(1, 4)
            self.mosi = [0x02, self.addr >> 8, self.addr & 255, 0, 0] + [0] * (2 * self.count)
        elif kind == 'run':             # mostly a stop of some threads, or a start of all, so threads come and go
            self.a, self.b = rng.choice(((rng.getrandbits(4), 0), (0, 15), (0, 15), (rng.getrandbits(8), rng.getrandbits(8))))
            self.mosi = [0x10, self.a, self.b]
        elif kind == 'setpc':
            self.a, self.b = rng.getrandbits(8), rng.randrange(DEPTH + 40)
            self.mosi = [0x11, self.a, self.b >> 8, self.b & 255]
        elif kind == 'status':
            self.mosi = [0x12, 0, 0, 0, 0]
        elif kind == 'step':
            self.a = rng.getrandbits(8)
            self.mosi = [0x13, self.a]
        elif kind == 'rdreg':
            self.a, self.b = rng.getrandbits(8), rng.choice((8, 10, 11, 11, rng.randrange(8), rng.randrange(13), rng.getrandbits(8)))
            self.mosi = [0x14, self.a, self.b, 0, 0, 0]
        elif kind == 'post':
            self.a, self.b = rng.randrange(8), rng.getrandbits(16)
            self.mosi = [0x20 + self.a, self.b >> 8, self.b & 255, 0]
        elif kind == 'take':
            self.a = rng.randrange(8)
            self.mosi = [0x28 + self.a, 0, 0, 0]
        elif kind == 'abort':
            self.mosi = [0x3F]
        else:
            self.mosi = [unlisted] + [rng.getrandbits(8) for _ in range(rng.randint(2, 6))]
            self.want = {k: 0 for k in range(1, len(self.mosi))}
        if kind in FIXED and rng.random() < .2:
            self.mosi += [rng.getrandbits(8) for _ in range(rng.randint(1, 8))]
        self.cut = None
        if rng.random() < .25:
            total = 8 * len(self.mosi)
            k = rng.randrange(1, len(self.mosi) + 1)
            self.cut = min(total - 1, max(1, rng.choice((8 * k - 1, 8 * k, 8 * k + 1, rng.randrange(1, total)))))

    @classmethod
    def exact(cls, kind, mosi, a=0, b=0):
        """A command the test chooses, sent whole, with nothing after it."""
        tx = cls.__new__(cls)
        tx.kind, tx.mosi, tx.a, tx.b = kind, list(mosi), a, b
        tx.want, tx.dead, tx.reads, tx.rdreg, tx.cut = {}, False, 0, None, None
        return tx

    def on_byte(self, k, sim, pre, lock):
        kind = self.kind
        if kind == 'wrmem' and k >= 4 and k % 2 == 0 and not self.dead:
            if all(t.RUN for t in sim.th):
                self.dead = True
                lock.busy_at = sim.c
            else:
                i = (k - 4) // 2
                lock.mem.append(['w', sim.c, (self.addr + i) % DEPTH, self.data[i]])
        elif kind == 'rdmem' and not self.dead and (k == 2 or (k >= 5 and k % 2 == 1 and (k - 3) // 2 < self.count)):
            if all(t.RUN for t in sim.th):
                self.dead = True
                lock.busy_at = sim.c
            else:
                lock.mem.append(['r', sim.c, (self.addr + self.reads) % DEPTH, (self, 5 + 2 * self.reads)])
                self.reads += 1
        elif kind == 'run' and k == 2:
            sim.host_run(self.a, self.b)
        elif kind == 'setpc' and k == 3:
            sim.host_setpc(self.a, self.b)
        elif kind == 'status':
            if k <= 3:
                self.want[k + 1] = pre[k]
            elif k == 4:
                sim.BUSY = 0
        elif kind == 'step' and k == 1:
            sim.host_step(self.a)
        elif kind == 'rdreg' and k == 2:
            if sim.th[self.a & 3].RUN:
                sim.BUSY = 1
                self.want[4] = self.want[5] = 0
            else:
                self.rdreg = []
                lock.rdreg = (self, sim.c + 1)
        elif kind == 'post' and k == 2:
            self.want[1] = self.want[2] = 0
            self.want[3] = sim.host_post(self.a, self.b)
        elif kind == 'take' and k == 0:
            full, v = sim.host_take(self.a)
            self.want = {1: full, 2: v >> 8, 3: v & 255}
        elif kind == 'abort' and k == 0:
            sim.host_abort()


def link_levels(mosi, cut, lo, hi, setup, last, hold):
    """Per cycle (cs, sck, mosi, byte finished at this rise, is a rise) of one transaction, CS low throughout: SCK
    low for `setup` cycles before the first rise and `lo` before each later one, high for `hi` and for `last` on the
    last bit, then low for `hold` cycles. MOSI changes only while SCK is low, as mode 0 has it."""
    total = 8 * len(mosi) if cut is None else cut
    out = []
    n = 0
    for k, byte in enumerate(mosi):
        for i in range(7, -1, -1):
            if n == total:
                break
            b = (byte >> i) & 1
            out += [(0, 0, b, None, False)] * (setup if n == 0 else lo)
            high = last if n == total - 1 else hi
            out += [(0, 1, b, k if i == 0 and j == 0 else None, j == 0) for j in range(high)]
            n += 1
    out += [(0, 0, 0, None, False)] * hold
    return out


class Lock:
    """The simulator, stepped one cycle per clock beside the RTL, plus the host timing it leaves out: a memory access
    lands in the first free memory cycle after h (9.4), BUSY from a memory command sets in h + 1, and RDREG returns
    the flops at the start of a cycle from h + 2 to h + 5 (9.2)."""

    def __init__(self, sim):
        self.sim = sim
        self.mem = []                   # [kind, h + 1, addr, data or (txn, reply byte)]
        self.busy_at = None
        self.rdreg = None

    def cycle(self, ui, uio):
        sim = self.sim
        th = sim.th[(sim.c - 1) & 3]
        free = not th.RUN and not th.MEM
        sim.cycle(ui, uio)
        if self.busy_at is not None and sim.c - 1 >= self.busy_at:
            sim.BUSY = 1
            self.busy_at = None
        if self.mem and free and sim.c - 1 >= self.mem[0][1]:
            kind, _, addr, x = self.mem.pop(0)
            if kind == 'w':
                sim.mem[addr] = x
            else:
                txn, k = x
                v = sim.mem[addr]
                txn.want[k], txn.want[k + 1] = v >> 8, v & 255
        if self.rdreg is not None:
            txn, first = self.rdreg
            if sim.c >= first:
                txn.rdreg.append(sim.rdreg(sim.th[txn.a & 3], txn.b))
            if len(txn.rdreg) == 4:
                self.rdreg = None


@cocotb.test(skip=os.environ.get("GATES") == "yes" and os.environ.get("LOCK_GATES") != "yes")
async def test_matches_simulator(dut):
    """Random programs on all four threads against the published simulator in sim/, with host traffic over the
    real link at the limits of section 9.1: SCK low or high for 2 cycles at a period of 6, CS falling 1 cycle before
    the first rising SCK and rising 1 cycle after the last falling one, CS high for 3 cycles between transactions,
    cut commands, bytes past a reply and every unlisted code. Every output pin but MISO is compared on every cycle,
    and every MISO byte with what its command returns. MISO may move only 3 cycles after a rising SCK or after CS
    rises.

    Each thread starts on a block that claims its pins, a lane with BYP and any other pin with PP, so the lanes show
    their inputs until a program moves them. After each transaction the host acts on what it finds. A thread that
    stopped itself resumes: after a halt, or after a WRMEM has put a legal word in place of the illegal one that
    stopped it. A thread the host stopped stays stopped while 3 others run, so that the memory commands, and half of
    the SETPC, STEP and RDREG commands, which name a stopped thread when there is one, find one. A running thread that
    has retired nothing since the last look is stopped, the word it sat on is rewritten, and it starts at a fresh PC.
    Which threads those are comes from the simulator, which runs in lockstep, so the stimulus stays strong whatever
    the seed. How often the other inputs change is drawn for each transaction. LOCK_SEED picks the seed (default 2).
    At gate level the test runs only with LOCK_GATES=yes, and checks the timebase on the RTL core that runs beside
    the netlist, since the netlist has no flop of that name."""
    sys.path.insert(0, SIM_DIR)
    sys.dont_write_bytecode = True      # leave no cache in sim/
    import sim as simulator
    from asm import ROWS, decode, encode, legal_value

    rare = {'halt': .85, 'st': .85, 'stp': .85, 'wrd': .85, 'recv': .7, 'send': .7, 'until': .8, 'untilm': .9,
            'txw': .9, 'rxw': .95}
    # The rows the simulator runs, in table order: all of them, the shift units' rows of section 5.9 included. A
    # simulator that lacks some rows lists them in PENDING, and the images leave those out.
    pending = set(getattr(simulator, 'PENDING', ()))
    rows = [r for r in ROWS if r.syntax not in pending]

    def noise(rng):
        """A random word, which is mostly illegal. One that is now a row the simulator can't run is still illegal."""
        w = rng.getrandbits(16)
        d = decode(w)
        return 0xDF80 if d is not None and d[0].syntax in pending else w

    def random_word(rng):
        """A legal word that rarely stops or blocks its thread for long, so threads keep running and moving pins."""
        while True:
            r = rng.choice(rows)
            f = {k: rng.getrandbits(len(b)) for k, b in r.bits.items()}
            if not all(legal_value(f[k], r.ftypes[k]) for k in f):
                continue
            if rng.random() < rare.get(r.mn, 0) or (r.mn == 'tick' and f['i'] > 64 and rng.random() < .95):
                continue
            if r.syntax == 'edge p, v, Rr' and rng.random() < .9:     # a step from a register can be 65,535 cycles
                continue
            return encode(r, f)

    def fresh_pc(sim, rng):
        """A PC in the random part of the image whose next 8 words are legal and neither stop nor block a thread."""
        for _ in range(64):
            pc = rng.randrange(64, DEPTH)
            words = [decode(sim.mem[a % DEPTH]) for a in range(pc, pc + 8)]
            if all(d is not None and d[0].mn not in STOPPERS for d in words):
                break
        return pc

    def rewrite(a, rng):
        """A WRMEM of a fresh legal word at address a."""
        w = random_word(rng)
        tx = Txn.exact('wrmem', [0x01, a >> 8, a & 255, w >> 8, w & 255])
        tx.addr, tx.data = a, [w]
        return tx

    def revive(sim, seen, rng):
        """The host's commands after a transaction. A thread that stopped itself resumes with RUN: after a halt, or
        after a WRMEM has put a legal word in place of the illegal one that stopped it. A thread the host stopped
        stays stopped, for the host's own commands to find, while 3 others run, and resumes with RUN when fewer do.
        A running thread that has retired nothing since seen is stopped, the word it sat on is rewritten, and it
        starts again at a fresh PC."""
        out, start = [], 0
        for t in sim.th:
            if not t.RUN:
                prev = decode(sim.mem[(t.PC - 1) % DEPTH])
                if t.ILL:
                    out.append(rewrite(t.PC, rng))
                    start |= 1 << t.TID
                elif prev is not None and prev[0].mn == 'halt':
                    start |= 1 << t.TID
        for t in sim.th:
            if sum(x.RUN for x in sim.th) + bin(start).count('1') >= 3:
                break
            if not t.RUN and not start >> t.TID & 1:
                start |= 1 << t.TID
        stuck = [t.TID for t in sim.th if t.RUN and t.retired == seen[t.TID]]
        if stuck:
            busy = sum(1 << t for t in stuck)
            out.insert(0, Txn.exact('run', [0x10, busy, 0], busy, 0))
            for t in stuck:
                out.append(rewrite(sim.th[t].PC, rng))
                pc = fresh_pc(sim, rng)
                out.append(Txn.exact('setpc', [0x11, t, pc >> 8, pc & 255], t, pc))
            start |= busy
        return out + [Txn.exact('run', [0x10, 0, start], 0, start)] if start else out

    host = await start(dut)
    seed = int(os.environ.get('LOCK_SEED', '2'))
    rng = random.Random(seed)
    # The timebase flop: the netlist has none by that name, so at gate level take the RTL core's beside it.
    core = dut.rtl_core if os.environ.get("GATES") == "yes" else dut.user_project.core
    unlisted = [c for c in range(256) if c not in LISTED]
    rng.shuffle(unlisted)
    kinds = ('caps', 'wrmem', 'rdmem', 'run', 'run', 'run', 'setpc', 'status', 'status', 'step', 'rdreg', 'rdreg', 'rdreg',
             'post', 'post', 'take', 'take', 'abort')
    count = restarted = 0
    toggles = running = retired = cycles = driven = 0
    last_pads = None
    for image in range(LOCK_IMAGES):
        await FallingEdge(dut.clk)
        dut.rst_n.value = 0                             # CS high and SCK low, as 9.1 asks while rst_n is low
        await ClockCycles(dut.clk, 4, rising=False)
        dut.rst_n.value = 1                             # between two rising edges
        released = get_sim_time(unit="ns")
        await ClockCycles(dut.clk, 8, rising=False)
        words = [random_word(rng) if rng.random() < .99 else noise(rng) for _ in range(DEPTH)]
        for t, claim in enumerate(CLAIMS):              # each thread starts by claiming pins, so its events show,
            block = [(0x90F8 if p in LANE_PINS else 0x90E8) | p << 8 for p in claim]   # and lanes follow their inputs
            block += [0x0100, 0x1000 | rng.randrange(64, DEPTH)]                        # pcfg ... / now / goto
            words[16 + 8 * t:16 + 8 * t + len(block)] = block
        await host.wrmem(0, words)
        await FallingEdge(dut.clk)
        host.other = 0
        host.drive(1, 0, 0)
        dut.uio_in.value = 0
        await host.wait(3)
        sim = simulator.Chip(DEPTH, image)
        sim.mem = list(words)
        sim.SYNC1 = sim.SYNC2 = 1
        # 9.1: cycle 0 starts 2 clock edges after rst_n rises, so TB is the rising edges since then, less 2.
        tb = round((get_sim_time(unit="ns") - released) / 20) - 2
        assert int(core.tb_q.value) == tb & 0xFFFF, "cycle 0 does not start 2 edges after rst_n rises"
        sim.c = tb
        lock = Lock(sim)
        ui_hi = io = 0
        last_miso, prev_cs = 0, 1
        rises, cs_rises = set(), set()
        c = 0
        queue = [Txn.exact('setpc', [0x11, t, 0, 16 + 8 * t], t, 16 + 8 * t) for t in range(THREADS)]
        queue.append(Txn.exact('run', [0x10, 0, 15], 0, 15))
        seen = [0] * THREADS                            # what each thread had retired at the last look
        n = looked = 0
        while n < LOCK_TRANSACTIONS:
            if not queue and n != looked:                   # after each random transaction
                looked = n
                queue = revive(sim, seen, rng)
                seen = [t.retired for t in sim.th]
            if queue:
                tx = queue.pop(0)
                if tx.kind == 'run' and n:
                    restarted += bin(tx.b).count('1')
            else:
                kind = 'other' if n % 12 == 11 else rng.choice(kinds)
                tx = Txn(rng, kind, unlisted[count % len(unlisted)], random_word)
                count += kind == 'other'
                n += 1
                if kind in ('setpc', 'step', 'rdreg') and rng.random() < .5:
                    idle = [t.TID for t in sim.th if not t.RUN]
                    if idle:                                # half of these name a thread that is stopped, if any
                        tx.a = (tx.a & 0xFC) | rng.choice(idle)
                        tx.mosi[1] = tx.a
            hold = rng.choice((1, 3, 20, 200))              # cycles between changes of the other inputs, on average
            lo, hi = rng.choice(DUTY)
            last = rng.choice(LAST) if rng.random() < .3 else hi
            hold_cs = rng.choice(HOLD) if rng.random() < .3 else lo
            levels = [(1, 0, 0, None, False)] * rng.choice(GAP)
            levels += link_levels(tx.mosi, tx.cut, lo, hi, rng.choice(SETUP), last, hold_cs)
            if n == LOCK_TRANSACTIONS:
                levels += [(1, 0, 0, None, False)] * 3      # the image's last: CS high before the next reset
            marks, sampled = [], []
            for cs, sck, mosi, finished, rise in levels:
                if hold == 1 or rng.randrange(hold) == 0:
                    ui_hi, io = rng.getrandbits(8), rng.getrandbits(8)
                ui = (ui_hi & 0xF8) | cs | sck << 1 | mosi << 2
                dut.ui_in.value = ui
                dut.uio_in.value = io
                await Timer(1, unit="ns")
                got = (int(dut.uo_out.value), int(dut.uio_out.value), int(dut.uio_oe.value))
                want = sim.pads(ui)
                if last_pads is not None and (got[0] & ~8, got[1], got[2]) != last_pads:
                    toggles += 1
                    change = ((got[0] & ~8) ^ last_pads[0]) | ((got[1] ^ last_pads[1]) | (got[2] ^ last_pads[2])) << 8
                    driven += bool(change & ~sum(1 << p for p in LANE_PINS if sim.BYP[p]))
                last_pads = (got[0] & ~8, got[1], got[2])
                running += any(th.RUN for th in sim.th)
                assert (got[0] & ~8, got[1], got[2]) == (want[0] & ~8, want[1], want[2]), \
                    f"image {image} transaction {n} ({tx.kind}): pads {got}, simulator {want}"
                miso = (got[0] >> MISO) & 1
                if miso != last_miso:
                    assert c - 3 in rises or c - 3 in cs_rises, f"image {image} transaction {n}: MISO moved in cycle {c}"
                last_miso = miso
                if rise:
                    rises.add(c)
                    sampled.append(miso)
                if cs and not prev_cs:
                    cs_rises.add(c)
                prev_cs = cs
                if finished is not None:
                    marks.append((c + 2, finished))
                here = [k for h, k in marks if h == c]
                pre = status_bytes(sim) if here else None
                lock.cycle(ui, io)
                for k in here:
                    tx.on_byte(k, sim, pre, lock)
                c += 1
                await FallingEdge(dut.clk)
            got = [sum(b << (7 - i) for i, b in enumerate(sampled[j:j + 8])) for j in range(0, len(sampled) - 7, 8)]
            if tx.rdreg is not None and len(got) > 4:
                fits = [v for v in tx.rdreg if v >> 8 == got[4] and (len(got) < 6 or v & 255 == got[5])]
                assert fits, f"image {image} transaction {n}: RDREG {tx.mosi} read {got[4:6]}, simulator {tx.rdreg}"
                tx.want[4], tx.want[5] = fits[0] >> 8, fits[0] & 255
            for k in range(len(got)):
                assert got[k] == tx.want.get(k, 0), \
                    f"image {image} transaction {n} ({tx.kind} {tx.mosi}, cut {tx.cut}): byte {k} is {got[k]:#x}, want {tx.want.get(k, 0):#x}"
            rises = {x for x in rises if x > c - 16}
            cs_rises = {x for x in cs_rises if x > c - 16}
        retired += sum(th.retired for th in sim.th)
        cycles += c
    dut._log.info(f"seed {seed}: {cycles} cycles, {retired} instructions retired, {toggles} output changes "
                  f"({driven} of them not only on a lane in bypass), {running} cycles with a thread running, "
                  f"{restarted} threads restarted")
    assert count >= 40, "too few unlisted codes"
    assert retired >= cycles // 8 and toggles >= cycles // 100, "stimulus too weak"


@cocotb.test(skip=os.environ.get("GATES") != "yes")
async def test_netlist_matches_rtl(dut):
    """Firmware runs on all four threads over the real link, with random pins and a reset between
    rounds. Every output pin must match the RTL on every cycle."""
    host = await start(dut)
    dut.compare.value = 1
    rng = random.Random(1)
    image = (SCATTER * (DEPTH // len(SCATTER) + 1))[:DEPTH]
    image[:len(UART_TX)] = UART_TX
    image[16:16 + len(WALK)] = WALK
    image[64:64 + len(LANES)] = LANES
    await host.wrmem(0, image)
    toggles = [0]

    async def inputs():
        last = 0
        while True:
            await FallingEdge(dut.clk)
            host.other = rng.getrandbits(8)
            dut.ui_in.value = (host.other & 0xF8) | host.link
            dut.uio_in.value = rng.getrandbits(8)
            now = int(dut.uo_out.value) & ~(1 << MISO)
            toggles[0] += now != last
            last = now

    pins = cocotb.start_soon(inputs())
    for _ in range(ROUNDS):
        await host.setpc(0, 0)
        await host.setpc(1, 16)
        await host.run(0, 3)
        await host.post(7, 16)
        for k in range(40):
            await host.setpc(2, rng.randrange(128, DEPTH))
            await host.setpc(3, 64 if k % 2 else rng.randrange(128, DEPTH))
            await host.run(0, 0xF)
            for _ in range(rng.randrange(3)):
                await host.post(rng.randrange(8), rng.getrandbits(16))
                await host.take(rng.randrange(8))
            await host.post(0, rng.getrandbits(8))
            await host.wait(300)
            await host.status()
            await host.abort()
            await host.wait(16)
            for _ in range(3):                  # what the threads did shows on MISO
                await host.rdreg(rng.randrange(4), rng.randrange(12))
        await host.rdmem(rng.randrange(DEPTH), 32)
        dut.rst_n.value = 0
        await ClockCycles(dut.clk, 4)
        dut.rst_n.value = 1
        await ClockCycles(dut.clk, 8)
    pins.cancel()
    assert int(dut.mismatches.value) == 0, f"{int(dut.mismatches.value)} cycles differ"
    assert toggles[0] >= MIN_TOGGLES, f"only {toggles[0]} output changes: stimulus too weak"
