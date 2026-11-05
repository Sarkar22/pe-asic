# SPDX-License-Identifier: Apache-2.0
"""Tests at the pins, from reset, over the real SPI host link."""

import os
import random
import sys

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
    assert changes[-1][0] == 1, "bit 0 of the version is 1"
    assert changes[-1][1] == [1, 1, 0], f"MISO moved at the wrong cycle: {changes[-1]}"
    host.drive(1, 0, 0)
    await host.wait(4)


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
    assert await host.rdreg(2, 12) == 0, "unknown reg8 reads 0"
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

    # Reset with CS low and SCK high, partway into a byte. CS stays low through the reset and after it.
    await FallingEdge(dut.clk)
    host.drive(0, 0, 0)
    await host.wait(HALF)
    for _ in range(3):
        host.drive(0, 0, 1)
        await host.wait(HALF)
        host.drive(0, 1, 1)
        await host.wait(HALF)
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 3, rising=False)
    assert int(dut.uo_out.value) == 0 and int(dut.uio_oe.value) == 0, "outputs clear in reset"
    host.drive(0, 0, 0)
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


# ---- test_matches_simulator: the RTL beside the published simulator ------------------------------------

DUTY = ((3, 3), (3, 3), (3, 3), (4, 4), (2, 4), (4, 2), (1, 5), (5, 1))   # SCK low, high cycles: period 6 or more
CLAIMS = ((0, 1, 8, 9), (2, 4, 10, 11), (5, 6, 12, 13), (7, 14, 15))     # pins each thread claims with pcfg PP first
FIXED = ('caps', 'run', 'setpc', 'status', 'step', 'rdreg', 'post', 'take', 'abort')


def reg_value(t, reg):
    """What RDREG returns for a stopped thread, from the simulator's state (9.2)."""
    if reg < 8:
        return t.R[reg]
    return {8: t.D, 9: t.S, 10: t.PC, 11: t.T | t.LATE << 1 | t.STALE << 2 | t.WAITING << 3}.get(reg, 0)


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


def link_levels(mosi, lo, hi, cut, quick):
    """Per cycle (cs, sck, mosi, byte finished at this rise, is a rise), as in rtl_pins_check of the benches."""
    total = 8 * len(mosi) if cut is None else cut
    out = [(0, 0, 0, None, False)] * lo
    n = 0
    for k, byte in enumerate(mosi):
        for i in range(7, -1, -1):
            if n == total:
                break
            b = (byte >> i) & 1
            out += [(0, 0, b, None, False)] * lo
            high = quick[0] if quick and n == total - 1 else hi
            out += [(0, 1, b, k if i == 0 and j == 0 else None, j == 0) for j in range(high)]
            n += 1
    out += [(0, 0, 0, None, False)] * (quick[1] if quick else lo)
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
                txn.rdreg.append(reg_value(sim.th[txn.a & 3], txn.b))
            if len(txn.rdreg) == 4:
                self.rdreg = None


@cocotb.test(skip=os.environ.get("GATES") == "yes")
async def test_matches_simulator(dut):
    """Random programs on all four threads against the published simulator in sim/, with host traffic over the
    real link: SCK phases of 1 to 5 cycles, cut commands, bytes past a reply, every unlisted code and CS rising
    as SCK falls. Every output pin but MISO is compared on every cycle, and every MISO byte with what its command
    returns. MISO may move only 3 cycles after a rising SCK or after CS rises."""
    sys.path.insert(0, SIM_DIR)
    sys.dont_write_bytecode = True      # leave no cache in sim/
    import sim as simulator
    from asm import ROWS, encode, legal_value

    rare = {'halt': .85, 'st': .85, 'stp': .85, 'wrd': .85, 'recv': .7, 'send': .7, 'until': .8, 'untilm': .9}

    def random_word(rng):
        """A legal word that rarely stops or blocks its thread for long, so threads keep running and moving pins."""
        while True:
            r = rng.choice(ROWS)
            f = {k: rng.getrandbits(len(b)) for k, b in r.bits.items()}
            if not all(legal_value(f[k], r.ftypes[k]) for k in f):
                continue
            if rng.random() < rare.get(r.mn, 0) or (r.mn == 'tick' and f['i'] > 64 and rng.random() < .95):
                continue
            if r.syntax == 'edge p, v, Rr' and rng.random() < .9:     # a step from a register can be 65,535 cycles
                continue
            return encode(r, f)

    host = await start(dut)
    rng = random.Random(2)
    unlisted = [c for c in range(256) if c not in LISTED]
    rng.shuffle(unlisted)
    kinds = ('caps', 'wrmem', 'rdmem', 'run', 'run', 'run', 'setpc', 'status', 'status', 'step', 'rdreg', 'rdreg', 'rdreg',
             'post', 'post', 'take', 'take', 'abort')
    count = 0
    toggles = running = retired = cycles = 0
    last_pads = None
    for image in range(LOCK_IMAGES):
        await FallingEdge(dut.clk)
        dut.rst_n.value = 0
        await ClockCycles(dut.clk, 4, rising=False)
        dut.rst_n.value = 1                             # between two rising edges
        released = get_sim_time(unit="ns")
        await ClockCycles(dut.clk, 8, rising=False)
        words = [random_word(rng) if rng.random() < .99 else rng.getrandbits(16) for _ in range(DEPTH)]
        for t, claim in enumerate(CLAIMS):              # each thread starts by claiming pins, so its events show
            block = [0x90E8 | p << 8 for p in claim] + [0x0100, 0x1000 | rng.randrange(64, DEPTH)]  # pcfg PP / now / goto
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
        assert int(dut.user_project.core.tb_q.value) == tb & 0xFFFF, "cycle 0 does not start 2 edges after rst_n rises"
        sim.c = tb
        lock = Lock(sim)
        ui_hi = io = 0
        hold = rng.choice((1, 3, 20, 200))
        last_miso, prev_cs = 0, 1
        rises, cs_rises = set(), set()
        c = 0
        for n in range(LOCK_TRANSACTIONS):
            if n < 4:
                tx = Txn(rng, 'setpc', 0)
                tx.a, tx.b, tx.cut = n, 16 + 8 * n, None
                tx.mosi = [0x11, n, tx.b >> 8, tx.b & 255]
            elif n == 4:
                tx = Txn(rng, 'run', 0)
                tx.a, tx.b, tx.cut, tx.mosi = 0, 15, None, [0x10, 0, 15]
            else:
                kind = 'other' if n % 12 == 11 else rng.choice(kinds)
                tx = Txn(rng, kind, unlisted[count % len(unlisted)], random_word)
                count += kind == 'other'
            lo, hi = rng.choice(DUTY)
            quick = (rng.randint(1, hi), rng.choice((0, 1, 2))) if rng.random() < .3 else None
            levels = [(1, 0, 0, None, False)] * rng.randrange(4, 20) + link_levels(tx.mosi, lo, hi, tx.cut, quick)
            levels += [(1, 0, 0, None, False)] * 6
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
                toggles += last_pads is not None and (got[0] & ~8, got[1], got[2]) != last_pads
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
    dut._log.info(f"{cycles} cycles, {retired} instructions retired, {toggles} output changes, {running} cycles with a thread running")
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
