# SPDX-License-Identifier: Apache-2.0

import os
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge

CLKS_PER_BIT = 434
TX_BIT = 4
BUSY_BIT = 0
LOCKSTEP_CYCLES = 200_000
MIN_LOCKSTEP_FRAMES = 30


def tx(dut):
    return (int(dut.uo_out.value) >> TX_BIT) & 1


def busy(dut):
    return (int(dut.uo_out.value) >> BUSY_BIT) & 1


async def reset(dut):
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


async def send(dut, byte):
    dut.uio_in.value = byte
    dut.ui_in.value = 1
    await ClockCycles(dut.clk, 6)
    dut.ui_in.value = 0


async def wait_for_start_bit(dut, timeout=20):
    for _ in range(timeout):
        await FallingEdge(dut.clk)
        if tx(dut) == 0:
            return
    assert False, "no start bit"


async def receive(dut):
    """Decode one 8N1 frame by sampling mid-bit, independently of the RTL's counters."""
    await wait_for_start_bit(dut)
    await ClockCycles(dut.clk, CLKS_PER_BIT // 2)
    assert tx(dut) == 0, "start bit did not last half a bit"
    byte = 0
    for i in range(8):
        await ClockCycles(dut.clk, CLKS_PER_BIT)
        byte |= tx(dut) << i
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    assert tx(dut) == 1, "bad stop bit"
    # A short stop bit still reads high, so check that busy lasts to the end of it.
    for _ in range(CLKS_PER_BIT // 2 - 4):
        assert busy(dut) == 1, "stop bit too short"
        await RisingEdge(dut.clk)
    return byte


@cocotb.test()
async def test_idle_after_reset(dut):
    await reset(dut)
    assert tx(dut) == 1, "line must idle high"
    assert busy(dut) == 0
    assert int(dut.uio_oe.value) == 0, "uio pins must stay inputs"


@cocotb.test()
async def test_bytes(dut):
    await reset(dut)
    for byte in (0x55, 0xAA, 0x00, 0xFF, 0x4A, 0x53):
        await send(dut, byte)
        assert await receive(dut) == byte
        await RisingEdge(dut.clk)
        for _ in range(CLKS_PER_BIT):
            if busy(dut) == 0:
                break
            await RisingEdge(dut.clk)
        assert busy(dut) == 0, "still busy after the stop bit"


@cocotb.test()
async def test_send_while_busy_is_ignored(dut):
    await reset(dut)
    await send(dut, 0xA5)
    receiver = cocotb.start_soon(receive(dut))
    await ClockCycles(dut.clk, 20 * CLKS_PER_BIT // 10)
    await send(dut, 0x3C)
    assert await receiver == 0xA5
    await ClockCycles(dut.clk, 3 * CLKS_PER_BIT)
    assert tx(dut) == 1, "second send must not start a frame"


@cocotb.test(skip=os.environ.get("GATES") != "yes")
async def test_netlist_matches_rtl(dut):
    """Random inputs and resets: every output pin must match the RTL on every cycle."""
    await reset(dut)
    dut.compare.value = 1
    rng = random.Random(1)
    frames = 0
    was_busy = False
    reset_left = 0
    for _ in range(LOCKSTEP_CYCLES):
        await FallingEdge(dut.clk)
        dut.ui_in.value = rng.getrandbits(8)
        dut.uio_in.value = rng.getrandbits(8)
        if reset_left > 0:
            reset_left -= 1
        elif rng.randrange(20_000) == 0:
            reset_left = rng.randint(1, 6)
        dut.rst_n.value = 0 if reset_left > 0 else 1
        is_busy = busy(dut) == 1
        frames += is_busy and not was_busy
        was_busy = is_busy
    assert int(dut.mismatches.value) == 0, f"{int(dut.mismatches.value)} cycles differ"
    assert frames >= MIN_LOCKSTEP_FRAMES, f"only {frames} frames: stimulus too weak"
