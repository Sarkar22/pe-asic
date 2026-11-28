# SPDX-License-Identifier: Apache-2.0
"""The chip on Tiny Tapeout's demo board v3 (RP2350B), from MicroPython with Tiny Tapeout's SDK.

At the board's REPL, after copying pehost.py, bringup.py, bringup.img and this file:

    import board
    host = board.setup(project)     # the project's name in the SDK's shuttle index, or its mux address
    board.memory_test(host)         # or just the memory test
    board.bringup(host)             # the whole first power-on test

setup() does, in this order, each step printed as it goes:
  1. selects the project through the SDK, with every ui, uio and uo pin left an input;
  2. holds rst_n low;
  3. checks that the DIP switches of ui0..ui2 are off, since the link drives those lines;
  4. sets the system and peripheral clocks to 100 MHz and makes the 50 MHz clock on GPIO16 with the PWM: one
     system cycle high and one low, 10 ns each;
  5. sets CS (GPIO17) high as a plain output, and SPI0 to mode 0 at 5 MHz on SCK GPIO18, MOSI GPIO19, MISO GPIO36;
  6. after 2 ms of reset with the clock running, releases rst_n to the board's reset circuit and waits 20 ms;
  7. reads CAPS.
It drives only rst_n, the clock and the three link lines. Don't change the SDK's mode afterwards: that makes
the clock pin an input again.
"""
import time

import machine
from machine import Pin

import pehost

# GPIO numbers on the demo board v3 (the board's pin table; the SDK's GPIOMapTTDBv3)
RST_N = 14
CLK = 16
CS = 17                 # ui0
SCK = 18                # ui1
MOSI = 19               # ui2
MISO = 36               # uo3
UIO = tuple(range(25, 33))
UO = tuple(range(33, 41))

# RP2350 registers (datasheet sections 12.3.5, 12.5.3, 8.1.7, 3.1.11 and 9.11.3)
SSPSR = 0x4008000C      # SPI0 status: BSY is bit 4
PWM_CH0_DIV = 0x400A8004
PWM_CH0_CC = 0x400A800C
PWM_CH0_TOP = 0x400A8010
CLK_PERI_CTRL = 0x40010048
GPIO_IN = 0xD0000004
GPIO_HI_IN = 0xD0000008
PADS_BANK0 = 0x40038000

SYS_HZ = 100000000      # both clocks; the PWM then makes 50 MHz with TOP 1, CC 1
CHIP_HZ = 50000000
SCK_HZ = 5000000        # 100 MHz / 20: each SCK phase is 5 chip cycles


class Link:
    """SPI0 as ISA section 9.1 asks: mode 0, MSB first, CS from a plain GPIO. One transaction per xfer()."""

    def __init__(self, baudrate=SCK_HZ):
        self.cs = Pin(CS, Pin.OUT, value=1)     # high before the pin starts to drive
        # miso must be named: left out, MicroPython takes the board's default SPI0 RX pin, GPIO16, the clock.
        self.spi = machine.SPI(0, baudrate=baudrate, polarity=0, phase=0, bits=8, firstbit=machine.SPI.MSB,
                               sck=Pin(SCK), mosi=Pin(MOSI), miso=Pin(MISO))

    def xfer(self, out):
        buf = bytearray(len(out))
        n = 0
        self.cs(0)
        try:
            self.spi.write_readinto(out, buf)
        finally:
            # write_readinto returns once the last byte is in, which can be before SCK falls; CS must not move
            # while SCK is high (section 10), so wait until the SPI block is idle. CS rises even after an error,
            # so the next transaction starts with a falling CS.
            while machine.mem32[SSPSR] & 0x10 and n <= 10000:
                n += 1
            self.cs(1)
            time.sleep_us(1)                    # CS high for at least 3 chip cycles between transactions
        if n > 10000:
            raise pehost.LinkError('SPI0 stays busy')
        return bytes(buf)

    def pause_us(self, us):
        time.sleep_us(us)


def probe():
    """(uo, uio) as the RP2350's own inputs read the chip's pads. uo3 is MISO."""
    lo = machine.mem32[GPIO_IN]
    hi = machine.mem32[GPIO_HI_IN]
    return hi >> 1 & 0xFF, (lo >> 25 & 0x7F) | (hi & 1) << 7


def release(gpio):
    """Leave a GPIO an input with its input buffer and both pulls off, for a net an outside part drives. Pin()
    always turns the input buffer on, so this writes the pad register (IE bit 6, PUE bit 3, PDE bit 2)."""
    Pin(gpio, Pin.IN)
    a = PADS_BANK0 + 4 + 4 * gpio
    machine.mem32[a] = machine.mem32[a] & ~(1 << 6 | 1 << 3 | 1 << 2)


def select(project, log=print):
    """Select the project on the chip's mux through the SDK, leaving every project pin an input."""
    from ttboard.demoboard import DemoBoard
    from ttboard.mode import RPMode
    tt = DemoBoard.get()
    tt.mode = RPMode.ASIC_MANUAL_INPUTS        # also takes the clock pin from any clock the SDK was making
    tt.apply_configs = False                    # the SDK's per-project settings would set clocks and reset
    if isinstance(project, int):
        tt.shuttle.reset_and_clock_mux(project)
        log('project: mux address %d selected' % project)
    elif tt.shuttle.has(project):
        if not tt.shuttle.get(project).enable():    # False when the SDK refuses it, and says why in its log
            raise ValueError('the SDK did not enable %r' % (project,))
        log('project: %s selected' % project)
    else:
        raise ValueError('the SDK does not know %r: give its mux address instead' % (project,))
    return tt


def check_switches(log=print):
    """ui0..ui2 must read low with the pull-down: each ui DIP switch ties its line to 3.3 V through 1 kOhm.
    They are driven low first for 10 us, so a pad left near 2.2 V by erratum RP2350-E9 reads low too."""
    lines = (CS, SCK, MOSI)
    for g in lines:
        Pin(g, Pin.OUT, value=0)
    time.sleep_us(10)
    for g in lines:
        Pin(g, Pin.IN, Pin.PULL_DOWN)
    time.sleep_us(10)
    on = [i for i in range(3) if Pin(lines[i]).value()]
    if on:
        raise pehost.LinkError('ui%s read high: turn their DIP switches off' % ', ui'.join(str(i) for i in on))
    log('switches: ui0..ui2 off')


def start_clock(log=print):
    """50 MHz on GPIO16 with the PWM, from a 100 MHz system clock: one system cycle high and one low. The SPI
    block's clock moves to the same 100 MHz. Changing the system clock restarts its PLL, so this runs while
    the chip is held in reset."""
    machine.freq(SYS_HZ, SYS_HZ)
    pwm = machine.PWM(Pin(CLK), freq=CHIP_HZ, duty_u16=32768)
    top = machine.mem32[PWM_CH0_TOP] & 0xFFFF
    cc = machine.mem32[PWM_CH0_CC] & 0xFFFF
    div = machine.mem32[PWM_CH0_DIV] & 0xFFF
    src = machine.mem32[CLK_PERI_CTRL] >> 5 & 7
    sys_hz = machine.freq()
    ok = sys_hz == SYS_HZ and top == 1 and cc == 1 and div == 0x10 and src == 1
    log('clock: %d Hz system clock, PWM TOP %d, CC %d, DIV %d.%d: %.3f MHz, %.1f ns high, %.1f ns low; '
        'SPI clock from %s' % (sys_hz, top, cc, div >> 4, div & 15, sys_hz / (top + 1) / (div / 16) / 1e6,
                               cc * 1e9 / sys_hz, (top + 1 - cc) * 1e9 / sys_hz,
                               'pll_sys' if src == 1 else 'source %d' % src))
    if not ok:
        raise pehost.LinkError('the clock is not 50 MHz at 50% duty: see the line above')
    return pwm


def setup(project, release_pins=(), log=print):
    """Select, clock and reset the chip, and return a pehost.Host on the link after checking CAPS. release_pins:
    GPIOs to leave fully undriven and unread (see release())."""
    select(project, log)
    rst = Pin(RST_N, Pin.OUT, value=0)
    log('reset: rst_n low')
    check_switches(log)
    start_clock(log)
    link = Link()
    log('link: CS high on GPIO17; SPI0 mode 0, MSB first, %d Hz' % SCK_HZ)
    for g in release_pins:
        release(g)
    time.sleep_ms(2)                            # reset held with the clock running, CS high and SCK low
    rst.init(Pin.IN)                            # released to the board's reset circuit, as the SDK does
    time.sleep_ms(20)                           # the reset line's 1.1 ms RC on board revision 3.2, many times over
    log('reset: released, waited 20 ms')
    host = pehost.Host(link)
    v, n, d = host.check_caps()
    log('CAPS: version %d, %d threads, %d words' % (v, n, 1 << d))
    return host


def memory_test(host, log=print):
    """The bring-up's memory test alone: all 512 words written and read back with two patterns. It overwrites any
    program, and needs a stopped thread (section 9.4)."""
    import bringup as b
    rep = b.Report(log)
    b.memory_test(host, rep)
    return rep


def bringup(host, image='bringup.img', pins=True, log=print):
    """The first power-on test (bringup.py), with the board reading the chip's pads too."""
    import bringup as b
    with open(image) as fh:
        img = pehost.read_image(fh)
    return b.run(host, img, probe=probe, log=log, pins=pins)
