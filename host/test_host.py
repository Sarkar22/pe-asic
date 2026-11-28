#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""CPython tests of host/: the library's bytes and replies, the image file, the bring-up on the simulator, and the
board script against stand-ins for MicroPython's machine module and Tiny Tapeout's SDK.

    python3 host/test_host.py                         # every test
    python3 host/test_host.py --transcript out.txt    # also write the bring-up's transcript, for replay.py

The expected bytes in BYTES are written from the table in ISA section 9.2, not taken from pehost.py.
"""
import binascii
import io
import os
import sys
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.dont_write_bytecode = True
sys.path.insert(0, HERE)
import bringup      # noqa: E402
import mkimage      # noqa: E402
import pehost       # noqa: E402
import replay       # noqa: E402
import simlink      # noqa: E402

BRINGUP_IMG = os.path.join(HERE, 'bringup.img')


class Script:
    """A transport that answers from a list of replies and keeps what was sent."""

    def __init__(self, *replies):
        self.replies = [bytes.fromhex(r) for r in replies]
        self.sent = []
        self.paused = 0

    def xfer(self, out):
        self.sent.append(bytes(out).hex())
        r = self.replies.pop(0) if self.replies else bytes(len(out))
        assert len(r) == len(out), 'the script reply has the wrong length'
        return r

    def pause_us(self, us):
        self.paused += us


# (call, bytes it must send, reply, what it must return): section 9.2, one row per command
BYTES = [
    (lambda h: h.caps(), '000000', '000149', (1, 4, 9)),
    (lambda h: h.wrmem(0x1FF, [0xBEEF]), '0101ffbeef', '0000000000', None),
    (lambda h: h.wrmem(3, [1, 0x8000]), '0100030001' + '8000', '00' * 7, None),
    (lambda h: h.rdmem(0x100, 2), '020100' + '0000' + '00000000', '00' * 5 + '1234abcd', [0x1234, 0xABCD]),
    (lambda h: h.run(0x5, 0xA), '10050a', '000000', None),
    (lambda h: h.setpc(2, 0x123), '11020123', '00000000', None),
    (lambda h: h.step(3), '1303', '0000', None),
    (lambda h: h.rdreg(1, 'PC'), '14010a000000', '00000000beef', 0xBEEF),
    (lambda h: h.rdreg(3, 'CHKLO'), '14031e000000', '000000000102', 0x0102),
    (lambda h: h.rdreg(0, 7), '140007000000', '000000008001', 0x8001),
    (lambda h: h.post(5, 0xCAFE), '25cafe00', '00000000', True),
    (lambda h: h.post(0, 0x0102), '20010200', '00000001', False),
    (lambda h: h.take(6), '2e000000', '00011234', (True, 0x1234)),
    (lambda h: h.take(7), '2f000000', '00000000', (False, 0)),
    (lambda h: h.take(0), '28000000', '00010042', (True, 0x0042)),
    (lambda h: h.abort(), '3f', '00', None),
]


class Commands(unittest.TestCase):
    def test_bytes_and_replies(self):
        for call, sent, reply, want in BYTES:
            s = Script(reply)
            got = call(pehost.Host(s))
            self.assertEqual(s.sent, [sent], 'bytes sent')
            self.assertEqual(got, want, 'decoded reply for %s' % sent)

    def test_status_fields(self):
        s = pehost.Host(Script('00a53c8101')).status()
        self.assertEqual((s.run, s.ill, s.late, s.stale, s.full, s.busy), (0xA, 0x5, 0x3, 0xC, 0x81, 1))
        s = pehost.Host(Script('0000000003')).status()       # reserved bit 1 of byte 3 kept in raw, not in busy
        self.assertEqual((s.busy, s.raw), (1, bytes.fromhex('00000003')))
        self.assertEqual(repr(pehost.Host(Script('0010000000')).status()), 'run 1 ill 0 late 0 stale 0 full 00 busy 0')

    def test_flags(self):
        f = pehost.Flags(0b1011)
        self.assertEqual((f.t, f.late, f.stale, f.waiting), (1, 1, 0, 1))

    def test_caps_names_a_slip(self):
        h = pehost.Host(Script('000149'))
        self.assertEqual(h.check_caps(), (1, 4, 9))
        for reply, words in (('000292', 'hold'), ('0000a4', 'setup'), ('000024', 'setup'), ('000000', 'stays low'),
                             ('ffffff', 'stays high'), ('000249', 'not a reply'), ('000139', 'not a reply'),
                             ('000148', 'not a reply')):
            with self.assertRaises(pehost.CapsError) as e:
                pehost.Host(Script(reply)).check_caps()
            self.assertIn(words, str(e.exception))

    def test_replies_that_make_no_sense(self):
        for call, reply in ((lambda h: h.rdmem(0, 1), '00000000010000'),     # a byte before the data not 0
                            (lambda h: h.post(1, 2), '00000002'),              # result bits 7..1
                            (lambda h: h.post(1, 2), '00010000'),              # 0 for 2 bytes
                            (lambda h: h.take(1), '00001234'),                 # not full, but data
                            (lambda h: h.take(1), '00ff1234')):                # flag byte
            with self.assertRaises(pehost.LinkError):
                call(pehost.Host(Script(reply)))

    def test_arguments(self):
        h = pehost.Host(Script())
        for bad in (lambda: h.setpc(4, 0), lambda: h.setpc(0, 512), lambda: h.step(-1), lambda: h.rdreg(0, 32),
                    lambda: h.rdreg(0, 'R8'), lambda: h.post(8, 0), lambda: h.post(0, 0x10000), lambda: h.take(8),
                    lambda: h.wrmem(511, [1, 2]), lambda: h.wrmem(0, [0x10000]), lambda: h.rdmem(512, 1),
                    lambda: h.rdmem(0, 0), lambda: h.run(16, 0)):
            self.assertRaises(ValueError, bad)
        self.assertEqual(h.link.sent, [], 'a refused call sends nothing')

    def test_register_names(self):
        self.assertEqual([pehost.REGS[n] for n in ('R0', 'R7', 'D', 'S', 'PC', 'FLAGS', 'SSTAT', 'CRC_LO', 'CRC_HI',
                                                   'ENDF', 'CFG0', 'CFG15', 'TXMODE', 'CODING')],
                         [0, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 31, 16, 31])

    def test_runs(self):
        words = dict((a, a) for a in list(range(0, 70)) + [100, 101, 300, 400, 402])
        got = [(a, len(ws)) for a, ws in pehost.runs(words, 64)]
        self.assertEqual(got, [(0, 64), (64, 6), (100, 2), (300, 1), (400, 1), (402, 1)])
        for a, ws in pehost.runs(words, 64):
            self.assertEqual(ws, list(range(a, a + len(ws))), 'each piece starts at its own address')

    def test_polling(self):
        s = Script('00000000', '00000000', '00010005')
        self.assertEqual(pehost.Host(s).take_wait(2, tries=5, pause_us=7), 5)
        self.assertEqual(s.paused, 14)
        with self.assertRaises(pehost.Timeout):
            pehost.Host(Script()).take_wait(2, tries=3)
        s = Script('00000001', '00000000')
        pehost.Host(s).post_wait(1, 9, tries=3)
        self.assertEqual(len(s.sent), 2, 'POST again while the word is full')
        with self.assertRaises(pehost.Timeout):
            pehost.Host(Script('00000001', '00000001')).post_wait(1, 9, tries=2)


def sim_host(seed=1, **kw):
    link = simlink.SimLink(seed=seed, **kw)
    link.run(10)
    return pehost.Host(link), link


def program(src):
    return mkimage.asm.assemble(src, depth=512)


class OnTheSimulator(unittest.TestCase):
    """The library against sim/sim.py at the byte level (simlink.py)."""

    def test_memory(self):
        h, link = sim_host(2)
        img = pehost.Image(dict((a, (a * 7919) & 0xFFFF) for a in list(range(5, 205)) + [300, 301, 400, 402, 511]))
        gap = link.chip.mem[401]
        h.load(img)
        self.assertEqual(h.verify(img), [])
        self.assertEqual([link.chip.mem[a] for a in (5, 204, 300, 400, 402, 511)],
                         [img.words[a] for a in (5, 204, 300, 400, 402, 511)])
        self.assertEqual(link.chip.mem[401], gap, 'nothing written in the gap')
        self.assertEqual(h.rdmem(100, 3), [img.words[a] for a in (100, 101, 102)])
        link.chip.mem[150] ^= 1
        self.assertEqual(h.verify(img), [(150, img.words[150], img.words[150] ^ 1)])

    def test_busy(self):
        h, link = sim_host(3)
        h.load(pehost.Image({0: 0x1000}))                   # l: goto l
        h.start({0: 0, 1: 0, 2: 0, 3: 0})
        self.assertEqual(h.status().run, 0xF)
        with self.assertRaises(pehost.Busy):
            h.load(pehost.Image({10: 1}))
        with self.assertRaises(pehost.Busy):
            h.verify(pehost.Image({0: 0x1000}))
        with self.assertRaises(pehost.Busy):
            h.start({1: 0})
        self.assertEqual(h.rdreg(2, 'PC'), 0)               # a running thread reads 0, and BUSY sets
        self.assertEqual(h.status().busy, 1)
        self.assertEqual(h.status().busy, 0, 'reading STATUS cleared BUSY')
        h.stop(0x2)
        self.assertEqual(h.wait_stopped(0x2).run, 0xD)
        h.load(pehost.Image({10: 1}))                       # one thread stopped is enough (section 9.4)
        self.assertEqual(h.rdmem(10, 1), [1])

    def test_step_rdreg_setpc(self):
        h, link = sim_host(4)
        img = program('.org 40\nldi R1, 5\nldi R2, 6\nlui R2, 0xBE\nhalt\n')
        h.load_and_verify(img)
        h.setpc(2, 40)
        self.assertEqual(h.rdreg(2, 'PC'), 40)
        h.step(2)
        link.pause_us(1)
        self.assertEqual((h.rdreg(2, 'R1'), h.rdreg(2, 'PC')), (5, 41))
        h.step(2)
        link.pause_us(1)
        h.step(2)
        link.pause_us(1)
        self.assertEqual((h.rdreg(2, 2), h.rdreg(2, 'PC')), (0xBE06, 43))

    def test_mailbox(self):
        h, link = sim_host(5)
        self.assertTrue(h.post(3, 0x1234))
        self.assertFalse(h.post(3, 0x5678), 'a full word refuses')
        self.assertEqual(h.status().full, 0x08)
        self.assertEqual(h.take(3), (True, 0x1234))
        self.assertEqual(h.take(3), (False, 0))
        self.assertEqual(h.status().full, 0)

    def test_abort_releases_uio(self):
        h, link = sim_host(6)
        h.load(program('pcfg io0, PP\nput io0, 1\nnow\nl: goto l\n'))
        h.start({0: 0})
        link.pause_us(1)
        self.assertEqual(link.probe()[1] & 1, 1)
        h.abort()
        link.pause_us(1)
        self.assertEqual(link.probe()[1], 0)
        self.assertEqual(h.wait_stopped().run, 0)

    def test_bringup(self):
        h, link = sim_host(7)
        with open(BRINGUP_IMG) as fh:
            img = pehost.read_image(fh)
        rep = bringup.run(h, img, probe=link.probe, log=lambda s: None)
        self.assertEqual(rep.failed(), [])
        self.assertEqual(len(rep.rows), 24)

    def test_bringup_finds_faults(self):
        """The bring-up's own checks fail on a chip with a fault."""
        with open(BRINGUP_IMG) as fh:
            img = pehost.read_image(fh)
        quiet = lambda s: None                              # noqa: E731
        h, link = sim_host(8)
        wr = link.chip.host_wrmem
        link.chip.host_wrmem = lambda a, d: wr(a, d | 0x0400 if a == 300 else d)    # a stuck bit at 300
        names = [r[1] for r in bringup.run(h, img, probe=link.probe, log=quiet).failed()]
        self.assertEqual(names, ['memory, pattern 1'])

        h, link = sim_host(9)
        cyc = link.chip.cycle
        link.chip.cycle = lambda ui, uio, *a: cyc(ui, uio & ~0x20, *a)  # io5 never reads high
        names = [r[1] for r in bringup.run(h, img, probe=link.probe, log=quiet).failed()]
        self.assertEqual(names, ['uio pads read back by the chip while it drives them', 'thread 0 registers after halt'])

        h, link = sim_host(10)
        post = link.chip.host_post
        link.chip.host_post = lambda n, d: post(n ^ 1 if n == 6 else n, d)  # word 6 crossed with 7
        lines = []
        with self.assertRaises(pehost.Timeout):
            bringup.run(h, img, probe=link.probe, log=lines.append)
        self.assertIn('FAIL  mailbox word 6 round trip: nothing in mailbox word 6', lines[-2])

        h, link = sim_host(14)
        post = link.chip.host_post
        link.chip.host_post = lambda n, d: post(n, d | 0x0008 if n == 2 else d)  # bit 3 of word 2 stuck at 1
        names = [r[1] for r in bringup.run(h, img, probe=link.probe, log=quiet).failed()]
        self.assertEqual(names, ['mailbox word 2 round trip', 'thread 2 registers after halt'])

        h, link = sim_host(13)
        run = link.chip.host_run

        def one_by_one(stop, start):                        # a RUN that starts its threads in different cycles
            run(stop, 0)
            for t in range(4):
                if (start & ~stop) >> t & 1:
                    link.run(1)
                    run(0, 1 << t)
        link.chip.host_run = one_by_one
        names = [r[1] for r in bringup.run(h, img, probe=link.probe, log=quiet).failed()]
        self.assertIn('the four threads got one D from one RUN', names)


class Images(unittest.TestCase):
    def test_round_trip(self):
        img = program('start: pcfg uo0, PP\n.org 100\nb: put uo0, 1\nnow\n.org 511\n.data 0xFFFF\n')
        text = mkimage.image_text(img, 'x.s', {0: 'start', 2: '0x64'})
        got = pehost.read_image(text.splitlines())
        self.assertEqual((got.words, got.labels, got.starts), (img.words, img.labels, {0: 0, 2: 100}))
        self.assertEqual(pehost.crc16(b'123456789'), binascii.crc_hqx(b'123456789', 0xFFFF), 'CRC-16/CCITT-FALSE')
        self.assertEqual(pehost.crc16(b'123456789'), 0x29B1)

    def test_damage_is_refused(self):
        good = mkimage.image_text(program('ldi R1, 2\nhalt\n.org 300\nidle\n'), 'x.s').splitlines()
        pehost.read_image(good)
        cases = [good[:-1],                                             # cut short
                 [x.replace('0080', '0081') for x in good],             # a word changed
                 [good[0]] + good[2:3] + good[1:],                      # a line copied twice
                 ['pe-image 2'] + good[1:],
                 good + ['@0010 0000'],                                 # after the end line
                 [x.replace('@012C', '@0200') for x in good],           # past the memory
                 [x.replace('@0000', '@00G0') for x in good],
                 [x.replace(' 0080', ' 080') for x in good],
                 []]
        # files whose end line matches their words, but that name an address past the memory
        crc = pehost.image_crc
        cases += [['pe-image 1', '@01FF 0001 0002', 'end 2 %04X' % crc({0x1FF: 1, 0x200: 2})],
                  ['pe-image 1', '@0000 0001', 'label x 0200', 'end 1 %04X' % crc({0: 1})],
                  ['pe-image 1', '@0000 0001', 'start 0 0200', 'end 1 %04X' % crc({0: 1})]]
        for lines in cases:
            with self.assertRaises(pehost.ImageError):
                pehost.read_image(lines)

    def test_bringup_img_is_current(self):
        img = mkimage.asm.read_program(os.path.join(HERE, 'bringup.s'), depth=512)
        self.assertEqual(img.warnings, [])
        with open(BRINGUP_IMG) as fh:
            self.assertEqual(fh.read(), mkimage.image_text(img, 'host/bringup.s'),
                             'run: python3 host/mkimage.py host/bringup.s -o host/bringup.img')


# ---- the board script, against stand-ins for machine, time and the SDK -----------------------------------------
class FakeBoard:
    """MicroPython's machine module and time on the RP2350, and the SDK, as far as board.py uses them. Time is
    virtual, in microseconds; every pin change is logged with it. The SPI block answers through simlink.py."""

    def __init__(self, switches=0):
        fb = self
        self.t = 0.0
        self.log = []                   # (t, what, args)
        self.pins = {}                  # gpio -> dict(mode, pull, value)
        self.switches = switches        # bit i: ui i's DIP switch on
        self.sys_hz = 150000000
        self.peri_src = 2               # pll_usb, 48 MHz, as MicroPython leaves it
        self.pwm = None
        self.busy_reads = 0
        self.bsy_polls = 0              # busy reads in a row
        self.pads = {}
        self.sim = simlink.SimLink(seed=11)
        self.sim.run(10)

        class Pin:
            IN, OUT, PULL_UP, PULL_DOWN = 0, 1, 1, 2

            def __init__(self, gpio, mode=-1, pull=-1, value=None):
                self.gpio = gpio
                if mode != -1 or pull != -1 or value is not None:
                    self.init(mode, pull, value)

            def init(self, mode=-1, pull=-1, value=None):
                p = fb.pins.setdefault(self.gpio, dict(mode=None, pull=None, value=0))
                if value is not None:
                    p['value'] = value
                if mode != -1:
                    p['mode'] = mode
                p['pull'] = None if pull == -1 else pull
                fb.log.append((fb.t, 'pin', (self.gpio, p['mode'], p['pull'], p['value'])))

            def value(self, v=None):
                p = fb.pins[self.gpio]
                if v is None:
                    if p['mode'] == Pin.OUT:
                        return p['value']
                    high = self.gpio in (17, 18, 19) and fb.switches >> (self.gpio - 17) & 1
                    return 1 if high else (1 if p['pull'] == Pin.PULL_UP else 0)
                p['value'] = v
                fb.log.append((fb.t, 'set', (self.gpio, v)))

            __call__ = value

        class PWM:
            def __init__(self, pin, freq=None, duty_u16=None):
                top = (fb.sys_hz + freq // 2) // freq - 1
                fb.pwm = dict(gpio=pin.gpio, top=top, cc=(duty_u16 * (top + 1) + 32767) // 65535)
                fb.log.append((fb.t, 'pwm', (pin.gpio, freq, duty_u16)))

        class SPI:
            MSB = 0

            def __init__(self, id, baudrate=None, polarity=None, phase=None, bits=None, firstbit=None, sck=None,
                         mosi=None, miso=-1):
                fb.spi_args = dict(id=id, baudrate=baudrate, polarity=polarity, phase=phase, bits=bits,
                                   firstbit=firstbit, sck=getattr(sck, 'gpio', sck), mosi=getattr(mosi, 'gpio', mosi),
                                   miso=getattr(miso, 'gpio', miso))
                fb.log.append((fb.t, 'spi', fb.spi_args))
                fb.spi_peri_src = fb.peri_src

            def write_readinto(self, out, buf):
                fb.log.append((fb.t, 'xfer', (bytes(out), fb.pins[17]['value'], fb.pins[17]['mode'])))
                buf[:] = fb.sim.xfer(out)
                fb.t += 8 * len(out) * 0.2
                fb.busy_reads = 2       # the last byte is in, but SCK is still moving

        class Mem32:
            def __getitem__(self, a):
                if a == 0x4008000C:
                    fb.log.append((fb.t, 'bsy', fb.busy_reads > 0))
                    b = fb.busy_reads > 0
                    fb.busy_reads = max(0, fb.busy_reads - 1)
                    fb.bsy_polls = fb.bsy_polls + 1 if b else 0
                    assert fb.bsy_polls <= 50000, 'BSY polled without end'
                    return 0x10 if b else 0x03
                if a == 0x400A8010:
                    return fb.pwm['top']
                if a == 0x400A800C:
                    return fb.pwm['cc']
                if a == 0x400A8004:
                    return 0x10
                if a == 0x40010048:
                    return fb.peri_src << 5 | 1 << 11
                if a in (0xD0000004, 0xD0000008):
                    return fb.pads.get(a, 0)
                return fb.pads.get(a, 0x56)             # a pad register at its reset value

            def __setitem__(self, a, v):
                fb.pads[a] = v
                fb.log.append((fb.t, 'mem32', (a, v)))

        def freq(*a):
            if not a:
                return fb.sys_hz
            fb.sys_hz = a[0]
            if len(a) > 1 and a[1] == a[0]:
                fb.peri_src = 1
            elif len(a) == 1:
                fb.peri_src = 2
            fb.log.append((fb.t, 'freq', a))

        def advance(us):
            fb.t += us
            fb.sim.pause_us(us) if us < 5000 else None

        self.machine = types.SimpleNamespace(Pin=Pin, PWM=PWM, SPI=SPI, mem32=Mem32(), freq=freq)
        self.time = types.SimpleNamespace(sleep_us=advance, sleep_ms=lambda ms: advance(1000 * ms))

        class Mux:
            def reset_and_clock_mux(self, n):
                fb.log.append((fb.t, 'mux', n))

            def has(self, name):
                return name == 'tt_um_sarkar22_pe'

            def get(self, name):        # the SDK's Design.enable() returns True once the project is selected
                return types.SimpleNamespace(enable=lambda: fb.log.append((fb.t, 'mux', name)) or True)

        class TT:
            def __init__(self):
                self.shuttle = Mux()
                self._mode = 1
                self.apply_configs = True

            @property
            def mode(self):
                return self._mode

            @mode.setter
            def mode(self, m):
                self._mode = m
                fb.log.append((fb.t, 'mode', m))

        tt = TT()
        self.sdk = {'ttboard': types.ModuleType('ttboard'),
                    'ttboard.demoboard': types.SimpleNamespace(DemoBoard=types.SimpleNamespace(get=lambda: tt)),
                    'ttboard.mode': types.SimpleNamespace(RPMode=types.SimpleNamespace(ASIC_MANUAL_INPUTS=2))}
        self.tt = tt

    def load_board(self, path=os.path.join(HERE, 'board.py')):
        """board.py with this stand-in for machine and time."""
        saved = dict((k, sys.modules.get(k)) for k in ['machine'] + list(self.sdk))
        sys.modules['machine'] = self.machine
        sys.modules.update(self.sdk)
        try:
            mod = types.ModuleType('board')
            mod.__file__ = path
            with open(path) as fh:
                exec(compile(fh.read(), path, 'exec'), mod.__dict__)
        finally:
            for k, v in saved.items():
                if v is None:
                    sys.modules.pop(k, None)
                else:
                    sys.modules[k] = v
        mod.time = self.time
        mod.machine = self.machine
        mod.Pin = self.machine.Pin
        self.board = mod
        return mod

    def setup(self, project='tt_um_sarkar22_pe', **kw):
        saved = dict((k, sys.modules.get(k)) for k in self.sdk)
        sys.modules.update(self.sdk)
        try:
            return self.board.setup(project, log=lambda s: None, **kw)
        finally:
            for k, v in saved.items():
                if v is None:
                    sys.modules.pop(k, None)
                else:
                    sys.modules[k] = v

    def first(self, what, pred=lambda a: True):
        for i, (t, w, a) in enumerate(self.log):
            if w == what and pred(a):
                return i, t
        return None, None


class Board(unittest.TestCase):
    def setUp(self):
        self.fb = FakeBoard()
        self.fb.load_board()

    def test_setup_order(self):
        fb = self.fb
        host = fb.setup()
        self.assertIsInstance(host, pehost.Host)
        mode, _ = fb.first('mode')
        mux, _ = fb.first('mux')
        rst_low, t_low = fb.first('pin', lambda a: a[0] == 14 and a[1] == 1 and a[3] == 0)
        dip, _ = fb.first('pin', lambda a: a[0] in (17, 18, 19))
        clk, t_clk = fb.first('freq')
        pwm, _ = fb.first('pwm')
        cs_high, _ = fb.first('pin', lambda a: a[0] == 17 and a[1] == 1 and a[3] == 1)
        spi, _ = fb.first('spi')
        rel, t_rel = fb.first('pin', lambda a: a[0] == 14 and a[1] == 0)
        x, t_x = fb.first('xfer')
        self.assertEqual(fb.tt.mode, 2, 'ASIC_MANUAL_INPUTS: every project pin an input')
        self.assertFalse(fb.tt.apply_configs)
        self.assertTrue(mode < mux < rst_low < dip < clk < pwm < cs_high < spi < rel < x, 'the order of setup()')
        self.assertGreaterEqual(t_rel - t_clk, 1000, 'reset held at least 1 ms with the clock running')
        self.assertGreaterEqual(t_x - t_rel, 20000, '20 ms after the release before the first transaction')
        self.assertEqual(fb.log[x][2][0], b'\x00\x00\x00', 'the first transaction is CAPS')

    def test_reset_rule(self):
        """Section 9.1: CS high or SCK low while rst_n is low and as it rises. board.py keeps both."""
        fb = self.fb
        fb.setup()
        rel, t_rel = fb.first('pin', lambda a: a[0] == 14 and a[1] == 0)
        cs = [a for t, w, a in fb.log[:rel] if w in ('pin', 'set') and a[0] == 17]
        self.assertEqual(cs[-1][-1], 1, 'CS is high as rst_n is released')
        self.assertEqual(fb.pins[17]['mode'], 1, 'CS is driven, not left to a pull')
        self.assertEqual(fb.spi_args['polarity'], 0, 'SCK idles low (mode 0)')
        spi, _ = fb.first('spi')
        self.assertLess(spi, rel, 'SPI0 owns SCK, idle low, before rst_n is released')

    def test_link_settings(self):
        fb = self.fb
        fb.setup()
        self.assertEqual(fb.spi_args, dict(id=0, baudrate=5000000, polarity=0, phase=0, bits=8, firstbit=0,
                                           sck=18, mosi=19, miso=36))
        self.assertEqual(fb.spi_peri_src, 1, 'SPI0 built after its clock moved to pll_sys')
        self.assertEqual(fb.log[fb.first('freq')[0]][2], (100000000, 100000000))
        self.assertEqual(fb.pwm, dict(gpio=16, top=1, cc=1), 'PWM: one 100 MHz cycle high, one low')

    def test_xfer_waits_for_busy(self):
        fb = self.fb
        host = fb.setup()
        start = len(fb.log)
        host.status()
        ev = fb.log[start:]
        kinds = [(w, a) for t, w, a in ev]
        x = [i for i, (w, a) in enumerate(kinds) if w == 'xfer'][0]
        self.assertEqual(kinds[x - 1], ('set', (17, 0)), 'CS falls before the transfer')
        idle = [i for i, (w, a) in enumerate(kinds) if w == 'bsy' and not a][0]
        rise = [i for i, (w, a) in enumerate(kinds) if w == 'set' and a == (17, 1)][0]
        self.assertTrue(x < idle < rise, 'CS rises only after SPI0 reads idle')
        t_rise = ev[rise][0]
        self.assertGreaterEqual(fb.t - t_rise, 1, 'CS stays high at least 1 us')

    def test_cs_rises_after_an_error(self):
        """An exception inside write_readinto (MicroPython raises OSError on an RX overrun) still ends the
        transaction: CS rises once SPI0 reads idle, so the next transaction starts with a falling CS."""
        fb = self.fb
        host = fb.setup()
        spi = host.link.spi

        def broken(out, buf):
            fb.busy_reads = 2
            raise OSError(5)
        spi.write_readinto = broken
        start = len(fb.log)
        with self.assertRaises(OSError):
            host.status()
        kinds = [(w, a) for t, w, a in fb.log[start:]]
        rise = kinds.index(('set', (17, 1)))
        self.assertIn(('bsy', False), kinds[:rise], 'SPI0 read idle before CS rose')
        self.assertEqual(fb.pins[17]['value'], 1, 'CS high after the error')
        del spi.write_readinto
        self.assertEqual(host.status().run, 0, 'the next transaction works')

    def test_stuck_busy(self):
        """SPI0 that never reads idle: xfer() stops polling after a bounded wait, raises LinkError, and CS is high."""
        fb = self.fb
        host = fb.setup()
        spi = host.link.spi

        def stuck(out, buf):
            buf[:] = bytes(len(out))
            fb.busy_reads = 10 ** 9
        spi.write_readinto = stuck
        with self.assertRaises(pehost.LinkError):
            host.status()
        self.assertLess(fb.bsy_polls, 10100, 'about 10,000 polls, then it gives up')
        self.assertEqual(fb.pins[17]['value'], 1, 'CS high after the error')

    def test_switch_on(self):
        for sw, name in ((1, 'ui0'), (2, 'ui1'), (4, 'ui2')):
            fb = FakeBoard(switches=sw)
            fb.load_board()
            with self.assertRaises(pehost.LinkError) as e:
                fb.setup()
            self.assertIn(name, str(e.exception))
            self.assertIsNone(fb.first('freq')[0], 'nothing else is set up')

    def test_project_by_address(self):
        fb = self.fb
        fb.setup(project=200)
        self.assertEqual(fb.log[fb.first('mux')[0]][2], 200)
        with self.assertRaises(ValueError):
            fb.setup(project='tt_um_someone_else')     # not in the shuttle index, and not an address
        fb.tt.shuttle.get = lambda name: types.SimpleNamespace(enable=lambda: False)
        with self.assertRaises(ValueError):
            fb.setup()                                  # the SDK refused to enable it

    def test_release_and_probe(self):
        fb = self.fb
        b = fb.board
        b.release(30)
        a = 0x40038000 + 4 + 4 * 30
        self.assertEqual(fb.pads[a] & (1 << 6 | 1 << 3 | 1 << 2), 0, 'input buffer and both pulls off')
        fb.pads[0xD0000004] = 0x55 << 25
        fb.pads[0xD0000008] = 1 | 0xA6 << 1
        self.assertEqual(b.probe(), (0xA6, 0xD5))

    def test_memory_test(self):
        fb = self.fb
        host = fb.setup()
        rep = fb.board.memory_test(host, log=lambda s: None)
        self.assertEqual([r[:2] for r in rep.rows], [(True, 'memory, pattern 1'), (True, 'memory, pattern 2')])
        self.assertEqual(fb.sim.chip.mem[:3], [bringup.pattern(a, 1) for a in range(3)])

    def test_bringup_through_the_board(self):
        fb = self.fb
        host = fb.setup()
        fb.board.probe = lambda: fb.sim.probe()
        lines = []
        cwd = os.getcwd()
        os.chdir(HERE)
        try:
            rep = fb.board.bringup(host, log=lines.append)
        finally:
            os.chdir(cwd)
        self.assertEqual(rep.failed(), [], '\n'.join(lines))


class Recorder:
    """A transport that passes calls to another and writes them as a transcript for replay.py."""

    def __init__(self, link):
        self.link, self.lines = link, []

    def xfer(self, out):
        r = self.link.xfer(out)
        self.lines.append('x %s %s' % (bytes(out).hex(), bytes(r).hex()))
        return r

    def pause_us(self, us):
        self.lines.append('p %s' % us)
        self.link.pause_us(us)

    def probe(self):
        uo, uio = self.link.probe()
        self.lines.append('r %d %d' % (uo, uio))
        return uo, uio


def transcript(seed=12):
    """The bring-up on the simulator, over a link fast enough (1 cycle per bit) that some TAKEs find their word still
    empty, so the polling and its pauses are in the transcript too."""
    link = simlink.SimLink(seed=seed, cycles_per_bit=1, gap=0)
    link.run(10)
    rec = Recorder(link)
    with open(BRINGUP_IMG) as fh:
        img = pehost.read_image(fh)
    rep = bringup.run(pehost.Host(rec), img, probe=rec.probe, log=lambda s: None)
    assert not rep.failed(), rep.failed()
    return '\n'.join(rec.lines) + '\n'


class Transcript(unittest.TestCase):
    def test_replay(self):
        text = transcript()
        out = io.StringIO()
        saved = sys.stdout
        sys.stdout = out
        try:
            path = os.path.join(HERE, '.transcript.tmp')
            with open(path, 'w') as fh:
                fh.write(text)
            self.assertEqual(replay.main(path), 0)
            bad = text.replace('x 3f 00', 'x 3e 00')
            with open(path, 'w') as fh:
                fh.write(bad)
            with self.assertRaises(replay.Mismatch):
                replay.main(path)
        finally:
            sys.stdout = saved
            os.remove(path)
        self.assertIn('all as recorded; bring-up 24 passed, 0 failed', out.getvalue())


if __name__ == '__main__':
    if '--transcript' in sys.argv:
        i = sys.argv.index('--transcript')
        with open(sys.argv[i + 1], 'w') as fh:
            fh.write(transcript())
        del sys.argv[i:i + 2]
    unittest.main(verbosity=2)
