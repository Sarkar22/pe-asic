"""Benches for the smaller programs on the shift units (section 5.9): 1-Wire, UART with parity, WS2812, PRBS and
an additive scrambler, and the idle send half as a CRC engine.

Each part has a reference written from the standard's text before the firmware, which the firmware doesn't
share: a 1-Wire device with a ROM code and the slot limits of the DS18B20 data sheet; a UART sender and judge
with even and odd parity; the WS2812B data sheet's high and low time windows; Fibonacci PRBS generators named by
ITU-T O.150's polynomials; zlib.crc32.
"""
import random
import zlib

from report import assemble, check
from simple import Chip, read

PROGRAMS = ('advanced/ow_unit.s', 'advanced/uart_par_unit.s', 'advanced/ws2812_unit.s', 'advanced/lfsr_unit.s',
            'advanced/crc_unit.s', 'advanced/xy2_unit.s')
OW_PROG = 'advanced/ow_unit.s'
UP_PROG = 'advanced/uart_par_unit.s'
WS_PROG = 'advanced/ws2812_unit.s'
LF_PROG = 'advanced/lfsr_unit.s'
CR_PROG = 'advanced/crc_unit.s'
XY_PROG = 'advanced/xy2_unit.s'
CLK = 50_000_000


def us(x):
    return int(round(x * CLK / 1e6))


class _Idle:
    def step(self, chip, c, uo, en, val):
        return 0xFF, 0xFF


# ---------------------------------------------------------------------------------------------------------------
# 1-Wire: advanced/ow_unit.s

def crc8_maxim(data):
    """CRC-8/MAXIM (Dallas/Maxim 1-Wire): x^8 + x^5 + x^4 + 1, reflected (0x8C), init 0, no final XOR, as Maxim's
    application note 27 computes it bit by bit, LSB first. Catalogue check value 0xA1 for '123456789'."""
    c = 0
    for b in data:
        for i in range(8):
            f = (c ^ (b >> i)) & 1
            c >>= 1
            if f:
                c ^= 0x8C
    return c


def rom_code(family, serial):
    """The 8 bytes of a 1-Wire ROM code in wire order: family code, 48-bit serial number LSB first, CRC byte."""
    b = [family] + [(serial >> (8 * i)) & 0xFF for i in range(6)]
    return b + [crc8_maxim(b)]


AN27_ROM = rom_code(0x02, 0x000001B81C)         # Maxim AN27's worked example: its CRC byte is 0xA2


class OwDevice:
    """A 1-Wire slave at standard speed with a 64-bit ROM code, written from the DS18B20 data sheet's 1-Wire
    signalling section. A low of 480 us or more is a reset: pd_wait_us after the line rises the device pulls it
    low for pd_low_us (presence). Then each fall of the line it didn't cause starts a slot: it samples a write slot
    sample_us after the fall (15 to 60 us in the data sheet), and after the command 0x33 (Read ROM) it answers 64
    read slots, LSB first, pulling the line low from resp cycles after the fall until hold_us after it for a 0
    (data valid at least 15 us after the fall). It sees the line as it was in the cycle before."""

    def __init__(self, rom, sample_us=30, hold_us=30, pd_wait_us=30, pd_low_us=120, resp=1, present=True):
        self.rom = sum(b << (8 * i) for i, b in enumerate(rom))
        self.sample, self.hold = us(sample_us), us(hold_us)
        self.pd_wait, self.pd_low, self.resp = us(pd_wait_us), us(pd_low_us), resp
        self.present = present
        self.prev = 1
        self.low = False
        self.fall = None
        self.sample_at = None
        self.pull = None                # (from, to): cycles the device pulls the line low
        self.state = 'idle'
        self.bits = []
        self.k = 0
        self.resets = 0
        self.cmds = []
        self.rom_bits_sent = 0

    def step(self, c, line):
        if self.prev and not line and not self.low:                 # a fall the device didn't cause
            self.fall = c
            if self.state == 'cmd':
                self.sample_at = c + self.sample
            elif self.state == 'rom' and self.k < 64:
                if not (self.rom >> self.k) & 1:
                    self.pull = (c + self.resp, c + self.hold)
                self.k += 1
                self.rom_bits_sent = self.k
                if self.k == 64:
                    self.state = 'done'
        if not self.prev and line and self.fall is not None and not self.low:
            if c - self.fall >= us(480):
                self.resets += 1
                self.state, self.bits, self.k = 'cmd', [], 0
                self.sample_at = None
                if self.present:
                    self.pull = (c + self.pd_wait, c + self.pd_wait + self.pd_low)
            self.fall = None
        if self.sample_at is not None and c >= self.sample_at:
            self.sample_at = None
            if self.state == 'cmd':
                self.bits.append(line)
                if len(self.bits) == 8:
                    v = sum(b << i for i, b in enumerate(self.bits))
                    self.cmds.append(v)
                    self.state = 'rom' if v == 0x33 else 'other'
        self.low = self.pull is not None and self.pull[0] <= c < self.pull[1]
        if self.pull is not None and c >= self.pull[1]:
            self.pull = None
        self.prev = line


class OwBus:
    """The 1-Wire line on io4: low while the chip's open-drain pad or the device pulls it, otherwise high once the
    pull-up has raised it, rise cycles after the last pull ends. Logs the line and the chip's own pull."""

    PIN = 4

    def __init__(self, dev, rise=0):
        self.dev, self.rise = dev, rise
        self.last_low = -10 ** 9
        self.level = 1
        self.line_ch = []               # (cycle, level) at each change of the line
        self.chip_ch = []               # (cycle, 1 if the chip pulls low)
        self.chip_was = 0
        self.drive_high_od = 0          # cycles the chip drove the pin high (must stay 0: it is in OD)

    def step(self, chip, c, uo, en, val):
        b = 1 << self.PIN
        chip_low = 1 if en & b and not val & b else 0
        if en & b and val & b:
            self.drive_high_od += 1
        pulled = chip_low or (self.dev is not None and self.dev.low)
        if pulled:
            self.last_low = c
        lvl = 0 if pulled else (1 if c - self.last_low > self.rise else 0)
        if lvl != self.level:
            self.line_ch.append((c, lvl))
            self.level = lvl
        if chip_low != self.chip_was:
            self.chip_ch.append((c, chip_low))
            self.chip_was = chip_low
        if self.dev is not None:
            self.dev.step(c, lvl)
        return 0xFF, 0xFF & ~(0 if lvl else b)


class OwResult:
    pass


def ow_run(cmds, rom=AN27_ROM, dev_kw=None, present=True, tid=0, align=0, rise=0, text=None, defs=None,
           limit=None):
    """Runs ow_unit.s for a list of MIN words. Returns the presence results and words per transaction, the bus
    and the device."""
    img = assemble(text or read(OW_PROG), defines=defs, depth=128)
    chip = Chip(img.words, 128)
    while chip.c % 4 != align:
        chip.cycle(_Idle())
    dev = OwDevice(rom, present=present, **(dev_kw or {}))
    bus = OwBus(dev, rise)
    chip.start(tid)
    posts = list(cmds)
    want = []
    for w in cmds:
        n = (w >> 8) & 0xFF
        want.append((8 + 8 * n + 15) // 16)
    res, cur = [], None
    limit = limit or sum(60000 + (8 + 8 * ((w >> 8) & 0xFF)) * 3900 + 8000 for w in cmds) + 2000
    while chip.c < limit and len(res) < len(cmds):
        if posts and chip.post(3, posts[0]):
            posts.pop(0)
        chip.cycle(bus)
        v = chip.take(5)
        if v is None:
            continue
        if cur is None:
            cur = dict(presence=v, words=[], end=None, cycle=chip.c)
            if v == 0:
                res.append(cur)
                cur = None
        elif v >= 0x100 and len(cur['words']) == want[len(res)]:
            cur['end'] = v
            res.append(cur)
            cur = None
        else:
            cur['words'].append(v)
    for _ in range(4000):                    # the last slot ends, so a decoder sees it whole
        chip.cycle(bus)
    r = OwResult()
    r.chip, r.bus, r.dev, r.res, r.cmds = chip, bus, dev, res, cmds
    r.late = [t.id for t in chip.t if t.late or t.th.ILL]
    return r


def ow_bytes(words, nbits):
    """The bit stream of a transaction's words, LSB first, as bytes."""
    bits = [(words[i // 16] >> (i % 16)) & 1 for i in range(nbits)]
    return [sum(bits[8 * k + i] << i for i in range(8)) for k in range(nbits // 8)]


def ow_judge(r, rom=AN27_ROM, present=True):
    """Every transaction: the presence result; for a device, the command read back, the bytes read equal to the
    ROM code when the command is Read ROM, and end flag 0 set exactly when the bytes' CRC-8 residue is 0."""
    bad = []
    if r.late:
        bad.append('LATE or ILL on threads %s' % r.late)
    if r.bus.drive_high_od:
        bad.append('the chip drove the open-drain pin high on %d cycles' % r.bus.drive_high_od)
    if len(r.res) != len(r.cmds):
        bad.append('%d of %d transactions reported' % (len(r.res), len(r.cmds)))
    for w, x in zip(r.cmds, r.res):
        if x['presence'] != (1 if present else 0):
            bad.append('presence %d, device %s' % (x['presence'], present))
            continue
        if not present:
            continue
        n = (w >> 8) & 0xFF
        got = ow_bytes(x['words'], 8 + 8 * n)
        if got[0] != (w & 0xFF):
            bad.append('command read back as %#x, sent %#x' % (got[0], w & 0xFF))
        if (w & 0xFF) == 0x33:
            if got[1:] != list(rom)[:n]:
                bad.append('bytes read %s, ROM %s' % (bytes(got[1:]).hex(), bytes(rom[:n]).hex()))
        good = crc8_maxim(got[1:]) == 0
        if x['end'] != 0x100 | (1 if good else 0):
            bad.append('end word %#x, the bytes\' CRC residue is %s' % (x['end'] or 0, '0' if good else 'not 0'))
    return bad


def ow_timing(r):
    """The chip's low pulses on the line, sorted into the data sheet's intervals, in microseconds: reset low, write
    1 low (and read-slot low), write 0 low, slot (fall to fall), recovery (the line's rise to the next fall), and
    the reset's high time to the first slot. Returns (measured, violations)."""
    m = dict(rstl=[], low1=[], low0=[], slot=[], rec=[], rsth=[])
    bad = []
    pulses, start = [], None
    for c, v in r.bus.chip_ch:
        if v:
            start = c
        elif start is not None:
            pulses.append((start, c))
            start = None
    line = r.bus.line_ch
    rises = [c for c, v in line if v == 1]
    last_rst_rise = None
    for i, (a, b) in enumerate(pulses):
        w = (b - a) / 50
        if w >= 480:
            m['rstl'].append(w)
            last_rst_rise = next((x for x in rises if x >= b), None)
            continue
        (m['low1'] if w <= 15 else m['low0']).append(w)
        if last_rst_rise is not None:
            m['rsth'].append((a - last_rst_rise) / 50)
            last_rst_rise = None
        if i + 1 < len(pulses) and (pulses[i + 1][1] - pulses[i + 1][0]) / 50 < 480:
            m['slot'].append((pulses[i + 1][0] - a) / 50)
            rise = next((x for x in rises if x >= b), None)
            if rise is not None and rise < pulses[i + 1][0]:
                m['rec'].append((pulses[i + 1][0] - rise) / 50)
    lim = dict(rstl=(480, 960), low1=(1, 15), low0=(60, 120), slot=(60, 120), rec=(1, 1e9), rsth=(480, 1e9))
    for k, (lo, hi) in lim.items():
        for v in m[k]:
            if not lo <= v <= hi:
                bad.append('%s %.2f us outside %s..%s' % (k, v, lo, hi))
                break
    return {k: (min(v), max(v)) if v else None for k, v in m.items()}, bad


def ow_case(cmds, rom=AN27_ROM, present=True, **kw):
    r = ow_run(cmds, rom=rom, present=present, **kw)
    bad = ow_judge(r, rom, present)
    tm, tbad = ow_timing(r)
    return r, bad + tbad, tm


READ_ROM = 0x0833               # Read ROM, then 8 bytes


def check_ow(log):
    log.group(OW_PROG, title='1-Wire master on the unit: slots from words on io4 in OD, CRC-8 residue')
    log.check('reference: CRC-8/MAXIM of "123456789" is 0xA1, AN27\'s ROM 02 1C B8 01 00 00 00 has CRC 0xA2',
              crc8_maxim(b'123456789') == 0xA1 and AN27_ROM[7] == 0xA2 and crc8_maxim(AN27_ROM) == 0)
    good, bad1, tms = 0, [], []
    for tid in range(4):
        for al in range(4):
            r, bad, tm = ow_case([READ_ROM], tid=tid, align=al)
            good += not bad
            tms.append(tm)
            if bad:
                bad1.append('t%d a%d: %s' % (tid, al, bad[:2]))
    log.check('Read ROM of AN27\'s device on 4 threads x 4 alignments: presence, 0x33 read back, the 8 ROM bytes, '
              'CRC flag set', good == 16, '%d of 16 %s' % (good, bad1[:2]))
    tm = tms[0]
    log.info('1-Wire intervals (us): ' + ', '.join('%s %s' % (k, v) for k, v in tm.items()))
    rng = random.Random(7)
    good, n, det = 0, 0, []
    for k in range(6):
        rom = rom_code(rng.choice((0x28, 0x10, 0x26, 0x01)), rng.getrandbits(48))
        r, bad, _ = ow_case([READ_ROM, READ_ROM], rom=rom, tid=k % 4, align=(k * 3) % 4)
        n += 1
        good += not bad
        if bad:
            det.append(bad[:2])
    log.check('two Read ROM transactions back to back, 6 random ROM codes', good == n, '%d of %d %s' % (good, n,
                                                                                                  det[:1]))
    bad_rom = list(AN27_ROM)
    bad_rom[3] ^= 0x10
    r, bad, _ = ow_case([READ_ROM], rom=bad_rom, tid=2, align=1)
    log.check('a ROM code with one bit flipped: bytes read as sent, CRC flag clear', not bad and
              r.res and r.res[0]['end'] == 0x100, str(bad[:2]) + ' end %s' % (r.res[0]['end'] if r.res else None))
    r, bad, _ = ow_case([READ_ROM], present=False, tid=1, align=3)
    log.check('no device: presence 0, no slots', not bad and len(r.bus.chip_ch) == 2, str(bad[:2]))
    r, bad, _ = ow_case([0x0033, 0x0433], tid=3, align=2)
    log.check('Read ROM with 0 and 4 bytes read (RCNT 8 and 40)', not bad, str(bad[:2]))
    corners = [dict(sample_us=15), dict(sample_us=60), dict(hold_us=15.5), dict(hold_us=60), dict(pd_wait_us=15),
               dict(pd_wait_us=60), dict(pd_low_us=60), dict(pd_low_us=240), dict(resp=250)]
    good, det = 0, []
    for k, kw in enumerate(corners):
        r, bad, _ = ow_case([READ_ROM], dev_kw=kw, tid=k % 4, align=k % 4)
        good += not bad
        if bad:
            det.append((kw, bad[:1]))
    log.check('device timing corners: write sample 15 and 60 us, read 0 held 15.5 and 60 us, presence 15 and 60 us '
              'after the release and 60 and 240 us long, a 5 us response', good == len(corners),
              '%d of %d %s' % (good, len(corners), det[:2]))
    good, det = 0, []
    for rise in (50, 150, 300, 340):
        r, bad, _ = ow_case([READ_ROM], rise=rise, tid=rise % 4, align=1)
        good += not bad
        if bad:
            det.append((rise, bad[:1]))
    log.check('line rise times 1, 3, 6 and 6.8 us', good == 4, '%d of 4 %s' % (good, det[:2]))


def check_ow_limits(log):
    """Where the margins end: the rise time against the 13 us sample, and the device's read-0 hold."""
    log.group(OW_PROG, title='1-Wire limits')
    r, bad, _ = ow_case([READ_ROM], rise=352)
    log.check('a 7.04 us rise reads a 1 slot as 0: the sample is 7 us after the master\'s release', bad != [],
              str(bad[:1]))
    r, bad, _ = ow_case([READ_ROM], dev_kw=dict(hold_us=12.9))
    log.check('a device that holds a read 0 only 12.9 us is read as a 1 (the sample is at 13 us)', bad != [],
              str(bad[:1]))


# ---------------------------------------------------------------------------------------------------------------
# UART 8E1 and 8O1: advanced/uart_par_unit.s

def par_bit(byte, odd):
    """The parity bit that makes the count of ones in the 8 data bits and itself even (odd = 0) or odd (odd = 1)."""
    return (bin(byte & 0xFF).count('1') + odd) & 1


class UartSender:
    """A UART sender: start bit 0, 8 data bits LSB first, a parity bit, one stop bit 1, at bit cycles per bit (any
    positive number), the first start bit at t0, gap cycles of idle between frames. Planted faults: frames in flip
    get the wrong parity bit, frames in nostop a stop bit of 0, followed by one bit of idle."""

    def __init__(self, data, bit, odd, t0=300.0, gap=0.0, flip=(), nostop=()):
        ev, t = [], t0
        self.frames = []
        for k, b in enumerate(data):
            p = par_bit(b, odd) ^ (1 if k in flip else 0)
            stop = 0 if k in nostop else 1
            bits = [0] + [(b >> i) & 1 for i in range(8)] + [p, stop]
            for j, v in enumerate(bits):
                ev.append((int(t + j * bit), v))
            if not stop:                        # a stop bit of 0, then one bit of idle so the next start bit falls
                ev.append((int(t + 11 * bit), 1))
                t += bit
            self.frames.append((b, p, stop))
            t += 11 * bit + gap
        ev.append((int(t), 1))
        self.end = t
        self.ev = sorted(ev)
        self.i, self.lv = 0, 1

    def level(self, c):
        while self.i < len(self.ev) and self.ev[self.i][0] <= c:
            self.lv = self.ev[self.i][1]
            self.i += 1
        return self.lv


class UartBoard:
    """io0 is the chip's TX pad (pulled up while not driven) and reads back its own level; ui4 carries a sender
    model's line, if there is one."""

    def __init__(self, sender=None):
        self.sender = sender
        self.tx = 1
        self.tx_ch = []

    def step(self, chip, c, uo, en, val):
        lvl = (val & 1) if en & 1 else 1
        if lvl != self.tx:
            self.tx_ch.append((c, lvl))
            self.tx = lvl
        ui = 0xFF
        if self.sender is not None and not self.sender.level(c):
            ui &= ~(1 << 4)
        return ui, 0xFE | lvl


def uart_par_decode(changes, bit, end):
    """Frames on a line from its changes: a fall from idle starts a frame, each of its 11 bits is read in its
    middle. Returns [(byte, parity bit, stop bit)]; a reference reading of the UART frame, not the firmware's."""
    out = []
    lv, t_ok, i = 1, 0, 0

    def at(t):
        v = 1
        for c, x in changes:
            if c > t:
                break
            v = x
        return v
    for c, v in changes:
        if v == 0 and c >= t_ok:
            if c + 11 * bit > end:
                break
            s = [at(int(c + (k + 0.5) * bit)) for k in range(11)]
            out.append((sum(s[1 + i] << i for i in range(8)), s[9], s[10]))
            t_ok = c + 10.5 * bit
    return out


def up_run(bursts=(), bit=50, odd=0, ttx=0, trx=1, align=0, sender=None, nrx=8, text=None, tail=None):
    """Runs both threads of uart_par_unit.s: tx sends the bursts (lists of bytes) through io0, rx receives on
    input pin nrx (io0, or ui4 for a sender model). Returns the frames rx reported, tx's statuses, the board."""
    img = assemble(text or read(UP_PROG), defines=dict(ODD=odd, NRX=nrx), depth=128)
    chip = Chip(img.words, 128)
    while chip.c % 4 != align:
        chip.cycle(_Idle())
    if sender is not None:
        sender.i, sender.lv = 0, 1              # a sender can be run again, as sigrok's --rtl does
    board = UartBoard(sender)
    chip.sim.host_setpc(ttx, img.labels['tx'])
    chip.sim.host_setpc(trx, img.labels['rx'])
    chip.sim.host_run(0, (1 << ttx) | (1 << trx))
    posts = [(7, bit), (7, bit)]
    nframes = 0
    for bb in bursts:
        for k, b in enumerate(bb):
            posts.append((0, b | (0x100 if k == len(bb) - 1 else 0)))
        nframes += len(bb)
    if sender is not None:
        nframes += len(sender.frames)
    got, sts = [], []
    limit = 3400 + int(bit * 11 * (nframes + 2 * len(bursts) + 2)) + (int(sender.end) if sender else 0)
    t_run = chip.c
    while chip.c < limit:
        # the bytes go only once the receiver hunts: its set-up takes about 30 slots after RUN
        if posts and (posts[0][0] == 7 or chip.c >= t_run + 400) and chip.post(posts[0][0], posts[0][1]):
            posts.pop(0)
        chip.cycle(board)
        v = chip.take(1)
        if v is not None:
            got.append(v)
        v = chip.take(2)
        if v is not None:
            sts.append(v)
        if not posts and len(got) >= nframes and len(sts) >= len(bursts) and chip.c > (sender.end if sender
                                                                                         else 0) + 20 * bit:
            break
    for _ in range(int(2 * bit) + 50):
        chip.cycle(board)
    r = OwResult()
    r.chip, r.board, r.got, r.sts, r.bursts, r.bit, r.odd, r.sender = chip, board, got, sts, list(bursts), bit, odd, \
        sender
    r.late = [t.id for t in chip.t if t.late or t.th.ILL]
    return r


def up_judge(r):
    """tx: the line carries every byte of every burst, each with the parity bit of the build and a stop bit, and no
    send underran. rx: one word per frame, its data, parity and stop bits as on the line, and bit 15 set exactly
    when the parity is right and the stop bit is 1, or both are wrong."""
    bad = []
    if r.late:
        bad.append('LATE or ILL on threads %s' % r.late)
    sent = [b for bb in r.bursts for b in bb]
    if sent:
        line = uart_par_decode(r.board.tx_ch, r.bit, r.chip.c)
        want = [(b, par_bit(b, r.odd), 1) for b in sent]
        if line != want:
            bad.append('TX line %s, want %s' % (line[:6], want[:6]))
        if len(r.sts) != len(r.bursts) or any(s & 0x80 for s in r.sts):
            bad.append('TX statuses %s' % ['%04x' % s for s in r.sts])
        frames = want
    else:
        frames = list(r.sender.frames)
    words = [(w & 0xFF, (w >> 8) & 1, (w >> 9) & 1) for w in r.got]
    if words != frames:
        bad.append('RX words %s, frames %s' % (words[:6], frames[:6]))
    for w, (b, p, stop) in zip(r.got, frames):
        flag = (w >> 15) & 1
        good_p = p == par_bit(b, r.odd)
        if flag != (1 if good_p == (stop == 1) else 0):
            bad.append('RX flag %d for byte %#x parity %d stop %d' % (flag, b, p, stop))
            break
    return bad


def up_case(**kw):
    r = up_run(**kw)
    return r, up_judge(r)


def check_up(log):
    log.group(UP_PROG, title='UART 8E1 and 8O1 through two units: parity appended and checked by the CRC register')
    rng = random.Random(11)
    for odd in (0, 1):
        good, det, n = 0, [], 0
        for k, (bit, ttx, trx) in enumerate(((434, 0, 1), (50, 1, 2), (50, 2, 3), (50, 3, 0), (12, 0, 3), (12, 2, 1),
                                             (6, 3, 1), (3, 1, 0))):
            bursts = [[rng.getrandbits(8) for _ in range(m)] for m in (1, 5, 16)]
            r, bad = up_case(bursts=bursts, bit=bit, odd=odd, ttx=ttx, trx=trx, align=k % 4)
            n += 1
            good += not bad
            if bad:
                det.append((bit, bad[:1]))
        log.check('8%s1 loopback on io0, bursts of 1, 5 and 16 bytes at 434, 50, 12, 6 and 3 cycles per bit, 8 thread '
                  'pairs and alignments: the line, and every word and flag the receiver gives' % 'EO'[odd], good == n,
                  '%d of %d %s' % (good, n, det[:1]))
    for odd in (0, 1):
        data = [rng.getrandbits(8) for _ in range(12)]
        flip, nostop = (2, 7, 9), (5, 9)
        good, det = 0, []
        for k, bit in enumerate((434, 50, 17.3)):
            snd = UartSender(data, bit, odd, flip=flip, nostop=nostop, gap=[0, 3.5, 0][k])
            r, bad = up_case(bit=int(round(bit)), odd=odd, nrx=4, trx=k, align=k, sender=snd)
            good += not bad
            if bad:
                det.append((bit, bad[:1]))
        log.check('8%s1 receive from a sender model on ui4, 12 frames with wrong parity in 3, a stop bit of 0 in 2 '
                  '(one of them also with wrong parity): data, bits and flags per frame, at 434, 50 and 17.3 cycles '
                  'per bit' % 'EO'[odd], good == 3, '%d of 3 %s' % (good, det[:1]))
    good, det = 0, []
    for err in (-0.04, -0.03, 0.03, 0.04):
        data = [rng.getrandbits(8) for _ in range(10)]
        snd = UartSender(data, 50 * (1 + err), 0)
        r, bad = up_case(bit=50, odd=0, nrx=4, sender=snd)
        good += not bad
        if bad:
            det.append((err, bad[:1]))
    log.check('receive with the sender\'s clock 4% slow to 4% fast, 50 cycles per bit, frames back to back', good == 4,
              '%d of 4 %s' % (good, det[:1]))


def check_up_limits(log):
    log.group(UP_PROG, title='UART parity limits')
    rng = random.Random(12)
    bursts = [[rng.getrandbits(8) for _ in range(16)]]
    res = {}
    for bit in (2, 3):
        res[bit] = 0
        for ttx in range(4):
            for al in range(4):
                r, bad = up_case(bursts=bursts, bit=bit, odd=1, ttx=ttx, trx=(ttx + 1 + al) % 4 if (ttx + 1 + al) % 4
                                 != ttx else (ttx + 2) % 4, align=al)
                res[bit] += not bad
    log.check('a 16-byte burst is clean at 3 cycles per bit (16.7 Mbaud) on 16 of 16 thread pairs and alignments, '
              'and at 2 on none: the send half underruns', res[3] == 16 and res[2] == 0, str(res))


# ---------------------------------------------------------------------------------------------------------------
# WS2812B: advanced/ws2812_unit.s

WS2812B = dict(t0h=(250, 550), t1h=(650, 950), t0l=(700, 1000), t1l=(300, 600), bit=(650, 1850), res=50_000)
# ns, from the WS2812B data sheet: T0H 0.4 us, T1H 0.8 us, T0L 0.85 us, T1L 0.45 us, each +-150 ns; TH + TL 1.25 us
# +-600 ns; RES above 50 us


class PinLog:
    """Logs one uo pin's changes."""

    def __init__(self, pin):
        self.pin, self.lv, self.ch = pin, 0, []

    def step(self, chip, c, uo, en, val):
        v = (uo >> self.pin) & 1
        if v != self.lv:
            self.ch.append((c, v))
            self.lv = v
        return 0xFF, 0xFF


def ws_decode(ch, end):
    """Frames of bits from a WS2812B line's changes, judged by the data sheet: each high time must be in T0H's or
    T1H's window, which gives the bit, each low time inside a frame in that bit's TxL window, and a low of RES or
    more ends a frame. Returns (frames, violations, measured cycles)."""
    ns = 1e9 / CLK
    frames, cur, bad = [], [], []
    m = dict(t0h=set(), t1h=set(), t0l=set(), t1l=set(), bit=set(), gap=set())
    rises = [i for i, (c, v) in enumerate(ch) if v == 1]
    for k, i in enumerate(rises):
        r = ch[i][0]
        f = ch[i + 1][0] if i + 1 < len(ch) else None
        if f is None:
            bad.append('line left high at %d' % r)
            break
        nr = ch[rises[k + 1]][0] if k + 1 < len(rises) else end
        hi, lo = f - r, nr - f
        if WS2812B['t0h'][0] <= hi * ns <= WS2812B['t0h'][1]:
            bit = 0
        elif WS2812B['t1h'][0] <= hi * ns <= WS2812B['t1h'][1]:
            bit = 1
        else:
            bad.append('high %d cycles at %d in neither T0H nor T1H' % (hi, r))
            continue
        m['t%dh' % bit].add(hi)
        cur.append(bit)
        if lo * ns >= WS2812B['res']:
            m['gap'].add(lo)
            frames.append(cur)
            cur = []
            continue
        key = 't%dl' % bit
        m[key].add(lo)
        m['bit'].add(hi + lo)
        if not WS2812B[key][0] <= lo * ns <= WS2812B[key][1]:
            bad.append('%s %d cycles at %d' % (key, lo, f))
        if not WS2812B['bit'][0] <= (hi + lo) * ns <= WS2812B['bit'][1]:
            bad.append('bit %d cycles at %d' % (hi + lo, r))
    if cur:
        bad.append('frame not ended by a reset low (%d bits)' % len(cur))
        frames.append(cur)
    return frames, bad, m


def ws_run(frames, tid=0, align=0, tper=None, tfrac=None, text=None, host_gap=0):
    defs = {}
    if tper is not None:
        defs['TPER'] = tper
    if tfrac is not None:
        defs['TFRAC'] = tfrac
    img = assemble(text or read(WS_PROG), defines=defs or None, depth=128)
    chip = Chip(img.words, 128)
    while chip.c % 4 != align:
        chip.cycle(_Idle())
    log = PinLog(6)
    chip.start(tid)
    posts = []
    for fr in frames:
        posts += [len(fr)] + list(fr)
    limit = 2000 + sum(16000 + 600 * len(fr) for fr in frames) + host_gap * len(posts)
    nxt = 0
    while chip.c < limit:
        if posts and chip.c >= nxt and chip.post(3, posts[0]):
            posts.pop(0)
            nxt = chip.c + host_gap
        chip.cycle(log)
    r = OwResult()
    r.chip, r.log, r.frames = chip, log, frames
    r.late = [t.id for t in chip.t if t.late or t.th.ILL]
    return r


def ws_case(frames, **kw):
    r = ws_run(frames, **kw)
    got, bad, m = ws_decode(r.log.ch, r.chip.c)
    want = [[(b >> (7 - i)) & 1 for b in fr for i in range(8)] for fr in frames]
    if got != want:
        bad.append('frames %s, want %s' % ([len(f) for f in got], [len(f) for f in want]))
    if r.late:
        bad.append('LATE or ILL on threads %s' % r.late)
    return r, bad, m


def check_ws(log):
    log.group(WS_PROG, title='WS2812B driver on the unit: SYMS 2, output codes 1, b, 0, TPER 20 + TFRAC 213')
    frames = [[0x12, 0x34, 0x56, 0xFF, 0x00, 0xA5, 0x80, 0x01, 0x7E]]
    good, det, ms = 0, [], []
    for tid in range(4):
        for al in range(4):
            r, bad, m = ws_case(frames, tid=tid, align=al)
            good += not bad
            ms.append(m)
            if bad:
                det.append(bad[:1])
    log.check('3 LEDs, 72 bits, 4 threads x 4 alignments, every high and low time in the WS2812B windows',
              good == 16, '%d of 16 %s' % (good, det[:1]))
    m = ms[0]
    log.info('WS2812B cycles: T0H %s T1H %s T0L %s T1L %s bit %s' % tuple(sorted(m[k]) for k in
                                                                      ('t0h', 't1h', 't0l', 't1l', 'bit')))
    rises = [c for c, v in r.log.ch if v == 1][:72]
    drift = max(abs((x - rises[0]) - k * (20 + 213 / 256) * 3) for k, x in enumerate(rises))
    log.check('bit period 62 or 63 cycles (1.24 to 1.26 us), and bit k starts within one cycle of k x 62.496 cycles: '
              'the fraction keeps the 1.25 us mean with no drift', m['bit'] <= {62, 63} and drift < 1,
              'worst offset %.3f cycles over 72 bits' % drift)
    rng = random.Random(5)
    fr = [[rng.getrandbits(8) for _ in range(3 * n)] for n in (1, 4, 2)]
    r, bad, m = ws_case(fr, tid=2, align=3)
    log.check('three frames of 1, 4 and 2 LEDs, each latched by a low of 300 us', not bad and
              min(m['gap']) >= 15000, str(bad[:2]))
    r, bad, m = ws_case([[0x00] * 6, [0xFF] * 6], tid=1, align=1)
    log.check('all-zero and all-one frames', not bad, str(bad[:2]))
    ok = {}
    for tper in range(16, 26):
        r, bad, m = ws_case(frames, tper=tper, tfrac=0, tid=tper % 4, align=(tper // 4) % 4)
        ok[tper] = not bad
    log.check('symbol time: TPER 18 to 23 with no fraction meets every window, 17 (T0L 680 ns) and 24 (T1H 960 ns) '
              'do not', [t for t in ok if ok[t]] == list(range(18, 24)), str(ok))
    r, bad, m = ws_case(frames, tper=17, tfrac=128, tid=0, align=2)
    log.check('TPER 17 with TFRAC 128 (17.5 cycles, 350 ns a symbol) meets every window', not bad, str(bad[:2]))
    r, bad, m = ws_case(frames, host_gap=600, tid=3, align=0)
    log.check('a host 12 us late with every byte: the send underruns and the frame is cut, which the judge sees',
              bad != [], str(bad[:1]))


# ---------------------------------------------------------------------------------------------------------------
# PRBS and an additive scrambler: advanced/lfsr_unit.s

PRBS = {7: 6, 9: 5, 15: 14}     # w: m of the generator polynomial x^w + x^m + 1 (PRBS7; ITU-T O.150 2^9 - 1, 2^15 - 1)


def fib(w, m, n, seed=None):
    """A Fibonacci PRBS generator written from the polynomial x^w + x^m + 1, as O.150 draws it: a w-stage shift
    register whose stages m and w are XORed and fed back, so bit k is the XOR of bits k - m and k - w. Starts from
    the w bits of seed (all ones by default)."""
    b = list(seed) if seed is not None else [1] * w
    while len(b) < n:
        b.append(b[-m] ^ b[-w])
    return b[:n]


def prbs_errors(bits, w, inv=0):
    """A PRBS checker: it loads its register with the first w bits and predicts every later bit by the generator's
    recurrence, for the sequence or (inv) its complement; returns the bits it got wrong."""
    m = PRBS[w]
    b = [x ^ inv for x in bits]
    return sum(1 for k in range(w, len(b)) if b[k] != b[k - m] ^ b[k - w])


def line_bits(ch, bit, n, skip=8):
    """n bits of a line from its changes: the first fall from the idle 1 starts the first bit, each bit is read in its
    middle; the first skip bits (the sync byte) are left out."""
    f = next(c for c, v in ch if v == 0)

    def at(t):
        v = 1
        for c, x in ch:
            if c > t:
                break
            v = x
        return v
    return [at(f + k * bit + bit // 2) for k in range(skip, skip + n)], f


def lf_run(w=7, bit=4, frames=(), endless=0, inv=0, rx=None, ttx=0, trx=1, align=0, text=None, tail=200):
    """frames: lists of words to scramble (a frame of data), or an int K for K zero words (the PRBS). endless: run the
    endless PRBS for that many cycles instead. rx: None, 'raw' or 'desc' (or a CODING value). Returns the line, the
    received words and the statuses."""
    img = assemble(text or read(LF_PROG), defines=dict(W=w, INV=inv), depth=128)
    chip = Chip(img.words, 128)
    while chip.c % 4 != align:
        chip.cycle(_Idle())
    board = UartBoard()
    chip.sim.host_setpc(ttx, img.labels['tx'])
    mask = 1 << ttx
    posts = []
    nwords = 0
    if rx is not None:
        chip.sim.host_setpc(trx, img.labels['rx'])
        mask |= 1 << trx
        k0 = frames[0] if isinstance(frames[0], int) else len(frames[0])
        coding = {'raw': 0, 'desc': 0x0120 | (w - 1)}.get(rx, rx)
        posts += [(6, bit), (6, coding), (6, k0)]
    chip.sim.host_run(0, mask)
    posts.append((7, bit))
    if endless:
        posts.append((0, 0x4000 | 1))
    for fr in frames:
        if isinstance(fr, int):
            posts.append((0, fr))
            nwords += fr
        else:
            posts.append((0, 0x8000 | len(fr)))
            posts += [(0, x) for x in fr]
            nwords += len(fr)
    got, sts = [], []
    t_run = chip.c
    per = max(bit, 1)
    limit = chip.c + 1000 + (endless or 0) + sum(per * (8 + 16 * (fr if isinstance(fr, int) else len(fr))) + 600
                                                 for fr in frames)
    while chip.c < limit:
        if posts and (posts[0][0] != 0 or chip.c >= t_run + 400) and chip.post(posts[0][0], posts[0][1]):
            posts.pop(0)
        chip.cycle(board)
        v = chip.take(1)
        if v is not None:
            got.append(v)
        v = chip.take(2)
        if v is not None:
            sts.append(v)
        if not endless and not posts and len(sts) == len(frames) and (rx is None or len(got) >= nwords):
            break
    for _ in range(tail):
        chip.cycle(board)
    r = OwResult()
    r.chip, r.board, r.got, r.sts, r.frames, r.bit, r.w, r.inv = chip, board, got, sts, list(frames), per, w, inv
    r.late = [t.id for t in chip.t if t.late or t.th.ILL]
    return r


def words_bits(words):
    return [(x >> i) & 1 for x in words for i in range(16)]


def lf_judge(r, rx=None):
    """PRBS frames: the bits after the sync byte pass the checker of the named polynomial with no error, in the sense
    INV says. Data frames: the line XOR the data passes that checker (an additive scrambler adds the PRBS) and the
    receiver gives back the data. Raw reception: the received words are the line's bits."""
    bad = []
    if r.late:
        bad.append('LATE or ILL on threads %s' % r.late)
    if any(st & 0x80 for st in r.sts) or len(r.sts) != len(r.frames):
        bad.append('TX statuses %s' % ['%04x' % x for x in r.sts])
    ch = r.board.tx_ch
    rxw = list(r.got)
    for k, fr in enumerate(r.frames):
        n = 16 * (fr if isinstance(fr, int) else len(fr))
        bits, f = line_bits(ch, r.bit, n)
        ch = [(c, v) for c, v in ch if c > f + (8 + n) * r.bit]
        if isinstance(fr, int):
            e = prbs_errors(bits, r.w, r.inv)
            if e or len(set(bits)) < 2:
                bad.append('frame %d: the checker finds %d errors in %d bits' % (k, e, n))
            if rx == 'raw' and words_bits(rxw[:fr]) != [b ^ r.inv for b in bits] and words_bits(rxw[:fr]) != bits:
                bad.append('frame %d: the raw words are not the line' % k)
            rxw = rxw[fr:]
        else:
            data = words_bits(fr)
            e = prbs_errors([a ^ b for a, b in zip(bits, data)], r.w)
            if e:
                bad.append('frame %d: line XOR data has %d checker errors' % (k, e))
            if bits == data:
                bad.append('frame %d: sent in the clear' % k)
            if rx not in (None, 'raw') and rxw[:len(fr)] != list(fr):
                bad.append('frame %d: descrambled %s, sent %s' % (k, rxw[:3], list(fr[:3])))
            rxw = rxw[len(fr):]
    return bad


def lf_case(rx=None, **kw):
    r = lf_run(rx=rx, **kw)
    return r, lf_judge(r, rx)


def check_lf(log):
    log.group(LF_PROG, title='PRBS7, PRBS9 and PRBS15 from the LFSR (CMODE 2), and an additive scrambler between two units')
    log.check('reference: the Fibonacci generators have periods 127, 511 and 32767 and no shorter one',
              all(fib(w, PRBS[w], 2 ** w - 1 + w)[2 ** w - 1:] == [1] * w and
                  fib(w, PRBS[w], 2 ** w - 1).count(1) == 2 ** (w - 1) for w in (7, 9, 15)))
    rng = random.Random(21)
    good, det, n = 0, [], 0
    for w in (7, 9):
        for k, (bit, ttx, trx) in enumerate(((4, 0, 1), (2, 2, 3), (10, 3, 0), (3, 1, 2))):
            r, bad = lf_case(w=w, bit=bit, frames=[64], rx='raw', ttx=ttx, trx=trx, align=k)
            n += 1
            good += not bad
            if bad:
                det.append((w, bit, bad[:1]))
    log.check('PRBS7 and PRBS9: 1024 bits after the sync byte, 0 checker errors, at 2, 3, 4 and 10 cycles a bit, '
              '4 thread pairs; the raw receiver gives the same bits', good == n, '%d of %d %s' % (good, n, det[:1]))
    for w in (7, 9, 15):
        r = lf_run(w=w, bit=0, endless=2 ** w + 3000, frames=())
        bits, f = line_bits(r.board.tx_ch, 1, 2 ** w - 1 + 2 * w)
        e = prbs_errors(bits, w)
        per = bits[2 ** w - 1:2 ** w - 1 + w] == bits[:w] and bits[:2 ** w - 1].count(1) == 2 ** (w - 1)
        log.check('PRBS%d endless at TPER 0, 50 Mbit/s: one whole period (%d bits) and more, 0 checker errors, '
                  'the period repeats, %d ones in it' % (w, 2 ** w - 1, 2 ** (w - 1)), not e and per and not r.late,
                  'errors %d' % e)
    r, bad = lf_case(w=9, bit=4, frames=[40], inv=1, ttx=1, trx=0)
    log.check('INV 1 (output code 1): the complement passes the checker of the other sense', not bad, str(bad[:1]))
    good, det, n = 0, [], 0
    for w in (7, 9, 15):
        for k, (bit, ttx, trx) in enumerate(((4, 0, 1), (2, 3, 2), (7, 1, 3))):
            m = (5, 1, 20)[k]           # the receiver takes one frame length per run
            fr = [[rng.getrandbits(16) for _ in range(m)] for _ in range(3)]
            r, bad = lf_case(w=w, bit=bit, frames=fr, rx='desc', ttx=ttx, trx=trx, align=k)
            n += 1
            good += not bad
            if bad:
                det.append((w, bit, bad[:1]))
    log.check('scrambler loopback, 3 frames of 5, 1 or 20 words, each with its sync byte: the line XOR the data is the '
              'PRBS, and the other unit descrambles the data, for w 7, 9 and 15', good == n,
              '%d of %d %s' % (good, n, det[:1]))
    fr = [[0x0000] * 4 + [0xFFFF] * 4]
    r, bad = lf_case(w=7, bit=4, frames=fr, rx=0x0120 | 8, ttx=0, trx=1)
    log.check('a descrambler of the wrong width (CW 8 against w 7) gives other words', bad != [], str(bad[:1]))


# ---------------------------------------------------------------------------------------------------------------
# The idle send half as a CRC engine: advanced/crc_unit.s

CRC32_CFG = (0, 0x04C1, 0x1DB7, 0x02BF, 3)     # TXMODE, POLYHI, POLYLO, CODING, FLAGS for CRC-32 (zlib)
CRC8_CFG = (0, 0x3100, 0, 0x0287, 1)           # CRC-8/MAXIM


def cr_run(bufs, cfg=CRC32_CFG, tid=0, align=0, depth=512, text=None):
    """Puts the buffers (byte strings) in memory after the code, two bytes a word, and asks for each one's CRC.
    Returns the results (value, status) and the cycles each took from its length's post to its last word."""
    img = assemble(text or read(CR_PROG), depth=depth)
    words = dict(img.words)
    addr, where = img.size + 4, []
    for b in bufs:
        where.append(addr)
        for k in range(0, len(b), 2):
            words[addr] = b[k] | ((b[k + 1] << 8) if k + 1 < len(b) else 0)
            addr += 1
    assert addr <= depth, 'buffers do not fit'
    chip = Chip(words, depth)
    while chip.c % 4 != align:
        chip.cycle(_Idle())
    chip.start(tid)
    posts = list(cfg)
    for a, b in zip(where, bufs):
        posts += [a, len(b)]
    out, res, t_len, times = [], [], None, []
    limit = chip.c + 3000 + sum(400 + 10 * len(b) for b in bufs)
    while chip.c < limit and len(res) < len(bufs):
        if posts and chip.post(3, posts[0]):
            if len(posts) % 2 == 1 and len(posts) <= 2 * len(bufs):
                t_len = chip.c
            posts.pop(0)
        chip.cycle(_Idle())
        v = chip.take(5)
        if v is not None:
            out.append(v)
            if len(out) == 3:
                res.append(((out[0] << 16) | out[1], out[2]))
                times.append(chip.c - (t_len or 0))
                out = []
    r = OwResult()
    r.chip, r.res, r.times, r.bufs = chip, res, times, bufs
    r.late = [t.id for t in chip.t if t.late or t.th.ILL]
    return r


def check_cr(log):
    log.group(CR_PROG, title='the idle send half as a CRC engine: srd hi and lo after words of kind crc at TPER 0')
    log.check('reference: zlib.crc32 of "123456789" is 0xCBF43926', zlib.crc32(b'123456789') == 0xCBF43926)
    rng = random.Random(61)
    bufs = [b'123456789', b'', b'a', b'ab'] + [bytes(rng.getrandbits(8) for _ in range(n)) for n in
                                               (3, 4, 5, 16, 17, 31, 32, 33, 100, 255, 256)]
    bufs = [b for b in bufs if b]
    good, det = 0, []
    for tid in range(4):
        r = cr_run(bufs, tid=tid, align=(tid + 2) % 4)
        want = [(zlib.crc32(b), 0) for b in bufs]
        got = [(v, st & 0x80) for v, st in r.res]
        ok = got == want and not r.late
        good += ok
        if not ok:
            det.append([(hex(a), hex(b)) for (a, _), (b, _) in zip(got, want) if a != b][:2] or r.late)
    log.check('CRC-32 of "123456789" (0xCBF43926) and of 14 buffers of 1 to 256 bytes against zlib.crc32, no underrun, '
              'on 4 threads', good == 4, '%d of 4 %s' % (good, det[:1]))
    r = cr_run(bufs[:1])
    log.check('the result for "123456789" is 0xCBF43926', r.res and r.res[0][0] == 0xCBF43926,
              hex(r.res[0][0]) if r.res else 'none')
    per = [(t, len(b)) for t, b in zip(cr_run(bufs[-2:]).times, bufs[-2:])]
    log.info('CRC-32 engine: %s cycles for %s bytes, from the length\'s post to the last result word' % (
        [t for t, _ in per], [n for _, n in per]))
    rom = AN27_ROM
    r = cr_run([b'123456789', bytes(rom), bytes(rom[:7])], cfg=CRC8_CFG, tid=3, align=1, depth=128)
    got = [v for v, _ in r.res]
    log.check('CRC-8/MAXIM on a 128-word memory: 0xA1 for "123456789", 0 for AN27\'s ROM code with its CRC byte, '
              '0xA2 for its first 7 bytes', got == [0xA1, 0, 0xA2], [hex(x) for x in got])


# ---------------------------------------------------------------------------------------------------------------
# XY2-100 on three units started at one cycle: advanced/xy2_unit.s

class XyLog:
    """Logs the changes of CLK uo4, SYNC uo5, X uo6 and Y uo7: (cycle, uo bits 7:4)."""

    def __init__(self):
        self.v, self.ch = None, []

    def step(self, chip, c, uo, en, val):
        v = (uo >> 4) & 15
        if v != self.v:
            self.ch.append((c, v))
            self.v = v
        return 0xFF, 0xFF


def xy_frames(ch):
    """The XY2-100 frames on the four lines, read as the interface defines them: a bit at each falling edge of CLK
    (SYNC, X and Y sampled there), a frame ending at the bit whose SYNC is 0. Returns [(x bits, y bits)] of the
    frames, each 20 bits, and the errors: a frame of another length, or SYNC low anywhere but the last bit."""
    frames, xs, ys, bad = [], [], [], []
    prev = None
    for c, v in ch:
        if prev is not None and prev & 1 and not v & 1:          # CLK falls
            xs.append((v >> 2) & 1)
            ys.append((v >> 3) & 1)
            if not (v >> 1) & 1:
                if len(xs) != 20:
                    bad.append('frame of %d bits at %d' % (len(xs), c))
                frames.append((xs, ys))
                xs, ys = [], []
        prev = v
    if xs:
        bad.append('%d bits after the last frame' % len(xs))
    return frames, bad


def xy_word(pos):
    """The 20 bits of a 16-bit position frame: control 0 0 1, the position MSB first, even parity."""
    b = [0, 0, 1] + [(pos >> (15 - i)) & 1 for i in range(16)]
    return b + [sum(b) & 1]


def xy_run(xs, ys, threads=(0, 1, 2), align=0, text=None, lead=3000):
    img = assemble(text or read(XY_PROG), depth=128)
    chip = Chip(img.words, 128)
    while chip.c % 4 != align:
        chip.cycle(_Idle())
    log = XyLog()
    tc, tx, ty = threads
    for t, lab in ((tc, 'clk'), (tx, 'xax'), (ty, 'yax')):
        chip.sim.host_setpc(t, img.labels[lab])
    chip.sim.host_run(0, (1 << tc) | (1 << tx) | (1 << ty))
    t0 = (chip.c + lead) & 0xFFFF
    n = len(xs)
    posts = {4: [t0, n], 5: [t0, n] + list(xs), 6: [t0, n] + list(ys)}
    limit = chip.c + lead + 500 * n + 2000
    while chip.c < limit:
        for k in (4, 5, 6):
            if posts[k] and chip.post(k, posts[k][0]):
                posts[k].pop(0)
        chip.cycle(log)
        if chip.c > chip.c - 1 and all(not chip.t[t].run for t in threads) and not any(posts.values()):
            break
    r = OwResult()
    r.chip, r.log, r.xs, r.ys, r.t0 = chip, log, list(xs), list(ys), t0
    r.late = [t.id for t in chip.t if t.late or t.th.ILL]
    return r


def xy_judge(r):
    """Every frame as the interface defines it, X and Y positions as sent with even parity; every change of SYNC, X
    and Y in a cycle where CLK rises (one grid); CLK rising every 25 cycles, the first at T0 + 1."""
    bad = []
    if r.late:
        bad.append('LATE or ILL on threads %s' % r.late)
    fr, e = xy_frames(r.log.ch)
    bad += e
    want = [(xy_word(x), xy_word(y)) for x, y in zip(r.xs, r.ys)]
    if fr != want:
        bad.append('%d frames, want %d; first %s' % (len(fr), len(want), fr[:1]))
    ch = [(c, v) for c, v in r.log.ch if c > r.t0]     # from T0 on; before it the threads commit idle levels
    prev0 = r.log.ch[len(r.log.ch) - len(ch) - 1][1] if len(ch) < len(r.log.ch) else 0
    prev, rises = prev0, []
    for c, v in ch:
        if v & 1 and not prev & 1:
            rises.append(c)
        prev = v
    end = rises[-1] + 24 if rises else 0                # the last bit's last cycle; then the idle levels
    prev = prev0
    for c, v in ch:
        if c <= end and (v ^ prev) & 0xE and c not in rises:
            bad.append('SYNC, X or Y moved at %d, off a CLK rising edge' % c)
            break
        prev = v
    gaps = {b - a for a, b in zip(rises, rises[1:])}
    if rises and ((rises[0] - 1 - r.t0) % 65536 != 0 or not gaps <= {25}):
        bad.append('CLK rises from %d (T0 + 1 is %d), gaps %s' % (rises[0], r.t0 + 1, sorted(gaps)))
    return bad


def xy_case(xs, ys, **kw):
    r = xy_run(xs, ys, **kw)
    return r, xy_judge(r)


def check_xy(log):
    log.group(XY_PROG, title='XY2-100 on three units started at one cycle by wrd and tick 0, even parity appended')
    rng = random.Random(71)
    good, det = 0, []
    for k, th in enumerate(((0, 1, 2), (1, 2, 3), (2, 3, 0), (3, 0, 1), (3, 2, 1), (1, 3, 0), (2, 0, 3), (0, 3, 2))):
        xs = [rng.getrandbits(16) for _ in range(6)]
        ys = [rng.getrandbits(16) for _ in range(6)]
        r, bad = xy_case(xs, ys, threads=th, align=k % 4)
        good += not bad
        if bad:
            det.append((th, bad[:1]))
    log.check('6 frames back to back, 8 thread placements and alignments: every frame read at the falling edges, '
              'positions and even parity right, SYNC low on the parity bit only, SYNC, X and Y moving only on CLK\'s '
              'rising edges, CLK every 25 cycles (2 Mbit/s) from T0 + 1', good == 8, '%d of 8 %s' % (good, det[:1]))
    r, bad = xy_case([0x0000, 0xFFFF, 0x8001, 0x7FFE], [0xFFFF, 0x0000, 0x5555, 0xAAAA], threads=(1, 0, 3))
    log.check('positions 0, 0xFFFF, 0x8001, 0x7FFE against 0xFFFF, 0, 0x5555, 0xAAAA', not bad, str(bad[:1]))
    r, bad = xy_case([0x1234], [0x4321], threads=(2, 1, 0), align=3)
    log.check('a single frame', not bad, str(bad[:1]))
    r, bad = xy_case([1, 2, 3], [4, 5, 6], threads=(0, 1, 2), lead=40)
    log.check('a start cycle the threads reach late (T0 40 cycles ahead) breaks the grid, and the judge sees it',
              bad != [], str(bad[:1]))


# words (section 11.2) and the memory each bench run uses
WORDS = {'advanced/ow_unit.s': (108, 128), 'advanced/uart_par_unit.s': (77, 128), 'advanced/ws2812_unit.s': (44, 128),
         'advanced/lfsr_unit.s': (110, 128), 'advanced/crc_unit.s': (85, 128), 'advanced/xy2_unit.s': (108, 128)}


def check_words(log):
    log.group(*PROGRAMS, title='word counts')
    for rel, (want, depth) in WORDS.items():
        try:
            got, err = assemble(read(rel), depth=depth).size, ''
        except Exception as e:      # an assembler error is a failed check, not a crash
            got, err = None, str(e)
        check('%s assembles to %d words, the count in section 11.2, and fits %d words' % (rel, want, depth),
              got == want, err or '%d words' % got, progs=(rel,))


def main(log):
    check_words(log)
    check_ow(log)
    check_ow_limits(log)
    check_up(log)
    check_up_limits(log)
    check_ws(log)
    check_lf(log)
    check_cr(log)
    check_xy(log)
