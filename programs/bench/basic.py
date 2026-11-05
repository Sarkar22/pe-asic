"""Benches for the programs in basic/: UART, SPI, I2C write, WS2812, 1-Wire, and a memory load.

Each program runs on sim/sim.py against device models that see only pad levels, and is checked from
its pin trace by decoders that know only the protocol's rules and parameters.
"""
import random
import re

import rig
from report import Chip, assemble
from report import src as read
from rig import (Board, Host, I2cTarget, OneWireTarget, SpiTarget, UartSender, i2c_decode,
                 onewire_decode, spi_decode, uart_decode, ws2812_decode)

NAMES = ('uart_tx', 'uart_rx', 'spi', 'i2c_write', 'ws2812', 'onewire', 'memhog')
PROGRAMS = tuple('basic/%s.s' % n for n in NAMES)
CLK = rig.CLK


def src(name):
    return read('basic/%s.s' % name)


def P(*names):
    return tuple('basic/%s.s' % n for n in names)


class Run:
    def __init__(self, depth=128, devices=(), pullups=0, rise=0, seed=1):
        self.chip = Chip(depth, seed)
        self.board = Board(self.chip, devices, pullups, rise)
        self.board.run(8)

    def load(self, img, thread):
        for a, w in sorted(img.words.items()):
            if not self.chip.host_wrmem(a, w):
                raise RuntimeError('memory write refused')
        self.chip.host_setpc(thread, min(img.words))

    def start(self, mask, align=0):
        """RUN lands in a cycle h with h mod 4 = align."""
        while (self.chip.c - 1) % 4 != align:
            self.board.run(1)
        self.chip.host_run(0, mask)

    def wait_done(self, limit, margin=0):
        b = self.board
        b.run(limit, stop=lambda: all(a[0] is None for a in b.agents))
        if margin:
            b.run(margin)

    def health(self):
        out = []
        for t in self.chip.th:
            if t.LATE:
                out.append('thread %d LATE at cycle %s' % (t.TID, t.late_at))
            if t.ILL:
                out.append('thread %d ILL at PC %d' % (t.TID, t.PC))
        out.extend('bus conflict at %d' % c for c, m in self.board.conflicts[:3])
        return out


# ---- UART ------------------------------------------------------------------

def uart_tx_case(text, baud, thread=0, align=0, data=(0x55, 0xAA, 0x00, 0xFF, 0x31, 0x80)):
    img = assemble(text, depth=128)
    r = Run()
    r.load(img, thread)
    host = Host(r.chip)
    bit = round(CLK / baud)

    def script():
        yield from host.post(7, bit)
        for b in data:
            yield from host.post(0, b)
    r.board.agent(script())
    r.start(1 << thread, align)
    r.wait_done(int(bit * 11 * len(data)) + 2000, int(bit * 24))
    got, errs = uart_decode(r.board.trace.wave('uo', 0), baud)
    return got == list(data) and not errs and not r.health(), got, errs + r.health()


def uart_rx_case(text, baud, phase, data, thread=0, frames=None, err=0.0, gap=0.0, cfg=None):
    """The sender runs at baud*(1+err). The firmware is told the nominal bit time.
    frames overrides data and may hold faults."""
    bit_nom = CLK / baud
    bit = bit_nom / (1 + err)
    frames = list(data) if frames is None else frames
    snd = UartSender(3, frames, bit, 300 + phase, gap=gap)
    img = assemble(text, depth=128)
    r = Run(devices=[snd])
    r.load(img, thread)
    host = Host(r.chip)
    got = []

    def script():
        yield from host.post(7, cfg if cfg is not None else round(bit_nom))
        while True:
            yield from host.take(4, got, tries=1)
    r.board.agent(script())
    r.start(1 << thread)
    r.board.run(snd.last + 3000)
    return [w for _, w in got], r, snd


def check_uart(log):
    log.group(*P('uart_tx'), title='UART transmitter')
    t = src('uart_tx')
    for baud in (115200, 1_000_000):
        good = runs = 0
        for th in range(4):
            for al in range(4):
                ok, got, e = uart_tx_case(t, baud, th, al)
                runs += 1
                good += ok
                if not ok:
                    log.line('   thread %d align %d: %s %s' % (th, al, got, e[:2]))
        log.check('TX %d baud, 4 threads x 4 alignments, 6 bytes each' % baud, good == runs,
                  '%d of %d' % (good, runs))
    ok, got, e = uart_tx_case(t, 50e6 / 12)
    log.check('TX 12 cycles per bit, 4.17 Mbaud', ok, str(e[:3]) if e else '')
    ok, got, e = uart_tx_case(t, 9600, data=(0x41, 0x7E))
    log.check('TX 9600 baud', ok, str(e[:3]) if e else '')
    ok, got, e = uart_tx_case(t, CLK / 32767, data=(0x5A,))
    log.check('TX 32767 cycles per bit, the top of the range in the example', ok, str(e[:3]) if e else '')

    log.group(*P('uart_rx'), title='UART receiver')
    t = src('uart_rx')
    data = [0x55, 0xAA, 0x00, 0xFF, 0x31, 0x80, 0x0F, 0xF0]
    long_data = [(i * 37 + 11) & 255 for i in range(64)]
    for baud in (115200, 1_000_000):
        good = 0
        for ph in range(8):
            got, r, snd = uart_rx_case(t, baud, ph, data)
            truth, errs = uart_decode(r.board.trace.wave('ui', 3), baud)
            if got == data and truth == data and not errs and not r.health():
                good += 1
            else:
                log.line('   baud %d phase %d got %s health %s' % (baud, ph, got, r.health()))
        log.check('RX %d baud, abutting frames, 8 phases' % baud, good == 8, '%d of 8' % good)
    for baud in (115200, 1_000_000):
        for err in (-0.02, 0.02):
            got, r, snd = uart_rx_case(t, baud, 3, long_data, err=err)
            log.check('RX %d baud, sender off by %+.0f%%, 64 abutting bytes' % (baud, err * 100),
                      got == long_data and not r.health(), '' if got == long_data else str(got[:12]))
    got, r, snd = uart_rx_case(t, 115200, 0, None,
                               frames=[('glitch', 10), 0x5A, ('badstop', 0xC3), ('gap', 5000), 0x3C])
    log.check('RX glitch ignored, framing error flagged, next byte clean',
              got == [0x5A, 0xC3, 0x100, 0x3C], str([hex(x) for x in got]))
# ---- SPI -------------------------------------------------------------------

def spi_case(text, mode, half, thread=0, align=0, frames=((0xA5,), (0x3C, 0xFF, 0x00)), tco=2, seed=7):
    cpol, cpha = mode >> 1, mode & 1
    rng = random.Random(seed + mode)
    nbytes = sum(len(f) for f in frames)
    replies = [rng.getrandbits(8) for _ in range(nbytes)]
    tgt = SpiTarget(1, 2, 5, 4, cpol, cpha, replies, tco)
    img = assemble(text, depth=128, defines=dict(CPOL=cpol, CPHA=cpha))
    r = Run(devices=[tgt])
    r.load(img, thread)
    host = Host(r.chip)
    got = []

    def script():
        yield from host.post(7, half)
        for f in frames:
            for i, b in enumerate(f):
                yield from host.post(1, b | (0x100 if i == len(f) - 1 else 0))
                yield from host.take(5, got)
    r.board.agent(script())
    r.start(1 << thread, align)
    r.wait_done(200000 + nbytes * half * 60, half * 8)
    tr = r.board.trace
    fr, errs = spi_decode(tr.wave('uo', 1), tr.wave('uo', 2), tr.wave('ui', 4), tr.wave('uo', 5),
                          cpol, cpha, tsu=half // 2, thd=half // 2)
    want_m = [list(f) for f in frames]
    p = []
    k = 0
    if [x for x, _ in fr] != want_m:
        p.append('MOSI on the pins %s, wanted %s' % ([x for x, _ in fr], want_m))
    if tgt.frames != want_m:
        p.append('target saw %s' % tgt.frames)
    i = 0
    want_i = []
    for f in frames:
        want_i.append(replies[i:i + len(f)])
        i += len(f)
    if [y for _, y in fr] != want_i:
        p.append('MISO on the pins %s, wanted %s' % ([y for _, y in fr], want_i))
    if [w & 255 for _, w in got] != replies:
        p.append('firmware read %s, wanted %s' % ([w & 255 for _, w in got], replies))
    p += errs[:3] + r.health()
    return not p, p


def check_spi(log):
    log.group(*P('spi'), title='SPI controller')
    t = src('spi')
    for half in (9, 25, 250):
        good = 0
        runs = 0
        for mode in range(4):
            for th in range(4):
                for al in range(4):
                    runs += 1
                    ok, p = spi_case(t, mode, half, th, al)
                    good += ok
                    if not ok and runs - good < 3:
                        log.line('   mode %d half %d thread %d align %d: %s' % (mode, half, th, al, p[:2]))
        log.check('half period %d, modes 0-3 x 4 threads x 4 alignments' % half, good == runs,
                  '%d of %d, %.2f MHz' % (good, runs, CLK / 2e6 / half))
# ---- I2C -------------------------------------------------------------------

I2C_NS = {'Sm': (400, 4600, 5000), 'Fm': (400, 1040, 1000)}     # SCL fall to SDA, rest of low, high


def i2c_cfg(mode):
    return tuple(round(x * CLK / 1e9) for x in I2C_NS[mode])


def i2c_case(text, mode, txs, rise=0, target=None, thread=2, align=0, addr=0x50):
    """txs: list of byte lists, first byte is the address byte."""
    kw = dict(target or {})
    tgt = I2cTarget(2, 1, addr, **kw)
    img = assemble(text, depth=128)
    r = Run(devices=[tgt], pullups=0b0110, rise=rise)
    r.load(img, thread)
    host = Host(r.chip)
    status = []

    def script():
        for w in i2c_cfg(mode):
            yield from host.post(6, w)
        for tx in txs:
            for i, b in enumerate(tx):
                w = b | (0x100 if i == 0 else 0) | (0x200 if i == len(tx) - 1 else 0)
                yield from host.post(2, w)
                yield from host.take(4, status)
                if status[-1][1] != 0:
                    break
            yield from host.delay(2000)
    r.board.agent(script())
    r.start(1 << thread, align)
    r.wait_done(3_000_000)
    tr = r.board.trace
    scl, sda = tr.wave('io', 2), tr.wave('io', 1)
    dec, errs, m = i2c_decode(scl, sda, mode)
    p = list(errs[:4]) + r.health()
    for pin in (1, 2):
        if tr.drive_high(pin).v != [0]:
            p.append('chip drove io%d high' % pin)
    return dec, [w for _, w in status], tgt, m, p


def check_i2c(log):
    log.group(*P('i2c_write'), title='I2C controller, write only')
    t = src('i2c_write')
    write = [0xA0, 0x10, 0x5A, 0xC3]        # target 0x50, write
    for mode in ('Sm', 'Fm'):
        for rise in (0, 50 if mode == 'Sm' else 15):
            dec, st, tgt, m, p = i2c_case(t, mode, [write, write[:2]], rise=rise)
            ok = ([d['bytes'] for d in dec] == [write, write[:2]] and all(d['stop'] for d in dec)
                  and all(all(d['acks']) for d in dec) and st == [0, 0, 0, 2, 0, 2]
                  and tgt.tx == [write, write[:2]] and not p)
            log.check('%s write, target ACKs, bus rise %d cycles' % (mode, rise), ok,
                      (str(p[:3]) + ' ' + str(st)) if not ok else _i2c_meas(m))
        good = 0
        for th in range(4):
            for al in range(4):
                dec, st, tgt, m, p = i2c_case(t, mode, [write], thread=th, align=al)
                good += [d['bytes'] for d in dec] == [write] and st == [0, 0, 0, 2] and not p
        log.check('%s write, 4 threads x 4 alignments' % mode, good == 16, '%d of 16' % good)
        # every clock stretched
        dec, st, tgt, m, p = i2c_case(t, mode, [write],
                                      target=dict(stretch=lambda byte, b: 1000 + 37 * (b + byte)))
        ok = [d['bytes'] for d in dec] == [write] and st == [0, 0, 0, 2] and tgt.stretched >= 30 and not p
        log.check('%s target stretches every clock' % mode, ok,
                  'stretched %d clocks, longest low %d cycles' % (tgt.stretched, m['t_low'][1]))
        # the stretch only on the ACK clocks, the usual case
        dec, st, tgt, m, p = i2c_case(t, mode, [write],
                                      target=dict(stretch=lambda byte, b: 5000 if b == 8 else 0))
        ok = [d['bytes'] for d in dec] == [write] and st == [0, 0, 0, 2] and not p
        log.check('%s target stretches after each ACK' % mode, ok, str(p[:2]) if p else '')
        # NACK on request
        dec, st, tgt, m, p = i2c_case(t, mode, [write], target=dict(nack={2}))
        ok = ([d['bytes'] for d in dec] == [write[:3]] and dec[0]['acks'] == [1, 1, 0] and dec[0]['stop']
              and st == [0, 0, 1] and not p)
        log.check('%s target NACKs the second data byte' % mode, ok, str((st, p[:2])))
        dec, st, tgt, m, p = i2c_case(t, mode, [write], addr=0x51)
        ok = ([d['bytes'] for d in dec] == [write[:1]] and dec[0]['acks'] == [0] and dec[0]['stop']
              and st == [1] and not p)
        log.check('%s wrong address gets no ACK' % mode, ok, str((st, p[:2])))
        # back-to-back transactions, measured against tBUF
        dec, st, tgt, m, p = i2c_case(t, mode, [write[:2]] * 3)
        log.check('%s three transactions, tBUF' % mode, len(dec) == 3 and not p,
                  'tBUF min %s cycles' % (m.get('t_buf', ['-'])[0]))


def _i2c_meas(m):
    return ' '.join('%s=%s' % (k, m[k][0]) for k in ('t_low', 't_high', 't_su_dat', 't_hd_dat',
                                                     't_hd_sta', 't_su_sto', 'period') if k in m)


# ---- WS2812 ----------------------------------------------------------------

def ws_case(text, frames, thread=3, align=0):
    img = assemble(text, depth=128)
    r = Run()
    r.load(img, thread)
    host = Host(r.chip)

    def script():
        for fr in frames:
            yield from host.post(3, len(fr))
            for b in fr:
                yield from host.post(3, b)
            yield from host.delay(2000 + 62 * 8 * len(fr))
    r.board.agent(script())
    r.start(1 << thread, align)
    r.wait_done(2_000_000, 20000)
    w = r.board.trace.wave('uo', 6)
    dec, errs, m = ws2812_decode(w)
    want = [[(b >> (7 - i)) & 1 for b in fr for i in range(8)] for fr in frames]
    return dec == want, errs + r.health(), m, dec


def check_ws(log):
    log.group(*P('ws2812'), title='WS2812 driver')
    t = src('ws2812')
    frames = [[0x12, 0x34, 0x56, 0xFF, 0x00, 0xA5, 0x80, 0x01, 0x7E]]
    good = 0
    for th in range(4):
        for al in range(4):
            ok, e, m, dec = ws_case(t, frames, th, al)
            good += ok and not e
    log.check('3 LEDs, 72 bits, 4 threads x 4 alignments', good == 16, '%d of 16' % good)
    ok, e, m, dec = ws_case(t, [[0xF0, 0x0F, 0x55], [0xAA, 0x01, 0x80]])
    log.check('two frames with a latch gap', ok and not e, str(e[:2]))
    log.line('INFO  cycles: T0H %s T1H %s T0L %s T1L %s period %s' % (
        sorted(m['t0h']), sorted(m['t1h']), sorted(m['t0l']), sorted(m['t1l']), sorted(m['period'])))


# ---- 1-Wire ----------------------------------------------------------------

def ow_case(text, data, present=True, sample_us=30, pd_wait=30, pd_low=120, thread=3, align=0):
    tgt = OneWireTarget(4, sample_us, pd_wait, pd_low, present)
    img = assemble(text, depth=128)
    r = Run(devices=[tgt], pullups=1 << 4)
    r.load(img, thread)
    host = Host(r.chip)
    pres = []

    def script():
        yield from host.post(3, 0x100)
        yield from host.take(5, pres)
        for b in data:
            yield from host.post(3, b)
        yield from host.delay(3600 * 8 * len(data) + 4000)
    r.board.agent(script())
    r.start(1 << thread, align)
    r.wait_done(1_000_000, 6000)
    tr = r.board.trace
    ev, errs, m = onewire_decode(tr.wave('io', 4), tr.pull_low(4))
    p = errs[:3] + r.health()
    if tr.drive_high(4).v != [0]:
        p.append('chip drove the bus high')
    return ev, [w for _, w in pres], tgt, m, p


def check_ow(log):
    log.group(*P('onewire'), title='1-Wire controller')
    t = src('onewire')
    data = [0xCC, 0x44, 0xA5, 0x00, 0xFF]
    ev, pres, tgt, m, p = ow_case(t, data)
    ok = ([e[0] for e in ev] == ['reset'] + ['byte'] * 5 and [e[2] for e in ev[1:]] == data
          and ev[0][2] and pres == [1] and tgt.bytes == data and tgt.resets == 1 and not p)
    log.check('reset, presence, write 5 bytes', ok, str(p[:3]) if p else '')
    log.line('INFO  cycles: tRSTL %s tRSTH %s tLOW1 %s tLOW0 %s tREC min %s tSLOT %s' % (
        m['t_rstl'][0], m['t_rsth'][0], sorted(set(m['t_low1'])), sorted(set(m['t_low0'])),
        min(m['t_rec']), sorted(set(m['t_slot']))))
    ev, pres, tgt, m, p = ow_case(t, [0x33], present=False)
    log.check('no device: reports no presence', pres == [0] and ev[0][2] is False and not p, str(pres))
    good = bad = 0
    for s in (15, 30, 60):
        for wait, low in ((15, 60), (30, 120), (60, 60), (60, 240)):
            ev, pres, tgt, m, p = ow_case(t, data[:2], sample_us=s, pd_wait=wait, pd_low=low)
            if pres == [1] and tgt.bytes == data[:2] and not p:
                good += 1
            else:
                bad += 1
                log.line('   sample %d presence %d+%d: %s %s %s' % (s, wait, low, pres, tgt.bytes, p[:2]))
    log.check('device timing corners: sample 15/30/60 us, presence wait and length', bad == 0,
              '%d of %d' % (good, good + bad))
    good = 0
    for th in range(4):
        for al in range(4):
            ev, pres, tgt, m, p = ow_case(t, [0x5A], thread=th, align=al)
            good += pres == [1] and tgt.bytes == [0x5A] and not p
    log.check('4 threads x 4 alignments', good == 16, '%d of 16' % good)


# ---- concurrency -----------------------------------------------------------

TDM = 1200                      # link windows of 200 cycles, none overlapping
WIN = {'tx': [0], 'spi': [400], 'i2c': [800], 'ws': [200, 600, 1000], 'ow': [200, 600, 1000],
       'rx': [200, 600, 1000], 'mem': []}
RX_BYTES = [0x52, 0x58, 0x00, 0xFF, 0x7E]


def conc_setup(names, rotate=0):
    """One image, one thread each. Returns the parts needed to run any subset of the
    programs with exactly the same placement and host schedule."""
    pieces = {}
    base = 0
    for i, name in enumerate(names):
        tid = (i + rotate) % 4
        mo = 4 + tid
        if name == 'tx':
            text, d = src('uart_tx'), dict(MCFG=tid, MIN=tid)
        elif name == 'rx':
            text, d = src('uart_rx'), dict(MCFG=tid, MOUT=mo)
        elif name == 'spi':
            text, d = src('spi'), dict(MCFG=tid, MIN=tid, MOUT=mo, CPOL=1, CPHA=1)
        elif name == 'i2c':
            text, d = src('i2c_write'), dict(MCFG=tid, MIN=tid, MOUT=mo)
        elif name == 'ws':
            text, d = src('ws2812'), dict(MIN=tid)
        elif name == 'ow':
            text, d = src('onewire'), dict(MIN=tid, MOUT=mo)
        elif name == 'mem':
            text, d = src('memhog'), dict(TABLE=480)
        img = assemble(text, defines=d, base=base, depth=512)
        pieces[name] = (img, tid)
        base = max(img.words) + 1
    return pieces, base


UO = {'tx': 1 << 0, 'spi': (1 << 1) | (1 << 2) | (1 << 5), 'ws': 1 << 6, 'rx': 0}
IO = {'i2c': (1 << 1) | (1 << 2), 'ow': 1 << 4}


def conc_run(pieces, active, depth, cycles=170_000):
    """Runs the programs named in active, with the others' code loaded but stopped."""
    devs = []
    spi_t = i2c_t = ow_t = None
    if 'spi' in active:
        spi_t = SpiTarget(1, 2, 5, 4, 1, 1, [0x96, 0x69, 0x3C, 0xC3, 0x11, 0x22], 2)
        devs.append(spi_t)
    if 'i2c' in active:
        i2c_t = I2cTarget(2, 1, 0x50, stretch=lambda byte, b: 600 if b == 8 else 0)
        devs.append(i2c_t)
    if 'ow' in active:
        ow_t = OneWireTarget(4)
        devs.append(ow_t)
    if 'rx' in active:
        devs.append(UartSender(3, RX_BYTES, CLK / 115200, 1500))
    pu = (0b0110 if 'i2c' in active else 0) | ((1 << 4) if 'ow' in active else 0)
    r = Run(depth=depth, devices=devs, pullups=pu)
    for name, (img, tid) in pieces.items():
        for a, w in sorted(img.words.items()):
            r.chip.host_wrmem(a, w)
        r.chip.host_setpc(tid, min(img.words))
    res = {}
    mask = 0
    for name in active:
        img, tid = pieces[name]
        mask |= 1 << tid
        h = Host(r.chip, tdm=(TDM, WIN[name]))
        if name == 'tx':
            def s(h=h, tid=tid):
                yield from h.post(tid, 434)
                for b in (0x4A, 0x53, 0x00, 0xFF):
                    yield from h.post(tid, b)
            r.board.agent(s())
        elif name == 'rx':
            got = res['rx'] = []

            def s(h=h, tid=tid, got=got):
                yield from h.post(tid, round(CLK / 115200))
                while True:
                    yield from h.take(4 + tid, got, tries=1)
            r.board.agent(s())
        elif name == 'spi':
            got = res['spi'] = []

            def s(h=h, tid=tid, got=got):
                yield from h.post(tid, 25)
                for b in (0x101, 0x02, 0x103, 0x104, 0x05, 0x106):
                    yield from h.post(tid, b)
                    yield from h.take(4 + tid, got)
            r.board.agent(s())
        elif name == 'i2c':
            got = res['i2c'] = []

            def s(h=h, tid=tid, got=got):
                for w in i2c_cfg('Sm'):
                    yield from h.post(tid, w)
                for rep in range(2):
                    tx = [0xA0, 0x10 + rep, 0x5A]
                    for i, b in enumerate(tx):
                        yield from h.post(tid, b | (0x100 if i == 0 else 0) | (0x200 if i == 2 else 0))
                        yield from h.take(4 + tid, got)
            r.board.agent(s())
        elif name == 'ws':
            def s(h=h, tid=tid):
                for rep in range(4):
                    fr = [0x10 + rep, 0x20, 0x30, 0x40, 0x50, 0x60]
                    yield from h.post(tid, len(fr))
                    for b in fr:
                        yield from h.post(tid, b)
                    yield from h.delay(20000)
            r.board.agent(s())
        elif name == 'ow':
            got = res['ow'] = []

            def s(h=h, tid=tid, got=got):
                yield from h.post(tid, 0x100)
                yield from h.take(4 + tid, got)
                for b in (0xCC, 0x44):
                    yield from h.post(tid, b)
            r.board.agent(s())
    r.start(mask)
    r.board.run(cycles)
    return r, res, dict(spi=spi_t, i2c=i2c_t, ow=ow_t)


def pin_mask(name):
    return UO.get(name, 0) | (IO.get(name, 0) << 8) | (IO.get(name, 0) << 16) | (IO.get(name, 0) << 32)


def conc_check(log, r, tag, active, targets, res):
    tr = r.board.trace
    h = r.health()
    log.check('%s: no LATE, ILL or bus conflict on any thread' % tag, not h, str(h[:3]))
    if 'tx' in active:
        got, errs = uart_decode(tr.wave('uo', 0), 115200)
        log.check('%s: UART decoded from the pad' % tag, got == [0x4A, 0x53, 0, 0xFF] and not errs,
                  str((got, errs[:2])))
    if 'rx' in active:
        truth, errs = uart_decode(tr.wave('ui', 3), 115200)
        got = [w for _, w in res['rx']]
        log.check('%s: UART RX read what the pin carried' % tag, got == RX_BYTES == truth and not errs,
                  str((got, truth, errs[:2])))
    if 'spi' in active:
        fr, errs = spi_decode(tr.wave('uo', 1), tr.wave('uo', 2), tr.wave('ui', 4), tr.wave('uo', 5),
                              1, 1, tsu=10, thd=10)
        want = [[1], [2, 3], [4], [5, 6]]
        log.check('%s: SPI mode 3 decoded from the pad' % tag,
                  [a for a, _ in fr] == want and targets['spi'].frames == want and not errs,
                  str(([a for a, _ in fr], errs[:2])))
    if 'i2c' in active:
        dec, errs, m = i2c_decode(tr.wave('io', 2), tr.wave('io', 1), 'Sm')
        want = [[0xA0, 0x10, 0x5A], [0xA0, 0x11, 0x5A]]
        log.check('%s: I2C decoded from the bus' % tag,
                  [d['bytes'] for d in dec] == want and not errs, str(([d['bytes'] for d in dec], errs[:2])))
    if 'ws' in active:
        dec, errs, m = ws2812_decode(tr.wave('uo', 6))
        ok = len(dec) == 4 and all(len(f) == 48 for f in dec) and not errs
        log.check('%s: WS2812 decoded from the pad' % tag, ok, str(([len(f) for f in dec], errs[:2])))
    if 'ow' in active:
        ev, errs, m = onewire_decode(tr.wave('io', 4), tr.pull_low(4))
        log.check('%s: 1-Wire decoded from the bus' % tag,
                  [e[2] for e in ev] == [True, 0xCC, 0x44] and not errs, str((ev, errs[:2])))


def check_conc(log):
    full = dict(tx='uart_tx', rx='uart_rx', spi='spi', i2c='i2c_write', ws='ws2812', ow='onewire', mem='memhog')
    for names, rotate, depth, tag in ((('tx', 'rx', 'spi', 'i2c'), 0, 128, '128-word image, UART RX'),
                                      (('tx', 'spi', 'i2c', 'ws'), 1, 128, '128-word image, WS2812'),
                                      (('tx', 'spi', 'i2c', 'ow'), 2, 128, '128-word image, 1-Wire'),
                                      (('tx', 'i2c', 'ow', 'mem'), 3, 512, '512 words with a memory hog')):
        log.group(*P(*[full[n] for n in names]), title='%s, one thread each' % tag)
        pieces, used = conc_setup(names, rotate)
        log.line('INFO  %s: %s, %d words of %d' % (
            tag, ', '.join('%s=%d on T%d' % (n, pieces[n][0].size, pieces[n][1]) for n in names),
            used, depth))
        if used > depth:
            log.check('%s fits' % tag, False, '%d words' % used)
            continue
        active = [n for n in names]
        r_all, res, targets = conc_run(pieces, active, depth)
        conc_check(log, r_all, tag, active, targets, res)
        for n in names:
            if n == 'mem':
                continue
            r_one, res_one, _ = conc_run(pieces, [n], depth)
            a = r_all.board.trace.projected(pin_mask(n))
            b = r_one.board.trace.projected(pin_mask(n))
            same = a == b and res.get(n) == res_one.get(n)
            log.check('%s: %s pins and host traffic match its solo run, cycle for cycle' % (tag, n), same,
                      '%d pin changes, %d host words, over %d cycles'
                      % (len(a), len(res.get(n, ())), r_all.board.trace.end) if same else _first_diff(a, b))


def _first_diff(a, b):
    for i, (x, y) in enumerate(zip(a, b)):
        if x != y:
            return 'first difference at change %d: %s vs %s' % (i, x, y)
    return 'lengths %d and %d' % (len(a), len(b))


# ---- seeded bugs -----------------------------------------------------------

def mutate(text, old, new):
    if old not in text:
        raise ValueError('seed not found: ' + old)
    return text.replace(old, new, 1)


def check_bugs(log):
    rx_data = [0x55, 0xAA, 0x00, 0xFF]
    log.group(*P('uart_tx'), title='the checks reject broken copies')
    t = src('uart_tx')
    ok, got, e = uart_tx_case(t, 115200, data=(0x55, 0xA3))
    log.check('control: the program as written passes', ok)
    ok, got, e = uart_tx_case(mutate(t, 'ldi   R1, 8', 'ldi   R1, 7'), 115200, data=(0x55, 0xA3))
    log.check('rejects a copy that sends 7 data bits', not ok, 'decoder saw %s, %s' % (got, e[:1]))
    ok, got, e = uart_tx_case(mutate(t, 'put   TX, 1         ; stop bit', 'put   TX, 0'), 115200, data=(0x55, 0xA3))
    log.check('rejects a copy with the stop bit low', not ok, 'decoder saw %s, %s' % (got, e[:1]))

    ok, got, e = uart_tx_case(mutate(t, 'recv  R2, MCFG      ; R2 = cycles per bit, sent by the host once',
                                     'recv  R2, MCFG\n        addi  R2, 13'), 115200, data=(0x55, 0xA3))
    log.check('rejects a copy 3% slow whose bytes still decode at mid-bit', not ok, 'decoder saw %s, %s' % (got, e[:1]))

    log.group(*P('uart_rx'), title='the checks reject broken copies')
    t = src('uart_rx')
    got, r, snd = uart_rx_case(mutate(t, 'shr16 R3, -, -      ; R3 = half a bit', 'idle'), 115200, 0, rx_data)
    log.check('rejects a copy that samples a full bit after the edge, not half', got != rx_data, str(got))

    log.group(*P('spi'), title='the checks reject broken copies')
    t = src('spi')
    bad = mutate(t, '.if CPOL ^ CPHA', '.if (CPOL ^ CPHA) ^ 1')
    log.check('rejects a copy with the edges swapped (wrong CPHA)', not spi_case(bad, 0, 25)[0], str(spi_case(bad, 0, 25)[1][:1]))
    bad = mutate(t, 'ldi   R1, 8', 'ldi   R1, 9')
    log.check('rejects a copy that clocks 9 bits per byte', not spi_case(bad, 0, 25)[0], str(spi_case(bad, 0, 25)[1][:1]))

    log.group(*P('i2c_write'), title='the checks reject broken copies')
    t = src('i2c_write')
    write = [0xA0, 0x10, 0x5A, 0xC3]
    dec, st, tgt, m, p = i2c_case(t, 'Sm', [write], target=dict(stretch=lambda b, n: 800))
    log.check('control: the program as written passes, with stretching', [d['bytes'] for d in dec] == [write] and not p)
    bad = mutate(t, '        until SCL, 1        ; a target may hold it low: this is the stretch\n',
                 '        tick  R4\n')
    dec, st, tgt, m, p = i2c_case(bad, 'Sm', [write], target=dict(stretch=lambda b, n: 800))
    log.check('rejects a copy that ignores clock stretching', [d['bytes'] for d in dec] != [write] or bool(p),
              'decoder: %s, %s' % ([d['bytes'] for d in dec], p[:1]))
    bad = mutate(t, '        jt    nack\n', '        idle\n')
    dec, st, tgt, m, p = i2c_case(bad, 'Sm', [write], target=dict(nack={2}))
    wrong = not ([d['bytes'] for d in dec] == [write[:3]] and st == [0, 0, 1])
    log.check('rejects a copy that ignores a NACK', wrong, 'status %s, decoder %s' % (st, [d['bytes'] for d in dec]))
    bad = mutate(t, 'pcfg  SDA, OD', 'pcfg  SDA, PP')
    dec, st, tgt, m, p = i2c_case(bad, 'Sm', [write])
    log.check('rejects a copy with SDA left push-pull', bool(p), str(p[:1]))

    log.group(*P('ws2812'), title='the checks reject broken copies')
    t = src('ws2812')
    fr = [[0x12, 0x34, 0x56]]
    ok, e, m, dec = ws_case(t, fr)
    log.check('control: the program as written passes', ok and not e)
    ok, e, m, dec = ws_case(mutate(t, '.equ T1H 35', '.equ T1H 44'), fr)
    log.check('rejects a copy with T1H 880 ns, above the 850 ns window', not (ok and not e), str(e[:1]))
    ok, e, m, dec = ws_case(mutate(t, '.equ T0H 18', '.equ T0H 8'), fr)
    log.check('rejects a copy with T0H 160 ns, below the 200 ns window', not (ok and not e), str(e[:1]))
    ok, e, m, dec = ws_case(mutate(t, '.equ TRES 15000', '.equ TRES 2000'), [[1, 2, 3], [4, 5, 6]])
    log.check('rejects a copy whose latch gap is only 40 us', not (ok and not e), str(e[:1]))

    log.group(*P('onewire'), title='the checks reject broken copies')
    t = src('onewire')
    ev, pres, tgt, m, p = ow_case(t, [0xA5])
    log.check('control: the program as written passes', pres == [1] and tgt.bytes == [0xA5] and not p)
    ev, pres, tgt, m, p = ow_case(mutate(t, '.equ TRSTL 24500', '.equ TRSTL 20000'), [0xA5])
    log.check('rejects a copy whose reset is only 400 us', pres != [1] or bool(p), str((pres, p[:1])))
    ev, pres, tgt, m, p = ow_case(mutate(t, '.equ TLOW0 3250', '.equ TLOW0 2000'), [0xA5])
    log.check('rejects a copy whose write-0 low is only 40 us', bool(p) or tgt.bytes != [0xA5], str(p[:1]))
    ev, pres, tgt, m, p = ow_case(mutate(t, 'pcfg  OW, OD', 'pcfg  OW, PP'), [0xA5])
    log.check('rejects a copy with the bus left push-pull', bool(p), str(p[:1]))


# ---- other clocks ----------------------------------------------------------

def scaled(text, mhz, names):
    """Rescales .equ constants given at 50 MHz. Only constants change, as section 7.3 says."""
    for n in names:
        m = re.search(r'^\.equ %s (\d+)' % n, text, re.M)
        text = text.replace(m.group(0), '.equ %s %d' % (n, round(int(m.group(1)) * mhz / 50)))
    return text


def check_clk(log):
    global CLK
    log.group(*P('uart_tx', 'uart_rx', 'spi', 'i2c_write', 'ws2812', 'onewire'), title='other clock rates, firmware constants only')
    write = [0xA0, 0x10, 0x5A, 0xC3]
    for mhz in (40, 45):
        rig.CLK = CLK = mhz * 1_000_000
        p = []
        for baud in (115200, 1_000_000):
            ok, got, e = uart_tx_case(src('uart_tx'), baud)
            p += [] if ok else ['TX %d: %s' % (baud, e[:2])]
            data = [0x55, 0xAA, 0x00, 0xFF]
            got, r, snd = uart_rx_case(src('uart_rx'), baud, 3, data)
            p += [] if got == data and not r.health() else ['RX %d: %s' % (baud, got)]
        for mode in range(4):
            ok, e = spi_case(src('spi'), mode, 25)
            p += [] if ok else ['SPI %d: %s' % (mode, e[:2])]
        for m in ('Sm', 'Fm'):
            dec, st, tgt, _, e = i2c_case(src('i2c_write'), m, [write])
            p += [] if [d['bytes'] for d in dec] == [write] and not e else ['I2C %s: %s' % (m, e[:2])]
        ws = scaled(src('ws2812'), mhz, ('T0H', 'T1H', 'TBIT', 'TRES'))
        ok, e, m_, dec = ws_case(ws, [[0x12, 0x34, 0x56]])
        p += [] if ok and not e else ['WS2812: %s' % e[:2]]
        ow = scaled(src('onewire'), mhz, ('TRSTL', 'TPRES', 'TRSTH', 'TLOW1', 'TLOW0', 'TREC'))
        ev, pres, tgt, m_, e = ow_case(ow, [0xA5])
        p += [] if pres == [1] and tgt.bytes == [0xA5] and not e else ['1-Wire: %s %s' % (pres, e[:2])]
        log.check('%d MHz: UART, SPI, I2C, WS2812, 1-Wire' % mhz, not p, str(p[:4]) if p else '')
    rig.CLK = CLK = 50_000_000


# ---- word counts -----------------------------------------------------------

def check_fit(log):
    total = 0
    for name in ('uart_tx', 'uart_rx', 'spi', 'i2c_write', 'ws2812', 'onewire'):
        log.group(*P(name))
        img = assemble(src(name), depth=128)
        total += img.size
        log.check('%s fits 128 words' % name, img.size <= 128, '%d words' % img.size)
    log.group(*P('uart_tx', 'uart_rx', 'spi', 'i2c_write'))
    four = sum(assemble(src(n), depth=128).size for n in ('uart_tx', 'uart_rx', 'spi', 'i2c_write'))
    log.check('UART TX + UART RX + SPI + I2C write in one image', four <= 128, '%d of 128 words' % four)


def main(log):
    check_fit(log)
    check_uart(log)
    check_spi(log)
    check_i2c(log)
    check_ws(log)
    check_ow(log)
    check_conc(log)
    check_clk(log)
    check_bugs(log)
