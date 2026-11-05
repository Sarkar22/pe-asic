"""A pad-level 24C02-style EEPROM, and a bench that drives the host-driven I2C controllers
(standard/i2c.s, extra/i2c.s, trimmed/i2c.s) against it: SCL on io1, SDA on io0."""
from simple import Bus, Run, read

SCL, SDA = 1, 0                                 # uio numbers


class Eeprom:
    def __init__(s, scl, sda, dev=0xA0, tco=8, stretch=0, nack_data=False):
        s.scl, s.sda = 1 << scl, 1 << sda
        s.dev, s.tco, s.stretch, s.nack_data = dev, tco, stretch, nack_data
        s.mem = [0xFF] * 256
        s.ptr = 0
        s.ps = s.pc = 1
        s.st = 'idle'
        s.n = s.b = s.tx = 0
        s.starts = s.stops = s.falls = 0
        s.pend = []
        s.hold = 0

    def later(s, c, f):
        s.pend.append((c + s.tco, f))

    def step(s, c, bus, lvl):
        sda, scl = int(bool(lvl & s.sda)), int(bool(lvl & s.scl))
        if s.pc and scl and s.ps and not sda:
            s.starts += 1
            s.st, s.n, s.b = 'addr', 0, 0
        elif s.pc and scl and not s.ps and sda:
            s.stops += 1
            s.st = 'idle'
            bus.ext_low &= ~s.sda
            s.pend = []
        elif not s.pc and scl:                         # rising: sample
            if s.st in ('addr', 'ptr', 'wr'):
                s.b = (s.b << 1 | sda) & 0xFF
                s.n += 1
            elif s.st == 'mack':                       # controller's ack bit after a read byte
                s.st = 'rd' if not sda else 'idle'
                s.n = 0
                if s.st == 'rd':
                    s.tx = s.mem[s.ptr]
                    s.ptr = (s.ptr + 1) & 255
        elif s.pc and not scl:                         # falling: drive
            s.falls += 1
            if s.stretch:
                bus.ext_low |= s.scl
                s.hold = c + s.stretch
            if s.st in ('addr', 'ptr', 'wr') and s.n == 8:
                ack = True
                if s.st == 'addr':
                    ack = (s.b & 0xFE) == s.dev
                    nxt = ('ptr', 'rda')[s.b & 1] if ack else 'idle'
                elif s.st == 'ptr':
                    s.ptr = s.b
                    nxt = 'wr'
                else:
                    if s.nack_data:
                        ack = False
                    else:
                        s.mem[s.ptr] = s.b
                        s.ptr = (s.ptr + 1) & 255
                    nxt = 'wr'
                s.nxt = nxt
                if ack:
                    s.later(c, lambda: setattr(bus, 'ext_low', bus.ext_low | s.sda))
                    s.st = 'ack'
                else:
                    s.st = 'idle'
            elif s.st == 'ack':
                s.later(c, lambda: setattr(bus, 'ext_low', bus.ext_low & ~s.sda))
                s.st, s.n, s.b = s.nxt, 0, 0
                if s.st == 'rda':
                    s.st = 'rd'
                    s.tx = s.mem[s.ptr]
                    s.ptr = (s.ptr + 1) & 255
                    s.n = 0
                    s._drive(c, bus)
            elif s.st == 'rd':
                s._drive(c, bus)
        s.ps, s.pc = sda, scl
        if s.hold and c >= s.hold:
            bus.ext_low &= ~s.scl
            s.hold = 0
        while s.pend and s.pend[0][0] <= c:
            s.pend.pop(0)[1]()

    def _drive(s, c, bus):
        if s.n < 8:
            bit = (s.tx >> (7 - s.n)) & 1
            s.n += 1

            def f(b=bit):
                bus.ext_low = (bus.ext_low & ~s.sda) | (0 if b else s.sda)
            s.later(c, f)
        else:
            s.later(c, lambda: setattr(bus, 'ext_low', bus.ext_low & ~s.sda))
            s.st = 'mack'


def run(rel, ops, stretch=0, nack_data=False, hook=None, fast=False, limit=900000):
    """ops: (device byte, register, data). Returns (bytes from MOUT, eeprom, run, bus trace, bus)."""
    r = Run(read(rel))
    bus = Bus()
    dev = Eeprom(SCL, SDA, stretch=stretch, nack_data=nack_data)
    dev.mem[0x10] = 0xC3
    dev.mem[0x20] = 0x3C
    trace = []

    def w(c, uo, o, oe):
        lvl = bus.level(o, oe)
        if hook:
            hook(c, bus, dev, lvl)
        dev.step(c, bus, lvl)
        l2 = bus.level(o, oe)
        s = (int(bool(l2 & (1 << SCL))), int(bool(l2 & (1 << SDA))))
        if not trace or trace[-1][1] != s:
            trace.append((c, s))
        return 0xFF, l2
    cfg = [45, 37] if fast else [250, 210]          # tHIGH, then tLOW minus the data hold, in cycles
    res, q = [], []
    for devb, reg, data in ops:
        words = [(devb << 8) | reg]
        if not devb & 1:
            words.append(data << 8)
        q.append(words)
    want = 2 * len(ops)
    cur, sent = [], 0
    while r.c < limit and len(res) < want:
        if cfg:
            if r.chip.post(7, cfg[0]):
                cfg.pop(0)
        elif r.c > 300:
            if not cur and q and sent == len(res) // 2:
                cur = q.pop(0)
                sent += 1
            if cur and r.post(cur[0]):
                cur.pop(0)
        r.step(w)
        v = r.take()
        if v is not None:
            res.append(v & 0xFF)
    for _ in range(3000):
        r.step(w)
    return res, dev, r, trace, bus


def timing(trace):
    """Minimum tLOW, tHIGH, tSU;STA on a repeated START, tHD;STA, tSU;STO at the pad, in ns."""
    m = dict(low=1 << 30, high=1 << 30, su_sta=1 << 30, su_sto=1 << 30, hd_sta=1 << 30)
    lf = lr = None
    sda_fall = None
    prev = None
    for c, (scl, sda) in trace:
        if prev is not None:
            ps, pd = prev
            if ps and not scl:
                if lr is not None:
                    m['high'] = min(m['high'], c - lr)
                if sda_fall is not None:
                    m['hd_sta'] = min(m['hd_sta'], c - sda_fall)
                    sda_fall = None
                lf = c
            if not ps and scl:
                if lf is not None:
                    m['low'] = min(m['low'], c - lf)
                lr = c
            if scl and pd and not sda:
                if lr is not None:
                    m['su_sta'] = min(m['su_sta'], c - lr)
                sda_fall = c
            if scl and not pd and sda and lr is not None:
                m['su_sto'] = min(m['su_sto'], c - lr)
        prev = (scl, sda)
    return {k: v * 20 for k, v in m.items()}


# UM10204 bus timing table, minimums in ns, for the intervals timing() measures
UM10204 = {'Sm': dict(low=4700, high=4000, su_sta=4700, su_sto=4000, hd_sta=4000),
           'Fm': dict(low=1300, high=600, su_sta=600, su_sto=600, hd_sta=600)}


def within(t, mode):
    return all(t[k] >= v for k, v in UM10204[mode].items())
