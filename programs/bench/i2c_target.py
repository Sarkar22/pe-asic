"""I2C target against a controller model at the exact UM10204 minimums.
Oracles: the bytes the controller sees, the pad-level i2c_decode, and reaction measured from
pad to pad."""
import random
from protolib import *  # noqa: F401,F403
PROGRAMS = ('advanced/i2c_tgt.s', 'advanced/i2c_fp.s')
from rig import i2c_decode

SDA, SCL = 2, 3
# both shapes of each mode stay at or below fSCL max: minimum low with a long high, then minimum high
TIM = {'Sm': dict(low=235, high=265, su_dat=13, hd=15, su_sta=235, hd_sta=200, su_sto=200, buf=235),
       'SmH': dict(low=300, high=200, su_dat=13, hd=15, su_sta=235, hd_sta=200, su_sto=200, buf=235),
       'Fm': dict(low=65, high=60, su_dat=5, hd=2, su_sta=30, hd_sta=30, su_sto=30, buf=65),
       'FmH': dict(low=95, high=30, su_dat=5, hd=2, su_sta=30, hd_sta=30, su_sto=30, buf=65),
       'Fp': dict(low=25, high=25, su_dat=3, hd=2, su_sta=13, hd_sta=13, su_sto=13, buf=25),
       'FpH': dict(low=37, high=13, su_dat=3, hd=2, su_sta=13, hd_sta=13, su_sto=13, buf=25)}


class Ctl(Device):
    """Controller. All waits are cycles at the pad; rise is the bus rise time so that the timing
    holds at the pad, as UM10204 measures it. Honours stretching."""

    def __init__(self, mode, script, jitter=0, seed=0):
        self.t = TIM[mode]
        self.lscl = self.lsda = 0
        self.scl = self.sda = 1
        self.io_low = 0
        self.gen = self.main(script)
        self.script = script
        self.rng = random.Random(seed)
        self.jitter = jitter
        self.log = []                   # ('w', byte, acked) or ('r', byte)
        self.done = False

    def step(self, c, uo, io):
        self.scl, self.sda = (io >> SCL) & 1, (io >> SDA) & 1
        if self.gen is not None:
            try:
                next(self.gen)
            except StopIteration:
                self.gen = None
                self.done = True
        self.io_low = (self.lscl << SCL) | (self.lsda << SDA)

    def wait(self, n):
        n = int(n) + (self.rng.randrange(self.jitter + 1) if self.jitter else 0)
        for _ in range(n):
            yield

    def release_scl(self):
        self.lscl = 0
        while not self.scl:
            yield

    def main(self, script):
        yield from self.wait(1200)
        yield from script(self)

    def start(self):
        self.lsda = 1
        yield from self.wait(self.t['hd_sta'])
        self.lscl = 1

    def restart(self):
        yield from self.wait(self.t['hd'])
        self.lsda = 0
        yield from self.wait(self.t['low'] - self.t['hd'])
        yield from self.release_scl()
        yield from self.wait(self.t['su_sta'])
        yield from self.start()

    def bit(self, b):
        """called with SCL just pulled low. Returns via self.got the level at the end of high."""
        yield from self.wait(self.t['hd'])
        self.lsda = 1 - b
        yield from self.wait(self.t['low'] - self.t['hd'])
        yield from self.release_scl()
        yield from self.wait(self.t['high'])
        self.got = self.sda
        self.lscl = 1

    def write(self, v):
        for i in range(7, -1, -1):
            yield from self.bit((v >> i) & 1)
        yield from self.bit(1)
        ok = self.got == 0
        self.log.append(('w', v, ok))
        self.lastack = ok

    def read(self, ack):
        v = 0
        for i in range(8):
            yield from self.bit(1)
            v = (v << 1) | self.got
        yield from self.bit(0 if ack else 1)
        self.log.append(('r', v))

    def stop(self):
        yield from self.wait(self.t['hd'])
        self.lsda = 1
        yield from self.wait(self.t['su_dat'])
        self.lsda = 1
        yield from self.wait(self.t['low'] - self.t['hd'] - self.t['su_dat'])
        yield from self.release_scl()
        yield from self.wait(self.t['su_sto'])
        self.lsda = 0
        while not self.sda:             # tBUF runs from SDA high at the pad
            yield
        yield from self.wait(self.t['buf'])


def write_read(c, addr, ptr, data, nread):
    yield from c.start()
    yield from c.write(addr << 1)
    yield from c.write(ptr)
    for d in data:
        yield from c.write(d)
    yield from c.stop()
    yield from c.start()
    yield from c.write(addr << 1)
    yield from c.write(ptr)
    yield from c.restart()
    yield from c.write((addr << 1) | 1)
    for i in range(nread):
        yield from c.read(i != nread - 1)
    yield from c.stop()


def run(mode, script, rise=0, thread=0, align=0, depth=128, addr=0x50, name='i2c_tgt', cycles=None, jitter=0, seed=1, defines=None):
    c = Ctl(mode, script, jitter, seed)
    r = Rig(devices=[c], pullups=(1 << SDA) | (1 << SCL), rise=rise, depth=depth)
    img = assemble(src(name), depth=depth, defines=defines)
    r.load(img, {thread: 'start'})

    def agent():
        yield from r.host.post(7, addr)
    r.board.agent(agent())
    r.start(align)
    r.run(cycles or 400000, stop=lambda: c.done)
    r.run(500)
    return r, c, img


def reactions(r):
    """Every time the chip starts pulling SDA, how long after the SCL fall at the pad."""
    scl = r.wave('io', SCL)
    falls = [t for t, v in scl.edges() if v == 0]
    pulls = [t for t, v in r.board.trace.pull_low(SDA).edges() if v == 1]
    out = []
    import bisect
    for p in pulls:
        i = bisect.bisect_right(falls, p) - 1
        if i >= 0:
            out.append(p - falls[i])
    rel = [t for t, v in r.board.trace.pull_low(SDA).edges() if v == 0]
    rl = []
    for p in rel:
        i = bisect.bisect_right(falls, p) - 1
        if i >= 0:
            rl.append(p - falls[i])
    return out, rl


def main(log):
    log.group('advanced/i2c_tgt.s', title='I2C target with 32 bytes of storage')
    data = [0xA5, 0x3C, 0xFF, 0x00, 0x81]

    def s1(c):
        yield from write_read(c, 0x50, 4, data, len(data))
    for mode, spec, rises in (('Sm', 'Sm', (0, 50)), ('SmH', 'Sm', (0,)), ('Fm', 'Fm', (0, 15)), ('FmH', 'Fm', (0, 15))):
        for rise in rises:
            r, c, img = run(mode, s1, rise, cycles=3000000)
            got = [x[1] for x in c.log if x[0] == 'r']
            acks = [x[2] for x in c.log if x[0] == 'w']
            tx, er, m = i2c_decode(r.wave('io', SCL), r.wave('io', SDA), spec, lo=800)
            ro, rl = reactions(r)
            ok = got == data and all(acks) and not r.health() and not er
            log.check('%s rise %d: write, stop, random read with repeated START' % (mode, rise), ok,
                      'read %s, acks %d/%d, ACK reaction pad to pad %s cycles, release %s, decoder errs %s %s' % (
                          [hex(x) for x in got], sum(acks), len(acks), span(ro), span(rl), er[:2], r.health()))
    log.line('INFO  i2c_tgt is %d words of code, 32 bytes at 96..127' % img.size)
    ok = 0
    worst = 0
    for th in range(4):
        for al in range(4):
            r, c, img = run('Fm', s1, 10, th, al, cycles=3000000)
            ro, rl = reactions(r)
            worst = max(worst, max(ro))
            ok += [x[1] for x in c.log if x[0] == 'r'] == data and all(x[2] for x in c.log if x[0] == 'w') and not r.health()
    log.check('Fm, 4 threads x 4 alignments', ok == 16, '%d of 16, worst ACK reaction %d cycles' % (ok, worst))

    def s2(c):
        yield from c.start()
        yield from c.write(0x51 << 1)
        yield from c.stop()
        yield from c.start()
        yield from c.write(0x10 << 1)   # 0x10 and 0x50 differ in one bit
        yield from c.stop()
        yield from c.start()
        yield from c.write(0x50 << 1)
        yield from c.write(1)
        yield from c.write(0x77)
        yield from c.stop()
        yield from c.start()
        yield from c.write(0x50 << 1)
        yield from c.write(1)
        yield from c.restart()
        yield from c.write((0x50 << 1) | 1)
        yield from c.read(False)
        yield from c.stop()
    r, c, img = run('Fm', s2, 10, cycles=500000)
    a = [x[2] for x in c.log if x[0] == 'w']
    log.check('wrong addresses stay silent, next transaction is ours', a == [False, False, True, True, True, True, True, True] and [x[1] for x in c.log if x[0] == 'r'] == [0x77] and not r.health(), (a, c.log[-2:]))

    def s3(c):
        yield from c.start()
        yield from c.write(0x50 << 1)
        yield from c.write(8)
        yield from c.write(0x11)
        yield from c.restart()          # repeated START after a data byte, in the first bit
        yield from c.write(0x50 << 1)
        yield from c.write(8)
        yield from c.restart()
        yield from c.write((0x50 << 1) | 1)
        yield from c.read(True)
        yield from c.read(False)
        yield from c.stop()
    r, c, img = run('Fm', s3, 10, cycles=500000)
    log.check('repeated START right after a data byte; read with ACK then NACK', [x[1] for x in c.log if x[0] == 'r'][0] == 0x11 and all(x[2] for x in c.log if x[0] == 'w') and not r.health(), c.log[-3:])

    def s4(c):
        yield from c.start()
        yield from c.write(0x50 << 1)
        yield from c.write(30)
        for i in range(6):
            yield from c.write(0x40 + i)
        yield from c.stop()
        yield from c.start()
        yield from c.write(0x50 << 1)
        yield from c.write(30)
        yield from c.restart()
        yield from c.write((0x50 << 1) | 1)
        for i in range(6):
            yield from c.read(i != 5)
        yield from c.stop()
    r, c, img = run('Fm', s4, 10, cycles=500000)
    log.check('pointer wraps at the top of the 32 bytes', [x[1] for x in c.log if x[0] == 'r'] == [0x40 + i for i in range(6)], c.log[-3:])

    # The 1 MHz build. UM10204, bus timing for Fast-mode Plus: tLOW >= 0.5 us (25 cycles), tHIGH >= 0.26 us
    # (13), tSU;DAT >= 50 ns (3), tSU;STA, tHD;STA, tSU;STO >= 0.26 us (13), tBUF >= 0.5 us (25), rise time
    # <= 120 ns (6). The controller model runs at those minimums (TIM 'Fp': 25 low and 25 high, 'FpH': 37 low
    # and the minimum 13 high, both at fSCL = 1 MHz). Limit on the chip: data and ACK valid within
    # tVD;DAT = tVD;ACK <= 0.45 us (22 cycles) of the SCL fall, counting the rise time when the chip lets go.
    log.group('advanced/i2c_fp.s', title='I2C target, 1 MHz build')
    for mode, rise in (('Fp', 0), ('Fp', 3), ('Fp', 6), ('FpH', 0), ('FpH', 3), ('FpH', 6)):
        good, worst = 0, 0
        for al in range(4):
            r, c, img = run(mode, s1, rise, cycles=500000, name='i2c_fp', align=al)
            got = [x[1] for x in c.log if x[0] == 'r']
            acks = [x[2] for x in c.log if x[0] == 'w']
            ro, rl = reactions(r)
            worst = max([worst] + ro + [x + rise for x in rl])
            good += got == data and all(acks) and not r.health()
        log.check('1 MHz, %s, rise %d ns: write, stop, random read with repeated START, 4 alignments; SDA valid within 0.45 us'
                  % ('tLOW 0.5 us, tHIGH 0.5 us' if mode == 'Fp' else 'tLOW 0.74 us, tHIGH 0.26 us', rise * 20),
                  good == 4 and worst <= 22, '%d of 4 clean, slowest SDA change %d cycles after the SCL fall, limit 22' % (good, worst))
