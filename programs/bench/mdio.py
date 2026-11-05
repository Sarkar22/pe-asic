"""MDIO, IEEE 802.3 Clause 22. MDC at most 2.5 MHz, high and low at least 160 ns, station data
setup and hold 10 ns around MDC rise, PHY output 0 to 300 ns after MDC rise."""
from protolib import *  # noqa: F401,F403
PROGRAMS = ('advanced/mdio_sta.s', 'advanced/mdio_phy.s')

DIO = 0


class Phy(Device):
    """PHY model. Samples MDIO on MDC rise (MDC on a uo pin), answers tco cycles after the rise."""

    def __init__(self, mdc, addr, regs, tco=10):
        self.mdc, self.addr, self.regs, self.tco = mdc, addr, regs, tco
        self.p = 0
        self.bits = []
        self.ones = 0
        self.state = 'pre'
        self.pend = []
        self.drive = None
        self.io_low = 0
        self.driven = []                # (from, to) cycles this PHY held the line
        self.frames = []
        self.since = None

    def step(self, c, uo, io):
        mdc = (uo >> self.mdc) & 1
        d = (io >> DIO) & 1
        if mdc and not self.p:
            self.rise(c, d)
        self.p = mdc
        while self.pend and self.pend[0][0] <= c + 1:
            _, v = self.pend.pop(0)
            if v is None:
                if self.since is not None:
                    self.driven.append((self.since, c + 1))
                self.since = None
                self.io_low = 0
            else:
                if self.since is None:
                    self.since = c + 1
                self.io_low = (1 - v) << DIO

    def rise(self, c, d):
        if self.state == 'pre':
            if d:
                self.ones += 1
            else:
                if self.ones >= 32:
                    self.state = 'hdr'
                    self.bits = [0]
                self.ones = 0
        elif self.state == 'hdr':
            self.bits.append(d)
            if len(self.bits) == 14:
                b = self.bits
                self.op = (b[2] << 1) | b[3]
                self.pa = sum(x << (4 - i) for i, x in enumerate(b[4:9]))
                self.ra = sum(x << (4 - i) for i, x in enumerate(b[9:14]))
                self.n = 0
                if self.pa != self.addr:
                    self.state = 'pre'
                elif self.op == 2:
                    self.state = 'rta'
                    self.word = self.regs.get(self.ra, 0)
                    self.pend.append((c + self.tco, None))
                else:
                    self.state = 'wr'
                    self.word = 0
        elif self.state == 'rta':
            # this rise is the first TA bit; the zero must be there by the next rise
            self.pend.append((c + self.tco, 0))
            self.state = 'rd'
        elif self.state == 'rd':
            self.pend.append((c + self.tco, (self.word >> (15 - self.n)) & 1))
            self.n += 1
            if self.n == 16:
                self.state = 'rdend'
        elif self.state == 'rdend':
            self.pend.append((c + self.tco, None))
            self.state = 'pre'
            self.ones = 0
            self.frames.append(('r', self.pa, self.ra, self.word))
        elif self.state == 'wr':
            self.n += 1
            if self.n > 2:
                self.word = (self.word << 1) | d
            if self.n == 18:
                self.regs[self.ra] = self.word
                self.frames.append(('w', self.pa, self.ra, self.word))
                self.state = 'pre'
                self.ones = 0


def mdio_decode(mdc, mdio, oe, data_uo_driven=True):
    """Pads only. Walks MDC rises. Returns (frames, errs, measured)."""
    rises = [t for t, v in mdc.edges() if v == 1]
    falls = [t for t, v in mdc.edges() if v == 0]
    errs, frames = [], []
    m = dict(hi=[], lo=[], per=[], su=[], hd=[])
    for a, b in zip(rises, rises[1:]):
        if b - a < 3000:
            m['per'].append(b - a)
    for r in rises:
        f = next((x for x in falls if x > r), None)
        if f is not None and f - r < 3000:
            m['hi'].append(f - r)
        f0 = [x for x in falls if x < r][-1:]
        if f0 and r - f0[0] < 3000:
            m['lo'].append(r - f0[0])
    bits = [(r, mdio.at(r)) for r in rises]
    i, n = 0, len(bits)
    while i < n:
        ones = 0
        while i < n and bits[i][1] == 1:
            ones += 1
            i += 1
        if i + 32 > n:
            break
        if ones < 32:
            errs.append('frame after only %d preamble ones at %d' % (ones, bits[i][0]))
        b = [x[1] for x in bits[i:i + 32]]
        ts = [x[0] for x in bits[i:i + 32]]
        if b[0:2] != [0, 1]:
            errs.append('bad ST %s at %d' % (b[:2], ts[0]))
        op = (b[2] << 1) | b[3]
        pa = sum(x << (4 - k) for k, x in enumerate(b[4:9]))
        ra = sum(x << (4 - k) for k, x in enumerate(b[9:14]))
        data = sum(x << (15 - k) for k, x in enumerate(b[16:32]))
        if op == 2:
            if oe.at(ts[14] + 1) or any(oe.at(t + 1) for t in ts[14:32]):
                errs.append('station still driving during read at %d' % ts[14])
            if b[15] != 0:
                errs.append('TA second bit high at %d' % ts[15])
        else:
            if b[14:16] != [1, 0]:
                errs.append('write TA %s at %d' % (b[14:16], ts[14]))
        frames.append((op, pa, ra, data))
        # setup and hold around each rise for bits the station drove
        drv = range(0, 14) if op == 2 else range(0, 32)
        ed = [t for t, v in mdio.edges()]
        for k in drv:
            r = ts[k]
            before = [t for t in ed if t <= r]
            after = [t for t in ed if t > r]
            if before:
                m['su'].append(r - before[-1])
            if after:
                m['hd'].append(after[0] - r)
        i += 32
    for k, lo in (('hi', 8), ('lo', 8), ('per', 20)):
        if m[k] and min(m[k]) < lo:
            errs.append('MDC %s %d cycles' % (k, min(m[k])))
    return frames, errs, m


def sta_case(half, ops, tco=10, thread=0, align=0, regs=None):
    regs = dict(regs or {})
    phy = Phy(1, 0x11, regs, tco)
    r = Rig(devices=[phy], pullups=1 << DIO)
    img = assemble(src('mdio_sta'), depth=128)
    r.load(img, {thread: 'start'})
    got = []

    def agent():
        yield from r.host.post(7, half)
        for op in ops:
            yield from r.host.post(0, op[0])
            if len(op) > 1:
                yield from r.host.post(0, op[1])
            else:
                yield from r.host.take(1, got)
    r.board.agent(agent())
    r.start(align)
    r.run(len(ops) * (70 * 2 * half + 4000) + 4000, stop=r.agents_done)
    r.run(300 * half)
    return r, phy, [w for _, w in got], img


def hdr(op, phy, reg, ta=2):
    return (1 << 14) | (op << 12) | (phy << 7) | (reg << 2) | ta


def conflict(r, phy):
    oe = r.board.trace.wave('iooe', DIO)
    return [(a, b) for a, b in phy.driven if any(oe.at(t) for t in range(a, b, 2))]


class Station(Device):
    """Station model for the chip-as-PHY case. MDC on ui5, MDIO on the io pin. Runs at half cycles
    high and low. Drives MDIO after MDC falls and samples on MDC rise; releases during read TA."""

    def __init__(self, ops, half, t_start=1000, dirty=True):
        self.dirty = dirty
        self.ui_mask = 1 << 5
        self.ui_val = 0
        self.io_low = 0
        self.half = half
        self.t = t_start
        self.got = []
        self.gen = self.script(ops)
        self.io = 1

    def drive(self, b):
        self.io_low = (1 - b) << DIO

    def clock(self, b):
        """one MDC period carrying data b (None = released). Returns the level sampled at the rise."""
        h = self.half
        for _ in range(h):
            self.ui_val = 0
            yield
        if b is None:
            self.io_low = 0
        else:
            self.drive(b)
        for _ in range(h):
            yield
        self.ui_val = 1 << 5
        self.sampled = self.io
        yield                           # 20 ns of hold, more than the 10 ns the spec gives
        if b is not None and self.dirty:
            self.drive(1 - b)           # worst legal station: garbage straight after the hold
        for _ in range(h - 1):
            yield

    def script(self, ops):
        for _ in range(self.t):
            yield
        for op in ops:
            for _ in range(32):
                yield from self.clock(1)
            hd = op[0]
            nhead = 14
            for i in range(nhead):
                yield from self.clock((hd >> (15 - i)) & 1)
            if len(op) > 1:
                yield from self.clock(1)
                yield from self.clock(0)
                for i in range(16):
                    yield from self.clock((op[1] >> (15 - i)) & 1)
                yield from self.clock(None)
            else:
                yield from self.clock(None)
                yield from self.clock(None)
                self.ta = self.sampled
                v = 0
                for i in range(16):
                    yield from self.clock(None)
                    v = (v << 1) | self.sampled
                self.got.append((self.ta, v))
                yield from self.clock(None)
            for _ in range(4):
                yield from self.clock(1)

    def step(self, c, uo, io):
        self.io = (io >> DIO) & 1
        if self.gen is not None:
            try:
                next(self.gen)
            except StopIteration:
                self.gen = None


def phy_case(half, ops, regs, addr=0x11, thread=0, align=0, depth=128):
    st = Station(ops, half)
    r = Rig(devices=[st], pullups=1 << DIO, depth=depth)
    img = assemble(src('mdio_phy'), depth=depth)
    r.load(img, {thread: 'start'})
    for k, v in regs.items():
        r.chip.host_wrmem(96 + k, v)

    def agent():
        yield from r.host.post(7, addr)
    r.board.agent(agent())
    r.start(align)
    r.run(1000 + sum(70 for _ in ops) * 3 * half * 2 + 8000, stop=lambda: st.gen is None)
    r.run(2000)
    return r, st, img


def main(log):
    log.group('advanced/mdio_sta.s', title='MDIO station')
    ops = [(hdr(2, 0x11, 2),), (hdr(1, 0x11, 4), 0xA5C3), (hdr(2, 0x11, 4),), (hdr(2, 0x11, 3),)]
    for half, tco in ((20, 15), (20, 0), (50, 15), (250, 15)):
        r, phy, got, img = sta_case(half, ops, tco, regs={2: 0x0141, 3: 0x0CC2})
        fr, er, m = mdio_decode(r.wave('uo', 1), r.wave('io', DIO), r.board.trace.wave('iooe', DIO))
        c = conflict(r, phy)
        ok = got == [0x0141, 0xA5C3, 0x0CC2] and not er and not c and not r.health() and [f[0] for f in fr] == [2, 1, 2, 2]
        log.check('station half period %d, PHY tco %d: read, write, read back, read' % (half, tco), ok,
                  '%s %s period %s hi %s lo %s su %s hd %s conflicts %s health %s' % ([hex(x) for x in got], er[:2], *[min(m[k], default=None) for k in ('per', 'hi', 'lo', 'su', 'hd')], c[:2], r.health()))
    log.line('INFO  mdio_sta is %d words' % img.size)
    ok = 0
    for th in range(4):
        for al in range(4):
            r, phy, got, img = sta_case(20, ops, 15, th, al, regs={2: 0x0141, 3: 0x0CC2})
            fr, er, m = mdio_decode(r.wave('uo', 1), r.wave('io', DIO), r.board.trace.wave('iooe', DIO))
            ok += got == [0x0141, 0xA5C3, 0x0CC2] and not er and not conflict(r, phy) and not r.health()
    log.check('1.25 MHz, half period 20, tco 300 ns, 4 threads x 4 alignments', ok == 16, '%d of 16' % ok)
    log.group('advanced/mdio_phy.s', title='MDIO PHY')
    regs = {2: 0x0141, 3: 0x0CC2, 31: 0xFFFF, 0: 0x8000}
    ops2 = [(hdr(2, 0x11, 2),), (hdr(1, 0x11, 5), 0x1234), (hdr(2, 0x11, 5),), (hdr(2, 0x11, 31),), (hdr(2, 0x11, 0),)]
    for half in (100, 50, 30, 20):
        ok = 0
        for al in range(4):
            r, st, img = phy_case(half, ops2, regs, align=al)
            want = [(0, 0x0141), (0, 0x1234), (0, 0xFFFF), (0, 0x8000)]
            ok += st.got == want and not r.health()
            if st.got != want:
                bad = st.got
        mdc = r.wave('ui', 5)
        rises = [t for t, v in mdc.edges() if v == 1]
        oe = r.board.trace.wave('iooe', DIO)
        lat = []
        for t, v in r.board.trace.wave('ioout', DIO).edges() + oe.edges():
            prev = [x for x in rises if x <= t]
            if prev and oe.at(t) or (prev and v == 1):
                lat.append(t - prev[-1])
        log.check('chip as PHY, MDC half period %d, worst-case 20 ns station hold, 4 alignments' % half, ok == 4, '%d of 4, MDIO moves %d..%d cycles after MDC rise, limit 15 %s' % (ok, min(lat), max(lat), '' if ok == 4 else bad))
    log.line('INFO  mdio_phy is %d words of code, regs at 96..127' % img.size)
