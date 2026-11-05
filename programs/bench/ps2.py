"""PS/2 device firmware against a host model written from the published host procedure
(Chapweske, 'The PS/2 Mouse/Keyboard Protocol'). Host and decoder see only pad levels."""
from protolib import *  # noqa: F401,F403
PROGRAMS = ('advanced/ps2_dev.s',)

CK, DT = 3, 2
US = 50


def parity_ok(bits):
    return sum(bits) % 2 == 1


class Ps2Host(Device):
    """Runs a script generator once per cycle. Listens all the time: on each clock fall with the
    host not transmitting it takes one bit of a device-to-host frame."""

    def __init__(self, script):
        self.script = script(self)
        self.lck = self.ldt = 0
        self.ck = self.dt = 1
        self.rx = []                    # (cycle, byte, ok)
        self.fr = []
        self.txing = False
        self.c = 0
        self.frame_abort = 0

    def step(self, c, uo, io):
        self.c = c
        ck, dt = (io >> CK) & 1, (io >> DT) & 1
        if not self.txing:
            if self.ck and not ck and not self.lck:
                self.fr.append((c, dt))
                if len(self.fr) == 11:
                    b = [x[1] for x in self.fr]
                    val = sum(v << i for i, v in enumerate(b[1:9]))
                    self.rx.append((c, val, b[0] == 0 and b[10] == 1 and parity_ok(b[1:10])))
                    self.fr = []
        self.ck, self.dt = ck, dt
        if self.script is not None:
            try:
                next(self.script)
            except StopIteration:
                self.script = None
        self.io_low = (self.lck << CK) | (self.ldt << DT)

    def wait(self, n):
        for _ in range(int(n)):
            yield

    def until(self, fn, limit=2000000):
        while not fn():
            limit -= 1
            if limit == 0:
                raise RuntimeError('host timed out')
            yield

    def tx(self, val, flip_parity=False):
        """Steps 1 to 12 of the host procedure."""
        self.txing = True
        self.fr = []
        self.lck = 1
        yield from self.wait(120 * US)
        self.ldt = 1
        yield from self.wait(10 * US)
        self.lck = 0
        yield from self.until(lambda: self.ck == 1)     # the line needs time to rise
        bits = [(val >> i) & 1 for i in range(8)]
        bits.append(1 - sum(bits) % 2)
        if flip_parity:
            bits[8] ^= 1
        for b in bits:
            yield from self.until(lambda: self.ck == 0)
            yield from self.wait(5 * US)
            self.ldt = 1 - b
            yield from self.until(lambda: self.ck == 1)
        yield from self.until(lambda: self.ck == 0)
        yield from self.wait(5 * US)
        self.ldt = 0                    # step 9: release data, the stop bit
        yield from self.until(lambda: self.ck == 1)
        self.acked = None
        yield from self.until(lambda: self.dt == 0)
        yield from self.until(lambda: self.ck == 0)
        self.acked = self.dt == 0
        yield from self.until(lambda: self.ck == 1 and self.dt == 1)
        self.txing = False
        self.fr = []

    def inhibit_after(self, nclk, hold_us, then_rts=None):
        """Waits for nclk device clocks of a frame, then pulls clock low."""
        n = 0
        p = 1
        while n < nclk:
            if p and not self.ck:
                n += 1
            p = self.ck
            yield
        yield from self.until(lambda: self.ck == 1)
        yield from self.wait(10 * US)
        self.lck = 1
        self.fr = []
        yield from self.wait(hold_us * US)
        if then_rts is None:
            self.lck = 0
        else:
            yield from self.tx(then_rts)


def ps2_decode(ck, dt, ck_chip, dt_chip):
    """From resolved pad levels plus who pulled. Returns (device frames, errors, measured).
    Clock highs and lows are taken only for pulses this chip made, inside frames."""
    errs, m = [], dict(low=[], high=[], rise2dt=[], dt2fall=[])
    out, cur = [], []
    falls = [t for t, v in ck_chip.edges() if v == 1]
    rises = [t for t, v in ck_chip.edges() if v == 0]
    for f in falls:
        r = next((x for x in rises if x > f), None)
        if r is None:
            errs.append('chip holds clock low at the end')
            break
        m['low'].append(r - f)
    lvl = ck.edges()
    for (a, va), (b, vb) in zip(lvl, lvl[1:]):
        if va == 1 and vb == 0 and b - a < 60 * US and b in falls:  # highs this chip ended
            m['high'].append(b - a)
    for t, v in dt_chip.edges():
        prev = [x for x in ck.edges(0, t + 1) if x[1] == 1]
        nxt = [x for x in ck.edges(t, ck.end) if x[1] == 0]
        if prev and nxt and t - prev[-1][0] < 60 * US and nxt[0][0] - t < 60 * US and ck.at(t) == 1:
            m['rise2dt'].append(t - prev[-1][0])
            m['dt2fall'].append(nxt[0][0] - t)
    for k, lo, hi in (('low', 30, 50), ('high', 30, 50)):
        if m[k] and (min(m[k]) < lo * US or max(m[k]) > hi * US):
            errs.append('clock %s %d..%d cycles' % (k, min(m[k]), max(m[k])))
    if m['rise2dt'] and min(m['rise2dt']) < 5 * US:
        errs.append('data moved %d cycles after clock rise' % min(m['rise2dt']))
    if m['dt2fall'] and (min(m['dt2fall']) < 5 * US or max(m['dt2fall']) > 25 * US):
        errs.append('data to clock fall %d..%d' % (min(m['dt2fall']), max(m['dt2fall'])))
    return errs, m


def case(script, posts=(), thread=0, align=0, cycles=2000000, rise=100):
    h = Ps2Host(script)
    r = Rig(devices=[h], pullups=(1 << CK) | (1 << DT), rise=rise)
    # `inbox` watches M[TID], so the input word has to be this thread's word
    img = assemble(src('ps2_dev'), defines={'MIN': thread, 'MOUT': 5}, depth=128)
    r.load(img, {thread: 'start'})
    got = []

    def agent():
        for p in posts:
            if isinstance(p, tuple):
                yield p[0]
                yield from r.host.post(thread, p[1])
            else:
                yield from r.host.post(thread, p)
        while True:
            yield from r.host.take(5, got, tries=1)
    r.board.agent(agent())
    r.start(align)
    r.run(cycles)
    errs, m = ps2_decode(r.wave('io', CK), r.wave('io', DT), r.board.trace.pull_low(CK), r.board.trace.pull_low(DT))
    return h, [w for _, w in got], errs + r.health(), m, img


def main(log):
    log.group('advanced/ps2_dev.s', title='PS/2 device')

    def idle(h):
        yield
    # Each run lasts at least 150000 cycles (3 ms, three frames) past the last bus edge it needs.
    h, w, e, m, img = case(idle, [0x1C, 0xF0, 0x1C], cycles=300000)
    log.line('INFO  ps2_dev is %d words' % img.size)
    log.check('device sends 1C F0 1C, host model decodes, parity ok, timing in range',
              [(v, ok) for _, v, ok in h.rx] == [(0x1C, True), (0xF0, True), (0x1C, True)] and not e,
              'rx %s, low %s high %s rise-to-data %s data-to-fall %s, %s' % (
                  [hex(v) for _, v, _ in h.rx], span(m['low']), span(m['high']), span(m['rise2dt']), span(m['dt2fall']), e[:3]))
    ok = 0
    for th in range(4):
        for al in range(4):
            h, w, e, m, img = case(idle, [0x55, 0x80], th, al, 220000)      # parity bits 1 and 0
            ok += [v for _, v, o in h.rx if o] == [0x55, 0x80] and not e
    log.check('4 threads x 4 alignments, 2 bytes', ok == 16, '%d of 16' % ok)

    def cmds(*seq):
        def s(h):
            yield from h.wait(1000 * US)
            for c in seq:
                if isinstance(c, tuple):
                    yield from h.tx(c[0], flip_parity=True)
                else:
                    yield from h.tx(c)
                assert h.acked
                yield from h.wait(6000 * US)
        return s
    h, w, e, m, img = case(cmds(0xFF), cycles=350000)
    log.check('host sends FF, device ACKs and answers FA AA', [v for _, v, o in h.rx if o] == [0xFA, 0xAA] and w == [0xFF] and not e, ([hex(v) for _, v, _ in h.rx], e[:2]))
    h, w, e, m, img = case(cmds(0xF2), cycles=400000)
    log.check('host sends F2, answers FA AB 83', [v for _, v, o in h.rx if o] == [0xFA, 0xAB, 0x83] and not e, ([hex(v) for _, v, _ in h.rx], e[:2]))
    h, w, e, m, img = case(cmds(0xED, 0x02), cycles=650000)
    log.check('host sends ED 02 (LEDs), FA after each, mask reaches the mailbox', [v for _, v, o in h.rx if o] == [0xFA, 0xFA] and w == [0xED, 0x02] and not e, ([hex(v) for _, v, _ in h.rx], w, e[:2]))
    h, w, e, m, img = case(cmds(0xEE, 0xF4, 0xFE), cycles=1000000)
    log.check('echo, plain FA, then FE resends the FA', [v for _, v, o in h.rx if o] == [0xEE, 0xFA, 0xFA] and not e, ([hex(v) for _, v, _ in h.rx], e[:2]))
    h, w, e, m, img = case(cmds((0x42,)), cycles=300000)
    log.check('bad parity from the host: device answers FE, flags the byte', [v for _, v, o in h.rx if o] == [0xFE] and w == [0x142] and not e, ([hex(v) for _, v, _ in h.rx], w, e[:2]))

    good = 0
    for n in range(1, 11):
        def s(h, n=n):
            yield from h.inhibit_after(n, 300)
        h, w, e, m, img = case(s, [0x5A], cycles=220000)
        vals = [v for _, v, o in h.rx if o]
        good += vals == [0x5A] and not [x for x in e if 'clock' not in x]
    log.check('host inhibits after clock 1..10 of a frame: byte is resent whole', good == 10, '%d of 10' % good)

    def s(h):
        yield from h.inhibit_after(4, 120, then_rts=0xF2)
    h, w, e, m, img = case(s, [0x77], cycles=300000)
    v = [x for _, x, o in h.rx if o]
    log.check('host takes the bus mid-frame with RTS: command served, interrupted byte resent', 0x77 in v and w and w[0] == 0xF2, ([hex(x) for x in v], w, [x for x in e if 'clock' not in x][:2]))

    def s(h):
        h.lck = 1
        yield from h.wait(3000 * US)
        h.lck = 0
    h, w, e, m, img = case(s, [0x33], cycles=350000)
    tfirst = h.rx[0][0] if h.rx else None
    log.check('byte queued while the host inhibits waits for the release', [v for _, v, o in h.rx if o] == [0x33] and tfirst and tfirst > 3000 * US, tfirst)
    h, w, e, m, img = case(idle, [0xA5, 0x5A], cycles=260000, rise=500)
    log.check('slow bus, 10 us clock and data rise', [v for _, v, o in h.rx if o] == [0xA5, 0x5A], ([hex(v) for _, v, _ in h.rx], e[:2]))

