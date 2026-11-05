"""SWD against an ADIv5 target model. The target samples on SWCLK rise and changes SWDIO tco cycles
after a rise. One turnaround clock. Line reset is 50 or more ones."""
from protolib import *  # noqa: F401,F403
PROGRAMS = ('advanced/swd.s',)

CK, IO = 1, 0
OK, WAIT, FAULT = 1, 2, 4


class Target(Device):
    def __init__(self, tco=3, dpidr=0x2BA01477):
        self.tco = tco
        self.dpidr = dpidr
        self.p = 0
        self.ones = 0
        self.state = 'hunt'
        self.pend = []
        self.io_low = 0
        self.regs = {4: 0, 8: 0}
        self.ap = {0: 0x11223344, 4: 0x55667788}
        self.rdbuff = 0
        self.inject = []                # per transfer: None, 'wait', 'fault', 'mute', 'parity'
        self.waits = 0
        self.log = []
        self.resets = 0
        self.errors = []
        self.driven = []
        self.since = None
        self.history = 0

    def out(self, c, v):
        self.pend.append((c + self.tco, v))

    def step(self, c, uo, io):
        ck = (uo >> CK) & 1
        d = (io >> IO) & 1
        if ck and not self.p:
            self.rise(c, d)
        self.p = ck
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
                self.io_low = (1 - v) << IO

    def rise(self, c, d):
        self.ones = self.ones + 1 if d else 0
        self.history = ((self.history << 1) | d) & 0xFFFF
        if self.ones >= 50:
            self.state = 'idle' if self.state != 'hunt' or self.resets else 'idle'
            if self.ones == 50:
                self.resets += 1
                self.errors = []        # what came before a line reset no longer matters
            self.pend = []
            return
        st = self.state
        if st == 'hunt':
            return
        if st == 'idle':
            if d:
                self.req = [1]
                self.state = 'req'
            return
        if st == 'req':
            self.req.append(d)
            if len(self.req) == 8:
                r = self.req
                good = (r[1] ^ r[2] ^ r[3] ^ r[4]) == r[5] and r[6] == 0 and r[7] == 1
                self.ap_dp, self.rnw, self.a = r[1], r[2], (r[3] << 2) | (r[4] << 3)
                if not good:
                    self.errors.append(('bad request', r))
                    self.state = 'idle'
                    return
                self.state = 'trn1'
            return
        if st == 'trn1':                # this rise is the turnaround clock; ACK starts after it
            kind = self.inject.pop(0) if self.inject and self.waits == 0 else None
            if kind and isinstance(kind, tuple):
                self.waits = kind[1]
                kind = None
            if self.waits:
                self.waits -= 1
                self.ack = WAIT
            elif kind == 'fault':
                self.ack = FAULT
            elif kind == 'mute':
                self.ack = 0
            else:
                self.ack = OK
            self.bad = kind == 'parity'
            self.n = 0
            self.sent = 0
            if self.ack:
                self.out(c, self.ack & 1)
            self.state = 'ack'
            return
        if st == 'ack':
            self.n += 1
            if self.n < 3:
                if self.ack:
                    self.out(c, (self.ack >> self.n) & 1)
                return
            if self.ack != OK:
                self.out(c, None)
                self.state = 'after'
                self.log.append(('ack', self.ack))
                return
            if self.rnw:
                self.word = self.read()
                self.n = 0
                self.par = 0
                b = self.word & 1
                self.par ^= b
                self.out(c, b)
                self.state = 'rdata'
            else:
                self.out(c, None)
                self.n = 0
                self.word = 0
                self.par = 0
                self.state = 'wtrn'
            return
        if st == 'rdata':
            self.n += 1
            if self.n < 32:
                b = (self.word >> self.n) & 1
                self.par ^= b
                self.out(c, b)
            elif self.n == 32:
                self.out(c, self.par ^ (1 if self.bad else 0))
            else:
                self.out(c, None)
                self.log.append(('r', self.ap_dp, self.a, self.word))
                self.state = 'after'    # the turnaround clock that follows is not a start bit
            return
        if st == 'after':
            self.state = 'idle'
            return
        if st == 'wtrn':
            self.state = 'wdata'
            return
        if st == 'wdata':
            if self.n < 32:
                self.word |= d << self.n
                self.par ^= d
            self.n += 1
            if self.n == 33:
                if d != self.par:
                    self.errors.append(('write parity', self.word))
                self.write()
                self.log.append(('w', self.ap_dp, self.a, self.word))
                self.state = 'idle'
            return

    def read(self):
        if self.ap_dp:                  # AP reads are posted
            v = self.rdbuff
            self.rdbuff = self.ap.get(self.a, 0)
            return v
        if self.a == 0:
            return self.dpidr
        if self.a == 12:
            return self.rdbuff
        return self.regs.get(self.a, 0)

    def write(self):
        if self.ap_dp:
            self.ap[self.a] = self.word
        else:
            self.regs[self.a] = self.word


def swd_decode(ck, io, oe):
    """Pads only: bus level at each SWCLK rise, and whether the chip drove at that rise. Applies the
    wire rules. Every target-owned or turnaround rise must find the chip's enable off, and every
    host-owned rise must find it on."""
    rises = [t for t, v in ck.edges() if v == 1]
    lv = [io.at(t) for t in rises]
    dr = [oe.at(t) for t in rises]
    n = len(rises)
    i, errs, tx = 0, [], []

    def owner(a, b, want, what):
        for k in range(a, min(b, n)):
            if dr[k] != want:
                errs.append('%s: chip enable %d at rise %d' % (what, dr[k], k))
                return
    while i < n:
        if lv[i] == 0:
            i += 1
            continue
        j = i
        while j < n and lv[j]:
            j += 1
        if j - i >= 50:
            owner(i, j, 1, 'line reset')
            tx.append(('reset', j - i))
            i = j
            if i + 16 <= n and sum(lv[i + k] << k for k in range(16)) == 0xE79E:
                tx.append(('jtag2swd',))
                i += 16
            continue
        if i + 13 > n:
            break
        q = lv[i:i + 8]
        owner(i, i + 8, 1, 'request')
        if (q[1] ^ q[2] ^ q[3] ^ q[4]) != q[5] or q[6] != 0 or q[7] != 1:
            errs.append('bad request %s at rise %d' % (q, i))
        owner(i + 8, i + 12, 0, 'turnaround and ACK')
        ack = lv[i + 9] | (lv[i + 10] << 1) | (lv[i + 11] << 2)
        apdp, rnw, a = q[1], q[2], (q[3] << 2) | (q[4] << 3)
        if ack != OK:
            tx.append(('ack', ack, apdp, rnw, a))
            owner(i + 12, i + 13, 0, 'turnaround after ACK')
            i += 13
            continue
        if rnw:
            d = lv[i + 12:i + 44]
            owner(i + 12, i + 46, 0, 'read data and turnaround')
            val = sum(b << k for k, b in enumerate(d))
            bad = sum(d) % 2 != lv[i + 44]
            tx.append(('r', apdp, a, val, bad))
            i += 46
        else:
            owner(i + 12, i + 13, 0, 'write turnaround')
            owner(i + 13, i + 46, 1, 'write data')
            d = lv[i + 13:i + 45]
            val = sum(b << k for k, b in enumerate(d))
            if sum(d) % 2 != lv[i + 45]:
                errs.append('write parity at rise %d' % i)
            tx.append(('w', apdp, a, val))
            i += 46
    return tx, errs


def host_ops(r, ops, got):
    def post(v):
        yield from r.host.post(0, v)

    def agent(half):
        yield from post(half) if False else r.host.post(7, half)
        for n in (16, 16, 16, 8):
            yield from raw(n, [0xFFFF])
        yield from raw(16, [0xE79E])
        for n in (16, 16, 16, 8):
            yield from raw(n, [0xFFFF])
        yield from raw(8, [0])
        for op in ops:
            kind = op[0]
            if kind == 'r':
                yield from post(0x8000 | op[1] | 2 | ((op[2] >> 2) << 2))
                b = []
                yield from r.host.take(1, b)
                st = b[0][1]
                if st in (0, 4):
                    yield from r.host.take(1, b)
                    yield from r.host.take(1, b)
                    got.append((st, b[1][1] | (b[2][1] << 16)))
                else:
                    got.append((st, None))
            else:
                yield from post(0x8000 | op[1] | ((op[2] >> 2) << 2))
                yield from post(op[3] & 0xFFFF)
                yield from post(op[3] >> 16)
                b = []
                yield from r.host.take(1, b)
                got.append((b[0][1], None))

    def raw(n, words):
        yield from post(n)
        for w in words:
            yield from post(w)
    return agent


OPS = [('r', 0, 0), ('w', 0, 4, 0x50000000), ('r', 0, 4), ('w', 0, 8, 0x000000F0), ('r', 1, 0), ('r', 0, 12),
       ('r', 1, 4), ('w', 1, 4, 0xCAFEF00D), ('r', 0, 8)]
WANT = [(0, 0x2BA01477), (0, None), (0, 0x50000000), (0, None), (0, 0), (0, 0x11223344), (0, 0x11223344),
        (0, None), (0, 0x000000F0)]


def case(half, ops=OPS, inject=(), tco=3, thread=0, align=0):
    t = Target(tco)
    t.inject = list(inject)
    r = Rig(devices=[t], pullups=1 << IO)
    img = assemble(src('swd'), depth=128)
    r.load(img, {thread: 'start'})
    got = []
    r.board.agent(host_ops(r, ops, got)(half))
    r.start(align)
    r.run(4000 + len(ops) * (60 * 2 * half + 3000) + 400 * half, stop=r.agents_done)
    r.run(16 * 2 * half + 50)
    tx, er = swd_decode(r.wave('uo', CK), r.wave('io', IO), r.board.trace.wave('iooe', IO))
    return r, t, got, tx, er, img


def clean(r, t, er, conflict=True):
    oe = r.board.trace.wave('iooe', IO)
    # the chip's enable must be off for every cycle the target held the line
    both = [(a, b) for a, b in t.driven if any(oe.at(x) for x in range(a, b))]
    return not er and not r.health() and not t.errors and not both


def main(log):
    log.group('advanced/swd.s', title='SWD host')
    for half in (250, 25, 24):
        r, t, got, tx, er, img = case(half)
        log.check('half %d (%.2f MHz): reset, JTAG-to-SWD, DPIDR, writes, posted AP reads' % (half, 25 / half),
                  got == WANT and clean(r, t, er) and t.resets == 2,
                  '%s %s %s' % ([(s, hex(v) if v is not None else v) for s, v in got[:3]], er[:2], r.health()))
    for half in (23, 22, 21):           # past the limit: negative controls, LATE has to say so
        r, t, got, tx, er, img = case(half)
        late = any('LATE' in h for h in r.health())
        log.check('half %d (%.2f MHz) is past the limit, and LATE reports it' % (half, 25 / half), late, r.health()[:1])
    log.line('INFO  swd is %d words' % img.size)
    ok = 0
    for th in range(4):
        for al in range(4):
            r, t, got, tx, er, img = case(25, thread=th, align=al)
            ok += got == WANT and clean(r, t, er)
    log.check('1 MHz, 4 threads x 4 alignments, contention checked every cycle', ok == 16, '%d of 16' % ok)
    for tco in (0, 3, 15):
        r, t, got, tx, er, img = case(25, tco=tco)
        log.check('target clock-to-out %d cycles' % tco, got == WANT and clean(r, t, er), er[:2])
    r, t, got, tx, er, img = case(25, ops=[('r', 0, 0)], inject=[('wait', 3)])
    log.check('WAIT 3 times, then OK', got == [(0, 0x2BA01477)] and clean(r, t, er) and [x[1] for x in tx if x[0] == 'ack'] == [WAIT] * 3, (got, er[:2]))
    r, t, got, tx, er, img = case(25, ops=[('r', 0, 0), ('r', 0, 0)], inject=[('wait', 20)])
    log.check('WAIT forever: gives up with status 1, bus still usable', got[:1] and got[0][0] == 1 and clean(r, t, er), (got, er[:2]))
    r, t, got, tx, er, img = case(25, ops=[('r', 0, 0), ('r', 0, 0)], inject=['fault'])
    log.check('FAULT reported, next transfer fine', got == [(2, None), (0, 0x2BA01477)] and clean(r, t, er), (got, er[:2]))
    r, t, got, tx, er, img = case(25, ops=[('r', 0, 0), ('r', 0, 0)], inject=['mute'])
    log.check('no response reported as status 3', got[:1] and got[0][0] == 3, (got, er[:2]))
    r, t, got, tx, er, img = case(25, ops=[('r', 0, 0), ('r', 0, 0)], inject=['parity'])
    log.check('bad read parity reported as status 4, next transfer fine', got[:1] and got[0][0] == 4 and got[1:2] == [(0, 0x2BA01477)], (got, er[:2]))
    r, t, got, tx, er, img = case(25, ops=[('w', 0, 4, 0xFFFFFFFF), ('r', 0, 4), ('w', 0, 4, 0), ('r', 0, 4), ('w', 0, 4, 0x80000001), ('r', 0, 4)])
    log.check('write parity for 0xFFFFFFFF, 0, 0x80000001', [g[1] for g in got[1::2]] == [0xFFFFFFFF, 0, 0x80000001] and clean(r, t, er), (got, er[:2]))
