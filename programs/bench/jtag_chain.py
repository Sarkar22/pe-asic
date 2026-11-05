"""JTAG against an IEEE 1149.1 TAP chain. TMS and TDI are sampled on TCK rise, TDO moves on fall."""
import random
from protolib import *  # noqa: F401,F403
PROGRAMS = ('advanced/jtag.s',)

TLR, RTI, SDS, CDR, SDR, E1D, PDR, E2D, UDR, SIS, CIR, SIR, E1I, PIR, E2I, UIR = range(16)
NEXT = {TLR: (RTI, TLR), RTI: (RTI, SDS), SDS: (CDR, SIS), CDR: (SDR, E1D), SDR: (SDR, E1D), E1D: (PDR, UDR),
        PDR: (PDR, E2D), E2D: (SDR, UDR), UDR: (RTI, SDS), SIS: (CIR, TLR), CIR: (SIR, E1I), SIR: (SIR, E1I),
        E1I: (PIR, UIR), PIR: (PIR, E2I), E2I: (SIR, UIR), UIR: (RTI, SDS)}
TCK, TMS, TDI, TDO = 1, 2, 5, 4


class Tap:
    def __init__(self, irlen, idcode, bypass_code=None):
        self.irlen, self.idcode = irlen, idcode
        self.bypass = (1 << irlen) - 1
        self.idcode_ir = 1
        self.st = TLR
        self.ir = self.idcode_ir
        self.irsh = 0
        self.dr = 0
        self.drlen = 32
        self.tdo = 1

    def clock(self, tms, tdi):
        s = self.st
        out = None
        if s == SIR:
            out = self.irsh & 1
            self.irsh = (self.irsh >> 1) | (tdi << (self.irlen - 1))
        elif s == SDR:
            out = self.dr & 1
            self.dr = (self.dr >> 1) | (tdi << (self.drlen - 1))
        elif s == UIR:
            pass
        self.st = n = NEXT[s][tms]
        if n == TLR:
            self.ir = self.idcode_ir
        elif n == CIR:
            self.irsh = 1
        elif n == UIR:
            self.ir = self.irsh
        elif n == CDR:
            if self.ir == self.idcode_ir:
                self.dr, self.drlen = self.idcode, 32
            else:
                self.dr, self.drlen = 0, 1
        return n

    def head(self):
        if self.st == SIR:
            return self.irsh & 1
        if self.st == SDR:
            return self.dr & 1
        return 1


class Chain(Device):
    def __init__(self, taps):
        self.taps = taps
        self.p = 0
        self.ui_mask = 1 << TDO
        self.ui_val = self.ui_mask
        self.resets = 0

    def step(self, c, uo, io):
        tck, tms, tdi = (uo >> TCK) & 1, (uo >> TMS) & 1, (uo >> TDI) & 1
        if tck and not self.p:
            d = tdi
            # all flops move together: take every head first
            heads = [t.head() for t in self.taps]
            for t, h in zip(self.taps, [tdi] + heads[:-1]):
                t.clock(tms, h)
        elif self.p and not tck:
            self.ui_val = self.taps[-1].head() << TDO
        self.p = tck


def tap_decode(tck, tms, tdi, tdo, su=1, hd=1):
    """Pads only. Each scan: (kind, tdi bits, tdo bits). TDO bit is what a controller sampling at
    the rise sees. Checks TMS and TDI stable around every rise."""
    st = TLR
    scans, errs = [], []
    cur = None
    ed = sorted([t for t, v in tms.edges()] + [t for t, v in tdi.edges()])
    rises = [t for t, v in tck.edges() if v == 1]
    import bisect
    su_min, hd_min = 10 ** 9, 10 ** 9
    for r in rises:
        i = bisect.bisect_right(ed, r)
        if i:
            su_min = min(su_min, r - ed[i - 1])
        if i < len(ed):
            hd_min = min(hd_min, ed[i] - r)
        m, di, do = tms.at(r), tdi.at(r), tdo.at(r)
        if st in (SDR, SIR):
            if cur is None:
                cur = ['D' if st == SDR else 'I', [], []]
            cur[1].append(di)
            cur[2].append(do)
        st2 = NEXT[st][m]
        if st in (SDR, SIR) and st2 not in (SDR, SIR):
            pass
        if st2 in (UDR, UIR) and cur is not None:
            scans.append(cur)
            cur = None
        st = st2
    if su_min < su:
        errs.append('setup %d' % su_min)
    if hd_min < hd:
        errs.append('hold %d' % hd_min)
    return scans, errs, st, (su_min, hd_min)


class Probe:
    """Python host of the scan engine."""

    def __init__(self, r):
        self.r, self.got = r, []

    def tms(self, bits):
        for i in range(0, len(bits), 16):
            ch = bits[i:i + 16]
            yield from self.r.host.post(0, len(ch))
            yield from self.r.host.post(0, sum(b << k for k, b in enumerate(ch)))

    def scan(self, bits, exit_=True):
        """returns TDO bits via self.out"""
        out = []
        for i in range(0, len(bits), 16):
            ch = bits[i:i + 16]
            last = i + 16 >= len(bits)
            yield from self.r.host.post(0, 0x8000 | (0x4000 if last and exit_ else 0) | len(ch))
            yield from self.r.host.post(0, sum(b << k for k, b in enumerate(ch)))
            box = []
            yield from self.r.host.take(1, box)
            w = box[0][1]
            out += [(w >> (16 - len(ch) + k)) & 1 for k in range(len(ch))]
        self.out = out


def bitsof(v, n):
    return [(v >> i) & 1 for i in range(n)]


def val(b):
    return sum(x << i for i, x in enumerate(b))


def case(half, thread=0, align=0, ids=(0x4BA00477, 0x0BB11477), seed=1):
    taps = [Tap(4, ids[0]), Tap(5, ids[1])]
    ch = Chain(taps)
    r = Rig(devices=[ch])
    img = assemble(src('jtag'), depth=128)
    r.load(img, {thread: 'start'})
    p = Probe(r)
    res = {}
    rng = random.Random(seed)
    pat = [rng.randrange(2) for _ in range(40)]

    def agent():
        yield from r.host.post(7, half)
        yield from p.tms([1, 1, 1, 1, 1, 0])    # reset, idle
        yield from p.tms([1, 0, 0])             # to Shift-DR
        yield from p.scan([0] * 64)
        res['ids'] = (val(p.out[:32]), val(p.out[32:]))
        yield from p.tms([1, 0])                # update, idle
        yield from p.tms([1, 1, 0, 0])          # to Shift-IR: dev1 (last in chain) IR comes first out
        ir = bitsof((1 << 4) - 1, 4) + bitsof((1 << 5) - 1, 5)   # BYPASS in both
        yield from p.scan(ir)
        res['ircap'] = list(p.out)
        yield from p.tms([1, 0])
        yield from p.tms([1, 0, 0])
        yield from p.scan(pat)
        res['pat'] = list(p.out)
        yield from p.tms([1, 0])
        # pause in the middle of a scan and go on
        yield from p.tms([1, 0, 0])
        yield from p.scan(pat[:10], exit_=True)
        res['p1'] = list(p.out)
        yield from p.tms([0, 1, 0])             # Pause-DR x1, Exit2, back to Shift-DR
        yield from p.scan(pat[10:20])
        res['p2'] = list(p.out)
        yield from p.tms([1, 0])
    r.board.agent(agent())
    r.start(align)
    r.run(600 * 2 * half + 80000, stop=r.agents_done)
    r.run(4 * half + 200)
    return r, res, pat, img, taps


def main(log):
    log.group('advanced/jtag.s', title='JTAG scan engine')
    for half in (250, 25, 12, 10, 9):
        r, res, pat, img, taps = case(half)
        tr = [r.wave('uo', TCK), r.wave('uo', TMS), r.wave('uo', TDI), r.wave('ui', TDO)]
        sc, er, st, sh = tap_decode(*tr)
        ok = res.get('ids') == (0x0BB11477, 0x4BA00477) and res['ircap'][:5] == [1, 0, 0, 0, 0] and res['ircap'][5:] == [1, 0, 0, 0]
        ok = ok and res['pat'][2:] == pat[:-2] and not er and not r.health()
        ok = ok and st == RTI and [s[0] for s in sc] == ['D', 'I', 'D', 'D'] and len(sc[3][1]) == 20
        log.check('half %d (%.2f MHz): 2-TAP chain IDCODEs, IR, 40 bits through BYPASS x2, Pause-DR mid scan' % (half, 25 / half), ok,
                  '%s setup %d hold %d %s %s' % ([hex(x) for x in res.get('ids', ())], sh[0], sh[1], er[:2], r.health()))
    log.line('INFO  jtag is %d words' % img.size)
    ok = 0
    for th in range(4):
        for al in range(4):
            r, res, pat, img, taps = case(12, th, al)
            sc, er, st, sh = tap_decode(r.wave('uo', TCK), r.wave('uo', TMS), r.wave('uo', TDI), r.wave('ui', TDO))
            ok += res.get('ids') == (0x0BB11477, 0x4BA00477) and res['pat'][2:] == pat[:-2] and not er and not r.health()
    log.check('half 12, 4 threads x 4 alignments', ok == 16, '%d of 16' % ok)
