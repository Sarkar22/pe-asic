#!/usr/bin/env python3
"""Cycle model of ISA v0.1, written by hand from sections 2-5, 6.3, 8 and 9.

Names follow the document so each function can be read against its code block.
The order inside one cycle is section 3.2. Row semantics are hand-written here.

The shift units of section 5.9 are not in this model yet. Their eight rows raise
NotImplementedError, and nothing else differs: while no unit runs, no pin is on a
shift output and every unit reads as after reset, so RDREG 12 to 31 read 0 here
and ABORT has nothing of a unit to stop.
"""
import random

from asm import ROWS, decode

TICK, WAIT = 0, 1
LOAD, STORE = 0, 1
LANES = (0, 4, 5, 6, 7)
SRC = {0: 3, 4: 4, 5: 5, 6: 6, 7: 7}
HOLD, PCSET = 1, 2                      # run_row outcomes; None retires with PC := next


class Thread:
    def __init__(self, tid):
        self.TID = tid
        self.PC = 0
        self.R = [0] * 8
        self.T = 0
        self.D = 0
        self.S = 0
        self.LATE = self.STALE = self.ILL = self.RUN = 0
        self.P = self.KIND = self.DUE = self.WAITING = 0
        self.MASK = self.PAT = self.BANK = self.NE = self.TIMED = self.INBOX = 0
        self.MEM = self.MEM_DONE = self.MEM_KIND = self.MEM_REG = 0
        self.MEM_ADDR = self.MEM_DATA = 0
        self.STOPREQ = self.STEPPING = 0
        self.IR = self.IV = 0
        self.HIT = self.D_WRITTEN = 0
        self.retired = 0                # not architectural: instruction count
        self.late_at = None             # not architectural: first cycle LATE rose


class Chip:
    def __init__(self, depth=128, seed=1):
        self.DEPTH = depth
        rng = random.Random(seed)       # section 10: unwritten memory is arbitrary
        self.mem = [rng.getrandbits(16) for _ in range(depth)]
        self.th = [Thread(i) for i in range(4)]
        self.c = 0
        self.M = [0] * 8
        self.FULL = [0] * 8
        self.SYNC1 = self.SYNC2 = 0
        self.OUT = [0] * 16
        self.SHD = [0] * 16
        self.OE = [0] * 8
        self.SHOE = [0] * 8
        self.OD = [0] * 8
        self.BYP = [0] * 8              # only LANES are used
        self.SHBYP = [0] * 8
        self.OWN = [0] * 16
        self.BUSY = 0
        self.host_miso = 0
        self._dirty = True
        self._pads = (0, 0, 0)
        self.log = None                 # optional list of (cycle, tid, pc, word) at execute

    # ---- 3.4 arming -------------------------------------------------------
    def schedule(self, t, step, a):
        if t.STALE:
            t.D = (a + 1) & 0xFFFF
            t.LATE = 1
        else:
            t.D = (t.D + step) & 0xFFFF
        t.D_WRITTEN = 1
        delta = (t.D - a) % 65536
        t.DUE = 1 if delta == 0 or delta >= 0x8000 else 0

    def arm(self, t, step, a):
        self.schedule(t, step, a)
        t.P = 1
        t.KIND = TICK

    def now(self, t, a):
        t.D = (a + 1) & 0xFFFF
        t.D_WRITTEN = 1
        t.P = 1
        t.KIND = TICK
        t.DUE = 0

    def wait(self, t, mask, pat, bank, ne, rt, inbox, a):
        t.MASK = mask & 255
        t.PAT = pat & 255
        t.BANK = bank
        t.NE = ne
        t.INBOX = inbox
        t.TIMED = rt is not None
        t.DUE = 0
        if t.TIMED:
            self.schedule(t, rt, a)
        t.P = 1
        t.KIND = WAIT
        t.WAITING = 1
        t.PC = self.next(t)

    # ---- 3.4 firing -------------------------------------------------------
    def events(self, t, TB):
        if t.KIND == TICK:
            if t.DUE or t.D == TB:
                if t.DUE:
                    t.LATE = 1
                self.commit(t)
                t.S = self.SYNC2
                t.P = 0
        else:
            byte = (self.SYNC2 >> (8 * t.BANK)) & 255
            match = ((byte ^ t.PAT) & t.MASK) == 0
            if t.NE:
                match = not match
            if match:
                self.commit(t)
                t.S = self.SYNC2
                t.D = TB
                t.D_WRITTEN = 1
                t.T = 1
                t.P = 0
            elif t.INBOX and self.FULL[t.TID]:
                t.S = self.SYNC2
                t.D = TB
                t.D_WRITTEN = 1
                t.T = 0
                t.P = 0
            elif t.TIMED and (t.DUE or t.D == TB):
                t.S = self.SYNC2
                t.T = 0
                t.P = 0

    # ---- 3.5 --------------------------------------------------------------
    @staticmethod
    def update_stale(t):
        if t.D_WRITTEN:
            t.STALE = 0
        elif t.HIT:
            t.STALE = 1
        t.D_WRITTEN = 0

    # ---- 3.6 --------------------------------------------------------------
    def next(self, t):
        return (t.PC + 1) % self.DEPTH

    def slot(self, t, P_start, a):
        if not t.RUN:
            pass
        elif t.MEM and t.MEM_DONE:
            self.finish_mem(t)
        elif t.STOPREQ:
            self.stop(t)
        elif not t.IV:
            pass
        elif t.WAITING:
            if not P_start:
                t.WAITING = 0
                self.retire(t)
                if t.RUN:
                    self.execute(t, t.IR, 0, a)
        else:
            self.execute(t, t.IR, P_start, a)

    def execute(self, t, word, P_start, a):
        d = decode(word)
        if d is None:
            t.ILL = 1
            self.stop(t)
            return
        row, f = d
        w = row.w == 'y' or (row.w == 'pin' and (f['i'] != 0 or f['o'] != 3))
        if w and P_start:
            return
        if self.log is not None:
            self.log.append((self.c, t.TID, t.PC, word))
        out = row.fn(self, t, f, a)
        if out == HOLD:
            return
        if out != PCSET:
            t.PC = self.next(t)
        self.retire(t)

    @staticmethod
    def stop(t):
        t.RUN = 0
        if not t.P:
            t.WAITING = 0
        t.P = 0
        t.STOPREQ = 0
        t.STEPPING = 0

    @staticmethod
    def retire(t):
        t.retired += 1
        if t.STEPPING:
            t.RUN = 0
            t.STEPPING = 0
            if t.STOPREQ:
                t.P = 0
            t.STOPREQ = 0

    # ---- 4.2 --------------------------------------------------------------
    def memory(self, t):
        if t.MEM:
            if not t.MEM_DONE:
                if t.MEM_KIND == LOAD:
                    t.MEM_DATA = self.mem[t.MEM_ADDR]
                else:
                    self.mem[t.MEM_ADDR] = t.MEM_DATA
                t.MEM_DONE = 1
        elif t.RUN:
            t.IR = self.mem[t.PC]
            t.IV = 1
        else:
            t.IV = 0

    def load(self, t, d, A):
        t.MEM = 1
        t.MEM_KIND = LOAD
        t.MEM_REG = d
        t.MEM_ADDR = A % self.DEPTH

    def store(self, t, V, A):
        t.MEM = 1
        t.MEM_KIND = STORE
        t.MEM_DATA = V
        t.MEM_ADDR = A % self.DEPTH

    def finish_mem(self, t):
        if t.MEM_KIND == LOAD:
            t.R[t.MEM_REG] = t.MEM_DATA
        t.MEM = 0
        t.MEM_DONE = 0
        t.PC = self.next(t)
        self.retire(t)

    # ---- 5.3 / 5.4 --------------------------------------------------------
    def shadow(self, t, p, v):
        if p != 3 and self.OWN[p] == t.TID:
            self.SHD[p] = v

    def shadow_oe(self, t, p, v):
        if self.OWN[p] == t.TID and not self.OD[p - 8]:
            self.SHOE[p - 8] = v

    def put8(self, t, b, v):
        for i in range(8):
            self.shadow(t, 8 * b + i, (v >> i) & 1)

    def oe8(self, t, v):
        for i in range(8):
            self.shadow_oe(t, 8 + i, (v >> i) & 1)

    def commit(self, t):
        tid = t.TID
        OWN, OUT, SHD = self.OWN, self.OUT, self.SHD
        for p in range(16):
            if OWN[p] == tid:
                OUT[p] = SHD[p]
                if p >= 8:
                    self.OE[p - 8] = self.SHOE[p - 8]
                elif p in LANES:
                    self.BYP[p] = self.SHBYP[p]
        self._dirty = True

    def pcfg_in(self, t, p):
        self.OWN[p] = t.TID
        self.OD[p - 8] = 0
        self.shadow_oe(t, p, 0)
        self._dirty = True

    def pcfg_pp(self, t, p):
        self.OWN[p] = t.TID
        if p >= 8:
            self.OD[p - 8] = 0
            self.shadow_oe(t, p, 1)
        elif p in LANES:
            self.SHBYP[p] = 0
        self._dirty = True

    def pcfg_od(self, t, p):
        self.OWN[p] = t.TID
        self.OD[p - 8] = 1
        self.OE[p - 8] = 0
        self.SHOE[p - 8] = 0
        self.OUT[p] = 1
        self.SHD[p] = 1
        self._dirty = True

    def pcfg_byp(self, t, p):
        self.OWN[p] = t.TID
        self.SHBYP[p] = 1

    # ---- 5.2 --------------------------------------------------------------
    def pads(self, ui_in):
        """uo_out, uio_out, uio_oe for the cycle whose ui_in is given.

        The bypass path reads ui_in of this same cycle, so there is no lag.
        """
        if not self._dirty and not any(self.BYP):
            return self._pads
        OUT, OD = self.OUT, self.OD
        uo = uio = oe = 0
        for i in range(8):
            if i == 3:
                b = self.host_miso
            elif self.BYP[i]:
                b = (ui_in >> SRC[i]) & 1
            else:
                b = OUT[i]
            uo |= b << i
            if OD[i]:
                oe |= (1 - OUT[8 + i]) << i
            else:
                uio |= OUT[8 + i] << i
                oe |= self.OE[i] << i
        self._pads = (uo, uio, oe)
        self._dirty = False
        return self._pads

    # ---- 3.2 --------------------------------------------------------------
    def cycle(self, ui_in, uio_in):
        """Steps 2 to 5 of one cycle. Step 1 is pads() plus the board."""
        c = self.c
        TB = c & 0xFFFF
        th = self.th
        P_start = [t.P for t in th]
        for t in th:
            t.HIT = TB == (t.D ^ 0x8000)
        for t in th:
            if t.P:
                self.events(t, TB)
        i = c & 3
        self.slot(th[i], P_start[i], TB)
        self.memory(th[(i - 1) & 3])
        for t in th:
            self.update_stale(t)
            if t.LATE and t.late_at is None:
                t.late_at = c
        self.SYNC2 = self.SYNC1
        self.SYNC1 = (uio_in << 8) | ui_in
        self.c = c + 1

    # ---- 9: byte-level host commands. Call between cycles: h = c - 1 --------
    # thread8: only its low 2 bits count (9.2)
    def host_wrmem(self, addr, data):
        if all(t.RUN for t in self.th):
            self.BUSY = 1
            return False
        self.mem[addr % self.DEPTH] = data & 0xFFFF
        return True

    def host_rdmem(self, addr):
        if all(t.RUN for t in self.th):
            self.BUSY = 1
            return 0
        return self.mem[addr % self.DEPTH]

    def host_setpc(self, tid, pc):
        t = self.th[tid & 3]
        if t.RUN:
            self.BUSY = 1
            return
        t.PC = pc % self.DEPTH

    def host_run(self, stop, start):
        h = (self.c - 1) & 0xFFFF
        for t in self.th:
            s, g = (stop >> t.TID) & 1, (start >> t.TID) & 1
            if s:
                if t.RUN:
                    t.STOPREQ = 1
                else:
                    t.P = 0
            elif g and not t.RUN:
                t.RUN = 1
                t.D = h
                t.D_WRITTEN = 1
                t.P = 0
                t.WAITING = 0
                t.STOPREQ = 0
                t.LATE = 0
                t.ILL = 0
                t.STALE = 0

    def host_step(self, tid):
        t = self.th[tid & 3]
        if t.RUN:
            self.BUSY = 1
            return
        t.RUN = 1
        t.STEPPING = 1
        t.WAITING = 0

    def host_rdreg(self, tid, reg):
        t = self.th[tid & 3]
        if t.RUN:
            self.BUSY = 1
            return 0
        if reg < 8:
            return t.R[reg]
        return {8: t.D, 9: t.S, 10: t.PC,
                11: t.T | (t.LATE << 1) | (t.STALE << 2) | (t.WAITING << 3)}.get(reg, 0)

    def host_post(self, n, data):
        """Result bit 0 is 1 if refused."""
        if self.FULL[n]:
            return 1
        self.M[n] = data & 0xFFFF
        self.FULL[n] = 1
        return 0

    def host_take(self, n):
        if self.FULL[n]:
            self.FULL[n] = 0
            return 1, self.M[n]
        return 0, 0

    def host_status(self):
        def nib(name):
            return sum(getattr(t, name) << t.TID for t in self.th)
        full = sum(f << i for i, f in enumerate(self.FULL))
        out = [(nib('RUN') << 4) | nib('ILL'), (nib('LATE') << 4) | nib('STALE'),
               full, self.BUSY]
        self.BUSY = 0
        return out

    def host_abort(self):
        for t in self.th:
            t.P = 0
            t.WAITING = 0
            if t.RUN:
                t.STOPREQ = 1
        for i in range(8):
            self.OE[i] = 0
            self.SHOE[i] = 0
            if self.OD[i]:
                self.OUT[8 + i] = 1
                self.SHD[8 + i] = 1
        self._dirty = True


# ---- 6.3: one handler per row. fn(chip, thread, fields, a) ------------------
H = {}


def op(*names):
    def deco(fn):
        for n in names:
            H[n] = fn
        return fn
    return deco


op('idle')(lambda c, t, f, a: None)


@op('halt')
def _halt(c, t, f, a):
    t.PC = c.next(t)
    c.stop(t)
    return PCSET


op('now')(lambda c, t, f, a: c.now(t, a))


@op('clrt')
def _clrt(c, t, f, a):
    t.T = 0


@op('sett')
def _sett(c, t, f, a):
    t.T = 1


@op('nott')
def _nott(c, t, f, a):
    t.T = 1 - t.T


@op('tlate')
def _tlate(c, t, f, a):
    t.T = t.LATE
    t.LATE = 0


@op('jr Rn')
def _jr(c, t, f, a):
    t.PC = t.R[f['n']] % c.DEPTH
    return PCSET


@op('swap Rn')
def _swap(c, t, f, a):
    v = t.R[f['n']]
    t.R[f['n']] = ((v >> 8) | (v << 8)) & 0xFFFF


@op('not Rn')
def _not(c, t, f, a):
    t.R[f['n']] = ~t.R[f['n']] & 0xFFFF


@op('rds Rn')
def _rds(c, t, f, a):
    t.R[f['n']] = t.S


@op('rdd Rn')
def _rdd(c, t, f, a):
    t.R[f['n']] = t.D


@op('wrd Rn')
def _wrd(c, t, f, a):
    t.D = t.R[f['n']]
    t.D_WRITTEN = 1


@op('rdtb Rn')
def _rdtb(c, t, f, a):
    t.R[f['n']] = a


op('put8 Rn, b')(lambda c, t, f, a: c.put8(t, f['b'], t.R[f['n']]))
op('oe8 Rn')(lambda c, t, f, a: c.oe8(t, t.R[f['n']]))


@op('tid Rn')
def _tid(c, t, f, a):
    t.R[f['n']] = t.TID


def _setpc(c, t, v):
    t.PC = v % c.DEPTH
    return PCSET


op('goto a')(lambda c, t, f, a: _setpc(c, t, f['a']))
op('jt a')(lambda c, t, f, a: _setpc(c, t, f['a']) if t.T else None)
op('jf a')(lambda c, t, f, a: _setpc(c, t, f['a']) if not t.T else None)


@op('call a')
def _call(c, t, f, a):
    t.R[7] = (t.PC + 1) % c.DEPTH
    return _setpc(c, t, f['a'])


@op('djnz Rn, a')
def _djnz(c, t, f, a):
    n = f['n']
    t.R[n] = (t.R[n] - 1) & 0xFFFF
    if t.R[n] != 0:
        return _setpc(c, t, f['a'])


def _alu(fn):
    def h(c, t, f, a):
        t.R[f['n']] = fn(t.R[f['n']], f['i']) & 0xFFFF
    return h


op('ldi Rn, i')(_alu(lambda x, i: i))
op('lui Rn, i')(_alu(lambda x, i: (x & 0xFF) | (i << 8)))
op('addi Rn, i')(_alu(lambda x, i: x + i))
op('subi Rn, i')(_alu(lambda x, i: x - i))
op('andi Rn, i')(_alu(lambda x, i: x & i))
op('ori Rn, i')(_alu(lambda x, i: x | i))


@op('cmpi Rn, i')
def _cmpi(c, t, f, a):
    t.T = 1 if t.R[f['n']] == f['i'] else 0


@op('tsti Rn, i')
def _tsti(c, t, f, a):
    t.T = 1 if t.R[f['n']] & f['i'] else 0


def _alr(fn):
    def h(c, t, f, a):
        t.R[f['d']] = fn(t.R[f['d']], t.R[f['s']]) & 0xFFFF
    return h


op('cp Rd, Rs')(_alr(lambda d, s: s))
op('add Rd, Rs')(_alr(lambda d, s: d + s))
op('sub Rd, Rs')(_alr(lambda d, s: d - s))
op('and Rd, Rs')(_alr(lambda d, s: d & s))
op('or Rd, Rs')(_alr(lambda d, s: d | s))
op('xor Rd, Rs')(_alr(lambda d, s: d ^ s))


@op('xort Rd, Rs')
def _xort(c, t, f, a):
    if t.T:
        t.R[f['d']] ^= t.R[f['s']]


@op('crcr Rd, Rs')
def _crcr(c, t, f, a):
    d = f['d']
    fb = t.T ^ (t.R[d] & 1)
    t.R[d] >>= 1                        # statements run in order: Rd = Rs sees the shift
    if fb:
        t.R[d] ^= t.R[f['s']]


@op('crcl Rd, Rs')
def _crcl(c, t, f, a):
    d = f['d']
    fb = t.T ^ ((t.R[d] >> 15) & 1)
    t.R[d] = (t.R[d] << 1) & 0xFFFF
    if fb:
        t.R[d] ^= t.R[f['s']]


@op('cmp Rd, Rs')
def _cmp(c, t, f, a):
    t.T = 1 if t.R[f['d']] == t.R[f['s']] else 0


@op('lt Rd, Rs')
def _lt(c, t, f, a):
    t.T = 1 if t.R[f['d']] < t.R[f['s']] else 0


@op('bt Rn, b')
def _bt(c, t, f, a):
    t.T = (t.R[f['n']] >> f['b']) & 1


@op('bn Rn, b')
def _bn(c, t, f, a):
    t.T = 1 - ((t.R[f['n']] >> f['b']) & 1)


@op('bx Rn, b')
def _bx(c, t, f, a):
    t.T ^= (t.R[f['n']] >> f['b']) & 1


@op('ba Rn, b')
def _ba(c, t, f, a):
    t.T &= (t.R[f['n']] >> f['b']) & 1


@op('bo Rn, b')
def _bo(c, t, f, a):
    t.T |= (t.R[f['n']] >> f['b']) & 1


@op('bs Rn, b')
def _bs(c, t, f, a):
    n, b = f['n'], f['b']
    t.R[n] = (t.R[n] & ~(1 << b) & 0xFFFF) | (t.T << b)


op('put p, 0')(lambda c, t, f, a: c.shadow(t, f['p'], 0))
op('put p, 1')(lambda c, t, f, a: c.shadow(t, f['p'], 1))
op('put p, T')(lambda c, t, f, a: c.shadow(t, f['p'], t.T))
op('put p, !T')(lambda c, t, f, a: c.shadow(t, f['p'], 1 - t.T))
op('put p, ^T')(lambda c, t, f, a: c.shadow(t, f['p'], c.SHD[f['p']] ^ t.T))
op('put p, ^!T')(lambda c, t, f, a: c.shadow(t, f['p'], c.SHD[f['p']] ^ t.T ^ 1))
op('oe p, v')(lambda c, t, f, a: c.shadow_oe(t, f['p'], f['v']))
op('pcfg p, IN')(lambda c, t, f, a: c.pcfg_in(t, f['p']))
op('pcfg p, PP')(lambda c, t, f, a: c.pcfg_pp(t, f['p']))
op('pcfg p, OD')(lambda c, t, f, a: c.pcfg_od(t, f['p']))
op('pcfg p, BYP')(lambda c, t, f, a: c.pcfg_byp(t, f['p']))


@op('get p')
def _get(c, t, f, a):
    t.T = (t.S >> f['p']) & 1


@op('getn p')
def _getn(c, t, f, a):
    t.T = 1 - ((t.S >> f['p']) & 1)


@op('xget p')
def _xget(c, t, f, a):
    t.T ^= (t.S >> f['p']) & 1


@op('aget p')
def _aget(c, t, f, a):
    t.T &= (t.S >> f['p']) & 1


@op('recv Rn, m')
def _recv(c, t, f, a):
    m = f['m']
    if not c.FULL[m]:
        return HOLD
    t.R[f['n']] = c.M[m]
    c.FULL[m] = 0


@op('send Rn, m')
def _send(c, t, f, a):
    m = f['m']
    if c.FULL[m]:
        return HOLD
    c.M[m] = t.R[f['n']]
    c.FULL[m] = 1


@op('tryrecv Rn, m')
def _tryrecv(c, t, f, a):
    m = f['m']
    t.T = c.FULL[m]
    if t.T:
        t.R[f['n']] = c.M[m]
        c.FULL[m] = 0


@op('trysend Rn, m')
def _trysend(c, t, f, a):
    m = f['m']
    t.T = 1 - c.FULL[m]
    if t.T:
        c.M[m] = t.R[f['n']]
        c.FULL[m] = 1


@op('ld Rd, Ra')
def _ld(c, t, f, a):
    c.load(t, f['d'], t.R[f['a']])
    return HOLD


@op('ldp Rd, Ra')
def _ldp(c, t, f, a):
    c.load(t, f['d'], t.R[f['a']])
    t.R[f['a']] = (t.R[f['a']] + 1) & 0xFFFF
    return HOLD


@op('st Rd, Ra')
def _st(c, t, f, a):
    c.store(t, t.R[f['d']], t.R[f['a']])
    return HOLD


@op('stp Rd, Ra')
def _stp(c, t, f, a):
    c.store(t, t.R[f['d']], t.R[f['a']])
    t.R[f['a']] = (t.R[f['a']] + 1) & 0xFFFF
    return HOLD


@op('until p, l [, inbox]')
def _until(c, t, f, a):
    p = f['p']
    c.wait(t, 1 << (p & 7), f['l'] << (p & 7), p >> 3, 0, None, f['m'], a)
    return HOLD


@op('until p, l, Rt [, inbox]')
def _until_t(c, t, f, a):
    p = f['p']
    c.wait(t, 1 << (p & 7), f['l'] << (p & 7), p >> 3, 0, t.R[f['t']], f['m'], a)
    return HOLD


def _untilm(c, t, f, a, rt):
    r, b, m = t.R[f['r']], f['b'], f['c']
    pat = (t.S >> (8 * b)) & 255 if m == 2 else r & 255
    c.wait(t, r >> 8, pat, b, m != 0, rt, f['m'], a)
    return HOLD


op('untilm Rr, b, c [, inbox]')(lambda c, t, f, a: _untilm(c, t, f, a, None))
op('untilm Rr, b, c, Rt [, inbox]')(lambda c, t, f, a: _untilm(c, t, f, a, t.R[f['t']]))
op('tick i')(lambda c, t, f, a: c.arm(t, f['i'], a))


def _eact(t, v):
    return (0, 1, t.T, 1 - t.T)[v]


@op('edge p, v, i')
def _edge_i(c, t, f, a):
    c.shadow(t, f['p'], _eact(t, f['v']))
    c.arm(t, f['i'], a)


@op('edge p, v, Rr')
def _edge_r(c, t, f, a):
    c.shadow(t, f['p'], _eact(t, f['v']))
    c.arm(t, t.R[f['r']], a)


@op('shl8 Rr, i, o')
def _shl8(c, t, f, a):
    if f['i'] != 0:
        t.T = (t.S >> f['i']) & 1
    r = f['r']
    v = t.R[r]
    out = (v >> 7) & 1
    t.R[r] = (v & 0xFF00) | ((v << 1) & 0xFE) | t.T
    t.T = out
    c.shadow(t, f['o'], t.T)


@op('shl16 Rr, i, o')
def _shl16(c, t, f, a):
    if f['i'] != 0:
        t.T = (t.S >> f['i']) & 1
    r = f['r']
    v = t.R[r]
    out = (v >> 15) & 1
    t.R[r] = ((v << 1) | t.T) & 0xFFFF
    t.T = out
    c.shadow(t, f['o'], t.T)


@op('shr8 Rr, i, o')
def _shr8(c, t, f, a):
    if f['i'] != 0:
        t.T = (t.S >> f['i']) & 1
    r = f['r']
    v = t.R[r]
    out = v & 1
    t.R[r] = (v & 0xFF00) | ((v & 0xFF) >> 1) | (t.T << 7)
    t.T = out
    c.shadow(t, f['o'], t.T)


@op('shr16 Rr, i, o')
def _shr16(c, t, f, a):
    if f['i'] != 0:
        t.T = (t.S >> f['i']) & 1
    r = f['r']
    v = t.R[r]
    out = v & 1
    t.R[r] = (v >> 1) | (t.T << 15)
    t.T = out
    c.shadow(t, f['o'], t.T)


PENDING = ('txw Rs, n [, k] [, last]', 'scfg Rs, k', 'smap p, l', 'rxw Rd', 'sstat Rd', 'srd Rd, s', 'sgo m',
           'sstop m')                   # section 5.9.4: the shift unit's rows


@op(*PENDING)
def _shift_unit(c, t, f, a):
    raise NotImplementedError('shift unit: hand-written model pending')


for _r in ROWS:
    _r.fn = H[_r.syntax]
if len(H) != len(ROWS):
    raise ValueError('handlers without a row: %s' % (set(H) - {r.syntax for r in ROWS}))
