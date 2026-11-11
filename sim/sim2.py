#!/usr/bin/env python3
"""ISA v0.1, a second cycle model, written by hand from the document like sim.py but apart from it.
Instruction decode and semantics come from the document's table through isatable.py. Everything
here is the cycle model, the event rules, the memory cycles, the pin layer and the host commands.

Order inside cycle c, all reads seeing start-of-cycle flop values, except stop(), which reads P as step 2 left it:
 1 pads settle, the board answers
 2 event logic of every thread
 2b every thread's shift unit: tx_step(), then rx_step()
 3 the thread owning cycle c executes (it sees its pending flag as it was before step 2)
 4 the thread owning memory cycle c fetches or does its data access
 5 flops update, CMT := 0, SYNC3 := SYNC2, c += 1. Host calls run between cycles, so they follow the thread of that cycle.

Shift units (section 5.9). The unit itself is not written here: its code blocks, 5.9.4 to 5.9.7, are compiled from
the document's text, so sim.py is the only simulator whose unit is written by hand. This compiler makes every name in
capitals an attribute of one unit object, S.NAME; docsim.py's compiler maps each name on its own. Written by hand
here: the unit object (the state names of 5.9.2, the fields of 5.9.3 as views of CFG, the globals the blocks read),
the step order with step 2b, the step-3 rule of section 3.2, the pin layer's output switch (claim, commit, smap and
V(p) in the pads), and the host's RDREG 12 to 31 and ABORT additions (9.2, 9.3). The step-3 rule is done another
way than in docsim.py: step 3 runs before step 2b, on the unit as it was at the start of the cycle, its writes to
the unit are journaled, undone for step 2b, and then laid over step 2b's, so the instruction wins where both write.

pads() uses the ui of the cycle before, so a bypass path lags by one cycle, as section 10 allows (undefined, item 3).
The host is modelled at byte level: xfer() serves a whole command between two cycles, in cycle h, as section 10 allows
a byte-level simulator (undefined, item 6). SCK timing is not modelled.
"""
import atexit, json, os, random, re
from isatable import ROWS, DECODE, DOC, UNIT

K = 4
LANES = 0xF1                    # uo0, uo4..uo7
SRC = {0: 3, 4: 4, 5: 5, 6: 6, 7: 7}
RETRY = 'retry'
ISA_VERSION = 0x01              # 0.1
COVER = {}                      # table row -> times executed, across every chip in this process


def _dump_cover():
    d = os.environ.get('COVER_DIR')
    if d:
        with open(f'{d}/cover_{os.getpid()}.json', 'w') as f:
            json.dump(COVER, f)


atexit.register(_dump_cover)


# ---- shift unit: compiled from the code blocks of section 5.9 -----------------------------------------------------
def _unit_source(text):
    if '### 5.9 Shift units' not in text:
        return ''
    sec = text.split('### 5.9 Shift units', 1)[1].split('\n## 6. ', 1)[0]
    return ''.join(b for b in re.findall(r'```\n(.*?)```', sec, re.S) if re.match(r'\w+\(.*\):\n', b))


def _compile_unit(src, journal):
    """The rules: `name(args):` is a function of S and its args, `X := e` an assignment, every name in
    capitals S.NAME, and a call of a block function passes S first. With journal, an assignment to the unit's own
    state first records the old value (Unit._j), so a step can be undone."""
    funcs = re.findall(r'^(\w+)\(.*\):$', src, re.M)
    py = []
    for raw in src.splitlines():
        line = raw.split('#')[0].rstrip()
        if not line.strip():
            continue
        m = re.match(r'^(\w+)\((.*)\):$', line)
        if m:
            py.append(f"def {m[1]}({', '.join(x for x in ['S', m[2]] if x)}):")
            continue
        ind = re.match(r'^(\s*)', line)[1]
        body = line.strip()
        m = re.match(r'^(.+?) := (.+)$', body)
        if m:
            body = f'{m[1]} = {m[2]}'
            base = re.match(r'([A-Z][A-Z0-9_]*)\b', m[1])
            if journal and base and base[1] in UNIT:
                py.append(f"{ind}S._j('{base[1]}')")
        body = re.sub(r'\b([A-Z][A-Z0-9_]*)\b', r'S.\1', body)
        for fn in funcs:
            body = re.sub(rf'\b{fn}\(\)', f'{fn}(S)', body)
            body = re.sub(rf'\b{fn}\((?!S\))', f'{fn}(S, ', body)
        py.append(ind + body)
    ns = {'bit': lambda x, k: (x >> k) & 1}
    exec(compile('\n'.join(py) + '\n', 'unit_journal' if journal else 'unit', 'exec'), ns)
    return ns


def _fields(text):
    """The field table of 5.9.3, between its markers: name -> (slot, low bit, width)."""
    if '<!-- shift-fields-begin -->' not in text:
        return {}
    tab = text.split('<!-- shift-fields-begin -->')[1].split('<!-- shift-fields-end -->')[0]
    out = {}
    for name, slot, hi, lo, meaning in re.findall(r'^\| ([A-Z][A-Z0-9]*) \| (\d+)[^|]*\| (\d+)(?::(\d+))? \| ([^|]*) \|$',
                                                  tab, re.M):
        out[name] = (int(slot), int(lo or hi), int(hi) - int(lo or hi) + 1, meaning.lower().startswith('reserved'))
    return out


_TEXT = open(DOC).read()
_SRC = _unit_source(_TEXT)
HAS_UNIT = bool(_SRC)
UNITF = _compile_unit(_SRC, False) if HAS_UNIT else {}      # steps 2b and the host
UNITJ = _compile_unit(_SRC, True) if HAS_UNIT else {}       # step 3: the table's helpers, journaled
FIELD = _fields(_TEXT)
_KEEP = [0] * 16                        # 5.9.4: the bits the field table defines in each slot, reserved rows left out
for _n, (_s, _lo, _w, _rsv) in FIELD.items():
    if not _rsv:
        _KEEP[_s] |= ((1 << _w) - 1) << _lo


class Unit:
    """One thread's shift unit. Its own state is plain attributes (5.9.2, all 0 after reset, section 8); the
    configuration fields of 5.9.3 read CFG; SYNC2, SYNC3, OWN, SHSEL, TID, R and T are the chip's and the thread's."""
    KEEP = _KEEP

    def __init__(self, chip, thread):
        self._c, self._t, self._old = chip, thread, None
        for n in UNIT:
            setattr(self, n, 0)
        self.CFG = [0] * 16

    def _j(self, name):
        o = self._old
        if o is not None and name not in o:
            v = getattr(self, name)
            o[name] = list(v) if name == 'CFG' else v

    SYNC2 = property(lambda s: s._c.sync2)
    SYNC3 = property(lambda s: s._c.sync3)
    OWN = property(lambda s: s._c.owner)
    SHSEL = property(lambda s: s._c.shsel)
    TID = property(lambda s: s._t.id)
    R = property(lambda s: s._t.r)
    T = property(lambda s: s._t.T, lambda s, v: setattr(s._t, 'T', v))


for _n, (_s, _lo, _w, _rsv) in FIELD.items():
    setattr(Unit, _n, property(lambda u, s=_s, lo=_lo, m=(1 << _w) - 1: (u.CFG[s] >> lo) & m))


class Thread:
    def __init__(self, tid):
        self.id, self.pc, self.r, self.T, self.D, self.S = tid, 0, [0] * 8, 0, 0, 0
        self.late = self.stale = self.ill = 0
        self.run = self.stopreq = self.stepping = 0
        self.iv = self.ir = 0                   # fetched word, valid
        self.P = self.due = self.waiting = 0
        self.kind = None
        self.mask = self.pat = self.bank = self.ne = self.timed = self.inbox = 0
        self.mem = None                         # second-slot state of ld/st
        self.dw = False                         # D written this cycle
        self.latecount = 0                      # statistic only, not architectural


class Chip:
    def __init__(self, words=None, size=512, seed=1):
        self.size = size
        rng = random.Random(seed)
        self.mem = [rng.getrandbits(16) for _ in range(size)]   # contents are undefined at reset
        self.written = set()
        for a, w in (words or {}).items():
            self.mem[a] = w
            self.written.add(a)
        self.t = [Thread(i) for i in range(K)]
        self.c = self.sync1 = self.sync2 = 0
        self.out = self.sh_out = 0              # 16 bits: uo[7:0], uio[15:8]
        self.oe = self.sh_oe = self.od = 0      # 8 bits, uio
        self.byp = self.sh_byp = 0              # 8 bits, uo, lanes only
        self.owner = [0] * 16
        self.own = [0xFFFF, 0, 0, 0]            # owner table as one mask per thread
        self.mail, self.full = [0] * 8, [0] * 8
        self.busy = 0
        self.ui_last = 0
        self.retired = [0] * K
        self.sync3 = 0                          # 5.9: SYNC2 one cycle older
        self.sel, self.shsel = [0] * 16, [0] * 16   # output switch per pin and its shadow: 0 OUT, 1 shift output A, 2 B
        for t in self.t:
            t.u = Unit(self, t)

    @property
    def illegal(self):
        return int(any(t.ill for t in self.t))

    # ---- table helpers -----------------------------------------------------------------------
    def set_d(self, t, v):
        t.D, t.dw = v & 0xFFFF, True

    def shadow(self, t, p, v):
        if p != 3 and self.owner[p] == t.id:
            self.sh_out = (self.sh_out & ~(1 << p)) | (v & 1) << p

    def shadow_oe(self, t, p, v):
        b = p - 8
        if self.owner[p] == t.id and not (self.od >> b) & 1:
            self.sh_oe = (self.sh_oe & ~(1 << b)) | (v & 1) << b

    def put8(self, t, bank, v):
        for i in range(8):
            self.shadow(t, 8 * bank + i, (v >> i) & 1)

    def oe8(self, t, v):
        for i in range(8):
            self.shadow_oe(t, 8 + i, (v >> i) & 1)

    def claim(self, t, p):
        self.own[self.owner[p]] &= ~(1 << p)
        self.owner[p] = t.id
        self.own[t.id] |= 1 << p
        self.sel[p] = self.shsel[p] = 0         # 5.3: any pcfg takes the pin off its shift output at once

    # the shift unit's helpers of 5.9.4, which only step 3 calls: the journaled compile of the text
    def tx_push(self, t, v, n, l, k):
        UNITJ['tx_push'](t.u, v, n, l, k)

    def rx_pop(self, t):
        UNITJ['rx_pop'](t.u)

    def sstat(self, t, d):
        UNITJ['sstat'](t.u, d)

    def srd(self, t, d, s):
        UNITJ['srd'](t.u, d, s)

    def set_cfg(self, t, k, v):
        UNITJ['set_cfg'](t.u, k, v)

    def sgo(self, t, m):
        UNITJ['sgo'](t.u, m)

    def sstop(self, t, m):
        UNITJ['sstop'](t.u, m)

    def shadow_sel(self, t, p, l):
        if self.owner[p] == t.id:               # 5.9.4: smap prepares the switch, like put
            self.shsel[p] = l

    def pcfg_in(self, t, p):
        self.claim(t, p)
        self.od &= ~(1 << (p - 8))
        self.shadow_oe(t, p, 0)

    def pcfg_pp(self, t, p):
        self.claim(t, p)
        if p >= 8:
            self.od &= ~(1 << (p - 8))
            self.shadow_oe(t, p, 1)
        else:
            self.sh_byp &= ~(1 << p)

    def pcfg_od(self, t, p):
        self.claim(t, p)
        self.od |= 1 << (p - 8)
        self.oe &= ~(1 << (p - 8))
        self.sh_oe &= ~(1 << (p - 8))
        self.out |= 1 << p
        self.sh_out |= 1 << p

    def pcfg_byp(self, t, p):
        self.claim(t, p)
        self.sh_byp |= 1 << p

    def halt(self, t):
        t.pc = (t.pc + 1) % self.size
        self.stop(t)

    def stop(self, t):
        if not t.P:
            t.waiting = 0
        t.run = t.P = t.stopreq = t.stepping = 0
        t.mem = None

    def now(self, t, c):
        self.set_d(t, c + 1)
        t.P, t.kind, t.due = 1, 'tick', 0

    def schedule(self, t, c, step):
        """The deadline half of an event instruction. A stale D is replaced, not stepped."""
        if t.stale:
            self.set_d(t, c + 1)
            t.late = 1
            t.latecount += 1
        else:
            self.set_d(t, t.D + step)
        delta = (t.D - c) & 0xFFFF
        t.due = int(delta == 0 or delta >= 0x8000)

    def arm(self, t, c, step):
        self.schedule(t, c, step)
        t.P, t.kind = 1, 'tick'

    def wait(self, t, c, mask, pat, bank, ne, rt, inbox):
        t.mask, t.pat, t.bank, t.ne, t.inbox = mask & 255, pat & 255, bank, int(ne), inbox
        t.timed = int(rt is not None)
        t.due = 0
        if rt is not None:
            self.schedule(t, c, rt)
        t.P, t.kind, t.waiting = 1, 'wait', 1
        t.pc = (t.pc + 1) % self.size

    def load(self, t, d, addr):
        t.mem = ['l', d, addr % self.size, 0, False]

    def store(self, t, v, addr):
        t.mem = ['s', v, addr % self.size, 0, False]

    # ---- pins --------------------------------------------------------------------------------
    def pads(self):
        byp = self.byp
        ui = self.ui_last
        follow = (byp & 1 and (ui >> 3) & 1) | (byp & ui & 0xF0)
        v = self.out                            # V(p) of 5.2: OUT, or a shift output of the pin's owner
        if any(self.sel):
            for p, s in enumerate(self.sel):
                if s:
                    u = self.t[self.owner[p]].u
                    v = (v & ~(1 << p)) | (u.LA if s == 1 else u.LB) << p
        uo = (v & 0xF7 & ~byp) | follow
        io = v >> 8
        en = (~io & self.od | self.oe & ~self.od) & 0xFF
        val = io & ~self.od & 0xFF
        return uo, en, val

    def commit(self, t):
        m = self.own[t.id]
        self.out = (self.out & ~m) | (self.sh_out & m)
        hi = m >> 8
        self.oe = (self.oe & ~hi) | (self.sh_oe & hi)
        lo = m & LANES
        self.byp = (self.byp & ~lo) | (self.sh_byp & lo)
        for p in range(16):                     # 5.4: the output switch commits with the pins
            if m >> p & 1:
                self.sel[p] = self.shsel[p]
        t.u.CMT = 1

    # ---- events ------------------------------------------------------------------------------
    def events(self, tb):
        sync = self.sync2
        for t in self.t:
            if not t.P:
                continue
            if t.kind == 'tick':
                if t.due or t.D == tb:
                    if t.due:
                        t.late = 1
                        t.latecount += 1
                    self.commit(t)
                    t.S, t.P = sync, 0
            else:
                byte = (sync >> (8 * t.bank)) & 255
                m = ((byte ^ t.pat) & t.mask) == 0
                if t.ne:
                    m = not m
                if m:
                    self.commit(t)
                    t.S, t.T, t.P = sync, 1, 0
                    self.set_d(t, tb)
                elif t.inbox and self.full[t.id]:
                    t.S, t.T, t.P = sync, 0, 0
                    self.set_d(t, tb)
                elif t.timed and (t.due or t.D == tb):
                    t.S, t.T, t.P = sync, 0, 0

    # ---- execute -----------------------------------------------------------------------------
    def retire(self, t):
        self.retired[t.id] += 1
        if t.stepping:
            if t.stopreq:
                t.P = 0
            t.run = t.stepping = t.stopreq = 0

    def slot(self, c, t, was_pending):
        if not t.run:
            return
        if t.mem and t.mem[4]:                  # second slot of ld/st
            if t.mem[0] == 'l':
                t.r[t.mem[1]] = t.mem[3]
            t.mem = None
            t.pc = (t.pc + 1) % self.size
            self.retire(t)
            return
        if t.stopreq:
            self.stop(t)
            return
        if not t.iv:
            return
        if t.waiting:
            if was_pending:
                return
            t.waiting = 0
            self.retire(t)
            if not t.run:
                return
            was_pending = 0                     # the next instruction runs in this slot
        d = DECODE(t.ir)
        if d is None:
            t.ill = 1
            self.stop(t)
            return
        row, fields = d
        if was_pending:
            w = row.world
            if w == 'y' or (w == 'pin' and (fields[row.letters.index('i')] != 0 or
                                            fields[row.letters.index('o')] != 3)):
                return
        npc = row.fn(self, t, c, *fields)
        if npc == RETRY:
            return
        COVER[row.syntax] = COVER.get(row.syntax, 0) + 1
        if npc is not None:
            t.pc = npc
            self.retire(t)

    def mem_cycle(self, t):
        if t.mem:
            if not t.mem[4]:
                if t.mem[0] == 'l':
                    t.mem[3] = self.mem[t.mem[2]]
                else:
                    self.mem[t.mem[2]] = t.mem[1]
                    self.written.add(t.mem[2])
                t.mem[4] = True
            return
        if t.run:
            t.ir, t.iv = self.mem[t.pc], 1
            return
        t.iv = 0

    # ---- clock -------------------------------------------------------------------------------
    def cycle(self, env):
        c = self.c
        uo, en, val = self.pads()
        ui, lvl = env.step(self, c, uo, en, val)
        self.ui_last = ui
        tb = c & 0xFFFF
        owner = self.t[c % K]
        was_pending = owner.P
        stale_hit = [t.D == tb ^ 0x8000 for t in self.t]
        self.events(tb)
        self.slot_and_units(c, owner, was_pending)
        self.mem_cycle(self.t[(c - 1) % K])
        for t, hit in zip(self.t, stale_hit):
            if t.dw:
                t.stale, t.dw = 0, False
            elif hit:
                t.stale = 1
            t.u.CMT = 0
        self.sync3 = self.sync2
        self.sync2, self.sync1, self.c = self.sync1, (ui & 255) | (lvl & 255) << 8, c + 1

    def slot_and_units(self, c, owner, was_pending):
        """Steps 3 and 2b of cycle c. Step 3 runs first, so it reads the unit as it was before step 2b (3.2), and its
        writes to the unit are journaled: undone while step 2b runs, which reads the start of the cycle too, and then
        laid over step 2b's, because where both write a flop the instruction wins."""
        u = owner.u
        u._old = {}
        self.slot(c, owner, was_pending)
        old, u._old = u._old, None
        if not HAS_UNIT:
            return
        new = {k: getattr(u, k) for k in old}
        for k, v in old.items():
            setattr(u, k, v)
        for t in self.t:                        # step 2b
            UNITF['tx_step'](t.u)
            UNITF['rx_step'](t.u)
        for k, v in new.items():
            setattr(u, k, v)

    def run(self, cycles, env):
        for _ in range(cycles):
            self.cycle(env)

    # ---- host, behavioural, byte level -------------------------------------------------------
    def start(self, tid, pc=0):
        """SETPC followed by RUN for one thread."""
        self.host_setpc(self.t[tid], pc)
        self.host_run(0, 1 << tid)

    def host_run(self, stop, start):
        h = (self.c - 1) & 0xFFFF
        for t in self.t:
            b = 1 << t.id
            if stop & b:
                if t.run:
                    t.stopreq = 1
                else:
                    t.P = 0
            elif start & b and not t.run:
                t.run, t.late, t.ill = 1, 0, 0
                t.P = t.waiting = t.stopreq = 0
                self.set_d(t, h)
                t.stale, t.dw = 0, False

    def host_setpc(self, t, pc):
        if t.run:
            self.busy = 1
        else:
            t.pc = pc % self.size

    def host_step(self, t):
        if t.run:
            self.busy = 1
        else:
            t.run, t.stepping, t.waiting = 1, 1, 0

    def post(self, n, v):
        if self.full[n]:
            return False
        self.mail[n], self.full[n] = v & 0xFFFF, 1
        return True

    def take(self, n):
        if not self.full[n]:
            return None
        self.full[n] = 0
        return self.mail[n]

    def stopped(self):
        return [t for t in self.t if not t.run]

    def abort(self):
        for t in self.t:
            t.P = t.waiting = 0
            if t.run:
                t.stopreq = 1
            if HAS_UNIT:                        # 9.3: every unit stops
                UNITF['tx_stop'](t.u)
                UNITF['rx_stop'](t.u)
        self.oe = self.sh_oe = 0
        self.out |= self.od << 8
        self.sh_out |= self.od << 8
        for i in range(8):                      # and every uio pin leaves its shift output; uo pins keep theirs
            self.sel[8 + i] = self.shsel[8 + i] = 0

    def rdreg(self, t, r):
        """RDREG reg8 on a stopped thread (9.2). 12 is the status word of 5.9.4 with nothing cleared."""
        if r < 8:
            return t.r[r]
        if r < 12:
            return {8: t.D, 9: t.S, 10: t.pc, 11: t.T | t.late << 1 | t.stale << 2 | t.waiting << 3}[r]
        if not HAS_UNIT or r > 31:
            return 0
        u = t.u
        if r == 12:
            busy_tx = 1 if u.TXARM or u.TXRUN or u.TXTL else 0
            busy_rx = 1 if u.RXARM or u.RXHUNT or u.RXEP else 0
            return (u.RXPN | busy_tx << 5 | u.TXFULL << 6 | u.TXUR << 7 | u.RXRUN << 8 | u.RXFULL << 9 | u.RXOVR << 10 |
                    u.RXSYN << 11 | u.RXSEEN << 12 | u.LA << 13 | u.LB << 14 | busy_rx << 15)
        if r == 13:
            return u.CRC & 0xFFFF
        if r == 14:
            return u.CRC >> 16
        if r == 15:
            return u.RXPE
        return u.CFG[r - 16]

    def xfer(self, mosi):
        """One CS-low transaction at byte level. Returns the MISO bytes, including the zeros clocked
        out during the command and argument bytes. SCK timing is not modelled here. A command cut before
        its last argument byte does nothing (9.1)."""
        cmd, n = mosi[0], len(mosi)
        miso = [0] * n
        arg = lambda i: mosi[i]
        word = lambda i: arg(i) << 8 | arg(i + 1)

        def reply(start, data):
            for i, x in enumerate(data):
                if start + i < n:
                    miso[start + i] = x
        if n < (3 if 0x20 <= cmd < 0x28 else {0x01: 3, 0x02: 3, 0x10: 3, 0x11: 4, 0x13: 2, 0x14: 3}.get(cmd, 1)):
            return miso
        if cmd == 0x00:
            depth = {128: 7, 512: 9}.get(self.size, 0)
            reply(1, [ISA_VERSION, K << 4 | depth])
        elif cmd == 0x01:                       # WRMEM
            a = word(1)
            for i in range(3, n - 1, 2):
                if not self.stopped():
                    self.busy = 1
                    break
                self.mem[a % self.size] = word(i)
                self.written.add(a % self.size)
                a += 1
        elif cmd == 0x02:                       # RDMEM. The BUSY test runs when addr16 arrives (9.4)
            if not self.stopped():
                self.busy = 1
            else:
                a = word(1)
                for i in range(5, n, 2):
                    v = self.mem[a % self.size]
                    reply(i, [v >> 8, v & 255])
                    a += 1
        elif cmd == 0x10:
            self.host_run(arg(1), arg(2))
        elif cmd == 0x11:
            self.host_setpc(self.t[arg(1) & 3], word(2))
        elif cmd == 0x12:
            run = sum(t.run << t.id for t in self.t)
            ill = sum(t.ill << t.id for t in self.t)
            late = sum(t.late << t.id for t in self.t)
            stale = sum(t.stale << t.id for t in self.t)
            full = sum(f << i for i, f in enumerate(self.full))
            reply(1, [run << 4 | ill, late << 4 | stale, full, self.busy])
            if n >= 5:
                self.busy = 0
        elif cmd == 0x13:
            self.host_step(self.t[arg(1) & 3])
        elif cmd == 0x14:
            t = self.t[arg(1) & 3]
            if t.run:
                self.busy = 1
            else:
                v = self.rdreg(t, arg(2))
                reply(4, [v >> 8, v & 255])
        elif 0x20 <= cmd < 0x28:
            refused = not self.post(cmd & 7, word(1))
            reply(3, [int(refused)])
        elif 0x28 <= cmd < 0x30:
            v = self.take(cmd & 7)
            reply(1, [0, 0, 0] if v is None else [1, v >> 8, v & 255])
        elif cmd == 0x3F:
            self.abort()
        return miso
