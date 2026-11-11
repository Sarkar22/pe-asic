#!/usr/bin/env python3
"""An ISA v0.1 simulator compiled from the document's text. Every function in a code block of the
form `name(args):` becomes Python here, with no hand-written behaviour for those functions, and each
row of the instruction table runs its Semantics cell as isatable.py compiles it. What is written by
hand is the step order of section 3.2, the pad equations of section 5.2 and the host side. The byte
framing, the mailbox, RDREG 0 to 11 and the reset values come from sim2.py, which this builds on.
compare.py runs it beside sim.py and sim2.py and compares the three on every cycle.

Shift units (section 5.9). The code blocks of 5.9 compile like the others: a name in capitals is
state, of the thread's unit if section 5.9.2 lists it per thread (kept as thread.u.NAME), and a field
name of the table in 5.9.3 stands for its value in the configuration slot, (CFG[slot] >> low bit) &
mask, read when the line runs. KEEP is that table's bits per slot. The hand-written parts for the unit
are the ones the text puts in the step order and the pads: step 2b (every thread runs tx_step(), then
rx_step()), the rule that step 3 reads the unit's flops as they were before step 2b, with the
instruction winning where both write a flop (3.2: step 3 runs on a copy of the start-of-cycle unit and
its writes are applied after step 2b's), step 5's CMT := 0 and SYNC3 := SYNC2, V(p) in the pad
equations, and RDREG 12 to 31, which the host framing serves by running the compiled sstat and srd on
a scratch copy of the thread.

As in sim2.py, a bypass path lags by one cycle (section 10, undefined, item 3), and a host command is
served whole between two cycles (section 10, undefined, item 6)."""
import copy
import re
import sim2
from sim2 import K, RETRY, ISA_VERSION
from isatable import DOC, DECODE, UNIT, ROWS

THREAD = dict(STALE='stale', LATE='late', D='D', S='S', T='T', P='P', KIND='kind', DUE='due',
              WAITING='waiting', MASK='mask', PAT='pat', BANK='bank', NE='ne', TIMED='timed',
              INBOX='inbox', MEM='memb', MEM_DONE='mem_done', MEM_KIND='mem_kind', MEM_ADDR='mem_addr',
              MEM_DATA='mem_data', MEM_REG='mem_reg', RUN='run', ILL='ill', STOPREQ='stopreq', STEPPING='stepping',
              IR='ir', IV='iv', PC='pc', TID='id', D_WRITTEN='dw', HIT='hit', R='r')
GLOBAL = dict(OWN='owner', OUT='OUT', SHD='SHD', OE='OE', SHOE='SHOE', OD='OD', BYP='BYP', SHBYP='SHBYP',
              FULL='full', SYNC2='sync2', DEPTH='size', LANES='lanes', mem='mem', BUSY='busy',
              SYNC3='sync3', SEL='sel', SHSEL='shsel', KEEP='KEEP')


def field_table(text):
    """5.9.3, between its markers: field name -> (slot, low bit, width), and KEEP per slot."""
    if '<!-- shift-fields-begin -->' not in text:
        return {}, [0] * 16
    tab = text.split('<!-- shift-fields-begin -->')[1].split('<!-- shift-fields-end -->')[0]
    fields, keep = {}, [0] * 16
    for line in tab.splitlines():
        m = re.match(r'^\| ([A-Z][A-Z0-9]*) \| (\d+)[^|]*\| (\d+)(?::(\d+))? \| (.*) \|$', line)
        if not m:
            continue
        hi, lo = int(m[3]), int(m[4] if m[4] else m[3])
        fields[m[1]] = (int(m[2]), lo, hi - lo + 1)
        if not m[5].lower().startswith('reserved'):         # 5.9.4: KEEP leaves out any row marked reserved
            keep[int(m[2])] |= ((1 << (hi - lo + 1)) - 1) << lo
    return fields, keep


FIELDS, KEEP = field_table(open(DOC).read())
UNIT_MAP = {n: f't.u.{n}' for n in UNIT}
UNIT_MAP.update({n: f'((t.u.CFG[{s}] >> {lo}) & {(1 << w) - 1})' for n, (s, lo, w) in FIELDS.items()})
assert not set(FIELDS) & set(UNIT) and not set(UNIT_MAP) & (set(THREAD) | set(GLOBAL))
UNIT_RE = re.compile(r'\b(%s)\b' % '|'.join(sorted(UNIT_MAP, key=len, reverse=True))) if UNIT_MAP else None


def blocks(text):
    out = []
    for b in re.findall(r'```\n(.*?)```', text, re.S):
        if re.match(r'\w+\(.*\):\n', b):
            out.append(b)
    return out


def names(line, funcs):
    line = re.sub(r'\bKIND (:=|==) (\w+)', r"KIND \1 '\2'", line)
    line = re.sub(r'\bMEM_KIND (:=|==) (\w+)', r"MEM_KIND \1 '\2'", line)
    line = re.sub(r'\bnext\b', '((PC + 1) % DEPTH)', line)
    line = re.sub(r'\bTB\b', '(a & 0xFFFF)', line)
    for k, v in THREAD.items():
        line = re.sub(rf'\b{k}\b', f't.{v}', line)
    for k, v in GLOBAL.items():
        line = re.sub(rf'\b{k}\b', f'c.{v}', line)
    if UNIT_RE:                                         # one pass, so a replacement is never matched again
        line = UNIT_RE.sub(lambda m: UNIT_MAP[m[1]], line)
    for f in funcs:
        line = re.sub(rf'\b{f}\(\)', f'{f}(c, t, a)', line)
        line = re.sub(rf'\b{f}\((?!c, t, a)', f'{f}(c, t, a, ', line)
    return line


def compile_doc():
    text = open(DOC).read()
    src = ''.join(blocks(text))
    funcs = re.findall(r'^(\w+)\(.*\):$', src, re.M)
    py = []
    for line in src.splitlines():
        line = line.split('#')[0].rstrip()
        m = re.match(r'(\w+)\((.*)\):$', line)
        if m:
            args = ', '.join(x for x in ['c', 't', 'a', m[2]] if x)
            py.append(f'def {m[1]}({args}):')
            continue
        m = re.match(r'(\s+)(.+?) := (.+)$', line)
        if m:
            lhs, rhs = names(m[2], funcs), names(m[3], funcs)
            if m[2] in ('KIND', 'MEM_KIND'):
                rhs = repr(m[3])
            if lhs == 't.D':
                py.append(f'{m[1]}t.D = ({rhs}) & 0xFFFF; t.dw = 1')
            elif lhs == 't.pc':
                py.append(f'{m[1]}t.pc = ({rhs}) % c.size')
            else:
                py.append(f'{m[1]}{lhs} = {rhs}')
            continue
        py.append(names(line, funcs))
    env = {'bit': lambda x, n: (x >> n) & 1}
    env.update(illegal=lambda c, t, a, w: DECODE(w) is None)
    code = '\n'.join(py) + '\n'
    # natives the blocks call: the table, section 6
    code = re.sub(r'\billegal\(', 'illegal(c, t, a, ', code)
    code = re.sub(r'\bworld\(', 'world(c, t, a, ', code)
    code = re.sub(r'\brun_row\(', 'run_row(c, t, a, ', code)
    params = set(re.findall(r'\b(\w+)\b', ' '.join(re.findall(r'^def \w+\((.*)\):$', code, re.M))))
    left = sorted(set(re.findall(r'(?<![\w.\'])([A-Z][A-Z0-9_]*)\b', code)) - params)
    if left:
        raise ValueError('names in the code blocks with no meaning here: %s' % left)

    def world(c, t, a, w):
        row, fields = DECODE(w)
        x = row.world
        return x == 'y' or (x == 'pin' and (fields[row.letters.index('i')] != 0 or fields[row.letters.index('o')] != 3))

    def run_row(c, t, a, w):
        row, fields = DECODE(w)
        sim2.COVER[row.syntax] = sim2.COVER.get(row.syntax, 0) + 1
        npc = row.fn(c, t, a, *fields)
        if npc == RETRY:
            return
        if npc is not None:
            t.pc = npc
            env['retire'](c, t, a)
    env.update(world=world, run_row=run_row)
    exec(compile(code, 'isa_doc_blocks', 'exec'), env)
    return env, funcs, code


ENV, FUNCS, CODE = compile_doc()
HAS_UNIT = 'tx_step' in ENV
# Rows whose semantics read or write the thread's unit (5.9.2 names, or a 5.9.4 helper): step 3 runs them on
# the start-of-cycle copy. A slot runs only the fetched word IR, so the copy is needed only when IR is one.
_UNIT_SEM = re.compile(r'\b(%s|tx_push|rx_pop|sstat|srd|set_cfg|sgo|sstop)\b' % '|'.join(UNIT)) if UNIT else None
UNIT_ROWS = {r.syntax for r in ROWS if _UNIT_SEM and _UNIT_SEM.search(r.sem)}


class Unit:
    """One thread's shift unit: every per-thread name of section 5.9.2, 0 after reset (section 8)."""
    def __init__(self):
        for n in UNIT:
            setattr(self, n, 0)
        self.CFG = [0] * 16


class Step3Unit(Unit):
    """The start-of-cycle copy step 3 runs on. Remembers which names it wrote."""
    def __init__(self, live):
        self.__dict__.update(live.__dict__)
        self.__dict__['CFG'] = list(live.CFG)
        self.__dict__['_w'] = set()

    def __setattr__(self, k, v):
        self._w.add(k)
        self.__dict__[k] = v


class Thread(sim2.Thread):
    def __init__(self, tid):
        super().__init__(tid)
        self.memb = self.mem_done = self.mem_kind = self.mem_addr = self.mem_data = self.mem_reg = 0
        self.dw = self.hit = 0
        self.u = Unit()

    @property
    def latecount(self):
        return self.late_events

    @latecount.setter
    def latecount(self, v):
        self.late_events = v


def int_of(bits):
    return sum(b << i for i, b in enumerate(bits))


class Chip(sim2.Chip):
    lanes = (0, 4, 5, 6, 7)
    KEEP = KEEP

    def __init__(self, words=None, size=512, seed=1):
        super().__init__(words, size, seed)
        self.t = [Thread(i) for i in range(K)]
        self.OUT, self.SHD = [0] * 16, [0] * 16
        self.OE, self.SHOE, self.OD = [0] * 8, [0] * 8, [0] * 8
        self.BYP, self.SHBYP = [0] * 8, [0] * 8

    # int views for the benches and for the table's bit(SHD, p)
    out = property(lambda s: int_of(s.OUT), lambda s, v: None)
    sh_out = property(lambda s: int_of(s.SHD), lambda s, v: None)
    oe = property(lambda s: int_of(s.OE), lambda s, v: None)
    sh_oe = property(lambda s: int_of(s.SHOE), lambda s, v: None)
    od = property(lambda s: int_of(s.OD), lambda s, v: None)
    byp = property(lambda s: int_of(s.BYP), lambda s, v: None)
    sh_byp = property(lambda s: int_of(s.SHBYP), lambda s, v: None)

    # the table's calls land on the compiled blocks
    def shadow(self, t, p, v): ENV['shadow'](self, t, 0, p, v)
    def shadow_oe(self, t, p, v): ENV['shadow_oe'](self, t, 0, p, v)
    def put8(self, t, b, v): ENV['put8'](self, t, 0, b, v)
    def oe8(self, t, v): ENV['oe8'](self, t, 0, v)
    def pcfg_in(self, t, p): ENV['pcfg_in'](self, t, 0, p)
    def pcfg_pp(self, t, p): ENV['pcfg_pp'](self, t, 0, p)
    def pcfg_od(self, t, p): ENV['pcfg_od'](self, t, 0, p)
    def pcfg_byp(self, t, p): ENV['pcfg_byp'](self, t, 0, p)
    def halt(self, t): ENV['halt'](self, t, 0)
    def now(self, t, c): ENV['now'](self, t, c)
    def arm(self, t, c, step): ENV['arm'](self, t, c, step)
    def wait(self, t, c, *a): ENV['wait'](self, t, c, *a)
    def load(self, t, d, addr): ENV['load'](self, t, 0, d, addr)
    def store(self, t, v, addr): ENV['store'](self, t, 0, v, addr)
    def tx_push(self, t, v, n, l, k): ENV['tx_push'](self, t, 0, v, n, l, k)
    def rx_pop(self, t): ENV['rx_pop'](self, t, 0)
    def sstat(self, t, d): ENV['sstat'](self, t, 0, d)
    def srd(self, t, d, s): ENV['srd'](self, t, 0, d, s)
    def set_cfg(self, t, k, v): ENV['set_cfg'](self, t, 0, k, v)
    def sgo(self, t, m): ENV['sgo'](self, t, 0, m)
    def sstop(self, t, m): ENV['sstop'](self, t, 0, m)
    def shadow_sel(self, t, p, l): ENV['shadow_sel'](self, t, 0, p, l)

    def set_d(self, t, v):
        t.D, t.dw = v & 0xFFFF, 1

    def V(self, p):
        """5.2: what output pin p shows before bypass and open drain."""
        s = self.sel[p]
        if s == 1:
            return self.t[self.owner[p]].u.LA
        if s == 2:
            return self.t[self.owner[p]].u.LB
        return self.OUT[p]

    def pads(self):
        ui = self.ui_last
        uo = 0
        for i in range(8):
            if i == 3:
                continue
            src = {0: 3}.get(i, i)
            v = (ui >> src) & 1 if self.BYP[i] else self.V(i)
            uo |= v << i
        en = val = 0
        for i in range(8):
            out = self.V(8 + i)
            if self.OD[i]:
                en |= (1 - out) << i
            else:
                en |= self.OE[i] << i
                val |= out << i
        return uo, en, val

    def cycle(self, env):
        a = self.c
        uo, en, val = self.pads()                                   # step 1
        ui, lvl = env.step(self, a, uo, en, val)
        self.ui_last = ui
        owner = self.t[a % K]
        P_start = owner.P
        tb = a & 0xFFFF
        late_before = [t.late for t in self.t]
        for t in self.t:                                            # step 2
            t.hit = t.D == tb ^ 0x8000
        for t in self.t:
            if t.P:
                ENV['events'](self, t, a)
        self.steps_2b_3(owner, a, P_start)
        ENV['memory'](self, self.t[(a - 1) % K], a)                 # step 4
        for t, was in zip(self.t, late_before):                     # step 5
            ENV['update_stale'](self, t, a)
            t.u.CMT = 0
            if t.late and not was:
                t.late_events += 1
        self.sync3 = self.sync2
        self.sync2, self.sync1, self.c = self.sync1, (ui & 255) | (lvl & 255) << 8, a + 1

    def steps_2b_3(self, owner, a, P_start):
        """Step 2b, then step 3 for the thread owning cycle a (section 3.2)."""
        u3 = None
        if HAS_UNIT:
            d = DECODE(owner.ir)
            if d is not None and d[0].syntax in UNIT_ROWS:
                u3 = Step3Unit(owner.u)                             # the unit's flops before step 2b
            for t in self.t:                                        # step 2b
                ENV['tx_step'](self, t, a)
                ENV['rx_step'](self, t, a)
        if u3 is None:
            ENV['slot'](self, owner, a, P_start)                    # step 3
            return
        live = owner.u
        owner.u = u3
        try:
            ENV['slot'](self, owner, a, P_start)                    # step 3, on the start-of-cycle copy
        finally:
            owner.u = live
        for k in u3._w:                                             # where both wrote a flop, the instruction wins
            setattr(live, k, getattr(u3, k))
        live.CFG = u3.CFG                                           # step 2b never writes CFG

    def host_run(self, stop, start):
        h = (self.c - 1) & 0xFFFF
        for t in self.t:
            ENV['host_run'](self, t, 0, (stop >> t.id) & 1, (start >> t.id) & 1, h)

    def host_setpc(self, t, pc):
        ENV['host_setpc'](self, t, 0, pc)

    def host_step(self, t):
        ENV['host_step'](self, t, 0)

    def abort(self):
        for t in self.t:
            ENV['abort_thread'](self, t, 0)
        ENV['abort_pins'](self, self.t[0], 0)

    def rdreg(self, t, r):
        """9.2: 12 is sstat's word with nothing cleared, 13 to 15 what srd reads, 16 + k slot k. The compiled sstat
        and srd run on a scratch copy of the thread, so the thread and its unit don't change."""
        if not HAS_UNIT or r < 12 or r > 31:
            return super().rdreg(t, r)
        if r >= 16:
            return t.u.CFG[r - 16]
        x = copy.copy(t)
        x.r, x.u = list(t.r), copy.copy(t.u)
        if r == 12:
            ENV['sstat'](self, x, 0, 0)
        else:
            ENV['srd'](self, x, 0, 0, r - 13)
        return x.r[0]
