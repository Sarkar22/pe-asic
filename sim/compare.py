#!/usr/bin/env python3
"""Runs the three simulators in this folder side by side and compares them on every cycle.

    sim.py      written by hand from the text
    sim2.py     written by hand from the text too, apart from the row semantics, which it compiles from the
                table, and the shift unit, which it compiles from the code blocks of section 5.9
    docsim.py   compiled from the code blocks and the table; it uses sim2.py's host framing

All three run the same memory image, with the same inputs and the same host commands. After every cycle this
compares the pad values, every thread's registers and flags, every shift unit's state and its 16 slots (5.9.2),
the pin flops and pin owners, the output switch, the mailbox and BUSY, and the whole memory every 64 cycles. It
also compares the reply to every host command.

What runs:
- every program under programs/, three ways: thread 0 from its first word with the inputs changing every cycle,
  the same with the inputs changing every 40 cycles on average, both with mailbox traffic (POST and TAKE), and
  all four threads started at labels of the program, with the uio pads looped back and every host command;
- random images of 128 words. Even seeds draw each word uniformly over the table's rows, then over its legal
  field values, with 7% random words, some of them illegal. Odd seeds are biased to the shift-unit rows of
  section 5.9.4, with slot values that make the units run, and the uio pads looped back, so units receive their
  own and each other's sends.

The host commands are POST, TAKE, RUN with stop and start masks, STEP, SETPC, RDREG of registers 0 to 39,
STATUS, WRMEM, RDMEM and ABORT. sim2.py and docsim.py take them as bytes, through their framing, and sim.py
through its host_* calls. Threads start a few cycles after reset, so the cycle h of section 9.1 is never before
cycle 0.

sim2.py and docsim.py lag a bypass path by one cycle, as section 10 allows (undefined, item 3), so sim.py is
given the previous cycle's ui for its pads here. All three serve a host command in one call between two cycles,
as section 10 allows a byte-level simulator (undefined, item 6).

Usage, from the repository root. Python 3, standard library only:
    python3 sim/compare.py                  every program, and 300 random images of 6000 cycles
    python3 sim/compare.py --quick          every program with shorter runs, and 40 images
    python3 sim/compare.py --images N --seed S      N images from seed S
    python3 sim/compare.py --no-programs    random images only
    python3 sim/compare.py --jobs 4         in 4 processes
    python3 sim/compare.py --stop           stop at the first run that differs
    python3 sim/compare.py --program-cycles N --image-cycles M      run lengths (8000 and 6000)
    python3 sim/compare.py --doc FILE       sim2.py and docsim.py read FILE instead of docs/ISA.md

--doc is for testing the comparison itself: plant a change in a copy of docs/ISA.md and see which simulators
follow it. sim2.py and docsim.py read the table and the code blocks through isatable.py, which takes the path
from ISA_DOC; sim.py and asm.py go on reading docs/ISA.md, so the images and the programs' words don't change.

Prints one line per run that differs, naming the simulator that stands apart and the first thing in which it
differs, as "its value, not the others' value", then a summary. Exits 1 if any run differs.
"""
import glob
import multiprocessing
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.dont_write_bytecode = True


def _arg(name, default=None):
    if name in sys.argv:
        i = sys.argv.index(name)
        if i + 1 < len(sys.argv):
            return sys.argv[i + 1]
        sys.exit('%s needs a value' % name)
    return default


if _arg('--doc'):                       # before isatable is imported: sim2.py and docsim.py read the text through it
    os.environ['ISA_DOC'] = os.path.abspath(_arg('--doc'))

import asm                              # noqa: E402  reads docs/ISA.md
import sim                              # noqa: E402
import isatable                         # noqa: E402  reads ISA_DOC if it is set, otherwise docs/ISA.md
import sim2                             # noqa: E402
import docsim                           # noqa: E402

ROWS = asm.ROWS
NAMES = sim.UNIT_STATE                  # 5.9.2: every per-thread name but CFG, which is compared slot by slot
THREAD = ('PC', 'R', 'T', 'D', 'S', 'LATE', 'STALE', 'ILL', 'RUN', 'P', 'WAITING', 'STOPREQ', 'STEPPING')
PARTS = ('pads', 'threads', 'units', 'SEL', 'SHSEL', 'SYNC3', 'SYNC2', 'FULL', 'M', 'OUT', 'SHD', 'OE', 'SHOE',
         'OD', 'BYP', 'SHBYP', 'OWN', 'BUSY')
UNIT_MN = ('txw', 'rxw', 'sstat', 'srd', 'scfg', 'sgo', 'sstop', 'smap')


def bits(lst):
    v = 0
    for i, x in enumerate(lst):
        v |= x << i
    return v


# ---- the three simulators behind one interface ----------------------------------------------------------------

class Pins:
    """The board. ui and the uio levels; with loop, a uio pad the chip drives reads its own drive (5.2)."""

    def __init__(self, loop):
        self.ui = self.io = 0
        self.loop = loop

    def levels(self, en, val):
        return (val & en) | (self.io & ~en & 0xFF) if self.loop else self.io


class Hand:
    """sim.py."""

    def __init__(self, words, depth, seed):
        self.m = sim.Chip(depth, seed)
        for a, w in words.items():
            self.m.mem[a] = w
        self.m.log = []                 # (cycle, thread, PC, word) of every instruction it executes
        self.prev_ui = 0

    @property
    def mem(self):
        return self.m.mem

    def cycle(self, pins):
        m = self.m
        uo, val, en = m.pads(self.prev_ui)          # the previous ui: the other two lag a bypass path by a cycle
        m.cycle(pins.ui, pins.levels(en, val))
        self.prev_ui = pins.ui
        return uo, en, val

    def state(self):
        m = self.m
        th = tuple((t.PC, tuple(t.R), t.T, t.D, t.S, t.LATE, t.STALE, t.ILL, t.RUN, t.P, t.WAITING, t.STOPREQ,
                    t.STEPPING) for t in m.th)
        units = tuple(tuple([getattr(t.u, n) for n in NAMES]) + tuple(t.u.CFG) for t in m.th)
        return (th, units, tuple(m.SEL), tuple(m.SHSEL), m.SYNC3, m.SYNC2, tuple(m.FULL), tuple(m.M), bits(m.OUT),
                bits(m.SHD), bits(m.OE), bits(m.SHOE), bits(m.OD), bits(m.BYP), bits(m.SHBYP), tuple(m.OWN), m.BUSY)

    def host(self, cmd):
        m, k = self.m, cmd[0]
        if k == 'post':
            return m.host_post(cmd[1], cmd[2])
        if k == 'take':
            return tuple(m.host_take(cmd[1]))
        if k == 'run':
            return m.host_run(cmd[1], cmd[2])
        if k == 'setpc':
            return m.host_setpc(cmd[1], cmd[2])
        if k == 'step':
            return m.host_step(cmd[1])
        if k == 'rdreg':
            return m.host_rdreg(cmd[1], cmd[2])
        if k == 'status':
            return tuple(m.host_status())
        if k == 'wrmem':
            m.host_wrmem(cmd[1], cmd[2])
            return None
        if k == 'rdmem':
            return m.host_rdmem(cmd[1])
        if k == 'abort':
            return m.host_abort()
        raise ValueError(cmd)


class Text:
    """sim2.py or docsim.py: Chip(words, depth, seed), cycle(env) calling env.step for the inputs, xfer(bytes)."""

    def __init__(self, mod, words, depth, seed):
        self.ch = mod.Chip(words, depth, seed)
        self.pins = self.seen = None

    @property
    def mem(self):
        return self.ch.mem

    def step(self, chip, c, uo, en, val):           # called by Chip.cycle, after the pads, for this cycle's inputs
        self.seen = (uo, en, val)
        return self.pins.ui, self.pins.levels(en, val)

    def cycle(self, pins):
        self.pins = pins
        self.ch.cycle(self)
        return self.seen

    def state(self):
        ch = self.ch
        th = tuple((t.pc, tuple(t.r), t.T, t.D, t.S, t.late, t.stale, t.ill, t.run, t.P, t.waiting, t.stopreq,
                    t.stepping) for t in ch.t)
        units = tuple(tuple([getattr(t.u, n) for n in NAMES]) + tuple(t.u.CFG) for t in ch.t)
        return (th, units, tuple(ch.sel), tuple(ch.shsel), ch.sync3, ch.sync2, tuple(ch.full), tuple(ch.mail),
                ch.out, ch.sh_out, ch.oe, ch.sh_oe, ch.od, ch.byp, ch.sh_byp, tuple(ch.owner), ch.busy)

    def host(self, cmd):
        x, k = self.ch.xfer, cmd[0]
        if k == 'post':
            n, v = cmd[1:]
            return x([0x20 | n, v >> 8, v & 255, 0])[3]
        if k == 'take':
            r = x([0x28 | cmd[1], 0, 0, 0])
            return r[1], r[2] << 8 | r[3]
        if k == 'run':
            x([0x10, cmd[1], cmd[2]])
            return None
        if k == 'setpc':
            x([0x11, cmd[1], cmd[2] >> 8, cmd[2] & 255])
            return None
        if k == 'step':
            x([0x13, cmd[1]])
            return None
        if k == 'rdreg':
            r = x([0x14, cmd[1], cmd[2], 0, 0, 0])
            return r[4] << 8 | r[5]
        if k == 'status':
            return tuple(x([0x12, 0, 0, 0, 0])[1:5])
        if k == 'wrmem':
            a, w = cmd[1:]
            x([0x01, a >> 8, a & 255, w >> 8, w & 255])
            return None
        if k == 'rdmem':
            a = cmd[1]
            r = x([0x02, a >> 8, a & 255, 0, 0, 0, 0])
            return r[5] << 8 | r[6]
        if k == 'abort':
            x([0x3F])
            return None
        raise ValueError(cmd)


# ---- what differs ---------------------------------------------------------------------------------------------

def explain(odd, ref):
    """The first part of the state where odd differs from ref: each value as odd shows it, then as ref does."""
    k = next(i for i, (p, q) in enumerate(zip(odd, ref)) if p != q)
    part = PARTS[k]
    if part == 'threads':
        tid = next(i for i in range(4) if odd[1][i] != ref[1][i])
        return 'thread %d: %s' % (tid, ', '.join('%s %s, not %s' % (n, p, q)
                                                for n, p, q in zip(THREAD, odd[1][tid], ref[1][tid]) if p != q))
    if part == 'units':
        tid = next(i for i in range(4) if odd[2][i] != ref[2][i])
        names = list(NAMES) + ['CFG[%d]' % i for i in range(16)]
        diff = ['%s %s, not %s' % (n, p, q) for n, p, q in zip(names, odd[2][tid], ref[2][tid]) if p != q]
        return 'unit of thread %d: %s' % (tid, ', '.join(diff[:6]))
    return '%s %s, not %s' % (part, odd[k], ref[k])


def odd_one(s, t, d):
    """Which simulator stands apart. s, t, d: what sim.py, sim2.py and docsim.py show."""
    if s == t:
        return 'docsim.py differs'
    if s == d:
        return 'sim2.py differs'
    if t == d:
        return 'sim.py differs'
    return 'all three differ'


def verdict(s, t, d):
    """odd_one, and the first part of the state where the odd one differs from the others (for three different
    states, where sim2.py differs from sim.py)."""
    v = odd_one(s, t, d)
    return v, explain(*{'docsim.py differs': (d, s), 'sim2.py differs': (t, s), 'sim.py differs': (s, t)}.get(v, (t, s)))


def three(s, t, d):
    return 'sim.py %s, sim2.py %s, docsim.py %s' % (s, t, d)


def memory_diff(s, t, d):
    a = next(i for i, (x, y, z) in enumerate(zip(s, t, d)) if not x == y == z)
    return odd_one(s, t, d), 'memory word %d: %s' % (a, three(*('%#06x' % m[a] for m in (s, t, d))))


# ---- stimulus. Words come from asm.py's table (docs/ISA.md), so --doc doesn't change them ---------------------

def random_word(rng):
    """Uniform over rows, then over that row's legal field values."""
    while True:
        r = rng.choice(ROWS)
        f = {k: rng.getrandbits(len(b)) for k, b in r.bits.items()}
        if all(asm.legal_value(f[k], r.ftypes[k]) for k in f):
            return asm.encode(r, f)


def enc(mn, **f):
    r = next(x for x in ROWS if x.mn == mn and set(x.bits) == set(f))
    w = asm.encode(r, f)
    assert asm.decode(w)[0] is r, (mn, f)
    return w


_POOL = []


def legal_words():
    if not _POOL:
        _POOL.extend(w for w in range(65536) if asm.decode(w))
    return _POOL


def slot_value(rng, k):
    """5.9.3: values that make a unit do something. Short periods, decoders on the looped-back uio pads."""
    return {
        0: rng.choice((0x0000, 0x0081, 0x0018, 0x0060, 0x0001, rng.getrandbits(15))),
        1: rng.choice((0x0401, 0x0E00, 0x0101, 0x0000, rng.getrandbits(16))),
        2: rng.choice((0, 1, 2, 3, 5, rng.randrange(1, 20))),
        3: rng.choice((0, 3, 0x800F, 0x8002, rng.getrandbits(16))),
        4: rng.choice((0x0821, 0x0A61, 0x0961, 0x0B22, rng.getrandbits(16) & 0xF0FF | rng.randrange(8, 12) << 8)),
        5: rng.choice((1, 2, 5, 8, rng.randrange(1, 40))),
        6: rng.choice((0, 1, 2, 4, 8)),
        7: rng.choice((0, 0, 30, 200)),
        8: rng.choice((0, 0x8000, 0x4010, rng.getrandbits(16))),
        9: rng.choice((0xD555, 0x8000, 0x7E00, rng.getrandbits(16))),
        10: rng.choice((0, 0x0C00, 0x0100, 0x0E00, rng.getrandbits(16))),
        11: rng.choice((0x04C1, 0x8005, 0x1021, rng.getrandbits(16))),
        12: rng.choice((0x1DB7, 0, rng.getrandbits(16))),
        13: rng.choice((0xC704, 0x800D, 0, rng.getrandbits(16))),
        14: rng.choice((0xDD7B, 0, rng.getrandbits(16))),
        15: rng.choice((0x02FF, 0x00FF, 0xE6EF, 0xE4EF, 0x56EF, 0x5A8E, 0x0287, rng.getrandbits(16))),
    }[k]


def unit_image(rng):
    """128 words: 'ldi, lui, scfg' blocks that write a slot, the other unit rows, pin claims on uio, events, branches
    and random legal words."""
    words = []
    pool = legal_words()
    while len(words) < 128:
        x = rng.random()
        if x < 0.25:
            k = rng.randrange(16)
            v = slot_value(rng, k)
            words += [enc('ldi', n=1, i=v & 255), enc('lui', n=1, i=v >> 8), enc('scfg', s=1, k=k)]
        elif x < 0.32:
            words.append(enc('sgo', m=rng.choice((1, 2, 3))))
        elif x < 0.34:
            words.append(enc('sstop', m=rng.choice((1, 2, 3))))
        elif x < 0.44:
            words.append(enc('txw', s=rng.randrange(8), n=rng.randrange(16), l=int(rng.random() < 0.2),
                             k=rng.randrange(4)))
        elif x < 0.50:
            words.append(enc('rxw', d=rng.randrange(8)))
        elif x < 0.53:
            words.append(enc('sstat', d=rng.randrange(8)))
        elif x < 0.55:
            words.append(enc('srd', d=rng.randrange(8), s=rng.randrange(3)))
        elif x < 0.61:
            words.append(enc('smap', p=rng.choice((0, 1, 2, 4, 5, 8, 9, 10, 11)), l=rng.randrange(3)))
        elif x < 0.66:
            p = rng.choice((8, 9, 10, 11, 12))
            words.append(0x9000 | p << 8 | 7 << 5 | rng.randrange(3) << 3)     # pcfg p, IN, PP or OD
        elif x < 0.74:
            words.append(rng.choice((enc('now'), enc('tick', i=rng.randrange(1, 40)))))
        elif x < 0.78:
            words.append(enc('goto', a=rng.randrange(128)))
        else:
            words.append(rng.choice(pool) if rng.random() < 0.95 else rng.getrandbits(16))
    return {a: w for a, w in enumerate(words[:128])}


def mail_value(rng):
    return rng.choice((rng.randrange(1, 64), rng.randrange(256), rng.getrandbits(16)))


def thread8(rng):
    return rng.randrange(4) + 4 * rng.choice((0, 0, 0, 0, 1, 63))     # 9.2: only the low 2 bits count


def host_command(rng, host, depth):
    """One host command or None, for the cycle that just ended."""
    if host == 'mail':
        k = rng.randrange(60)
        if k == 0:
            return 'post', rng.randrange(8), mail_value(rng)
        if k == 1:
            return 'take', rng.randrange(8)
        return None
    k = rng.randrange(100)
    if k == 0:
        return 'post', rng.randrange(8), mail_value(rng)
    if k == 1:
        return 'take', rng.randrange(8)
    if k == 2:
        return 'run', rng.getrandbits(4), 0
    if k == 3:
        return 'run', 0, rng.getrandbits(4)
    if k == 4 and rng.random() < 0.3:
        return 'run', rng.getrandbits(4), rng.getrandbits(4)
    if k == 5 and rng.random() < 0.1:
        return ('abort',)
    if k == 6:
        return 'step', thread8(rng)
    if k == 7:
        return 'setpc', thread8(rng), rng.randrange(depth) if rng.random() < 0.9 else rng.getrandbits(16)
    if k in (8, 9, 10):
        return 'rdreg', thread8(rng), rng.choice((rng.randrange(40), rng.randrange(12, 32)))
    if k == 11:
        return ('status',)
    if k == 12 and rng.random() < 0.1:
        return 'wrmem', rng.getrandbits(16), random_word(rng) if rng.random() < 0.9 else rng.getrandbits(16)
    if k == 13:
        return 'rdmem', rng.getrandbits(16)
    return None


# ---- one run --------------------------------------------------------------------------------------------------

def run(words, depth, seed, cycles, starts, inputs, loop, host):
    """The three simulators from reset to cycles, compared after every cycle. Returns (why, stats): why is None, or
    (cycle, verdict, detail) at the first difference."""
    sims = (Hand(words, depth, seed), Text(sim2, words, depth, seed), Text(docsim, words, depth, seed))
    a, b, d = sims
    rng = random.Random(seed * 7919 + 1)
    stats = dict(cycles=0, host={}, rdreg_unit=0, rdreg_unit_nonzero=0, sel_cycles=0, rows={})

    def done(why):
        for _, _, _, w in a.m.log:
            r = asm.decode(w)
            if r:
                stats['rows'][r[0].syntax] = stats['rows'].get(r[0].syntax, 0) + 1
        return why, stats

    if not a.mem == b.mem == d.mem:
        return done((0,) + memory_diff(a.mem, b.mem, d.mem))
    pins = Pins(loop)
    lead = 1 + seed % 4                                 # cycles before the first RUN, so h is 0 to 3
    for c in range(cycles):
        if c == lead:
            for tid, pc in starts:
                for x in sims:
                    x.host(('setpc', tid, pc))
                    x.host(('run', 0, 1 << tid))
        inputs(rng, pins)
        sa, sb, sd = (a.cycle(pins),) + a.state(), (b.cycle(pins),) + b.state(), (d.cycle(pins),) + d.state()
        stats['cycles'] = c + 1
        if not sa == sb == sd:
            return done((c,) + verdict(sa, sb, sd))
        if any(a.m.SEL):
            stats['sel_cycles'] += 1
        if c % 64 == 63 and not a.mem == b.mem == d.mem:
            return done((c,) + memory_diff(a.mem, b.mem, d.mem))
        cmd = host_command(rng, host, depth) if c >= lead else None
        if cmd:
            ra, rb, rd = a.host(cmd), b.host(cmd), d.host(cmd)
            stats['host'][cmd[0]] = stats['host'].get(cmd[0], 0) + 1
            if cmd[0] == 'rdreg' and 12 <= cmd[2] < 32:
                stats['rdreg_unit'] += 1
                stats['rdreg_unit_nonzero'] += ra != 0
            if not ra == rb == rd:
                return done((c, odd_one(ra, rb, rd), 'reply to %s: %s' % (cmd, three(ra, rb, rd))))
    if not a.mem == b.mem == d.mem:
        return done((cycles - 1,) + memory_diff(a.mem, b.mem, d.mem))
    return done(None)


def held(hold):
    def inputs(rng, pins):
        if hold == 1 or rng.randrange(hold) == 0:
            pins.ui, pins.io = rng.getrandbits(8), rng.getrandbits(8)
    return inputs


def busy_inputs(rng, pins):
    if rng.random() < 0.3:
        pins.ui = rng.getrandbits(8)
    if rng.random() < 0.3:
        pins.io = rng.getrandbits(8)


def job(spec):
    """One run, from its description, so it can run in another process."""
    kind, label, seed, cycles = spec[:4]
    rng = random.Random(seed)
    if kind == 'program':
        mode = spec[4]
        with open(os.path.join(ROOT, label)) as fh:
            img = asm.assemble(fh.read(), depth=512)
        entry = img.labels.get('start', min(img.words))
        if mode == 'labels':
            pcs = sorted(set(img.labels.values()) | {entry})
            why, st = run(img.words, 512, seed, cycles, [(t, rng.choice(pcs)) for t in range(4)],
                          held(rng.choice((1, 3, 20, 200))), True, 'all')
        else:
            why, st = run(img.words, 512, seed, cycles, [(0, entry)], held(1 if mode == 'entry' else 40), False,
                          'mail')
    elif seed % 2 == 0:
        words = {x: (random_word(rng) if rng.random() < 0.93 else rng.getrandbits(16)) for x in range(128)}
        hold = rng.choice((1, 3, 20, 200))
        why, st = run(words, 128, seed, cycles, [(i, rng.randrange(128)) for i in range(4)], held(hold), False,
                      'all')
    else:
        words = unit_image(rng)
        why, st = run(words, 128, seed, cycles, [(i, rng.randrange(128)) for i in range(4)], busy_inputs, True,
                      'all')
    return spec, why, st


def main(argv):
    quick = '--quick' in argv
    n_images = int(_arg('--images', 40 if quick else 300))
    seed0 = int(_arg('--seed', 0))
    jobs = int(_arg('--jobs', 1))
    stop = '--stop' in argv
    p_cycles = int(_arg('--program-cycles', 2000 if quick else 8000))
    i_cycles = int(_arg('--image-cycles', 3000 if quick else 6000))
    names = set(isatable.UNIT) - {'CFG'}
    if names != set(NAMES):
        print('the per-thread names of 5.9.2 in %s and in sim.py differ: %s' % (isatable.DOC, sorted(names ^ set(NAMES))))
        return 2
    print('sim.py and asm.py read %s' % os.path.normpath(asm.DOC))
    print('sim2.py and docsim.py read %s' % os.path.normpath(isatable.DOC))
    specs = []
    if '--no-programs' not in argv:
        files = sorted(os.path.relpath(p, ROOT) for p in glob.glob(os.path.join(ROOT, 'programs', '*', '*.s')))
        for i, f in enumerate(files):
            for k, mode in enumerate(('entry', 'entry40', 'labels')):
                specs.append(('program', f, 1000 + 3 * i + k, p_cycles, mode))
    specs += [('image', 'image %d' % s, s, i_cycles) for s in range(seed0, seed0 + n_images)]
    t0 = time.time()
    tot = dict(cycles=0, host={}, rdreg_unit=0, rdreg_unit_nonzero=0, sel_cycles=0)
    groups = {}
    bad = []
    rows = {}
    pool = multiprocessing.Pool(jobs) if jobs > 1 else None
    it = pool.imap(job, specs) if pool else map(job, specs)
    for n, (spec, why, st) in enumerate(it, 1):
        g = groups.setdefault(spec[0] if spec[0] == 'program' else ('image, uniform' if spec[2] % 2 == 0
                                                                     else 'image, shift units'), [0, 0, 0])
        g[0] += 1
        g[1] += st['cycles']
        for k in ('cycles', 'rdreg_unit', 'rdreg_unit_nonzero', 'sel_cycles'):
            tot[k] += st[k]
        for k, v in st['host'].items():
            tot['host'][k] = tot['host'].get(k, 0) + v
        for k, v in st['rows'].items():
            rows[k] = rows.get(k, 0) + v
        if why:
            g[2] += 1
            bad.append((spec, why))
            label = spec[1] + (' (%s)' % spec[4] if spec[0] == 'program' else '')
            print('DIFF  %s: cycle %d, %s: %s' % (label, why[0], why[1], why[2]), flush=True)
            if stop:
                break
        elif n % 100 == 0:
            print('      %d of %d runs, %d differ, %.0f s' % (n, len(specs), len(bad), time.time() - t0), flush=True)
    if pool:
        if bad and stop:
            pool.terminate()
        else:
            pool.close()
            pool.join()
    for g, (runs, cyc, nbad) in groups.items():
        print('%-20s %5d runs, %10d cycles, %d differ' % (g, runs, cyc, nbad))
    print('host commands compared: %s; RDREG of 12 to 31: %d, %d of them not 0'
          % (', '.join('%s %d' % kv for kv in sorted(tot['host'].items())), tot['rdreg_unit'],
             tot['rdreg_unit_nonzero']))
    print('cycles with a pin on a shift output: %d' % tot['sel_cycles'])
    print('rows sim.py ran: %d of %d; shift-unit rows, times run (a row that blocks counts at every slot it tries): %s'
          % (len(rows), len(ROWS), ', '.join('%s %d' % (r.mn, rows.get(r.syntax, 0)) for r in ROWS if r.mn in UNIT_MN)))
    kinds = {}
    for _, why in bad:
        kinds[why[1]] = kinds.get(why[1], 0) + 1
    print('=== %d runs, %d cycles, %d differ%s, %.0f s' % (sum(g[0] for g in groups.values()), tot['cycles'], len(bad),
                                                          ': ' + ', '.join('%s in %d' % kv for kv in sorted(kinds.items()))
                                                          if bad else '', time.time() - t0))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
