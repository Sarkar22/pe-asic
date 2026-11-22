#!/usr/bin/env python3
"""Runs the three simulators in this folder side by side and compares them on every cycle.

    sim.py      written by hand from the text
    sim2.py     written by hand from the text too, apart from the row semantics, which it compiles from the
                table, and the shift unit, which it compiles from the code blocks of section 5.9
    docsim.py   compiled from the code blocks and the table; it uses sim2.py's host framing

All three run the same memory image, with the same inputs and the same host commands. After every cycle this
compares the pad values in both halves of the cycle (3.1, 5.2), every thread's registers and flags, every shift unit's state and its 16 slots (5.9.2),
the pin flops and pin owners, the output switch, the mailbox and BUSY, and the whole memory every 64 cycles. It
also compares the reply to every host command.

What runs:
- every program under programs/, three ways: thread 0 from its first word with the inputs changing every cycle,
  the same with the inputs changing every 40 cycles on average, both with mailbox traffic (POST and TAKE), and
  all four threads started at labels of the program, with the uio pads looped back and every host command;
- random images of 128 words. Even seeds draw each word uniformly over the table's rows, then over its legal
  field values, with 7% random words, some of them illegal. Odd seeds are biased to the shift-unit rows of
  section 5.9.4, with slot values that make the units run, and the uio pads looped back, so units receive their
  own and each other's sends;
- random images of the same kind with half cycles (section 3.1): slot values biased to THALF, RKIND 3 and
  REOP's byte end (5.9.5 to 5.9.7), inputs with their own level at each falling edge, and the uio pads looped
  back with each half's level, so a pad moved at a falling edge reaches the falling-edge sample with its old
  level and the rising-edge sample with its new one. The other runs give one level per cycle;
- directed images for RKIND 3 (5.9.6): each thread's receive half hunts on its own ui pin, which carries 802.3
  Manchester frames at the decoder's bit time or near it, timed to the half cycle, with random edge jitter, the
  fixed patterns of section 7.3 (bit-boundary edges late and mid-bit edges early, and the reverse), phase steps,
  pulses of half a cycle and frames of 1 to 7 bits more than whole octets, so that edges come in both halves of a
  cycle, candidates give their bits at the deadline and REOP's byte end of a Manchester frame runs (5.9.7).

The host commands are POST, TAKE, RUN with stop and start masks, STEP, SETPC, RDREG of registers 0 to 39,
STATUS, WRMEM, RDMEM and ABORT. sim2.py and docsim.py take them as bytes, through their framing, and sim.py
through its host_* calls. Threads start a few cycles after reset, so the cycle h of section 9.1 is never before
cycle 0.

sim2.py and docsim.py lag a bypass path by one cycle, as section 10 allows (undefined, item 3), so sim.py is
given the previous cycle's ui for its pads here. All three serve a host command in one call between two cycles,
as section 10 allows a byte-level simulator (undefined, item 6).

Usage, from the repository root. Python 3, standard library only:
    python3 sim/compare.py                  every program, 300 random images and 100 with half cycles, 6000 cycles
    python3 sim/compare.py --quick          every program with shorter runs, 40 images and 20 with half cycles
    python3 sim/compare.py --images N --seed S      N images from seed S
    python3 sim/compare.py --half-images N  N images with half cycles
    python3 sim/compare.py --rk3-images N   N directed RKIND 3 images (100, or 20 with --quick)
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
PARTS = ('pads', 'threads', 'units', 'SEL', 'SHSEL', 'SYNC3', 'SYNC2', 'SYNC2F', 'FULL', 'M', 'OUT', 'SHD', 'OE', 'SHOE',
         'OD', 'BYP', 'SHBYP', 'OWN', 'BUSY')
UNIT_MN = ('txw', 'rxw', 'sstat', 'srd', 'scfg', 'sgo', 'sstop', 'smap')


def bits(lst):
    v = 0
    for i, x in enumerate(lst):
        v |= x << i
    return v


# ---- the three simulators behind one interface ----------------------------------------------------------------

class Pins:
    """The board. ui and the uio levels; with loop, a uio pad the chip drives reads its own drive (5.2). With half,
    ui_f and io_f are the levels at each falling edge, and a looped-back pad gives the falling-edge sample its level
    in the first half and the rising-edge sample its level in the second (3.1); without, one level per cycle."""

    def __init__(self, loop, half=False):
        self.ui = self.io = 0
        self.ui_f = self.io_f = 0
        self.loop, self.half = loop, half

    def levels(self, en, val, io=None):
        io = self.io if io is None else io
        return (val & en) | (io & ~en & 0xFF) if self.loop else io

    def edges(self, first, second):
        """(ui, uio) at the rising edge that ends the cycle, then at its falling edge, from the pads of each half as
        (uo, en, val)."""
        if not self.half:
            lv = self.levels(first[1], first[2])
            return self.ui, lv, self.ui, lv
        return (self.ui, self.levels(second[1], second[2]), self.ui_f,
                self.levels(first[1], first[2], self.io_f))


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
        uo2, val2, en2 = m.pads2(self.prev_ui)
        first, second = (uo, en, val), (uo2, en2, val2)
        ui, lv, ui_f, lv_f = pins.edges(first, second)
        if pins.half:
            m.cycle(ui, lv, ui_f, lv_f)
        else:
            m.cycle(ui, lv)
        self.prev_ui = pins.ui
        return first, second

    def state(self):
        m = self.m
        th = tuple((t.PC, tuple(t.R), t.T, t.D, t.S, t.LATE, t.STALE, t.ILL, t.RUN, t.P, t.WAITING, t.STOPREQ,
                    t.STEPPING) for t in m.th)
        units = tuple(tuple([getattr(t.u, n) for n in NAMES]) + tuple(t.u.CFG) for t in m.th)
        return (th, units, tuple(m.SEL), tuple(m.SHSEL), m.SYNC3, m.SYNC2, m.SYNC2F, tuple(m.FULL), tuple(m.M),
                bits(m.OUT),
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

    def step2(self, chip, c, first, second):        # called by Chip.cycle, after the pads, for this cycle's inputs
        self.seen = (first, second)
        return self.pins.edges(first, second)

    def cycle(self, pins):
        self.pins = pins
        self.ch.cycle(self)
        return self.seen

    def state(self):
        ch = self.ch
        th = tuple((t.pc, tuple(t.r), t.T, t.D, t.S, t.late, t.stale, t.ill, t.run, t.P, t.waiting, t.stopreq,
                    t.stepping) for t in ch.t)
        units = tuple(tuple([getattr(t.u, n) for n in NAMES]) + tuple(t.u.CFG) for t in ch.t)
        return (th, units, tuple(ch.sel), tuple(ch.shsel), ch.sync3, ch.sync2, ch.sync2f, tuple(ch.full),
                tuple(ch.mail),
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


def slot_value(rng, k, half=False):
    """5.9.3: values that make a unit do something. Short periods, decoders on the looped-back uio pads. With half,
    biased to THALF with fractions, RKIND 3 with short bits, and REOP."""
    v = {
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
    if half:
        if k == 0 and rng.random() < 0.7:
            v |= 0x8000                                             # THALF
        elif k == 2 and rng.random() < 0.5:
            v = rng.choice((1, 2, 2, 3, 4))
        elif k == 3 and rng.random() < 0.6:
            v = (v & 0xFF) | rng.choice((128, 128, 64, 192, 200, 255, 1)) << 8     # TFRAC
        elif k == 4 and rng.random() < 0.7:
            v |= 3                                                  # RKIND 3
        elif k == 5 and rng.random() < 0.6:
            v = rng.choice((0, 1, 2, 3, 5, 5, 8))
        elif k == 15 and rng.random() < 0.4:
            v |= 0x8000                                             # REOP, with the byte end
    return v


def unit_image(rng, half=False):
    """128 words: 'ldi, lui, scfg' blocks that write a slot, the other unit rows, pin claims on uio, events, branches
    and random legal words. half: slot values for half cycles (slot_value)."""
    words = []
    pool = legal_words()
    while len(words) < 128:
        x = rng.random()
        if x < 0.25:
            k = rng.randrange(16)
            v = slot_value(rng, k, half)
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


def half_image(rng):
    """Directed, for half cycles: thread 0's unit sends Manchester with THALF on io0 and io1 (shift outputs A and B),
    thread 1's receives io0 with RKIND 3 at that bit time or near it, thread 3's unit sends NRZ on io2 and io3 with
    a tail where both are low, and thread 2's receives io2 with timed samples and REOP, RCLK on io3, so a reception
    ends at that SE0 and often with 1 to 6 ones after its last byte boundary (5.9.7, the byte end). The uio pads loop
    back. Each thread ends in a loop that feeds or drains its unit, with random unit rows among them. Returns (words,
    starts)."""
    words, starts = [], []

    def li(v):
        words.extend((enc('ldi', n=1, i=v & 255), enc('lui', n=1, i=(v >> 8) & 255)))

    def cfg(k, v):
        li(v)
        words.append(enc('scfg', s=1, k=k))

    def pin(p, mode):                                       # pcfg p, IN (0), PP (1) or OD (2)
        words.append(0x9000 | p << 8 | 7 << 5 | mode << 3)

    def noise(n):
        for _ in range(n):
            x = rng.random()
            if x < 0.3:
                words.append(enc('sstat', d=rng.randrange(2, 8)))
            elif x < 0.5:
                words.append(enc('srd', d=rng.randrange(2, 8), s=rng.randrange(3)))
            elif x < 0.6:
                words.append(enc('tick', i=rng.randrange(1, 60)))
            else:
                words.append(enc('now'))

    tper, tfrac, syms = rng.choice((1, 2, 2, 3)), rng.choice((0, 64, 128, 128, 192, 200)), 1
    bt = (syms + 1) * (tper * 256 + tfrac)                  # the sender's bit, in 256ths of a cycle
    if rng.random() < 0.3:
        bt += rng.choice((-40, -12, 12, 40))
    tper3, tfrac3 = rng.choice((2, 3, 5)), rng.choice((0, 128, 64))
    for tid in range(4):
        starts.append((tid, len(words)))
        if tid == 0:                                        # Manchester with THALF
            cfg(0, 0x8000 * (rng.random() < 0.9) | syms | rng.getrandbits(1) << 5 | rng.getrandbits(1) << 6 |
                rng.getrandbits(1) << 7 | rng.getrandbits(1) << 8)
            cfg(1, rng.choice((0x0401, 0x0401, 0x0104, 0x0E00)))
            cfg(2, tper)
            cfg(3, rng.randrange(0, 12) | tfrac << 8)
            cfg(15, rng.choice((0, 0, 0x02FF, rng.getrandbits(16) & 0x7FFF)))
        elif tid == 1:                                      # RKIND 3, HUNT, from io0
            cfg(4, 3 | rng.getrandbits(1) << 3 | rng.getrandbits(1) << 4 | 1 << 5 | rng.getrandbits(1) << 6 |
                8 << 8 | rng.getrandbits(4) << 12)
            cfg(5, max(0, bt >> 8) if rng.random() < 0.85 else rng.choice((0, 1, 2)))
            cfg(10, (bt & 255) | rng.choice((0, 0x0200, 0x0600, 0x0E00)))
            cfg(8, rng.choice((0, 0, 0x4010, 0x8000 | rng.randrange(1, 60))))
            cfg(9, rng.getrandbits(16))
        elif tid == 3:                                      # NRZ, a tail with both outputs low
            cfg(0, 0x8000 * (rng.random() < 0.7) | 1 << 5)
            cfg(1, 0x0100)
            cfg(2, tper3)
            cfg(3, rng.randrange(2 * tper3 + 1, 8 * tper3) | tfrac3 << 8)
        else:                                               # timed samples with REOP, from io2, RCLK io3
            cfg(4, 0 | 1 << 5 | 1 << 6 | 10 << 8 | 11 << 12)
            cfg(5, tper3)
            cfg(6, (tper3 + 1) // 2)
            cfg(10, tfrac3)
            cfg(15, 0x8000 | rng.choice((0, 0, 0x0400 | 6 << 12, rng.getrandbits(15))))
        if tid in (0, 3):
            p = 8 if tid == 0 else 10
            pin(p, 1)
            pin(p + 1, 1)
            words.append(enc('smap', p=p, l=1))
            words.append(enc('smap', p=p + 1, l=2))
            words.append(enc('now'))
            top = len(words)
            for _ in range(rng.randrange(1, 4)):
                li(rng.choice((0xFFFF, 0xFFFF, 0x7FFF, rng.getrandbits(16))))
                words.append(enc('txw', s=1, n=rng.randrange(16), l=int(rng.random() < 0.4), k=rng.randrange(4)))
            words.append(enc('sgo', m=1))
            noise(rng.randrange(1, 4))
            if rng.random() < 0.2:
                words.append(enc('sstop', m=1))
            words.append(enc('goto', a=top))
        else:
            words.append(enc('sgo', m=2))
            words.append(enc('now'))
            top = len(words)
            words.append(enc('rxw', d=rng.randrange(2, 8)))
            noise(rng.randrange(1, 4))
            if rng.random() < 0.15:
                words.append(enc('sstop', m=2))
                words.append(enc('sgo', m=2))
            words.append(enc('goto', a=top))
    assert len(words) <= 128, len(words)
    return {a: w for a, w in enumerate(words)}, starts


def rk3_image(rng, cycles):
    """Directed, for RKIND 3: thread k's receive half hunts on ui pin k with its own period, gain, window, sync,
    CRC and REOP, and takes words in a loop with random unit rows. Returns (words, starts, levels): levels[k][n] is
    ui pin k's level at half cycle n, n = 2c at the falling edge of cycle c and 2c + 1 at the rising edge that ends
    it (3.1). The line is 802.3 Manchester (a 1 is low then high), high when idle, so a hunt for a rising edge
    starts at the first preamble bit's clock edge."""
    words, starts, levels = [], [], []

    def li(v):
        words.extend((enc('ldi', n=1, i=v & 255), enc('lui', n=1, i=(v >> 8) & 255)))

    def cfg(k, v):
        li(v)
        words.append(enc('scfg', s=1, k=k))

    for tid in range(4):
        starts.append((tid, len(words)))
        rper = rng.choice((2, 2, 3, 4, 5, 5, 5, 6, 8, 11, 15, 0, 1, 16, 23))
        rfrac = rng.choice((0, 0, 128, 64, rng.getrandbits(8)))
        p = 512 if rper < 2 else (0xF00 | rfrac) if rper > 15 else (rper << 8) | rfrac
        rinv, rdiff, rmsb = rng.getrandbits(1), int(rng.random() < 0.12), rng.getrandbits(1)
        hlvl = 1 if rng.random() < 0.85 else 0
        cfg(4, 3 | rmsb << 2 | rinv << 3 | rdiff << 4 | 1 << 5 | hlvl << 6 | tid << 8 | rng.getrandbits(4) << 12)
        cfg(5, rper)
        cfg(10, rfrac | rng.choice((0, 1, 2, 3, 3)) << 9 | rng.getrandbits(1) << 11 | rng.choice((0, 0, 0, 3)) << 12)
        synclen = rng.choice((0, 8, 16, 16))
        sync = [rng.getrandbits(1) for _ in range(synclen)]
        rcnt = 0 if rng.random() < 0.9 else rng.randrange(1, 200)
        cfg(8, synclen << 11 | rcnt)
        dec = [b ^ rinv for b in sync]                      # the bits the decoder gives for them, without RDIFF
        if rmsb:
            pat = sum(b << (synclen - 1 - i) for i, b in enumerate(dec))
        else:
            pat = sum(b << (16 - synclen + i) for i, b in enumerate(dec))
        cfg(9, pat if rng.random() < 0.9 else rng.getrandbits(16))
        if rng.random() < 0.15:
            cfg(7, rng.randrange(1, 4 * max(2, rper)))      # REND, an end that keeps every bit
        cmode = rng.choice((1, 1, 1, 1, 2, 0))
        smode = 0 if rng.random() < 0.85 else rng.choice((1, 2))
        cw = rng.choice((31, 31, 15, 7, rng.randrange(32)))
        cfg(15, cw | rng.getrandbits(1) << 5 | rng.getrandbits(1) << 6 | cmode << 7 | int(rng.random() < 0.1) << 9 |
            smode << 10 | rng.randrange(8) << 12 | int(rng.random() < 0.7) << 15)
        if tid < 2:
            cfg(11, rng.choice((0x04C1, 0x8005, rng.getrandbits(16))))
            cfg(12, rng.choice((0x1DB7, 0, rng.getrandbits(16))))
        words.append(enc('sgo', m=2))
        words.append(enc('now'))
        top = len(words)
        words.append(enc('rxw', d=rng.randrange(2, 8)))
        for _ in range(rng.randrange(0, 3)):
            words.append(rng.choice((enc('srd', d=rng.randrange(2, 8), s=rng.randrange(3)),
                                     enc('sstat', d=rng.randrange(2, 8)))))
        words.append(enc('goto', a=top))
        levels.append(manch_line(rng, p, sync, 2 * cycles + 2))
    assert len(words) <= 128, len(words)
    return {a: w for a, w in enumerate(words)}, starts, levels


def manch_line(rng, p, sync, n):
    """n half-cycle levels of one pin: idle high for about 260 cycles, then frames of a preamble, the sync bits and
    random data with gaps between them. The bit time is p / 256 cycles, sometimes a little off, and each frame
    gets its own jitter: none, random, bit-boundary edges late and mid-bit ones early or the reverse, a phase step
    from a random bit on, or pulses of half a cycle."""
    bt = p / 256.0 * rng.choice((1.0, 1.0, 1.0, 1.002, 0.998, 1.02, 0.98))
    hb = bt / 2
    events, pulses = [], []
    t = 260.0 + rng.random() * 8
    while True:
        nbits = rng.randrange(6, 20) + len(sync) + 8 * rng.randrange(1, 6) + rng.choice((0, 0, 0, 1, 2, 3, 4, 5, 6, 7))
        if 2 * (t + nbits * bt) + 4 >= n:
            break
        pre = []
        while len(pre) < nbits - len(sync) - 8:
            pre.append(1 - len(pre) % 2)                    # 1, 0, 1, 0, ...: 802.3's preamble from its first bit
        bits = pre + sync + [rng.getrandbits(1) for _ in range(nbits - len(pre) - len(sync))]
        half = []
        for b in bits:
            half += [1 - b, b]
        mode = rng.choice(('none', 'rand', 'rand', 'late', 'early', 'step', 'pulse'))
        a = rng.uniform(0.05, 0.32) * hb
        jstep, s = rng.randrange(len(bits)), rng.uniform(-0.45, 0.45) * hb
        prev, edges = 1, []
        for i, v in enumerate(half + [1]):                 # half-bit cell i starts at t + i * hb; idle high after
            if v == prev:
                continue
            prev = v
            if mode == 'rand':
                j = rng.uniform(-a, a)
            elif mode in ('late', 'early'):
                j = (a if i % 2 == 0 else -a) * (1 if mode == 'late' else -1)
            elif mode == 'step':
                j = s if i >= 2 * jstep else 0.0
            else:
                j = 0.0
            edges.append((t + i * hb + j, v))
        for x, v in edges:                                  # from the first half-cycle sample at or after the edge
            events.append((int(-(-2 * x // 1)), len(events), v))
        if mode == 'pulse':
            pulses += [int(2 * (t + rng.uniform(0, len(bits) * bt))) for _ in range(rng.randrange(1, 4))]
        t += len(bits) * bt + rng.uniform(3, 40) * bt
    lv, q, v = [], 0, 1
    for k, _, w in sorted(events):
        lv += [v] * (min(k, n) - len(lv))
        v = w
    lv += [v] * (n - len(lv))
    for q in pulses:
        if 0 < q < n - 1 and lv[q - 1] == lv[q] == lv[q + 1]:
            lv[q] ^= 1                                      # half a cycle: one edge in each half of a cycle
    return lv


def rk3_inputs(levels):
    """The inputs of an rk3_image: ui pin k from levels[k] at both edges, ui 4 to 7 and the uio pads busy."""
    c = [0]

    def inputs(rng, pins):
        busy_inputs(rng, pins)
        n = 2 * c[0]
        lo_f = sum(levels[k][n] << k for k in range(4))
        lo = sum(levels[k][n + 1] << k for k in range(4))
        pins.ui = (pins.ui & 0xF0) | lo
        pins.ui_f = (pins.ui & 0xF0) | lo_f
        pins.io_f = pins.io
        c[0] += 1
    return inputs


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

def run(words, depth, seed, cycles, starts, inputs, loop, host, half=False):
    """The three simulators from reset to cycles, compared after every cycle. Returns (why, stats): why is None, or
    (cycle, verdict, detail) at the first difference."""
    sims = (Hand(words, depth, seed), Text(sim2, words, depth, seed), Text(docsim, words, depth, seed))
    a, b, d = sims
    rng = random.Random(seed * 7919 + 1)
    stats = dict(cycles=0, host={}, rdreg_unit=0, rdreg_unit_nonzero=0, sel_cycles=0, rows={}, halves=0, rk3=0,
                 byte_end=0, rk3_both=0, rk3_deadline=0, rk3_mend=0)

    def done(why):
        for _, _, _, w in a.m.log:
            r = asm.decode(w)
            if r:
                stats['rows'][r[0].syntax] = stats['rows'].get(r[0].syntax, 0) + 1
        return why, stats

    if not a.mem == b.mem == d.mem:
        return done((0,) + memory_diff(a.mem, b.mem, d.mem))
    pins = Pins(loop, half)
    lead = 1 + seed % 4                                 # cycles before the first RUN, so h is 0 to 3
    for c in range(cycles):
        if c == lead:
            for tid, pc in starts:
                for x in sims:
                    x.host(('setpc', tid, pc))
                    x.host(('run', 0, 1 << tid))
        inputs(rng, pins)
        ends = [t.u.RXRUN and t.u.REOP and t.u.RXKIND == 0 and t.u.RXONES and 0 < (t.u.RXTOT & 7) < 7
                for t in a.m.th] if half else ()
        pre3 = [(t.u.RXRUN and t.u.RXKIND == 3, t.u.RXOPEN and t.u.RXCAND,
                 t.u.REOP and not t.u.RXVIO and (t.u.RXTOT & 7) != 0,
                 ((a.m.SYNC3 ^ a.m.SYNC2F) >> t.u.RPIN) & 1, ((a.m.SYNC2F ^ a.m.SYNC2) >> t.u.RPIN) & 1)
                for t in a.m.th] if half else ()
        sa, sb, sd = (a.cycle(pins),) + a.state(), (b.cycle(pins),) + b.state(), (d.cycle(pins),) + d.state()
        stats['cycles'] = c + 1
        if not sa == sb == sd:
            return done((c,) + verdict(sa, sb, sd))
        if any(a.m.SEL):
            stats['sel_cycles'] += 1
        if half:                                        # what the half-cycle images reach
            stats['halves'] += sa[0][0] != sa[0][1]
            stats['rk3'] += any(t.u.RXRUN and t.u.RXKIND == 3 for t in a.m.th)
            stats['byte_end'] += sum(1 for t, e in zip(a.m.th, ends) if e and not t.u.RXRUN and
                                     t.u.RXN == t.u.RXN8 and t.u.RXSR == t.u.RXSR8)
            for t, (run3, cand, mend, e1, e2) in zip(a.m.th, pre3):
                if run3:
                    stats['rk3_both'] += e1 and e2
                    stats['rk3_deadline'] += cand and not (e1 or e2) and not t.u.RXOPEN
                    stats['rk3_mend'] += mend and not t.u.RXRUN and t.u.RXN == t.u.RXN8 and t.u.RXSR == t.u.RXSR8
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


def half_inputs(rng, pins):
    """busy_inputs, and the levels at the falling edge: the old one (the edge comes in the second half), the new one
    (in the first half) or another (a pulse between two rising edges)."""
    ui, io = pins.ui, pins.io
    busy_inputs(rng, pins)
    x, y = rng.random(), rng.random()
    pins.ui_f = ui if x < 0.4 else pins.ui if x < 0.85 else rng.getrandbits(8)
    pins.io_f = io if y < 0.4 else pins.io if y < 0.85 else rng.getrandbits(8)


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
    elif kind == 'half' and seed % 2 == 0:
        words, starts = half_image(rng)
        why, st = run(words, 128, seed, cycles, starts, half_inputs, True, 'mail', half=True)
    elif kind == 'rk3':
        words, starts, levels = rk3_image(rng, cycles)
        why, st = run(words, 128, seed, cycles, starts, rk3_inputs(levels), True, 'mail', half=True)
    elif kind == 'half':
        words = unit_image(rng, half=True)
        why, st = run(words, 128, seed, cycles, [(i, rng.randrange(128)) for i in range(4)], half_inputs, True,
                      'all', half=True)
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
    n_half = int(_arg('--half-images', 20 if quick else 100))
    n_rk3 = int(_arg('--rk3-images', 20 if quick else 100))
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
    specs += [('half', 'half image %d' % s, s, i_cycles) for s in range(50000 + seed0, 50000 + seed0 + n_half)]
    specs += [('rk3', 'RKIND 3 image %d' % s, s, i_cycles) for s in range(70000 + seed0, 70000 + seed0 + n_rk3)]
    t0 = time.time()
    tot = dict(cycles=0, host={}, rdreg_unit=0, rdreg_unit_nonzero=0, sel_cycles=0, halves=0, rk3=0, byte_end=0,
               rk3_both=0, rk3_deadline=0, rk3_mend=0)
    groups = {}
    bad = []
    rows = {}
    pool = multiprocessing.Pool(jobs) if jobs > 1 else None
    it = pool.imap(job, specs) if pool else map(job, specs)
    for n, (spec, why, st) in enumerate(it, 1):
        g = groups.setdefault(spec[0] if spec[0] == 'program' else 'image, half cycles' if spec[0] == 'half' else
                              'image, RKIND 3' if spec[0] == 'rk3' else
                              ('image, uniform' if spec[2] % 2 == 0 else 'image, shift units'), [0, 0, 0])
        g[0] += 1
        g[1] += st['cycles']
        for k in ('cycles', 'rdreg_unit', 'rdreg_unit_nonzero', 'sel_cycles', 'halves', 'rk3', 'byte_end',
                  'rk3_both', 'rk3_deadline', 'rk3_mend'):
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
    if n_half:
        print('half-cycle images: %d cycles whose two halves differ on a pad, %d with an RKIND 3 reception, %d '
              'receptions ended at a byte boundary by REOP' % (tot['halves'], tot['rk3'], tot['byte_end']))
    if n_half or n_rk3:
        print('RKIND 3 receptions: %d cycles with an edge in both halves of the cycle, %d bits given by a candidate at '
              'its deadline, %d receptions ended at a byte boundary by the byte end of a Manchester frame'
              % (tot['rk3_both'], tot['rk3_deadline'], tot['rk3_mend']))
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
