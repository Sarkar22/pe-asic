#!/usr/bin/env python3
"""Tests the document's own statements, one small program at a time, on sim.py and asm.py.

Each check names the section it tests. A FAIL means the document and sim.py (or asm.py) disagree,
so each one is either a document error or a bug in sim.py, and has to be traced by hand. The
programs come from programs/, and the board and device models from programs/bench/.

Usage, from the repository root: python3 sim/claims.py
Prints PASS or FAIL for each claim, a few INFO lines, and the count. Exits 1 if any claim fails.
"""
import copy
import os
import random
import re
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(1, os.path.join(HERE, '..', 'programs', 'bench'))
sys.dont_write_bytecode = True

from asm import AsmError, ROWS, assemble, decode       # noqa: E402
from rig import Device                                  # noqa: E402
from basic import Run, spi_case, src, uart_tx_case      # noqa: E402
from asm import encode, legal_value                     # noqa: E402
from sim import Chip, HOLD, PCSET, UNIT_STATE           # noqa: E402


class Log:
    def __init__(self):
        self.n = self.bad = 0

    def line(self, s):
        print(s, flush=True)

    def check(self, name, ok, detail=''):
        self.n += 1
        self.bad += 0 if ok else 1
        self.line('%s  %s%s' % ('PASS' if ok else 'FAIL', name, '  ' + detail if detail else ''))
        return ok


def random_word(rng):
    """Uniform over rows, then over that row's legal field values."""
    while True:
        r = rng.choice(ROWS)
        f = {k: rng.getrandbits(len(b)) for k, b in r.bits.items()}
        if all(legal_value(f[k], r.ftypes[k]) for k in f):
            return encode(r, f)
LIMIT = 3000                            # cycles a check waits for a state, so a broken model fails and doesn't hang
ILLEGAL_WORD = '0xDF80'                  # an illegal word: group 1101 has words starting 1101 1111 1 free (6.3)
UNIT_ROWS = {r.syntax for r in ROWS if r.enc.startswith('1101')}     # 5.9.4: the shift unit's eight rows, group 1101


class Stim(Device):
    """ui and uio levels as functions of the cycle."""

    def __init__(self, ui=None, mask=0xFF):
        self.fn = ui
        self.ui_mask = mask
        self.ui_val = 0

    def step(self, c, uo, io):
        if self.fn:
            self.ui_val = self.fn(c + 1) & self.ui_mask


def go(text, thread=0, align=0, depth=128, devices=(), pullups=0, seed=1):
    img = assemble(text, depth=depth)
    r = Run(depth=depth, devices=devices, pullups=pullups, seed=seed)
    r.load(img, thread)
    r.start(1 << thread, align)
    return r


def first_rise(r, sig, n, after=0):
    return next((t for t, v in r.board.trace.wave(sig, n).edges(after) if v == 1), None)


def chk_reaction(log):
    # 5.7, 7.3, 11.4: pre-armed reaction is 3 cycles pad to pad on every thread and phase
    seen = set()
    for th in range(4):
        for ph in range(8):
            p = 300 + ph
            r = go('pcfg uo1, PP\nnow\nput uo1, 1\nuntil ui5, 1\nhalt\n', th,
                   devices=[Stim(lambda c: 0x20 if c >= p else 0)])
            r.board.run(600)
            seen.add(first_rise(r, 'uo', 1) - first_rise(r, 'ui', 5))
    log.check('5.7 pre-armed reaction is 3 cycles, 4 threads x 8 phases', seen == {3}, str(seen))

    # 5.7: timed reaction, n = 5 smallest on time, 8 pad to pad, no jitter
    res = {}
    for n in (4, 5):
        late = 0
        lat = set()
        for th in range(4):
            for ph in range(8):
                p = 300 + ph
                r = go('pcfg uo1, PP\nnow\nuntil ui5, 1\nedge uo1, 1, %d\ntlate\nhalt\n' % n, th,
                       devices=[Stim(lambda c: 0x20 if c >= p else 0)])
                r.board.run(600)
                lat.add(first_rise(r, 'uo', 1) - first_rise(r, 'ui', 5))
                late += r.chip.th[th].T
        res[n] = (late, lat)
    log.check('5.7 timed reaction n = 5: on time everywhere, 8 cycles, no jitter',
              res[5] == (0, {8}), str(res[5]))
    log.check('5.7 n = 4 is late on some alignment', res[4][0] > 0, '%d of 32 late' % res[4][0])

    # 7.3: untimed reaction 9 to 12; the next instruction 1 to 4 after the event
    lat, resume = set(), set()
    for th in range(4):
        for ph in range(8):
            p = 300 + ph
            r = go('pcfg uo1, PP\nnow\nuntil ui5, 1\nput uo1, 1\nnow\nhalt\n', th,
                   devices=[Stim(lambda c: 0x20 if c >= p else 0)])
            r.board.run(600)
            lat.add(first_rise(r, 'uo', 1) - first_rise(r, 'ui', 5))
            r = go('now\nuntil ui5, 1\nrdtb R1\nhalt\n', th,
                   devices=[Stim(lambda c: 0x20 if c >= p else 0)])
            r.board.run(600)
            resume.add(r.chip.th[th].R[1] - (first_rise(r, 'ui', 5) + 2))
    log.check('7.3 untimed reaction 9 to 12 cycles', lat == {9, 10, 11, 12}, str(sorted(lat)))
    log.check('7.2/7.3 next instruction runs 1 to 4 cycles after a wait ends', resume == {1, 2, 3, 4},
              str(sorted(resume)))


def chk_own_edge(log):
    # 5.5, 11.5: a wait right after a wait whose match committed an edge arms 1 to 4 cycles after E, and
    # compares the old level until E + 3. So it can match on the pin's old level, on one phase in four.
    prog = """
        pcfg  io0, PP
        put   io0, 0
        ldi   R1, 40
        now
        put   io0, 1
        until ui5, 1
        until io0, 0, R1
        rdd   R3
        halt
    """
    res = {}
    for th in range(4):
        for ph in range(8):
            p = 300 + ph
            r = go(prog, th, devices=[Stim(lambda c: 0x20 if c >= p else 0)])
            r.board.run(600)
            t = r.chip.th[th]
            E = first_rise(r, 'ui', 5) + 2
            res[(E - th) % 4] = (t.T, (t.R[3] - E) & 0xFFFF)
    log.check('5.5/11.5 a wait straight after a committing wait sees the old level only when it arms at E + 1',
              res[3] == (1, 2) and all(res[k] == (0, 40) for k in (0, 1, 2)), str(sorted(res.items())))


def chk_sampling(log):
    # 5.7: after a match, tick n snapshots the pad exactly n cycles after the edge
    ok = True
    for th in range(4):
        for n in (5, 6, 7, 8, 9, 10, 13, 20, 37):
            for off in (-1, 0, 1):
                p = 300 + th
                pulse = p + n + off
                r = go('until ui5, 1\ntick %d\nget ui6\nhalt\n' % n, th,
                       devices=[Stim(lambda c: (0x20 if c >= p else 0) | (0x40 if c == pulse else 0))])
                r.board.run(700)
                ok = ok and r.chip.th[th].T == (1 if off == 0 else 0)
    log.check('5.7 tick n after a match snapshots the pad n cycles after the edge', ok)

    # 5.5: S holds what the pads showed 2 cycles before the event
    r = go('tick 40\nrds R1\nrdd R2\nhalt\n', 0, devices=[Stim(lambda c: c & 255)])
    r.board.run(200)
    t = r.chip.th[0]
    age = (t.R[2] - t.R[1]) & 255
    log.check('5.5 S holds the pad value from 2 cycles before the event', age == 2, 'age %d' % age)

    # the same thing for a thread reading its own driven pin
    res = {}
    for gap in (0, 1, 2, 3, 4):
        a = assemble('pcfg io0, PP\nput io0, 1\ntick 40\nhalt\n', depth=128)
        b = assemble('tick %d\nget io0\nhalt\n' % (40 + gap), base=10, depth=128)
        r = Run()
        r.load(a, 0)
        r.load(b, 1)
        r.start(0b11, 2)
        r.board.run(100)
        res[gap] = r.chip.th[1].T
    log.line('INFO  commit of io0 at E, a snapshot at E+g on another thread: %s (1 = sees the edge)' % res)
    log.check('5.2/5.7 a uio change committed at E is in a snapshot from E + 3',
              res == {0: 0, 1: 0, 2: 0, 3: 1, 4: 1}, str(res))


def chk_late_stale(log):
    # 3.5: 1 to 32767 on time, otherwise late
    res = {}
    for delta in (1, 32767, 32768):
        step = delta + 3                # edge executes 3 cycles after D = a + 1
        r = go('li R1, %d\nnow\nedge -, 0, R1\ntlate\nhalt\n' % step, 0)
        r.board.run(33000)
        res[delta] = r.chip.th[0].T
    log.check('3.5 deadline 1 and 32767 cycles after arming on time, 32768 late',
              res == {1: 0, 32767: 0, 32768: 1}, str(res))

    # 10 item 2: lateness is on the distance from the arming cycle, not on the step
    r = go('li R1, 32768\nnow\nedge -, 0, R1\ntlate\nhalt\n')
    r.board.run(33000)
    log.check('10.2 after now, a step of 32768 lands 32765 ahead and is on time', r.chip.th[0].T == 0)
    r = go('li R1, 32772\nnow\nedge -, 0, R1\ntlate\nhalt\n')
    r.board.run(33000)
    log.check('10.2 after now, a step of 32772 lands 32769 ahead and is late', r.chip.th[0].T == 1)

    # 10 item 3: tick 0 from a D in the past is late; from a future D it is the way to land on D
    r = go('now\nidle\nidle\ntick 0\ntlate\nhalt\n')
    r.board.run(100)
    log.check('10.3 tick 0 with D in the past is late', r.chip.th[0].T == 1)
    r = go('rdtb R1\naddi R1, 100\nwrd R1\ntick 0\ntlate\nrdd R2\nhalt\n')
    r.board.run(400)
    t = r.chip.th[0]
    log.check('10.3 tick 0 after wrd of a future cycle fires on time at D', t.T == 0)

    # 3.5 stale
    prog = """
        li    R1, 9000
        now
l:      djnz  R1, l         ; 36000 cycles with D untouched
        rdtb  R2
        edge  uo1, 1, 20
        rdd   R3
        tlate
        halt
    """
    r = go('pcfg uo1, PP\n' + prog)
    r.board.run(37000)
    t = r.chip.th[0]
    log.check('3.5 tick from a stale D: fires next cycle, LATE, D is that cycle, step discarded',
              t.T == 1 and t.R[3] == (t.R[2] + 5) & 0xFFFF, 'D-rdtb=%d' % ((t.R[3] - t.R[2]) & 0xFFFF))

    prog = """
        li    R1, 9000
        ldi   R5, 50
        now
l:      djnz  R1, l
        until ui5, 1, R5
        rdtb  R2
        rdd   R3
        tlate
        halt
    """
    r = go(prog)
    r.board.run(37000)
    t = r.chip.th[0]
    log.check('3.5 timed wait from a stale D times out next cycle with LATE', t.T == 1)

    r = go('ldi R5, 0\nnow\nidle\nidle\nuntil ui5, 1, R5\nbs R6, 0\ntlate\nhalt\n')
    r.board.run(100)
    t = r.chip.th[0]
    log.check('3.5 until with Rt = 0 samples and tests, no LATE', t.R[6] == 0 and t.T == 0)
    r = go('ldi R5, 0\nnow\nidle\nidle\nidle\nidle\nuntil ui5, 1, R5\nbs R6, 0\ntlate\nhalt\n',
           devices=[Stim(lambda c: 0x20)])
    r.board.run(100)
    t = r.chip.th[0]
    log.check('3.5 the same with the input high gives T = 1', t.R[6] == 1 and t.T == 0)

    # a wait with no timeout, long enough to go stale, then now
    r = go('now\nuntil ui5, 1\ntlate\nbs R6, 0\nedge -, 0, 20\ntlate\nhalt\n',
           devices=[Stim(lambda c: 0x20 if c > 34000 else 0)])
    r.board.run(35000)
    t = r.chip.th[0]
    log.check('3.5 an untimed wait does not use D and its match clears STALE',
              t.R[6] == 0 and t.T == 0 and t.STALE == 0, 'LATE bits %d %d' % (t.R[6], t.T))


def chk_wait_exits(log):
    # 3.3: what each way out of a wait leaves in S and D, by value. The simulators could share a
    # mistake here, so nothing below compares them with each other.
    other = 0x40                        # ui6: no wait below watches it

    def timed(rt):
        hi = [10 ** 9]
        for _ in range(2):              # the first run finds the tick's cycle, the second raises ui6 after it
            r = go("""
                now
                tick  20
                rdd   R4
                rds   R5
                ldi   R1, %d
                until ui5, 1, R1
                rds   R2
                rdd   R3
                halt
            """ % rt, devices=[Stim(lambda c: other if c >= hi[0] else 0)])
            r.board.run(300)
            hi[0] = r.chip.th[0].R[4] + 2
        return r.chip.th[0]

    t = timed(40)
    log.check('3.3 a timeout snapshots S', t.R[5] & 255 == 0 and t.R[2] & 255 == other, str((hex(t.R[5]), hex(t.R[2]))))
    log.check('3.3 a timeout leaves the D arming set, old D + Rt', t.R[3] == t.R[4] + 40, str((t.R[3], t.R[4])))
    t = timed(0)
    log.check('3.3 a timeout past at arming snapshots S', t.R[5] & 255 == 0 and t.R[2] & 255 == other, str((hex(t.R[5]), hex(t.R[2]))))
    log.check('3.3 a timeout past at arming leaves D behind, where arming set it', t.R[3] == t.R[4], str((t.R[3], t.R[4])))

    r = go("""
        now
        tick  20
        rds   R5
        until ui5, 1, inbox
        rds   R2
        rdd   R3
        halt
    """, devices=[Stim(lambda c: other if c >= 70 else 0)])
    r.board.run(80)
    e = r.chip.c                        # after step 5 of e - 1, seen in e
    r.chip.host_post(0, 1)
    r.board.run(40)
    t = r.chip.th[0]
    log.check('3.3 an inbox exit snapshots S', t.R[5] & 255 == 0 and t.R[2] & 255 == other, str((hex(t.R[5]), hex(t.R[2]))))
    log.check('3.3 an inbox exit sets D to that cycle', t.R[3] == e & 0xFFFF, str((t.R[3], e)))

    # 7.2: a timeout past at arming leaves D behind E, so the rule from E needs a now first
    def after(extra):
        r = go("""
            pcfg  uo1, PP
            now
            tick  20
            rdd   R4
            ldi   R1, 0
            until ui5, 1, R1
            %s
            edge  uo1, 1, 5
            tlate
            halt
        """ % extra)
        r.board.run(300)
        return r.chip.th[0].T
    log.check('7.2 after a timeout past at arming, a step of 5 is late, and on time after now', (after(''), after('now')) == (1, 0),
              str((after(''), after('now'))))


def chk_wait_priority(log):
    # 3.3: all three exits at once, then each pair, with T set both ways first
    def setup(match, inbox, first):
        r = go("""
            pcfg  uo1, PP
            pcfg  uo2, PP
            now
            li    R1, 200
            put   uo1, 1
            %s
            until ui5, 1, R1, inbox
            rds   R2
            tryrecv R3, 0
            bs    R6, 0
            halt
        """ % first, devices=[Stim(lambda c: 0x20 if match and c >= X[0] - 2 else 0)])
        return r

    out = {}
    for first in ('sett', 'clrt'):
        for match in (0, 1):
            for inbox in (0, 1):
                X = [10 ** 9]
                r = setup(match, inbox, first)
                t = r.chip.th[0]
                b = r.board
                while not (t.WAITING and t.P) and r.chip.c < LIMIT:
                    b.run(1)
                X[0] = r.chip.c - (r.chip.c & 0xFFFF) + t.D     # the deadline cycle
                while r.chip.c < X[0]:
                    if inbox and r.chip.c == X[0] - 1 + 0:
                        pass
                    b.run(1)
                    if inbox and r.chip.c == X[0]:      # after step 5 of X-1, seen in X
                        r.chip.host_post(0, 0x1234)
                b.run(1)
                d_at_x = t.D
                res_t = t.T
                # the pin is decided at X and reaches the pad at X + 1
                b.run(30)
                out[(first, match, inbox)] = (res_t, r.chip.OUT[1], r.chip.FULL[0] if not t.RUN else None,
                                       (d_at_x - (X[0] & 0xFFFF)) & 0xFFFF)
    a = out
    both = ('sett', 'clrt')
    # at the deadline cycle a timeout leaves D = X too, so D can't tell the three exits apart here, and an inbox
    # exit and a timeout read the same in T, the pin and D. Only T and the pin tell a match from the others
    log.check('3.3 at the deadline cycle a match beats inbox and timeout: T 1, pins commit',
              all(a[(f, 1, i)][:2] == (1, 1) for f in both for i in (0, 1)), str(a))
    log.check('3.3 at the deadline cycle, no match: T 0, pins stay, D that cycle, with or without the inbox',
              all(a[(f, 0, i)][:2] == (0, 0) and a[(f, 0, i)][3] == 0 for f in both for i in (0, 1)), str(a))

    def past(first, match, inbox):
        """A timeout already past at arming also ends the wait at a + 1, but it leaves D behind, so here D tells
        all three exits apart: a match or an inbox exit writes D := E, and a timeout leaves D where it was."""
        r = go("""
            pcfg  uo1, PP
            now
            put   uo1, 1
            ldi   R1, 0
            idle
            %s
            rdd   R4
            until ui5, 1, R1, inbox
            rdd   R2
            halt
        """ % first, devices=[Stim(lambda c: 0x20 if match else 0)])
        if inbox:
            r.chip.host_post(0, 0x1234)
        r.chip.log = []
        r.board.run(100)
        t = r.chip.th[0]
        E = next(c for c, tid, pc, w in r.chip.log if pc == 7) + 1
        return t.T, r.chip.OUT[1], (t.R[2] - E) & 0xFFFF, (t.R[2] - t.R[4]) & 0xFFFF, r.chip.FULL[0]
    a = {(f, m, i): past(f, m, i) for f in both for m in (0, 1) for i in (0, 1)}
    log.check('3.3 match beats inbox and a timeout past at arming: T 1, pins commit, D the match cycle',
              all(a[(f, 1, i)][:3] == (1, 1, 0) for f in both for i in (0, 1)), str(a))
    log.check('3.3 inbox beats a timeout past at arming: T 0, pins stay, D that cycle, the word stays full',
              all(a[(f, 0, 1)][:3] == (0, 0, 0) and a[(f, 0, 1)][4] == 1 for f in both), str(a))
    log.check('3.3 a timeout past at arming alone: T 0, pins stay, D where it was', all(a[(f, 0, 0)][:2] == (0, 0) and a[(f, 0, 0)][3] == 0 for f in both), str(a))

    # 3.3 "leave the prepared pins uncommitted": they stay prepared
    r = go("""
        pcfg  uo1, PP
        now
        ldi   R1, 40
        put   uo1, 1
        until ui5, 1, R1    ; times out
        tick  60            ; an unrelated event
        halt
    """)
    r.board.run(300)
    log.check('3.3 a pin prepared before a timed-out wait lands at the next event, unasked',
              r.chip.OUT[1] == 1, 'OUT=%d; the text does not warn about this' % r.chip.OUT[1])

    # the timed wait moves D at arming: 3.3 says "D afterwards: Unchanged"
    r = go('now\nrdd R3\nldi R1, 40\nidle\nuntil ui5, 1, R1\nrdd R4\nhalt\n')
    r.board.run(200)
    t = r.chip.th[0]
    log.line('INFO  timed wait, timeout: D before %d, after %d (moved by Rt at arming, not by the firing)'
             % (t.R[3], t.R[4]))
    log.check('3.3 table: timeout leaves D at old D + Rt', t.R[4] == t.R[3] + 40)


def chk_untilm(log):
    r = go('li R1, 0x3010\nuntilm R1, ui, eq\nrdtb R2\nhalt\n',
           devices=[Stim(lambda c: 0x10 if c > 200 else (0x30 if c > 100 else 0))])
    r.board.run(400)
    log.check('5.6 untilm eq: mask in the high byte, pattern in the low byte',
              r.chip.th[0].RUN == 0 and r.chip.th[0].R[2] > 200)
    r = go('li R1, 0x0F05\nuntilm R1, ui, ne\nhalt\n', devices=[Stim(lambda c: 0x05 if c < 150 else 0x07)])
    r.board.run(100)
    a = r.chip.th[0].RUN
    r.board.run(100)
    log.check('5.6 untilm ne', a == 1 and r.chip.th[0].RUN == 0)
    # chg: reference is S as it was when the wait armed
    r = go('tick 50\nidle\nidle\nidle\nidle\nidle\nidle\nidle\nidle\nidle\nli R1, 0x4000\nuntilm R1, ui, chg\nrdtb R2\nhalt\n',
           devices=[Stim(lambda c: 0x40 if c > 100 else 0)])
    r.board.run(400)
    log.check('5.6 chg fires for a change that happened before the wait armed',
              r.chip.th[0].RUN == 0, 'finished at %d' % r.chip.th[0].R[2])
    # 10 item 4: empty mask
    r = go('li R1, 0x0000\nuntilm R1, ui, eq\nhalt\n')
    r.board.run(40)
    a = r.chip.th[0].RUN
    r = go('li R1, 0x0000\nuntilm R1, ui, ne\nhalt\n')
    r.board.run(200)
    log.check('10 untilm with an empty mask: eq at once, ne never', a == 0 and r.chip.th[0].RUN == 1)


def chk_memory(log):
    r = go("""
        ldi   R1, 100
        stp   R1, R1
        rdtb  R2
        ldi   R3, 100
        ldp   R3, R3
        ldi   R4, 100
        ld    R5, R4
        halt
    """)
    r.board.run(200)
    t = r.chip.th[0]
    log.check('4.2 stp R1, R1 stores the old value', r.chip.mem[100] == 100 and t.R[1] == 101)
    log.check('4.2 ldp R1, R1 leaves the loaded data', t.R[3] == 100)
    # slot counts through the thread's own log
    r = go('ldi R1, 100\nld R2, R1\nidle\nst R2, R1\nidle\nhalt\n', 2)
    r.chip.log = []
    r.board.run(100)
    cy = [c for c, tid, pc, w in r.chip.log]
    log.check('4.2 ld and st take 2 slots, stealing nothing', [b - a for a, b in zip(cy, cy[1:])] == [4, 8, 4, 8, 4],
              str([b - a for a, b in zip(cy, cy[1:])]))
    # store seen by the fetch of the next instruction
    w = assemble('ldi R3, 7\n').words[0]
    r = go('li R1, %d\nldi R2, 4\nst R1, R2\nldi R3, 1\nhalt\n' % w)
    r.board.run(100)
    log.check('4.2 a store is seen by the fetch of the next instruction', r.chip.th[0].R[3] == 7,
              'R3=%d' % r.chip.th[0].R[3])
    # wrap
    img = assemble('.org 127\ncall 5\n.org 5\nhalt\n', depth=128)
    r = Run()
    r.load(img, 0)
    r.chip.host_setpc(0, 127)
    r.start(1)
    r.board.run(40)
    log.check('6.3 call at the last word: the link wraps to 0', r.chip.th[0].R[7] == 0, str(r.chip.th[0].R[7]))
    r = go('li R1, 0x0264\nli R2, 0xBEEF\nst R2, R1\nli R1, 100\nld R3, R1\nhalt\n', depth=512)
    r.board.run(200)
    log.check('4.1 ld and st addresses wrap at DEPTH', r.chip.th[0].R[3] == 0xBEEF)


def chk_pins(log):
    r = Run()
    pads = r.chip.pads(0)
    log.check('8 after reset every uo pin is low, every uio undriven', pads == (0, 0, 0), str(pads))

    # ownership
    r = go('put uo1, 1\nnow\nhalt\n', 2)
    r.board.run(60)
    log.check('5.3 writes to a pin the thread does not own are dropped', r.chip.SHD[1] == 0 and r.chip.OUT[1] == 0)
    r = go('pcfg uo1, PP\nput uo1, 1\nhalt\n', 2)   # shadow set, no event
    r.board.run(60)
    a = r.chip.OUT[1]
    img = assemble('now\nhalt\n', base=20, depth=128)
    r.load(img, 1)
    r.chip.host_run(0, 2)
    r.board.run(60)
    log.check('5.3 a commit moves only pins the thread owns', a == 0 and r.chip.OUT[1] == 0)
    # shadow/commit: prepared pins move together
    r = go('pcfg uo1, PP\npcfg uo2, PP\npcfg io0, PP\nput uo1, 1\nput uo2, 1\nput io0, 1\nnow\nhalt\n')
    r.board.run(100)
    w = r.board.trace
    a, b, c = first_rise(r, 'uo', 1), first_rise(r, 'uo', 2), first_rise(r, 'iooe', 0)
    log.check('7.3 all of a thread\'s pins move in one cycle', a == b == c, str((a, b, c)))
    # OD
    r = go("""
        pcfg io0, OD
        put  io0, 1
        now
        put  io0, 0
        now
        oe   io0, 1
        now
        put  io0, 1
        now
        halt
    """, pullups=1)
    r.board.run(100)
    tr = r.board.trace
    log.check('5.3 OD: the pad never drives high, oe has no effect', tr.drive_high(0).v == [0])
    # pcfg OD takes effect at once and released, from a driven PP pin
    r = go('pcfg io0, PP\nput io0, 1\nnow\nidle\nidle\npcfg io0, OD\nidle\nhalt\n')
    r.board.run(60)
    log.check('5.3 pcfg OD starts released at once', r.chip.pads(0)[2] & 1 == 0, str(r.chip.pads(0)))
    # 5.3 table: IN and PP act "from the next commit". OD has no shadow (5.4).
    r = go('pcfg io0, PP\nput io0, 1\nnow\nidle\nidle\npcfg io0, OD\nidle\npcfg io0, PP\nidle\nhalt\n')
    r.board.run(70)
    tr = r.board.trace
    log.line('INFO  PP, then OD, then PP with no event after it: pad driven at the end = %d' % (r.chip.pads(0)[2] & 1))
    log.check('5.3 PP from OD: no drive before the next commit', r.chip.pads(0)[2] & 1 == 0,
              'pads %s' % str(r.chip.pads(0)))
    r = go('pcfg io0, PP\nput io0, 1\nnow\nidle\nidle\npcfg io0, OD\nidle\npcfg io0, PP\nnow\nidle\nidle\nhalt\n')
    r.board.run(90)
    log.check('5.3 PP from OD: drive starts at the next commit', r.chip.pads(0)[1:] == (1, 1), str(r.chip.pads(0)))
    r = go('pcfg io0, PP\nput io0, 1\nnow\nidle\nidle\npcfg io0, OD\nidle\npcfg io0, IN\nidle\nhalt\n')
    r.board.run(70)
    w = r.board.trace.drive_high(0).v
    log.check('5.3 PP, OD, IN: drive high ends at OD and never comes back', w == [0, 1, 0], 'drive-high levels %s' % w)
    r = go('pcfg io0, OD\nput io0, 0\nnow\nidle\nidle\npcfg io0, IN\nidle\nhalt\n')
    r.board.run(70)
    log.check('5.3 IN from OD holding the line low: released at once, no event needed',
              r.chip.pads(0)[2] & 1 == 0, str(r.chip.pads(0)))
    # oe and oe8 do nothing on a pin in OD: OE and SHOE stay 0, so leaving OD can't drive
    res = []
    for write in ('oe io0, 1', 'li R1, 0xFF\noe8 R1'):
        r = go('pcfg io0, OD\n%s\nnow\nidle\nidle\npcfg io0, IN\nidle\nhalt\n' % write)
        r.board.run(90)
        res.append((r.chip.OE[0], r.chip.SHOE[0], r.chip.pads(0)[2] & 1, max(r.board.trace.drive_high(0).v)))
    log.check('5.4 oe and oe8 have no effect in OD, even after leaving it', res == [(0, 0, 0, 0)] * 2, str(res))
    # oe8 still works on the thread's other pins
    r = go('pcfg io0, OD\npcfg io1, IN\nli R1, 0xFF\noe8 R1\nnow\nidle\nidle\nhalt\n')
    r.board.run(90)
    log.check('5.4 oe8 still reaches pins of this thread that are not in OD', r.chip.OE[1] == 1 and r.chip.OE[0] == 0)
    # the invariant, every cycle, on random images: a pin in OD has OE = SHOE = 0
    log.check('5.4 invariant: OD implies OE = SHOE = 0, 40 random images, every cycle', od_invariant(40) is None,
              str(od_invariant(1)))
    undo_old = Chip.pcfg_od

    def mutant(self, t, p):                 # the old pcfg_od, which leaves OE and SHOE alone
        self.OWN[p] = t.TID
        self.OD[p - 8] = 1
        self.OUT[p] = 1
        self.SHD[p] = 1
        self._dirty = True
    Chip.pcfg_od = mutant
    caught = od_invariant(40)
    Chip.pcfg_od = undo_old
    log.check('teeth: the old pcfg_od breaks that invariant', caught is not None, str(caught))
    undo_old = Chip.shadow_oe

    def mutant2(self, t, p, v):             # the old shadow_oe, which reaches SHOE on a pin in OD
        if self.OWN[p] == t.TID:
            self.SHOE[p - 8] = v
    Chip.shadow_oe = mutant2
    caught = od_invariant(40)
    Chip.shadow_oe = undo_old
    log.check('teeth: the old shadow_oe breaks it too', caught is not None, str(caught))

    # 11.3 worked example: nothing drives the line until the host posts the bit time
    r = go(src('uart_tx'))
    r.board.run(3000)
    log.line('INFO  11.3 example, no config posted, 3000 cycles: uo0 is %d, thread RUN %d, PC %d'
             % (r.chip.pads(0)[0] & 1, r.chip.th[0].RUN, r.chip.th[0].PC))

    # bypass
    r = go('pcfg uo4, BYP\nnow\npcfg uo4, PP\nput uo4, 0\nuntil ui5, 1\nhalt\n',
           devices=[Stim(lambda c: (0x10 if c % 16 > 7 else 0) | (0x20 if c > 200 else 0))])
    r.board.run(400)
    w4 = r.board.trace.wave('uo', 4)
    ui4 = r.board.trace.wave('ui', 4)
    follow = all(w4.at(c) == ui4.at(c) for c in range(40, 200))
    rise5 = first_rise(r, 'ui', 5)
    held = all(w4.at(c) == 0 for c in range(rise5 + 3, 400))
    last = max(t for t, v in w4.edges())
    log.check('5.8 bypass follows, and the wait\'s commit overrides it within 3 cycles',
              follow and held and last <= rise5 + 3, 'last change %d, input at %d' % (last, rise5))
    r = go('pcfg uo0, BYP\nnow\nhalt\n', devices=[Stim(lambda c: 0x08 if c % 8 > 3 else 0)])
    r.board.run(100)
    w0, ui3 = r.board.trace.wave('uo', 0), r.board.trace.wave('ui', 3)
    log.check('5.2 lane uo0 follows ui3', all(w0.at(c) == ui3.at(c) for c in range(50, 100)))


def pin_word(rng):
    """A word that works on the pins, so random images keep moving pins in and out of OD."""
    rows = [r for r in ROWS if r.mn in ('pcfg', 'oe', 'oe8', 'put', 'now')]
    while True:
        r = rng.choice(rows)
        f = {k: rng.getrandbits(len(b)) for k, b in r.bits.items()}
        if 'p' in f:
            f['p'] = 8 + rng.randrange(8)       # io pins only: that is where OD lives
        if all(legal_value(f[k], r.ftypes[k]) for k in f):
            return encode(r, f)


def od_invariant(n, cycles=4000):
    """First (seed, cycle, pin) where a pin in OD has OE or SHOE set, or None. Random words, random inputs,
    stopped threads restarted at random addresses."""
    for seed in range(n):
        rng = random.Random(seed)
        words = [pin_word(rng) if rng.random() < 0.6 else random_word(rng) for _ in range(128)]
        chip = Chip(128, seed)
        chip.mem = words
        for c in range(cycles):
            if c % 16 == 0:
                for t in chip.th:
                    if not t.RUN:
                        chip.host_setpc(t.TID, rng.randrange(128))
                chip.host_run(0, 15)
            chip.cycle(rng.getrandbits(8), rng.getrandbits(8))
            for i in range(8):
                if chip.OD[i] and (chip.OE[i] or chip.SHOE[i]):
                    return seed, c, i
    return None


def chk_spacing(log):
    # 7.2: spacing 5 clean on 16 alignments, 4 clean on 12
    def stream(step):
        return 'pcfg uo1, PP\n' + 'edge uo1, 1, 30\n' + ''.join(
            'edge uo1, %d, %d\n' % (i & 1, step) for i in range(40)) + 'tlate\nhalt\n'
    res = {}
    for step in (4, 5):
        clean = 0
        for th in range(4):
            for al in range(4):
                r = go(stream(step), th, al)
                r.board.run(800)
                clean += r.chip.th[th].T == 0
        res[step] = clean
    log.check('7.2 spacing 5 clean on 16 of 16 alignments', res[5] == 16, str(res[5]))
    log.check('7.2 spacing 4 clean on 12 of 16 alignments', res[4] == 12, str(res[4]))

    # the same stream started from now: phase is fixed, one cycle after the slot
    clean = 0
    spacing = set()
    for th in range(4):
        for al in range(4):
            text = 'pcfg uo1, PP\nnow\n' + ''.join(
                'edge uo1, %d, 4\n' % (i & 1) for i in range(40)) + 'tlate\nhalt\n'
            r = go(text, th, al)
            r.board.run(800)
            clean += r.chip.th[th].T == 0
            ed = [t for t, v in r.board.trace.wave('uo', 1).edges()]
            spacing |= {b - a for a, b in zip(ed, ed[1:])}
    log.check('7.2/7.4 spacing 4 after now is clean on 16 of 16, a 6.25 MHz burst', clean == 16 and spacing == {4},
              'clean on %d, edge spacing %s' % (clean, sorted(spacing)))

    # 7.2: which D phases fail at spacing 4. D is set by RUN, so it takes the start phase
    bad_phase, n_late = [], 0
    for th in range(4):
        for al in range(4):
            r = go(stream(4), th, al)
            d0 = None
            r.board.run(800)
            late = r.chip.th[th].T
            n_late += late
            h = al                      # RUN sets D := h, and h mod 4 = al (Run.start)
            first = (h + 30) & 3        # the stream starts with a step of 30
            bad_phase.append((late == 1) == (first == th))
    log.check('7.2 at spacing 4 a stream is late exactly when D mod 4 = TID', all(bad_phase) and n_late == 4,
              '%d of 16 late, rule right on %d of 16' % (n_late, sum(bad_phase)))

    # 7.4 "5 MHz square wave": a stream of edge has to end. A loop needs a branch slot.
    res = {}
    for step in (5, 6):
        clean = 0
        for th in range(4):
            for al in range(4):
                r = go('pcfg uo1, PP\nnow\nl: edge uo1, 1, %d\nedge uo1, 0, %d\ngoto l\n' % (step, step), th, al)
                r.board.run(2000)
                clean += r.chip.th[th].LATE == 0
        res[step] = clean
    log.check('7.4 sustained square wave after now: half period 6 clean on 16 of 16, 5 on 0 of 16, so 4.17 MHz',
              res[5] == 0 and res[6] == 16, 'edge, edge, goto loop: %s' % res)
    # the same loop with D moved off the phase now leaves
    res = {}
    for step in (6, 7, 8):
        clean = 0
        for th in range(4):
            for al in range(4):
                for skew in range(4):
                    r = go('pcfg uo1, PP\nnow\ntick %d\nl: edge uo1, 1, %d\nedge uo1, 0, %d\ngoto l\n' % (40 + skew, step, step),
                           th, al)
                    r.board.run(2000)
                    clean += r.chip.th[th].LATE == 0
        res[step] = clean
    log.line('INFO  sustained loop on all 4 phases of D, of 64 runs: %s' % res)
    log.check('7.4 sustained on any phase of D: half period 7 clean on 64 of 64, 6 is not', res[7] == 64 and res[6] < 64, str(res))

    # the UART TX loop on all 4 phases of D. The real program re-anchors with now, so this is its bare loop
    res = {}
    for step in (11, 12):
        clean = 0
        for th in range(4):
            for al in range(4):
                for skew in range(4):
                    r = go('pcfg uo1, PP\nnow\ntick %d\nli R1, 200\nl: shr8 R0, -, uo1\ntick %d\ndjnz R1, l\nhalt\n' % (40 + skew, step),
                           th, al)
                    r.board.run(3000)
                    clean += r.chip.th[th].LATE == 0
        res[step] = clean
    log.check('7.2 UART TX loop on any phase of D: bit 12 clean on 64 of 64, bit 11 on none', res == {11: 0, 12: 64}, str(res))

    # 3.5: a tick after a wait with no now is late, and hdlc_tx has its now for that reason
    with open(os.path.join(HERE, '..', 'programs', 'standard', 'hdlc_tx.s')) as fh:
        hdlc = fh.read()
    res = {}
    for name, text in (('with', hdlc), ('without', re.sub(r'\n\s+now\s+; the post can come[^\n]*', '', hdlc))):
        r = go(text, 0)
        r.board.run(40000)                  # 800 us before the host posts the bit time
        r.chip.host_post(7, 64)
        r.board.run(2000)
        res[name] = r.chip.th[0].LATE
    log.check('3.5 hdlc_tx: bit time posted late is clean with its now, late without it', res == {'with': 0, 'without': 1}, str(res))

    # SPI half period 8, the same four-instruction loop as half period 9
    t = src('spi')
    runs = good = 0
    for mode in range(4):
        for th in range(4):
            for al in range(4):
                runs += 1
                good += spi_case(t, mode, 8, th, al)[0]
    log.check('7.4 SPI half period 8 with D phased by now: clean on 64 of 64, 3.125 MHz', good == runs == 64,
              '%d of %d' % (good, runs))
    runs = good = 0
    for mode in range(4):
        for th in range(4):
            runs += 1
            good += spi_case(t, mode, 7, th, 0)[0]
    log.check('half period 7 fails', good < runs, '%d of %d clean' % (good, runs))

    # UART TX: 12 is the limit, 11 runs late
    tx = src('uart_tx')
    res = {}
    for bit in (11, 12):
        good = 0
        for th in range(4):
            for al in range(4):
                good += uart_tx_case(tx, 50e6 / bit, th, al)[0]
        res[bit] = good
    log.check('7.2/7.4 UART TX: 12 cycles per bit clean, 11 not', res[12] == 16 and res[11] == 0, str(res))


def chk_host(log):
    # 9.3: first instruction 4 to 7 cycles after h
    seen = set()
    for th in range(4):
        for al in range(4):
            img = assemble('rdtb R1\nhalt\n', depth=128)
            r = Run()
            r.load(img, th)
            r.start(1 << th, al)
            h = (r.chip.c - 1)
            r.board.run(40)
            seen.add(r.chip.th[th].R[1] - h)
    log.check('9.3 first instruction runs 4 to 7 cycles after h', seen == {4, 5, 6, 7}, str(sorted(seen)))

    # 9.3: shared D across threads
    img = assemble('pcfg uo1, PP\nput uo1, 1\ntick 40\nhalt\n', depth=128)
    img2 = assemble('pcfg uo2, PP\nput uo2, 1\ntick 40\nhalt\n', base=10, depth=128)
    r = Run()
    r.load(img, 0)
    r.load(img2, 3)
    r.start(0b1001, 1)
    r.board.run(100)
    a, b = first_rise(r, 'uo', 1), first_rise(r, 'uo', 2)
    log.check('9.3 threads started by one RUN place an edge on one cycle', a == b, str((a, b)))

    # 7.3: same cycle by wrd
    img = assemble('li R1, 200\nwrd R1\npcfg uo1, PP\nput uo1, 1\nedge -, 0, 0\nhalt\n', depth=128)
    img2 = assemble('li R1, 200\nwrd R1\npcfg uo2, PP\nput uo2, 1\nedge -, 0, 0\nhalt\n', base=10, depth=128)
    r = Run()
    r.load(img, 1)
    r.load(img2, 2)
    r.start(0b0110, 1)
    r.board.run(300)
    a, b = first_rise(r, 'uo', 1), first_rise(r, 'uo', 2)
    log.check('7.3 two threads hit one cycle by wrd, using a step of 0', a == b == 201, str((a, b)))

    # 9.3 both masks, start on a running thread
    r = go('l: goto l\n', 1)
    r.board.run(20)
    d0 = r.chip.th[1].D
    r.chip.host_run(0, 2)
    log.check('9.3 a start bit on a running thread does nothing', r.chip.th[1].D == d0)
    r.chip.host_run(2, 2)
    r.board.run(20)
    log.check('9.3 a thread named in both masks is only stopped', r.chip.th[1].RUN == 0)

    # 9.3 stop: pending event cancelled, pins hold; a stop in a wait leaves PC on the next instruction
    r = go('pcfg uo1, PP\nput uo1, 1\ntick 400\nhalt\n')
    r.board.run(100)
    r.chip.host_run(1, 0)
    r.board.run(600)
    log.check('9.3 a stop cancels the pending event', r.chip.OUT[1] == 0 and r.chip.th[0].RUN == 0)
    r = go('until ui5, 1\nhalt\n')
    r.board.run(60)
    pc = r.chip.th[0].PC
    r.chip.host_run(1, 0)
    r.board.run(20)
    log.check('9.3 a stop abandons a wait with PC on the next instruction', r.chip.th[0].PC == pc == 1 and r.chip.th[0].P == 0)
    log.check('9.2 RDREG shows WAITING on a thread stopped in a wait', r.chip.host_rdreg(0, 11) & 8 == 8,
              hex(r.chip.host_rdreg(0, 11)))
    r.chip.host_run(0, 1)
    r.board.run(20)
    t = r.chip.th[0]
    log.check('9.3 restarting runs that instruction and does not re-arm the wait', t.WAITING == 0 and t.RUN == 0 and t.PC == 2)
    r = go('until ui5, 1\nhalt\n')
    r.board.run(60)
    r.chip.host_run(1, 0)
    r.board.run(20)
    r.chip.host_setpc(0, 0)
    r.chip.host_run(0, 1)
    r.board.run(20)
    log.check('9.3 SETPC repeats the wait', r.chip.th[0].WAITING == 1 and r.chip.th[0].P == 1 and r.chip.th[0].PC == 1)

    # 9.3: the event fired, then the stop landed before the slot: the wait ended, and RDREG says so
    hi = [10 ** 9]
    r = go('until ui5, 1\nidle\nhalt\n', devices=[Stim(lambda c: 0x20 if c >= hi[0] else 0)])
    r.board.run(60)
    while (r.chip.c + 2) % 4 != 1:      # E mod 4 = 1, so thread 0's slot is 3 cycles after it
        r.board.run(1)
    hi[0] = r.chip.c
    e = hi[0] + 2
    while r.chip.c < e + 1:
        r.board.run(1)
    r.chip.host_run(1, 0)
    r.board.run(20)
    t = r.chip.th[0]
    log.check('9.3 a wait whose event fired before the stop shows WAITING 0, with the same PC', r.chip.host_rdreg(0, 11) & 9 == 1 and t.PC == 1,
              str((hex(r.chip.host_rdreg(0, 11)), t.PC)))

    # 9.3: STEP clears WAITING and runs the next instruction
    r = go('until ui5, 1\nldi R2, 6\nhalt\n')
    r.board.run(60)
    r.chip.host_run(1, 0)
    r.board.run(20)
    r.chip.host_step(0)
    r.board.run(16)
    t = r.chip.th[0]
    log.check('9.3 STEP on a thread stopped in a wait runs the next instruction', (t.R[2], t.WAITING, t.RUN, t.PC) == (6, 0, 0, 2),
              str((t.R[2], t.WAITING, t.RUN, t.PC)))

    # 9.3: ABORT on a thread blocked in a wait. The running case can't show ABORT's own line, because ABORT
    # clears P and stop() then clears WAITING anyway. chk_stop_rules tests a stopped thread for that
    r = go('until ui5, 1\nhalt\n')
    r.board.run(60)
    r.chip.host_abort()
    r.board.run(20)
    log.check('9.3 ABORT stops a thread blocked in a wait, and WAITING reads 0', r.chip.host_rdreg(0, 11) & 8 == 0 and r.chip.th[0].RUN == 0, hex(r.chip.host_rdreg(0, 11)))

    # 3.6, 9.3: a stop during a stepped ld must not use up the next STEP
    for act in ('stop', 'abort'):
        r = Run()
        r.load(assemble('ldi R3, 100\nld R2, R3\nldi R4, 7\nhalt\n.org 100\n.data 0x1234\n', depth=128), 0)
        r.board.run(4)
        r.chip.host_step(0)
        r.board.run(12)
        r.chip.host_step(0)
        t = r.chip.th[0]
        while not t.MEM and r.chip.c < LIMIT:
            r.board.run(1)
        r.chip.host_run(1, 0) if act == 'stop' else r.chip.host_abort()
        r.board.run(16)
        a = (t.RUN, t.STOPREQ, t.R[2])
        r.chip.host_step(0)
        r.board.run(16)
        log.check('9.3 %s during a stepped ld finishes it, and the next STEP runs one instruction' % act,
                  a == (0, 0, 0x1234) and t.R[4] == 7 and t.RUN == 0, str((a, t.R[4])))

    # 9.3: an event a STEP left pending fires on a stopped thread, and a stop cancels it
    def stepped(stop):
        r = Run()
        r.load(assemble('pcfg uo1, PP\nldi R1, 200\nnow\nidle\nput uo1, 1\nedge uo1, 1, R1\nhalt\n', depth=128), 0)
        r.board.run(4)
        t = r.chip.th[0]
        for _ in range(5):
            r.chip.host_step(0)
            r.board.run(12)
        r.chip.host_step(0)
        r.board.run(8)
        a = (t.RUN, t.P)
        if stop:
            r.chip.host_run(1, 0)
        r.board.run(300)
        return a, t.P, r.chip.OUT[1]
    log.check('9.3 an event a STEP left pending fires on the stopped thread', stepped(0) == ((0, 1), 0, 1), str(stepped(0)))
    log.check('9.3 a stop cancels an event a STEP left pending', stepped(1) == ((0, 1), 0, 0), str(stepped(1)))
    # halt after the pending event
    r = go('pcfg uo1, PP\nput uo1, 1\ntick 100\nhalt\nidle\n')
    r.board.run(40)
    a = r.chip.th[0].RUN
    r.board.run(100)
    log.check('9.3 halt waits for the pending event and leaves PC on the next word',
              a == 1 and r.chip.OUT[1] == 1 and r.chip.th[0].PC == 4)
    # STEP
    r = Run()
    r.load(assemble('ldi R1, 5\nldi R2, 6\nhalt\n', depth=128), 0)
    r.board.run(4)
    r.chip.host_step(0)
    r.board.run(16)
    a = (r.chip.th[0].R[1], r.chip.th[0].R[2], r.chip.th[0].RUN)
    r.chip.host_step(0)
    r.board.run(16)
    log.check('9.3 STEP runs one instruction', a == (5, 0, 0) and r.chip.th[0].R[2] == 6, str(a))
    # BUSY
    r = go('l: goto l\n', 1)
    r.board.run(10)
    r.chip.host_setpc(1, 5)
    s = r.chip.host_status()
    log.check('9 BUSY sets on SETPC of a running thread, STATUS clears it',
              s[3] == 1 and r.chip.host_status()[3] == 0)
    # ABORT
    r = go('pcfg io0, PP\npcfg io1, OD\nput io0, 1\nput io1, 0\nnow\nl: goto l\n', devices=[], pullups=0b10)
    r.board.run(60)
    a = r.chip.pads(0)
    r.chip.host_abort()
    r.board.run(10)
    b = r.chip.pads(0)
    log.check('9.3 ABORT leaves every uio undriven', a[2] == 0b11 and b[2] == 0 and r.chip.th[0].RUN == 0,
              str((a, b)))
    # illegal word. 0xDF80 starts 1101 1111 1, which 6.3 keeps free
    img = assemble('pcfg uo1, PP\nput uo1, 1\ntick 100\n.data %s\nhalt\n' % ILLEGAL_WORD, depth=128)
    r = Run()
    r.load(img, 0)
    r.start(1)
    r.board.run(300)
    t = r.chip.th[0]
    log.check('10 illegal word: ILL, stopped, PC on it, pending event cancelled',
              t.ILL == 1 and t.RUN == 0 and t.PC == 3 and r.chip.OUT[1] == 0, 'PC=%d OUT=%d' % (t.PC, r.chip.OUT[1]))
    r.chip.host_run(0, 1)
    log.check('2 RUN clears ILL', r.chip.th[0].ILL == 0)

    # world order: a mailbox message can't overtake a pin edge
    r = go('pcfg uo1, PP\nput uo1, 1\ntick 100\nsend R0, 1\nhalt\n')
    while not r.chip.FULL[1] and r.chip.c < 400:
        r.board.run(1)
    log.check('3.6 a mailbox message can not overtake a pin edge',
              r.chip.OUT[1] == 1 and r.chip.c >= first_rise(r, 'uo', 1), 'cycle %d' % r.chip.c)


def chk_stop_rules(log):
    # 9.3: every value below comes from the rule text. Thread 0, so its slots are the multiples of 4
    next_pc = 6                         # li R2 is 2 words, ldi, now, the T line, the wait
    forms = (('until', 'until ui5, 1', 0), ('timed until', 'until ui5, 1, R1', 0), ('inbox until', 'until ui5, 1, inbox', 0),
             ('timed inbox until', 'until ui5, 1, R1, inbox', 0), ('untilm eq', 'untilm R2, ui, eq', 0x2020),
             ('timed untilm eq', 'untilm R2, ui, eq, R1', 0x2020), ('inbox untilm eq', 'untilm R2, ui, eq, inbox', 0x2020),
             ('timed inbox untilm eq', 'untilm R2, ui, eq, R1, inbox', 0x2020), ('untilm ne', 'untilm R2, ui, ne', 0x2000),
             ('timed untilm ne', 'untilm R2, ui, ne, R1', 0x2000), ('untilm chg', 'untilm R2, ui, chg', 0x2000),
             ('timed untilm chg', 'untilm R2, ui, chg, R1', 0x2000))

    def blocked(wait='until ui5, 1', line='sett', mask=0):
        """Stopped in a wait, with the timeout of a timed form 200 cycles off."""
        r = go('li R2, 0x%x\nldi R1, 200\nnow\n%s\n%s\nidle\nhalt\n' % (mask or 0x2020, line, wait))
        r.board.run(60)
        r.chip.host_run(1, 0)
        r.board.run(8)
        return r

    for name, wait, mask in forms:
        res = {}
        for line, told in (('sett', 1), ('clrt', 0)):
            r = blocked(wait, line, mask)
            t = r.chip.th[0]
            res[told] = (t.RUN, t.P, t.WAITING, t.PC, t.T, r.chip.host_rdreg(0, 11))
        log.check('9.3 a stop before the event: %s keeps WAITING, PC on the next word, T kept' % name,
                  res == {1: (0, 0, 1, next_pc, 1, 9), 0: (0, 0, 1, next_pc, 0, 8)}, str(res))

    # 3.2, 3.6: a stop acting in the event's own cycle sees the wait as finished. One cycle earlier it abandons it
    other = 0x40

    def own_cycle(way, late):
        """The stop acts in s, a multiple of 4, and the event would fire in s + late. T starts opposite the
        result. Rt: now runs on a multiple of 4 and sets D one on, so 199 + late puts the deadline in s + late."""
        wait = {'match': 'until ui5, 1', 'inbox': 'until ui5, 1, inbox', 'timeout': 'until ui5, 1, R1'}[way]
        s = [10 ** 9]
        r = go('pcfg uo1, PP\nldi R1, %d\nnow\nput uo1, 1\n%s\n%s\nidle\nhalt\n' % (199 + late, 'clrt' if way == 'match' else 'sett', wait),
               devices=[Stim(lambda c: (other if c >= 40 else 0) | (0x20 if way == 'match' and c >= s[0] + late - 2 else 0))])
        t = r.chip.th[0]
        b = r.board
        while not (t.WAITING and t.P) and r.chip.c < LIMIT:
            b.run(1)
        d0 = t.D
        s[0] = d0 - late if way == 'timeout' else (r.chip.c + 40) & ~3
        while r.chip.c < s[0]:
            b.run(1)
        if way == 'inbox' and not late:
            r.chip.host_post(0, 1)      # seen in s
        r.chip.host_run(1, 0)           # lands in s - 1, so the thread acts on it in s
        b.run(1)
        if way == 'inbox' and late:
            r.chip.host_post(0, 1)      # seen in s + 1, after the stop has acted
        b.run(8)
        return (t.WAITING, r.chip.host_rdreg(0, 11), t.T, r.chip.OUT[1], t.S & 255, t.D), s[0], d0

    for way, told, result, commits, seen in (('match', 0, 1, 1, other | 0x20), ('inbox', 1, 0, 0, other), ('timeout', 1, 0, 0, other)):
        got, s, d0 = own_cycle(way, 0)
        log.check('3.2/9.3 a stop in the cycle of a %s: the wait counts as ended, WAITING 0, T the result' % way,
                  got == (0, result, result, commits, seen, s), str((got, s)))
        got, s, d0 = own_cycle(way, 1)
        log.check('3.2/9.3 a stop one cycle before a %s: the wait is abandoned, WAITING 1, nothing fires' % way,
                  got == (1, 8 | told, told, 0, 0, d0), str((got, d0)))

    # 9.3: on a thread already stopped, so stop() can't do the work
    r = blocked()
    was = r.chip.host_rdreg(0, 11)
    r.chip.host_abort()
    log.check('9.3 ABORT clears WAITING on a stopped thread', (was, r.chip.host_rdreg(0, 11)) == (9, 1), str((was, r.chip.host_rdreg(0, 11))))

    r = blocked()
    d = r.chip.th[0].D
    r.chip.host_run(1, 1)
    r.board.run(16)
    t = r.chip.th[0]
    log.check('9.3 a thread named in both masks is only stopped: on a stopped thread nothing starts', (t.RUN, t.WAITING, t.D, t.PC) == (0, 1, d, next_pc),
              str((t.RUN, t.WAITING, t.D, t.PC)))

    r = blocked()
    r.chip.host_setpc(0, 3)
    log.check('9.3 SETPC does not touch WAITING', r.chip.th[0].PC == 3 and r.chip.host_rdreg(0, 11) == 9, hex(r.chip.host_rdreg(0, 11)))

    def left():
        """A STEP over edge leaves its event pending on a stopped thread."""
        r = Run()
        r.load(assemble('pcfg uo1, PP\nldi R1, 200\nldi R3, 100\nnow\nidle\nput uo1, 1\nedge uo1, 1, R1\nld R2, R3\nldi R4, 7\nhalt\n.org 100\n.data 0x1234\n', depth=128), 0)
        r.board.run(4)
        for _ in range(7):
            r.chip.host_step(0)
            r.board.run(12)
        return r, r.chip.th[0]

    r, t = left()
    a = (t.RUN, t.P)
    r.chip.host_abort()
    r.board.run(300)
    log.check('9.3 ABORT cancels an event a STEP left', a == (0, 1) and (t.P, r.chip.OUT[1]) == (0, 0), str((a, t.P, r.chip.OUT[1])))
    r, t = left()
    r.chip.host_run(0, 1)
    h = r.chip.c - 1
    a = (t.P, t.D)
    r.board.run(300)
    log.check('9.3 a start cancels an event a STEP left, and D is h', a == (0, h) and r.chip.OUT[1] == 0, str((a, h, r.chip.OUT[1])))
    r, t = left()
    r.chip.host_run(1, 1)
    a = (t.RUN, t.P)
    r.board.run(300)
    log.check('9.3 both masks on a stopped thread with an event pending: only stopped, the event cancelled', a == (0, 0) and r.chip.OUT[1] == 0,
              str((a, r.chip.OUT[1])))

    # 3.6, 9.3: a stop between the slots of a stepped ld. retire() ends the STEP, and the stop still cancels the event
    for act in ('stop', 'abort', None):
        r, t = left()
        r.chip.host_step(0)
        while not t.MEM and r.chip.c < LIMIT:
            r.board.run(1)
        if act == 'stop':
            r.chip.host_run(1, 0)
        elif act == 'abort':
            r.chip.host_abort()
        r.board.run(300)
        got = (t.RUN, t.STOPREQ, t.P, t.R[2], t.PC, r.chip.OUT[1])
        log.check('9.3 a stepped ld with an event left pending, %s between its slots' % (act or 'nothing'),
                  got == (0, 0, 0, 0x1234, 8, 0 if act else 1), str(got))

    # 9.3: a stop, a start or ABORT cancels an event a STEP left, and SETPC is none of those. left() ends in cycle 96: now
    # ran in 52, so D was 53, and edge added 200 in 88. So the pin is written in 253, and the pad shows it in 254
    r, t = left()
    a = (t.RUN, t.P, t.D, t.PC)
    r.chip.host_setpc(0, 0)
    b = (t.P, t.PC)
    r.board.run(300)
    got = (t.P, t.PC, t.RUN, r.chip.OUT[1], first_rise(r, 'uo', 1))
    log.check('9.3 SETPC leaves an event a STEP left: it fires at D and the pin moves', a == (0, 1, 253, 7) and b == (1, 0) and got == (0, 0, 0, 1, 254),
              str((a, b, got)))

    # 9.3: a running thread stopped between the slots of ld or st finishes it, stops, and the stop isn't lost
    for op, load in (('ld R2, R3', 1), ('st R5, R3', 0)):
        r = go('ldi R3, 100\nli R5, 0xBEEF\n%s\nldi R4, 7\nhalt\n.org 100\n.data 0x1234\n' % op)
        t = r.chip.th[0]
        while not t.MEM and r.chip.c < LIMIT:
            r.board.run(1)
        r.chip.host_run(1, 0)
        r.board.run(16)
        got = (t.RUN, t.STOPREQ, t.PC, t.R[4], t.R[2] if load else r.chip.mem[100])
        log.check('9.3 a stop on a running thread between the slots of %s: it finishes and the thread stops' % op[:2],
                  got == (0, 0, 4, 0, 0x1234 if load else 0xBEEF), str(got))
        r.chip.host_run(0, 1)
        r.board.run(20)
        log.check('9.3 and a restart runs the next word', t.R[4] == 7 and t.RUN == 0, str((t.R[4], t.RUN)))

    # 9.3: a cancelled event's pins stay prepared, and the thread's next event commits them
    r = go('pcfg uo1, PP\nnow\nput uo1, 1\ntick 200\nnow\nhalt\n')
    r.board.run(40)
    r.chip.host_run(1, 0)
    r.board.run(300)
    a = (r.chip.OUT[1], r.chip.SHD[1], r.chip.th[0].P)
    r.chip.host_run(0, 1)
    r.board.run(20)
    log.check('9.3 a cancelled event leaves its pin prepared, and the next event commits it', a == (0, 1, 0) and r.chip.OUT[1] == 1,
              str((a, r.chip.OUT[1])))

    # 9.2: only the low 2 bits of thread8 count
    r = go('ldi R2, 6\nl: goto l\n', 2)
    r.board.run(40)
    r.chip.host_run(4, 0)
    r.board.run(8)
    got = [r.chip.host_rdreg(n, 2) for n in (2, 6, 10, 254)]
    r.chip.host_setpc(6, 0)
    a = r.chip.th[2].PC
    r.chip.host_step(10)
    r.board.run(12)
    log.check('9.2 RDREG, SETPC and STEP use the low 2 bits of thread8', got == [6] * 4 and a == 0 and r.chip.th[2].PC == 1, str((got, a, r.chip.th[2].PC)))

    # 7.2: the simple rule holds while the previous event was on time. tick 5 is late here, so D stays behind
    r = go('pcfg uo1, PP\nnow\nidle\nidle\nidle\nidle\nidle\nidle\ntick 5\ntlate\nbs R6, 0\nedge uo1, 1, 13\ntlate\nhalt\n')
    r.board.run(200)
    t = r.chip.th[0]
    log.check('7.2 after a late event, a step of 4k + 1 can be late', t.R[6] == 1 and t.T == 1, str((t.R[6], t.T)))


def chk_flags(log):
    # 2, 9.3: who writes LATE and ILL. Thread 0, RUN lands in 8, so the first word runs in 12. tick 0 in 24 finds D = 13 behind
    # it, so its event in 25 sets LATE. The word at 4 then sets ILL in 28 and stops the thread with PC on it. STATUS byte 0 is
    # RUN << 4 | ILL and byte 1 is LATE << 4 | STALE, so that reads 0x01, 0x10. Every value below is from the blocks
    idle = assemble('idle\n', depth=128).words[0]
    wait = assemble('until ui5, 1\n', depth=128).words[0]

    def flagged():
        r = go('now\nidle\nidle\ntick 0\n.data %s\nidle\ntlate\nhalt\n' % ILLEGAL_WORD)
        r.board.run(40)
        return r, r.chip.th[0]

    def looping():
        """The same late tick, on a thread that keeps running."""
        r = go('now\nidle\nidle\ntick 0\nl: goto l\n')
        r.board.run(40)
        return r, r.chip.th[0]

    def bytes01(r):
        s = r.chip.host_status()
        return s[0], s[1]

    r, t = flagged()
    log.check('2 a late tick and an illegal word: LATE and ILL set, stopped, PC on the word', (bytes01(r), t.PC, t.D) == ((0x01, 0x10), 4, 13),
              str((bytes01(r), t.PC, t.D)))

    r, t = flagged()
    r.chip.host_wrmem(4, idle)          # so no slot can set ILL again
    r.chip.host_run(0, 1)
    a = bytes01(r)
    r.board.run(40)
    got = (a, bytes01(r), t.T, t.PC)
    log.check('2 a RUN that starts the thread clears LATE and ILL at once, and a tlate after it reads 0', got == ((0x10, 0x00), (0x00, 0x00), 0, 8), str(got))

    for name, act in (('a stop bit', lambda c: c.host_run(1, 0)), ('a stop bit and a start bit together', lambda c: c.host_run(1, 1)),
                      ('SETPC', lambda c: c.host_setpc(0, 5)), ('ABORT', lambda c: c.host_abort())):
        r, t = flagged()
        act(r.chip)
        a = bytes01(r)
        r.board.run(16)
        got = (a, bytes01(r), t.RUN)
        log.check('9.3 %s on a stopped thread leaves LATE and ILL' % name, got == ((0x01, 0x10), (0x01, 0x10), 0), str(got))

    r, t = flagged()
    r.chip.host_wrmem(4, idle)
    r.chip.host_step(0)
    a = bytes01(r)
    r.board.run(16)
    b = (bytes01(r), t.PC)
    r.chip.host_step(0)
    r.board.run(16)
    r.chip.host_step(0)                 # the tlate at 6
    r.board.run(16)
    got = (a, b, bytes01(r), t.T, t.PC)
    log.check('9.3 STEP leaves LATE and ILL, and a stepped tlate reads LATE and clears only that', got == ((0x11, 0x10), ((0x01, 0x10), 5), (0x01, 0x00), 1, 7), str(got))

    r, t = looping()
    a = bytes01(r)
    r.chip.host_run(0, 1)
    b = bytes01(r)
    r.board.run(20)
    got = (a, b, bytes01(r), t.D)
    log.check('9.3 a start bit on a running thread leaves LATE', got == ((0x10, 0x10), (0x10, 0x10), (0x10, 0x10), 13), str(got))

    for name, act in (('a stop bit', lambda c: c.host_run(1, 0)), ('ABORT', lambda c: c.host_abort())):
        r, t = looping()
        act(r.chip)
        r.board.run(8)
        log.check('9.3 %s on a running thread leaves LATE' % name, bytes01(r) == (0x00, 0x10), str(bytes01(r)))

    # A start clears ILL, so RUN 1 with ILL 1 takes a STEP, and a STEP that blocks stays in that state
    for name, act, want in (('a start bit', lambda c: c.host_run(0, 1), (0x11, 0x10)), ('a stop bit', lambda c: c.host_run(1, 0), (0x01, 0x10)),
                            ('ABORT', lambda c: c.host_abort(), (0x01, 0x10))):
        r, t = flagged()
        r.chip.host_wrmem(4, wait)
        r.chip.host_step(0)
        r.board.run(16)
        a = bytes01(r)
        act(r.chip)
        r.board.run(8)
        got = (a, bytes01(r))
        log.check('9.3 %s on a STEP blocked in a wait leaves LATE and ILL' % name, got == ((0x11, 0x10), want), str(got))


def chk_stop_timing(log):
    # 9.3: a stop between the slots of ld or st. Thread 0, RUN lands in 8. Word 9 runs in 48, its memory cycle is 49, and slot 2 in
    # 52 tests MEM before STOPREQ, so it finishes the instruction, and stop() runs in 56. now ran in 32, so edge puts D at 33 + step
    prog = ('pcfg uo1, PP\nldi R3, 100\nli R5, 0xBEEF\nldi R1, %d\nnow\nidle\nput uo1, 1\nedge uo1, 1, R1\n%s\nldi R4, 7\nhalt\n'
            '.org 100\n.data 0x1234\n')

    def split(op, step, after):
        """The stop lands in 48 + after. Returns the cycle that wrote RUN := 0, the one that wrote OUT[1] := 1, and the end state."""
        r = go(prog % (step, op))
        t = r.chip.th[0]
        while not t.MEM and r.chip.c < LIMIT:
            r.board.run(1)
        r.board.run(after)
        r.chip.host_run(1, 0)
        fell = rose = None
        while r.chip.c < 68:
            r.board.run(1)
            if fell is None and not t.RUN:
                fell = r.chip.c - 1
            if rose is None and r.chip.OUT[1]:
                rose = r.chip.c - 1
        return fell, rose, t.P, t.LATE, t.PC, t.R[4], t.R[2] if op[:2] == 'ld' else r.chip.mem[100]

    for op, value in (('ld R2, R3', 0x1234), ('st R5, R3', 0xBEEF)):
        got = {after: split(op, 30, after) for after in range(4)}
        log.check('9.3 a stop between the slots of %s: slot 2 in 52 finishes it, RUN falls in 56, the next word never runs' % op[:2],
                  all(g == (56, None, 0, 0, 10, 0, value) for g in got.values()), str(got))
        got = {step: {split(op, step, after)[:2] for after in range(4)} for step in range(19, 25)}
        log.check('9.3 %s: an edge due by the slot of the stop fires, in that cycle too, and one due after it is cancelled' % op[:2],
                  got == {19: {(56, 52)}, 20: {(56, 53)}, 21: {(56, 54)}, 22: {(56, 55)}, 23: {(56, 56)}, 24: {(56, None)}}, str(got))

    # 9.3: a stop that lands g cycles after the RUN that started the thread. k is the phase of h, (h + 1 - thread) mod 4, and each
    # row of these is g = 0..3. Fetch is in h + 2, h + 1, h + 4, h + 3 for k = 0..3. slot() tests STOPREQ before IV, so stop() runs at
    # the next execute cycle whether or not a word has arrived, and it comes before execute(), so the first word never runs
    gone = {0: (1, 4, 3, 2), 1: (4, 3, 2, 1), 2: (3, 2, 1, 4), 3: (2, 1, 4, 3)}
    held = {0: (0, 1, 1, 1), 1: (1, 1, 1, 1), 2: (0, 0, 0, 1), 3: (0, 0, 1, 1)}
    bad, empty = [], 0
    for th in range(4):
        for al in range(4):
            for g in range(4):
                r = Run()
                r.load(assemble('rdtb R1\nhalt\n', depth=128), th)
                r.start(1 << th, al)
                k = (r.chip.c - th) % 4         # c is h + 1 here
                r.board.run(g)
                r.chip.host_run(1 << th, 0)
                t = r.chip.th[th]
                r.board.run(gone[k][g] - 1)     # its execute cycle is next
                before = t.RUN, t.IV
                r.board.run(1)
                after = t.RUN
                r.board.run(12)
                empty += 1 - before[1]
                if (before, after, t.R[1], t.PC) != ((1, held[k][g]), 0, 0, 0):
                    bad.append((th, al, g, before, after, t.R[1], t.PC))
    log.check('9.3 a stop 0 to 3 cycles after the start: RUN falls in the next execute cycle, 1 to 4 cycles after it, and the first word never runs', not bad, str(bad[:4]))
    log.check('9.3 in 24 of those 64 runs nothing has been fetched at that cycle, and it makes no difference', empty == 24, str(empty))

    # The same on a thread well under way, and with no fetch in question: every thread on every phase of h
    seen = set()
    for th in range(4):
        for al in range(4):
            r = go('l: goto l\n', th)
            r.board.run(20 + al)
            r.chip.host_run(1 << th, 0)
            h = r.chip.c - 1
            t = r.chip.th[th]
            n = 0
            while t.RUN and n < 12:
                r.board.run(1)
                n += 1
            seen.add((th, h % 4, n))
    want = {(th, x, 1 + (th - x - 1) % 4) for th in range(4) for x in range(4)}
    log.check('9.3 a stop on a running thread: RUN falls in its next execute cycle, 1 to 4 cycles after h', seen == want, str(sorted(seen ^ want)[:4]))


def chk_slots(log):
    # section 6.3 Slots column, one sample per row where it can run with no outside help
    bad = []
    n = 0
    for row in ROWS:
        if row.mn == 'halt':
            continue
        fields = {}
        for k, ty in row.ftypes.items():
            fields[k] = {'reg': 1, 'reg4': 1, 'addr': 50, 'opin': 1, 'opin-': 1, 'ipin': 5, 'ipin-': 5,
                         'iopin': 9, 'lane': 4, 'wmode': 1, 'eact': 1, 'obank': 1, 'ibank': 0,
                         'flag': 0, 'u1': 0, 'u3': 2, 'u4': 1, 'u5': 20, 'u8': 3, 'u12': 20,
                         'nbits': 8, 'lflag': 0, 'tkind': 0, 'sslot': 2, 'sout': 1, 'sdir': 1, 'sreg': 0}[ty]
        w = encode(row, fields)
        r = Run(depth=128)
        r.chip.host_wrmem(0, w)
        for a in range(1, 128):
            r.chip.host_wrmem(a, 0)         # idle
        r.chip.th[0].R = [0, 0, 0, 0, 0, 0, 0, 0]
        if row.mn in ('recv', 'tryrecv'):
            r.chip.FULL[2] = 1
        if row.mn == 'rxw':
            r.chip.th[0].u.RXFULL = 1       # a received word waiting, as recv gets a posted one
        if row.syntax.startswith('untilm'):
            r.chip.th[0].R[1] = 0x0000      # ne with an empty mask never matches, so match
            fields_c = dict(fields, c=0)
            w = encode(row, fields_c)
            r.chip.host_wrmem(0, w)
        if row.syntax.startswith('until p'):
            fields_l = dict(fields, l=0)
            w = encode(row, fields_l)
            r.chip.host_wrmem(0, w)
        r.start(1, 3)
        r.chip.log = []
        t = r.chip.th[0]
        r.board.run(40)
        if len(r.chip.log) < 2:
            bad.append((row.syntax, 'ran %d' % len(r.chip.log)))
            continue
        gap = (r.chip.log[1][0] - r.chip.log[0][0]) // 4
        want = {'1': 1, '2': 2, '1+': 1}[row.slots]
        n += 1
        if gap != want:
            bad.append((row.syntax, gap, want))
    log.check('6.3 Slots column, one sample each', not bad, '%d rows, %s' % (n, bad[:6]))


def chk_asm(log):
    def rejected(text, depth=128):
        try:
            assemble(text, depth=depth)
            return False
        except AsmError:
            return True

    for text, why in (('put uo3, 1\n', 'put on uo3'), ('put ui1, 1\n', 'put on ui'),
                      ('get uo1\n', 'read uo'), ('shr8 R0, ui0, -\n', 'ui0 as shift input'),
                      ('shl8 R4, -, -\n', 'shift of R4'), ('goto 128\n', 'target past DEPTH'),
                      ('edge: idle\n', 'label is a mnemonic'), ('pcfg uo1, OD\n', 'OD on uo'),
                      ('pcfg uo1, BYP\n', 'BYP on a non-lane'), ('oe uo1, 1\n', 'oe on uo'),
                      ('edge uo1, 1, 32\n', 'step past u5'), ('tick 4096\n', 'step past u12')):
        log.check('6 assembler rejects %s' % why, rejected(text))
    a = assemble('tick R3\nret\nli R1, 0x34\nli R1, 0x1234\n', depth=128)
    w = [a.words[i] for i in sorted(a.words)]
    e = assemble('edge -, 0, R3\njr R7\nldi R1, 0x34\nldi R1, 0x34\nlui R1, 0x12\n', depth=128)
    log.check('6.4 pseudo-instructions', w == [e.words[i] for i in sorted(e.words)])

    # expressions and directives from 6.6
    src_text = '.equ A 3\n.equ B (A + 1) * 2\n.if B - 8\nldi R1, 1\n.else\nldi R1, 2 + 1\n.endif\n.data 0xBEEF\n'
    img = assemble(src_text, depth=128)
    log.check('6.6 .equ, .if, .else, .data', decode(img.words[0])[1]['i'] == 3 and img.words[1] == 0xBEEF)
    # 11.4 worked example, with the names it uses
    img = assemble('.equ A uo1\n.equ TRIG ui5\nloop:   put   A, 1\n        until TRIG, 1\n', depth=128)
    log.check('11.4 worked example assembles', img.size == 2)
    # 11.4: the example's lines are in the source that ran
    doc = open(os.path.join(HERE, '..', 'docs', 'ISA.md')).read()
    block = doc.split('### 11.4 Worked example')[1].split('```')[1].strip()
    with open(os.path.join(HERE, '..', 'programs', 'standard', 'reflex.s')) as fh:
        reflex = fh.read()
    log.check('11.4 the example lines are in programs/standard/reflex.s, byte for byte', block in reflex)
    # 11.3: the worked example is the source that ran
    doc = open(os.path.join(HERE, '..', 'docs', 'ISA.md')).read()
    block = doc.split('### 11.3 Worked example')[1].split('```')[1].strip()
    log.check('11.3 the example in the document is programs/basic/uart_tx.s, byte for byte', block == src('uart_tx').strip())
    fw = open(os.path.join(HERE, '..', 'programs', 'standard', 'uart_tx.s')).read()
    log.check('11.3 and programs/standard/uart_tx.s is the same source', block == fw.strip())
    # 11.3: the line idles high with no config posted
    r = go(src('uart_tx'))
    r.board.run(3000)
    log.check('11.3 TX idles high before the host posts the bit time', r.chip.pads(0)[0] & 1 == 1, str(r.chip.pads(0)))
    r = go(src('spi'))
    r.board.run(3000)
    log.check('SPI: CS idles high before the host posts the half period', (r.chip.pads(0)[0] >> 5) & 1 == 1, str(r.chip.pads(0)))

    # 6.6 names
    for name in ('eq', 'ne', 'chg', 'T', 'inbox', 'IN', 'PP', 'OD', 'BYP', 'uo', 'ui', 'io', 'R3', 'uo1', 'edge', 'li'):
        log.check('6.6 label %s rejected' % name, rejected('%s: idle\n' % name))
        log.check('6.6 .equ %s rejected' % name, rejected('.equ %s 5\n' % name))
    for text, why in (('a: idle\na: idle\n', 'a label defined twice'), ('.equ A 1\n.equ A 2\n', '.equ defined twice'),
                      ('.equ A 1\nA: idle\n', 'a label with a .equ name'),
                      ('.equ A B\n.equ B 1\n', '.equ of a later name'), ('.if L\n.endif\nL: idle\n', '.if of a later label'),
                      ('L: idle\n.equ A L\n', '.equ of an earlier label'), ('L: idle\n.if L\n.endif\n', '.if of an earlier label'),
                      ('.data 65536\n', '.data over 16 bits'), ('.data -1\n', 'a negative .data'),
                      ('ldi R1, -1\n', 'a negative operand'), ('idle\nidle\n.org 0\nidle\n', '.org over used words'),
                      ('edge uo1, 2, 5\n', 'a pin value of 2'), ('put uo1, 2\n', 'put of 2'), ('IDLE\n', 'a wrong case mnemonic')):
        log.check('6.6 assembler rejects %s' % why, rejected(text))
    # 6.2 constant expressions as pin values
    a = assemble('.equ V 1\nput uo1, V\nedge uo1, V ^ 1, 5\n', depth=128)
    b = assemble('put uo1, 1\nedge uo1, 0, 5\n', depth=128)
    log.check('6.2 a constant expression works as a pin value', a.words == b.words)
    # 6.6 precedence, nesting
    a = assemble('.equ A 1 + 2 * 3\n.equ B 1 << 2 + 1\n.equ C 6 & 3 | 8\n.if 1\n.if 0\nldi R1, 1\n.else\nldi R1, A\n.endif\n.endif\nldi R2, B\nldi R3, C\n',
                 depth=128)
    got = [decode(a.words[i])[1]['i'] for i in range(3)]
    log.check('6.6 Python precedence and nested .if', got == [7, 8, 10], str(got))
    # 6.4 li
    a = assemble('li R1, L\nL: idle\n', depth=128)
    b = assemble('L: idle\nli R1, L\n', depth=128)
    log.check('6.4 li on a later label takes 2 words, on an earlier one 1 if the value fits', a.size == 3 and b.size == 2,
              str((a.size, b.size)))
    # warnings
    a = assemble('put uo1, 1\npcfg uo1, PP\n', depth=128)
    b = assemble('pcfg uo1, PP\nput uo1, 1\n', depth=128)
    c = assemble('edge uo2, 1, 5\nshr8 R0, -, io3\n', depth=128)
    log.check('6.6 warns on a pin written before its pcfg, not after', len(a.warnings) == 1 and not b.warnings and len(c.warnings) == 2,
              str(a.warnings))

    # the same language in both assemblers: every source, same words, and the same rejections
    import asm2 as v
    import glob
    root = os.path.join(HERE, '..', 'programs')
    files = sorted(glob.glob(os.path.join(root, '*', '*.s')))
    diff = []
    for f in files:
        x = assemble(open(f).read(), depth=512).words
        y = v.assemble(f, size=512).words
        if x != y:
            diff.append(os.path.relpath(f, root))
    log.check('6.6 both assemblers give the same words for every source', not diff, '%d files, differ: %s' % (len(files), diff))
    bad = []
    for text in ('eq: idle\n', '.equ T 1\n', 'a: idle\na: idle\n', '.equ A 1\n.equ A 2\n', 'idle\n.org 0\nidle\n', '.data 65536\n'):
        fd, tmp = tempfile.mkstemp(suffix='.s')
        with os.fdopen(fd, 'w') as fh:
            fh.write(text)
        try:
            v.assemble(tmp, size=128)
            bad.append(text)
        except v.AsmError:
            pass
        os.remove(tmp)
    log.check('6.6 the second assembler rejects them too', not bad, str(bad))

    # 11.2: the Words column is the assembler's count, and the table lists every source
    body = doc.split('### 11.2 ')[1].split('\n### ')[0]
    listed, wrong = set(), []
    for line in body.splitlines():
        cells = [c.strip() for c in line.split('|')[1:-1]]
        if len(cells) != 3 or not cells[0].startswith('`'):
            continue
        for rel in re.findall(r'`([\w/]+\.s)`', cells[0]):
            path = os.path.normpath(os.path.join(root, rel))
            listed.add(path)
            with open(path) as fh:
                n = assemble(fh.read(), depth=512).size
            if n != int(cells[1]):
                wrong.append((rel, n, cells[1]))
    missing = {os.path.normpath(f) for f in files} - listed
    log.check('11.2 the Words column matches the assembler for every source, and no source is missing',
              not wrong and not missing, '%d listed, wrong %s, missing %s' % (len(listed), wrong, sorted(missing)))


# ---- every row against its Semantics text ----------------------------------

class Need(Exception):
    pass


def translate(expr):
    expr = re.sub(r'\bR([a-z])\b', r'R[\1]', expr)
    expr = re.sub(r'\bR(\d)\b', r'R[\1]', expr)
    return expr


def run_doc(chip, t, row, fld, a):
    state = {'hold': False, 'pc': False}

    def _need(c):
        if not c:
            raise Need()

    def _load(d, A):
        chip.load(t, d, A)
        state['hold'] = True

    def _store(V, A):
        chip.store(t, V, A)
        state['hold'] = True

    def _wait(mask, pat, bank, ne, rt, inbox):
        chip.wait(t, mask, pat, bank, ne, rt, inbox, a)
        state['hold'] = True

    def _halt():
        t.PC = chip.next(t)
        chip.stop(t)
        state['pc'] = True

    env = {
        'bit': lambda x, n: (x >> n) & 1, 'need': _need, 'load': _load, 'store': _store,
        'wait': _wait, 'halt': _halt, 'now': lambda: chip.now(t, a),
        'arm': lambda s: chip.arm(t, s, a),
        'shadow': lambda p, v: chip.shadow(t, p, v),
        'shadow_oe': lambda p, v: chip.shadow_oe(t, p, v),
        'put8': lambda b, v: chip.put8(t, b, v), 'oe8': lambda v: chip.oe8(t, v),
        'pcfg_in': lambda p: chip.pcfg_in(t, p), 'pcfg_pp': lambda p: chip.pcfg_pp(t, p),
        'pcfg_od': lambda p: chip.pcfg_od(t, p), 'pcfg_byp': lambda p: chip.pcfg_byp(t, p),
        'TB': a, 'TID': t.TID, 'DEPTH': chip.DEPTH, 'R': t.R, 'M': chip.M, 'FULL': chip.FULL,
        'tx_push': lambda v, n, last, kind: chip.tx_push(t, v, n, last, kind), 'rx_pop': lambda: chip.rx_pop(t),
        'sstat': lambda d: chip.sstat(t, d), 'srd': lambda d, s: chip.srd(t, d, s),
        'set_cfg': lambda k, v: chip.set_cfg(t, k, v), 'sgo': lambda m: chip.sgo(t, m),
        'sstop': lambda m: chip.sstop(t, m), 'shadow_sel': lambda p, l: chip.shadow_sel(t, p, l),
    }
    env.update(fld)

    def sync():
        env.update(T=t.T, S=t.S, D=t.D, LATE=t.LATE, PC=t.PC,
                   SHD=sum(v << i for i, v in enumerate(chip.SHD)),
                   TXFULL=t.u.TXFULL, RXFULL=t.u.RXFULL, RXBUF=t.u.RXBUF, RXBL=t.u.RXBL)

    pc0 = t.PC
    try:
        for stmt in row.sem.split(';'):
            stmt = stmt.strip()
            cond = None
            m = re.match(r'if (.+?): (.+)$', stmt)
            if m:
                cond, stmt = m.group(1), m.group(2)
            sync()
            if cond is not None and not eval(translate(cond), env):
                continue
            if stmt == 'pass':
                continue
            if ':=' in stmt:
                lhs, rhs = [x.strip() for x in stmt.split(':=')]
                v = eval(translate(rhs), env)
                lhs = translate(lhs)
                if lhs == 'T':
                    t.T = int(v) & 1
                elif lhs == 'D':
                    t.D = v & 0xFFFF
                    t.D_WRITTEN = 1
                elif lhs == 'LATE':
                    t.LATE = int(v)
                elif lhs == 'PC':
                    t.PC = v % chip.DEPTH
                    state['pc'] = True
                elif lhs == 'f':
                    env['f'] = v
                elif lhs.startswith('R['):
                    t.R[eval(lhs[2:-1], env)] = v & 0xFFFF
                elif lhs.startswith('M['):
                    chip.M[env['m']] = v & 0xFFFF
                elif lhs.startswith('FULL['):
                    chip.FULL[env['m']] = int(v)
                else:
                    raise ValueError(stmt)
            else:
                eval(translate(stmt), env)
    except Need:
        return HOLD
    if state['hold']:
        return HOLD
    if state['pc']:
        return PCSET
    assert t.PC == pc0
    return None


def flat(x):
    return {k: (list(v) if isinstance(v, list) else v) for k, v in vars(x).items()}


def snapshot(chip):
    th = []
    for t in chip.th:
        d = {k: v for k, v in flat(t).items() if k not in ('u', 'pre', 'u0')}
        d['u'] = flat(t.u)                  # every name of 5.9.2, the slots and their fields
        d['pre_is_u'] = t.pre is t.u        # outside a cycle a row reads the unit itself
        th.append(d)
    return dict(th=th, M=list(chip.M), FULL=list(chip.FULL), OUT=list(chip.OUT), SHD=list(chip.SHD),
                OE=list(chip.OE), SHOE=list(chip.SHOE), OD=list(chip.OD), SHBYP=list(chip.SHBYP),
                OWN=list(chip.OWN), BYP=list(chip.BYP), mem=list(chip.mem), SEL=list(chip.SEL),
                SHSEL=list(chip.SHSEL), SYNC3=chip.SYNC3)


def random_chip(rng, depth):
    chip = Chip(depth, seed=rng.getrandbits(32))
    chip.M = [rng.getrandbits(16) for _ in range(8)]
    chip.FULL = [rng.getrandbits(1) for _ in range(8)]
    chip.SHD = [rng.getrandbits(1) for _ in range(16)]
    chip.OUT = [rng.getrandbits(1) for _ in range(16)]
    chip.OE = [rng.getrandbits(1) for _ in range(8)]
    chip.SHOE = [rng.getrandbits(1) for _ in range(8)]
    chip.OD = [rng.getrandbits(1) for _ in range(8)]
    chip.OWN = [rng.randrange(4) for _ in range(16)]
    chip.SYNC2 = rng.getrandbits(16)
    for t in chip.th:
        t.R = [rng.choice((0, 1, 0xFFFF, 0x8000, rng.getrandbits(16))) for _ in range(8)]
        t.PC = rng.randrange(depth)
        t.T = rng.getrandbits(1)
        t.S = rng.getrandbits(16)
        t.D = rng.getrandbits(16)
        t.LATE = rng.getrandbits(1)
        t.STALE = rng.getrandbits(1) if rng.random() < .3 else 0
        t.RUN = 1
        t.IV = 1
    return chip


# 5.9.2: widths of the unit's sequencing state that is wider than a bit; RXPH is signed
UNIT_WIDTH = dict(TXSR=16, TXN=5, TXBUF=16, TXBN=5, TXSYM=2, TXCD=17, TXACC=8, TXK=2, TXBK=2, TXAN=6, TXRL=3,
                  RXSR=16, RXN=5, RXTOT=11, RXKIND=2, RXG=5, RXIC=16, RXBUF=16, RXBN=5, RXPN=5, RXRL=3, RXEF=3,
                  RXBE=3, CRC=32, RXPE=3)


def random_unit(rng, chip):
    """Every unit in a random state, every slot written, and the output switches random (uo3 never switches)."""
    for t in chip.th:
        for k in range(16):
            chip.set_cfg(t, k, rng.getrandbits(16))
        for n in UNIT_STATE:
            if n == 'RXPH':
                t.u.RXPH = rng.randrange(-32768, 1 << 24)
            elif n != 'CMT':
                t.u.__dict__[n] = rng.getrandbits(UNIT_WIDTH.get(n, 1))
    chip.SEL = [0 if p == 3 else rng.randrange(3) for p in range(16)]
    chip.SHSEL = [0 if p == 3 else rng.randrange(3) for p in range(16)]
    chip.SYNC3 = rng.getrandbits(16)


def chk_rows(log, trials=300):
    rng = random.Random(20261031)
    urng = random.Random(20261110)      # the shift-unit rows draw their own cases, from unit states drawn here, so
    checked = bad = 0                   # every other row gets the cases it got before those rows existed
    for row in ROWS:
        unit = row.syntax in UNIT_ROWS
        g = urng if unit else rng
        free = [b for bits in row.bits.values() for b in bits]
        n = 0
        tries = 0
        while n < trials and tries < trials * 40:
            tries += 1
            word = row.val
            for b in free:
                if g.getrandbits(1):
                    word |= 1 << b
            d = decode(word)
            if d is None or d[0] is not row:
                continue
            n += 1
            depth = g.choice((128, 512))
            a0 = random_chip(g, depth)
            if unit:
                random_unit(g, a0)
            a1 = copy.deepcopy(a0)
            tid = g.randrange(4)
            a = g.getrandbits(16)
            r0 = row.fn(a0, a0.th[tid], d[1], a)
            r1 = run_doc(a1, a1.th[tid], row, d[1], a)
            checked += 1
            if r0 != r1 or snapshot(a0) != snapshot(a1):
                bad += 1
                if bad < 6:
                    log.line('   mismatch: %s %s %s %s' % (row.syntax, hex(word), r0, r1))
    log.check('6.3 every row\'s handler matches its Semantics text, compiled literally', bad == 0,
              '%d rows, %d random cases, %d mismatches; the %d shift-unit rows on random unit states'
              % (len(ROWS), checked, bad, len(UNIT_ROWS)))



def chk_world(log):
    # 3.6: the list of world instructions, typed from the prose, against the W column
    prose = {'put', 'oe', 'put8', 'oe8', 'pcfg', 'get', 'getn', 'xget', 'aget', 'rds', 'rdd', 'wrd',
             'tlate', 'halt', 'now', 'tick', 'edge', 'until', 'untilm', 'recv', 'send', 'tryrecv',
             'trysend',
             'scfg', 'smap', 'sgo', 'sstop'}      # the shift-unit rows 3.6 names
    table = {r.mn for r in ROWS if r.w == 'y'}
    pin = {r.mn for r in ROWS if r.w == 'pin'}
    log.check('3.6 prose list of world instructions equals the W column', prose == table and
              pin == {'shl8', 'shl16', 'shr8', 'shr16'}, str((prose ^ table, pin)))


ALL = [chk_rows, chk_world, chk_reaction, chk_own_edge, chk_sampling, chk_late_stale, chk_wait_exits, chk_wait_priority, chk_untilm, chk_memory,
       chk_pins, chk_spacing, chk_host, chk_stop_rules, chk_flags, chk_stop_timing, chk_slots, chk_asm]


def main():
    log = Log()
    log.line('=== claims')
    for fn in ALL:
        log.line('--- ' + fn.__name__)
        fn(log)
    log.line('=== %d claims, %d failed' % (log.n, log.bad))
    return 1 if log.bad else 0


if __name__ == '__main__':
    sys.exit(main())
