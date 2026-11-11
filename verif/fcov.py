#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Functional coverage against docs/ISA.md, measured on the sim/sim.py side of a lockstep run.

install() wraps methods of sim.Chip and the row handlers of sim/asm.py's table from outside, so sim/sim.py itself is
unchanged: each wrapper calls the original and counts what it saw. lockstep.py --fcov FILE runs it and writes the
counts as JSON; this script adds up such files and reports them against points(), the list of every point the
specification asks for, so a point that never happened shows as a hole.

Points, by group:
- row: each of the 89 rows of 6.3 executed on each thread (a blocked recv, send, txw or rxw counts as a stall, not
  as a run), and an illegal word on each thread (section 10);
- pin: every pcfg mode change on each kind of pin, from each mode to each (5.3), a claim from another thread and a
  pcfg on a pin that is on a shift output (10, defined 9); smap with each output on each kind of pin, by its owner
  and by another thread; every change of SEL at a commit (5.4, 5.9.2), and an open-drain pin on a shift output;
- unit: the settings a send or reception starts with (5.9.3): every CRC width with the CRC on, CINIT and CXOR,
  every CMODE, SMODE with every SN, TPER, TFRAC and TAIL ranges, line codes, the decoders and their options; every
  change of a half's state (idle, armed, running, tail; idle, armed, hunting, receiving, holding its last word); the
  things a half does (5.9.5 to 5.9.7: underruns, overruns, stuff bits, appends, sync, ends by each cause, the CRC
  check); and sstop and ABORT in each state of each half;
- host: RUN's stop, start and both masks, STEP, SETPC, RDREG and ABORT on a running and on a stopped thread, RDREG of
  each reg8 group, POST and TAKE on a full and an empty word, STATUS with BUSY 0 and 1, memory commands with a free
  memory cycle and with every thread running (lockstep.py counts the last two, since it sets sim.py's BUSY itself
  in h + 1);
- event: each way of arming (tick, edge, now, until, untilm, with and without a timeout and the inbox) and each way
  of ending (on time, late, a match, the inbox, a timeout, cancelled), with LATE 0 and 1 (3.3 to 3.5);
- mail: recv, send, tryrecv and trysend on a full and an empty word, the host's POST and TAKE on each word, and a
  host command on the word a thread uses in the same cycle (9.4).

Usage: fcov.py report FILE.json ... [--md FILE] [--holes FILE]
Standard library only.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.dont_write_bytecode = True

LANES = (0, 4, 5, 6, 7)
TX_STATES = ('idle', 'armed', 'sending', 'tail')
RX_STATES = ('idle', 'armed', 'hunting', 'receiving', 'holding')
ARMING = {'tick i': 'tick', 'edge p, v, i': 'edge', 'edge p, v, Rr': 'edge Rr', 'now': 'now',
          'until p, l [, inbox]': 'until', 'until p, l, Rt [, inbox]': 'until Rt',
          'untilm Rr, b, c [, inbox]': 'untilm', 'untilm Rr, b, c, Rt [, inbox]': 'untilm Rt'}
MAIL_ROWS = ('recv Rn, m', 'send Rn, m', 'tryrecv Rn, m', 'trysend Rn, m')
STALLS = ('recv Rn, m', 'send Rn, m', 'txw Rs, n [, k] [, last]', 'rxw Rd')


def pin_kind(p):
    return 'uio' if p >= 8 else ('lane' if p in LANES else 'uo')


def pin_mode(c, p):
    if p >= 8:
        return 'OD' if c.OD[p - 8] else ('PP' if c.SHOE[p - 8] else 'IN')
    if p in LANES:
        return 'BYP' if c.SHBYP[p] else 'PP'
    return 'PP'


def tx_state(u):
    return 'sending' if u.TXRUN else 'tail' if u.TXTL else 'armed' if u.TXARM else 'idle'


def rx_state(u):
    return 'holding' if u.RXEP else 'receiving' if u.RXRUN else 'hunting' if u.RXHUNT else 'armed' if u.RXARM \
        else 'idle'


def per_range(v):
    return '0' if v == 0 else '1' if v == 1 else '2..15' if v < 16 else '16..255' if v < 256 else '256..65535'


# ---- the points -------------------------------------------------------------------------------------------------
def points():
    """[(group, name)] of every point."""
    import asm
    p = []
    for r in asm.ROWS:
        for t in range(4):
            p.append(('row', 'run %s t%d' % (r.syntax, t)))
    for r in STALLS:
        for t in range(4):
            p.append(('row', 'stall %s t%d' % (r, t)))
    for t in range(4):
        p.append(('row', 'illegal word t%d' % t))
    for k, modes in (('uio', ('IN', 'PP', 'OD')), ('lane', ('PP', 'BYP')), ('uo', ('PP',))):
        for a in modes:
            for b in modes:
                p.append(('pin', 'pcfg %s %s->%s' % (k, a, b)))
        p.append(('pin', 'pcfg %s claims from another thread' % k))
        p.append(('pin', 'pcfg %s on a shift output' % k))
        for lv in ('-', 'A', 'B'):
            p.append(('pin', 'smap %s %s by the owner' % (k, lv)))
        p.append(('pin', 'smap %s by another thread' % k))
        for a in range(3):
            for b in range(3):
                if a != b:
                    p.append(('pin', 'commit %s SEL %d->%d' % (k, a, b)))
    p.append(('pin', 'uio OD pin on a shift output'))
    # unit settings at a start
    for half in ('tx', 'rx'):
        for w in range(1, 33):
            p.append(('unit', '%s CRC width %d' % (half, w)))
        for mode in (1, 2):
            for ci in range(2):
                for cx in range(2):
                    p.append(('unit', '%s CMODE %d CINIT %d CXOR %d' % (half, mode, ci, cx)))
        for mode in range(4):
            p.append(('unit', '%s CMODE %d' % (half, mode)))
        for sm in (1, 2):
            for sn in range(8):
                p.append(('unit', '%s SMODE %d SN %d' % (half, sm, sn)))
        for sm in (0, 3):
            p.append(('unit', '%s SMODE %d' % (half, sm)))
    for r in ('0', '1', '2..15', '16..255', '256..65535'):
        p.append(('unit', 'tx TPER %s' % r))
        p.append(('unit', 'rx timed RPER %s' % r))
        p.append(('unit', 'rx Manchester RPER %s' % r))
    for x in ('0', 'not 0'):
        p += [('unit', 'tx TFRAC %s' % x), ('unit', 'rx RFRAC %s' % x), ('unit', 'rx REND %s' % x),
              ('unit', 'rx RCNT %s' % x), ('unit', 'rx CSKIP %s' % x)]
    for x in ('0', '1', '2..255'):
        p.append(('unit', 'tx TAIL %s' % x))
    for s in range(4):
        p.append(('unit', 'tx SYMS %d' % s))
    for a in range(2):
        for b in range(2):
            p.append(('unit', 'tx TGLA %d TGLB %d' % (a, b)))
    for x in ('own timing', 'outside clock, rising', 'outside clock, falling'):
        p.append(('unit', 'tx %s' % x))
    for x in range(2):
        p.append(('unit', 'tx TMSB %d' % x))
    for code in range(4):
        p.append(('unit', 'tx output code %d used' % code))
    for k in range(4):
        p.append(('unit', 'rx RKIND %d' % k))
        for d in range(2):
            p.append(('unit', 'rx RKIND %d RDIFF %d' % (k, d)))
    for x in ('RINV', 'RMSB', 'RESYNC', 'RACQ', 'REOP'):
        for v in range(2):
            p.append(('unit', 'rx %s %d' % (x, v)))
    for g in range(4):
        p.append(('unit', 'rx RGAIN %d' % g))
    for x in ('no hunt', 'hunt for 0', 'hunt for 1'):
        p.append(('unit', 'rx %s' % x))
    for x in ('rising', 'falling'):
        p.append(('unit', 'rx outside clock, %s' % x))
    for x in ('0', '1..16', '17..31'):
        p.append(('unit', 'rx SYNCLEN %s' % x))
    for x in ('0', '1', '2 and up'):
        p.append(('unit', 'rx timed FIRST %s' % x))
    # state changes and what the halves do
    for a in TX_STATES:
        for b in TX_STATES:
            if a != b:
                p.append(('unit', 'tx %s->%s' % (a, b)))
    for a in RX_STATES:
        for b in RX_STATES:
            if a != b:
                p.append(('unit', 'rx %s->%s' % (a, b)))
    for a in TX_STATES:
        for b in RX_STATES:
            p.append(('unit', 'unit in tx %s and rx %s' % (a, b)))
    for x in ('underrun at the start', 'underrun after a word', 'end at the last word', 'stuff bit sent',
              'CRC appended', 'word loaded while sending', 'word moved into the idle shift register'):
        p.append(('unit', 'tx %s' % x))
    for x in ('word delivered', 'last word delivered', 'overrun', 'sync found', 'stuff bit removed',
              'stuff violation', 'CRC check passed', 'CRC check failed', 'end by RCNT', 'end by REND',
              'end by REOP', 'end by a stuff violation', 'end by the Manchester window', 'gear reached 16',
              'phase pulled by an edge'):
        p.append(('unit', 'rx %s' % x))
    for cmd in ('sstop', 'ABORT'):
        for s in TX_STATES:
            p.append(('unit', '%s with tx %s' % (cmd, s)))
        for s in RX_STATES:
            p.append(('unit', '%s with rx %s' % (cmd, s)))
    for x in ('a', 'b'):
        p.append(('unit', 'sstat cleared TXUR' if x == 'a' else 'sstat cleared RXOVR'))
    # host
    for cmd in ('stop', 'start', 'stop and start', 'STEP', 'SETPC', 'RDREG', 'ABORT'):
        for s in ('running', 'stopped'):
            p.append(('host', '%s on a %s thread' % (cmd, s)))
    for g in ('R0..R7', 'D', 'S', 'PC', 'flags', 'shift status', 'CRC 15..0', 'CRC 31..16', 'end flags',
              'slots', '32 and up'):
        p.append(('host', 'RDREG %s of a stopped thread' % g))
    for x in ('POST to an empty word', 'POST to a full word', 'TAKE of a full word', 'TAKE of an empty word',
              'STATUS with BUSY 0', 'STATUS with BUSY 1', 'WRMEM', 'RDMEM', 'WRMEM with every thread running',
              'RDMEM with every thread running'):
        p.append(('host', x))
    p.append(('host', 'ABORT with a uio pin on a shift output'))
    # events
    for name in sorted(set(ARMING.values())):
        timed = name in ('tick', 'edge', 'edge Rr', 'now') or name.endswith('Rt')
        ends = ['on time', 'late'] if name in ('tick', 'edge', 'edge Rr') else ['on time'] if name == 'now' else \
            ['match', 'inbox'] + (['timeout on time', 'timeout past due'] if timed else [])
        for e in ends:
            for late in range(2):
                if e == 'late' and late == 0:      # a late tick sets LATE
                    continue
                p.append(('event', '%s: %s, LATE %d' % (name, e, late)))
        p.append(('event', '%s: cancelled' % name))
    p += [('event', 'armed on a stale D, so LATE'), ('event', 'STALE set'), ('event', 'tlate read LATE 1'),
          ('event', 'tlate read LATE 0')]
    # mailbox
    for x in ('recv: a full word', 'recv: an empty word, so it stalls', 'send: an empty word',
              'send: a full word, so it stalls', 'tryrecv: a full word', 'tryrecv: an empty word',
              'trysend: an empty word', 'trysend: a full word'):
        p.append(('mail', x))
    for n in range(8):
        for x in ('thread puts', 'thread takes', 'host puts', 'host takes'):
            p.append(('mail', 'word %d: %s' % (n, x)))
    for x in ('POST', 'TAKE'):
        p.append(('mail', 'host %s in the cycle a thread used the same word' % x))
    return p


# ---- counting ---------------------------------------------------------------------------------------------------
class Coverage:
    def __init__(self):
        self.counts = {}

    def hit(self, name, n=1):
        self.counts[name] = self.counts.get(name, 0) + n

    def reset(self):
        self.counts = {}

    def snapshot(self):
        return dict(self.counts)


_cov = None


def process_coverage():
    """The coverage of this process, with the hooks installed the first time."""
    global _cov
    if _cov is None:
        _cov = Coverage()
        install(_cov)
    return _cov


def install(cov):
    sys.path.insert(0, os.path.join(ROOT, 'sim'))
    import asm
    import sim
    Chip = sim.Chip
    hit = cov.hit
    HOLD = sim.HOLD

    # rows
    def wrap_row(r, fn):
        syn, mn = r.syntax, r.mn
        stall = syn in STALLS
        mail = syn in MAIL_ROWS
        arming = ARMING.get(syn)

        def h(c, t, f, a):
            if mail:
                n = f['m']
                was = c.FULL[n]
            if syn == 'tlate':
                hit('event: tlate read LATE %d' % t.LATE)
            out = fn(c, t, f, a)
            if stall and out == HOLD:
                hit('row: stall %s t%d' % (syn, t.TID))
                if mail:
                    hit('mail: %s' % ('recv: an empty word, so it stalls' if mn == 'recv'
                                      else 'send: a full word, so it stalls'))
                return out
            hit('row: run %s t%d' % (syn, t.TID))
            if mail:
                hit('mail: %s: %s word' % (mn, 'a full' if was else 'an empty'))
                if (mn in ('recv', 'tryrecv') and was) or (mn in ('send', 'trysend') and not was):
                    hit('mail: word %d: thread %s' % (n, 'takes' if mn in ('recv', 'tryrecv') else 'puts'))
                    c._fc_mail = (c.c, n)
            if arming:
                t._fc_arm = arming
            return out
        return h
    for r in asm.ROWS:
        if not getattr(r, '_fc', False):
            r.fn = wrap_row(r, r.fn)
            r._fc = True

    def wrap(name, before=None, after=None, static=False):
        fn = Chip.__dict__[name]
        fn = fn.__func__ if static else fn

        def w(*a):
            ctx = before(*a) if before else None
            out = fn(*a)
            if after:
                after(ctx, out, *a)
            return out
        setattr(Chip, name, staticmethod(w) if static else w)

    # illegal words
    wrap('execute', lambda c, t, word, P, a: t.ILL,
         lambda ill, out, c, t, word, P, a: hit('row: illegal word t%d' % t.TID) if t.ILL and not ill else None)

    # pins
    def pcfg_before(c, t, p):
        k = pin_kind(p)
        if c.OWN[p] != t.TID:
            hit('pin: pcfg %s claims from another thread' % k)
        if c.SEL[p]:
            hit('pin: pcfg %s on a shift output' % k)
        return k, pin_mode(c, p)

    def pcfg_after(ctx, out, c, t, p):
        hit('pin: pcfg %s %s->%s' % (ctx[0], ctx[1], pin_mode(c, p)))
    for n in ('pcfg_in', 'pcfg_pp', 'pcfg_od', 'pcfg_byp'):
        wrap(n, pcfg_before, pcfg_after)

    def smap_before(c, t, p, lv):
        k = pin_kind(p)
        if c.OWN[p] == t.TID:
            hit('pin: smap %s %s by the owner' % (k, '-AB'[lv]))
        else:
            hit('pin: smap %s by another thread' % k)
    wrap('shadow_sel', smap_before)

    def commit_after(sel, out, c, t):
        for p in range(16):
            if c.SEL[p] != sel[p]:
                hit('pin: commit %s SEL %d->%d' % (pin_kind(p), sel[p], c.SEL[p]))
            if p >= 8 and c.SEL[p] and c.OD[p - 8]:
                hit('pin: uio OD pin on a shift output')
    wrap('commit', lambda c, t: list(c.SEL), commit_after)

    # unit settings, at a start of each half
    def tx_begin_before(c, u):
        if u.CTX:                                   # the CRC register belongs to this half
            hit('unit: tx CMODE %d' % u.CMODE)
        if u.CTX and u.CMODE in (1, 2):
            if u.CMODE == 1:
                hit('unit: tx CRC width %d' % (u.CW + 1))
            hit('unit: tx CMODE %d CINIT %d CXOR %d' % (u.CMODE, u.CINIT, u.CXOR))
        hit('unit: tx SMODE %d SN %d' % (u.SMODE, u.SN) if u.SMODE in (1, 2) else 'unit: tx SMODE %d' % u.SMODE)
        if u.TEXT:
            hit('unit: tx outside clock, %s' % ('falling' if u.TFALL else 'rising'))
        else:
            hit('unit: tx own timing')
            hit('unit: tx TPER %s' % per_range(u.TPER))
            if u.TPER:
                hit('unit: tx TFRAC %s' % ('0' if u.TFRAC == 0 else 'not 0'))
        hit('unit: tx TAIL %s' % ('0' if u.TAIL == 0 else '1' if u.TAIL == 1 else '2..255'))
        hit('unit: tx SYMS %d' % u.SYMS)
        hit('unit: tx TGLA %d TGLB %d' % (u.TGLA, u.TGLB))
        hit('unit: tx TMSB %d' % u.TMSB)
        for s in range(u.SYMS + 1):
            hit('unit: tx output code %d used' % ((u.MAPA >> (2 * s)) & 3))
            hit('unit: tx output code %d used' % ((u.MAPB >> (2 * s)) & 3))
    wrap('tx_begin', tx_begin_before)

    def rx_begin_before(c, u):
        k = u.RKIND
        hit('unit: rx RKIND %d' % k)
        hit('unit: rx RKIND %d RDIFF %d' % (k, u.RDIFF))
        if not u.CTX:
            hit('unit: rx CMODE %d' % u.CMODE)
            if u.CMODE in (1, 2):
                if u.CMODE == 1:
                    hit('unit: rx CRC width %d' % (u.CW + 1))
                hit('unit: rx CMODE %d CINIT %d CXOR %d' % (u.CMODE, u.CINIT, u.CXOR))
                hit('unit: rx CSKIP %s' % ('0' if u.CSKIP == 0 else 'not 0'))
        hit('unit: rx SMODE %d SN %d' % (u.SMODE, u.SN) if u.SMODE in (1, 2) else 'unit: rx SMODE %d' % u.SMODE)
        hit('unit: rx RINV %d' % u.RINV)
        hit('unit: rx RMSB %d' % u.RMSB)
        hit('unit: rx no hunt' if not u.HUNT else 'unit: rx hunt for %d' % u.HLVL)
        hit('unit: rx SYNCLEN %s' % ('0' if u.SYNCLEN == 0 else '1..16' if u.SYNCLEN <= 16 else '17..31'))
        hit('unit: rx REND %s' % ('0' if u.REND == 0 else 'not 0'))
        hit('unit: rx RCNT %s' % ('0' if u.RCNT == 0 else 'not 0'))
        hit('unit: rx RGAIN %d' % u.RGAIN)
        if k == 2:
            hit('unit: rx outside clock, %s' % ('falling' if u.RFALL else 'rising'))
        else:
            hit('unit: rx %s RPER %s' % ('Manchester' if k == 1 else 'timed', per_range(u.RPER)))
            if u.RPER:
                hit('unit: rx RFRAC %s' % ('0' if u.RFRAC == 0 else 'not 0'))
        if k in (0, 3):
            hit('unit: rx RESYNC %d' % u.RESYNC)
            hit('unit: rx REOP %d' % u.REOP)
            hit('unit: rx timed FIRST %s' % ('0' if u.FIRST == 0 else '1' if u.FIRST == 1 else '2 and up'))
        if k == 1 and u.RGAIN:
            hit('unit: rx RACQ %d' % u.RACQ)
    wrap('rx_begin', rx_begin_before)

    # sstop and ABORT in each state
    def sstop_before(c, t, m):
        u = t.u
        if m & 1:
            hit('unit: sstop with tx %s' % tx_state(u))
        if m & 2:
            hit('unit: sstop with rx %s' % rx_state(u))
    wrap('sstop', sstop_before)

    def sstat_before(t, d):
        p = t.pre
        if p.TXUR:
            hit('unit: sstat cleared TXUR')
        if p.RXOVR:
            hit('unit: sstat cleared RXOVR')
    wrap('sstat', sstat_before, static=True)

    def tx_load_before(u):
        if u.TXRUN:
            hit('unit: tx word loaded while sending')
        elif u.TXN == 0:
            hit('unit: tx word moved into the idle shift register')
    wrap('tx_load', tx_load_before, static=True)

    def rx_end_before(u):
        if u.RXVIO:
            hit('unit: rx end by a stuff violation')
        elif u.RCNT and u.RXTOT == u.RCNT:
            hit('unit: rx end by RCNT')
        elif u.REND and u.RXIC >= u.REND:
            hit('unit: rx end by REND')
        elif u.RXKIND == 1:
            hit('unit: rx end by the Manchester window')
        elif u.REOP:
            hit('unit: rx end by REOP')
        if u.CMODE == 1 and not u.CTX:
            hit('unit: rx CRC check %s' % ('passed' if u.CRC == ((u.CHKH << 16) | u.CHKL) else 'failed'))
    wrap('rx_end', rx_end_before, static=True)

    def pull_before(u, r):
        hit('unit: rx phase pulled by an edge')
    wrap('rx_pull', pull_before, static=True)

    # events
    def events_before(c, t, TB):
        return t.P, t.KIND, t.DUE, t.D, t.LATE

    def events_after(ctx, out, c, t, TB):
        P, kind, due, d, late0 = ctx
        if not P or t.P:
            return
        name = getattr(t, '_fc_arm', None)
        if name is None:
            return
        if kind == sim.TICK:
            e = 'late' if due else 'on time'
        elif t.T:
            e = 'match'
        elif t.INBOX and c.FULL[t.TID]:
            e = 'inbox'
        else:
            e = 'timeout past due' if due else 'timeout on time'
        hit('event: %s: %s, LATE %d' % (name, e, t.LATE))
        t._fc_arm = None
    wrap('events', events_before, events_after)

    def sched_before(c, t, step, a):
        if t.STALE:
            hit('event: armed on a stale D, so LATE')
    wrap('schedule', sched_before)

    def stale_before(t):
        return t.STALE

    def stale_after(s, out, t):
        if t.STALE and not s:
            hit('event: STALE set')
    wrap('update_stale', stale_before, stale_after, static=True)

    def cancelled(t):
        name = getattr(t, '_fc_arm', None)
        if name is not None and t.P:
            hit('event: %s: cancelled' % name)
            t._fc_arm = None

    def stop_before(t):
        cancelled(t)
    wrap('stop', stop_before, static=True)

    def retire_before(t):
        if t.STEPPING and t.STOPREQ:
            cancelled(t)
    wrap('retire', retire_before, static=True)

    # the host
    def run_before(c, stop, start):
        for t in c.th:
            s, g = (stop >> t.TID) & 1, (start >> t.TID) & 1
            st = 'running' if t.RUN else 'stopped'
            if s and g:
                hit('host: stop and start on a %s thread' % st)
            elif s:
                hit('host: stop on a %s thread' % st)
            elif g:
                hit('host: start on a %s thread' % st)
            if (s and not t.RUN) or (g and not s and not t.RUN):
                cancelled(t)
    wrap('host_run', run_before)
    wrap('host_step', lambda c, tid: hit('host: STEP on a %s thread' % ('running' if c.th[tid & 3].RUN
                                                                        else 'stopped')))
    wrap('host_setpc', lambda c, tid, pc: hit('host: SETPC on a %s thread' % ('running' if c.th[tid & 3].RUN
                                                                             else 'stopped')))
    RD = {8: 'D', 9: 'S', 10: 'PC', 11: 'flags', 12: 'shift status', 13: 'CRC 15..0', 14: 'CRC 31..16',
          15: 'end flags'}

    def rdreg_before(c, tid, reg):
        t = c.th[tid & 3]
        hit('host: RDREG on a %s thread' % ('running' if t.RUN else 'stopped'))
        if not t.RUN:
            g = 'R0..R7' if reg < 8 else RD.get(reg) or ('slots' if reg < 32 else '32 and up')
            hit('host: RDREG %s of a stopped thread' % g)
    wrap('host_rdreg', rdreg_before)

    def post_before(c, n, v):
        hit('host: POST to %s word' % ('a full' if c.FULL[n] else 'an empty'))
        if not c.FULL[n]:
            hit('mail: word %d: host puts' % n)
        if getattr(c, '_fc_mail', None) == (c.c - 1, n):
            hit('mail: host POST in the cycle a thread used the same word')
    wrap('host_post', post_before)

    def take_before(c, n):
        hit('host: TAKE of %s word' % ('a full' if c.FULL[n] else 'an empty'))
        if c.FULL[n]:
            hit('mail: word %d: host takes' % n)
        if getattr(c, '_fc_mail', None) == (c.c - 1, n):
            hit('mail: host TAKE in the cycle a thread used the same word')
    wrap('host_take', take_before)
    wrap('host_status', lambda c: hit('host: STATUS with BUSY %d' % c.BUSY))

    def abort_before(c):
        if any(c.SEL[8:]):
            hit('host: ABORT with a uio pin on a shift output')
        for t in c.th:
            hit('host: ABORT on a %s thread' % ('running' if t.RUN else 'stopped'))
            hit('unit: ABORT with tx %s' % tx_state(t.u))
            hit('unit: ABORT with rx %s' % rx_state(t.u))
            cancelled(t)
    wrap('host_abort', abort_before)

    def mem_before(name):
        def b(c, *a):
            hit('host: %s%s' % (name, ' with every thread running' if all(t.RUN for t in c.th) else ''))
        return b
    wrap('host_wrmem', mem_before('WRMEM'))
    wrap('host_rdmem', mem_before('RDMEM'))

    # every cycle: the state of each half, and what each did
    def snap(u):
        return (tx_state(u), rx_state(u), u.TXUR, u.TXSTF, u.TXAN != 0, u.RXFULL, u.RXOVR, u.RXSYN, u.RXEF,
                u.RXG, u.RXBL, u.TXLW, u.RXRL)

    def cycle_before(c, ui, uio):
        return [snap(t.u) for t in c.th]

    def cycle_after(pre, out, c, ui, uio):
        for t, a in zip(c.th, pre):
            u = t.u
            b = snap(u)
            if a == b:
                continue
            if a[0] != b[0]:
                hit('unit: tx %s->%s' % (a[0], b[0]))
                if a[0] == 'armed' and b[0] in ('idle', 'tail') and b[2] and not a[2]:
                    hit('unit: tx underrun at the start')
                elif a[0] == 'sending' and b[2] and not a[2]:
                    hit('unit: tx underrun after a word')
                elif a[0] == 'sending' and b[0] != 'sending' and a[11]:
                    hit('unit: tx end at the last word')
            if a[1] != b[1]:
                hit('unit: rx %s->%s' % (a[1], b[1]))
            hit('unit: unit in tx %s and rx %s' % (b[0], b[1]))
            if b[3] and not a[3]:
                hit('unit: tx stuff bit sent')
            if b[4] and not a[4]:
                hit('unit: tx CRC appended')
            if b[5] and not a[5]:
                hit('unit: rx last word delivered' if u.RXBL else 'unit: rx word delivered')
            if b[6] and not a[6]:
                hit('unit: rx overrun')
            if b[7] and not a[7] and b[1] == 'receiving' and a[1] == 'receiving':
                hit('unit: rx sync found')
            if (b[8] & 2) and not (a[8] & 2):
                hit('unit: rx stuff violation')
            if b[9] == 16 and a[9] != 16:
                hit('unit: rx gear reached 16')
    wrap('cycle', cycle_before, cycle_after)

    def rx_bit_before(c, u, b):
        stuffing = u.SMODE in (1, 2)
        if u.RXSYN and stuffing and not u.RXVIO and u.RXRL != 0 and u.RXRL >= u.SN and \
                (u.SMODE == 1 or u.RXRL >= 2) and (b ^ u.RINV) != u.RXRV:
            hit('unit: rx stuff bit removed')
    wrap('rx_bit', rx_bit_before)


# ---- the report -------------------------------------------------------------------------------------------------
def report(argv):
    md = holes_out = None
    files = []
    it = iter(argv)
    for x in it:
        if x == '--md':
            md = next(it)
        elif x == '--holes':
            holes_out = next(it)
        else:
            files.append(x)
    sys.path.insert(0, os.path.join(ROOT, 'sim'))
    counts = {}
    runs = []
    for f in files:
        with open(f) as fh:
            d = json.load(fh)
        runs.append(d.get('run', f))
        for k, v in d['counts'].items():
            counts[k] = counts.get(k, 0) + v
    pts = points()
    known = {'%s: %s' % (g, n) for g, n in pts}
    groups = []
    for g, _ in pts:
        if g not in groups:
            groups.append(g)
    tab = ['| Group | Points | Hit | Holes |', '|---|---|---|---|']
    holes = []
    for g in groups:
        ps = [n for gg, n in pts if gg == g]
        h = [n for n in ps if not counts.get('%s: %s' % (g, n))]
        holes += ['%s: %s' % (g, n) for n in h]
        tab.append('| %s | %d | %d | %d |' % (g, len(ps), len(ps) - len(h), len(h)))
    tab.append('| all | %d | %d | %d |' % (len(pts), len(pts) - len(holes), len(holes)))
    extra = sorted(k for k in counts if k not in known)
    print('\n'.join(tab))
    print('%d holes:' % len(holes))
    for h in holes:
        print('  ' + h)
    if extra:
        print('%d counted names that are not points (a bug in fcov.py):' % len(extra))
        for e in extra[:40]:
            print('  ' + e)
    if md:
        with open(md, 'w') as fh:
            fh.write('\n'.join(tab) + '\n')
    if holes_out:
        with open(holes_out, 'w') as fh:
            fh.write('\n'.join(['%s  %d' % (k, counts.get(k, 0)) for k in sorted(known)]) + '\n')
    return 0


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'report':
        sys.exit(report(sys.argv[2:]))
    print(__doc__)
    sys.exit(2)
