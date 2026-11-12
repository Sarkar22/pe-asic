"""Bench for advanced/uart_bypass.s: a UART line through the bypass lane of uo4, with one byte replaced in flight.

The sender model drives 8N1 frames on ui4, B cycles per bit on an exact grid, abutting or with gaps. The host posts
the bit time, the new byte and the rule, takes MACK, starts the stream, and takes MOUT. What uo4 must show comes from
the stimulus alone: on every cycle the level of ui4, except the chosen frame's 8 data bits, which are the new byte's,
each on that frame's own bit boundaries. The lane's switches move no pad, by design, so the bench reads where each
one fell from the chip, as BYP and OUT of uo4, and checks that nothing moved around it.

Bit times are cycles at 50 MHz: 16 (3.125 Mbaud, the fastest the program is written for), 17, 25 (2 Mbaud),
50 (1 Mbaud), 434 (115200 baud), 5208 (9600 baud) and 6553 (7630 baud, the slowest).
"""
import random

from protolib import Device, Rig, assemble, src
from rig import CLK, uart_decode

PROGRAMS = ('advanced/uart_bypass.s',)
RX = TX = 4                     # ui4 in, uo4 out: in BYP the lane of uo4 shows ui4 (ISA 5.2)
MCFG, MRULE, MREP, MACK, MOUT = 7, 0, 1, 4, 5
BYTE = 0x100                    # MRULE bit 8: a byte rule
EDGE_ROOM = 3                   # cycles a switch keeps from any edge of ui4, and from any change of OUT or uo4


class Line(Device):
    """Drives ui4, idle high. frames() schedules 8N1 frames, LSB first, one bit every `bit` cycles from `start`,
    with gaps[k] idle cycles after frame k. It can be called while the board runs."""

    def __init__(self):
        self.ui_mask = 1 << RX
        self.ui_val = self.ui_mask
        self.sched, self.i, self.starts = [], 0, []

    def frames(self, start, data, bit, gaps=()):
        """start must be 2 or more cycles ahead of the board: step() sets the level of the cycle after the one it
        runs in."""
        t = start
        for k, v in enumerate(data):
            self.starts.append(t)
            for j, b in enumerate([0] + [(v >> i) & 1 for i in range(8)] + [1]):
                self.sched.append((t + j * bit, b))
            t += 10 * bit + (gaps[k] if k < len(gaps) else 0)
        return t

    def step(self, c, uo, io):
        s = self.sched
        while self.i < len(s) and s[self.i][0] <= c + 1:
            self.ui_val = s[self.i][1] << RX
            self.i += 1


class Watch:
    """BYP and OUT of uo4 as the chip holds them in each cycle, as (cycle, value) at each change. Board.run calls it
    after each cycle, when chip.c is the next cycle and the flops hold what that cycle sees."""

    def __init__(self, chip):
        self.chip, self.byp, self.out = chip, [], []

    def __call__(self):
        c = self.chip
        for rec, v in ((self.byp, c.BYP[TX]), (self.out, c.OUT[TX])):
            if not rec or rec[-1][1] != v:
                rec.append((c.c, v))
        return False


def target(rule, data):
    """The frame a rule replaces in a stream, or None: frame N of a frame rule, the frame after the first one whose
    byte is the rule's byte."""
    if rule is None:
        return None
    if rule & BYTE:
        for k, v in enumerate(data[:-1]):
            if v == rule & 255:
                return k + 1
        return None
    return rule if rule < len(data) else None


class Run:
    pass


def case(bit, plan, thread=0, align=0, text=None, watch=True):
    """One run. plan is a list of (rule, new byte, data, lead, gaps), one stream each: the host posts the rule and
    the byte, takes MACK, starts the stream `lead` cycles later, and takes MOUT if the rule replaces a frame. A rule
    of None posts nothing, and its stream starts 400 cycles after the bit time. watch=False leaves the chip's flops
    unread, for a run that only its pads judge."""
    line = Line()
    r = Rig(devices=[line])
    img = assemble(text if text is not None else src('uart_bypass'), depth=128)
    r.load(img, {thread: 'start'})
    res = Run()
    res.r, res.img, res.bit, res.line = r, img, bit, line
    res.acks, res.outs, res.streams = [], [], []

    def agent():
        yield from r.host.post(MCFG, bit)
        for rule, new, data, lead, gaps in plan:
            if rule is None:
                yield 400
            else:
                yield from r.host.post(MREP, new)
                yield from r.host.post(MRULE, rule)
                yield from r.host.take(MACK, res.acks)
            end = line.frames(r.chip.c + lead, data, bit, gaps)
            res.streams.append((line.starts[-len(data):], list(data), rule, new))
            if target(rule, data) is not None:
                yield from r.host.take(MOUT, res.outs)
            yield max(1, end - r.chip.c) + 20 * bit

    w = Watch(r.chip) if watch else None
    r.board.agent(agent())
    r.start(align)
    while not r.agents_done() and r.chip.c < 10 ** 7:
        r.run(1000, stop=w)
    res.byp, res.out = (w.byp, w.out) if w else ([], [])
    return res


def levels(w, lo, hi):
    """The levels of a wave in cycles lo to hi - 1."""
    a = bytearray(hi - lo)
    cur, last = w.at(lo), lo
    for t, v in w.edges(lo + 1, hi):
        a[last - lo:t - lo] = bytes([cur]) * (t - last)
        cur, last = v, t
    a[last - lo:] = bytes([cur]) * (hi - last)
    return a


def history(rec, lo, hi):
    """The levels of a (cycle, value) change list in cycles lo to hi - 1."""
    a = bytearray(hi - lo)
    for i, (t, v) in enumerate(rec):
        end = rec[i + 1][0] if i + 1 < len(rec) else hi
        a0, a1 = max(t, lo) - lo, min(end, hi) - lo
        if a1 > a0:
            a[a0:a1] = bytes([v]) * (a1 - a0)
    return a


def judge(res):
    """Every way a run differs from what its stimulus says, as text: an empty list is a pass."""
    r, bit = res.r, res.bit
    bad = list(r.health())
    tr = r.board.trace
    first = next((t for t, v in res.byp if v == 1), None)
    if first is None:
        return bad + ['uo4 never went on the lane']
    lo, hi = first, tr.end
    ui, uo = levels(tr.wave('ui', RX), lo, hi), levels(tr.wave('uo', TX), lo, hi)
    want = bytearray(ui)
    switches, outs, acks = [], [], []
    for starts, data, rule, new in res.streams:
        if rule is not None:
            acks.append(rule)
        k = target(rule, data)
        if k is None:
            continue
        s = starts[k]
        for j in range(8):
            a, b = s + (j + 1) * bit - lo, s + (j + 2) * bit - lo
            want[a:b] = bytes([(new >> j) & 1]) * (b - a)
        switches += [(s + 3, 0, s), (s + 9 * bit + 9, 1, s)]
        outs.append(data[k])
    diff = [lo + i for i in range(hi - lo) if uo[i] != want[i]]
    if diff:
        bad.append('uo4 differs from what the stimulus says on %d cycles, the first at %d' % (len(diff), diff[0]))
    got = [(t, v) for t, v in res.byp if t > first]
    if [(t, v) for t, v, _ in switches] != got:
        bad.append('lane switches at %s, the header says %s' % (got[:4], [(t, v) for t, v, _ in switches][:4]))
    outq = history(res.out, lo, hi) if switches else []
    for t, v, s in switches:
        span = range(t - EDGE_ROOM - lo, t + EDGE_ROOM - lo)
        if not all(0 <= i < hi - lo for i in span):
            bad.append('switch at %d too near the end of the run' % t)
            continue
        sides = {(ui[i], outq[i]) for i in span}
        if len(sides) != 1 or len({uo[i] for i in span}) != 1 or ui[span[0]] != outq[span[0]]:
            bad.append('switch at %d: ui4, OUT and uo4 are not one steady level from 3 cycles before it to 3 after '
                       '(%s)' % (t, sorted(sides)))
        where = 'start' if v == 0 else 'stop'
        a, b = (s, s + bit) if v == 0 else (s + 9 * bit, s + 10 * bit)
        if not a + EDGE_ROOM <= t <= b - EDGE_ROOM:
            bad.append('switch at %d is not inside the %s bit %d..%d with %d cycles to spare' % (t, where, a, b - 1,
                                                                                               EDGE_ROOM))
    if [v for _, v in res.acks] != acks:
        bad.append('MACK %s, the rules were %s' % ([v for _, v in res.acks], acks))
    if [v for _, v in res.outs] != outs or r.chip.FULL[MOUT]:
        bad.append('MOUT %s, the replaced frames brought in %s' % ([v for _, v in res.outs], outs))
    # the bus as a receiver sees it: the bench's own 8N1 decoder, on the bit grid within 2% of a bit
    t0 = min(st[0] for st, _, _, _ in res.streams) - 5
    sent = sum((d for _, d, _, _ in res.streams), [])
    shown = []
    for starts, data, rule, new in res.streams:
        k = target(rule, data)
        shown += [new if i == k else v for i, v in enumerate(data)]
    for name, wave, wanted in (('ui4', tr.wave('ui', RX), sent), ('uo4', tr.wave('uo', TX), shown)):
        got, errs = uart_decode(wave, CLK / bit, lo=t0)
        if got != wanted or errs:
            bad.append('%s decodes to %s %s, wanted %s' % (name, [hex(x) for x in got][:8], errs[:2],
                                                          [hex(x) for x in wanted][:8]))
    return bad


def stream(rng, n):
    return [rng.randrange(256) for _ in range(n)]


def other(rng, v):
    """A byte that differs from v in data bits 0 and 7, so the replaced window's first and last edges show."""
    return v ^ 0x81 ^ (rng.randrange(256) & 0x7E)


def sweep(log, name, bit, rule_kind, rng, threads=range(4), aligns=range(4), phases=range(4), n=6, gaps=False,
          text=None):
    """Runs over threads x alignments x sender phases, each with its own stream and rule. Returns (good, runs, the
    first failures)."""
    good = runs = 0
    fails = []
    for th in threads:
        for al in aligns:
            for ph in phases:
                data = stream(rng, n)
                if rule_kind == 'frame':
                    k = rng.randrange(n)
                    rule = k
                else:
                    k = rng.randrange(1, n)
                    data[k - 1] = rng.choice([v for v in range(256) if v not in data[:k - 1]])
                    rule = BYTE | data[k - 1]
                new = other(rng, data[k])
                gp = [rng.choice((0, 0, 1, bit // 2, 3 * bit + 7)) for _ in range(n)] if gaps else ()
                res = case(bit, [(rule, new, data, 40 + ph, gp)], th, al, text=text)
                bad = judge(res)
                runs += 1
                good += not bad
                if bad and len(fails) < 3:
                    fails.append('thread %d align %d phase %d: %s' % (th, al, ph, '; '.join(bad[:2])))
    if log:
        log.check(name, good == runs, '%d of %d' % (good, runs) + ('; ' + ' | '.join(fails) if fails else ''))
    return good, runs, fails


TEETH = {
    'late': ('the new bits land one cycle late: data bit 0\'s commit one cycle after the bit boundary',
             'subi  R3, 3 ', 'subi  R3, 2 '),
    'bittime': ('a wrong bit time: the new bits one cycle longer than the line\'s',
                '        ldi   R4, 7\n', '        ldi   R4, 7\n        addi  R2, 1\n'),
    'early': ('uo4 back on the lane 3 cycles before the stop bit, in data bit 7',
              '        put   TXP, 1\n        tick  R2              ; the stop bit, from s + 9B\n'
              '        pcfg  TXP, BYP\n        tick  9 ',
              '        pcfg  TXP, BYP\n        tick  R3 '),
    'edge': ('uo4 back on the lane in the stop bit\'s first cycle, on the edge ui4 may have there',
             '        put   TXP, 1\n        tick  R2              ; the stop bit, from s + 9B\n'
             '        pcfg  TXP, BYP\n        tick  9 ',
             '        pcfg  TXP, BYP\n        tick  R2 '),
    'backlate': ('uo4 back on the lane one cycle later than the header says, s + 9B + 10',
                 '        tick  9 ', '        tick  10 '),
}


def plant(name):
    """A broken copy of the program, made in memory from its source text."""
    _, old, new = TEETH[name]
    text = src('uart_bypass')
    assert text.count(old) == 1, name
    return text.replace(old, new)


def main(log):
    log.group(*PROGRAMS, title='a UART line on the bypass lane of uo4, one byte replaced in flight')
    rng = random.Random(11)
    img = assemble(src('uart_bypass'), depth=128)
    log.info('uart_bypass is %d words' % img.size)

    # every thread, every alignment of its start, every phase of the sender against the thread's slots
    for kind in ('frame', 'byte'):
        sweep(log, '%s rule at 16 cycles per bit (3.125 Mbaud), abutting frames, 4 threads x 4 alignments x 4 '
              'sender phases' % kind, 16, kind, rng)
    for bit, aligns in ((17, range(4)), (25, (0, 2)), (50, (1, 3))):
        for kind in ('frame', 'byte'):
            sweep(log, '%s rule at %d cycles per bit, frames abutting or with gaps of 1 cycle to 3 bits, 4 threads '
                  'x %d alignments x 4 phases' % (kind, bit, len(aligns)), bit, kind, rng, aligns=aligns, gaps=True)
    for bit, n, threads, phases in ((434, 4, range(4), (0, 3)), (5208, 3, (0, 3), (1,)), (6553, 3, (1, 2), (2,))):
        for kind in ('frame', 'byte'):
            sweep(log, '%s rule at %d cycles per bit (%d baud), %d threads, 1 alignment, %d phases'
                  % (kind, bit, round(CLK / bit), len(threads), len(phases)), bit, kind, rng, threads=threads,
                  aligns=(1,), phases=phases, n=n)

    # where the chosen frame sits, and what the new byte is
    data = [0x41, 0x55, 0x00, 0xFF, 0x0F, 0xF0]
    for name, rule, new in (('the first frame after MACK', 0, 0x5A), ('the last frame', 5, 0x0F),
                            ('a frame number past the stream: nothing replaced', 9, 0x00),
                            ('the new byte equal to the old one: switches, and no visible change', 3, 0xFF),
                            ('byte rule on the first frame\'s byte', BYTE | 0x41, 0xAA),
                            ('byte rule on a byte that comes in the last frame only: nothing replaced',
                             BYTE | 0xF0, 0x11),
                            ('byte rule on a byte that never comes: nothing replaced', BYTE | 0x99, 0x11)):
        bad = judge(case(16, [(rule, new, data, 41, ())], thread=1, align=2))
        log.check('%s, 16 cycles per bit' % name, not bad, '; '.join(bad[:2]))
    twice = [0x10, 0x7E, 0x22, 0x7E, 0x33, 0x44]
    res = case(25, [(BYTE | 0x7E, 0xC3, twice, 40, ())], thread=2)
    bad = judge(res)
    log.check('byte rule whose byte comes twice: only the frame after the first is replaced', not bad and
              [v for _, v in res.outs] == [0x22], '; '.join(bad[:2]))

    # no rule: the output is the input on every cycle, and the lane never switches
    for bit in (16, 50, 434):
        res = case(bit, [(None, 0, stream(rng, 6), 40, ())], thread=3, align=1)
        bad = judge(res)
        log.check('no rule posted, %d cycles per bit: uo4 equals ui4 on every cycle, no switch, no MACK, no MOUT'
                  % bit, not bad and len(res.byp) == 2, '; '.join(bad[:2]))

    # two rules, one after the other, on one run
    s1, s2 = stream(rng, 6), stream(rng, 6)
    res = case(50, [(2, other(rng, s1[2]), s1, 40, ()), (BYTE | s2[0], other(rng, s2[1]), s2, 43, (0, 13, 0, 50, 7))],
               thread=1, align=3)
    bad = judge(res)
    log.check('two rules in turn on one run, a frame rule then a byte rule: both frames replaced, 4 switches',
              not bad and len(res.byp) == 6, '; '.join(bad[:2]))

    # below the limit: the arming after a frame's word comes too late on some phases
    for kind, bits in (('frame', (12, 11)), ('byte', (15, 14))):
        for bit in bits:
            g, n, _ = sweep(None, '', bit, kind, random.Random(5), aligns=(0,))
            log.info('below the limit, %s rule at %d cycles per bit, abutting frames, 4 threads x 4 phases: %d of %d '
                     'runs pass' % (kind, bit, g, n))

    # broken copies: each must fail somewhere in a small sweep
    for name, (what, _, _) in TEETH.items():
        text = plant(name)
        bad = []
        for bit in (16, 50):
            for th in (0, 3):
                res = case(bit, [(2, 0x7E, [0x00, 0xFF, 0x81, 0x55, 0x0F], 41 + th, ())], th, 1, text=text)
                bad += judge(res)
        log.check('a broken copy is caught: %s' % what, bad, bad[0] if bad else 'all runs passed')
