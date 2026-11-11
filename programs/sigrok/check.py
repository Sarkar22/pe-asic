#!/usr/bin/env python3
"""Checks the programs' buses with sigrok's protocol decoders.

The benches in programs/bench/ check each program with device models and decoders written alongside the
firmware, so a misreading shared by both would not show. This script runs the same bench scenarios, records the
board as a logic analyzer would see it (the 8 uo pins, the 8 ui pins, and the levels of the 8 uio lines, which
are the wired-AND of the chip's pads, the device models' pulls and the pull-ups), writes each run as a VCD file,
decodes it with sigrok-cli, and compares the decoded payload with what the scenario's stimulus says it must be:
the bytes the host posted, the replies a device model was given, the target's memory. The bench's own decoder
is run on the same record as a cross-check, and any disagreement is reported. Error annotations of sigrok's
decoders (framing, parity, NACK, CRC, stuffing, PID and timing warnings) fail a good run.

Usage, from the repository root:
    python3 programs/sigrok/check.py               every check, on sim/sim.py
    python3 programs/sigrok/check.py uart can      only the groups whose names contain one of the words
    python3 programs/sigrok/check.py --rtl         on the RTL through verif/rtl_chip.py; each run's record must
                                                   also equal the simulator's, cycle for cycle
Options: --build DIR (the Verilator models for --rtl, default verif/build or $PE_BUILD), --work DIR (where the
VCD files go, default programs/sigrok/work), --list (the groups), --json FILE (the results table).

Needs sigrok-cli; the checks were written against sigrok-cli 0.7.2 with libsigrokdecode 0.5.3. Without it the
script says so and exits 0. Python 3 standard library otherwise.
"""
import json
import os
import re
import shutil
import subprocess
import sys
import time
import traceback

if __name__ == '__main__':
    sys.modules['check'] = sys.modules['__main__']     # cases.py imports this module as check: one copy only

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'programs', 'bench'))
sys.dont_write_bytecode = True

import report                                           # noqa: E402  (puts sim/ on the path)
import sim                                              # noqa: E402


# ---------------------------------------------------------------------------------------------------------------
# the recorder: a chip class that keeps every pad and board level, run-length stored

class State:
    cls = None                  # the recording chip class in use
    chips = []                  # chips made since the last reset, in order
    thread = 0                  # the thread simple.Run starts (the benches' Run always starts thread 0)


def recording(base):
    """A subclass of a chip class (sim.Chip or rtl_chip.Chip) that records, for every cycle, the pads the board saw
    (uo, uio_out, uio_oe, from the last pads() call before cycle()) and the levels the board gave the chip (ui and
    the uio lines). The bit layout is the benches' (rig.Trace): uo, uio_out << 8, uio_oe << 16, ui << 24, uio << 32."""

    class Recording(base):
        def __init__(self, *a, **k):
            base.__init__(self, *a, **k)
            self.rec_t, self.rec_v = [], []
            self._rec_pads = (0, 0, 0)
            State.chips.append(self)

        def pads(self, ui_in):
            p = base.pads(self, ui_in)
            self._rec_pads = p
            return p

        def cycle(self, ui_in, uio_in):
            uo, o, oe = self._rec_pads
            v = uo | o << 8 | oe << 16 | (ui_in & 255) << 24 | (uio_in & 255) << 32
            if not self.rec_v or self.rec_v[-1] != v:
                self.rec_t.append(self.c)
                self.rec_v.append(v)
            base.cycle(self, ui_in, uio_in)

    Recording.__name__ = 'Recording' + base.__name__
    return Recording


def make_chip(*a, **k):
    return State.cls(*a, **k)


SIM_CLS = recording(sim.Chip)
State.cls = SIM_CLS
report.Chip = make_chip         # before any bench module is imported: they all take Chip from report

import simple                                           # noqa: E402


class ThreadRun(simple.Run):
    """simple.Run, with the program started on State.thread instead of thread 0."""

    def __init__(self, text, defs=None, depth=128):
        img = report.assemble(text, defines=dict(defs) if defs else None, depth=depth)
        self.img, self.words = img, img.size
        self.chip = simple.Chip(img.words, depth)
        self.chip.start(State.thread, 0)
        self.env = simple._Callback()


simple.Run = ThreadRun          # before extra.py and i2c_eeprom.py import it

import rig                                              # noqa: E402

CLK = rig.CLK


def trace_of(chip):
    """The record as a rig.Trace, so the benches' own decoders can read it."""
    t = rig.Trace()
    t.t, t.v, t.end = list(chip.rec_t), list(chip.rec_v), chip.c
    return t


def record(chip):
    return list(zip(chip.rec_t, chip.rec_v)), chip.c


# ---------------------------------------------------------------------------------------------------------------
# VCD files: what a logic analyzer on the board would capture

NAMES = ['uo%d' % i for i in range(8)] + ['ui%d' % i for i in range(8)] + ['uio%d' % i for i in range(8)]
TICKS = 2                       # a 10 ns timescale and two ticks per 20 ns cycle (sigrok takes 1, 10 or 100 units)


def board_levels(v):
    """uo, ui and the uio lines, as one 24-bit word in NAMES order."""
    return (v & 255) | ((v >> 24) & 255) << 8 | ((v >> 32) & 255) << 16


def write_vcd(path, chip, lo=0, hi=None):
    """Cycles lo to hi of the record. Sample n of the file is cycle lo + n / TICKS."""
    hi = chip.c if hi is None else hi
    ids = [chr(33 + i) for i in range(len(NAMES))]
    out = ['$timescale 10 ns $end', '$scope module board $end']
    out += ['$var wire 1 %s %s $end' % (ids[i], n) for i, n in enumerate(NAMES)]
    out += ['$upscope $end', '$enddefinitions $end']
    prev, started = 0, False
    for t, v in zip(chip.rec_t, chip.rec_v):
        if t >= hi:
            break
        lv = board_levels(v)
        if t <= lo:                     # the level at cycle lo is the last change at or before it
            prev = lv
            continue
        if not started:
            out.append('#0')
            out += ['%d%s' % ((prev >> i) & 1, ids[i]) for i in range(len(NAMES))]
            started = True
        if lv != prev:
            out.append('#%d' % (TICKS * (t - lo)))
            d = lv ^ prev
            out += ['%d%s' % ((lv >> i) & 1, ids[i]) for i in range(len(NAMES)) if (d >> i) & 1]
            prev = lv
    if not started:
        out.append('#0')
        out += ['%d%s' % ((prev >> i) & 1, ids[i]) for i in range(len(NAMES))]
    out.append('#%d' % (TICKS * (hi - lo)))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as fh:
        fh.write('\n'.join(out) + '\n')
    return path


# ---------------------------------------------------------------------------------------------------------------
# sigrok-cli

ANN = re.compile(r'^(\d+)-(\d+) ([A-Za-z0-9_]+)-(\d+): ([^:]+): (.*)$')
TEXT = re.compile(r'"((?:[^"\\]|\\.)*)"')
SIGROK = shutil.which('sigrok-cli')


class Ann:
    __slots__ = ('ss', 'es', 'pd', 'cls', 'texts')

    def __init__(self, ss, es, pd, cls, texts):
        self.ss, self.es, self.pd, self.cls, self.texts = ss, es, pd, cls, texts

    @property
    def text(self):
        return self.texts[0] if self.texts else ''

    @property
    def cycle(self):
        return self.ss // TICKS

    def __repr__(self):
        return '%d %s.%s %r' % (self.ss // TICKS, self.pd, self.cls, self.text)


class Decoded:
    def __init__(self, cmd, anns, problems):
        self.cmd, self.anns, self.problems = cmd, anns, problems

    def of(self, pd=None, *classes):
        return [a for a in self.anns if (pd is None or a.pd == pd) and (not classes or a.cls in classes)]


def stack_arg(stack):
    """[(decoder, {channel: VCD name}, {option: value})] to sigrok-cli's -P argument."""
    parts = []
    for pd, ch, opt in stack:
        parts.append(':'.join([pd] + ['%s=%s' % kv for kv in list(ch.items()) + list(opt.items())]))
    return ','.join(parts)


def decode(vcd, stack):
    """Runs one decoder stack over a VCD file. Annotations of every decoder in the stack, with their classes (the
    debug log level prints them). A non-zero exit, a Python traceback in a decoder, or a channel name sigrok
    didn't find is a problem: sigrok-cli maps an unknown channel to another one and carries on."""
    cmd = [SIGROK, '-l', '4', '-i', vcd, '-P', stack_arg(stack), '-A', ','.join(pd for pd, _, _ in stack)]
    p = subprocess.run(cmd, capture_output=True, text=True)
    anns = []
    for line in p.stdout.splitlines():
        m = ANN.match(line)
        if m:
            anns.append(Ann(int(m.group(1)), int(m.group(2)), m.group(3), m.group(5), TEXT.findall(m.group(6))))
    problems = []
    if p.returncode:
        problems.append('sigrok-cli exit %d: %s' % (p.returncode, p.stderr.strip().splitlines()[-1:]))
    for line in p.stderr.splitlines():
        if 'Traceback' in line or 'No channel with name' in line or re.search(r'srd: \w*Error', line):
            problems.append(line.strip())
    return Decoded(' '.join(cmd[1:]), anns, problems)


# ---------------------------------------------------------------------------------------------------------------
# results

class Row:
    """One program's line of the table. ok[mode] counts decoded runs that passed and failed."""

    def __init__(self, prog):
        self.prog = prog
        self.stacks = []
        self.scenarios = 0
        self.items = 0
        self.unit = ''
        self.ok = {'sim': [0, 0], 'rtl': [0, 0]}
        self.secs = 0.0
        self.note = ''


class Log:
    def __init__(self):
        self.rows = {}
        self.checks = []                # (programs, ok, name)
        self.diffs = []                 # (programs, VCD file, reason, detail): known differences
        self.progs = ()
        self.mode = 'sim'
        self.t0 = time.time()
        self.t_group = None

    def row(self, prog):
        if prog not in self.rows:
            self.rows[prog] = Row(prog)
        return self.rows[prog]

    def close_group(self):
        if self.t_group is not None and self.progs:
            dt = time.time() - self.t_group
            for p in self.progs:
                self.row(p).secs += dt / len(self.progs)
        self.t_group = None

    def group(self, *progs, title=''):
        self.close_group()
        self.progs = progs
        self.t_group = time.time()
        for p in progs:
            self.row(p)
        print('--- %s%s' % (', '.join(progs), ': ' + title if title else ''), flush=True)

    def check(self, name, ok, detail='', progs=None):
        ok = bool(ok)
        progs = tuple(progs) if progs else self.progs
        self.checks.append((progs, ok, name))
        d = '' if detail is None or detail == '' else str(detail)
        if len(d) > 500:
            d = d[:500] + ' ...'
        print('%s  %s%s' % ('PASS' if ok else 'FAIL', name, '  ' + d if d else ''), flush=True)
        return ok

    def known(self, vcd, reason, detail, progs=None):
        """A run where sigrok and the stimulus differ in a documented way."""
        self.diffs.append((tuple(progs or self.progs), vcd, reason, detail))

    def outcome(self, mode, ok, progs=None):
        for p in (progs or self.progs):
            self.row(p).ok[mode][0 if ok else 1] += 1

    def info(self, s):
        print('INFO  ' + s, flush=True)


LOG = Log()


class Ctx:
    """What one scenario's run gives its judge: the chips it made, in order, and whatever the scenario returned."""

    def __init__(self, value, chips):
        self.value, self.chips = value, chips

    @property
    def chip(self):
        return self.chips[-1]


class Runner:
    def __init__(self, work, rtl=False):
        self.work, self.rtl = work, rtl
        self.rtl_cls = None
        self.n_vcd = 0

    def run(self, fn, *a, **k):
        """Runs a bench scenario on the chip in use and returns Ctx. Under --rtl it runs on the simulator first, then
        on the RTL, and checks that every chip's record is the same, cycle for cycle."""
        if not self.rtl:
            State.cls, State.chips = SIM_CLS, []
            value = fn(*a, **k)
            return Ctx(value, State.chips)
        State.cls, State.chips = SIM_CLS, []
        fn(*a, **k)
        sim_chips = State.chips
        State.cls, State.chips = self.rtl_cls, []
        try:
            value = fn(*a, **k)
        finally:
            State.cls = SIM_CLS
        ctx = Ctx(value, State.chips)
        ctx.sim_chips = sim_chips
        return ctx

    def vcd(self, ctx, name, chip=None, lo=0, hi=None, sim_side=False):
        chip = chip or ctx.chip
        safe = re.sub(r'[^A-Za-z0-9_.-]+', '_', name)
        path = os.path.join(self.work, 'rtl' if self.rtl and not sim_side else 'sim', safe + '.vcd')
        self.n_vcd += 1
        return write_vcd(path, chip, lo, hi)


def first_difference(a, b):
    if len(a) != len(b):
        return 'the RTL made %d chips, the simulator %d' % (len(b), len(a))
    for k, ((ra, ea), (rb, eb)) in enumerate(zip(a, b)):
        if ea != eb:
            return 'chip %d ran %d cycles on the RTL, %d on the simulator' % (k, eb, ea)
        for i, (x, y) in enumerate(zip(ra, rb)):
            if x != y:
                return 'chip %d, change %d: RTL cycle %d value %#x, simulator cycle %d value %#x' % (
                    k, i, y[0], y[1], x[0], x[1])
        if len(ra) != len(rb):
            return 'chip %d: %d changes on the RTL, %d on the simulator' % (k, len(rb), len(ra))
    return 'no difference found'


# ---------------------------------------------------------------------------------------------------------------

def load_rtl(build):
    if build:
        os.environ['PE_BUILD'] = os.path.abspath(build)
    sys.path.insert(0, os.path.join(ROOT, 'verif'))
    import rtl_chip
    for d in (128, 512):
        rtl_chip._load('stub', d)       # fail early if a model is missing
    return recording(rtl_chip.Chip)


HIDE = {'format'}                # options that only change how sigrok prints


def chains(stacks):
    """The decoder stacks a program used, merged: one entry per chain of decoders, with every value each option
    took."""
    merged = {}
    for st in stacks:
        key = tuple(pd for pd, _, _ in st)
        m = merged.setdefault(key, [{} for _ in st])
        for slot, (pd, ch, opt) in zip(m, st):
            for k, v in opt.items():
                if k not in HIDE:
                    vals = slot.setdefault(k, [])
                    if str(v) not in vals:
                        vals.append(str(v))
    out = []
    for key, m in merged.items():
        parts = []
        for pd, slot in zip(key, m):
            o = ' '.join('%s=%s' % (k, '/'.join(v)) for k, v in slot.items())
            parts.append(pd + (' (%s)' % o if o else ''))
        out.append(' > '.join(parts))
    return '; '.join(out)


ORDER = ('basic', 'standard', 'trimmed', 'extra', 'advanced', 'test', 'teeth')


def table(log):
    key = lambda p: (ORDER.index(p.split('/')[0]) if p.split('/')[0] in ORDER else len(ORDER), p)
    rows = [log.rows[p] for p in sorted(log.rows, key=key)]
    out = ['| Program | sigrok decoders and options | Scenarios | Decoded | Simulator | RTL | Time |',
           '|---|---|---|---|---|---|---|']
    for r in rows:
        def res(m):
            p, f = r.ok[m]
            if p + f == 0:
                return '-'
            return 'PASS %d' % p if not f else 'FAIL %d of %d' % (f, p + f)
        chain = chains(r.stacks) or r.note
        out.append('| `%s` | %s | %d | %s | %s | %s | %.0f s |' % (
            r.prog, chain, r.scenarios, ('%d %s' % (r.items, r.unit)) if r.unit else '-', res('sim'), res('rtl'),
            r.secs))
    return '\n'.join(out)


def main(argv):
    args = list(argv)
    opt = {'--build': None, '--work': os.path.join(HERE, 'work'), '--json': None}
    flags = set()
    picks = []
    i = 0
    while i < len(args):
        a = args[i]
        if a in opt:
            opt[a] = args[i + 1]
            i += 2
            continue
        if a.startswith('--'):
            flags.add(a)
        else:
            picks.append(a)
        i += 1
    import cases
    groups = cases.GROUPS
    if '--list' in flags:
        for name, fn in groups:
            print(name)
        return 0
    if SIGROK is None:
        print('SKIP  sigrok-cli is not installed (on Debian or Ubuntu: apt-get install sigrok-cli); nothing checked')
        return 0
    v = subprocess.run([SIGROK, '--version'], capture_output=True, text=True).stdout
    m = re.search(r'libsigrokdecode (\S+)', v)
    rtl = '--rtl' in flags
    runner = Runner(os.path.abspath(opt['--work']), rtl)
    if rtl:
        runner.rtl_cls = load_rtl(opt['--build'])
    LOG.mode = 'rtl' if rtl else 'sim'
    LOG.info('sigrok-cli %s, libsigrokdecode %s; chip: %s; VCD files in %s' % (
        v.split()[1] if v else '?', m.group(1) if m else '?',
        'the RTL, checked against sim/sim.py' if rtl else 'sim/sim.py', runner.work))
    run = [(n, fn) for n, fn in groups if not picks or any(p in n for p in picks)]
    crashed = []
    for name, fn in run:
        try:
            fn(runner, LOG)
        except Exception:
            crashed.append(name)
            traceback.print_exc()
            LOG.check('group %s crashed' % name, False, 'see the traceback above')
            LOG.outcome(LOG.mode, False)
    LOG.close_group()
    print()
    print('=== decoded runs per program (in an image with several programs, a decode counts for the program whose '
          'bus it is)')
    print(table(LOG))
    n_ok = sum(ok for _, ok, _ in LOG.checks)
    n_bad = len(LOG.checks) - n_ok
    if LOG.diffs:
        print('=== runs where sigrok and the stimulus differ in a known way (reported, not failed)')
        by = {}
        for progs, vcd, reason, detail in LOG.diffs:
            by.setdefault((progs, reason), []).append((vcd, detail))
        for (progs, reason), runs in by.items():
            print('DIFF  %s: %s; %d run(s): %s; first: %s' % (', '.join(progs), reason, len(runs),
                                                            ', '.join(v for v, _ in runs[:6]) +
                                                            (' ...' if len(runs) > 6 else ''), runs[0][1][:300]))
    print('=== %d checks: %d PASS, %d FAIL; %d VCD files; %.0f s%s' % (
        len(LOG.checks), n_ok, n_bad, runner.n_vcd, time.time() - LOG.t0,
        '; groups that crashed: ' + ', '.join(crashed) if crashed else ''))
    if opt['--json']:
        rows = {p: dict(chains=chains(r.stacks), scenarios=r.scenarios, items=r.items, unit=r.unit,
                        sim=r.ok['sim'], rtl=r.ok['rtl'], secs=round(r.secs, 1), note=r.note)
                for p, r in LOG.rows.items()}
        with open(opt['--json'], 'w') as fh:
            json.dump(dict(mode=LOG.mode, rows=rows, fail=n_bad, secs=round(time.time() - LOG.t0),
                           checks=[dict(progs=list(p), ok=ok, name=n) for p, ok, n in LOG.checks],
                           diffs=[dict(progs=list(p), vcd=v, reason=r) for p, v, r, _ in LOG.diffs]), fh, indent=1)
    return 1 if n_bad or crashed else 0


if __name__ == '__main__':
    sys.path.insert(0, HERE)
    sys.exit(main(sys.argv[1:]))
