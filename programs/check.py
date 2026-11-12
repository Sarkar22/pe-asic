#!/usr/bin/env python3
"""Runs every program under programs/ on sim/sim.py and checks what it does at the pads.

Usage, from the repository root:
    python3 programs/check.py            all benches
    python3 programs/check.py uart can   only the benches whose names contain one of the words

Prints one PASS or FAIL line per check, then a summary per program. Exits non-zero if any check
fails, a bench crashes, or a program has no check at all. Python 3 standard library only.
"""
import glob
import os
import sys
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, 'bench'))
sys.dont_write_bytecode = True

import report                                           # noqa: E402

# Bench modules in programs/bench/, in the order they run.
BENCHES = ['words', 'basic', 'standard', 'trimmed', 'extra', 'manchester', 'dmx', 'ps2', 'mdio', 'jtag_chain',
           'swd_host', 'i2c_target', 'usb_tx', 'usb_rx', 'usb_dev', 'can', 'bypass']


def main(argv):
    picks = [a for a in argv if not a.startswith('-')]
    run = [b for b in BENCHES if not picks or any(p in b for p in picks)]
    log = report.LOG
    crashed = []
    times = []
    for name in run:
        t = time.time()
        mod = __import__(name)
        start = len(log.rows)
        try:
            mod.main(log)
        except Exception:
            # A crash fails the program being checked, and every program of the bench it didn't reach.
            crashed.append(name)
            traceback.print_exc()
            now = [p for p in log.progs if p in mod.PROGRAMS] or list(mod.PROGRAMS)
            log.group(*now)
            log.check('bench %s crashed while checking this' % name, False, 'see the traceback above')
            seen = {p for progs, _, _ in log.rows[start:] for p in progs}
            for p in mod.PROGRAMS:
                if p not in seen:
                    log.check('%s: not reached, bench %s crashed first' % (p, name), False, progs=(p,))
        times.append((name, time.time() - t))

    programs = sorted(os.path.relpath(p, HERE) for p in glob.glob(os.path.join(HERE, '*', '*.s')))
    count = {p: [0, 0] for p in programs}
    for progs, ok, _ in log.rows:
        for p in progs:
            count.setdefault(p, [0, 0])[0 if ok else 1] += 1
    print()
    print('=== summary: checks per program (a check on a shared image counts for each program in it)')
    print('%5s %5s  %s' % ('PASS', 'FAIL', 'program'))
    for p in sorted(count):
        n_pass, n_fail = count[p]
        note = ''
        if p not in programs:
            note = '  (no such file)'
        elif n_pass + n_fail == 0:
            note = '  (no check ran)' if not picks else ''
        print('%5d %5d  %s%s' % (n_pass, n_fail, p, note))
    n_ok = sum(ok for _, ok, _ in log.rows)
    n_bad = len(log.rows) - n_ok
    unchecked = [p for p in programs if sum(count[p]) == 0] if not picks else []
    missing = [p for p in count if p not in programs]
    print('=== %d checks: %d PASS, %d FAIL; %d programs, %d without a check; %.0f s'
          % (len(log.rows), n_ok, n_bad, len(programs), len(unchecked), time.time() - log.t_start))
    print('    time per bench: ' + ', '.join('%s %.0f s' % x for x in times))
    if crashed:
        print('    benches that crashed: ' + ', '.join(crashed))
    return 1 if n_bad or unchecked or missing or crashed else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
