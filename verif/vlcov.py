#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Line and toggle coverage of the RTL, from the Verilator models that verif/coverage.sh builds with
--coverage-line and --coverage-toggle.

Counting. Each chip registers its coverage points in its library's Verilator context when it is made, and the
context reads the counters only when it writes them. collector() hooks rtl_chip.Chip so that whenever the last
live chip of a library goes, the counts are written (to /dev/shm if there is one), read back and added up here, and
the context is cleared before the chip is freed. A point's name is its source file, line, column, kind and comment,
without the instance, so the points of the four threads, and of every chip, add up into one.

Point kinds: line (a block of statements), branch (each arm of an if, case or ?:) and toggle (each bit of each
signal, 0 to 1 and 1 to 0, separately).

Usage:
  vlcov.py benches OUT.json [--build DIR] [WORD ...]
        every bench of programs/check.py, with the RTL in place of sim/sim.py (all must pass)
  vlcov.py report IN.json ... [--md FILE] [--list FILE] [--dat FILE]
        adds up count files; prints coverage per module of src/, and lists every point never hit
Standard library only.
"""
import atexit
import ctypes
import gc
import json
import os
import re
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.dont_write_bytecode = True

def parse_dat(path):
    """{name: count} of one Verilator coverage file, the hierarchy left out of each name."""
    out = {}
    with open(path, encoding='utf-8', errors='replace') as fh:
        for line in fh:
            if not line.startswith("C '"):
                continue
            body, n = line[3:].rstrip('\n').rsplit("' ", 1)
            name = '\x01'.join(kv for kv in body.split('\x01') if kv and not kv.startswith('h\x02'))
            out[name] = out.get(name, 0) + int(n)
    return out


def fields(name):
    return dict(kv.split('\x02', 1) for kv in name.split('\x01') if kv)


def add(total, part):
    """Adds part, {name: count}, into total, which may be None."""
    if total is None:
        total = {}
    for k, v in part.items():
        total[k] = total.get(k, 0) + v
    return total


def write(path, counts, note=''):
    with open(path, 'w') as fh:
        json.dump({'note': note, 'points': counts}, fh)


def read(path):
    with open(path) as fh:
        return json.load(fh)['points']


# ---- counting in a process ------------------------------------------------------------------------------------
class Collector:
    def __init__(self, rtl_chip):
        self.rc = rtl_chip
        self.live = {}                  # library path: ids of its chips not yet freed
        self.grave = {}                 # library path: chip handles waiting for the count
        self.libs = {}
        self.counts = {}
        d = '/dev/shm' if os.path.isdir('/dev/shm') and os.access('/dev/shm', os.W_OK) else tempfile.gettempdir()
        self.tmp = os.path.join(d, 'pe_cov_%d.dat' % os.getpid())     # one file, written over, removed at exit
        atexit.register(lambda: os.path.exists(self.tmp) and os.remove(self.tmp))
        Chip = rtl_chip.Chip
        init = Chip.__init__
        col = self

        def __init__(self, *a, **k):
            init(self, *a, **k)
            col.born(self)

        def __del__(self):
            col.release(self)
        Chip.__init__, Chip.__del__ = __init__, __del__

    def born(self, chip):
        lib = chip._lib
        path = lib._name
        if path not in self.libs:
            if not hasattr(lib, 'pe_cov_write'):
                raise RuntimeError('%s is not a coverage model (verif/coverage.sh builds them)' % path)
            lib.pe_cov_write.argtypes, lib.pe_cov_write.restype = [ctypes.c_char_p], None
            lib.pe_cov_clear.argtypes, lib.pe_cov_clear.restype = [], None
            lib.pe_cov_zero.argtypes, lib.pe_cov_zero.restype = [], None
            self.libs[path] = lib
            self.live[path] = set()
            self.grave[path] = []
        self.live[path].add(id(chip))

    def release(self, chip):
        """The chip goes: its counts are taken with the others' when the last chip of its library goes."""
        h = getattr(chip, '_h', None)
        if not h:
            return
        chip._h = None
        path = chip._lib._name
        self.live[path].discard(id(chip))
        self.grave[path].append(h)
        if not self.live[path]:
            self.take(path, clear=True)

    def take(self, path, clear):
        lib = self.libs[path]
        lib.pe_cov_write(self.tmp.encode())
        add(self.counts, parse_dat(self.tmp))
        if clear:
            lib.pe_cov_clear()
            for h in self.grave[path]:
                lib.pe_free(h)
            self.grave[path] = []
        else:
            lib.pe_cov_zero()

    def harvest_all(self):
        """Everything counted so far in this process, and starts again from 0."""
        gc.collect()
        for path in self.libs:
            if self.live[path] or self.grave[path]:
                self.take(path, clear=not self.live[path])
        out, self.counts = self.counts, {}
        return out


_collector = None


def collector():
    global _collector
    if _collector is None:
        sys.path.insert(0, HERE)
        import rtl_chip
        _collector = Collector(rtl_chip)
    return _collector


# ---- the program benches on the RTL ----------------------------------------------------------------------------
def benches(argv):
    """programs/check.py with rtl_chip.Chip in place of sim.Chip: the benches build their chips through
    report.Chip. Every check must pass, as on the simulator."""
    out = argv[0]
    rest = argv[1:]
    if '--build' in rest:
        i = rest.index('--build')
        os.environ['PE_BUILD'] = os.path.abspath(rest[i + 1])
        del rest[i:i + 2]
    col = collector()
    import rtl_chip
    sys.path.insert(0, os.path.join(ROOT, 'programs', 'bench'))
    sys.path.insert(0, os.path.join(ROOT, 'programs'))
    import report
    report.Chip = rtl_chip.Chip
    import check
    rc = check.main(rest)
    write(out, col.harvest_all(), 'programs/check.py on the RTL, exit %d' % rc)
    return rc


# ---- the report ------------------------------------------------------------------------------------------------
RTL_FILES = ('pe_core', 'pe_thread', 'pe_exec', 'pe_pins', 'pe_shift', 'pe_host', 'pe_spi', 'pe_mailbox',
             'pe_regfile', 'pe_mem', 'pe_decode', 'pe_pkg')
TOGGLE_BIT = re.compile(r'^(.*?)(\[\d+\])?:(0->1|1->0)$')


def report(argv):
    md = lst = dat = None
    files = []
    it = iter(argv)
    for x in it:
        if x == '--md':
            md = next(it)
        elif x == '--list':
            lst = next(it)
        elif x == '--dat':
            dat = next(it)
        else:
            files.append(x)
    total = None
    for f in files:
        total = add(total, read(f))
    pts = []
    for name, n in total.items():
        fd = fields(name)
        mod = os.path.splitext(os.path.basename(fd.get('f', '')))[0]
        if os.path.basename(os.path.dirname(fd.get('f', ''))) != 'src' or mod not in RTL_FILES:
            continue                    # the stand-ins of verif/ and Verilator's own library
        pts.append((mod, fd, n))
    src = {}

    def text(path, line):
        if path not in src:
            try:
                with open(path) as fh:
                    src[path] = fh.read().splitlines()
            except OSError:
                src[path] = []
        ls = src[path]
        return ls[line - 1].strip() if 0 < line <= len(ls) else ''

    rows = []
    for mod in RTL_FILES:
        p = [x for x in pts if x[0] == mod]
        if not p:
            continue
        r = [mod]
        for kind in ('line', 'branch', 'toggle'):
            k = [x for x in p if x[1]['t'] == kind]
            r += [sum(1 for x in k if x[2]), len(k)]
        rows.append(r)

    def pct(a, b):
        return '%.1f%%' % (100.0 * a / b) if b else '-'
    tab = ['| Module | Line points hit | Branch points hit | Toggle points hit |', '|---|---|---|---|']
    tot = [0] * 6
    for r in rows:
        tab.append('| `%s` | %d of %d (%s) | %d of %d (%s) | %d of %d (%s) |'
                   % (r[0], r[1], r[2], pct(r[1], r[2]), r[3], r[4], pct(r[3], r[4]), r[5], r[6], pct(r[5], r[6])))
        tot = [a + b for a, b in zip(tot, r[1:])]
    tab.append('| all | %d of %d (%s) | %d of %d (%s) | %d of %d (%s) |'
               % (tot[0], tot[1], pct(tot[0], tot[1]), tot[2], tot[3], pct(tot[2], tot[3]), tot[4], tot[5],
                  pct(tot[4], tot[5])))
    listing = ['Line and branch points never hit (file:line, column, kind: source line):']
    for mod, fd, n in sorted(pts, key=lambda x: (x[0], int(x[1]['l']), int(x[1].get('n', 0)))):
        if n == 0 and fd['t'] in ('line', 'branch'):
            listing.append('  %s.sv:%s col %s %s: %s' % (mod, fd['l'], fd.get('n', '?'), fd.get('o', ''),
                                                         text(fd['f'], int(fd['l']))))
    listing.append('Toggle points never hit, by signal (bits that never rose, bits that never fell):')
    sig = {}
    for mod, fd, n in pts:
        if fd['t'] != 'toggle':
            continue
        m = TOGGLE_BIT.match(fd.get('o', ''))
        if not m:
            continue
        key = (mod, m.group(1))
        s = sig.setdefault(key, {'0->1': [], '1->0': [], 'width': set()})
        bit = m.group(2)[1:-1] if m.group(2) else '0'
        s['width'].add(bit)
        if n == 0:
            s[m.group(3)].append(int(bit))
    for (mod, name), s in sorted(sig.items()):
        if s['0->1'] or s['1->0']:
            listing.append('  %s %s (%d bits): never 0->1 %s; never 1->0 %s'
                           % (mod, name, len(s['width']), ranges(s['0->1']), ranges(s['1->0'])))
    print('\n'.join(tab))
    print('%d line and branch points never hit; %d signals with a bit that never toggled'
          % (sum(1 for x in pts if x[2] == 0 and x[1]['t'] != 'toggle'),
             sum(1 for s in sig.values() if s['0->1'] or s['1->0'])))
    if md:
        with open(md, 'w') as fh:
            fh.write('\n'.join(tab) + '\n')
    if lst:
        with open(lst, 'w') as fh:
            fh.write('\n'.join(listing) + '\n')
    if dat:                             # for verilator_coverage --annotate
        with open(dat, 'w') as fh:
            fh.write('# SystemC::Coverage-3\n')
            for name, n in sorted(total.items()):
                fh.write("C '\x01%s' %d\n" % (name, n))
    return 0


def ranges(bits):
    bits = sorted(set(bits))
    if not bits:
        return '-'
    out, a = [], bits[0]
    for x, y in zip(bits, bits[1:] + [None]):
        if y != x + 1:
            out.append(str(a) if a == x else '%d..%d' % (a, x))
            a = y
    return ','.join(out)


if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else ''
    if cmd == 'benches':
        sys.exit(benches(sys.argv[2:]))
    if cmd == 'report':
        sys.exit(report(sys.argv[2:]))
    print(__doc__)
    sys.exit(2)
