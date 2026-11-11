#!/usr/bin/env python3
"""Reads the one instruction table out of docs/ISA.md (section 6.3) and turns it into a decoder,
an operand description and executable semantics. Nothing here knows any instruction by name: the
table is the only source. sim2.py, docsim.py and asm2.py use it; sim.py and asm.py don't. The
document path can be overridden with ISA_DOC.

python3 sim/isatable.py prints the number of rows and of legal words, and any overlap between rows."""
import os, re

HERE = os.path.dirname(os.path.abspath(__file__))
DOC = os.environ.get('ISA_DOC', os.path.join(HERE, '..', 'docs', 'ISA.md'))

# operand types: legal raw field values. A word with any other value in the field is illegal.
LEGAL = {
    'opin': set(range(16)) - {3},       # output pin; 3 is the host data-out pin
    'opin-': set(range(16)),            # output pin, or 3 = no pin
    'ipin': set(range(16)),
    'ipin-': set(range(16)),            # input pin, or 0 = no pin
    'iopin': set(range(8, 16)),
    'lane': {0, 4, 5, 6, 7},
    'wmode': {0, 1, 2},
    'sdir': {1, 2, 3},                  # 6.2: sgo and sstop with a direction of 0 are illegal (section 10)
    'sout': {0, 1, 2},                  # smap with output value 3 is illegal
    'sreg': {0, 1, 2},                  # srd with a register of 3 is illegal
}
ENUM = {
    'obank': {'uo': 0, 'io': 1},
    'ibank': {'ui': 0, 'io': 1},
    'wmode': {'eq': 0, 'ne': 1, 'chg': 2},
    'eact': {'0': 0, '1': 1, 'T': 2, '!T': 3},
    'sslot': {w: i for i, w in enumerate(('TXMODE', 'TXMAP', 'TXPER', 'TXTAIL', 'RXMODE', 'RXPER', 'RXFIRST', 'RXEND',
                                          'RXCNT', 'RXSYNC', 'RXOPT', 'POLYHI', 'POLYLO', 'CHKHI', 'CHKLO', 'CODING'))},
    'sdir': {'tx': 1, 'rx': 2, 'txrx': 3},
    'sout': {'-': 0, 'A': 1, 'B': 2},
    'sreg': {'lo': 0, 'hi': 1, 'end': 2},
}
OPT_WORDS = {                           # 6.2: types written as a word or as nothing, which is 0
    'flag': {'inbox': 1},
    'lflag': {'last': 1},
    'tkind': {'stuff': 1, 'crc': 2, 'fcs': 3},
}
OUT_PIN = {f'uo{i}': i for i in range(8)} | {f'io{i}': 8 + i for i in range(8)}
IN_PIN = {f'ui{i}': i for i in range(8)} | {f'io{i}': 8 + i for i in range(8)}


class Row:
    def __init__(self, syntax, enc, fields, slots, world, sem):
        self.syntax, self.slots, self.world, self.sem = syntax, slots, world, sem
        bits = enc.replace(' ', '')
        if len(bits) != 16:
            raise ValueError(f'{syntax}: encoding has {len(bits)} bits')
        self.bits = bits
        self.mask = self.match = 0
        self.fields = {}                # letter -> (lsb, width)
        for i, ch in enumerate(bits):
            b = 15 - i
            if ch in '01':
                self.mask |= 1 << b
                self.match |= int(ch) << b
            else:
                lo, w = self.fields.get(ch, (b, 0))
                self.fields[ch] = (b, w + 1)
        self.types = dict(f.split(':') for f in fields.split()) if fields else {}
        if set(self.types) != set(self.fields):
            raise ValueError(f'{syntax}: fields {self.fields} types {self.types}')
        self.letters = sorted(self.fields)
        m = re.match(r'(\S+)\s*(.*)$', syntax)
        self.mnem, rest = m[1], m[2]
        self.opt_inbox = '[, inbox]' in rest
        rest = rest.replace('[, inbox]', '').strip()
        # other optional operands, in the order written: txw's `[, k] [, last]` (6.6: the kind before last)
        self.opt = re.findall(r'\[,\s*([^\]]+?)\s*\]', rest)
        rest = re.sub(r'\s*\[,\s*[^\]]+\]', '', rest).strip()
        self.ops = [o.strip() for o in rest.split(',')] if rest else []

    def field_values(self, w):
        return tuple((w >> self.fields[l][0]) & ((1 << self.fields[l][1]) - 1) for l in self.letters)

    def legal(self, w):
        for l, v in zip(self.letters, self.field_values(w)):
            ty = self.types[l]
            if ty in LEGAL and v not in LEGAL[ty]:
                return False
        return True


def parse(doc=None):
    text = open(doc or DOC).read()
    body = text.split('<!-- isa-table-begin -->')[1].split('<!-- isa-table-end -->')[0]
    rows = []
    for line in body.splitlines():
        if not line.startswith('|') or line.startswith('|---') or line.startswith('| Syntax'):
            continue
        cells = [c.strip() for c in re.split(r'(?<!\\)\|', line)[1:-1]]
        cells = [c.replace('\\|', '|').strip('`') for c in cells]
        rows.append(Row(*cells))
    return rows


def unit_state(doc=None):
    """The shift unit's state names, read from the tables of section 5.9.2: (per-thread names, global names).
    `CFG[0..15]` is the name CFG. Both simulators keep a thread's names as attributes of `thread.u`."""
    text = open(doc or DOC).read()
    if '#### 5.9.2 State' not in text:
        return [], []
    sec = text.split('#### 5.9.2 State', 1)[1].split('\n#### ', 1)[0]
    per, glob_ = sec.split('**Global state:**', 1)
    out = []
    for part in (per, glob_):
        names = []
        for line in part.splitlines():
            cells = [c.strip() for c in line.strip().strip('|').split('|')]
            if len(cells) != 3 or cells[0] in ('State', '') or set(cells[0]) <= {'-'}:
                continue
            for n in cells[0].split(','):
                n = re.sub(r'\[.*\]$', '', n.strip())
                assert re.fullmatch(r'[A-Z][A-Z0-9]*', n), (n, line)
                names.append(n)
        out.append(names)
    return out[0], out[1]


UNIT, UNIT_GLOBAL = unit_state()
GLOBAL_ATTR = {'SYNC3': 'sync3', 'SEL': 'sel', 'SHSEL': 'shsel'}   # both simulators' chip attributes


# ---- semantics column -> Python ------------------------------------------------------------
ASSIGN = re.compile(r'^(R[a-z0-9]|T|D|PC|LATE|f|FULL\[m\]|M\[m\]) := (.+)$')
HELPERS = ('shadow', 'shadow_oe', 'put8', 'oe8', 'pcfg_in', 'pcfg_pp', 'pcfg_od', 'pcfg_byp',
           'tx_push', 'rx_pop', 'sstat', 'srd', 'set_cfg', 'sgo', 'sstop', 'shadow_sel')     # the 6.1 helper table


def expr(s):
    s = re.sub(r'\bR([a-z])\b', r'_r[\1]', s)
    s = re.sub(r'\bR7\b', '_r[7]', s)
    for name, py in (('T', '_t.T'), ('D', '_t.D'), ('S', '_t.S'), ('PC', '_t.pc'), ('LATE', '_t.late'),
                     ('TB', '(_c & 0xFFFF)'), ('TID', '_t.id'), ('FULL', '_m.full'), ('M', '_m.mail'),
                     ('SHD', '_m.sh_out'), ('DEPTH', '_m.size')):
        s = re.sub(rf'\b{name}\b', py, s)
    if UNIT:                            # 5.9.2: a name of the thread's unit, then the global ones
        s = re.sub(r'\b(%s)\b' % '|'.join(UNIT), r'_t.u.\1', s)
        s = re.sub(r'\b(%s)\b' % '|'.join(UNIT_GLOBAL), lambda m: '_m.' + GLOBAL_ATTR[m[1]], s)
    s = re.sub(r'\bbit\(', '_bit(', s)
    return s


def stmt(s):
    """One semantics statement to one line of Python."""
    s = s.strip()
    if s == 'pass':
        return 'pass'
    m = re.match(r'if (.+?): (.+)$', s)
    if m:
        return f'if {expr(m[1])}: {stmt(m[2])}'
    m = re.match(r'need\((.+)\)$', s)
    if m:
        return f'if not ({expr(m[1])}): return _RETRY'
    m = ASSIGN.match(s)
    if m:
        lhs, rhs = m[1], expr(m[2])
        if lhs == 'PC':
            return f'_npc = ({rhs}) % _m.size'
        if lhs == 'T':
            return f'_t.T = int({rhs}) & 1'
        if lhs == 'f':
            return f'f = int({rhs})'
        if lhs == 'D':
            return f'_m.set_d(_t, ({rhs}) & 0xFFFF)'
        if lhs == 'LATE':
            return f'_t.late = int({rhs})'
        if lhs.startswith('R'):
            return f'{expr(lhs)} = ({rhs}) & 0xFFFF'
        if lhs.startswith('FULL'):
            return f'_m.full[m] = int({rhs})'
        return f'_m.mail[m] = ({rhs}) & 0xFFFF'
    m = re.match(r'(%s)\((.*)\)$' % '|'.join(HELPERS), s)
    if m:
        return f'_m.{m[1]}(_t, {expr(m[2])})' if m[2].strip() else f'_m.{m[1]}(_t)'
    if s == 'halt()':
        return '_m.halt(_t)'
    if s == 'now()':
        return '_m.now(_t, _c)'
    m = re.match(r'arm\((.+)\)$', s)
    if m:
        return f'_m.arm(_t, _c, {expr(m[1])})'
    m = re.match(r'(load|store)\((.+)\)$', s)
    if m:
        return f'_npc = None; _m.{m[1]}(_t, {expr(m[2])})'
    m = re.match(r'wait\((.+)\)$', s)
    if m:
        return f'_npc = None; _m.wait(_t, _c, {expr(m[1])})'
    raise ValueError('cannot translate: ' + s)


def split_statements(sem):
    parts, depth, cur = [], 0, ''
    for ch in sem:
        depth += ch in '(['
        depth -= ch in ')]'
        if ch == ';' and depth == 0:
            parts.append(cur)
            cur = ''
        else:
            cur += ch
    parts.append(cur)
    return parts


def compile_rows(rows):
    env = {'_RETRY': 'retry', '_bit': lambda x, n: (x >> n) & 1}
    for k, row in enumerate(rows):
        args = ', '.join(['_m', '_t', '_c'] + row.letters)
        body = [stmt(s) for s in split_statements(row.sem)]
        src = f'def _f({args}):\n    _r = _t.r\n    _npc = (_t.pc + 1) % _m.size\n'
        src += ''.join(f'    {b}\n' for b in body) + '    return _npc\n'
        exec(src, env)
        row.fn = env['_f']
        row.index = k


class Decoder:
    def __init__(self, rows):
        self.rows, self.cache = rows, {}
        self.groups = {}
        for r in rows:
            top = (r.match >> 12) & 15 if (r.mask >> 12) == 15 else None
            self.groups.setdefault(top, []).append(r)

    def __call__(self, w):
        """(row, field tuple) or None for an illegal word."""
        d = self.cache.get(w, 0)
        if d != 0:
            return d
        hit = [r for r in self.groups.get(w >> 12, []) if (w & r.mask) == r.match and r.legal(w)]
        if len(hit) > 1:
            raise ValueError(f'{w:04x} matches {[r.syntax for r in hit]}')
        d = (hit[0], hit[0].field_values(w)) if hit else None
        self.cache[w] = d
        return d


ROWS = parse()
compile_rows(ROWS)
DECODE = Decoder(ROWS)

if __name__ == '__main__':
    legal = {}
    overlap = 0
    for w in range(65536):
        hit = [r for r in ROWS if (w & r.mask) == r.match and r.legal(w)]
        overlap += len(hit) > 1
        if hit:
            legal[hit[0].syntax] = legal.get(hit[0].syntax, 0) + 1
    n = sum(legal.values())
    print(f'{len(ROWS)} rows, {n} legal words, {65536 - n} illegal ({(65536 - n) / 655.36:.1f}%), {overlap} overlaps')
    print('rows with no legal word:', [r.syntax for r in ROWS if r.syntax not in legal] or 'none')
    print('source:', os.path.normpath(DOC))
