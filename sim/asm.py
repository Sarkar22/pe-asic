#!/usr/bin/env python3
"""Instruction table, decoder and assembler for ISA v0.1.

The table is parsed from docs/ISA.md (section 6.3), so an encoding can't be
mistyped here. Source syntax follows sections 6.2, 6.4, 6.6.
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DOC = os.path.join(HERE, '..', 'docs', 'ISA.md')

# 6.2: operand types written as words. Each word is read as a word only in an operand of its type (6.6).
FLAG_WORD = {'flag': 'inbox', 'lflag': 'last'}      # the word, or nothing
SLOTS = ('TXMODE', 'TXMAP', 'TXPER', 'TXTAIL', 'RXMODE', 'RXPER', 'RXFIRST', 'RXEND', 'RXCNT', 'RXSYNC', 'RXOPT',
         'POLYHI', 'POLYLO', 'CHKHI', 'CHKLO', 'CODING')
WORD_TYPES = {
    'sslot': {w: i for i, w in enumerate(SLOTS)},
    'sdir': {'tx': 1, 'rx': 2, 'txrx': 3},
    'sout': {'-': 0, 'A': 1, 'B': 2},
    'tkind': {'stuff': 1, 'crc': 2, 'fcs': 3},      # nothing is 0
    'sreg': {'lo': 0, 'hi': 1, 'end': 2},
}


class Row:
    def __init__(self, syntax, enc, fields, slots, w, sem):
        self.syntax, self.enc, self.ftypes = syntax, enc, fields
        self.slots, self.w, self.sem = slots, w, sem
        self.mask = self.val = 0
        self.bits = {}                  # letter -> bit positions, MSB first
        for i, ch in enumerate(enc):
            b = 15 - i
            if ch in '01':
                self.mask |= 1 << b
                self.val |= int(ch) << b
            else:
                self.bits.setdefault(ch, []).append(b)
        if set(self.bits) != set(fields):
            raise ValueError('field letters differ from encoding: ' + syntax)
        self.has_inbox = '[, inbox]' in syntax
        text = syntax.replace(' [, inbox]', '')
        # other optional operands, `[, k]` or `[, last]`, in the order written (6.6: txw's kind before last)
        self.opt = []                   # ('F', letter) for a field written as a word or left out,
        for o in re.findall(r'\s*\[,\s*([^\]]+?)\s*\]', text):    # ('W', letter, word) for a flag word
            if o in fields:
                self.opt.append(('F', o))
            else:
                k = [x for x, ty in fields.items() if FLAG_WORD.get(ty) == o]
                if len(k) != 1:
                    raise ValueError('optional operand %s matches no field: %s' % (o, syntax))
                self.opt.append(('W', k[0], o))
        text = re.sub(r'\s*\[,\s*[^\]]+\]', '', text)
        parts = text.split(None, 1)
        self.mn = parts[0]
        ops = [o.strip() for o in parts[1].split(',')] if len(parts) > 1 else []
        self.tmpl = []
        for o in ops:
            if len(o) == 2 and o[0] == 'R' and o[1] in fields:
                self.tmpl.append(('R', o[1]))
            elif o in fields:
                self.tmpl.append(('F', o))
            else:
                self.tmpl.append(('L', o))
        self.fn = None                  # handler, set by sim.py


def _cells(line):
    cells = re.split(r'(?<!\\)\|', line)[1:-1]
    return [c.strip().replace('\\|', '|') for c in cells]


def load_table(path=DOC):
    with open(path) as fh:
        text = fh.read()
    body = text.split('<!-- isa-table-begin -->')[1].split('<!-- isa-table-end -->')[0]
    rows = []
    for line in body.strip().splitlines():
        c = _cells(line)
        if c[0] == 'Syntax' or set(c[0]) <= {'-'}:
            continue
        if len(c) != 6:
            raise ValueError('row has %d columns: %s' % (len(c), line))
        syntax, enc = c[0].strip('`'), c[1].strip('`').replace(' ', '')
        if len(enc) != 16:
            raise ValueError('encoding is not 16 chars: ' + line)
        fields = {}
        for item in c[2].strip('`').split():
            k, v = item.split(':')
            fields[k] = v
        rows.append(Row(syntax, enc, fields, c[3], c[4], c[5].strip('`')))
    return rows


ROWS = load_table()
BY_MN = {}
for _r in ROWS:
    BY_MN.setdefault(_r.mn, []).append(_r)


def legal_value(v, ty):
    if ty == 'opin':
        return v != 3
    if ty == 'iopin':
        return 8 <= v <= 15
    if ty == 'lane':
        return v in (0, 4, 5, 6, 7)
    if ty == 'wmode':
        return v in (0, 1, 2)
    if ty == 'sdir':
        return v in (1, 2, 3)
    if ty in ('sout', 'sreg'):
        return v in (0, 1, 2)
    return True                         # every other type: all values


def _extract(word, bits):
    v = 0
    for b in bits:
        v = (v << 1) | ((word >> b) & 1)
    return v


_memo = {}


def decode(word):
    """(row, fields) for a legal word, None for an illegal one (section 10)."""
    if word in _memo:
        return _memo[word]
    hit = None
    for r in ROWS:
        if word & r.mask == r.val:
            f = {k: _extract(word, b) for k, b in r.bits.items()}
            if all(legal_value(f[k], r.ftypes[k]) for k in f):
                if hit is not None:
                    raise ValueError('overlap: %s and %s' % (hit[0].syntax, r.syntax))
                hit = (r, f)
    _memo[word] = hit
    return hit


def encode(row, fields):
    w = row.val
    for k, bits in row.bits.items():
        v = fields[k]
        n = len(bits)
        for i, b in enumerate(bits):
            w |= ((v >> (n - 1 - i)) & 1) << b
    return w


# ---------------------------------------------------------------------------
# assembler

class AsmError(Exception):
    pass


class NoMatch(Exception):
    pass


REGS = {'R%d' % i: i for i in range(8)}
PIN_NAMES = {'uo%d' % i for i in range(8)} | {'ui%d' % i for i in range(8)} | {'io%d' % i for i in range(8)}
WORDS = {'T', 'inbox', 'eq', 'ne', 'chg', 'uo', 'ui', 'io', 'IN', 'PP', 'OD', 'BYP'}
MNEMONICS = set(BY_MN) | {'li', 'ret'}
_tok = re.compile(r'\s*(0x[0-9a-fA-F]+|\d+|[A-Za-z_][A-Za-z_0-9]*|//|<<|>>|[-+*%&|^()])')


class Ctx:
    def __init__(self, depth):
        self.depth = depth
        self.names = {}                 # constants and labels
        self.pins = {}                  # pin aliases
        self.final = False              # last pass: unknown name is an error
        self.seen = set()               # names defined so far in this pass
        self.consts = set()             # the .equ constants among them
        self.equs = set()               # names any .equ defines, kept across passes


_name = re.compile(r'[A-Za-z_][A-Za-z_0-9]*')


def _names_in(expr):
    return _name.findall(re.sub(r'0x[0-9a-fA-F]+', ' ', expr))


def forward_label(expr, ctx):
    """True if the expression uses a label defined later in the source (6.4)."""
    return any(n not in ctx.seen and n not in ctx.equs for n in _names_in(expr))


def check_new_name(name, ctx):
    """6.6: a name can't collide with the language, and is defined once."""
    if name in MNEMONICS or name in REGS or name in PIN_NAMES or name in WORDS:
        raise AsmError('a name can not be a mnemonic, register, pin or reserved word: ' + name)
    if name in ctx.seen:
        raise AsmError('name defined twice: ' + name)


def evaluate(expr, ctx):
    out, pos = [], 0
    while pos < len(expr):
        m = _tok.match(expr, pos)
        if not m:
            raise NoMatch('bad expression: ' + expr)
        pos = m.end()
        t = m.group(1)
        if t[0].isalpha() or t[0] == '_':
            if t in REGS or t in PIN_NAMES or t in ctx.pins or t in WORDS:
                raise NoMatch('not a number: ' + t)
            if t in ctx.names:
                t = str(ctx.names[t])
            elif ctx.final:
                raise AsmError('unknown name: ' + t)
            else:
                t = '0'
        out.append(t)
    if not out:
        raise NoMatch('empty expression')
    try:
        return int(eval(' '.join(out), {'__builtins__': {}}, {}))
    except AsmError:
        raise
    except Exception as e:
        raise AsmError('bad expression %r: %s' % (expr, e))


def _pin(tok, ctx):
    return ctx.pins.get(tok, tok)


def parse_pin(tok, ty, ctx):
    t = _pin(tok, ctx)
    if t not in PIN_NAMES and t != '-':
        raise NoMatch('not a pin: ' + tok)
    if t == '-':
        if ty == 'opin-':
            return 3
        if ty == 'ipin-':
            return 0
        raise AsmError('"-" is not allowed for a %s operand' % ty)
    bank, n = t[:2], int(t[2])
    if ty in ('opin', 'opin-'):
        if bank == 'ui' or t == 'uo3':
            raise AsmError('%s can not be written as an output' % t)
        return n if bank == 'uo' else 8 + n
    if ty in ('ipin', 'ipin-'):
        if bank == 'uo':
            raise AsmError('%s can not be read' % t)
        if ty == 'ipin-' and t == 'ui0':
            raise AsmError('ui0 can not be a shift input, field 0 means no pin')
        return n if bank == 'ui' else 8 + n
    if ty == 'iopin':
        if bank != 'io':
            raise AsmError('%s operand must be io0..io7, not %s' % (ty, t))
        return 8 + n
    if ty == 'lane':
        if t not in ('uo0', 'uo4', 'uo5', 'uo6', 'uo7'):
            raise AsmError('%s is not a bypass lane' % t)
        return n
    raise NoMatch(ty)


def parse_field(tok, ty, ctx, bits):
    if ty in ('opin', 'opin-', 'ipin', 'ipin-', 'iopin', 'lane'):
        return parse_pin(tok, ty, ctx)
    if ty in WORD_TYPES:                # 6.6: such an operand takes nothing else
        if tok not in WORD_TYPES[ty]:
            raise NoMatch('%s is not one of %s' % (tok, ', '.join(WORD_TYPES[ty])))
        return WORD_TYPES[ty][tok]
    if ty == 'nbits':                   # 6.2: written 1..16, the field holds n mod 16
        v = evaluate(tok, ctx)
        if ctx.final and not 1 <= v <= 16:
            raise AsmError('bit count %d is not 1 to 16' % v)
        return v % 16
    if ty == 'obank':
        return {'uo': 0, 'io': 1}[tok] if tok in ('uo', 'io') else _nomatch(tok)
    if ty == 'ibank':
        return {'ui': 0, 'io': 1}[tok] if tok in ('ui', 'io') else _nomatch(tok)
    if ty == 'wmode':
        return {'eq': 0, 'ne': 1, 'chg': 2}[tok] if tok in ('eq', 'ne', 'chg') else _nomatch(tok)
    if ty == 'eact':                    # 6.2: the four spellings, or a constant equal to 0 or 1
        v = {'0': 0, '1': 1, 'T': 2, '!T': 3}.get(tok)
        if v is not None:
            return v
        v = evaluate(tok, ctx)
        if v not in (0, 1):
            raise AsmError('pin value %s is %d, not 0 or 1' % (tok, v))
        return v
    v = evaluate(tok, ctx)
    if ty == 'addr':
        if ctx.final and not 0 <= v < ctx.depth:
            raise AsmError('address %d does not fit DEPTH %d' % (v, ctx.depth))
        return v
    n = int(ty[1:])                     # u1 .. u12
    if ctx.final and not 0 <= v < (1 << n):
        raise AsmError('value %d does not fit %s' % (v, ty))
    return v & ((1 << n) - 1)


def _nomatch(tok):
    raise NoMatch('operand ' + tok)


def literal_ok(name, tok, ctx):
    """A literal in the syntax column. A literal 0 or 1 can be a constant expression (6.2)."""
    if tok == name:
        return True
    if name not in ('0', '1'):
        return False
    try:
        return evaluate(tok, ctx) == int(name)
    except NoMatch:
        return False


def match_row(row, ops, ctx):
    toks = list(ops)
    f = {}
    if row.has_inbox:
        f['m'] = 0
        if toks and toks[-1] == 'inbox':
            f['m'] = 1
            toks.pop()
    for o in reversed(row.opt):         # from the end, so a later optional operand is taken first
        name = o[1]
        f[name] = 0                     # left out
        if len(toks) > len(row.tmpl):
            words = {o[2]: 1} if o[0] == 'W' else WORD_TYPES[row.ftypes[name]]
            if toks[-1] in words:
                f[name] = words[toks.pop()]
    if len(toks) != len(row.tmpl):
        raise NoMatch('operand count')
    for (kind, name), tok in zip(row.tmpl, toks):   # literals pick the row first
        if kind == 'L' and not literal_ok(name, tok, ctx):
            raise NoMatch('literal ' + name)
    for (kind, name), tok in zip(row.tmpl, toks):
        if kind == 'L':
            continue
        if kind == 'R':
            if tok not in REGS:
                raise NoMatch('register')
            v = REGS[tok]
            if row.ftypes[name] == 'reg4' and v > 3:
                raise AsmError('only R0..R3 can be shifted')
            f[name] = v
        else:
            f[name] = parse_field(tok, row.ftypes[name], ctx, len(row.bits[name]))
    return encode(row, f)


def assemble_stmt(mn, ops, ctx):
    """Returns a list of words."""
    if mn == 'li':
        if len(ops) != 2 or ops[0] not in REGS:
            raise AsmError('li needs a register and a value')
        v = evaluate(ops[1], ctx) if ctx.final else evaluate(ops[1], ctx)
        if ctx.final and not 0 <= v <= 0xFFFF:
            raise AsmError('li value out of range')
        r = BY_MN['ldi'][0]
        words = [encode(r, {'n': REGS[ops[0]], 'i': v & 255})]
        if (v >> 8) & 255 or forward_label(ops[1], ctx):    # 6.4: size must not depend on a later label
            r = BY_MN['lui'][0]
            words.append(encode(r, {'n': REGS[ops[0]], 'i': (v >> 8) & 255}))
        return words
    if mn == 'ret':
        return [encode(BY_MN['jr'][0], {'n': 7})]
    if mn == 'tick' and len(ops) == 1 and ops[0] in REGS:
        return assemble_stmt('edge', ['-', '0', ops[0]], ctx)
    if mn not in BY_MN:
        raise AsmError('unknown mnemonic: ' + mn)
    last = None
    for r in BY_MN[mn]:
        try:
            return [match_row(r, ops, ctx)]
        except NoMatch as e:
            last = e
    raise AsmError('no form of %s takes these operands: %s (%s)' % (mn, ', '.join(ops), last))


class Image:
    def __init__(self, words, labels, listing, warnings=()):
        self.words, self.labels, self.listing = words, labels, listing
        self.warnings = list(warnings)

    @property
    def size(self):
        return len(self.words)


def _lines(src):
    for no, line in enumerate(src.splitlines(), 1):
        yield no, line.split(';', 1)[0].strip()


def _written_pin(word):
    """The output pin a word writes, or None. pcfg counts as a claim, not a write."""
    d = decode(word)
    if d is None:
        return None
    row, f = d
    if row.mn in ('put', 'oe', 'smap'):      # 6.6: the warning also covers smap
        return f['p']
    if row.mn == 'edge':
        return None if f['p'] == 3 else f['p']
    if row.mn in ('shl8', 'shl16', 'shr8', 'shr16'):
        return None if f['o'] == 3 else f['o']
    return None


def _pin_name(p):
    return 'uo%d' % p if p < 8 else 'io%d' % (p - 8)


def _one_pass(src, ctx, base, defines):
    pc = base
    words, listing = {}, []
    stack = []                          # (.if state): (taken, seen_else)
    labels = {}
    ctx.seen = set(defines)
    ctx.consts = set(k for k, v in defines.items() if not isinstance(v, str))
    claimed, warned = set(), []
    for no, line in _lines(src):
        try:
            if not line:
                continue
            active = all(s[0] for s in stack)
            head = line.split(None, 1)
            if head[0] == '.if':
                if active:
                    late = [n for n in _names_in(head[1]) if n not in ctx.consts]
                    if late:
                        raise AsmError('.if can use only earlier constants, not ' + late[0])
                stack.append([evaluate(head[1], ctx) != 0 if active else False, False, active])
                continue
            if head[0] == '.else':
                s = stack[-1]
                if s[1]:
                    raise AsmError('second .else')
                s[1] = True
                s[0] = s[2] and not s[0]
                continue
            if head[0] == '.endif':
                stack.pop()
                continue
            if not active:
                continue
            m = re.match(r'([A-Za-z_][A-Za-z_0-9]*)\s*:\s*(.*)$', line)
            while m:
                name = m.group(1)
                check_new_name(name, ctx)
                labels[name] = pc
                ctx.names[name] = pc
                ctx.seen.add(name)
                line = m.group(2).strip()
                m = re.match(r'([A-Za-z_][A-Za-z_0-9]*)\s*:\s*(.*)$', line)
            if not line:
                continue
            head = line.split(None, 1)
            if head[0] == '.equ':
                name, expr = head[1].split(None, 1)
                if name in defines:
                    continue            # bench override
                check_new_name(name, ctx)
                expr = expr.strip()
                if expr in PIN_NAMES:
                    ctx.pins[name] = expr
                elif expr in ctx.pins:
                    ctx.pins[name] = ctx.pins[expr]
                else:
                    late = [n for n in _names_in(expr) if n not in ctx.consts]
                    if late:
                        raise AsmError('.equ can use only earlier constants, not ' + late[0])
                    ctx.names[name] = evaluate(expr, ctx)
                ctx.seen.add(name)
                ctx.consts.add(name)
                ctx.equs.add(name)
                continue
            if head[0] == '.org':
                pc = base + evaluate(head[1], ctx)
                continue
            if head[0] == '.data':
                v = evaluate(head[1], ctx)
                if ctx.final and not 0 <= v <= 0xFFFF:
                    raise AsmError('.data value %d does not fit 16 bits' % v)
                out = [v & 0xFFFF]
            elif head[0].startswith('.'):
                raise AsmError('unknown directive: ' + head[0])
            else:
                ops = [o.strip() for o in head[1].split(',')] if len(head) > 1 else []
                out = assemble_stmt(head[0], ops, ctx)
            for i, w in enumerate(out):
                if ctx.final and head[0] != '.data':
                    d = decode(w)
                    if d is not None and d[0].mn == 'pcfg':
                        claimed.add(d[1]['p'])
                    p = _written_pin(w)
                    if p is not None and p not in claimed:
                        claimed.add(p)  # one warning per pin
                        warned.append('line %d: %s written before any pcfg of it: dropped unless thread 0 runs this'
                                      % (no, _pin_name(p)))
                if pc in words:
                    raise AsmError('address %d used twice' % pc)
                if ctx.final and pc >= ctx.depth:
                    raise AsmError('address %d does not fit DEPTH %d' % (pc, ctx.depth))
                words[pc] = w
                listing.append((pc, w, line if i == 0 else ''))
                pc += 1
        except (AsmError, NoMatch, IndexError, ValueError) as e:
            raise AsmError('line %d: %s: %s' % (no, line, e))
    if stack:
        raise AsmError('.if without .endif')
    ctx.warnings = warned
    return words, labels, listing


def assemble(src, defines=None, base=0, depth=512):
    """defines: names preset by the bench; a matching .equ in the source is skipped."""
    defines = defines or {}
    ctx = Ctx(depth)
    for k, v in defines.items():
        if isinstance(v, str):
            ctx.pins[k] = v
        else:
            ctx.names[k] = v
    prev = None
    for _ in range(6):                  # settle forward references and li sizes
        words, labels, listing = _one_pass(src, ctx, base, defines)
        if labels == prev:
            break
        prev = labels
    ctx.final = True
    words, labels, listing = _one_pass(src, ctx, base, defines)
    return Image(words, labels, listing, ctx.warnings)


def read_program(path, **kw):
    with open(path) as fh:
        return assemble(fh.read(), **kw)


# ---------------------------------------------------------------------------
# disassembler: used only for the round-trip check

def disasm(word):
    d = decode(word)
    if d is None:
        return None
    row, f = d
    ops = []
    for kind, name in row.tmpl:
        if kind == 'L':
            ops.append(name)
        elif kind == 'R':
            ops.append('R%d' % f[name])
        else:
            ops.append(_field_text(f[name], row.ftypes[name]))
    for o in row.opt:                   # a field at 0 is left out
        if f[o[1]]:
            ops.append(o[2] if o[0] == 'W' else _field_text(f[o[1]], row.ftypes[o[1]]))
    if row.has_inbox and f['m']:
        ops.append('inbox')
    return row.mn + (' ' + ', '.join(ops) if ops else '')


def _field_text(v, ty):
    if ty in ('opin', 'opin-'):
        if ty == 'opin-' and v == 3:
            return '-'
        return 'uo%d' % v if v < 8 else 'io%d' % (v - 8)
    if ty in ('ipin', 'ipin-'):
        if ty == 'ipin-' and v == 0:
            return '-'
        return 'ui%d' % v if v < 8 else 'io%d' % (v - 8)
    if ty == 'iopin':
        return 'io%d' % (v - 8)
    if ty == 'lane':
        return 'uo%d' % v
    if ty == 'obank':
        return ['uo', 'io'][v]
    if ty == 'ibank':
        return ['ui', 'io'][v]
    if ty == 'wmode':
        return ['eq', 'ne', 'chg'][v]
    if ty == 'eact':
        return ['0', '1', 'T', '!T'][v]
    if ty in WORD_TYPES:
        return {x: w for w, x in WORD_TYPES[ty].items()}[v]
    if ty == 'nbits':
        return str(v if v else 16)
    return str(v)


if __name__ == '__main__':
    img = read_program(sys.argv[1], depth=int(sys.argv[2]) if len(sys.argv) > 2 else 128)
    for pc, w, text in img.listing:
        print('%3d  %04x  %s' % (pc, w, text))
    print('%d words' % img.size)
    for line in img.warnings:
        print('warning: ' + line, file=sys.stderr)
