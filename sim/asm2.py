#!/usr/bin/env python3
"""ISA v0.1 assembler, driven by the instruction table in the document.
One statement per line, ';' starts a comment, labels end in ':'.
Directives: .equ NAME expr, .org n, .data expr, .if expr / .else / .endif.
Pseudo-instructions: li Rd, imm16 (1 or 2 words), ret (jr R7), tick Rn (edge -, 0, Rn).

A second assembler, written apart from asm.py, on isatable.py's reading of the table. claims.py
checks that the two give the same words for every program under programs/, and that this one
rejects the same mistakes. python3 sim/asm2.py prog.s prints a listing."""
import re, sys
from isatable import ROWS, ENUM, OPT_WORDS, IN_PIN, OUT_PIN

BY_MNEM = {}
for _r in ROWS:
    BY_MNEM.setdefault(_r.mnem, []).append(_r)
REGS = {f'R{i}': i for i in range(8)}
PINS = {f'{b}{i}' for b in ('uo', 'ui', 'io') for i in range(8)}
RESERVED = {'T', 'inbox', 'eq', 'ne', 'chg', 'uo', 'ui', 'io', 'IN', 'PP', 'OD', 'BYP'}


class AsmError(Exception):
    pass


class NoMatch(Exception):
    pass


class Assembler:
    def __init__(self, size=512):
        self.size = size
        self.sym, self.pinsym = {}, {}
        self.words, self.listing = {}, []
        self.fixed = set()
        self.org = 0
        self.defined = set()            # names, each defined once (section 6.6)
        self.li_long = set()            # source lines whose li must take 2 words (section 6.4)

    def new_name(self, name, no):
        if name in BY_MNEM or name in ('li', 'ret') or name in REGS or name in PINS or name in RESERVED:
            raise AsmError(f'line {no}: {name} is a mnemonic, register, pin or reserved word')
        if name in self.defined:
            raise AsmError(f'line {no}: name defined twice: {name}')
        self.defined.add(name)

    def val(self, s, lo=None, hi=None):
        names = dict(self.sym)
        try:
            v = int(eval(s, {'__builtins__': {}}, names))   # own source files only
        except Exception as e:
            raise AsmError(f'bad expression {s!r}: {e}')
        if lo is not None and not lo <= v <= hi:
            raise AsmError(f'{s} = {v} outside {lo}..{hi}')
        return v

    def pin(self, s, table, what):
        name = self.pinsym.get(s, s)
        if name not in table:
            raise AsmError(f'{s} is not usable as {what}')
        return table[name]

    def operand(self, ty, a):
        if ty == 'reg':
            if a not in REGS:
                raise NoMatch
            return REGS[a]
        if ty == 'reg4':
            if a not in REGS:
                raise NoMatch
            if REGS[a] > 3:
                raise AsmError(f'{a}: shifts work on R0..R3 only')
            return REGS[a]
        if a in REGS:
            raise NoMatch
        if ty == 'opin':
            v = self.pin(a, OUT_PIN, 'an output pin')
            if v == 3:
                raise AsmError('uo3 belongs to the host link')
            return v
        if ty == 'opin-':
            if a == '-':
                return 3
            return self.operand('opin', a)
        if ty == 'ipin':
            return self.pin(a, IN_PIN, 'an input pin')
        if ty == 'ipin-':
            if a == '-':
                return 0
            v = self.pin(a, IN_PIN, 'an input pin')
            if v == 0:
                raise AsmError('ui0 cannot be a shift input: its code means "no pin"')
            return v
        if ty == 'iopin':
            v = self.pin(a, OUT_PIN, 'an output pin')
            if v < 8:
                raise AsmError(f'{a}: only uio pins allowed here')
            return v
        if ty == 'lane':
            v = self.pin(a, OUT_PIN, 'an output pin')
            if v not in (0, 4, 5, 6, 7):
                raise AsmError(f'{a} has no bypass lane')
            return v
        if ty == 'eact':
            if a in ('T', '!T'):
                return ENUM['eact'][a]
            return self.val(a, 0, 1)
        if ty in ENUM:
            if a not in ENUM[ty]:
                raise AsmError(f'{a}: expected one of {sorted(ENUM[ty])}')
            return ENUM[ty][a]
        if ty == 'addr':
            return self.val(a, 0, self.size - 1)
        if ty == 'nbits':                       # 6.2: written 1..16, the field holds n mod 16
            return self.val(a, 1, 16) % 16
        m = re.fullmatch(r'u(\d+)', ty)
        if m:
            return self.val(a, 0, (1 << int(m[1])) - 1)
        raise AssertionError(ty)

    def try_row(self, row, args):
        args = list(args)
        fields = {}
        if row.opt_inbox:
            fields['m'] = 0
            if args and args[-1] == 'inbox':
                args.pop()
                fields['m'] = 1
        for o in reversed(row.opt):             # txw's [, k] [, last]: the kind before last, either left out (6.6)
            letter = o if o in row.fields else next(k for k, ty in row.types.items() if o in OPT_WORDS.get(ty, ()))
            words = OPT_WORDS[row.types[letter]]
            fields[letter] = 0
            if len(args) > len(row.ops) and args[-1] in words:
                fields[letter] = words[args.pop()]
        if len(args) != len(row.ops):
            raise NoMatch
        for pat, a in zip(row.ops, args):
            if re.fullmatch(r'R[a-z]', pat):
                fields[pat[1]] = self.operand(row.types[pat[1]], a)
            elif pat in row.fields:
                fields[pat] = self.operand(row.types[pat], a)
            elif pat in ('0', '1'):
                try:
                    if self.val(a) != int(pat):
                        raise NoMatch
                except AsmError:
                    raise NoMatch
            elif a != pat:
                raise NoMatch
        w = row.match
        for letter, v in fields.items():
            lo, width = row.fields[letter]
            w |= (v & ((1 << width) - 1)) << lo
        return w

    def encode(self, m, a, long=False):
        if m == 'li':
            x = self.val(a[1], 0, 0xFFFF)
            out = self.encode('ldi', [a[0], str(x & 255)])
            if x >> 8 or long:
                out += self.encode('lui', [a[0], str(x >> 8)])
            return out
        if m == 'ret':
            return self.encode('jr', ['R7'])
        if m == 'tick' and a and a[0] in REGS:
            return self.encode('edge', ['-', '0', a[0]])
        if m not in BY_MNEM:
            raise AsmError(f'unknown mnemonic {m}')
        err = None
        for row in BY_MNEM[m]:
            try:
                return [self.try_row(row, a)]
            except NoMatch:
                pass
            except AsmError as e:
                err = e
        raise err or AsmError(f'no form of {m} takes {a}')

    def size_of(self, m, a):
        if m != 'li':
            return 1
        try:
            return 2 if self.val(a[1], 0, 0xFFFF) >> 8 else 1
        except AsmError:
            return 2

    def lines(self, text):
        out, stack = [], []
        for no, raw in enumerate(text.splitlines(), 1):
            s = raw.split(';')[0].strip()
            if not s:
                continue
            if s.startswith('.if'):
                stack.append(bool(self.val(s[3:].strip())))
            elif s == '.else':
                stack[-1] = not stack[-1]
            elif s == '.endif':
                stack.pop()
            elif all(stack):
                if s.startswith('.equ'):
                    _, name, expr = s.split(None, 2)
                    if name in self.fixed:
                        continue
                    self.new_name(name, no)
                    if expr in IN_PIN or expr in OUT_PIN or expr in self.pinsym:
                        self.pinsym[name] = self.pinsym.get(expr, expr)
                    else:
                        self.sym[name] = self.val(expr)
                else:
                    out.append((no, s))
        return out

    def assemble(self, text):
        items, pc = [], self.org
        for no, s in self.lines(text):          # pass 1: labels
            while (m := re.match(r'(\w+):\s*(.*)', s)):
                self.new_name(m[1], no)
                self.sym[m[1]] = pc
                s = m[2]
            if not s:
                continue
            if s.startswith('.org'):
                pc = self.val(s[4:].strip())
                continue
            if s.startswith('.data'):
                items.append((no, pc, '.data', [s[5:].strip()]))
                pc += 1
                continue
            if s.startswith('.'):
                raise AsmError(f'line {no}: unknown directive {s}')
            parts = s.split(None, 1)
            args = [x.strip() for x in parts[1].split(',')] if len(parts) > 1 else []
            items.append((no, pc, parts[0], args))
            size = self.size_of(parts[0], args)
            if parts[0] == 'li' and size == 2:
                self.li_long.add(no)
            pc += size
        for no, at, m, a in items:              # pass 2: encode
            try:
                ws = [self.val(a[0], 0, 0xFFFF)] if m == '.data' else self.encode(m, a, no in self.li_long)
            except (AsmError, KeyError, IndexError) as e:
                raise AsmError(f'line {no}: {m} {a}: {e}')
            for i, w in enumerate(ws):
                if at + i >= self.size:
                    raise AsmError(f'line {no}: past end of memory ({self.size} words)')
                if at + i in self.words:
                    raise AsmError(f'line {no}: address {at + i} used twice')
                self.words[at + i] = w
            self.listing.append(f'{at:03x}: {" ".join(f"{w:04x}" for w in ws):<10} {m} {", ".join(a)}')
        return self.words


def assemble(path, defs=None, size=512, org=0):
    a = Assembler(size)
    a.sym.update(defs or {})
    a.fixed, a.org = set(defs or {}), org
    with open(path) as f:
        a.assemble(f.read())
    return a


if __name__ == '__main__':
    a = assemble(sys.argv[1])
    print('\n'.join(a.listing))
    print(f'; {len(a.words)} words')
