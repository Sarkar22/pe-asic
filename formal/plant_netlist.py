#!/usr/bin/env python3
"""Writes a scratch copy of a gate netlist with planted bugs, for the equivalence check's teeth (teeth.sh).
Usage: plant_netlist.py <netlist in> <netlist out> <edit> [<edit> ...]
Edits (instances by name, as in the netlist; nets by name):
  hold_d:<flop>            the flop's D becomes its own Q (it never loads)
  invert_d:<flop>          an inverter in front of the flop's D
  swap_reset:<flop>        inverters before D and after Q: same next-state function, reset value flipped
  rewire_d:<flop>:<other>  the flop's D becomes the other flop's D net
  buf_to_inv:<cell>        a buffer (pins A, X) becomes an inverter (A, Y)
  drop_flop:<flop>         the flop is removed and its Q net tied to 0 (a tielo cell)
  flop_of:<Q net>          (lookup only) print the flop whose Q is this net
Every edit is printed; the input file is only read.
SPDX-License-Identifier: Apache-2.0
"""
import re
import sys

INST = re.compile(r"(^[ \t]*)(sg13cmos5l_[A-Za-z0-9_]+)[ \t]+(\\\S+|[A-Za-z_][A-Za-z0-9_$]*)[ \t]*\((.*?)\);[ \t]*\n",
                  re.S | re.M)
PIN = re.compile(r"\.([A-Za-z_][A-Za-z0-9_]*)\s*\(\s*(.*?)\s*\)\s*(?:,|$)", re.S)


def esc(n):
    return n if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_$]*", n) else "\\" + n + " "


def unesc(n):
    n = n.strip()
    return n[1:].strip() if n.startswith("\\") else n


def main():
    src, dst, edits = sys.argv[1], sys.argv[2], sys.argv[3:]
    text = open(src).read()
    insts = {}
    for m in INST.finditer(text):
        insts[unesc(m.group(3))] = m
    by_q = {}
    for name, m in insts.items():
        for p, n in PIN.findall(m.group(4)):
            if p == "Q":
                by_q[unesc(n)] = name
    new_wires, new_cells, repl = [], [], {}

    def pins(name):
        """pin -> the connection's text as written in the netlist (kept verbatim when re-emitted)"""
        return {p: n.strip() for p, n in PIN.findall(insts[name].group(4))}

    def emit(name, typ, pmap, indent="  "):
        # an escaped identifier ends at white space, so it keeps one before the ')'
        body = ",\n".join(f"{indent}  .{p}({n}{' ' if n.startswith(chr(92)) else ''})" for p, n in pmap.items())
        return f"{indent}{typ} {esc(name)} (\n{body}\n{indent});\n"

    for e in edits:
        kind, *args = e.split(":")
        if kind == "flop_of":
            print(f"flop_of {args[0]}: {by_q.get(args[0])}")
            continue
        name = args[0]
        m = insts[name]
        typ, pm = m.group(2), pins(name)
        if kind == "hold_d":
            pm["D"] = pm["Q"]
        elif kind == "invert_d":
            w = f"tooth_d_{len(new_wires)}"
            new_wires.append(w)
            new_cells.append(emit(f"tooth_inv_{len(new_cells)}", "sg13cmos5l_inv_1", {"A": pm["D"], "Y": w}))
            pm["D"] = w
        elif kind == "swap_reset":
            wd, wq = f"tooth_d_{len(new_wires)}", f"tooth_q_{len(new_wires)}"
            new_wires += [wd, wq]
            new_cells.append(emit(f"tooth_inv_{len(new_cells)}", "sg13cmos5l_inv_1", {"A": pm["D"], "Y": wd}))
            new_cells.append(emit(f"tooth_inv_{len(new_cells)}", "sg13cmos5l_inv_1", {"A": wq, "Y": pm["Q"]}))
            pm["D"], pm["Q"] = wd, wq
        elif kind == "rewire_d":
            pm["D"] = pins(args[1])["D"]
        elif kind == "buf_to_inv":
            assert set(pm) == {"A", "X"}, pm
            typ, pm = "sg13cmos5l_inv_1", {"A": pm["A"], "Y": pm["X"]}
        elif kind == "drop_flop":
            typ, pm = "sg13cmos5l_tielo", {"L_LO": pm["Q"]}
        else:
            sys.exit(f"unknown edit {kind}")
        repl[m.start()] = (m.end(), emit(name, typ, pm, m.group(1)))
        print(f"{e}: {m.group(2)} {name} {pins(name)} -> {typ} {pm}")
    out, pos = [], 0
    for start in sorted(repl):
        end, s = repl[start]
        out.append(text[pos:start])
        out.append(s)
        pos = end
    out.append(text[pos:])
    text = "".join(out)
    # new wires are declared before the module's first wire; new cells go just before its end
    k = re.search(r"^[ \t]*wire[ \t]", text, re.M).start()
    text = text[:k] + "".join(f"  wire {w};\n" for w in new_wires) + text[k:]
    k = text.rindex("endmodule")
    text = text[:k] + "".join(new_cells) + text[k:]
    open(dst, "w").write(text)


if __name__ == "__main__":
    main()
