#!/usr/bin/env python3
"""
Checks in the final GDS that every power pin of the SRAM macro is connected.

An unpowered macro passes DRC and LVS, and precheck has no power check on this process. So this
reads the real metal and nothing from the flow's database. It starts at the block's VPWR and VGND
pins in the final LEF, the shapes the chip top connects to, and follows Metal4 and the vias and
metal below it. Then it looks at every Metal4 power column of the macro, from the macro's LEF, placed
as the GDS places the macro.

It passes only if each column is real metal, is reached by its own net and is not reached by the
other net, if no shape belongs to both nets, and if no block stripe lands on any other Metal4 in the
macro. Exit status 0 is a pass, 1 a failure.

Usage: tools/check_macro_power.py <final.gds> <final.lef> <macro.lef> [--macro-prefix RM_IHPSG13_]
Needs the klayout Python module.
SPDX-License-Identifier: Apache-2.0
"""

import argparse
import re
import sys

import klayout.db as db

# sg13cmos5l GDS numbers, drawing purpose: these are the shapes that become metal.
GDS = {
    "Metal1": (8, 0),
    "Via1": (19, 0),
    "Metal2": (10, 0),
    "Via2": (29, 0),
    "Metal3": (30, 0),
    "Via3": (49, 0),
    "Metal4": (50, 0),
}
# Upper metal, via, lower metal.
STACK = [("Metal4", "Via3", "Metal3"), ("Metal3", "Via2", "Metal2"), ("Metal2", "Via1", "Metal1")]
# How the macro's LEF names its supplies, and the block net each one belongs on.
SUPPLIES = {"VDD!": "VPWR", "VDDARRAY!": "VPWR", "VSS!": "VGND"}


def lef_pins(path):
    """Returns {macro: {pin: [(layer, x0, y0, x1, y1), ...]}} in microns."""
    macros = {}
    macro = pin = layer = None
    for line in open(path):
        words = line.split()
        if not words:
            continue
        if words[0] == "MACRO":
            macro = words[1]
            macros[macro] = {}
        elif words[0] == "PIN" and macro:
            pin = words[1]
            macros[macro][pin] = []
        elif words[0] == "END" and len(words) > 1 and words[1] == pin:
            pin = None
        elif words[0] == "LAYER" and pin:
            layer = words[1]
        elif words[0] == "RECT" and pin:
            numbers = re.findall(r"-?\d+\.?\d*", line)
            x0, y0, x1, y1 = (float(n) for n in numbers[:4])
            macros[macro][pin].append((layer, x0, y0, x1, y1))
    return macros


def flat(layout, top, name, window=None):
    """The merged metal of one layer, flattened, within a window if one is given."""
    index = layout.find_layer(*GDS[name])
    if index is None:
        return db.Region()
    shapes = top.begin_shapes_rec_touching(index, window) if window else top.begin_shapes_rec(index)
    return db.Region(shapes).merged()


def flood(seed, metal):
    """Everything connected to the seed, as {layer: region}."""
    hit = {name: db.Region() for name in GDS}
    hit["Metal4"] = metal["Metal4"].overlapping(seed)
    before = -1
    while True:
        for upper, via, lower in STACK:
            hit[via] = (hit[via] + metal[via].overlapping(hit[upper])).merged()
            hit[lower] = (hit[lower] + metal[lower].overlapping(hit[via])).merged()
        for upper, via, lower in reversed(STACK):
            hit[via] = (hit[via] + metal[via].overlapping(hit[lower])).merged()
            hit[upper] = (hit[upper] + metal[upper].overlapping(hit[via])).merged()
        now = sum(region.count() for region in hit.values())
        if now == before:
            return hit
        before = now


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("gds")
    parser.add_argument("lef")
    parser.add_argument("macro_lef")
    parser.add_argument("--macro-prefix", default="RM_IHPSG13_")
    args = parser.parse_args()

    layout = db.Layout()
    layout.read(args.gds)
    dbu = layout.dbu
    top = layout.top_cell()

    block = lef_pins(args.lef)
    block_pins = block.get(top.name) or next(iter(block.values()))
    macro_cells = lef_pins(args.macro_lef)

    instances = []
    for inst in top.each_inst():
        name = layout.cell(inst.cell_index).name
        if name.startswith(args.macro_prefix):
            instances.append((inst, name))
    if not instances:
        print(f"FAIL: no instance of {args.macro_prefix}* at the top of {top.name}")
        return 1

    ok = True
    print(f"top {top.name}, {len(instances)} macro instance(s)")
    for inst, name in instances:
        pins = macro_cells.get(name)
        if pins is None:
            print(f"FAIL: {name} is not in {args.macro_lef}")
            ok = False
            continue
        trans = inst.trans
        box = inst.bbox()
        print(f"{name}: placed {trans}, die box x {box.left * dbu:.3f}..{box.right * dbu:.3f} "
              f"y {box.bottom * dbu:.3f}..{box.top * dbu:.3f}")

        # Metal4 over the whole die, so a stripe is seen to its full length. Below that only the
        # macro's own window matters, and it holds most of the shapes in the file.
        window = box.enlarged(1000, 1000)
        metal = {name2: flat(layout, top, name2, None if name2 == "Metal4" else window) for name2 in GDS}
        top_m4 = db.Region(top.shapes(layout.find_layer(*GDS["Metal4"]))).merged()

        hits = {}
        for net in ("VPWR", "VGND"):
            rects = [r for r in block_pins.get(net, []) if r[0] == "Metal4"]
            if not rects:
                print(f"FAIL: {net} has no Metal4 pin in {args.lef}")
                return 1
            seed = db.Region()
            for _, x0, y0, x1, y1 in rects:
                pin_box = db.Box(round(x0 / dbu), round(y0 / dbu), round(x1 / dbu), round(y1 / dbu))
                if (metal["Metal4"] & db.Region(pin_box)).is_empty():
                    print(f"FAIL: {net} pin at x {x0}..{x1} y {y0}..{y1} has no Metal4 in the GDS")
                    ok = False
                seed.insert(pin_box)
            hits[net] = flood(seed, metal)
            print(f"{net}: {len(rects)} Metal4 pin rectangle(s), reaches "
                  + ", ".join(f"{hits[net][n].count()} {n}" for n in ("Metal4", "Metal3", "Metal2", "Metal1")))

        for layer in GDS:
            shorted = hits["VPWR"][layer] & hits["VGND"][layer]
            if not shorted.is_empty():
                print(f"FAIL: VPWR and VGND share {shorted.count()} shape(s) on {layer}, "
                      f"first at {shorted.bbox()}")
                ok = False

        # The stripes cross a Metal4 OBS in the macro's LEF, which is why the flow's overlap check is
        # waived. What counts is the GDS: a stripe may only touch the macro's power columns.
        macro_cell = layout.cell(inst.cell_index)
        macro_m4 = db.Region(macro_cell.begin_shapes_rec(layout.find_layer(*GDS["Metal4"])))
        macro_m4 = macro_m4.transformed(trans).merged()
        all_columns = db.Region()
        for pin in SUPPLIES:
            for _, x0, y0, x1, y1 in pins.get(pin, []):
                local = db.Box(round(x0 / dbu), round(y0 / dbu), round(x1 / dbu), round(y1 / dbu))
                all_columns.insert(local.transformed(trans))
        other_metal = macro_m4 - all_columns
        clash = other_metal & top_m4
        print(f"macro Metal4 outside its power columns: {other_metal.count()} shape(s), "
              f"{clash.count()} under a block stripe")
        if not clash.is_empty():
            print(f"FAIL: a block stripe lands on macro Metal4 that is not a power column, at {clash.bbox()}")
            ok = False

        for pin, net in SUPPLIES.items():
            other = "VGND" if net == "VPWR" else "VPWR"
            columns = [r for r in pins.get(pin, []) if r[0] == "Metal4"]
            if not columns:
                print(f"FAIL: {pin} has no Metal4 column in {args.macro_lef}")
                ok = False
                continue
            by_stripe = through_macro = 0
            bad = []
            for _, x0, y0, x1, y1 in columns:
                local = db.Box(round(x0 / dbu), round(y0 / dbu), round(x1 / dbu), round(y1 / dbu))
                column = db.Region(local.transformed(trans))
                where = f"local x {x0}..{x1} y {y0}..{y1}"
                covered = (metal["Metal4"] & column).area() / column.area()
                if covered < 0.99:
                    bad.append(f"{where}: only {covered:.1%} is Metal4 in the GDS")
                elif (hits[net]["Metal4"] & column).is_empty():
                    bad.append(f"{where}: not reached by {net}")
                elif not (hits[other]["Metal4"] & column).is_empty():
                    bad.append(f"{where}: also reached by {other}")
                elif (top_m4 & column).is_empty():
                    through_macro += 1
                else:
                    by_stripe += 1
            state = "ok" if not bad else "FAIL"
            print(f"{pin:10s} -> {net}: {len(columns)} columns, {by_stripe} under a block stripe, "
                  f"{through_macro} reached through the macro's own metal, {len(bad)} bad: {state}")
            for line in bad:
                print(f"    {line}")
                ok = False

    print("PASS: every macro power column is on its own net" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
