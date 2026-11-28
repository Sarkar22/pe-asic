#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Write a program image that host/pehost.py loads, so the board needs no assembler.

    python3 host/mkimage.py host/bringup.s -o host/bringup.img
    python3 host/mkimage.py programs/basic/uart_tx.s --start 0=start -o uart_tx.img

The source is assembled by sim/asm.py, which reads its instruction table from docs/ISA.md. --start T=LABEL (or
T=ADDRESS) adds a start line for thread T. The format is described in pehost.py.
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.dont_write_bytecode = True          # leave no cache beside the sources
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'sim'))
import asm      # noqa: E402
import pehost   # noqa: E402


def image_text(img, source, starts=None, depth=pehost.DEPTH):
    """The image file for an assembled image img (sim/asm.py's Image), as one string. starts maps a thread to a
    label or an address."""
    lines = ['pe-image 1', '# %s, assembled by sim/asm.py: %d words' % (source, len(img.words))]
    for a, ws in pehost.runs(img.words, 16):
        lines.append('@%04X %s' % (a, ' '.join('%04X' % w for w in ws)))
    for name, a in sorted(img.labels.items(), key=lambda x: (x[1], x[0])):
        lines.append('label %s %04X' % (name, a))
    for t, where in sorted((starts or {}).items()):
        a = img.labels[where] if where in img.labels else int(where, 0)
        if not 0 <= t < pehost.THREADS or not 0 <= a < depth:
            raise ValueError('start %d=%s is not a thread and an address' % (t, where))
        lines.append('start %d %04X' % (t, a))
    lines.append('end %d %04X' % (len(img.words), pehost.image_crc(img.words)))
    return '\n'.join(lines) + '\n'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('source')
    ap.add_argument('-o', '--output', help='image file to write; stdout if left out')
    ap.add_argument('--start', action='append', default=[], metavar='T=LABEL',
                    help='thread T starts at LABEL or at an address')
    args = ap.parse_args(argv)
    img = asm.read_program(args.source, depth=pehost.DEPTH)
    for w in img.warnings:
        print('warning: ' + w, file=sys.stderr)
    starts = {}
    for s in args.start:
        t, _, where = s.partition('=')
        starts[int(t)] = where
    text = image_text(img, os.path.relpath(args.source, os.path.join(HERE, '..')).replace(os.sep, '/'), starts)
    pehost.read_image(text.splitlines())        # the reader takes what was written
    if args.output:
        with open(args.output, 'w') as fh:
            fh.write(text)
    else:
        sys.stdout.write(text)


if __name__ == '__main__':
    main()
