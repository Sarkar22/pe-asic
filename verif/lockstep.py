#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""The RTL in lockstep with the cycle simulator sim/sim.py: the whole chip, every cycle.

Each image runs on two chips with the same memory, the same pad inputs and the same host commands: the RTL, as
the Verilator model that verif/Makefile builds (the stub kind, behind rtl_chip.py, so a host command lands in the
cycle the bench picks), and sim/sim.py. After every cycle the bench compares everything both can show. It exits 0
only if no image differs, and prints the first difference of each image that does.

Compared after every cycle:
- the pads: uo_out, uio_out and uio_oe, the bypass lanes on this cycle's ui (sim.py doesn't lag them, section 10);
- every thread: PC, R0..R7, T, D, S, LATE, STALE, ILL, RUN, P, WAITING, STOPREQ, STEPPING, MEM, MEM_DONE, MEM_KIND
  and MEM_REG, and while P the event: KIND, DUE, MASK, PAT, BANK, NE, TIMED and INBOX;
- the pipeline, by where each thread's memory cycle puts its word (3.1, 4.2): IV and IR in stages 2 to 4, a load's
  data, and a ld or st's address and data in its memory cycle; TB, SYNC1, SYNC2 and SYNC3;
- every shift unit: each name of section 5.9.2 (CMT aside: it lasts one cycle, so between cycles it reads 0) and
  the 16 slots of 5.9.3; and the RTL's window points PWQ and PWH (5.9.6, implementation note) against p - p/4 and
  p - p/2 of the slots as they stand;
- the pin layer of 5.2 to 5.4 and 5.9.2: OUT, SHD, OE, SHOE, OD, BYP, SHBYP, OWN, SEL and SHSEL;
- M0..M7, FULL and BUSY;
- all of memory, every 64 cycles and at the end.

Images (--gen):
- chips: 128 words biased to the eight rows of 5.9.4 and to unit settings that do something (scfg of a slot value
  from a small table, sgo, txw, rxw, sstat, srd, smap, pcfg), the rest random legal words; threads start at random
  addresses. The uio pads are looped back (a pin the chip drives reads its own drive, 5.2), so units receive their
  own and each other's sends.
- prog: a program per thread, at 128 * i in 512 words, that writes 6 to 16 slots (RPIN on io0 to io3), claims one or
  two of io0 to io3 (and sometimes a uo pin), maps them to output A or B, starts the unit, then loops: bursts of txw,
  sstat with rxw when RXFULL is set, srd, ticks, and now and then scfg, sstop, sgo, smap, pcfg or a random legal word.
- rows: every word of memory a random legal word of any of the 89 rows of 6.3, and one in 14 a random 16-bit word,
  so some are illegal (section 10); the inputs hold for 1, 3, 20 or 200 cycles; every 1500 cycles one thread's D is
  put half the ring from TB, so STALE sets (3.5) without waiting 32,768 cycles.
- wide: like prog, but every slot is written, each drawn field by field over the field's whole range (5.9.3): every
  CRC width, CINIT, CXOR, CMODE, SMODE and SN, TPER and RPER up to 65535, TFRAC, TAIL, every decoder and option; the
  pins are any of io0 to io7 in IN, PP or OD, uo pins and bypass lanes. With extras, one image in four is reset in
  mid-run (section 8): the RTL's rst_n goes low for 4 edges and is released through its synchroniser, and sim.py
  starts again as a new chip with the RTL's memory, which has no reset; then the threads start again.
- directed: hand-written programs (the assembly of 6.6, in DIRECTED below) for corners chance doesn't reach, one case
  per seed: RXIC saturating at 65535 and starting again at an edge, TPER 65535 with a TFRAC carry, and CTX moved in
  the middle of a CRC append. No host traffic.

Corners (--corners, instead of images): scripted sequences, each on every thread, with host commands landing in the
cycle the script picks through the real command block, compared after every cycle as above: a stop
during a stepped ld (3.6); a stop in the cycle of a wait's timeout, match or inbox exit (3.2); STALE setting and
each way of clearing it, 34,000 cycles each (3.5); pcfg in and out of OD beside an ABORT (5.3); wrd and tick 0 on the
corners of TB's ring (3.4, 7.2); STALE read in stage 4 (7.1); a start in the slot where a stepped tick sets LATE
(9.3); deadlines on both sides of the 32767-cycle boundary (3.5, section 10, defined 2); now on a stale D (3.5); and
every row behind a pending tick, world rows waiting for it, and words with a forbidden field value, illegal (3.6, 6.3,
section 10); and ABORT in the cycle of an oe8 or a put8 (9.3, 3.2).

Host commands (--host), one at a time, each through the RTL's real command block (pe_host.sv):
- quick: only the byte that acts (cycle h, 9.1) goes to the block, with the bytes before it taken as received, and a
  command in every gap if the traffic asks for one. Replies are read from the block's reply register. sim.py serves
  the command in the same gap, so both act after step 5 of cycle h.
- full: every byte of the transaction goes through the block, one per gap between cycles, through its own byte
  counter, so the command decodes and its arguments latch as on silicon. A byte that would start a transaction
  resets the counter first, as CS high does. Every reply byte the block offers is compared with section 9.2: STATUS
  bytes with sim.py's flops at the start of the cycle in which the byte before arrives, TAKE's full flag and data,
  POST's result, RDREG's value and RDMEM's data, CAPS, and 0 everywhere else. Also unknown command codes, commands
  cut before their last argument, and bytes after a command's last argument (all must do nothing).
- mixed (default): quick on even seeds, full on odd.
- pins: the pins model, with the real SPI block (pe_spi.sv) at the pads: every bit goes to ui0 to ui2 with SCK high and
  low phases of 2 to 5 cycles (at most clk/6) and CS high for 3 to 8 cycles between transactions; each command acts in
  cycle h = p + 2 of its last rising SCK p, where sim.py serves it; every reply byte is read from MISO at the rising
  edges, as a host would; MISO must hold its last bit for 2 cycles after CS rises and be 0 from the third. uo3 is
  MISO, so the pads compare leaves it out. Also commands whose last byte CS cuts part-way (9.1), and memory commands
  that CS ends right after their last word, before the access lands or BUSY sets (9.4).
Both: POST, TAKE, RUN stop and start, STEP, SETPC, STATUS, ABORT, RDREG of 0 to 39 (12 to 31 most often) and, with
extras, WRMEM, RDMEM, RDREG of any reg8 on any thread, CAPS and aimed core corners. The aimed traffic: ABORT while a
uio pin is on a shift output or a unit runs, RDREG of a stopped thread whose unit runs, STEP of a stopped thread
whose next word is a unit row, a stop of a thread while its unit sends; with extras, POST or TAKE in the cycle a
thread's mailbox row runs, ABORT beside a pcfg or another pin write, a stop in the slot of a timed wait's deadline,
a stop during a stepped ld or st, and a start in the slot where a stepped row sets LATE; and RUN naming a thread in
both masks (9.3: it is only stopped).

Where sim.py serves in cycle h what the RTL serves later (section 10, item 6), the bench compares at the RTL's cycle:
- RDREG: the value against sim.py's rdreg() at the start of the cycle the RTL serves it, which must be h + 2 to h + 5
  (9.2);
- WRMEM and RDMEM: sim.py's host_wrmem or host_rdmem runs in the cycle the RTL takes the memory, which must be the
  first free memory cycle (a stopped thread's, with no ld or st under way), h + 1 to h + 4; RDMEM's data must be in
  the reply register 2 cycles later (9.4). A memory command that finds every thread running sets BUSY in h + 1 on
  both, and does nothing more.

Usage: lockstep.py [--corners | --gen chips|prog|rows|wide|directed] [--images N] [--cycles C] [--first SEED]
                   [--depths 128,512] [--host quick|full|mixed|pins] [--no-extra] [--jobs J] [--build DIR]
                   [--stop-first] [--out FILE] [--fcov FILE] [--cov FILE]
--no-extra leaves out the extra traffic, so quick images then get exactly the host commands of the first version of
this bench. --fcov records functional coverage on the sim.py side (verif/fcov.py) and writes it as JSON; --cov needs
coverage models (verif/coverage.sh) and writes the RTL's line and toggle counts (verif/vlcov.py).
Standard library only, besides the Verilator models. Run from anywhere.
"""
import argparse
import concurrent.futures
import json
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'sim'))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

import asm                      # noqa: E402
import rtl_chip                 # noqa: E402
import sim                      # noqa: E402

# 5.9.2's names, as sim.py and rtl_chip.py spell them. CMT lasts one cycle; PWQ and PWH are the RTL's own.
UNIT_NAMES = [n for n in sim.UNIT_STATE if n != 'CMT']
RTL_UNIT = dict(rtl_chip.UNIT)
assert set(RTL_UNIT) == set(UNIT_NAMES) | {'PWQ', 'PWH'}, set(RTL_UNIT) ^ set(UNIT_NAMES)
M25 = (1 << 25) - 1             # RXPH is 25 bits in the RTL, a signed integer in sim.py
UNIT_MN = ('txw', 'scfg', 'smap', 'rxw', 'sstat', 'srd', 'sgo', 'sstop')
C = rtl_chip                    # command codes of pe_host.sv: C.C_POST and so on
LANES = (0, 4, 5, 6, 7)         # the bypass lanes (5.8)


class Mismatch(Exception):
    pass


# ---- images -----------------------------------------------------------------------------------------------------
_pool = []


def pool():
    """Every legal word, in order (section 10: a word is legal if it matches a row with legal field values)."""
    if not _pool:
        _pool.extend(w for w in range(65536) if asm.decode(w))
    return _pool


def enc(mn, **f):
    r = next(x for x in asm.ROWS if x.mn == mn and set(x.bits) == set(f))
    w = asm.encode(r, f)
    assert asm.decode(w)[0] is r, (mn, f)
    return w


def slot_value(rng, k):
    """Slot values that make a unit do something: short periods, decoders on the looped-back io pins."""
    return {
        0: rng.choice((0x0000, 0x0081, 0x0018, 0x0060, 0x0001, rng.getrandbits(15))),
        1: rng.choice((0x0401, 0x0E00, 0x0101, 0x0000, rng.getrandbits(16))),
        2: rng.choice((0, 1, 2, 3, 5, rng.randrange(1, 20))),
        3: rng.choice((0, 3, 0x800F, 0x8002, rng.getrandbits(16))),
        4: rng.choice((0x0821, 0x0A61, 0x0961, 0x0B22, rng.getrandbits(16) & 0xF0FF | rng.randrange(8, 12) << 8)),
        5: rng.choice((1, 2, 5, 8, rng.randrange(1, 40))),
        6: rng.choice((0, 1, 2, 4, 8)),
        7: rng.choice((0, 0, 30, 200)),
        8: rng.choice((0, 0x8000, 0x4010, rng.getrandbits(16))),
        9: rng.choice((0xD555, 0x8000, 0x7E00, rng.getrandbits(16))),
        10: rng.choice((0, 0x0C00, 0x0100, 0x0E00, rng.getrandbits(16))),
        11: rng.choice((0x04C1, 0x8005, 0x1021, rng.getrandbits(16))),
        12: rng.choice((0x1DB7, 0, rng.getrandbits(16))),
        13: rng.choice((0xC704, 0x800D, 0, rng.getrandbits(16))),
        14: rng.choice((0xDD7B, 0, rng.getrandbits(16))),
        15: rng.choice((0x02FF, 0x00FF, 0xE6EF, 0xE4EF, 0x56EF, 0x5A8E, 0x0287, rng.getrandbits(16))),
    }[k]


def image_chips(rng):
    """128 words: blocks of 'ldi/lui R1, value; scfg R1, slot', unit rows, pin claims, events, random legal words."""
    words = []
    while len(words) < 128:
        x = rng.random()
        if x < 0.25:
            k = rng.randrange(16)
            v = slot_value(rng, k)
            words += [enc('ldi', n=1, i=v & 255), enc('lui', n=1, i=v >> 8), enc('scfg', s=1, k=k)]
        elif x < 0.32:
            words.append(enc('sgo', m=rng.choice((1, 2, 3))))
        elif x < 0.34:
            words.append(enc('sstop', m=rng.choice((1, 2, 3))))
        elif x < 0.44:
            words.append(enc('txw', s=rng.randrange(8), n=rng.randrange(16), l=int(rng.random() < 0.2),
                             k=rng.randrange(4)))
        elif x < 0.50:
            words.append(enc('rxw', d=rng.randrange(8)))
        elif x < 0.53:
            words.append(enc('sstat', d=rng.randrange(8)))
        elif x < 0.55:
            words.append(enc('srd', d=rng.randrange(8), s=rng.randrange(3)))
        elif x < 0.61:
            words.append(enc('smap', p=rng.choice((0, 1, 2, 4, 5, 8, 9, 10, 11)), l=rng.randrange(3)))
        elif x < 0.66:
            p = rng.choice((8, 9, 10, 11, 12))           # pcfg p, IN / PP / OD on a uio pin
            words.append(0x9000 | p << 8 | 7 << 5 | rng.randrange(3) << 3)
        elif x < 0.74:
            words.append(rng.choice((enc('now'), enc('tick', i=rng.randrange(1, 40)))))
        elif x < 0.78:
            words.append(enc('goto', a=rng.randrange(128)))
        else:
            words.append(rng.choice(pool()) if rng.random() < 0.95 else rng.getrandbits(16))
    return {a: w for a, w in enumerate(words[:128])}


def pcfg(p, mode):
    """pcfg p, IN / PP / OD / BYP."""
    return 0x9000 | p << 8 | 7 << 5 | mode << 3


def li(rng, reg, v):
    return [enc('ldi', n=reg, i=v & 255), enc('lui', n=reg, i=(v >> 8) & 255)]


def push(rng, n):
    """n words into the send buffer: li R2, v / txw R2, n [, k] [, last]."""
    w = []
    for _ in range(n):
        w += li(rng, 2, rng.getrandbits(16))
        w.append(enc('txw', s=2, n=rng.randrange(16), l=int(rng.random() < 0.15), k=rng.randrange(4)))
    return w


def thread_program(rng, base):
    """One thread's program at base, 128 words: set up, then a loop that keeps the unit busy."""
    w = []
    slots = [0, 1, 2, 4, 5] + [k for k in (3, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15) if rng.random() < 0.6]
    rng.shuffle(slots)
    for k in slots:
        v = slot_value(rng, k)
        if k == 4 and rng.random() < 0.8:                   # receive on io0 to io3, where the sends are
            v = (v & 0xF0FF) | rng.randrange(8, 12) << 8
        if k == 2 and rng.random() < 0.7:
            v = rng.randrange(1, 9)
        w += li(rng, 1, v) + [enc('scfg', s=1, k=k)]
    pins = rng.sample(range(8, 12), rng.choice((1, 2)))
    if rng.random() < 0.3:
        pins.append(rng.choice((0, 1, 2, 4, 5, 6, 7)))
    for p in pins:
        w += [pcfg(p, 2 if p >= 8 and rng.random() < 0.2 else 1), enc('smap', p=p, l=rng.choice((1, 1, 2)))]
    w += push(rng, rng.randrange(1, 3)) + [enc('sgo', m=rng.choice((3, 3, 3, 1, 2))), enc('now')]
    loop = base + len(w)
    while len(w) < 120:
        x = rng.random()
        if x < 0.30:
            w += push(rng, rng.randrange(1, 4))
        elif x < 0.45:                                       # sstat; if RXFULL (bit 9): rxw
            d = rng.choice((4, 5))
            w += [enc('sstat', d=d), enc('bt', n=d, b=9), enc('jf', a=base + len(w) + 4), enc('rxw', d=3)]
        elif x < 0.52:
            w.append(enc('srd', d=rng.choice((5, 6)), s=rng.randrange(3)))
        elif x < 0.60:
            w.append(enc('tick', i=rng.randrange(1, 30)))
        elif x < 0.65:
            w += push(rng, 1) + [enc('sgo', m=rng.choice((1, 2, 3))), enc('now')]
        elif x < 0.68:
            w.append(enc('sstop', m=rng.choice((1, 2, 3))))
        elif x < 0.73:
            k = rng.randrange(16)
            w += li(rng, 1, slot_value(rng, k)) + [enc('scfg', s=1, k=k)]
        elif x < 0.77:
            w.append(enc('smap', p=rng.choice((0, 1, 2, 4, 5, 8, 9, 10, 11)), l=rng.randrange(3)))
        elif x < 0.79:
            w.append(pcfg(rng.choice((8, 9, 10, 11)), rng.randrange(3)))
        elif x < 0.83:
            w.append(enc('rxw', d=3))
        elif x < 0.86:
            w.append(rng.choice(pool()))
        else:
            w.append(enc('idle'))
    w = w[:126] + [enc('goto', a=loop)]
    return {base + i: x for i, x in enumerate(w)}


def image_prog(rng):
    words = {}
    for t in range(4):
        words.update(thread_program(rng, 128 * t))
    return words


def per_value(rng):
    """A TPER or RPER: mostly short, sometimes 16 to 255, 256 and up, or any 16-bit value."""
    x = rng.random()
    return (0 if x < 0.1 else 1 if x < 0.25 else rng.randrange(2, 16) if x < 0.7 else rng.randrange(16, 256)
            if x < 0.85 else rng.randrange(256, 320) if x < 0.97 else rng.getrandbits(16))


def slot_wide(rng, k):
    """A slot value drawn field by field over the field's whole range (5.9.3), weighted so that sends and receptions
    start and end inside an image: decoders mostly on io0 to io7, which the units drive."""
    r = rng.random
    if k == 0:                                      # TXMODE, bit 15 reserved: KEEP drops it
        v = rng.getrandbits(15) & ~(1 << 9) if r() < 0.85 else rng.getrandbits(15)
        return v | (r() < 0.05) << 15
    if k == 2 or k == 5:                            # TPER, RPER
        return per_value(rng)
    if k == 3:                                      # TAIL, TFRAC
        tail = 0 if r() < 0.4 else 1 if r() < 0.3 else rng.randrange(2, 256) if r() < 0.3 else rng.randrange(2, 12)
        return tail | (0 if r() < 0.5 else rng.getrandbits(8)) << 8
    if k == 4:                                      # RXMODE: RPIN on an io pin most of the time
        v = rng.getrandbits(16)
        return (v & 0xF0FF) | rng.randrange(8, 16) << 8 if r() < 0.8 else v
    if k == 6:                                      # FIRST
        return 0 if r() < 0.2 else 1 if r() < 0.25 else rng.randrange(2, 40)
    if k == 7:                                      # REND
        return 0 if r() < 0.5 else rng.randrange(1, 300)
    if k == 8:                                      # RCNT, SYNCLEN
        cnt = 0 if r() < 0.5 else rng.randrange(1, 40)
        sl = 0 if r() < 0.4 else rng.randrange(1, 17) if r() < 0.85 else rng.randrange(17, 32)
        return cnt | sl << 11
    if k == 13 or k == 14:                          # CHKHI, CHKLO: often 0
        return 0 if r() < 0.3 else rng.getrandbits(16)
    return rng.getrandbits(16)                      # TXMAP, RXSYNC, RXOPT, POLYHI, POLYLO, CODING: all fields


def thread_program_wide(rng, base):
    """Like thread_program, with every slot written from slot_wide and every pin kind in play: io0 to io7 in IN, PP
    and OD, uo pins, and bypass lanes switched in and out."""
    w = []
    slots = list(range(16))
    rng.shuffle(slots)
    for k in slots:
        w += li(rng, 1, slot_wide(rng, k)) + [enc('scfg', s=1, k=k)]
    pins = rng.sample(range(8, 16), rng.choice((1, 2, 3)))
    if rng.random() < 0.5:
        pins.append(rng.choice((0, 1, 2, 4, 5, 6, 7)))
    for p in pins:
        mode = rng.choice((1, 1, 2, 0)) if p >= 8 else 3 if p in LANES and rng.random() < 0.3 else 1
        w += [pcfg(p, mode), enc('smap', p=p, l=rng.choice((1, 1, 2, 0)))]
    w += push(rng, rng.randrange(1, 3)) + [enc('sgo', m=rng.choice((3, 3, 3, 1, 2))), enc('now')]
    loop = base + len(w)
    while len(w) < 120:
        x = rng.random()
        if x < 0.28:
            w += push(rng, rng.randrange(1, 4))
        elif x < 0.42:
            d = rng.choice((4, 5))
            w += [enc('sstat', d=d), enc('bt', n=d, b=9), enc('jf', a=base + len(w) + 4), enc('rxw', d=3)]
        elif x < 0.48:
            w.append(enc('srd', d=rng.choice((5, 6)), s=rng.randrange(3)))
        elif x < 0.55:
            w.append(enc('tick', i=rng.randrange(1, 60)))
        elif x < 0.61:
            w += push(rng, 1) + [enc('sgo', m=rng.choice((1, 2, 3))), enc('now')]
        elif x < 0.64:
            w.append(enc('sstop', m=rng.choice((1, 2, 3))))
        elif x < 0.70:
            k = rng.randrange(16)
            w += li(rng, 1, slot_wide(rng, k)) + [enc('scfg', s=1, k=k)]
        elif x < 0.75:
            w.append(enc('smap', p=rng.choice((0, 1, 2, 4, 5, 6, 7) + tuple(range(8, 16))), l=rng.randrange(3)))
        elif x < 0.79:
            p = rng.choice((0, 4, 5, 6, 7) + tuple(range(8, 16)))
            w.append(pcfg(p, rng.randrange(3) if p >= 8 else rng.choice((1, 3))))
        elif x < 0.83:
            w.append(enc('rxw', d=3))
        elif x < 0.87:
            w.append(rng.choice(pool()))
        else:
            w.append(enc('idle'))
    w = w[:126] + [enc('goto', a=loop)]
    return {base + i: x for i, x in enumerate(w)}


def image_wide(rng):
    words = {}
    for t in range(4):
        words.update(thread_program_wide(rng, 128 * t))
    return words


# Corners that chance does not reach, as programs in the assembly language of 6.6, each on its own thread at 128 * i.
DIRECTED = {
    # 10, defined 17: CTX moved during a send takes the CRC register from the send half at once, in the middle of an
    # append too, whose remaining bits are then CXOR. A CRC-32 fcs word at 20 cycles a bit; 300 cycles after the start
    # the append is under way, and CODING is written with CTX 0.
    'append_ctx': ["""
        li   R1, 20
        scfg R1, TXPER
        li   R1, 0x04C1
        scfg R1, POLYHI
        li   R1, 0x1DB7
        scfg R1, POLYLO
        li   R1, 0x02FF         ; CODING: CRC-32, CINIT, CXOR, the CRC register on the send half
        scfg R1, CODING
        li   R2, 0x5A
        txw  R2, 8, fcs, last
        sgo  tx
        now
        tick 300
        li   R1, 0x00FF         ; the same with CTX 0
        scfg R1, CODING
        halt
    """],
    # 5.9.6: RXIC stops at 65535. A reception with no reference edge for longer than that: an outside clock (RKIND
    # 2) and a timed decoder (RKIND 0), each on an io pin its own thread drives at a constant level; REND 0 and RCNT
    # 0, so neither reception ends. After 68,000 cycles the pin rises, a reference edge, and RXIC starts again.
    'rxic': ["""
        pcfg io7, PP
        li   R1, 0xFF02         ; RXMODE: RKIND 2, RPIN io7, RCLK io7
        scfg R1, RXMODE
        sgo  rx
        now
        li   R3, 17
wait:   tick 4000
        djnz R3, wait
        put  io7, 1
        now
        halt
    """, """
        pcfg io6, PP
        li   R1, 0x0E00         ; RXMODE: RKIND 0, RPIN io6
        scfg R1, RXMODE
        sgo  rx
        now
        li   R3, 17
wait:   tick 4000
        djnz R3, wait
        put  io6, 1
        now
        halt
    """],
    # 5.9.5: a symbol lasts TPER cycles plus the carry of TXACC + TFRAC, so TPER 65535 with a carry is 65536 cycles,
    # the top bit of TXCD. TFRAC 255: the first symbol has no carry (255), the second has one (510). The host then
    # ABORTs in the very cycle that writes the second symbol, so the stopped half keeps TXCD 65536 (tx_stop() leaves
    # it), and later starts the thread at 128 for one more send (DIRECTED_HOST).
    'tper_max': ["""
        li   R1, 0xFFFF
        scfg R1, TXPER
        li   R1, 0xFF00         ; TFRAC 255, TAIL 0
        scfg R1, TXTAIL
        li   R2, 3
        txw  R2, 2, last
        sgo  tx
        now
        halt
    """, """
        li   R2, 1
        txw  R2, 1, last
        sgo  tx
        now
        halt
    """],
}
DIRECTED_START = {'tper_max': 1}          # threads started at the beginning, if not one per program


def long_symbol_host():
    """tper_max's host: ABORT in the cycle TXCD becomes 65536, then 50 cycles later SETPC 128 and start thread 0."""
    state = {'at': None, 'step': 0}

    def hook(m, h, c):
        if state['step'] == 0 and m.th[0].u.TXCD == 0x10000:
            h.abort()
            state['step'], state['at'] = 1, c
        elif state['step'] == 1 and c >= state['at'] + 50:
            h.setpc(0, 128)
            state['step'] = 2
        elif state['step'] == 2:
            h.run(0, 1)
            state['step'] = 3
    return hook


DIRECTED_HOST = {'tper_max': long_symbol_host}


def image_directed(rng, seed):
    """The directed case seed mod the number of cases, so consecutive seeds run each once. Returns its words, how many
    threads start, and its host, if any."""
    name = sorted(DIRECTED)[seed % len(DIRECTED)]
    words = {}
    for t, text in enumerate(DIRECTED[name]):
        img = asm.assemble(text, base=128 * t, depth=512)
        words.update(img.words)
    host = DIRECTED_HOST.get(name)
    return words, DIRECTED_START.get(name, len(DIRECTED[name])), host() if host else None


def random_word(rng):
    """A random legal word: a row, then each field, until the values are legal (6.2)."""
    while True:
        r = rng.choice(asm.ROWS)
        f = {k: rng.getrandbits(len(b)) for k, b in r.bits.items()}
        if all(asm.legal_value(f[k], r.ftypes[k]) for k in f):
            return asm.encode(r, f)


def image_rows(rng, depth):
    return {x: (random_word(rng) if rng.random() < 0.93 else rng.getrandbits(16)) for x in range(depth)}


# ---- state ------------------------------------------------------------------------------------------------------
def bits(lst):
    v = 0
    for i, x in enumerate(lst):
        v |= x << i
    return v


def window_points(per, opt):
    """5.9.6, implementation note: p - p/4 and p - p/2, with p in 256ths of a cycle from RPER and RFRAC."""
    rs = 0xFF if per >> 8 else per & 0xFF
    pm = 256 if per == 0 else (rs << 8) | (opt & 0xFF)
    return pm - (pm >> 2), pm - (pm >> 1)


def sim_state(m):
    """The compared state of sim.py, as a dict of parts."""
    th, ev, units, mem = [], [], [], []
    for t in m.th:
        th.append((t.PC, tuple(t.R), t.T, t.D, t.S, t.LATE, t.STALE, t.ILL, t.RUN, t.P, t.WAITING, t.STOPREQ,
                   t.STEPPING))
        ev.append((t.KIND, t.DUE, t.MASK, t.PAT, t.BANK, t.NE, t.TIMED, t.INBOX) if t.P else None)
        mem.append((t.MEM, t.MEM_DONE, t.MEM_KIND, t.MEM_REG))
        u = t.u
        units.append(tuple(getattr(u, n) & M25 if n == 'RXPH' else getattr(u, n) for n in UNIT_NAMES) + tuple(u.CFG))
    return dict(threads=tuple(th), events=tuple(ev), mem=tuple(mem), units=tuple(units), SEL=tuple(m.SEL),
                SHSEL=tuple(m.SHSEL), TB=m.c & 0xFFFF, SYNC1=m.SYNC1, SYNC2=m.SYNC2, SYNC3=m.SYNC3, OUT=bits(m.OUT),
                SHD=bits(m.SHD), OE=bits(m.OE), SHOE=bits(m.SHOE), OD=bits(m.OD), BYP=bits(m.BYP),
                SHBYP=bits(m.SHBYP), OWN=tuple(m.OWN), FULL=tuple(m.FULL), M=tuple(m.M), BUSY=m.BUSY)


class RtlView:
    """The same dict from the RTL's flops, with the field offsets of rtl_chip.py computed once."""

    def __init__(self, chip):
        f = self.f = chip._f
        self.unit = [f['u_' + RTL_UNIT[n]] for n in UNIT_NAMES]
        self.slots = [f['u_' + s] for s in rtl_chip.SLOTS]
        self.th = [f[x] for x in ('pc_q', 't_q', 'd_q', 's_q', 'late_q', 'stale_q', 'ill_q', 'run_q', 'p_q',
                                  'waiting_q', 'stopreq_q', 'stepping_q')]
        self.ev = [f[x] for x in ('kind_q', 'due_q', 'mask_q', 'pat_q', 'bank_q', 'ne_q', 'timed_q', 'inbox_q')]
        self.mem = [f[x] for x in ('mem_q', 'mem_done_q', 'mem_kind_q', 'mem_reg_q')]

    def state(self, chip):
        v = chip.snapshot()
        f = self.f
        th, ev, mem, units, pw = [], [], [], [], []
        for i in range(4):
            pc, t, d, s, late, stale, ill, run, p, wait, stopreq, stepping = (v[k + i] for k in self.th)
            regs = tuple(v[f['reg0'] + 8 * i:f['reg0'] + 8 * i + 8])
            th.append((pc, regs, t, d, s, late, stale, ill, run, p, wait, stopreq, stepping))
            ev.append(tuple(v[k + i] for k in self.ev) if p else None)
            mem.append(tuple(v[k + i] for k in self.mem))
            cfg = tuple(v[k + i] for k in self.slots)
            units.append(tuple(v[k + i] for k in self.unit) + cfg)
            pw.append((v[f['u_pwq_q'] + i], v[f['u_pwh_q'] + i], window_points(cfg[5], cfg[10])))

        def pairs(x):
            return tuple((v[f[x]] >> (2 * p)) & 3 for p in range(16))
        st = dict(threads=tuple(th), events=tuple(ev), mem=tuple(mem), units=tuple(units), SEL=pairs('sel_q'),
                  SHSEL=pairs('shsel_q'), TB=v[f['tb_q']], SYNC1=v[f['sync1_q']], SYNC2=v[f['sync2_q']],
                  SYNC3=v[f['sync3_q']], OUT=v[f['out_q']], SHD=v[f['shd_q']], OE=v[f['oe_q']],
                  SHOE=v[f['shoe_q']], OD=v[f['od_q']], BYP=v[f['byp_q']], SHBYP=v[f['shbyp_q']],
                  OWN=pairs('own_q'), FULL=tuple((v[f['full_q']] >> k) & 1 for k in range(8)),
                  M=tuple(v[f['m0']:f['m0'] + 8]), BUSY=v[f['busy_q']])
        return st, pw, v


THREAD_NAMES = ('PC', 'R', 'T', 'D', 'S', 'LATE', 'STALE', 'ILL', 'RUN', 'P', 'WAITING', 'STOPREQ', 'STEPPING')
EVENT_NAMES = ('KIND', 'DUE', 'MASK', 'PAT', 'BANK', 'NE', 'TIMED', 'INBOX')
MEM_NAMES = ('MEM', 'MEM_DONE', 'MEM_KIND', 'MEM_REG')


def first_diff(c, sm, sr):
    """The differing parts of sim.py's state sm and the RTL's sr, by name."""
    out = []
    for k in sm:
        a, b = sm[k], sr[k]
        if a == b:
            continue
        if k in ('threads', 'events', 'mem', 'units'):
            names = {'threads': THREAD_NAMES, 'events': EVENT_NAMES, 'mem': MEM_NAMES,
                     'units': UNIT_NAMES + ['CFG%d' % i for i in range(16)]}[k]
            for tid in range(4):
                if a[tid] != b[tid]:
                    if a[tid] is None or b[tid] is None:
                        out.append('thread %d %s: sim.py %s, RTL %s' % (tid, k, a[tid], b[tid]))
                    else:
                        out += ['thread %d %s: sim.py %s, RTL %s' % (tid, n, x, y)
                                for n, x, y in zip(names, a[tid], b[tid]) if x != y]
        elif k in ('SEL', 'SHSEL', 'OWN', 'FULL', 'M'):
            out += ['%s[%d]: sim.py %s, RTL %s' % (k, i, x, y) for i, (x, y) in enumerate(zip(a, b)) if x != y]
        else:
            out.append('%s: sim.py %#x, RTL %#x' % (k, a, b))
    return 'cycle %d: ' % c + '; '.join(out[:12])


def pipeline(c, m, v, f):
    """The word in flight, by where each thread's memory cycle puts it (3.1, 4.2), and a ld or st's address and data
    in its memory cycle. c is the cycle just run; v the RTL's flops after it."""
    th = m.th
    bad = []
    for j, vf, wf in (((c - 1) & 3, 'valid2_q', 'dr_r'), ((c - 2) & 3, 'valid3_q', 'ir3_q'),
                      ((c - 3) & 3, 'valid4_q', 'ir4_q')):
        t = th[j]
        if t.MEM:
            if t.MEM_DONE and t.MEM_KIND == sim.LOAD and v[f[wf]] != t.MEM_DATA:
                bad.append('load data of thread %d in %s: sim.py %#x, RTL %#x' % (j, wf, t.MEM_DATA, v[f[wf]]))
            continue
        if v[f[vf]] != t.IV:
            bad.append('IV of thread %d (%s): sim.py %d, RTL %d' % (j, vf, t.IV, v[f[vf]]))
        elif t.IV and v[f[wf]] != t.IR:
            bad.append('IR of thread %d (%s): sim.py %#x, RTL %#x' % (j, wf, t.IR, v[f[wf]]))
    t = th[c & 3]
    if t.MEM and not t.MEM_DONE:
        if v[f['ls_addr_q']] != t.MEM_ADDR:
            bad.append('MEM_ADDR of thread %d: sim.py %#x, RTL %#x' % (c & 3, t.MEM_ADDR, v[f['ls_addr_q']]))
        if t.MEM_KIND == sim.STORE and v[f['ls_data_q']] != t.MEM_DATA:
            bad.append('MEM_DATA of thread %d: sim.py %#x, RTL %#x' % (c & 3, t.MEM_DATA, v[f['ls_data_q']]))
    if bad:
        raise Mismatch('cycle %d: %s' % (c, '; '.join(bad)))


def status_bytes(m):
    """The four STATUS bytes of 9.2 from sim.py's flops as they stand."""
    th = m.th
    run = sum(t.RUN << t.TID for t in th)
    ill = sum(t.ILL << t.TID for t in th)
    late = sum(t.LATE << t.TID for t in th)
    stale = sum(t.STALE << t.TID for t in th)
    return (run << 4 | ill, late << 4 | stale, bits(m.FULL), m.BUSY & 1)


def free_cycle(m):
    """At the start of the cycle about to run: is its memory cycle free for the host (9.4)? It is the memory cycle of
    thread (c - 1) mod 4, free if that thread is stopped with no ld or st under way."""
    t = m.th[(m.c - 1) & 3]
    return not t.RUN and not t.MEM


# ---- host commands ----------------------------------------------------------------------------------------------
KNOWN = {0x00, 0x01, 0x02, 0x10, 0x11, 0x12, 0x13, 0x14, 0x3F} | set(range(0x20, 0x30))
UNKNOWN = [b for b in range(256) if b not in KNOWN]


class Host:
    """One host command at a time, on both chips. A command that spans gaps is a generator: the main loop runs one
    step of it in each gap until it ends, and starts no other command meanwhile. What must happen in a given cycle
    whatever step the command is at (an RDREG served, a memory access taken, BUSY of a failed memory command, a
    byte's last bit acting at the pads) is a watcher, which service() runs in every gap.

    quick: one byte, the one that acts, with the bytes before it taken as received.
    full: every byte, one per gap, through the block's own byte counter (the stub model).
    pins: every bit, through pe_spi at the pads (the pins model); a byte acts in cycle h = p + 2 of its last rising
    SCK (9.1), and the reply is read from MISO at each rising SCK, as a host would."""

    def __init__(self, rc, m, rng, stats, mode, extra, depth, region):
        self.rc, self.m, self.rng, self.stats = rc, m, rng, stats
        self.mode, self.extra, self.depth, self.region = mode, extra, depth, region
        self.full = mode != 'quick'
        self.pins = mode == 'pins'
        self.L, self.H, self.F = rc._lib, rc._h, rc._f
        self.txn = None
        self.watch = []                 # functions run in every gap until they return True
        self.watched = False            # some ran in this gap
        self.c = -1                     # the cycle just run: a command lands in it (h = c)
        self.h_cycle = None             # h of the last byte that acted
        self.rd = None                  # (thread, reg8) of an RDREG waiting to be served
        self.rd_pre = None              # sim.py's rdreg() at the start of this cycle
        self.pre_bytes = None           # sim.py's STATUS bytes at the start of this cycle
        self.was_free = None            # this cycle's memory cycle was free for the host, at its start
        self.before = None              # (LATE, STEPPING) of each thread at the start of this cycle
        self.link = (1, 0, 0)           # pins: CS, SCK, MOSI for the next cycle (CS high: idle)
        self.uo = 0                     # pins: uo_out of the cycle just run, for MISO
        self.cs_high = 0                # pins: cycles CS has been high, the next one included
        self.miso_rise = 0              # pins: MISO in the cycle CS rose
        self.act_at = None              # pins: (h, act, want, what) of a byte whose last bit is on its way
        self.act_result = None
        self.want_next = None           # pins: (want, what) for the MISO byte during the next byte
        self.hi = self.lo = 3           # pins: SCK high and low, in cycles

    # -- what the main loop calls
    def before_cycle(self):
        m = self.m
        if self.rd is not None:
            self.rd_pre = m.rdreg(m.th[self.rd[0]], self.rd[1])
        self.pre_bytes = status_bytes(m)
        self.was_free = free_cycle(m)
        self.before = [(t.LATE, t.STEPPING) for t in m.th]

    def link_bits(self):
        cs, sck, mosi = self.link
        self.cs_high = self.cs_high + 1 if cs else 0
        return cs | sck << 1 | mosi << 2

    def service(self):
        self.watched = bool(self.watch)
        if self.watch:
            self.watch = [w for w in self.watch if not w()]
        a = self.act_at
        if a is not None and self.c == a[0]:
            self.act_at = None
            self.h_cycle = self.c
            self.act_result = a[1]() if a[1] else None
            self.want_next = (a[2]() if callable(a[2]) else a[2], a[3])

    def busy(self):
        return self.txn is not None

    def idle(self):
        """No command in flight, and none of its effects to come: commands are at least 48 cycles apart on silicon,
        so a command never lands in the cycle where the last one's BUSY or memory access does."""
        return self.txn is None and not self.watch and not self.watched

    def step(self):
        try:
            next(self.txn)
        except StopIteration:
            self.txn = None

    def start(self, gen):
        if self.pins:
            gen = self.pins_txn(gen)
        self.txn = gen
        self.step()

    def wait(self, done):
        """Gaps pass until a watcher puts its result in done."""
        while not done:
            yield
        return done[0]

    # -- the RTL's command block
    def real(self, x, what):
        if x < 0:
            raise RuntimeError('%s missed the command block' % what)     # a fault of the bench, not of the RTL
        return x

    def quick(self, cmd, idx, byte, arg=0, n=0, thread=0, what=''):
        """The byte that acts, with the bytes before it taken as received (cmd, idx, arg, n, thread)."""
        return self.real(self.rc._host(cmd, idx, byte & 255, arg=arg & 255, n=n, thread=thread & 3), what)

    def byte(self, b, first=False, want=None, act=None, what=''):
        """One byte of a transaction; first: CS was high before it. act runs where the byte acts (cycle h); want, a
        value or a function called at h, is the reply byte the chip must offer for the next byte (None: any).
        Returns what act returned."""
        if self.pins:
            return (yield from self.pins_byte(b & 255, first, want, act, what))
        r = act() if act else None
        w = want() if callable(want) else want
        L, H, F = self.L, self.H, self.F
        cmd, idx = (0, 0) if first else (L.pe_peek(H, F['cmd_q']), L.pe_peek(H, F['idx_q']))
        x = self.real(L.pe_host(H, cmd, idx, L.pe_peek(H, F['arg_q']), L.pe_peek(H, F['n_q']),
                                L.pe_peek(H, F['thread_q']), b & 255), what)
        self.h_cycle = self.c
        if w is not None and x != w:
            raise Mismatch('cycle %d: %s: the reply byte offered after byte %#04x is %#04x, want %#04x'
                           % (self.c, what, b & 255, x, w))
        return r

    def gap(self):
        """full: the next byte goes in the next gap. pins: a byte takes its own cycles."""
        if not self.pins:
            yield

    def pins_byte(self, b, first, want, act, what):
        prev, self.want_next = self.want_next, None
        if first:
            self.link = (0, 0, 0)                   # CS falls, SCK low for a phase before the first rise
            for _ in range(self.lo):
                yield
        got = 0
        for i in range(7, -1, -1):
            bit = (b >> i) & 1
            self.link = (0, 0, bit)
            for _ in range(self.lo):
                yield
            self.link = (0, 1, bit)
            yield                                   # the cycle p with the rising SCK has run
            got = got << 1 | ((self.uo >> 3) & 1)   # MISO, read at the rising edge
            if i == 0:
                if prev is not None and prev[0] is not None and got != prev[0]:
                    raise Mismatch('cycle %d: %s: MISO during byte %#04x read %#04x, want %#04x'
                                   % (self.c, prev[1], b, got, prev[0]))
                self.act_at = (self.c + 2, act, want, what)
            for _ in range(self.hi - 1):
                yield
        self.link = (0, 0, 0)
        while self.act_at is not None:
            yield
        return self.act_result

    def pins_cut(self, b, bits):
        """pins: the first bits of a byte, then CS rises: the partial byte is dropped (9.1)."""
        for i in range(7, 7 - bits, -1):
            bit = (b >> i) & 1
            self.link = (0, 0, bit)
            for _ in range(self.lo):
                yield
            self.link = (0, 1, bit)
            for _ in range(self.hi):
                yield
        self.link = (0, 0, 0)
        yield

    def pins_txn(self, gen):
        """pins: SCK phases for this transaction, then CS high for 3 to 8 cycles at its end (9.1)."""
        self.hi, self.lo = self.rng.choice(((3, 3), (3, 3), (2, 4), (4, 2), (3, 4), (4, 4), (2, 5), (5, 3)))
        yield from gen
        self.link = (1, 0, 0)
        self.want_next = (0, 'MISO before a command byte')
        for _ in range(self.rng.randrange(3, 9)):
            yield

    def junk(self):
        return self.rng.getrandbits(8)

    def trail(self, gen):
        """A transaction, then up to two more bytes before CS rises: they must do nothing (9.1)."""
        yield from gen
        if self.extra and self.rng.random() < 0.2:
            for _ in range(self.rng.randrange(1, 3)):
                yield from self.gap()
                yield from self.byte(self.junk(), want=0, what='a byte after the last argument')

    # -- watchers
    def served(self, tid, reg, what, h, done):
        """An RDREG of a stopped thread: the RTL must serve it at the start of a cycle from h + 2 to h + 5, with
        sim.py's rdreg() at the start of that cycle (9.2)."""
        self.rd = (tid, reg)
        ch = self.rc

        def w():
            age = self.c - h
            if age > 12:
                raise Mismatch('cycle %d: %s never served' % (self.c, what))
            if age < 2 or ch._get(ch._h, ch._f['rd_cap_q']) or ch._get(ch._h, ch._f['rd_req_q']):
                return False
            if age > 5:
                raise Mismatch('cycle %d: %s served %d cycles after h, not 2 to 5 (9.2)' % (self.c, what, age))
            got, want = ch.hold(), self.rd_pre
            if got != want:
                raise Mismatch('cycle %d: %s: RTL %#06x, sim.py %#06x' % (self.c, what, got, want))
            self.rd = None
            st = self.stats
            u = self.m.th[tid].u
            st['rdreg'] += 1
            st['rdreg_unit'] += 12 <= reg < 32
            st['rdreg_unit_busy'] += 12 <= reg < 32 and bool(u.TXRUN or u.RXRUN)
            st['rdreg_nonzero'] += 12 <= reg < 32 and got != 0
            done.append(got)
            return True
        self.watch.append(w)

    def access(self, h, addr, write, data, what, done):
        """A host memory access after h: in the first free memory cycle, h + 1 to h + 4, to addr (9.4); sim.py's
        host_wrmem or host_rdmem runs after that cycle; a read's word is in the reply register 2 cycles after it."""
        state = []

        def w():
            age = self.c - h
            if state:
                if self.rc.hold() != state[0]:
                    raise Mismatch('cycle %d: %s: reply register %#x, sim.py %#x' % (self.c, what, self.rc.hold(),
                                                                                    state[0]))
                done.append(state[0])
                return True
            at, a, d, wr = self.rc.host_mem()
            if self.was_free and at != self.c:
                raise Mismatch('cycle %d: %s: a free memory cycle not taken, %d after h' % (self.c, what, age))
            if at != self.c:
                if age > 12:
                    raise Mismatch('cycle %d: %s: no host access' % (self.c, what))
                return False
            if not self.was_free:
                raise Mismatch('cycle %d: %s: a host access in a memory cycle that was not free' % (self.c, what))
            if age > 4:
                raise Mismatch('cycle %d: %s: the host access came %d cycles after h' % (self.c, what, age))
            if a != addr or wr != write or (write and d != data):
                raise Mismatch('cycle %d: %s: host access to %#x (write %d, data %#x), want %#x (write %d, data %#x)'
                               % (self.c, what, a, wr, d, addr, write, data))
            if write:
                if not self.m.host_wrmem(addr, data):
                    raise Mismatch('cycle %d: %s: sim.py refused the write' % (self.c, what))
                done.append(None)
                return True
            state.append(self.m.host_rdmem(addr))
            return False
        self.watch.append(w)

    def fails(self, what):
        """9.4: if all four threads run, a memory command sets BUSY in h + 1, from the RUN flops of that cycle.
        Those are the flops after cycle h, which sim.py has now, and no thread starts by itself. So the bench
        decides here, in h, and sets sim.py's BUSY in h + 1, where the RTL sets it."""
        if not all(t.RUN for t in self.m.th):
            return False
        h = self.c

        def w():
            if self.c == h + 1:
                self.m.BUSY = 1
                self.stats['mem_busy'] += 1
                self.stats['wrmem_busy' if what.startswith('WRMEM') else 'rdmem_busy'] += 1
                return True
            return False
        self.watch.append(w)
        return True

    # -- the commands. Each acts in this gap in quick mode, and starts a transaction otherwise.
    def post(self, n, v):
        if self.full:
            return self.start(self.trail(self.g_post(n, v)))
        rm = self.m.host_post(n, v)
        x = self.quick(C.C_POST, 2, v, arg=v >> 8, n=n, what='POST')
        if x != rm:
            raise Mismatch('cycle %d: POST %d of %#x: RTL result %d, sim.py %d' % (self.c, n, v, x, rm))

    def g_post(self, n, v, cut=None):
        what = 'POST %d' % n
        yield from self.byte(0x20 + n, True, 0, what=what)
        yield from self.gap()
        yield from self.byte(v >> 8, want=0, what=what)
        if cut == 2:
            return
        yield from self.gap()
        res = []
        yield from self.byte(v & 255, want=lambda: res[0], act=lambda: res.append(self.m.host_post(n, v)),
                             what=what)                     # h: POST decides on the flag; the result byte follows
        if self.pins or self.rng.random() < 0.5:            # the optional dummy8 that carries it
            yield from self.gap()
            yield from self.byte(self.junk(), want=0, what=what)

    def take(self, n):
        if self.full:
            return self.start(self.trail(self.g_take(n)))
        full, v = self.m.host_take(n)
        x = self.quick(C.C_NONE, 0, 0x28 + n, what='TAKE')
        got = (x & 1, self.rc.hold() if x & 1 else 0)
        if x > 1 or got != (full, v):
            raise Mismatch('cycle %d: TAKE %d: RTL %s, sim.py %s' % (self.c, n, got, (full, v)))

    def g_take(self, n):
        what = 'TAKE %d' % n
        res = []
        yield from self.byte(0x28 + n, True, want=lambda: res[0], act=lambda: res.extend(self.m.host_take(n)),
                             what=what)                     # h: TAKE decides when its command byte arrives
        for k in (0, 1):
            yield from self.gap()
            yield from self.byte(self.junk(), want=lambda k=k: (res[1] >> 8, res[1] & 255)[k], what=what)
        yield from self.gap()
        yield from self.byte(self.junk(), want=0, what=what)

    def run(self, stop, start):
        if self.full:
            if self.extra:                          # bits 7..4 name no thread
                stop |= self.rng.getrandbits(4) << 4
                start |= self.rng.getrandbits(4) << 4
            return self.start(self.trail(self.g_run(stop, start)))
        self.m.host_run(stop, start)
        self.quick(C.C_RUN, 2, start, arg=stop, what='RUN')

    def g_run(self, stop, start, cut=None):
        yield from self.byte(0x10, True, 0, what='RUN')
        yield from self.gap()
        yield from self.byte(stop, want=0, what='RUN')
        if cut == 2:
            return
        yield from self.gap()
        yield from self.byte(start, want=0, act=lambda: self.m.host_run(stop & 15, start & 15), what='RUN')

    def thread8(self, tid):
        """thread8 with bits 7..2 random when every byte goes to the block, with extras: only the low 2 bits count."""
        return tid | (self.rng.getrandbits(6) << 2 if self.full and self.extra else 0)

    def step_cmd(self, tid):
        if self.full:
            return self.start(self.trail(self.g_step(self.thread8(tid))))
        self.m.host_step(tid)
        self.quick(C.C_STEP, 1, tid, what='STEP')

    def g_step(self, t8, cut=None):
        yield from self.byte(0x13, True, 0, what='STEP')
        if cut == 1:
            return
        yield from self.gap()
        yield from self.byte(t8, want=0, act=lambda: self.m.host_step(t8 & 3), what='STEP')

    def setpc(self, tid, pc):
        if self.full:
            if self.extra:                          # pc16 is taken mod DEPTH
                lg = self.depth.bit_length() - 1
                pc |= self.rng.getrandbits(16 - lg) << lg
            return self.start(self.trail(self.g_setpc(self.thread8(tid), pc)))
        self.m.host_setpc(tid, pc)
        self.quick(C.C_SETPC, 3, pc, arg=pc >> 8, thread=tid, what='SETPC')

    def g_setpc(self, t8, pc, cut=None):
        yield from self.byte(0x11, True, 0, what='SETPC')
        yield from self.gap()
        yield from self.byte(t8, want=0, what='SETPC')
        yield from self.gap()
        yield from self.byte(pc >> 8, want=0, what='SETPC')
        if cut == 3:
            return
        yield from self.gap()
        yield from self.byte(pc & 255, want=0, act=lambda: self.m.host_setpc(t8 & 3, pc), what='SETPC')

    def status(self):
        if self.full:
            return self.start(self.trail(self.g_status()))
        self.m.host_status()
        self.quick(C.C_STATUS, 4, 0, what='STATUS')

    def g_status(self, cut=None):
        """9.2: each reply byte shows the flops at the start of the cycle in which the byte before it arrives."""
        busy = []

        def want(i):
            def f():
                if i == 3:
                    busy.append(self.pre_bytes[3])
                return self.pre_bytes[i]
            return f
        yield from self.byte(0x12, True, want=want(0), what='STATUS byte 0')
        for i in (1, 2, 3):
            if cut == i:
                return
            yield from self.gap()
            yield from self.byte(self.junk(), want=want(i), what='STATUS byte %d' % i)
        if cut == 4:
            return
        yield from self.gap()
        got = yield from self.byte(self.junk(), want=0, act=self.m.host_status, what='STATUS')    # h: BUSY := 0
        if got[3] != busy[0]:                       # no command ran since byte 3 took it
            raise Mismatch('cycle %d: STATUS: BUSY in the reply %d, sim.py %d' % (self.c, busy[0], got[3]))

    def abort(self):
        if self.full:
            return self.start(self.trail(self.g_abort()))
        self.m.host_abort()
        self.quick(C.C_NONE, 0, 0x3F, what='ABORT')

    def g_abort(self):
        yield from self.byte(0x3F, True, 0, act=self.m.host_abort, what='ABORT')

    def rdreg(self, tid, reg):
        """RDREG reg8 of thread tid. The value is compared in the cycle the RTL serves it."""
        if self.full:
            return self.start(self.trail(self.g_rdreg(self.thread8(tid), reg)))
        return self.start(self.q_rdreg(tid, reg))

    def rd_act(self, tid, reg, what, done):
        """RDREG at h: sim.py serves it now, setting BUSY if the thread runs; the RTL's value of a stopped thread is
        compared when it is served."""
        def act():
            if self.m.th[tid].RUN:
                self.m.host_rdreg(tid, reg)
                self.stats['rdreg_running'] += 1
                done.append(0)                      # 9.2: RDREG reads 0 then
            else:
                self.m.host_rdreg(tid, reg)
                self.served(tid, reg, what, self.c, done)
        return act

    def q_rdreg(self, tid, reg):
        what = 'RDREG %d of thread %d' % (reg, tid)
        done = []
        act = self.rd_act(tid, reg, what, done)
        self.quick(C.C_RDREG, 2, reg, thread=tid, what=what)
        act()
        yield from self.wait(done)

    def g_rdreg(self, t8, reg, cut=None):
        tid = t8 & 3
        what = 'RDREG %d of thread %d' % (reg, tid)
        yield from self.byte(0x14, True, 0, what=what)
        yield from self.gap()
        yield from self.byte(t8, want=0, what=what)
        if cut == 2:
            return
        yield from self.gap()
        done = []
        yield from self.byte(reg, want=0, act=self.rd_act(tid, reg, what, done), what=what)    # h: reg8
        h = self.h_cycle
        v = yield from self.wait(done)
        while self.c - h < 6:                       # the host's next byte comes later than the read
            yield
        yield from self.gap()
        yield from self.byte(self.junk(), want=v >> 8, what=what)
        yield from self.gap()
        yield from self.byte(self.junk(), want=v & 255, what=what)
        yield from self.gap()
        yield from self.byte(self.junk(), want=0, what=what)

    # -- extras
    def caps(self):
        self.start(self.trail(self.g_caps()))

    def g_caps(self):
        yield from self.byte(0x00, True, 0x01, what='CAPS')    # ISA version 0x01, then thread count << 4 | log2(DEPTH)
        yield from self.gap()
        yield from self.byte(self.junk(), want=4 << 4 | (self.depth.bit_length() - 1), what='CAPS')
        yield from self.gap()
        yield from self.byte(self.junk(), want=0, what='CAPS')

    def unknown(self):
        self.start(self.g_unknown(self.rng.choice(UNKNOWN), self.rng.randrange(5)))

    def g_unknown(self, code, n):
        """Any other code is ignored for the rest of the transaction, a command byte among the rest included."""
        what = 'unknown command %#04x' % code
        yield from self.byte(code, True, 0, what=what)
        for _ in range(n):
            yield from self.gap()
            yield from self.byte(self.rng.choice((self.junk(), 0x3F, 0x28, 0x13, 0x12)), want=0, what=what)

    def cut(self):
        """A command cut before its last argument bit: it does nothing (9.1). STATUS keeps BUSY."""
        rng = self.rng
        k = rng.randrange(6)
        if k == 0:
            g = self.g_post(rng.randrange(8), rng.getrandbits(16), cut=2)
        elif k == 1:
            g = self.g_run(rng.getrandbits(8), rng.getrandbits(8), cut=2)
        elif k == 2:
            g = self.g_step(self.thread8(rng.randrange(4)), cut=1)
        elif k == 3:
            g = self.g_setpc(self.thread8(rng.randrange(4)), rng.getrandbits(16), cut=3)
        elif k == 4:
            g = self.g_status(cut=rng.randrange(1, 5))
        else:
            g = self.g_rdreg(self.thread8(rng.randrange(4)), rng.randrange(32), cut=2)
        if self.pins and rng.random() < 0.5:        # and the last byte cut part-way: CS rises mid-byte
            g = self.cut_mid(g, rng.getrandbits(8), rng.randrange(1, 8))
        self.stats['cut'] += 1
        self.start(g)

    def cut_mid(self, gen, b, bits):
        yield from gen
        yield from self.pins_cut(b, bits)

    def wrmem(self, addr, words):
        self.start(self.g_wrmem(addr, words) if self.full else self.q_wrmem(addr, words[0]))

    def rdmem(self, addr, n):
        self.start(self.g_rdmem(addr, n) if self.full else self.q_rdmem(addr))

    def q_wrmem(self, addr, data):
        what = 'WRMEM %#x' % addr
        a = addr % self.depth
        self.real(self.rc.start_wrmem(a, data), what)
        if self.fails(what):
            return
        done = []
        self.access(self.c, a, 1, data, what, done)
        yield from self.wait(done)

    def q_rdmem(self, addr):
        what = 'RDMEM %#x' % addr
        a = addr % self.depth
        self.real(self.rc.start_rdmem(a), what)
        if self.fails(what):
            return
        done = []
        self.access(self.c, a, 0, 0, what, done)
        yield from self.wait(done)

    def g_wrmem(self, addr, words):
        what = 'WRMEM %#x' % addr
        early = self.pins and self.rng.random() < 0.5     # CS rises before the last word's access lands (9.4)
        yield from self.byte(0x01, True, 0, what=what)
        yield from self.gap()
        yield from self.byte(addr >> 8, want=0, what=what)
        yield from self.gap()
        yield from self.byte(addr & 255, want=0, what=what)
        dead = False
        for k, d in enumerate(words):
            yield from self.gap()
            yield from self.byte(d >> 8, want=0, what=what)
            yield from self.gap()
            done = []

            def act(k=k, d=d, done=done):
                if dead:
                    return True
                if self.fails(what):                # dead until CS rises: no later word is written
                    return True
                self.access(self.c, (addr + k) % self.depth, 1, d, what, done)
                return False
            dead = (yield from self.byte(d & 255, want=0, act=act, what=what))   # h of this word
            if not dead and not (early and k == len(words) - 1):
                yield from self.wait(done)

    def g_rdmem(self, addr, n):
        what = 'RDMEM %#x' % addr
        yield from self.byte(0x02, True, 0, what=what)
        yield from self.gap()
        yield from self.byte(addr >> 8, want=0, what=what)
        yield from self.gap()
        done = []

        def act(k=0, done=done):
            if self.fails(what):
                return True
            self.access(self.c, (addr + k) % self.depth, 0, 0, what, done)
            return False
        dead = yield from self.byte(addr & 255, want=0, act=act, what=what)    # h of the first word
        if self.pins and self.rng.random() < 0.25:  # CS rises at once: the read, or BUSY, still comes (9.4)
            return
        if dead:
            for _ in range(2 + 2 * n):              # dead: every reply byte reads 0
                yield from self.gap()
                yield from self.byte(self.junk(), want=0, what=what)
            return
        v = yield from self.wait(done)
        yield from self.gap()
        yield from self.byte(self.junk(), want=0, what=what)       # byte 3: MISO still 0 in byte 4
        yield from self.gap()
        yield from self.byte(self.junk(), want=v >> 8, what=what)
        for k in range(n):
            yield from self.gap()
            done = []
            yield from self.byte(self.junk(), want=v & 255, act=lambda k=k, done=done: act(k + 1, done),
                                 what=what)         # the low byte starts: h of the next word
            v = yield from self.wait(done)
            if k + 1 < n:
                yield from self.gap()
                yield from self.byte(self.junk(), want=v >> 8, what=what)
        if self.pins:                               # one more byte, to read the last low byte from MISO
            yield from self.byte(self.junk(), what=what)

    def poke_d(self, tid, d):
        """Puts thread tid's D at d on both chips, between cycles (a harness write, the same on both)."""
        t = self.m.th[tid]
        t.D = d
        t.D_WRITTEN = 0                             # it replaces a D that a RUN in this gap wrote
        self.rc.th[tid].D = d


def traffic(rng, m, h, c, st):
    """One host command, or none, for the gap after cycle c."""
    if h.extra and h.mode == 'quick' and targeted(rng, m, h, m.c - 1, st):
        return
    x = rng.random()
    if x < 0.004:
        if any(m.SEL[8:]) or any(t.u.TXRUN or t.u.RXRUN for t in m.th):
            h.abort()
            st['abort'] += 1
            st['abort_aimed'] += 1
            return
    elif x < 0.010:
        stopped = [t.TID for t in m.th if not t.RUN and (t.u.TXRUN or t.u.RXRUN or t.u.RXFULL)]
        if stopped:
            h.rdreg(rng.choice(stopped), rng.randrange(12, 32))
            return
    elif x < 0.013:
        cand = [t.TID for t in m.th if not t.RUN and m.mem[t.PC] >> 12 == 0xD]
        if cand:
            h.step_cmd(rng.choice(cand))
            st['step_unit'] += 1
            return
    elif x < 0.016:
        cand = [t.TID for t in m.th if t.RUN and t.u.TXRUN]
        if cand:
            h.run(1 << rng.choice(cand), 0)
            st['stop_sending'] += 1
            return
    k = rng.randrange(h.rate)
    if k == 0:
        n, v = rng.randrange(8), rng.getrandbits(16)
        h.post(n, v)
        st['post'] += 1
    elif k == 1:
        h.take(rng.randrange(8))
        st['take'] += 1
    elif k == 2:
        h.run(rng.getrandbits(4), 0)
        st['stop'] += 1
    elif k == 3:
        h.run(0, rng.getrandbits(4))
        st['start'] += 1
    elif k == 4 and rng.random() < 0.1:
        h.abort()
        st['abort'] += 1
    elif k == 5:
        h.step_cmd(rng.randrange(4))
        st['step'] += 1
    elif k == 6:
        tid = rng.randrange(4)
        h.setpc(tid, h.region * tid + rng.randrange(128))
        st['setpc'] += 1
    elif k == 7 and rng.random() < 0.3:
        h.status()
        st['status'] += 1
    elif k in (8, 9, 10):
        stopped = [t.TID for t in m.th if not t.RUN]
        if stopped:
            reg = rng.choice((rng.randrange(40), rng.randrange(12, 32), rng.randrange(12, 16)))
            h.rdreg(rng.choice(stopped), reg)
    elif not h.extra:
        pass
    elif k == 11:
        n = rng.randrange(1, 4) if h.full else 1
        h.wrmem(rng.getrandbits(16), [rng.choice(pool()) if rng.random() < 0.9 else rng.getrandbits(16)
                                      for _ in range(n)])
        st['wrmem'] += 1
    elif k == 12:
        h.rdmem(rng.getrandbits(16), rng.randrange(1, 4))
        st['rdmem'] += 1
    elif k == 13:
        h.rdreg(rng.randrange(4), rng.choice((rng.randrange(256), rng.randrange(40))))
        st['rdreg_any'] += 1
    elif k == 14 and h.full:
        h.caps()
        st['caps'] += 1
    elif k == 15 and h.full:
        h.unknown()
        st['unknown'] += 1
    elif k == 16 and h.full:
        h.cut()
    elif k == 17:                               # 9.3: a thread named in both masks is only stopped
        h.run(rng.getrandbits(4), rng.getrandbits(4))
        st['stop_start'] += 1


MAIL_ROW = 0b101001                     # word >> 10 of recv, send, tryrecv and trysend (6.3)


def targeted(rng, m, h, c, st):
    """Corners of the core that random traffic rarely hits, aimed at this cycle (quick mode: a command lands in the
    gap the bench picks). A peek shows the word in stage 4 of this cycle. Returns True if a command went."""
    rc = h.rc
    L, H, F = rc._lib, rc._h, rc._f
    t = m.th[c & 3]
    late0, stepping0 = h.before[c & 3]
    if stepping0 and not late0 and t.LATE and rng.random() < .5:     # a start in the slot where a stepped row sets LATE
        h.run(0, 1 << t.TID)
        st['aimed_late_run'] += 1
        return True
    if L.pe_peek(H, F['valid4_q']):
        word = L.pe_peek(H, F['ir4_q'])
        if word >> 10 == MAIL_ROW and rng.random() < .5:            # the host on the same mailbox word, or another
            n = (word >> 5) & 7
            if rng.random() < .3:
                n = rng.randrange(8)
            if rng.getrandbits(1):
                h.post(n, rng.getrandbits(16))
            else:
                h.take(n)
            st['aimed_mail'] += 1
            return True
        if word >> 12 == 0b1001 and (word >> 5) & 7 == 7 and rng.random() < .06:     # ABORT beside a pcfg
            h.abort()
            st['aimed_abort_pin'] += 1
            return True
        d = asm.decode(word)
        if d and d[0].mn in ('oe', 'oe8', 'put', 'put8', 'smap') and rng.random() < .06:
            h.abort()
            st['aimed_abort_pin'] += 1
            return True
    for t in m.th:
        if t.RUN and not t.STOPREQ:
            d = (t.D - c) & 0xFFFF
            if t.P and t.WAITING and t.TIMED and 1 <= d <= 4 and t.D & 3 == t.TID and rng.random() < .5:
                h.run(1 << t.TID, 0)                 # the stop lands in the slot of the wait's deadline
                st['aimed_stop'] += 1
                return True
            if t.STEPPING and t.MEM and rng.random() < .5:
                h.run(1 << t.TID, 0)                 # a stop during a stepped ld or st
                st['aimed_step'] += 1
                return True
    return False


# ---- one image --------------------------------------------------------------------------------------------------
STATS = ('post', 'take', 'stop', 'start', 'abort', 'abort_aimed', 'step', 'step_unit', 'stop_sending', 'setpc',
         'status', 'rdreg', 'rdreg_unit', 'rdreg_unit_busy', 'rdreg_nonzero', 'tx_busy', 'rx_busy', 'sel_cycles',
         'shift_differs', 'images_light', 'tx_starts', 'rx_words', 'crc_pass', 'stuff_bits', 'ur', 'ovr',
         'rdreg_running', 'rdreg_any', 'wrmem', 'rdmem', 'mem_busy', 'caps', 'unknown', 'cut', 'aimed_late_run',
         'aimed_mail', 'aimed_abort_pin', 'aimed_stop', 'aimed_step', 'stale_poke', 'cycles', 'retired', 'ill',
         'wrmem_busy', 'rdmem_busy', 'resets', 'stop_start')


class Env:
    """ui as drawn; a uio pin the chip drives reads its own drive, the others are drawn (5.2)."""
    ui = io = 0


def reset(rc, m, h, depth, seed, pcs):
    """Section 8 in mid-run: rst_n low for 4 edges on the RTL, then released through its synchroniser, as at power-up
    (the shim's own sequence); sim.py starts again as a new chip with the RTL's memory, which has no reset. Then
    every thread starts again where the image started it. Returns the new sim.py chip."""
    L, H = rc._lib, rc._h
    L.pe_rst(H, 0)
    for _ in range(4):
        L.pe_raw_edge(H)
    L.pe_rst(H, 1)
    for _ in range(2):
        L.pe_raw_edge(H)
    rc.c = 0                                    # the Python side's count, which its host_run takes h from
    m2 = sim.Chip(depth, seed)
    m2.mem[:] = list(rc.mem)
    m2.log = m.log
    for tid in range(4):
        rc.host_setpc(tid, pcs[tid]), rc.host_run(0, 1 << tid)
        m2.host_setpc(tid, pcs[tid]), m2.host_run(0, 1 << tid)
    h.m = m2
    return m2


def run(seed, cycles, depth, gen, mode, extra, st, rows_log=None, release=None):
    """One image. Raises Mismatch at the first difference."""
    rng = random.Random(seed)
    nthreads, hook = 4, None
    if gen == 'prog':
        words, depth, pcs = image_prog(rng), 512, [128 * t for t in range(4)]
    elif gen == 'wide':
        words, depth, pcs = image_wide(rng), 512, [128 * t for t in range(4)]
        reset_at = rng.randrange(500, max(501, cycles - 500)) if extra and rng.random() < 0.25 else None
    elif gen == 'directed':
        (words, nthreads, hook), depth, pcs = image_directed(rng, seed), 512, [128 * t for t in range(4)]
    elif gen == 'rows':
        words = image_rows(rng, depth)
        pcs = [rng.randrange(depth) for _ in range(4)]
        hold = rng.choice((1, 3, 20, 200))
    else:
        words, pcs = image_chips(rng), None
    rc = rtl_chip.Chip(depth, seed, 'pins' if mode == 'pins' else 'stub')
    m = sim.Chip(depth, seed)                   # both fill unwritten memory from random.Random(seed)
    try:
        for a, w in words.items():
            rc.mem[a] = w
            m.mem[a] = w
        if list(rc.mem) != m.mem:
            raise RuntimeError('memory differs before the first cycle')
        if rows_log is not None:
            m.log = rows_log
        view = RtlView(rc)
        for tid in range(nthreads):
            pc = pcs[tid] if pcs else rng.randrange(128)
            rc.host_setpc(tid, pc), rc.host_run(0, 1 << tid)
            m.host_setpc(tid, pc), m.host_run(0, 1 << tid)
        e = Env()
        h = Host(rc, m, rng, st, mode, extra, depth, 128 if gen in ('prog', 'wide', 'directed') else 0)
        if gen != 'wide':
            reset_at = None
        h.rate = rng.choice((60, 80, 400, 2000) if gen == 'rows' else (80, 80, 400, 2000))
        st['images_light'] += h.rate > 80
        last = [(0, 0, 0, 0, 0, 0)] * 4
        f = view.f
        pins = mode == 'pins'
        for c in range(cycles):
            if gen == 'rows':
                if hold == 1 or rng.randrange(hold) == 0:
                    e.ui, e.io = rng.getrandbits(8), rng.getrandbits(8)
            else:
                if rng.random() < 0.3:
                    e.ui = rng.getrandbits(8)
                if rng.random() < 0.3:
                    e.io = rng.getrandbits(8)
            h.before_cycle()
            ui = e.ui
            if pins:                            # ui0 to ui2 are the link: CS, SCK, MOSI
                ui = (ui & 0xF8) | h.link_bits()
            pr, pm = rc.pads(ui), m.pads(ui)
            if pins:                            # uo3 is MISO, which sim.py doesn't model: replies are read from it
                h.uo = pr[0]
                miso = (pr[0] >> 3) & 1
                if h.cs_high == 1:
                    h.miso_rise = miso
                elif h.cs_high in (2, 3) and miso != h.miso_rise:   # 9.1: it returns to 0 three cycles after CS rises
                    raise Mismatch('cycle %d: MISO moved %d cycles after CS rose (9.1)' % (c, h.cs_high - 1))
                elif h.cs_high >= 4 and miso:
                    raise Mismatch('cycle %d: MISO is 1 with CS high for %d cycles (9.1)' % (c, h.cs_high))
                pr, pm = (pr[0] & 0xF7,) + pr[1:], (pm[0] & 0xF7,) + pm[1:]
            if pr != pm:
                raise Mismatch('cycle %d: pads (uo, uio_out, uio_oe): sim.py %s, RTL %s'
                               % (c, tuple('%#04x' % x for x in pm), tuple('%#04x' % x for x in pr)))
            lvl = (pm[1] & pm[2]) | (e.io & ~pm[2] & 0xFF)
            rc.cycle(ui, lvl)
            m.cycle(ui, lvl)
            h.c = c
            h.service()
            if h.busy():                        # a host command lands in cycle c of both: h = c
                h.step()
            elif h.idle() and gen != 'directed':
                traffic(rng, m, h, c, st)
            elif h.idle() and hook:
                hook(m, h, c)
            sm = sim_state(m)
            sr, pw, v = view.state(rc)
            if sm != sr:
                raise Mismatch(first_diff(c, sm, sr))
            pipeline(m.c - 1, m, v, f)      # sim.py's count: a reset in mid-run starts it again
            for i, (q, hh, want) in enumerate(pw):
                if (q, hh) != want:
                    raise Mismatch('cycle %d: unit %d window points PWQ, PWH %s, want %s' % (c, i, (q, hh), want))
            if c % 64 == 63 and list(rc.mem) != m.mem:
                raise Mismatch('cycle %d: memory differs at %s' % (c, [a for a, (x, y) in enumerate(
                    zip(rc.mem, m.mem)) if x != y][:8]))
            if gen == 'rows' and extra and c % 1500 == 1499 and h.idle():
                h.poke_d(rng.randrange(4), ((m.c + rng.randrange(8)) ^ 0x8000) & 0xFFFF)
                st['stale_poke'] += 1
            if reset_at is not None and c >= reset_at and h.idle():
                m = reset(rc, m, h, depth, seed, pcs)   # 8: reset in mid-run, against a new sim.py
                reset_at = None
                st['resets'] += 1
            for i in range(4):                  # what the units did
                u = m.th[i].u
                o = last[i]
                st['tx_busy'] += u.TXRUN or u.TXTL
                st['rx_busy'] += u.RXRUN or u.RXHUNT
                st['tx_starts'] += u.TXRUN and not o[0]
                st['rx_words'] += u.RXFULL and not o[1]
                st['crc_pass'] += (u.RXEF & 1) and not (o[2] & 1)
                st['stuff_bits'] += u.TXSTF and not o[3]
                st['ur'] += u.TXUR and not o[4]
                st['ovr'] += u.RXOVR and not o[5]
                last[i] = (u.TXRUN, u.RXFULL, u.RXEF, u.TXSTF, u.TXUR, u.RXOVR)
            on = [p for p in range(16) if m.SEL[p]]
            st['sel_cycles'] += bool(on)
            for p in on:
                u = m.th[m.OWN[p]].u
                if (u.LA if m.SEL[p] == 1 else u.LB) != m.OUT[p]:
                    st['shift_differs'] += 1     # V(p) differs from OUT[p]: the switch shows
                    break
        if list(rc.mem) != m.mem:
            raise Mismatch('cycle %d: memory differs at the end' % c)
        st['cycles'] += cycles
        st['retired'] += sum(t.retired for t in m.th)
        st['ill'] += sum(t.ILL for t in m.th)
    finally:
        if release:
            release(rc)



# ---- corners: scripted sequences --------------------------------------------------------------------------------
class Pair:
    """One program on both chips at depth 128, stepped together and compared after every cycle as run() compares, for
    a scripted host (--corners). A host call through edge() lands in that cycle through the real command block; one
    through both() between cycles is applied to the RTL's flops by rtl_chip.py, as at the start of an image."""

    def __init__(self, text, release=None):
        img = asm.assemble(text, depth=128)
        self.m, self.rc = sim.Chip(128, 1), rtl_chip.Chip(128, 1)
        for a, w in img.words.items():
            self.m.mem[a] = w
            self.rc.mem[a] = w
        self.view = RtlView(self.rc)
        self.ui = self.io = 0
        self.release = release

    def close(self):
        if self.release:
            self.release(self.rc)

    def set_tb(self, v):
        """Moves TB on both, between cycles, so a case can start just before a corner of the ring."""
        v &= 0xFFFF
        self.rc._set(self.rc._h, self.rc._f['tb_q'], v)
        self.rc.c = self.m.c = v

    def poke_d(self, tid, d):
        t = self.m.th[tid]
        t.D = d
        t.D_WRITTEN = 0
        self.rc.th[tid].D = d

    def check(self):
        sm = sim_state(self.m)
        sr, pw, v = self.view.state(self.rc)
        if sm != sr:
            raise Mismatch(first_diff(self.m.c - 1, sm, sr))
        pipeline(self.m.c - 1, self.m, v, self.view.f)

    def step(self):
        pr, pm = self.rc.pads(self.ui), self.m.pads(self.ui)
        if pr != pm:
            raise Mismatch('cycle %d: pads: sim.py %s, RTL %s' % (self.m.c, pm, pr))
        self.rc.cycle(self.ui, self.io)
        self.m.cycle(self.ui, self.io)

    def run(self, n=1):
        for _ in range(n):
            self.step()
            self.check()

    def both(self, name, *args):
        return getattr(self.rc, name)(*args), getattr(self.m, name)(*args)

    def edge(self, name, *args):
        """One cycle, with the host command landing in it."""
        self.step()
        r = self.both(name, *args)
        self.check()
        return r


def corner_stepld(tid, release):
    """3.6, rule 1: a stop during a stepped ld, with an event a STEP left pending: retire() cancels it."""
    p = Pair('tick 400\nldi R3, 100\nld R2, R3\nldi R4, 7\nhalt\n', release)
    p.run(4)
    p.both('host_setpc', tid, 0)
    for _ in range(2):
        p.both('host_step', tid)
        p.run(12)
    p.both('host_step', tid)
    while not p.m.th[tid].MEM:
        p.run(1)
    p.edge('host_run', 1 << tid, 0)
    p.run(16)
    p.close()
    return 1


def corner_stopslot(tid, release):
    """3.2, rule 2: a stop in the event's own cycle sees the wait as finished: timeout, match and inbox."""
    n = 0
    for way in ('timeout', 'match', 'inbox'):
        for lead in range(1, 6):
            for extra in range(4):
                p = Pair('ldi R1, %d\nnow\nuntil ui5, 1, R1, inbox\nldi R2, 9\nhalt\n' % (40 + extra), release)
                p.run(4)
                p.both('host_setpc', tid, 0)
                p.both('host_run', 0, 1 << tid)
                t = p.m.th[tid]
                while not (t.P and t.WAITING):
                    p.run(1)
                fire = t.D if way == 'timeout' else p.m.c + 8 + extra
                if way == 'match':
                    while p.m.c < fire - 2:
                        p.run(1)
                    p.ui = 0x20
                while p.m.c < fire - lead:
                    p.run(1)
                if way == 'inbox':
                    p.both('host_post', tid, 1)
                p.edge('host_run', 1 << tid, 0)
                p.run(24)
                p.close()
                n += 1
    return n


def corner_stale(tid, release):
    """3.5: STALE sets, a stale D is replaced with LATE, and each way of writing D clears it."""
    n = 0
    for text in ('li R1, 9000\nnow\nl: djnz R1, l\nedge uo1, 1, 20\nrdd R3\ntlate\nhalt\n',
                 'li R1, 9000\nldi R5, 50\nnow\nl: djnz R1, l\nuntil ui5, 1, R5\nrdd R3\ntlate\nhalt\n',
                 'now\nuntil ui5, 1\nnow\nhalt\n'):
        p = Pair('pcfg uo1, PP\n' + text, release)
        p.run(4)
        p.both('host_setpc', tid, 0)
        p.both('host_run', 0, 1 << tid)
        p.run(34000)
        p.ui = 0x20
        p.run(3000)
        p.close()
        n += 1
    return n


def corner_pcfgod(tid, release):
    """5.3: pcfg moves a pin in and out of OD, with OE set, and ABORT lands beside it."""
    n = 0
    for mode in ('IN', 'PP', 'OD'):
        text = ('pcfg io0, PP\npcfg io1, OD\nput io1, 0\nput io0, 1\nnow\nidle\nidle\npcfg io0, OD\npcfg io1, %s\n'
                'halt\n' % mode)
        for delay in range(20, 56):
            p = Pair(text, release)
            p.run(4)
            p.both('host_setpc', tid, 0)
            p.both('host_run', 0, 1 << tid)
            p.run(delay)
            p.edge('host_abort')
            p.run(24)
            p.close()
            n += 1
    return n


def corner_ring(tid, release):
    """3.4 and 7.2 on the corners of the ring: wrd puts D on or just before TB 0x7FFF, 0xFFFF, 0 or 0x8000, tick 0
    fires there, and the world instruction behind it runs in the thread's next slot, so now, tlate, rdtb and a second
    tick run on the corner for each thread. TB starts a little before D."""
    n = 0
    for d in (0x0000, 0x8000, 0xFFFF, 0x7FFF, 0x0001, 0x8001, 0xFFFB, 0x7FFB, 0xFFFC, 0x7FFC, 0xFFFE, 0x7FFE):
        for tail in ('now\ntick 3', 'tlate\nrdtb R3', 'rdtb R3\ntick 1', 'tick 2\nnow'):
            for lead in (60, 63):
                p = Pair('ldi R1, %d\nlui R1, %d\nwrd R1\ntick 0\n%s\nrdd R4\nhalt\n' % (d & 255, d >> 8, tail),
                         release)
                p.run(4)
                p.both('host_setpc', tid, 0)
                p.set_tb(d - lead)
                p.both('host_run', 0, 1 << tid)
                p.run(120)
                p.close()
                n += 1
    return n


def corner_stale4(tid, release):
    """7.1: STALE is read in stage 4. A stale thread in an untimed wait, the match landing at each alignment, so in one
    of them the event that clears STALE is in the cycle just before the slot of the tick behind it."""
    n = 0
    for lead in range(8):
        p = Pair('now\nuntil ui5, 1\ntick 5\nrdd R2\nhalt\n', release)
        p.run(4)
        p.both('host_setpc', tid, 0)
        p.both('host_run', 0, 1 << tid)
        t = p.m.th[tid]
        while not (t.P and t.WAITING):
            p.run(1)
        p.poke_d(tid, ((p.m.c + 10) ^ 0x8000) & 0xFFFF)     # HIT 10 cycles from now, so STALE sets in the wait
        p.run(20 + lead)
        p.ui = 0x20
        p.run(40)
        p.close()
        n += 1
    return n


def corner_steplate(tid, release):
    """9.3 after the slot: a stepped tick on a stale D sets LATE and ends the STEP, and a RUN in that same cycle starts
    the thread and clears LATE. The RUN sweeps the cycles around the slot."""
    n = 0
    for off in range(12):
        p = Pair('tick 5\nrdd R2\nhalt\n', release)
        p.run(4)
        p.both('host_setpc', tid, 0)
        p.poke_d(tid, ((p.m.c + 5) ^ 0x8000) & 0xFFFF)      # HIT in 5 cycles: STALE sets on the stopped thread
        p.run(10)
        p.both('host_step', tid)
        p.run(off)
        p.edge('host_run', 0, 1 << tid)
        p.run(30)
        p.close()
        n += 1
    return n


def corner_boundary(tid, release):
    """3.5 and 7.2: a deadline must be 1 to 32767 cycles after the cycle that arms it, and the test is on that distance,
    not on the step (section 10, defined 2). Steps that put an edge's and a timed wait's deadline on either side of
    the boundary, each run until the event has fired."""
    n = 0
    for text, steps in (('li R1, %d\nnow\nedge -, 0, R1\nrdd R2\ntlate\nhalt\n', range(32760, 32776)),
                        ('li R1, %d\nnow\nuntil ui5, 1, R1\nrdd R2\ntlate\nhalt\n', range(32766, 32782))):
        for step in steps:
            p = Pair(text % step, release)
            p.run(4)
            p.both('host_setpc', tid, 0)
            p.both('host_run', 0, 1 << tid)
            t = p.m.th[tid]
            for _ in range(33200):
                if not t.RUN:
                    break
                p.run(1)
            p.run(8)
            p.close()
            n += 1
    return n


def corner_nowstale(tid, release):
    """3.5: now sets D to the next cycle and leaves LATE alone, whatever STALE is; tlate reads LATE. D is put half the
    ring away so that STALE sets 10 to 80 cycles before the now."""
    n = 0
    for k in (10, 20, 40, 80):
        p = Pair('ldi R1, 30\nl: djnz R1, l\nnow\ntlate\nrdd R2\nhalt\n', release)
        p.run(4)
        p.both('host_setpc', tid, 0)
        p.both('host_run', 0, 1 << tid)
        p.run(1)
        p.poke_d(tid, ((p.m.c + 120 - k) ^ 0x8000) & 0xFFFF)
        p.run(200)
        p.close()
        n += 1
    return n


def corner_abortpin(tid, release):
    """9.3 and 3.2: a host command acts after step 5 of its cycle, so an ABORT in the cycle of an oe8 or a put8 still
    releases every uio pad, and an OD pin's OUT and SHD are 1 after it. ABORT swept over 52 cycles around them."""
    n = 0
    text = ('pcfg io0, PP\npcfg io1, OD\npcfg io2, PP\npcfg io3, OD\nli R1, 0x00FF\nli R2, 0x0000\nnow\nidle\n'
            'oe8 R1\nput8 R2, io\nnow\nhalt\n')
    for delay in range(24, 76):
        p = Pair(text, release)
        p.run(4)
        p.both('host_setpc', tid, 0)
        p.both('host_run', 0, 1 << tid)
        p.run(delay)
        p.edge('host_abort')
        p.run(24)
        p.close()
        n += 1
    return n


RESTRICTED = ('opin', 'iopin', 'lane', 'wmode', 'sdir', 'sout', 'sreg')    # types with illegal values (6.2)


def world_words(rng):
    """For each row of 6.3: 3 words with random legal fields; for the shifts, one word with each of the four pin
    combinations of the i and o fields (the W column's pin rule); and for each field of a type with illegal values,
    a word with one, which section 10 makes illegal."""
    out = []
    for r in asm.ROWS:
        def fields(**fix):
            while True:
                f = {k: rng.getrandbits(len(b)) for k, b in r.bits.items()}
                f.update(fix)
                if all(asm.legal_value(f[k], r.ftypes[k]) for k in f):
                    return f
        for _ in range(3):
            out.append(asm.encode(r, fields()))
        if r.w == 'pin':
            for i, o in ((0, 3), (5, 3), (0, 1), (5, 1)):
                out.append(asm.encode(r, fields(i=i, o=o)))
        for k, ty in r.ftypes.items():
            if ty in RESTRICTED:
                bad = [v for v in range(1 << len(r.bits[k])) if not asm.legal_value(v, ty)]
                f = fields()
                f[k] = rng.choice(bad)
                w = asm.encode(r, f)
                assert asm.decode(w) is None, (r.syntax, k)
                out.append(w)
    return out


def corner_world(tid, release):
    """3.6 and 6.3: every row behind a tick that is still pending: a world row waits for the tick, any other runs at
    once (and the shifts are world if they name a pin); a word with a field value its type forbids is illegal
    (section 10): ILL, and the thread stops on it."""
    n = 0
    for w in world_words(random.Random(tid)):
        p = Pair('now\ntick 30\n.data %d\nhalt\n' % w, release)
        p.run(4)
        p.both('host_setpc', tid, 0)
        p.both('host_run', 0, 1 << tid)
        p.run(80)
        p.close()
        n += 1
    return n


CORNERS = (corner_stepld, corner_stopslot, corner_stale, corner_pcfgod, corner_ring, corner_stale4, corner_steplate,
           corner_boundary, corner_nowstale, corner_world, corner_abortpin)


def corner_job(args):
    """One block of corners on one thread. Returns (name, tid, cases, why), the stats and the coverage."""
    name, tid, opts = args
    fc = vc = release = None
    if opts.get('fcov'):
        import fcov
        fc = fcov.process_coverage()
        fc.reset()
    if opts.get('cov'):
        import vlcov
        vc = vlcov.collector()
        release = vc.release
    fn = dict((f.__name__, f) for f in CORNERS)[name]
    try:
        n, why = fn(tid, release), None
    except Mismatch as ex:
        n, why = 0, str(ex)
    except Exception as ex:
        import traceback
        n, why = 0, 'bench fault: %s: %s\n%s' % (type(ex).__name__, ex, traceback.format_exc()[-1500:])
    return (name, tid, n, why), (fc.snapshot() if fc else None), (vc.harvest_all() if vc else None)


# ---- runs ---------------------------------------------------------------------------------------------------------
def job(args):
    """One image, or with code coverage a list of them. Returns per image (seed, depth, mode, why, secs), and the
    stats, rows, functional and code coverage summed."""
    seeds, cycles, depths, gen, host, extra, opts = args
    st = dict.fromkeys(STATS, 0)
    rows = {}
    fc = vc = None
    if opts.get('fcov'):
        import fcov
        fc = fcov.process_coverage()            # the hooks go in once per process
        fc.reset()
    release = None
    if opts.get('cov'):
        import vlcov
        vc = vlcov.collector()
        release = vc.release
    out = []
    for seed, depth in zip(seeds, depths):
        mode = host if host != 'mixed' else ('full' if seed % 2 else 'quick')
        t0 = time.time()
        log = []
        try:
            run(seed, cycles, depth, gen, mode, extra, st, log, release)
            why = None
        except Mismatch as ex:
            why = str(ex)
        except Exception as ex:                 # a fault of the bench: reported, and counted as a difference
            import traceback
            why = 'bench fault: %s: %s\n%s' % (type(ex).__name__, ex, traceback.format_exc()[-1500:])
        if why:
            why += ('\n    rerun: verif/lockstep.py --gen %s --first %d --images 1 --cycles %d --depths %d --host %s%s'
                    % (gen, seed, cycles, depth, mode, '' if extra else ' --no-extra'))
        for _, tid, _, w in log:
            r = asm.decode(w)[0].syntax
            rows[r] = rows.get(r, 0) + 1
        out.append((seed, depth, mode, why, time.time() - t0))
    if fc:                                      # sim.py serves these in h; the bench sets its BUSY in h + 1
        fc.hit('host: WRMEM with every thread running', st['wrmem_busy'])
        fc.hit('host: RDMEM with every thread running', st['rdmem_busy'])
    return out, st, rows, (fc.snapshot() if fc else None), (vc.harvest_all() if vc else None)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--gen', default='chips', choices=('chips', 'prog', 'rows', 'wide', 'directed'))
    ap.add_argument('--images', type=int, default=200)
    ap.add_argument('--cycles', type=int, default=6000)
    ap.add_argument('--first', type=int, default=0, help='first seed')
    ap.add_argument('--depths', default='128,512', help='chips and rows: depths taken in turn (prog is 512)')
    ap.add_argument('--host', default='mixed', choices=('quick', 'full', 'mixed', 'pins'))
    ap.add_argument('--no-extra', action='store_true')
    ap.add_argument('--jobs', type=int, default=min(8, os.cpu_count() or 1))
    ap.add_argument('--build', default=None, help='the models, default verif/build or $PE_BUILD')
    ap.add_argument('--stop-first', action='store_true')
    ap.add_argument('--out', default=None, help='also write the summary here')
    ap.add_argument('--fcov', default=None, help='functional coverage of sim.py, as JSON')
    ap.add_argument('--cov', default=None, help='the RTL\'s line and toggle counts (coverage models only)')
    ap.add_argument('--corners', action='store_true', help='the scripted corner cases instead of images')
    a = ap.parse_args(argv)
    if a.build:
        os.environ['PE_BUILD'] = os.path.abspath(a.build)
        rtl_chip.BUILD = os.environ['PE_BUILD']
    if a.corners:
        return corners_main(a)
    depths = [512] if a.gen in ('prog', 'wide', 'directed') else [int(x) for x in a.depths.split(',')]
    for d in depths:
        rtl_chip._load('pins' if a.host == 'pins' else 'stub', d)     # fail early if a model is missing
    pool()
    seeds = list(range(a.first, a.first + a.images))
    dl = [depths[i % len(depths)] for i in range(len(seeds))]
    opts = {'fcov': bool(a.fcov), 'cov': bool(a.cov)}
    per = 1
    if a.cov:                                   # code coverage adds up in each worker: a few images per job
        per = max(1, -(-len(seeds) // (a.jobs * 4)))
    work = [(seeds[i:i + per], a.cycles, dl[i:i + per], a.gen, a.host, not a.no_extra, opts)
            for i in range(0, len(seeds), per)]
    t0 = time.time()
    total = dict.fromkeys(STATS, 0)
    rows, fcounts, vcounts = {}, {}, None
    bad, lines, done, modes = [], [], 0, {'quick': 0, 'full': 0, 'pins': 0}
    with concurrent.futures.ProcessPoolExecutor(a.jobs) as ex:
        futs = [ex.submit(job, w) for w in work]
        for fu in concurrent.futures.as_completed(futs):
            out, st, rw, fc, vc = fu.result()
            for k in STATS:
                total[k] += st[k]
            for k, v in rw.items():
                rows[k] = rows.get(k, 0) + v
            if fc:
                for k, v in fc.items():
                    fcounts[k] = fcounts.get(k, 0) + v
            if vc:
                import vlcov
                vcounts = vlcov.add(vcounts, vc)
            for seed, depth, mode, why, secs in out:
                done += 1
                modes[mode] += 1
                if why:
                    bad.append((seed, depth, mode, why))
                    lines.append('FAIL image %d: %s' % (seed, why))
                    print(lines[-1], flush=True)
            if bad and a.stop_first:
                for f2 in futs:
                    f2.cancel()
                break
    unit = {k: v for k, v in rows.items() if k.split()[0] in UNIT_MN}
    out = ['generator %s: %d images x %d cycles = %d cycles compared (depths %s; host quick %d, full %d, pins %d%s), '
           '%.0f s, %d jobs' % (a.gen, done, a.cycles, total['cycles'], ','.join(map(str, depths)), modes['quick'],
                                modes['full'], modes['pins'], ', no extras' if a.no_extra else '', time.time() - t0,
                                a.jobs),
           'host: ' + ', '.join('%s %d' % (k, total[k]) for k in STATS[:15]),
           'units: cycles with a send busy %d, a receive busy %d (summed over units); cycles with a pin on a shift '
           'output %d, of them showing a level other than OUT %d' % (total['tx_busy'], total['rx_busy'],
                                                                    total['sel_cycles'], total['shift_differs']),
           'events: sends started %d, words received %d, CRC passes (end flag 0) %d, stuff bits sent %d, underruns %d, '
           'overruns %d; images with light or sparse host traffic %d'
           % (total['tx_starts'], total['rx_words'], total['crc_pass'], total['stuff_bits'], total['ur'], total['ovr'],
              total['images_light']),
           'more host: ' + ', '.join('%s %d' % (k, total[k]) for k in STATS[26:40] + ('resets', 'stop_start')),
           'sim.py: instructions retired %d, threads stopped by an illegal word %d; shift-unit rows it ran (its '
           'execute log, a blocked txw or rxw counted at every slot it retries): %s; rows it ran: %d of %d'
           % (total['retired'], total['ill'], {k: unit[k] for k in sorted(unit)}, len(rows), len(asm.ROWS)),
           '%d images differ%s' % (len(bad), ', first %s' % (bad[0],) if bad else '')]
    lines += out
    print('\n'.join(out))
    if a.out:
        with open(a.out, 'w') as fh:
            fh.write('\n'.join(lines) + '\n')
    if a.fcov:
        with open(a.fcov, 'w') as fh:
            json.dump({'run': ' '.join(sys.argv[1:] if argv is None else argv), 'counts': fcounts}, fh)
    if a.cov and vcounts:
        import vlcov
        vlcov.write(a.cov, vcounts)
    return 1 if bad else 0


def corners_main(a):
    """--corners: every block of scripted cases on every thread, in parallel. Exit 0 only if none differs."""
    rtl_chip._load('stub', 128)
    opts = {'fcov': bool(a.fcov), 'cov': bool(a.cov)}
    work = [(f.__name__, tid, opts) for f in CORNERS for tid in range(4)]
    t0 = time.time()
    cases, bad, fcounts, vcounts = 0, [], {}, None
    with concurrent.futures.ProcessPoolExecutor(a.jobs) as ex:
        futs = [ex.submit(corner_job, w) for w in work]
        for fu in concurrent.futures.as_completed(futs):
            (name, tid, n, why), fc, vc = fu.result()
            cases += n
            for k, v in (fc or {}).items():
                fcounts[k] = fcounts.get(k, 0) + v
            if vc:
                import vlcov
                vcounts = vlcov.add(vcounts, vc)
            if why:
                bad.append((name, tid, why))
                print('FAIL corners %s, thread %d: %s' % (name, tid, why), flush=True)
                if a.stop_first:
                    for f2 in futs:
                        f2.cancel()
                    break
    out = ['corners: %d blocks on 4 threads, %d cases, %.0f s, %d jobs' % (len(CORNERS), cases, time.time() - t0,
                                                                          a.jobs),
           '%d blocks differ%s' % (len(bad), ', first %s' % (bad[0],) if bad else '')]
    print('\n'.join(out))
    if a.out:
        with open(a.out, 'w') as fh:
            fh.write('\n'.join(['FAIL corners %s, thread %d: %s' % b for b in bad] + out) + '\n')
    if a.fcov:
        with open(a.fcov, 'w') as fh:
            json.dump({'run': 'corners', 'counts': fcounts}, fh)
    if a.cov and vcounts:
        import vlcov
        vlcov.write(a.cov, vcounts)
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
