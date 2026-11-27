"""Bench for advanced/sd_unit.s: an SD host in SD mode on two shift units, against bench/sd_card.py's card, which
follows the SD Physical Layer Simplified Specification and was written before the firmware.

The host side here plays the RP2350: it posts each command's five words, takes the two status words, and reads the
response and the block from memory with RDMEM (section 9.2). The judge compares what the chip reports with what the
card says it received and sent: every command arrives with a good CRC7, transmitter bit and end bit; every response
the card sends is stored bit for bit after its start bit (the start bit is the receive half's sync pattern, so the
47 bits from the transmission bit, or R2's 135), with the chip's CRC7 check set exactly when the reference's own CRC7
of the token holds (R3 has none); every block arrives whole, with the chip's CRC16 check set exactly when the block
on the wire is good. On the bus: the chip and the card never drive CMD or DAT0 at once, the power-up has 74 clocks
with CMD high, every command comes at least 8 clocks after the last end bit on CMD, and LATE and ILL stay clear.

The card's timing is swept over what the SD Physical Layer Simplified Specification allows and past it: NCR, the
clocks from a command's end bit to its response's start bit, over every value from 2 to 64 (4.12, blank in the
simplified text, gives 2 to 64 in the full specification); NAC, the clocks from a read command's end bit to the
block's start bit, from 2 (the block starts during the response) to the 100 ms read time-out of 4.6.2.1, through the
fixed 1 ms access time of a CSD version 2.0 card (5.3, TAAC) and NSAC's largest 25.5 k clocks; and the card's output
delay d after the falling edge over every whole cycle of a clock period, 0 to 2H, where 6.6.6 allows 14 ns at
default speed. The model reads a change in the first cycle at or after it, so whole cycles give every phase the chip
can tell apart, the cycle of a rising edge among them (d in (H - 1, H]).
"""
import random

from report import assemble, check
from simple import Chip, read
import sd_card as SD

PROGRAMS = ('advanced/sd_unit.s',)
PROG = 'advanced/sd_unit.s'
CLK = 50_000_000
MOUT, MBLK, MCFG = 1, 5, 7
RBUF, DCNT, DBUF = 240, 254, 255
NONE, R48, R136 = 0, 1, 2
RCA = 0xB368

# The initialization of 4.2 as far as the card model goes, then a read: (index, argument, response, block).
INIT = [(0, 0, NONE, 0), (8, 0x1AA, R48, 0), (55, 0, R48, 0), (41, 0x40FF8000, R48, 0), (2, 0, R136, 0),
        (3, 0, R48, 0)]
SELECT = (7, RCA << 16, R48, 0)


def read_cmd(block):
    return (17, block, R48, 1)


class Result:
    pass


def run(cmds, card=None, h0=63, h=None, ta=0, tb=1, align=0, lim=None, text=None, defs=None, fill=0xDEAD,
        cycles=None):
    """Runs the commands in order, each posted after the result of the one before. h: the H of each command (a
    number for all, or a list). lim: the time limit of each command in words of 16 clocks after its end bit."""
    img = assemble(text or read(PROG), defines=defs, depth=512)
    chip = Chip(img.words, 512)
    card = card or SD.Card()
    bus = SD.Bus(card)
    for a in range(DBUF, 512):
        chip.sim.host_wrmem(a, fill)
    while chip.c % 4 != align:
        chip.cycle(bus)
    chip.sim.host_setpc(ta, img.labels['cmdth'])
    chip.sim.host_setpc(tb, img.labels['datth'])
    chip.sim.host_run(0, (1 << ta) | (1 << tb))
    hs = list(h) if isinstance(h, (list, tuple)) else [h or h0] * len(cmds)
    posts = [h0]
    res = []
    out = []
    i = 0
    stop = cycles or 3_000_000

    def queue(k):
        index, arg, kind, blk = cmds[k]
        word = index | kind << 6 | blk << 8
        limit = lim if lim is not None else ((300 + card.nac // 16) if blk else 15)
        posts.extend([hs[k], word, arg >> 16, arg & 0xFFFF, limit])

    if cmds:
        queue(0)
    while chip.c < stop:
        if posts and chip.post(MCFG, posts[0]):
            posts.pop(0)
        chip.cycle(bus)
        v = chip.take(MOUT)
        if v is not None:
            out.append(v)
            if len(out) == 2:
                st, bs = out
                out = []
                n = st >> 8
                words = [chip.sim.host_rdmem(RBUF + j) for j in range(n)]
                blk = [chip.sim.host_rdmem(DBUF + j) for j in range(257)] if bs else None
                dcnt = chip.sim.host_rdmem(DCNT) if bs else None
                res.append(dict(status=st, bstat=bs, words=words, block=blk, dcnt=dcnt, cycle=chip.c))
                if bs:
                    for a in range(DBUF, 512):
                        chip.sim.host_wrmem(a, fill)
                i += 1
                if i < len(cmds):
                    queue(i)
                else:
                    stop = min(stop, chip.c + 400)
    r = Result()
    r.chip, r.card, r.bus, r.res, r.img, r.cycles = chip, card, bus, res, img, chip.c
    r.late = [t.id for t in chip.t if t.late or t.th.ILL]
    r.h, r.cmds = hs, cmds
    return r


def token_bits(words, n):
    """n bits of a response stored as words, most significant bit first; the last word holds the rest in its low
    bits (section 5.9.6, Words)."""
    bits = []
    for j, w in enumerate(words):
        k = min(16, n - 16 * j)
        bits += [(w >> (k - 1 - i)) & 1 for i in range(k)]
    return bits


def judge(r):
    """Every failure found, as text. Empty means the run is good."""
    bad = []
    card, bus = r.card, r.bus
    if len(r.res) != len(r.cmds):
        bad.append('%d of %d commands reported' % (len(r.res), len(r.cmds)))
    seen = [(i, a) for i, a, ok in card.cmd_seen]
    want = [(i, a) for i, a, _, _ in r.cmds]
    if seen != want:
        bad.append('the card saw %s, the host sent %s' % (seen, want))
    if not all(ok for _, _, ok in card.cmd_seen):
        bad.append('a command failed the card\'s CRC7, transmitter or end bit check')
    for n, (cmd, got) in enumerate(zip(r.cmds, r.res)):
        index, arg, kind, blk = cmd
        rec = card.per_cmd[n] if n < len(card.per_cmd) else None
        resp = rec['resp'] if rec else None
        st = got['status']
        if resp is None:
            if st & 0x10 or st >> 8:
                bad.append('CMD%d: no response was sent, the chip reports 0x%04X' % (index, st))
        else:
            nbits = len(resp)
            want_words = 9 if nbits == 136 else 3
            stored = token_bits(got['words'], nbits - 1)
            if not st & 0x10 or st >> 8 != want_words:
                bad.append('CMD%d: response status 0x%04X' % (index, st))
            elif resp[0] != 0 or stored != resp[1:]:
                bad.append('CMD%d: response stored differs from the card\'s' % index)
            good = SD.value(resp[-8:-1]) == SD.crc7(resp[8:-8] if nbits == 136 else resp[:40]) and resp[-1] == 1
            if bool(st & 1) != good:
                bad.append('CMD%d: the chip\'s check says %d, the reference says %d' % (index, st & 1, good))
        blk_sent = rec['block'] if rec else None
        if blk_sent is not None:
            bs = got['bstat']
            wire = SD.data_block(blk_sent)
            for (bn, bit), _ in card.flip.items():
                if bn == arg:
                    wire[1 + bit] ^= 1
            if card.bad_end:
                wire[-1] = 0
            good = SD.crc16(wire[1:4097]) == SD.value(wire[4097:4113]) and wire[4113] == 1
            data_words = [SD.value(wire[1 + 16 * j:17 + 16 * j]) for j in range(257)]
            if not bs or not bs & 0x100 or (bs >> 3) & 31 != 1:
                bad.append('CMD%d: block status 0x%04X' % (index, bs or 0))
            elif got['block'] != data_words:
                k = next(j for j in range(257) if got['block'][j] != data_words[j])
                bad.append('CMD%d: block word %d is 0x%04X, the card sent 0x%04X' % (index, k, got['block'][k],
                                                                                     data_words[k]))
            elif got['dcnt'] != 257:
                bad.append('CMD%d: thread B counted %s words' % (index, got['dcnt']))
            if bs and bool(bs & 1) != good:
                bad.append('CMD%d: the chip\'s CRC16 check says %d, the reference says %d' % (index, bs & 1, good))
        elif got['bstat']:
            bad.append('CMD%d: a block status 0x%04X with no block sent' % (index, got['bstat']))
    if bus.both or bus.both_dat:
        bad.append('chip and card drive one line at once on %d cycles (CMD) and %d (DAT0), first at %s'
                   % (bus.both, bus.both_dat, bus.first_both))
    if card.faults:
        bad.append(card.faults[0])
    if card.first_cmd is None or card.first_cmd[1] < 74:
        bad.append('power-up clocks with CMD high before the first command: %s'
                   % (card.first_cmd[1] if card.first_cmd else None))
    if r.late:
        bad.append('LATE or ILL on thread(s) %s' % r.late)
    return bad


def margins(r):
    """From the bus log: the least CMD setup and hold at the card's rising edges while the chip drives CMD, the
    clock's least high and low time, and the least gap between one side releasing CMD and the other driving it."""
    log = r.bus.log
    clk_rise = [c for c, n, v in log if n == 'clk' and v == 1]
    cmd_ch = [c for c, n, v in log if n == 'cmd']
    drive = r.bus.chip_cmd
    su, ho = 1e9, 1e9
    j = 0

    def chip_drives(c):
        on = 0
        for t, v in drive:
            if t > c:
                break
            on = v
        return on

    for c in clk_rise:
        if not chip_drives(c):
            continue
        while j < len(cmd_ch) and cmd_ch[j] <= c:
            j += 1
        prev = cmd_ch[j - 1] if j else -1e9
        nxt = cmd_ch[j] if j < len(cmd_ch) else 1e9
        su, ho = min(su, c - prev), min(ho, nxt - c)
    return dict(setup=su, hold=ho, high=min(r.card.highs or [0]), low=min(r.card.lows or [0]),
                period=min(r.card.periods or [0]))


def sweep(cmds, hs, card_kw, pairs=((0, 1),), aligns=(0,), h0=None):
    """Runs every thread pair and alignment; returns (failures, runs)."""
    bad, n = [], 0
    for ta, tb in pairs:
        for al in aligns:
            r = run(cmds, card=SD.Card(**card_kw), h0=h0 or (hs[0] if isinstance(hs, list) else hs), h=hs,
                    ta=ta, tb=tb, align=al)
            n += 1
            e = judge(r)
            if e:
                bad.append(((ta, tb), al, e[0]))
    return bad, n


def fmt(bad, n):
    return '%d of %d runs%s' % (n - len(bad), n, ('; ' + str(bad[0])) if bad else '')


PAIRS = [(a, b) for a in range(4) for b in range(4) if a != b]


def check_unit(log):
    log.group(PROG, title='SD host in SD mode on two shift units, against a card model from the SD Physical Layer '
              'Simplified Specification')
    bad = SD.self_test()
    check('reference: CRC7 and CRC16 by long division reproduce 4.5\'s examples (CMD0 ends 0x95, CMD17, CMD17\'s '
          'response, 512 bytes of 0xFF give 0x7FA1) and CMD8 0x1AA ending 0x87', not bad, bad)
    # The initialization (4.2) at 400 kHz, then CMD7 and a read at a data-transfer clock, as a host does.
    cmds = INIT + [SELECT, read_cmd(5)]
    hs = [63] * len(INIT) + [4, 4]
    for (ta, tb), al in (((0, 1), 0), ((1, 2), 1), ((2, 3), 2), ((3, 0), 3)):
        r = run(cmds, card=SD.Card(ncr=2, nac=20), h0=63, h=hs, ta=ta, tb=tb, align=al)
        e = judge(r)
        m = margins(r)
        st = [x['status'] for x in r.res]
        check('threads %d (CMD, CLK) and %d (DAT0), alignment %d: CMD0, CMD8 (R7), CMD55 + ACMD41 (R3), CMD2 (R2), '
              'CMD3 (R6) at H 63 (397 kHz), then CMD7 and CMD17 with its block at H 4 (6.25 MHz): every command, '
              'response and block as the card sent it; 74 clocks first; 8 clocks between commands; no contention'
              % (ta, tb, al), not e and r.card.first_cmd[1] >= 74,
              (e[0] if e else 'statuses %s; power-up clocks %d; CMD setup %d and hold %d cycles at the card' % (
                  ['%04X' % x for x in st], r.card.first_cmd[1], m['setup'], m['hold'])))
    resp = token_bits(r.res[-1]['words'], 47)
    check('CMD17\'s response is stored as 4.5\'s example gives it after the start bit, 0 010001 0x00000900 0110011 1, '
          'with the chip\'s CRC7 check set', resp == SD.response48(17, 0x900)[1:] and r.res[-1]['status'] & 1, resp)
    check('ACMD41\'s R3 has no CRC (4.9.4): stored whole, and the chip\'s CRC7 check is clear', not judge(r) and
          not r.res[3]['status'] & 1 and r.res[3]['status'] & 0x10, '0x%04X' % r.res[3]['status'])
    check('CMD2\'s R2 (136 bits): 9 words; the start bit is the sync pattern and CSKIP 7 leaves the transmission and '
          '111111 bits out, so the check covers CID bits 127:1 and the end bit', r.res[4]['status'] == 0x0911,
          '0x%04X' % r.res[4]['status'])
    # Reads on every thread pair, at three clocks.
    bad, n = [], 0
    for k, (ta, tb) in enumerate(PAIRS):
        b, m = sweep([read_cmd(9)], 3, dict(state='tran', ncr=2, nac=30), pairs=((ta, tb),), aligns=(k % 4,))
        bad += b
        n += m
    check('CMD17 and its block at H 3 (8.33 MHz) on all 12 thread pairs, each alignment 3 times', not bad,
          fmt(bad, n))
    for h, pairs, aligns in ((4, PAIRS[::3], (0, 1, 2, 3)), (25, PAIRS[::5], (0, 3))):
        res = sweep([read_cmd(9)], h, dict(state='tran', ncr=2, nac=30), pairs=pairs, aligns=aligns)
        check('CMD17 and its block at H %d (%.2f MHz) on %d thread pairs x %d alignments' % (
            h, CLK / 2e6 / h, len(pairs), len(aligns)), not res[0], fmt(*res))
    # NCR across 2 to 64 clocks (SD 4.12, not in the simplified text).
    for h in (3, 8):
        bad, n = [], 0
        for ncr in (2, 3, 7, 16, 33, 64):
            b, k = sweep([read_cmd(3)], h, dict(state='tran', ncr=ncr, nac=80), aligns=(0, 1, 2, 3))
            bad += [(ncr,) + x for x in b]
            n += k
        check('NCR 2, 3, 7, 16, 33 and 64 clocks at H %d, 4 alignments each: the reception finds the response '
              'wherever it starts' % h, not bad, fmt(bad, n))
    # NAC: a block that starts while the response is still on CMD, and long waits.
    bad, n = [], 0
    for nac in (2, 5, 20, 45, 200, 1000):
        for ncr in (2, 64):
            b, k = sweep([read_cmd(4)], 3, dict(state='tran', ncr=ncr, nac=nac), aligns=(0, 2))
            bad += [(nac, ncr) + x for x in b]
            n += k
    check('NAC 2, 5, 20 and 45 (the block starts during the response) and 200 and 1000 clocks, with NCR 2 and 64, '
          'at H 3: both receive halves run at once on their own lines', not bad, fmt(bad, n))
    # The card's output delay after the falling edge (SD 6.6.6: 14 ns at default speed, plus the board): sampled at
    # the next rising edge up to H cycles, and a clock later past H, up to 2H. check_timing covers every whole cycle,
    # the cycle of the rising edge (d in (H - 1, H]) included.
    for h in (3, 8):
        bad, n = [], 0
        for d in (0, 0.5, 1.25, h / 2, h - 1, h + 0.25, 1.5 * h, 2 * h):
            b, k = sweep([read_cmd(2)], h, dict(state='tran', ncr=5, nac=20, d=d), aligns=(0, 1, 2, 3))
            bad += [(d,) + x for x in b]
            n += k
        check('card output delay 0 to H - 1 cycles, and H + 0.25 to 2H, after the falling edge at H %d, 4 '
              'alignments: response and block right' % h, not bad, fmt(bad, n))
    # 4.5's CRC16 example on the wire.
    r = run([read_cmd(1)], card=SD.Card(state='tran', blocks={1: [0xFF] * 512}), h0=4, h=4)
    got = r.res[0]['block'] if r.res else None
    check('a block of 512 bytes of 0xFF: the chip stores the CRC16 word 0x7FA1 (4.5\'s example) and its check '
          'passes', got and got[256] == 0x7FA1 and r.res[0]['bstat'] & 1 and not judge(r),
          '0x%04X' % got[256] if got else None)
    # Two blocks back to back: thread B hunts again and restores RXCNT.
    r = run([read_cmd(11), read_cmd(12)], card=SD.Card(state='tran', nac=9), h0=3, h=3)
    check('two reads back to back at H 3: both blocks whole and checked', not judge(r) and len(r.res) == 2,
          judge(r))


def check_faults(log):
    log.group(PROG, title='faults on the wire: the chip\'s checks against the reference\'s')
    r = run([read_cmd(6)], card=SD.Card(state='tran', flip={(6, 1000): 1}), h0=4, h=4)
    check('one data bit flipped on DAT0 after the card made its CRC16: the chip stores the block as on the wire '
          'and its CRC16 check is clear', not judge(r) and r.res and not r.res[0]['bstat'] & 1,
          judge(r) or '0x%04X' % r.res[0]['bstat'])
    r = run([read_cmd(6)], card=SD.Card(state='tran', bad_end=True), h0=4, h=4)
    check('the block\'s end bit sent as 0: the check, which includes the end bit (CHK = POLY), is clear',
          not judge(r) and r.res and not r.res[0]['bstat'] & 1, judge(r) or '0x%04X' % r.res[0]['bstat'])
    for bit, what in ((44, 'a CRC7 bit'), (20, 'an argument bit'), (47, 'the end bit')):
        r = run([read_cmd(6)], card=SD.Card(state='tran', resp_flip={17: bit}), h0=4, h=4)
        check('CMD17\'s response with %s flipped: stored as sent, CRC7 check clear' % what,
              not judge(r) and r.res and not r.res[0]['status'] & 1, judge(r) or '0x%04X' % r.res[0]['status'])
    r = run([INIT[0], INIT[1]], card=SD.Card(v2=False), h0=63, h=63)
    check('a card that rejects CMD8 (no R7, 4.3.13): the time limit ends the wait, status 0', not judge(r) and
          len(r.res) == 2 and r.res[1]['status'] == 0, judge(r) or ['%04X' % x['status'] for x in r.res])


def check_limits(log):
    log.group(PROG, title='limits: the fastest clock and the latest card output')
    good = sweep([read_cmd(8)], 3, dict(state='tran', ncr=2, nac=20), pairs=PAIRS[:4], aligns=(0, 1, 2, 3))
    r = run([read_cmd(8)], card=SD.Card(state='tran', ncr=2, nac=20), h0=2, h=2)
    st = r.res[0]['status'] if r.res else None
    check('H 3 (8.33 MHz) is the fastest clock thread A keeps fed on 16 of 16 runs; at H 2 (12.5 MHz) its loop '
          'falls behind and the firmware reports the stopped clock (status bit 5) instead of hanging',
          not good[0] and st is not None and st & 0x20, '%s; at H 2 status 0x%04X' % (fmt(*good), st or 0))
    bad, n = [], 0
    for d in (3.25, 3.75, 4.0):
        b, k = sweep([read_cmd(2)], 4, dict(state='tran', ncr=5, nac=20, d=d), aligns=(0, 1, 2, 3))
        bad += b
        n += k
    check('a card output delay in (H - 1, H] cycles at H 4, on every alignment: the start bits of the response and the '
          'block reach SYNC2 in the cycle of a rising CLK edge, where a hunt would start its reception and take no '
          'sample (5.9.6, Start); the receptions here started before them, so that edge samples each start bit',
          not bad, fmt(bad, n))

def start_edges(r, line):
    """From the bus log, each start bit the card put on CMD or DAT0 ('cmd' or 'dat'): (cycles after the last falling
    edge of CLK, cycles after the last rising edge), at the cycle the line fell while the chip wasn't driving it."""
    out, fall, rise, lv = [], None, None, None
    for c, name, v in r.bus.log:
        if name == 'clk':
            if v:
                rise = c
            else:
                fall = c
        elif name == line:
            drv = 0
            for t, on in r.bus.chip_cmd if line == 'cmd' else ():
                if t > c:
                    break
                drv = on
            if lv == 1 and v == 0 and not drv and fall is not None:
                out.append((c - fall, c - rise if rise is not None else None))
            lv = v
    return out


def grid(cmds, h, points, card_kw, cycles=None, line='cmd'):
    """One run per point (ncr, nac, d), with the thread pairs and alignments in turn; returns (failures, runs, the
    offsets seen). Each run also measures where the card's first start bit on `line` fell: the cycle after the last
    falling edge must be the first whole cycle at or after d (the model's rule), and a d in (H - 1, H] must put it in
    a rising edge's own cycle. So the sweep is shown to reach every phase it names."""
    bad, seen = [], set()
    for k, (ncr, nac, d) in enumerate(points):
        ta, tb = PAIRS[k % len(PAIRS)]
        kw = dict(card_kw, ncr=ncr, nac=nac, d=d)
        r = run(cmds, card=SD.Card(**kw), h0=h, h=h, ta=ta, tb=tb, align=k % 4, cycles=cycles)
        e = judge(r)
        edges = start_edges(r, line)
        want = -int(-d // 1) % (2 * h)
        if edges:
            seen.add(edges[0][0])
        if not edges:
            e = e + ['no start bit found on %s' % line]
        elif edges[0][0] != want or ((h - 1 < d <= h) != (edges[0][1] == 0)):
            e = e + ['the start bit on %s came %s cycles after a falling edge and %s after a rising one, for d %s' % (
                line, edges[0][0], edges[0][1], d)]
        if e:
            bad.append(((ncr, nac, d), (ta, tb), k % 4, e[0]))
    return bad, len(points), seen


def every_phase(res, h):
    """A grid result, its failures and runs, and whether its start bits fell at all 2H cycle offsets of a period."""
    bad, n, seen = res
    missing = sorted(set(range(2 * h)) - seen)
    return not bad and not missing, fmt(bad, n) + ('; offsets never reached: %s' % missing if missing else
                                                  '; start bits at all %d cycle offsets' % (2 * h))


def check_timing(log):
    log.group(PROG, title='the card\'s timing over the specification\'s range and every phase: NCR, NAC and the output '
              'delay')
    r1 = [(55, 0, R48, 0)]                      # CMD55 in transfer state: an R1 and no block
    for h in (3, 4, 8):
        pts = [(ncr, 8, d) for ncr in range(2, 65) for d in range(2 * h + 1)]
        ok, text = every_phase(grid(r1, h, pts, dict(state='tran')), h)
        check('every NCR from 2 to 64 clocks x every card output delay from 0 to 2H cycles at H %d (%.2f MHz), the '
              'cycle of a rising edge included, thread pairs and alignments in turn: CMD55\'s R1 stored after its start '
              'bit, its CRC7 check set, each start bit where the delay puts it' % (h, CLK / 2e6 / h), ok, text)
    h = 63
    ds = (0, 1, 2, 3, 31, 61, 62, 62.5, 63, 64, 94, 125, 126)
    res = grid(r1, h, [(ncr, 8, d) for ncr in (2, 64) for d in ds], dict(state='tran'))
    check('at H 63 (397 kHz, identification): NCR 2 and 64 x output delays 0, 1, 2 and 3 cycles (60 ns), H/2, H - 2, '
          'H - 1, H - 0.5, H, H + 1, 3H/2, 2H - 1 and 2H: R1 right, each start bit where the delay puts it',
          not res[0], fmt(res[0], res[1]))
    # A CID of all 120 bits, MID 0x9F first (5.2): its bit 127 is 1, so a CRC that left it out, or took one bit of
    # the 111111 before it, would differ. The card's default CID has fields only below bit 72.
    cid = 0x9F << 112 | 0x5449 << 96 | 0x5344333247 << 56 | 0x61 << 48 | 0x12345678 << 16 | 0x153
    for h in (3, 4):
        ok, text = every_phase(grid([(2, 0, R136, 0)], h, [(ncr, 8, d) for ncr in (2, 5, 17, 64)
                                                            for d in range(2 * h + 1)], dict(state='ready', cid=cid)), h)
        check('CMD2\'s R2 (136 bits) at NCR 2, 5, 17 and 64 x every output delay from 0 to 2H at H %d, from a CID whose '
              'bit 127 is 1: 9 words after the start bit, the CID\'s CRC7 and end bit checked' % h, ok, text)
    for h, nacs in ((3, (2, 3, 5, 9, 33, 64, 1000)), (4, (2, 3, 5, 9, 33, 64, 1000)), (8, (2, 64))):
        pts = [(2 if j % 2 else 64, nac, d) for j, nac in enumerate(nacs) for d in range(2 * h + 1)]
        ok, text = every_phase(grid([read_cmd(7)], h, pts, dict(state='tran'), line='dat'), h)
        check('NAC %s clocks (from 2, the block starting during the response, NCR 64 and 2 in turn) x every output '
              'delay from 0 to 2H at H %d: response and block right, both start bits taken at every phase' % (
                  ', '.join(map(str, nacs)), h), ok, text)
    # The top of NAC's range: a CSD version 2.0 card's fixed 1 ms access time (5.3, TAAC 0Eh), NSAC's largest
    # clock-dependent part (25.5 k clocks, 5.3), and the 100 ms read time-out (4.6.2.1) at the fastest clock, where it
    # is the most clocks: 833,333 at H 3, inside the 16-bit time limit of 52,383 words of 16 clocks the host posts.
    for nac, d, what in ((8_333, 1, '1 ms, a CSD 2.0 card\'s TAAC'), (25_500, 0, 'NSAC\'s largest 25.5 k clocks'),
                         (833_333, 3, 'the 100 ms read time-out, output delay H')):
        r = run([read_cmd(13)], card=SD.Card(state='tran', ncr=9, nac=nac, d=d), h0=3, h=3, cycles=6_000_000)
        e = judge(r)
        check('NAC %d clocks at H 3 (%s, SD 4.6.2.1 and 5.3): the clock keeps running and the block is right' % (
            nac, what), not e and len(r.res) == 1, e[0] if e else '%d cycles' % r.cycles)
    r = run([read_cmd(11), read_cmd(12)], card=SD.Card(state='tran', nac=9, d=4, bad_end=True), h0=4, h=4)
    r2 = run([read_cmd(11), read_cmd(12)], card=SD.Card(state='tran', nac=9, d=8, bad_end=True), h0=4, h=4)
    check('two reads back to back at H 4 with both blocks\' end bits sent as 0, at output delays H and 2H: thread B '
          'starts its next reception once DAT0 is high, and both blocks arrive whole with their checks clear',
          not judge(r) and not judge(r2) and len(r.res) == len(r2.res) == 2 and
          not any(x['bstat'] & 1 for x in r.res + r2.res), judge(r) or judge(r2))


def main(log):
    check_unit(log)
    check_timing(log)
    check_faults(log)
    check_limits(log)
