# SPDX-License-Identifier: Apache-2.0
"""The first power-on test: the host side of host/bringup.s, through pehost.Host.

It runs under MicroPython on the demo board (board.py calls it) and under CPython in the tests. run() prints one
line per check, PASS or FAIL, and returns the Report. It stops at once only if CAPS fails, because then nothing
else can be read.
"""

import pehost

# Memory test: an address-dependent word, then its complement, so a fault in the address path or a stuck bit
# shows. a * 0x8D3B is different for every address, since the multiplier is odd.
def pattern(a, k):
    w = (a * 0x8D3B + 0x2F61) & 0xFFFF
    return w if k == 0 else w ^ 0xFFFF

MAILBOX = (0x9E37, 0x3C6E, 0xDAA5, 0x78DC, 0x1713, 0xB54A, 0x5381, 0xF1B8)    # a different word for each mailbox word
PINS = (0x00, 0xFF, 0x55, 0xAA) + tuple(1 << i for i in range(8)) + tuple(0xFF ^ 1 << i for i in range(8))
DONE = 0x8000           # bit 15 ends thread 0's pin test (bringup.s)
MISO_BIT = 0x08         # uo3 is the link's MISO: the program never drives it
REG_NAMES = ('R0', 'R1', 'R2', 'R3', 'R4', 'R5', 'R6', 'R7', 'D', 'S', 'PC', 'FLAGS', 'SSTAT', 'CRC_LO', 'CRC_HI',
             'ENDF') + pehost.SLOTS


class Report:
    def __init__(self, log):
        self.log = log
        self.rows = []
        self.step = None                # the step running now, for a FAIL line if it stops

    def check(self, name, ok, detail=''):
        self.rows.append((bool(ok), name, detail))
        self.log('%s  %s%s' % ('PASS' if ok else 'FAIL', name, ': ' + detail if detail else ''))
        return ok

    def note(self, text):
        self.log('      ' + text)

    def failed(self):
        return [r for r in self.rows if not r[0]]


def expected_regs(t, mailbox, d_run, s0):
    """What RDREG reads of thread t after bringup.s halts it: a dict from reg8 to (value, mask), or None where the
    value depends on timing. mailbox is the words the host posted; d_run is D as RUN set it; s0 is thread 0's last
    pin snapshot's uio byte, or None if the pin test didn't run."""
    r3 = 0x5AF0 + t
    r5 = 2 * r3 & 0xFFFF
    want = {0: (t, 0xFFFF), 1: (mailbox[t] ^ 0xFFFF, 0xFFFF), 2: (mailbox[t + 4] ^ 0xFFFF, 0xFFFF),
            3: (r3, 0xFFFF), 4: (r3 ^ 0xFFFF, 0xFFFF), 5: (r5, 0xFFFF), 6: (r3 ^ r5, 0xFFFF), 7: (d_run, 0xFFFF)}
    # D: RUN set it, and threads 1 to 3 have no event after that. Thread 0's events move it.
    want[8] = None if t == 0 else (d_run, 0xFFFF)
    # S: 0 from reset on a thread with no event; thread 0's last snapshot holds the last pattern on uio.
    if t:
        want[9] = (0, 0xFFFF)
    else:
        want[9] = None if s0 is None else (s0 << 8, 0xFF00)
    want[10] = None                                     # filled in by the caller: the address after halt
    want[11] = (t & 1, 0xB)                             # T is bit 0 of t; LATE and WAITING 0. STALE moves with time
    for k in range(12, 32):
        want[k] = (0, 0xFFFF)                           # status, CRC, end flags and slots: reset values, 0
    want[16 + 2] = (r3, 0xFFFF)                         # TXPER
    want[16 + 8] = (r3 ^ 0xFFFF, 0xFFFF)                # RXCNT
    want[16 + 11] = (r5, 0xFFFF)                        # POLYHI
    want[16 + 14] = (r3 ^ r5, 0xFFFF)                   # CHKLO
    return want


def memory_test(host, rep):
    depth = host.depth
    for k in (0, 1):
        rep.step = 'memory, pattern %d' % (k + 1)
        img = pehost.Image(dict((a, pattern(a, k)) for a in range(depth)))
        host.load(img)
        bad = host.verify(img)
        detail = '%d words written and read back' % depth
        if bad:
            a, w, g = bad[0]
            detail = '%d of %d words differ, the first at %d: wrote %04X, read %04X' % (len(bad), depth, a, w, g)
        rep.check('memory, pattern %d' % (k + 1), not bad, detail)


def run(host, image, probe=None, log=print, memory=True, pins=True):
    """The whole bring-up. image is host/bringup.img as a pehost.Image. probe, if given, returns (uo, uio) as the
    board's own GPIOs read the chip's pads. pins=False skips driving the uio and uo pins, for a board with something
    on them. Returns the Report."""
    rep = Report(log)
    try:
        _steps(host, image, rep, probe, memory, pins)
    except pehost.LinkError as e:
        rep.check(rep.step, False, str(e))          # the step it stopped in
        raise
    finally:
        rep.log('bring-up: %d passed, %d failed' % (len(rep.rows) - len(rep.failed()), len(rep.failed())))
    return rep


def _steps(host, image, rep, probe, memory, pins):
    rep.step = 'CAPS'
    v, n, d = host.check_caps()
    rep.check('CAPS', True, 'version %d, %d threads, %d words' % (v, n, 1 << d))
    host.depth = 1 << d

    rep.step = 'STATUS after reset'
    s = host.status()
    rep.check('STATUS after reset', s.run == 0 and s.ill == 0 and s.late == 0 and s.full == 0 and s.busy == 0,
              repr(s))
    if s.run:
        host.stop(0xF)
        host.wait_stopped(0xF)
        rep.note('threads %X were running: stopped them' % s.run)

    if memory:
        memory_test(host, rep)

    rep.step = 'bring-up program'
    host.load(image)
    bad = host.verify(image)
    rep.check('bring-up program', not bad, '%d words loaded and read back' % len(image.words) if not bad else
              '%d words differ, the first at %d' % (len(bad), bad[0][0]))
    rep.step = 'every thread started by one RUN'
    entry = image.labels['start']
    host.start({0: entry, 1: entry, 2: entry, 3: entry})
    s = host.status()
    rep.check('every thread started by one RUN', s.run == 0xF and s.ill == 0, repr(s))

    for n in range(8):
        rep.step = 'mailbox word %d round trip' % n
        host.post_wait(n, MAILBOX[n])
        got = host.take_wait(n)
        want = MAILBOX[n] ^ 0xFFFF
        if got == MAILBOX[n]:
            detail = 'the host took back its own word: no thread took it'
        else:
            detail = 'posted %04X, thread %d sent back %04X' % (MAILBOX[n], n & 3, got)
        rep.check('mailbox word %d round trip' % n, got == want, detail)

    s0 = None
    if pins:
        rep.step = 'uio pads read back by the chip while it drives them'
        chip_bad, board_bad, ui = [], [], 0
        for p in PINS:
            host.post_wait(0, p)
            snap = host.take_wait(4)
            if snap >> 8 != p:
                chip_bad.append((p, snap >> 8))
            ui = snap & 0xFF
            if probe is not None:
                uo, uio = probe()                       # thread 0 still drives p: it waits for the next word
                if uio != p or uo & ~MISO_BIT != p & ~MISO_BIT:
                    board_bad.append((p, uio, uo))
            s0 = p
        rep.check('uio pads read back by the chip while it drives them', not chip_bad,
                  '%d patterns' % len(PINS) if not chip_bad else
                  '%d of %d patterns differ, the first drove %02X and read %02X' % ((len(chip_bad), len(PINS)) + chip_bad[0]))
        if probe is not None:
            rep.check('the board sees the chip drive uio and uo (uo3 is MISO)', not board_bad,
                      '%d patterns' % len(PINS) if not board_bad else
                      '%d differ, the first drove %02X: uio read %02X, uo read %02X' % ((len(board_bad),) + board_bad[0]))
        rep.note('ui3..ui7 read %d%d%d%d%d in the last snapshot: 1 where a DIP switch is on' % tuple(
            ui >> i & 1 for i in range(3, 8)))
    else:
        rep.note('pin test skipped')
    rep.step = 'every thread halted'
    host.post_wait(0, DONE)
    s = host.wait_stopped(0xF)
    rep.check('every thread halted', s.run == 0 and s.ill == 0 and s.late == 0 and s.full == 0 and s.busy == 0,
              repr(s))

    rep.step = 'registers after halt'
    regs = {}
    for t in range(4):
        regs[t] = [host.rdreg(t, k) for k in range(32)]
    d_run = regs[1][7]
    rep.check('the four threads got one D from one RUN', all(regs[t][7] == d_run for t in range(4)),
              'D %s' % ' '.join('%04X' % regs[t][7] for t in range(4)))
    for t in range(4):
        want = expected_regs(t, MAILBOX, d_run, s0)
        want[10] = (image.labels['halted'] + 1, 0xFFFF)
        bad = []
        for k in range(32):
            if want[k] is not None and regs[t][k] & want[k][1] != want[k][0]:
                bad.append('%s %04X, want %04X' % (REG_NAMES[k], regs[t][k], want[k][0]))
        checked = sum(1 for k in range(32) if want[k] is not None)
        rep.check('thread %d registers after halt' % t, not bad,
                  '%d registers as expected, FLAGS %r' % (checked, pehost.Flags(regs[t][11])) if not bad else '; '.join(bad))

    rep.step = 'every thread started and stopped by the host'
    spin = image.labels['spin']
    host.start({0: spin, 1: spin, 2: spin, 3: spin})
    s = host.status()
    started = s.run == 0xF
    host.stop(0xF)
    s = host.wait_stopped(0xF)
    pcs = [host.rdreg(t, 'PC') for t in range(4)]
    rep.check('every thread started and stopped by the host', started and s.run == 0 and pcs == [spin] * 4,
              'PCs %s, want %04X' % (' '.join('%04X' % p for p in pcs), spin))

    rep.step = 'ABORT'
    host.abort()
    s = host.status()
    rep.check('ABORT', s.run == 0 and s.busy == 0, repr(s))
