"""Bench for advanced/spi_unit.s: an SPI target on the shift unit, against a controller modelled on the RP2350's SPI
block, the ARM PrimeCell SSP (PL022), in its Motorola SPI frame format (RP2350 datasheet, 12.3.4.9 to 12.3.4.13).

The controller model follows that text, written before the firmware:
- SPO 0, SPH 0 (mode 0): SCK idles low. CS falls; half an SCK period later MOSI shows its first bit; half a period
  after that SCK rises, the first capture. Data is captured on rising edges and changed on falling edges. CS rises
  one SCK period after the last capture. Back to back, CS "pulses HIGH between each data word transfer", so each
  word is a frame of its own. The text gives no length for that pulse; the model takes it as a parameter, one SCK
  period by default, and the bench sweeps it.
- SPO 1, SPH 1 (mode 3): SCK idles high. CS falls; half a period later SCK falls and both sides put out their first
  bit; data is captured on rising edges and changed on falling edges. Back to back, CS stays low between words, so
  the words of a transfer are one frame. CS rises one SCK period after the last capture.
The PL022 itself needs its clock 12 times SCK as a target (12.3.4.4); the model's SCK can be any rate.

Edges are placed in real time and seen by the chip in the cycle they fall in (a sub-cycle phase moves them all), so
a half period that isn't a whole number of cycles walks the edges across the chip's clock. The controller reads
MISO at the instant of its capture edge, and the bench measures, for every capture, the time since MISO last moved
(setup) and until it next moves (hold), in cycles. MOSI can be moved against SCK (skew) to find the receive window.
"""
import math
import random

from report import assemble, check
from simple import Chip, read

PROGRAMS = ('advanced/spi_unit.s',)
PROG = 'advanced/spi_unit.s'
CLK = 50_000_000
CS, SCK, MOSI, MISO = 3, 4, 5, 2
TABLE, MOUT, MEND, MCFG = 256, 1, 2, 7


def bits_of(words, n=16):
    return [(w >> (n - 1 - i)) & 1 for w in words for i in range(n)]


def to_bytes(bits):
    return [int(''.join(map(str, bits[i:i + 8])), 2) for i in range(0, len(bits) - len(bits) % 8, 8)]


class Controller:
    """The PL022-style controller. frames: one list of MOSI bits per CS-low period. gap: CS high time before each
    frame after the first, in cycles, a number or a list (default one SCK period). half: SCK half period in cycles,
    any positive number. phase: sub-cycle offset of every edge. skew: MOSI's edges moved by this many cycles.
    cut: {frame: bits} stops that frame's clock after so many bits, as a controller that gives up mid-frame."""

    def __init__(self, mode, half, frames, gap=None, phase=0.0, skew=0.0, start=400.0, cut=None):
        T = 2.0 * half
        self.mode, self.half, self.frames = mode, half, frames
        gaps = gap if isinstance(gap, (list, tuple)) else [T if gap is None else gap] * len(frames)
        ev = []                                 # (time, signal, level)
        self.caps = []                          # (time, frame) of every capture edge
        self.cs_lo = []                         # (fall, rise) times of each frame
        idle = 1 if mode == 3 else 0
        t0 = start
        ev += [(0.0, CS, 1), (0.0, SCK, idle), (0.0, MOSI, 0)]
        for f, b in enumerate(frames):
            n = len(b) if not cut or f not in cut else cut[f]
            ev.append((t0, CS, 0))
            for k in range(n):
                if mode == 0:
                    if k == 0:
                        ev.append((t0 + T / 2 + skew, MOSI, b[0]))
                    r = t0 + T + k * T
                    ev.append((r, SCK, 1))
                    ev.append((r + T / 2, SCK, 0))
                    if k + 1 < n:
                        ev.append((r + T / 2 + skew, MOSI, b[k + 1]))
                else:
                    fall = t0 + T / 2 + k * T
                    ev.append((fall, SCK, 0))
                    ev.append((fall + skew, MOSI, b[k]))
                    r = fall + T / 2
                    ev.append((r, SCK, 1))
                self.caps.append((t0 + T + k * T, f))
            rise = t0 + max(n, 1) * T + T
            ev.append((rise, CS, 1))
            ev.append((rise + max(0.0, skew), MOSI, 0))     # idle: the PL022 forces its data line low
            self.cs_lo.append((t0, rise))
            t0 = rise + (gaps[f + 1] if f + 1 < len(frames) else T)
        self.end = t0
        self.events = sorted((math.floor(t + phase), s, v) for t, s, v in ev)
        self.caps = [(t + phase, f) for t, f in self.caps]
        self.i = 0
        self.lv = {CS: 1, SCK: idle, MOSI: 0}

    def ui(self, c):
        while self.i < len(self.events) and self.events[self.i][0] <= c:
            _, s, v = self.events[self.i]
            self.lv[s] = v
            self.i += 1
        return 0xFF & ~(7 << 3) | self.lv[CS] << CS | self.lv[SCK] << SCK | self.lv[MOSI] << MOSI


class Result:
    pass


def run(mode, half, frames, table, rcnt, rend=0, tedge=1, tid=0, align=0, text=None, **kw):
    """One scenario. Returns the words and frame ends the host took, the bits the controller captured on MISO
    per frame, and the setup and hold of every capture."""
    img = assemble(text or read(PROG), defines=dict(MODE=mode, TEDGE=tedge), depth=512)
    words = dict(img.words)
    for i, w in enumerate(table):
        words[TABLE + i] = w
    chip = Chip(words, 512)
    ctl = Controller(mode, half, list(frames) + [[]], **kw)     # a last CS pulse with no clock: the frame
    #                                                                 before it is reported at its fall
    while chip.c % 4 != align:
        chip.cycle(_Idle())
    cfg = [rcnt, rend]
    chip.start(tid)
    b = _Board(ctl)
    host = []                                   # ('w', word) and ('e', bit count) in the order taken
    end = int(ctl.end) + 200
    while chip.c < end:
        if cfg and chip.post(MCFG, cfg[0]):
            cfg.pop(0)
        chip.cycle(b)
        for n, tag in ((MOUT, 'w'), (MEND, 'e')):
            v = chip.take(n)
            if v is not None:
                host.append((tag, v))
    r = Result()
    r.chip, r.ctl, r.host, r.cycles = chip, ctl, host, chip.c
    r.late = any(t.late for t in chip.t) or any(t.th.ILL for t in chip.t)
    r.miso = [[] for _ in ctl.frames]
    r.setup, r.hold = [], []
    ch = b.changes                              # cycles where MISO moved
    j = 0
    for (t, f), (cc, v) in zip(ctl.caps, b.seen):
        r.miso[f].append(v)
        while j < len(ch) and ch[j] <= t:
            j += 1
        r.setup.append(t - (ch[j - 1] if j else -1e9))
        r.hold.append((ch[j] if j < len(ch) else 1e9) - t)
    return r


class _Idle:
    def step(self, chip, c, uo, en, val):
        return 0xFF & ~(7 << 3) | 1 << CS, 0xFF


class _Board:
    def __init__(self, ctl):
        self.ctl = ctl
        self.k = 0
        self.seen = []                          # (cycle, MISO) at each capture
        self.changes = []
        self.last = None

    def step(self, chip, c, uo, en, val):
        m = (uo >> MISO) & 1
        if self.last is not None and m != self.last:
            self.changes.append(c)
        self.last = m
        while self.k < len(self.ctl.caps) and math.floor(self.ctl.caps[self.k][0]) <= c:
            self.seen.append((c, m))
            self.k += 1
        return self.ctl.ui(c), 0xFF


def frames_of(host):
    """The host's words, cut into frames at each bit count: [(bits)] per ended frame, and the words after the
    last end (a reception still open)."""
    out, cur = [], []
    for tag, v in host:
        if tag == 'w':
            cur.append(v)
        else:
            n = v
            bits = bits_of(cur[:-1]) + [(cur[-1] >> (n - 1 - i)) & 1 for i in range(n)] if cur else []
            out.append(bits)
            cur = []
    return out, cur


def judge(r, frames, rcnt, rend=0, table=None):
    """Compares the firmware's words and the controller's MISO captures with the stimulus. Framed (RXCNT > 0):
    every frame but the last ended and came back whole, and every frame's MISO is the table from its start.
    Endless (RXCNT 0): the words are the MOSI stream, and MISO is the table as one stream."""
    tb = bits_of(table)
    bad = []
    if rcnt:
        got, _ = frames_of(r.host)
        for f, b in enumerate(frames):
            if f >= len(got) or got[f] != b:
                bad.append('frame %d: host got %s bits, sent %d' % (f, len(got[f]) if f < len(got) else None,
                                                                    len(b)))
                break
        for f, b in enumerate(frames):
            if r.miso[f] != tb[:len(b)]:
                bad.append('frame %d: MISO %s, want %s' % (f, to_bytes(r.miso[f]), to_bytes(tb[:len(b)])))
                break
    else:
        stream = [x for b in frames for x in b]
        words = [v for tag, v in r.host if tag == 'w']
        if bits_of(words) != stream[:16 * len(words)] or len(words) != len(stream) // 16:
            bad.append('host words %d, want %d; first %s' % (len(words), len(stream) // 16,
                                                             ['%04X' % w for w in words[:4]]))
        cap = [x for m in r.miso for x in m]
        if cap != tb[:len(cap)]:
            k = next(i for i, (a, b) in enumerate(zip(cap, tb)) if a != b)
            bad.append('MISO differs from bit %d' % k)
    if r.late:
        bad.append('LATE or ILL set')
    return bad


def rand_frames(rng, n, nbits):
    return [[rng.getrandbits(1) for _ in range(nbits)] for _ in range(n)]


def table_words(rng, n=96):
    return [rng.getrandbits(16) for _ in range(n)]


def margins(r):
    """The least setup and hold over the captures where MISO moved before and after within the run."""
    su = [x for x in r.setup if x < 1e8] or [1e9]
    ho = [x for x in r.hold if x < 1e8] or [1e9]
    return min(su), min(ho)


# ---- the checks --------------------------------------------------------------------------------------------------

def sweep_case(mode, half, nbits, nframes, rcnt, rend=0, tedge=1, gap=None, phases=(0.0, 0.25, 0.5, 0.75),
               seeds=(1,), threads=(0,), aligns=(0,), skew=0.0):
    """Runs every phase, seed, thread and alignment; returns (failures, runs, min setup, min hold)."""
    bad, n, su, ho = [], 0, 1e9, 1e9
    for seed in seeds:
        rng = random.Random(seed * 7919 + mode * 31 + int(half * 8) + nbits)
        table = table_words(rng)
        frames = rand_frames(rng, nframes, nbits)
        for ph in phases:
            for tid in threads:
                for al in aligns:
                    r = run(mode, half, frames, table, rcnt, rend, tedge, tid=tid, align=al, gap=gap, phase=ph,
                            skew=skew)
                    n += 1
                    e = judge(r, frames, rcnt, rend, table)
                    s, h = margins(r)
                    su, ho = min(su, s), min(ho, h)
                    if e:
                        bad.append((ph, tid, al, seed, e[0]))
    return bad, n, su, ho


def fmt(bad, n, su, ho):
    return '%d of %d runs; MISO setup at least %.2f cycles (%.0f ns), hold at least %.2f cycles (%.0f ns)%s' % (
        n - len(bad), n, su, su * 20, ho, ho * 20, ('; ' + str(bad[0])) if bad else '')


def check_unit(log):
    log.group(PROG, title='SPI target on the shift unit against a PL022-style controller (RP2350 datasheet 12.3.4)')
    # Mode 0, SPH 0: back-to-back bytes, CS pulsed high for one SCK period between them, each byte a frame.
    for half in (8, 10, 25, 60):
        res = sweep_case(0, half, 8, 6, 8, threads=(0, 1, 2, 3), aligns=(0, 1, 2, 3), phases=(0.0, 0.5))
        check('mode 0, SPH 0: 6 bytes back to back, CS high one SCK period between them, half period %d (%.2f MHz),'
              ' 4 threads x 4 alignments x 2 phases: each byte comes back, MISO is the table at every frame start'
              % (half, CLK / 2e6 / half), not res[0], fmt(*res))
    # Mode 3, SPH 1: CS held low for a whole transfer.
    for half, nbits in ((4, 80), (10, 72), (25, 40)):
        res = sweep_case(3, half, nbits, 3, nbits, gap=100, threads=(0, 1, 2, 3), aligns=(0, 1, 2, 3),
                         phases=(0.0, 0.5))
        check('mode 3, SPH 1: 3 transfers of %d bits with CS held low, CS high 100 cycles between them, half period '
              '%g (%.2f MHz), 4 threads x 4 alignments x 2 phases' % (nbits, half, CLK / 2e6 / half), not res[0],
              fmt(*res))
    # Mode 0 with CS held low (a controller that drives CS from a plain GPIO), frames of odd lengths.
    for nbits in (1, 7, 15, 16, 17, 33, 100):
        res = sweep_case(0, 5, nbits, 3, nbits, gap=60, phases=(0.0, 0.3, 0.6, 0.9))
        check('mode 0, CS held low for a frame of %d bits, half period 5, CS high 60 cycles, 4 phases: the last word '
              'has its bit count' % nbits, not res[0], fmt(*res))
    # One endless reception (RXCNT 0): the units stream through CS's pulses.
    for mode, half in ((0, 4), (3, 4), (0, 3.5), (3, 3.25)):
        res = sweep_case(mode, half, 8, 24, 0, phases=(0.0, 0.25, 0.5, 0.75), seeds=(1, 2))
        check('mode %d, RXCNT 0: 24 bytes as one stream, CS pulsed one SCK period between bytes, half period %g '
              '(%.2f MHz), 4 phases x 2 seeds' % (mode, half, CLK / 2e6 / half), not res[0], fmt(*res))
    # TEDGE 0: MISO moves 3 cycles after each capture edge, both modes, down to half period 2.
    for mode in (0, 3):
        res = sweep_case(mode, 2, 8, 24, 0, tedge=0, phases=(0.0, 0.25, 0.5, 0.75), seeds=(1, 2))
        check('TEDGE 0, mode %d, RXCNT 0: 24 bytes streamed at half period 2 (12.5 MHz), 4 phases x 2 seeds'
              % mode, not res[0], fmt(*res))
    # A frame the controller cuts short: RXEND ends it, and the next frame is in step again.
    rng = random.Random(5)
    table = table_words(rng)
    frames = rand_frames(rng, 4, 16)
    r = run(0, 10, frames, table, 16, rend=60, cut={1: 5}, gap=200)
    got, _ = frames_of(r.host)
    ok = len(got) >= 3 and got[0] == frames[0] and got[1] == frames[1][:5] and got[2] == frames[2] and not r.late
    check('mode 0, half period 10, RXEND 60: frame 1 cut after 5 bits ends 60 cycles after its last edge with 5 '
          'bits, and frame 2 arrives whole', ok, [len(g) for g in got])
    # Without RXEND the same cut frame runs into the next one, and every later frame is out of step.
    r = run(0, 10, frames, table, 16, rend=0, cut={1: 5}, gap=200)
    got, _ = frames_of(r.host)
    check('the same without RXEND: the cut frame takes the first 11 bits of frame 2, which is lost, and frame 3 is '
          'in step again, because the restart waits for CS high (expected: the reason for RXEND)',
          len(got) >= 3 and got[1] == frames[1][:5] + frames[2][:11] and got[2] == frames[3], [len(g) for g in got])


def check_limits(log):
    log.group(PROG, title='limits: the shortest CS high time between framed bytes, and the fastest SCK')
    # Framed bytes need the thread's restart in the CS high time: at half period 8 it is 11 cycles (PL022 gives 16).
    good = sweep_case(0, 8, 8, 6, 8, gap=11, phases=(0.0, 0.3, 0.6, 0.9), threads=(0, 1, 2, 3), aligns=(0, 1, 2, 3))
    short = sweep_case(0, 8, 8, 6, 8, gap=10, phases=(0.0, 0.3, 0.6, 0.9), threads=(0, 1, 2, 3), aligns=(0, 1, 2, 3))
    check('mode 0, framed bytes at half period 8: CS high 11 cycles is the shortest that works on 64 of 64 runs; 10 '
          'loses a frame on some (the restart must see CS high before it falls again)',
          not good[0] and short[0], 'at 11: %s; at 10: %d of %d fail' % (fmt(*good), len(short[0]), short[1]))
    # The stream's edge limit: MISO moves 3 cycles after the falling edge, so half period 3 is the first clean one.
    a = sweep_case(0, 3, 8, 24, 0, phases=tuple(i / 10 for i in range(10)))
    z = sweep_case(0, 2.75, 8, 24, 0, phases=tuple(i / 10 for i in range(10)))
    check('mode 0, endless reception, TEDGE 1: half period 3 (8.33 MHz) clean on 10 of 10 phases, 2.75 not',
          not a[0] and z[0], 'at 3: %s; at 2.75: %d of %d fail' % (fmt(*a), len(z[0]), z[1]))


def main(log):
    check_unit(log)
    check_limits(log)
