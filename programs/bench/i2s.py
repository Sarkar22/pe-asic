"""Bench for advanced/i2s_ctl.s and advanced/i2s_tgt.s: I2S and TDM on the shift units (section 5.9), as controller
and as target.

The references were written from the documents before the firmware, and share nothing with it:
- I2S: the Philips I2S bus specification (February 1986, revised June 5, 1996). 3.1: MSB first, two's complement;
  a receiver sent more bits than its word length ignores the rest, and the missing bits of a shorter word are 0.
  3.2: WS 0 is the left channel and 1 the right; WS changes one clock period before the MSB, and a slave latches
  it on the leading edge of SCK. 3.1: the receiver latches SD on the leading (rising) edge; a transmitter may
  change it on either edge. Table 1: SCK high and low at least 0.35T; a transmitter's delay tdtr at most 0.8T
  after the rising edge, and its hold thtr at least 0; a receiver's setup tsr at least 0.2T and hold thr at least
  0. 6.1: a slave transmitter loads its shift register at each WS change and sends zeros after the LSB.
- TDM: Texas Instruments' TLV320ADC3140 data sheet (SBAS993B), 8.3.1.2.1 and Table 5, in TDM mode with BCLK_POL 1
  and TX_OFFSET 1: FSYNC's rising edge starts a frame, the slots follow each other MSB first with no gap, FSYNC
  and the data change on falling edges of BCLK and are latched on rising edges, and the first bit of slot 0 comes
  one BCLK after the edge that latches FSYNC high; FSYNC may be one BCLK wide or wider. 7.10 and 7.11: BCLK period
  at least 40 ns, high and low at least 18 ns, FSYNC setup and hold at least 8 ns, BCLK to SDOUT and to FSYNC at
  most 18 ns. sigrok's tdm_audio decoder reads exactly this offset: it drops the bit of the edge where it first
  sees the frame sync high.

Two device models react to pad levels only: Controller, an I2S or TDM master whose SCK edges are placed in real
time with a sub-cycle phase (its WS or FS and SD change a set delay after each falling edge), which latches the
chip's SD at its own rising edges and measures setup and hold there; and TargetTx, a slave transmitter that
watches the chip's SCK and WS or FS at the pads and drives SD a set number of cycles after each falling edge.
Readers (read_i2s, read_tdm) and timing checks (timing) work on a pad record.
"""
import bisect
import math
import random

from report import assemble, check
from simple import Chip, read

PROGRAMS = ('advanced/i2s_ctl.s', 'advanced/i2s_tgt.s')
CTL, TGT = PROGRAMS
CLK = 50_000_000
I2S, TDM = 0, 1


# ---------------------------------------------------------------------------------------------------------------
# a pad record and the readers

class Lines:
    """Changes of named lines: name -> ([cycles], [levels]), with the level each line has before its first change."""

    def __init__(self, names, init=None):
        self.t = {n: [] for n in names}
        self.v = {n: [] for n in names}
        self.init = dict(init or {})
        self.end = 0

    def add(self, c, levels):
        for n, lv in levels.items():
            vs = self.v[n]
            last = vs[-1] if vs else self.init.setdefault(n, lv)
            if lv != last or (not vs and lv != self.init[n]):
                self.t[n].append(c)
                vs.append(lv)
        self.end = c

    def at(self, n, c):
        """The level of line n in cycle c."""
        i = bisect.bisect_right(self.t[n], c)
        return self.v[n][i - 1] if i else self.init.get(n, 0)

    def rises(self, n):
        out, prev = [], self.init.get(n, 0)
        for c, lv in zip(self.t[n], self.v[n]):
            if lv and not prev:
                out.append(c)
            prev = lv
        return out

    def falls(self, n):
        out, prev = [], self.init.get(n, 0)
        for c, lv in zip(self.t[n], self.v[n]):
            if prev and not lv:
                out.append(c)
            prev = lv
        return out


def latched(lines, sck, names):
    """At every rising edge of sck, in its cycle, the levels of the named lines: [(cycle, (levels))]."""
    return [(r, tuple(lines.at(n, r) for n in names)) for r in lines.rises(sck)]


def read_i2s(lines, sck='sck', ws='ws', sd='sd'):
    """Words as an I2S receiver takes them (I2S 3.1, 3.2, 6.2). At each leading edge it latches WS and SD. At the
    edge where WS is first seen changed, that edge's bit is the last bit of the word before, and the next edge's bit
    is the MSB of a word of channel WS. Before the first change no word has begun. The WS latch starts at the level
    WS has before the first edge. Returns [(channel, [bits])] for each word that ended, and the open word's bits."""
    words, cur, ch = [], None, None
    edges = latched(lines, sck, (ws, sd))
    prev = lines.at(ws, edges[0][0] - 1) if edges else 0
    for _, (w, d) in edges:
        if cur is not None:
            cur.append(d)
        if w != prev:
            if cur is not None:
                words.append((ch, cur))
            cur, ch = [], w
        prev = w
    return words, cur or []


def read_tdm(lines, nslots, nbits, sck='sck', fs='ws', sd='sd'):
    """Frames as a TDM receiver in the TLV320ADC3140's TDM mode with TX_OFFSET 1 takes them: an edge that latches
    FSYNC high after an edge that latched it low starts a frame; slot 0's MSB is the next edge's bit, and nslots
    slots of nbits follow with no gap. Bits after the last slot, up to the next frame, are idle BCLK cycles. Returns
    [[slot words]] for each complete frame."""
    edges = latched(lines, sck, (fs, sd))
    frames, bits, prev = [], None, lines.at(fs, edges[0][0] - 1) if edges else 0
    for _, (f, d) in edges:
        if bits is not None and len(bits) < nslots * nbits:
            bits.append(d)
            if len(bits) == nslots * nbits:
                frames.append([int(''.join(map(str, bits[k * nbits:(k + 1) * nbits])), 2) for k in range(nslots)])
        if f and not prev:
            bits = []
        prev = f
    return frames


def bits_of(word, n):
    return [(word >> (n - 1 - i)) & 1 for i in range(n)]


def timing(lines, outs, sck='sck', fmt=I2S, lo=None, hi=None):
    """The transmitter's side of the timing tables, on a pad record whose cycles are 20 ns: SCK high and low times
    against 0.35 of the period around them (I2S Table 1; for TDM, 18 ns and a 40 ns period, 7.10), and every change
    of each line in outs between two rising edges r1 < t <= r2: after r1, never in its cycle (thtr >= 0, and TDM's
    8 ns hold), and at most 0.8 of the period after r1 (tdtr; it leaves the 0.2T setup tsr), or for TDM at least 8
    ns before r2. Only edges from lo to hi are judged. Returns (problems, the least setup and hold in cycles)."""
    r = [x for x in lines.rises(sck) if (lo is None or x >= lo) and (hi is None or x <= hi)]
    f = lines.falls(sck)
    bad, su, ho = [], 1e9, 1e9
    for a, b in zip(r, r[1:]):
        T = b - a
        k = bisect.bisect_right(f, a)
        if k >= len(f) or f[k] >= b:
            bad.append('no falling edge between rises at %d and %d' % (a, b))
            continue
        hi_t, lo_t = f[k] - a, b - f[k]
        lim = 0.35 * T if fmt == I2S else 18 / 20
        if hi_t < lim or lo_t < lim or (fmt == TDM and T < 2):
            bad.append('SCK high %d low %d cycles in a period of %d at %d' % (hi_t, lo_t, T, a))
        for n in outs:
            ts = lines.t[n]
            for i in range(bisect.bisect_left(ts, a), bisect.bisect_right(ts, b)):
                t = ts[i]
                if t == a or t == b:
                    bad.append('%s changes in the cycle of a rising edge, %d' % (n, t))
                    continue
                su, ho = min(su, b - t), min(ho, t - a)
                if fmt == I2S and t - a > 0.8 * T:
                    bad.append('%s changes %d cycles after the rising edge at %d, past 0.8T of %d' % (n, t - a, a, T))
    return bad, su, ho


# ---------------------------------------------------------------------------------------------------------------
# the device models

def frame_bits(fmt, nslots, nbits, frames):
    """The data stream of a run, one bit per SCK period, and the frame sync stream: bit j's WS (I2S) or FS (TDM).
    Bit 0 is slot 0's MSB of the first frame. I2S: WS(j) is the channel of bit j + 1 (3.2, one clock period before
    the MSB). TDM: FS(j) is 1 for the bit before each frame's first bit (TX_OFFSET 1, a one-BCLK pulse)."""
    W = nslots * nbits
    sd = [b for fr in frames for w in fr for b in bits_of(w, nbits)]
    def sync(j):
        if fmt == I2S:
            return (((j + 1) // nbits) % 2) if j >= -1 else 1
        return 1 if j >= -1 and (j + 1) % W == 0 else 0
    return sd, sync


class Controller:
    """An I2S or TDM controller: SCK with half period `half` cycles (any positive number), its edges placed in real
    time and seen by the chip in the cycle they fall in, all moved by `phase`. Bit j starts at a falling edge and is
    latched at the rising edge half a period later; WS or FS and SD change `delay` cycles after that falling edge
    (I2S 3.1: synchronized with the trailing edge). `lead` bits go before the first frame, a phantom right slot for
    I2S (WS 1) so the target sees WS fall, and `tail` bits after the last. At every rising edge it latches the chip's
    SD (in that edge's cycle) and keeps the time since SD last moved and until it next moves."""

    def __init__(self, fmt, half, nslots, nbits, frames, delay=0.0, phase=0.0, lead=4, tail=3, start=300.0):
        T = 2.0 * half
        self.fmt, self.half, self.T = fmt, half, T
        self.nslots, self.nbits, self.frames = nslots, nbits, frames
        sd, sync = frame_bits(fmt, nslots, nbits, frames)
        self.sd = sd
        n = len(sd)
        ev = [(0.0, 'sck', 0), (0.0, 'ws', sync(-lead - 1)), (0.0, 'sd', 0)]
        self.caps = []
        for j in range(-lead, n + tail):
            t = start + (j + lead) * T
            ev.append((t, 'sck', 0))
            ev.append((t + delay, 'ws', sync(j)))
            ev.append((t + delay, 'sd', sd[j] if 0 <= j < n else 0))
            ev.append((t + T / 2, 'sck', 1))
            self.caps.append((t + T / 2 + phase, j))
        self.end = start + (n + lead + tail) * T
        self.events = sorted((math.floor(t + phase), s, v) for t, s, v in ev)
        self.i = 0
        self.lv = {'sck': 0, 'ws': sync(-lead - 1), 'sd': 0}
        self.init = dict(self.lv)

    def levels(self, c):
        while self.i < len(self.events) and self.events[self.i][0] <= c:
            _, s, v = self.events[self.i]
            self.lv[s] = v
            self.i += 1
        return self.lv


class TargetTx:
    """A slave transmitter (I2S 6.1, or a TDM slot transmitter as the TLV320ADC3140 is with TX_OFFSET 1). It
    latches WS or FS at each rising edge of the chip's SCK. I2S: at the edge where WS is first seen changed, the
    next word of channel WS is loaded; TDM: at the edge where FS is first seen high, the next frame's slots are.
    Each bit goes out `delay` cycles after a falling edge, MSB first, then zeros. words: I2S [(left, right)], TDM
    [[slot words]]."""

    def __init__(self, fmt, words, nslots, nbits, delay=1, ws0=1):
        self.fmt, self.nbits, self.nslots, self.delay = fmt, nbits, nslots, delay
        self.queue = {0: [w[0] for w in words], 1: [w[1] for w in words]} if fmt == I2S else list(words)
        self.prev_sck, self.ws_l, self.bits = None, ws0, []
        self.out, self.pend = 0, []

    def step(self, c, sck, ws):
        if self.prev_sck is None:                       # the level at power-up is no edge
            self.prev_sck = sck
        if sck and not self.prev_sck:                   # rising edge: latch WS or FS
            if self.fmt == I2S and ws != self.ws_l:
                q = self.queue[ws]
                self.bits = bits_of(q.pop(0), self.nbits) if q else []
            elif self.fmt == TDM and ws and not self.ws_l:
                self.bits = [b for w in self.queue.pop(0) for b in bits_of(w, self.nbits)] if self.queue else []
            self.ws_l = ws
        if self.prev_sck and not sck:                   # falling edge: the next bit, delay cycles later
            self.pend.append((c + self.delay, self.bits.pop(0) if self.bits else 0))
        self.prev_sck = sck
        while self.pend and self.pend[0][0] <= c:
            self.out = self.pend.pop(0)[1]
        return self.out


# ---------------------------------------------------------------------------------------------------------------
# runs

class Result:
    pass


class _Idle:
    def step(self, chip, c, uo, en, val):
        return 0xFF, 0xFF


SCK_C, WS_C, SDO_C, SDI_C = 8, 5, 6, 4         # controller: SCK io0, WS or FS uo5, SD out uo6, SD in ui4
MCLK_C, MDAT_C, MOUT_C = 4, 5, 1
TBL_C, PAT_C = 256, 448                         # data table and the frame sync pattern


class _CtlBoard:
    """The board of the controller: the chip's SCK (io0), WS (uo5) and SD (uo6) as a pad record, and a TargetTx on
    SD in (ui4)."""

    def __init__(self, tgt):
        self.tgt = tgt
        self.lines = Lines(('sck', 'ws', 'sd', 'sdi'))

    def step(self, chip, c, uo, en, val):
        lvl = ~(en & ~val) & 0xFF                       # io pins: the chip's drive over pull-ups
        sck, ws, sd = lvl & 1, (uo >> WS_C) & 1, (uo >> SDO_C) & 1
        sdi = self.tgt.step(c, sck, ws) if self.tgt else 0
        self.lines.add(c, {'sck': sck, 'ws': ws, 'sd': sd, 'sdi': sdi})
        return 0xFF & ~(1 << SDI_C) | sdi << SDI_C, lvl


def words16(bits):
    return [int(''.join(map(str, bits[i:i + 16])), 2) for i in range(0, len(bits), 16)]


def ctl_run(fmt, nslots, nbits, frames, tper, tfrac=0, threads=(0, 1), align=0, rx_words=None, delay=1, text=None,
            lead=3000):
    """The controller: thread clk makes SCK and WS (FS), thread dat sends `frames` and receives what a TargetTx sends
    (rx_words, default random). The frames' bits must fill whole 16-bit words. Returns the pad record, the host's
    words, LATE and ILL."""
    W = nslots * nbits
    img = assemble(text or read(CTL), defines=dict(FMT=fmt), depth=512)
    words = dict(img.words)
    sd, sync = frame_bits(fmt, nslots, nbits, frames)
    data = words16(sd)
    for i, w in enumerate(data):
        words[TBL_C + i] = w
    P = (W * 16 // math.gcd(W, 16)) // 16       # pattern words: a whole number of frames and of words
    pat = words16([sync(j) for j in range(16 * P)])
    for i, w in enumerate(pat):
        words[PAT_C + i] = w
    chip = Chip(words, 512)
    while chip.c % 4 != align:
        chip.cycle(_Idle())
    tc, td = threads
    chip.sim.host_setpc(tc, img.labels['clk'])
    chip.sim.host_setpc(td, img.labels['dat'])
    chip.sim.host_run(0, (1 << tc) | (1 << td))
    t0 = (chip.c + lead) & 0xFFFF
    nw = len(data)
    posts = {MCLK_C: [t0, nw, tper, tfrac, P, sync(-2) << 1 | sync(-1)],
             MDAT_C: [t0, nw, tper, tfrac, 16 * nw if 16 * nw <= 2047 else 0]}
    rng = random.Random(7 * nslots + nbits + len(frames))
    if rx_words is None:
        rx_words = [[rng.getrandbits(nbits) for _ in range(nslots)] for _ in frames]
    tgt = TargetTx(fmt, [tuple(f) for f in rx_words] if fmt == I2S else [list(f) for f in rx_words], nslots, nbits,
                   delay, ws0=1 - fmt)
    b = _CtlBoard(tgt)
    host = []
    bit = 2 * tper + tfrac / 128.0
    limit = chip.c + lead + int(bit * (16 * nw + 8)) + 4000
    while chip.c < limit:
        for k in (MCLK_C, MDAT_C):
            if posts[k] and chip.post(k, posts[k][0]):
                posts[k].pop(0)
        chip.cycle(b)
        v = chip.take(MOUT_C)
        if v is not None:
            host.append(v)
        if not any(posts.values()) and all(not chip.t[t].run for t in threads):
            break
    r = Result()
    r.chip, r.lines, r.host, r.t0 = chip, b.lines, host, t0
    r.fmt, r.nslots, r.nbits, r.frames, r.rx_words = fmt, nslots, nbits, frames, rx_words
    r.late = [t.id for t in chip.t if t.late or t.th.ILL]
    r.bit = bit
    return r


def ctl_judge(r, rx=True):
    """The chip's bus read by the reference reader against the frames sent; Table 1 (or TDM's 7.10) on SCK, WS or
    FS and SD; the words the host took against the TargetTx's stream; LATE and ILL."""
    bad = []
    if r.late:
        bad.append('LATE or ILL on threads %s' % r.late)
    if r.fmt == I2S:
        words, _ = read_i2s(r.lines)
        want = [(k % 2, bits_of(w, r.nbits)) for fr in r.frames for k, w in enumerate(fr)]
        if words != want:
            bad.append('the I2S reader got %d words, %d sent; first difference %s' % (
                len(words), len(want), next(((i, a, b) for i, (a, b) in enumerate(zip(words, want)) if a != b),
                                            'in the count')))
    else:
        got = read_tdm(r.lines, r.nslots, r.nbits)
        if got != [list(f) for f in r.frames]:
            bad.append('the TDM reader got %d frames, %d sent; first %s' % (len(got), len(r.frames), got[:1]))
    e, su, ho = timing(r.lines, ('ws', 'sd'), fmt=r.fmt)
    bad += e[:3]
    r.setup, r.hold = su, ho
    if rx:
        stream = [b for fr in r.rx_words for w in fr for b in bits_of(w, r.nbits)]
        got = [b for w in r.host for b in bits_of(w, 16)]
        if got != stream[:len(got)] or len(got) < len(stream) - 15:
            k = next((i for i, (a, b) in enumerate(zip(got, stream)) if a != b), min(len(got), len(stream)))
            bad.append('received %d bits of %d, first difference at bit %d' % (len(got), len(stream), k))
    return bad


def ctl_case(fmt, nslots, nbits, nframes, tper, tfrac=0, seed=1, **kw):
    rng = random.Random(seed)
    frames = [[rng.getrandbits(nbits) for _ in range(nslots)] for _ in range(nframes)]
    r = ctl_run(fmt, nslots, nbits, frames, tper, tfrac, **kw)
    return r, ctl_judge(r)


SCK_T, WS_T, SDI_T, SDO_T = 4, 5, 6, 2          # target: SCK ui4, WS or FS ui5, SD in ui6, SD out uo2
MCFG_T, MOUT_T = 7, 1
TBL_T = 256


class _TgtBoard:
    def __init__(self, ctl):
        self.ctl = ctl
        self.k = 0
        self.seen = []                          # (bit j, the chip's SD at that rising edge)
        self.changes = []
        self.last = None
        self.lines = Lines(('sck', 'ws', 'sd', 'sdo'), dict(ctl.init, sdo=0))

    def step(self, chip, c, uo, en, val):
        m = (uo >> SDO_T) & 1
        if self.last is not None and m != self.last:
            self.changes.append(c)
        self.last = m
        while self.k < len(self.ctl.caps) and math.floor(self.ctl.caps[self.k][0]) <= c:
            self.seen.append((self.ctl.caps[self.k], m))
            self.k += 1
        lv = self.ctl.levels(c)
        self.lines.add(c, dict(lv, sdo=m))
        ui = 0xFF & ~(7 << SCK_T) | lv['sck'] << SCK_T | lv['ws'] << WS_T | lv['sd'] << SDI_T
        return ui, 0xFF


def tgt_run(fmt, nslots, nbits, frames, half, tx_frames=None, tid=0, align=0, text=None, wb=None, **kw):
    """The target on one thread: a Controller drives SCK, WS or FS and SD; the chip receives every frame and sends
    tx_frames (default random) on SD out. Returns the host's words, the controller's captures of SD out with their
    setup and hold, LATE and ILL."""
    wb = wb or (16 if nbits >= 16 else nbits)
    W = nslots * nbits
    K = W // wb
    img = assemble(text or read(TGT), defines=dict(FMT=fmt, WB=wb), depth=512)
    words = dict(img.words)
    rng = random.Random(3 * W + len(frames))
    if tx_frames is None:
        tx_frames = [[rng.getrandbits(nbits) for _ in range(nslots)] for _ in frames]
    tx = [b for fr in tx_frames for w in fr for b in bits_of(w, nbits)]
    tx_words = [int(''.join(map(str, tx[i:i + wb])), 2) for i in range(0, len(tx), wb)]
    for i, w in enumerate(tx_words):
        words[TBL_T + i] = w
    chip = Chip(words, 512)
    # a frame of zeros first, which the target lets go by (it waits for WS to rise, or FS to fall, before it arms)
    kw.setdefault('lead', 2)
    ctl = Controller(fmt, half, nslots, nbits, [[0] * nslots] + list(frames), **kw)
    while chip.c % 4 != align:
        chip.cycle(_Idle())
    chip.start(tid)
    cfg = [W, K]
    b = _TgtBoard(ctl)
    host = []
    end = int(ctl.end) + 300
    while chip.c < end:
        if cfg and chip.post(MCFG_T, cfg[0]):
            cfg.pop(0)
        chip.cycle(b)
        v = chip.take(MOUT_T)
        if v is not None:
            host.append(v)
    r = Result()
    r.chip, r.ctl, r.host, r.lines = chip, ctl, host, b.lines
    r.fmt, r.nslots, r.nbits, r.frames, r.tx_frames, r.wb = fmt, nslots, nbits, frames, tx_frames, wb
    r.late = [t.id for t in chip.t if t.late or t.th.ILL]
    r.tx_bits = tx
    r.sdo = {j - W: v for (t, j), v in b.seen}
    r.setup, r.hold = [], []
    ch = b.changes
    i = 0
    for (t, j), v in b.seen:
        if 0 <= j - W < len(tx):
            while i < len(ch) and ch[i] <= t:
                i += 1
            r.setup.append(t - (ch[i - 1] if i else -1e9))
            r.hold.append((ch[i] if i < len(ch) else 1e9) - t)
    return r


def tgt_judge(r):
    """Every frame the controller sent, as the host's words (wb bits each, a frame's words MSB first); the chip's SD
    out as the controller latched it, bit for bit against tx_frames; setup and hold at each latch against I2S Table
    1 (tsr >= 0.2T, thr >= 0) or TDM's 8 ns; LATE and ILL."""
    bad = []
    if r.late:
        bad.append('LATE or ILL on thread %s' % r.late)
    W = r.nslots * r.nbits
    stream = [b for fr in r.frames for w in fr for b in bits_of(w, r.nbits)]
    n = W if W < 16 else 16
    got = [b for w in r.host for b in bits_of(w, n)]
    if got != stream:
        k = next((i for i, (a, b) in enumerate(zip(got, stream)) if a != b), min(len(got), len(stream)))
        bad.append('host got %d bits of %d, first difference at bit %d (frame %d)' % (len(got), len(stream), k,
                                                                                      k // W))
    sdo = [r.sdo.get(j) for j in range(len(r.tx_bits))]
    if sdo != r.tx_bits:
        k = next(i for i, (a, b) in enumerate(zip(sdo, r.tx_bits)) if a != b)
        bad.append('SD out differs from bit %d (frame %d, bit %d of it)' % (k, k // W, k % W))
    su = min([x for x in r.setup if x < 1e8] or [1e9])
    ho = min([x for x in r.hold if x < 1e8] or [1e9])
    T = 2 * r.ctl.half
    lim_su, lim_ho = (0.2 * T, 0.0) if r.fmt == I2S else (8 / 20, 8 / 20)
    if su < lim_su or ho < lim_ho:
        bad.append('SD out setup %.2f, hold %.2f cycles at the controller, need %.2f and %.2f' % (su, ho, lim_su,
                                                                                                lim_ho))
    r.su, r.ho = su, ho
    return bad


def tgt_case(fmt, nslots, nbits, nframes, half, seed=1, **kw):
    rng = random.Random(seed)
    frames = [[rng.getrandbits(nbits) for _ in range(nslots)] for _ in range(nframes)]
    r = tgt_run(fmt, nslots, nbits, frames, half, **kw)
    return r, tgt_judge(r)


# ---------------------------------------------------------------------------------------------------------------
# the checks

def half_of(rate, fs):
    """SCK half period in cycles for a frame rate and SCK periods a frame, and TPER, TFRAC for it."""
    h = CLK / (2.0 * rate * fs)
    tper = int(h)
    return h, tper, int(round((h - tper) * 256))


PAIRS = ((0, 1), (1, 2), (2, 3), (3, 0), (1, 0), (3, 2), (2, 0), (0, 3))

CTL_RATES = (
    ('I2S, 48 kHz, 2 x 32 bits (64 fs, 3.072 MHz)', I2S, 2, 32, 4, 48000),
    ('I2S, 48 kHz, 2 x 16 bits (32 fs, 1.536 MHz)', I2S, 2, 16, 6, 48000),
    ('I2S, 8 kHz, 2 x 16 bits (256 kbit/s, the rate the flexibility pool names)', I2S, 2, 16, 2, 8000),
    ('TDM, 48 kHz, 8 slots x 16 bits (128 fs, 6.144 MHz)', TDM, 8, 16, 2, 48000),
    ('TDM, 48 kHz, 4 slots x 32 bits (128 fs)', TDM, 4, 32, 2, 48000),
    ('TDM, 8 kHz, one 8-bit slot (64 kbit/s, the rate the flexibility pool names)', TDM, 1, 8, 4, 8000),
)


def check_ctl(log):
    log.group(CTL, title='I2S and TDM controller on two units started at one cycle, against a slave transmitter model')
    for title, fmt, ns, nb, nf, rate in CTL_RATES:
        h, tper, tfrac = half_of(rate, ns * nb)
        pairs = PAIRS if rate == 48000 else PAIRS[:2]
        good, det = 0, []
        for k, th in enumerate(pairs):
            r, bad = ctl_case(fmt, ns, nb, nf, tper, tfrac, seed=k + 1, threads=th, align=k % 4, delay=1 + k % 3)
            good += not bad
            if bad:
                det.append((th, bad[:1]))
        check('%s: TPER %d + TFRAC %d (%.3f cycles a half bit, %.1f Hz), %d frames on %d thread pairs and alignments: '
              'the bus read as the %s defines it, %s timing, every received bit' % (
                  title, tper, tfrac, tper + tfrac / 256.0, CLK / (2 * (tper + tfrac / 256.0) * ns * nb), nf,
                  len(pairs), 'I2S specification' if fmt == I2S else 'TLV320ADC3140 TDM mode',
                  'Table 1' if fmt == I2S else '7.10'), good == len(pairs), '%d of %d %s' % (good, len(pairs), det[:1]))


def check_ctl_limits(log):
    log.group(CTL, title='controller limits: the receive half\'s start and the feed')
    res = []
    for tper in (3, 4, 5):
        for k, tfrac in enumerate((0, 85, 128, 200, 255)):
            r, bad = ctl_case(I2S, 2, 16, 8, tper, tfrac, seed=tper * 10 + k, threads=PAIRS[k], align=k % 4)
            res.append((tper, tfrac, not bad))
    want = {(t, f): (t >= 5 or t + (3 * f) // 256 <= t + 1 and (t >= 4 or 3 * f < 256)) for t, f, _ in res}
    ok = all(g == want[(t, f)] for t, f, g in res)
    check('I2S 2 x 16, TPER 3 to 5 with TFRAC 0 to 255: transmit and receive clean from TPER 3 with TFRAC under 86 '
          '(8.33 MHz), from 4 up to TFRAC 170, and from 5 at any fraction; below that the receive half\'s start '
          'commit lands after the second dummy bit\'s rising edge is gone, and the received words are one bit off',
          ok, ' '.join('%d+%d:%s' % (t, f, 'ok' if g else 'bad') for t, f, g in res))
    r, bad = ctl_case(I2S, 2, 16, 8, 2, 0)
    rx = [b for b in bad if b.startswith('received')]
    check('TPER 2 (12.5 MHz SCK): the bus is clean (the transmit side keeps up), and the receive half misses its start '
          '(expected: the limit above)', not [b for b in bad if not b.startswith('received')] and rx, str(bad[:2]))


TGT_RATES = CTL_RATES[:3] + (
    ('TDM, 48 kHz, 4 slots x 16 bits (64 fs, 3.072 MHz)', TDM, 4, 16, 3, 48000),
    ('TDM, 48 kHz, 2 slots x 32 bits (64 fs)', TDM, 2, 32, 3, 48000),
    CTL_RATES[5])


def check_tgt(log):
    log.group(TGT, title='I2S and TDM target on one unit, both halves started again at every frame')
    for title, fmt, ns, nb, nf, rate in TGT_RATES:
        h, _, _ = half_of(rate, ns * nb)
        runs = [(t, (t + 1) % 4, 0.25 * t, t % 3) for t in range(4)] if rate == 48000 else [(1, 2, 0.5, 1)]
        good, det, su = 0, [], 1e9
        for k, (tid, al, ph, dl) in enumerate(runs):
            r, bad = tgt_case(fmt, ns, nb, nf, h, seed=k + 3, tid=tid, align=al, phase=ph, delay=dl)
            good += not bad
            su = min(su, r.su / (2 * h))
            if bad:
                det.append((tid, bad[:1]))
        check('%s, half period %.3f: %d frames on %d threads, alignments and phases, the controller\'s WS (FS) and SD '
              '0 to 2 cycles after its falling edges: every frame received, SD out latched right at every rising edge, '
              'setup at least 0.2T and hold at least 0' % (title, h, nf, len(runs)), good == len(runs),
              '%d of %d, least setup %.2fT %s' % (good, len(runs), su, det[:1]))


def check_tgt_limits(log):
    log.group(TGT, title='target limit: the restart at every frame')
    rows = []
    for half in (5.0, 4.75, 4.5):
        good = 0
        for k in range(8):
            r, bad = tgt_case(I2S, 2, 16, 6, half, seed=50 + k, tid=k % 4, align=(k // 2) % 4, phase=k / 8.0)
            good += not bad
        rows.append((half, good))
    check('I2S 2 x 16: SCK half period 5 (5 MHz, 156 kHz frames) clean on 8 of 8 threads, alignments and phases; 4.5 '
          'not, because the restart sends the next frame\'s first bit up to 8 cycles after its first edge, later '
          'than 0.8T', rows[0][1] == 8 and rows[-1][1] < 8, rows)
    h = half_of(48000, 128)[0]
    fails = 0
    for k in range(8):
        r, bad = tgt_case(TDM, 8, 16, 6, h, seed=60 + k, tid=k % 4, align=(k + 1) % 4, phase=k / 8.0)
        fails += bool(bad)
    check('TDM 8 x 16 at 48 kHz (128 fs, 6.144 MHz, half period %.3f): the restart misses the 8 ns setup of a later '
          'frame\'s first bit on some phases (expected: a period under 10 cycles); such a target sends one endless '
          'stream from the first frame instead' % h, fails > 0, '%d of 8 runs fail' % fails)
    # the first frame starts at the match itself, so a single frame meets the timing faster than the restart
    r, bad = tgt_case(I2S, 2, 16, 1, 3.0)
    check('a single frame at half period 3 (8.33 MHz): the start at the wait\'s own commit puts the first bit out 3 '
          'cycles after the edge, and the frame is right (the restart, not the start, sets the limit)', not bad,
          str(bad[:1]))


def mutate(text, old, new):
    if old not in text:
        raise ValueError('tooth not found: %r' % old)
    return text.replace(old, new, 1)


def check_teeth(log):
    log.group(*PROGRAMS, title='broken copies: each must fail')
    ctl, tgt = read(CTL), read(TGT)
    cases = [
        ('controller: SCK high then low in each bit (TXMAP 0x0B)', CTL,
         mutate(ctl, 'ldi   R0, 0x0E          ;', 'ldi   R0, 0x0B          ;')),
        ('controller: the receive half started 2 x TPER after T0, under the second dummy bit',
         CTL, mutate(ctl, '        add   R3, R3\n        add   R3, R3', '        add   R3, R3')),
        ('controller: a 1-bit word before the first WS bit, so WS lags SD by a bit', CTL,
         mutate(ctl, 'txw   R0, 2\n        li    R5, PAT', 'txw   R0, 1\n        li    R5, PAT')),
        ('target: the start wait on SCK low instead of high', TGT,
         mutate(tgt, '.equ START 0x3010 | (FMT << 5)', '.equ START 0x3000 | (FMT << 5)')),
        ('target: SD out moved at falling edges (TFALL 1)', TGT,
         mutate(tgt, 'li    R0, 0x0204 | (NSCK << 11)', 'li    R0, 0x0604 | (NSCK << 11)')),
        ('target: no now after the frame\'s last edge, so the restart waits for a later commit', TGT,
         mutate(tgt, '        now                     ; commits 2 to 5', '        idle                    ; commits 2 to 5')),
    ]
    for name, prog, text in cases:
        if prog == CTL:
            bad = ctl_judge(ctl_run(I2S, 2, 16, [[0x8001, 0x7FFE], [0x1234, 0xFEDC], [0xA5A5, 0x5A5A]], 8, 35,
                                    text=text))
        else:
            bad = tgt_judge(tgt_run(I2S, 2, 16, [[0x8001, 0x7FFE], [0x1234, 0xFEDC], [0xA5A5, 0x5A5A]], 8.138,
                                    text=text))
        check(name + ': the bench sees it', bad != [], str(bad[:1]), progs=(prog,))


WORDS = {CTL: 117, TGT: 55}


def check_words(log):
    log.group(*PROGRAMS, title='word counts')
    for rel, want in WORDS.items():
        for fmt in (I2S, TDM):
            try:
                got, err = assemble(read(rel), defines=dict(FMT=fmt), depth=512).size, ''
            except Exception as e:      # an assembler error is a failed check, not a crash
                got, err = None, str(e)
            check('%s with FMT %d assembles to %d words, the count in section 11.2' % (rel, fmt, want), got == want,
                  err or '%d words' % got, progs=(rel,))


def main(log):
    check_words(log)
    check_ctl(log)
    check_ctl_limits(log)
    check_tgt(log)
    check_tgt_limits(log)
    check_teeth(log)
