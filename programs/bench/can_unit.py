"""CAN 2.0A node on the shift unit (advanced/can_unit.s) on a wired-AND bus with transceiver delays.

Oracles, none of which knows the program: the frames of can_wire.py (Bosch CAN 2.0 part A), whose CRC-15 is
anchored on the catalogue check value 0x059E; can_wire.decode on the pads (hard sync on SOF, resync on
recessive-to-dominant edges only); and the other nodes of can.py (a reference sender, a node that ACKs, a rival
that arbitrates by the wire).  The bus here reads TXD from the open-drain pad io0 and drives RXD on ui6, with
separate delays for dominant and recessive edges, so the transceiver can stretch a dominant bit.
"""
from collections import deque
from protolib import *  # noqa: F401,F403
import can as K
import can_wire as C
PROGRAMS = ('advanced/can_unit.s',)

RXP = 6
MOUT = 4
MEND = 5
FRM = 256
B = 100


class Delay:
    """A transceiver path: dominant (0) edges arrive dd cycles late, recessive ones dr cycles late."""

    def __init__(self, dd, dr):
        self.dd, self.dr = dd, dr
        self.h = deque([1] * (max(dd, dr) + 1), maxlen=max(dd, dr) + 1)

    def step(self, v):
        self.h.append(v)
        a, b = self.h[-1 - self.dd], self.h[-1 - self.dr]
        return a & b if self.dr >= self.dd else a | b


class Bus(Device):
    """io0 (TXD) through the transmit path to the bus, the bus through the receive path to ui6 (RXD)."""

    def __init__(self, nodes, d1=5, d2=5, asym=0):
        self.nodes = nodes
        self.ui_mask = 1 << RXP
        self.ui_val = 1 << RXP
        self.tx = Delay(d1, d1 + asym)
        self.rx = Delay(d2, d2 + asym)
        self.level = 1

    def step(self, c, uo, io):
        lv = self.tx.step(io & 1)
        for n in self.nodes:
            lv &= n.drive(c, self.level)
        self.level = lv
        for n in self.nodes:
            n.see(c, lv)
        self.ui_val = self.rx.step(lv) << RXP


def frame_words(ident, data, rtr=0):
    """The host's layout of a frame to send: SOF to the end of the data field, the first (19 + 8n) mod 16 bits
    right-aligned in the first word, then 16 bits a word."""
    bits = C.unstuffed(ident, data, rtr)
    k = len(bits) % 16
    words = [sum(b << (k - 1 - i) for i, b in enumerate(bits[:k]))]
    for i in range(k, len(bits), 16):
        words.append(sum(b << (15 - j) for j, b in enumerate(bits[i:i + 16])))
    return words


def rx_words(ident, data, rtr=0, delim=1):
    """What a receiver that keeps SOF to the CRC delimiter, destuffed, MSB first in 16-bit words, must report for
    this frame: (words, bits in the last word)."""
    b = C.unstuffed(ident, data, rtr)
    c = crc15_can(b)
    b = b + [(c >> (14 - i)) & 1 for i in range(15)] + [delim]
    words = []
    for i in range(0, len(b), 16):
        ch = b[i:i + 16]
        words.append(sum(x << (len(ch) - 1 - j) for j, x in enumerate(ch)))
    return words, len(b) - 16 * (len(words) - 1)


def parse(got):
    """(MEND words, MOUT words) into reports, in order: ('tx', code) or ('rx', flags, last bits, words). Each
    frame's status says how many of the next MOUT words are its own."""
    ends, words = got
    out, i = [], 0
    for w in ends:
        if w >> 14 == 1:
            out.append(('tx', w & 0xFF))
        elif w >> 15:
            n = (w >> 8) & 7
            out.append(('rx', (w >> 5) & 7, w & 31, words[i:i + n]))
            i += n
        else:
            out.append(('junk', w))
    if i < len(words):
        out.append(('extra words', words[i:]))
    return out


def want_rx(ident, data, rtr=0, flags=1):
    w, n = rx_words(ident, data, rtr)
    return ('rx', flags, n, w)


def rig(nodes, d1=5, d2=5, asym=0, thread=0, align=0, depth=512, b=B, defines=None):
    bus = Bus(nodes, d1, d2, asym)
    r = Rig(devices=[bus], depth=depth, pullups=0x01)
    d = {'TID': thread, 'B': b}
    if b != B:
        d['SPU'] = b * 3 // 4
        d['SPF'] = b * 2 // 5
    d.update(defines or {})
    img = assemble(src('can_unit'), depth=depth, defines=d)
    r.load(img, {thread: 'start'})
    return r, img, bus


def run(nodes, sends=(), cycles=30000, thread=0, align=0, d1=5, d2=5, asym=0, b=B, defines=None, post_at=3000):
    """sends: (ident, data[, rtr]) frames for the chip, posted one after another once each report is in."""
    r, img, bus = rig(nodes, d1, d2, asym, thread, align, b=b, defines=defines)
    ends, words = [], []

    def host():
        yield post_at
        for s in sends:
            ident, data = s[0], s[1]
            rtr = s[2] if len(s) > 2 else 0
            for i, w in enumerate(frame_words(ident, data, rtr)):
                r.chip.host_wrmem(FRM + i, w)
            n = len(ends)
            yield from r.host.post(thread, len(data) if not rtr else s[3])
            while not any(w >> 14 == 1 and w & 0xFF != 2 for _, w in ends[n:]):
                yield 200

    def taker():
        while True:
            yield from r.host.take(MOUT, words, tries=1)
            yield from r.host.take(MEND, ends, tries=1)
    r.board.agent(host())
    r.board.agent(taker())
    r.start(align)
    r.run(cycles)
    return r, img, ([w for _, w in ends], [w for _, w in words]), bus


def sender(frames, t0=4000, gap=20, err=0.0, flip=None, b=B):
    """A reference sender for the frames (ident, data), each starting gap bit times after the last ended."""
    sched, t = [], t0
    actual = b * (1 + err)
    for fi, (ident, data) in enumerate(frames):
        bits = C.frame_bits(ident, data, flip_crc=(flip == fi))
        sched.append((t, bits))
        t += int(len(bits) * actual) + int(gap * actual)
    return K.Sender(sched, actual), t


def decode(r, b=B, err=0.0):
    return C.decode(r.wave('ui', RXP), b * (1 + err), lo=2000)


def acks(fr):
    return [f['ack'] for f in fr]


def bus_ack(r, t0, slot, b=B):
    """The ACK slot (bit slot of a frame a reference sender started at t0) reads dominant at RXD, 80% into it."""
    return r.wave('ui', RXP).at(int(t0 + (slot + 0.8) * b)) == 0


def flag_starts(r, lo=0):
    """Starts of dominant runs of 6 bit times or more on the bus side of RXD: error flags."""
    w = r.wave('ui', RXP)
    out, t0 = [], None
    for t, v in w.edges(lo):
        if v == 0:
            t0 = t
        elif t0 is not None:
            if t - t0 >= 6 * B - 20:
                out.append((t0, t - t0))
            t0 = None
    return out


FRAMES = [(0x555, [0xAA]), (0x123, [1, 2, 3, 4, 5, 6, 7, 8]), (0x000, [0] * 8), (0x7EF, [0xFF] * 8), (0x2A5, []),
          (0x0F0, [0xF0, 0x0F, 0xFF, 0x00])]


class LagRival(K.Rival):
    """can.Rival that starts at its own time, lam cycles after the winner's SOF on the bus, as a node whose
    transceiver loop and synchronisation take that long; it still drops out when it loses arbitration."""

    def see(self, c, lv):
        self.p = lv
        if self.t is not None and self.alive and not self.done:
            k = int((c - self.t) / self.bit)
            f = (c - self.t) / self.bit - k
            if 0.7 < f < 0.75 and k < len(self.bits) and self.bits[k] == 1 and lv == 0:
                self.alive = False


def third_node(lam, b=B):
    """The chip as a third node: two other nodes arbitrate, the loser starting lam cycles after the winner's SOF,
    so every dominant bit of the arbitration field is lam cycles longer at RXD (its falling edge from the winner,
    its rising edge from the loser). True if the chip reports the winner's frame and ACKs it, and the reference
    decoder (which resynchronizes on recessive-to-dominant edges only) reads it clean."""
    t0 = 4000
    win, lose = (0x244, [9]), (0x246, [1, 2])
    bw = C.frame_bits(*win)
    riv = LagRival(C.frame_bits(*lose), float(b), start=t0 + lam)
    r, img, w, bus = run([K.Sender([(t0, bw)], float(b)), riv], cycles=t0 + len(bw) * b + 3000, b=b)
    fr = decode(r, b=b)
    return (parse(w) == [want_rx(*win)] and not r.health() and len(fr) == 1 and fr[0]['id'] == win[0] and
            fr[0]['crc_ok'] and fr[0]['ack'] and not riv.alive)


def flag_start(ident, data, b=B):
    """A sender puts the run's value once more after five equal bits past bit 20 (a stuff error, Bosch 5) and then
    releases the bus, so the next dominant edge is the chip's error flag. Returns (where it starts, in bit times
    after the violating bit ends; the reports)."""
    st = C.stuff(C.with_crc(ident, data)[0])
    for k in range(5, len(st)):
        if st[k - 5:k] in ([0] * 5, [1] * 5) and k > 20:
            break
    t0 = 4000
    bad = st[:k] + [st[k - 1]] + [1] * 40
    r, img, w, bus = run([K.Sender([(t0, bad)], float(b))], cycles=t0 + len(bad) * b + 2000, b=b)
    e = [t for t, v in r.wave('io', 0).edges(t0) if v == 0]
    return ((e[0] - (t0 + (k + 1) * b)) / b if e else None), parse(w)


def crc_tail_stuffed():
    """Frames whose CRC sequence ends in five equal bits, so a stuff bit follows it (Bosch 5)."""
    out = []
    for ident in range(0x100, 0x7FF, 7):
        for data in ([ident & 255], []):
            b, c = C.with_crc(ident, data)
            if len(C.stuff(b)) > len(C.stuff(b[:-1])) + 1:
                out.append((ident, data, b[-1]))
        if len(out) >= 4 and {x[2] for x in out} == {0, 1}:
            break
    return out


def destuffed(st, k):
    """The bits a receiver keeps of the stuffed bits st[:k]: a bit after five equal ones is a stuff bit, dropped,
    and it starts the next run (Bosch 5)."""
    out, run, last = [], 0, None
    for x in st[:k]:
        if run == 5:
            last, run = x, 1
            continue
        run = run + 1 if x == last else 1
        last = x
        out.append(x)
    return out


def stuff_error_words(st, k):
    """What a receiver that keeps SOF on, destuffed, MSB first in 16-bit words, has when a stuff error at stuffed
    bit k ends the frame: None under 16 bits, else (bits in the last word, words), the last word holding the 0 to 15
    bits after the last full one."""
    b = destuffed(st, k)
    if len(b) < 16:
        return None
    m = len(b) - len(b) % 16
    words = [sum(x << (15 - j) for j, x in enumerate(b[i:i + 16])) for i in range(0, m, 16)]
    words.append(sum(x << (len(b) - m - 1 - j) for j, x in enumerate(b[m:])))
    return len(b) - m, words


def remote_stuffed(ident, dlc):
    """A remote frame's stuffed bits, SOF to the CRC sequence: RTR 1, no data field, the DLC as given."""
    bits = C.unstuffed(ident, [], 1)
    bits[15:19] = [(dlc >> (3 - i)) & 1 for i in range(4)]
    c = crc15_can(bits)
    return C.stuff(bits + [(c >> (14 - i)) & 1 for i in range(15)])


def stuff_positions(st):
    """The stuffed positions k at which a stuff violation can come: five equal bits before k (Bosch 5)."""
    return [k for k in range(5, len(st)) if st[k - 5:k] in ([0] * 5, [1] * 5)]


# Eight-byte frames stuffed close to the most that CAN's stuffing allows, whose stuffed bits hold the positions 112 to
# 119 that the fixed search below doesn't reach. 119, the stuff bit after the CRC sequence of the last frame, is the
# latest position at which any frame of 0 to 8 bytes can hold a stuff violation.
MOST_STUFFED = [(0x07C, [0x20, 0xF0, 0x07, 0xFC, 0x3C, 0x3C, 0x00, 0x00]),
                (0x7FE, [0x01, 0xE1, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00]),
                (0x41E, [0x3C, 0x3C, 0x01, 0x07, 0x87, 0x87, 0x87, 0x80]),
                (0x7FE, [0x3C, 0x3E, 0xF8, 0x03, 0xC3, 0xFE, 0x1E, 0x1E]),
                (0x7FE, [0x20, 0xFF, 0xFC, 0x3C, 0x01, 0xE1, 0xE1, 0xE0]),
                (0x7FE, [0x3F, 0xE7, 0xFE, 0x1E, 0x1E, 0x1E, 0x1E, 0x00]),
                (0x7FE, [0x01, 0xE1, 0xE1, 0xE1, 0xE7, 0xC3, 0xC0, 0x1E])]


def stuff_cover():
    """One frame for every stuffed position that can hold a stuff violation, over frames of 0 to 8 bytes, found by
    a fixed search and then MOST_STUFFED: (k, stuffed bits SOF to the CRC sequence), k from 5 to 119."""
    cover = {}
    for dlc in range(9):
        for fill in (0x00, 0xFF, 0x0F, 0xF0, 0x55, 0x33, 0x07, 0xE0):
            for ident in range(0, 0x7F0, 0x41):
                st = C.stuff(C.with_crc(ident, [fill] * dlc)[0])
                for k in stuff_positions(st):
                    cover.setdefault(k, st)
    for ident, data in MOST_STUFFED:
        st = C.stuff(C.with_crc(ident, data)[0])
        for k in stuff_positions(st):
            cover.setdefault(k, st)
    return sorted(cover.items())


def after_dlc_cover():
    """For data frames of 0 to 8 bytes and remote frames of DLC 0, 3 and 8: a frame whose stuffed bits can hold a
    stuff violation right after its 32nd bit, the end of the 16 bits that carry the DLC's bits 2 to 0, where the
    receiver learns the frame's length: (label, stuffed bits, k)."""
    out = []
    kinds = [('%d bytes' % n, n, 0) for n in range(9)] + [('remote, DLC %d' % n, n, 1) for n in (0, 3, 8)]
    for label, n, rtr in kinds:
        for seed in range(4096):
            ident = (seed * 0x29) % 0x700
            data = [(seed * 0x9D + j * 0x3B) & 0xFF for j in range(n)]
            st = remote_stuffed(ident, n) if rtr else C.stuff(C.with_crc(ident, data)[0])
            ks = [k for k in stuff_positions(st) if len(destuffed(st, k)) == 32]
            if ks:
                out.append((label, st, ks[0]))
                break
    return out


class Pulse:
    """Another node that pulls the bus dominant for n cycles from cycle t: a spike, or a dominant bit."""

    def __init__(self, t, n):
        self.t, self.n = t, n

    def drive(self, c, bus):
        return 0 if self.t <= c < self.t + self.n else 1

    def see(self, c, lv):
        pass


class Responder:
    """Another node's error handling, from Bosch CAN 2.0 part A (bit timing, error signalling, error delimiter),
    seeing the bus through its own receive path d2 and driving it through its transmit path d1: it hard-syncs on a
    falling edge of an idle bus, resynchronizes on recessive-to-dominant edges, samples at three quarters of the
    bit, and on six equal bits (a stuff error, or another node's flag) sends a flag of six dominant bits from its
    next bit; then it sends recessive bits, waits for a recessive sample and seven more, and is idle again."""

    def __init__(self, bit, d1=5, d2=5):
        self.b = bit
        self.rx = deque([1] * (d2 + 1), maxlen=d2 + 1)
        self.tx = deque([1] * (d1 + 1), maxlen=d1 + 1)
        self.state, self.prev = 'idle', 1
        self.flags = []                  # cycles where its flags start on its transmit pin

    def drive(self, c, bus):
        return self.tx[0]

    def see(self, c, lv):
        self.rx.append(lv)
        v, b, out = self.rx[0], self.b, 1
        fall = self.prev == 1 and v == 0
        if self.state == 'idle':
            if fall:
                self.state, self.bs, self.run, self.last = 'frame', c, 0, None
        elif self.state == 'frame':
            if fall:
                self.bs = c
            if c == self.bs + (3 * b) // 4:
                self.run = self.run + 1 if v == self.last else 1
                self.last = v
                if self.run == 6:
                    self.state, self.fs = 'flag', self.bs + b
                    self.flags.append(self.fs)
            if c >= self.bs + b:
                self.bs += b
        if self.state == 'flag':
            if self.fs <= c < self.fs + 6 * b:
                out = 0
            elif c >= self.fs + 6 * b:
                self.state, self.bs, self.rec = 'delim', c, 0
        elif self.state == 'delim':
            if fall:
                self.bs = c
            if c == self.bs + (3 * b) // 4:
                self.rec = self.rec + 1 if v else 0
                if self.rec >= 8:
                    self.state = 'idle'
            if c >= self.bs + b:
                self.bs += b
        self.prev = v
        self.tx.append(out)


class LocalBus(Bus):
    """Bus whose RXD for the chip alone is forced to val over cycles [t, t + n): a disturbance at its receiver."""

    def __init__(self, nodes, t, n, val, **kw):
        super().__init__(nodes, **kw)
        self.t, self.n, self.val = t, n, val

    def step(self, c, uo, io):
        super().step(c, uo, io)
        if self.t <= c < self.t + self.n:
            self.ui_val = self.val << RXP


def local_error(st, k, b=B, thread=0, rl=(5, 5)):
    """A stuff error that only the chip sees: a sender puts the stuffed bits st up to bit k, which five equal bits
    precede, then the stuff bit there and releases the bus, while the chip's RXD alone carries the run's value at
    bit k. A Responder with transceiver paths rl answers the chip's flag. Returns (the chip's flags, its first
    flag's length in bit times, the Responder's flags), counted until the bus has been idle long after."""
    t0 = 4000
    bits = st[:k] + [1 - st[k - 1]] + [1] * 40
    resp = Responder(b, *rl)
    bus = LocalBus([K.Sender([(t0, bits)], float(b)), resp], t0 + k * b + 5, b, st[k - 1])
    r = Rig(devices=[bus], depth=512, pullups=0x01)
    d = {'TID': thread, 'B': b}
    if b != B:
        d['SPU'], d['SPF'] = b * 3 // 4, b * 2 // 5
    r.load(assemble(src('can_unit'), depth=512, defines=d), {thread: 'start'})
    ends, words = [], []

    def taker():                         # the host takes every report, so the program never waits on a mailbox
        while True:
            yield from r.host.take(MOUT, words, tries=1)
            yield from r.host.take(MEND, ends, tries=1)
    r.board.agent(taker())
    r.start(thread)
    r.run(t0 + len(bits) * b + 20 * b)
    e = [(t, v) for t, v in r.wave('io', 0).edges(t0)]
    falls = [t for t, v in e if v == 0]
    ln = (e[1][0] - e[0][0]) / b if len(e) > 1 else None
    return len(falls), ln, len(resp.flags)


GOOD = (0x2A5, [])                       # the frame each stuff-error scenario sends afterwards


def span(xs):
    """'a to b bit times after the violating bit ends' for the flags that started, or 'no flag started'."""
    xs = [x for x in xs if x is not None]
    return 'flag starts %.2f to %.2f bit times after the violating bit ends' % (min(xs), max(xs)) if xs else \
        'no flag started'


# Where the flag starts, in cycles after the violating bit ends on the bus, with the rig's 100 ns each way through
# the transceiver: the range the program states (its header, section 11.2), the same from every word of a frame.
FLAG_AT = {100: (15, 18), 50: (27, 30)}


def flag_at(st, k, b=B, thread=0, align=0, d1=5, d2=5, gap=None):
    """A sender puts the stuffed bits st up to bit k, which five equal bits precede, then that run's value once
    more, a stuff error at bit k (Bosch 5), releases the bus, and 40 bit times later sends GOOD, or with gap, puts
    GOOD's SOF on the bus gap cycles after the violating bit ends there. Returns a dict: x, where the chip's first
    dominant level at io0 starts, in bit times after the violating bit ends on the bus; its length in bit times;
    flags, the number of dominant levels at io0 before GOOD starts; the reports; and whether GOOD was reported and
    ACKed."""
    t0 = 4000
    bad = st[:k] + [st[k - 1]] + [1] * (40 if gap is None else gap // b)
    gb = C.frame_bits(*GOOD)
    t1 = t0 + len(bad) * b if gap is None else t0 + (k + 1) * b + gap
    r, img, w, bus = run([K.Sender([(t0, bad), (t1, gb)], float(b))], cycles=t1 + len(gb) * b + 2000,
                         thread=thread, align=align, d1=d1, d2=d2, b=b)
    edges = [(t, v) for t, v in r.wave('io', 0).edges(t0) if t < t1]
    falls = [t for t, v in edges if v == 0]
    rises = [t for t, v in edges if v == 1 and falls and t > falls[0]]
    x = (falls[0] - (t0 + (k + 1) * b)) / b if falls else None
    ln = (rises[0] - falls[0]) / b if falls and rises else None
    fr = [f for f in decode(r, b=b) if f['t'] >= t1]
    rep = parse(w)
    good = bool(rep) and rep[-1] == want_rx(*GOOD) and len(fr) == 1 and fr[0]['ack'] and fr[0]['crc_ok']
    return dict(x=x, len=ln, flags=len(falls), rep=rep, good=good, health=r.health())


def flag_ok(res, st, k, b, d2):
    """flag_at's result as Bosch's error signalling asks: one error flag of 6 bit times, starting in the next bit
    and not before the violating bit has ended at RXD (d2 later than on the bus); the reception reported as a stuff
    error from 16 bits on (a reception under 16 bits isn't reported); GOOD after it received and ACKed."""
    want = stuff_error_words(st, k)
    rx = [x for x in res['rep'] if x[0] == 'rx']
    if want is None:
        rep_ok = len(rx) == 1 and len(res['rep']) == 1
    else:
        rep_ok = len(rx) == 2 and len(res['rep']) == 2 and rx[0][1] & 2 and (rx[0][2], rx[0][3]) == want
    return (res['flags'] == 1 and res['x'] is not None and d2 / b <= res['x'] < 1 and res['len'] == 6 and rep_ok
            and res['good'] and not res['health'])


def main(log):
    log.group('advanced/can_unit.s', title='CAN 2.0A node on the shift unit')
    for name, v, want in anchors()[2:]:
        log.check('reference ' + name, v == want, hex(v))
    want = [want_rx(i, d) for i, d in FRAMES]

    s, t = sender(FRAMES)
    r, img, w, bus = run([s], cycles=t + 3000)
    log.line('INFO  can_unit is %d words' % img.size)
    rep = parse(w)
    fr = decode(r)
    log.check('RX 6 frames incl. all-zero and all-one stuffing: words, bit count and good end flag reported',
              rep == want and not r.health(), (rep[:1], r.health()))
    log.check('RX: the chip ACKs every frame in its slot, the pad decoder agrees', acks(fr) == [True] * 6 and
              all(f['crc_ok'] and f['eof_ok'] and not f['errs'] for f in fr), [(f['id'], f['ack'], f['errs']) for f in fr])

    tails = crc_tail_stuffed()
    s, t = sender([(i, d) for i, d, _ in tails])
    r, img, w, bus = run([s], cycles=t + 3000)
    fr = decode(r)
    log.check('RX %d frames with a stuff bit after the CRC sequence (both values): reported and ACKed' % len(tails),
              parse(w) == [want_rx(i, d) for i, d, _ in tails] and acks(fr) == [True] * len(tails),
              (parse(w)[:1], acks(fr)))

    rtr = [(0x2A5, 3), (0x111, 8), (0x7EF, 0)]
    sched, t = [], 4000
    for ident, dlc in rtr:
        bits = C.unstuffed(ident, [], 1)
        bits[15:19] = [(dlc >> (3 - i)) & 1 for i in range(4)]
        c = crc15_can(bits)
        st = C.stuff(bits + [(c >> (14 - i)) & 1 for i in range(15)]) + [1] * 10
        sched.append((t, st))
        t += len(st) * B + 20 * B
    r, img, w, bus = run([K.Sender(sched, float(B))], cycles=t + 3000)
    exp = []
    for ident, dlc in rtr:
        bits = C.unstuffed(ident, [], 1)
        bits[15:19] = [(dlc >> (3 - i)) & 1 for i in range(4)]
        c = crc15_can(bits)
        bb = bits + [(c >> (14 - i)) & 1 for i in range(15)] + [1]
        ws = [sum(x << (len(bb[i:i + 16]) - 1 - j) for j, x in enumerate(bb[i:i + 16])) for i in range(0, len(bb), 16)]
        exp.append(('rx', 1, len(bb) - 16 * (len(ws) - 1), ws))
    ok_ack = [bus_ack(r, t0, len(st) - 9) for t0, st in sched]
    log.check('RX remote frames with DLC 3, 8 and 0: no data field, reported, ACKed', parse(w) == exp and
              ok_ack == [True] * 3, (parse(w)[:1], ok_ack))

    good = 0
    runs = 0
    for th in range(4):
        for al in range(4):
            for ph in (0, 37):
                s, t = sender(FRAMES[:3], t0=4000 + ph + 13 * th)
                r, img, w, bus = run([s], cycles=t + 3000, thread=th, align=al)
                runs += 1
                good += parse(w) == want[:3] and acks(decode(r)) == [True] * 3 and not r.health()
    log.check('RX on every thread and alignment at 2 sender phases', good == runs, '%d of %d' % (good, runs))

    for e in (0.005, -0.005, 0.01, -0.01, 0.015, -0.015, 0.02, -0.02, 0.03, -0.03):
        s, t = sender(FRAMES, err=e)
        r, img, w, bus = run([s], cycles=t + 3000)
        fr = decode(r, err=e)
        log.check('RX sender %+.1f%%' % (e * 100), parse(w) == want and acks(fr) == [True] * 6, (parse(w)[:1], acks(fr)))
    for d in ((2, 2), (11, 11)):
        s, t = sender(FRAMES)
        r, img, w, bus = run([s], cycles=t + 3000, d1=d[0], d2=d[1])
        log.check('RX transceiver loop %d ns' % ((d[0] + d[1]) * 20), parse(w) == want and acks(decode(r)) == [True] * 6)
    for a in (2, 5, 10):
        s, t = sender(FRAMES)
        r, img, w, bus = run([s], cycles=t + 3000, asym=a)
        fr = decode(r)
        log.check('RX dominant bits stretched by %d ns in each transceiver path' % (a * 20),
                  parse(w) == want and acks(fr) == [True] * 6, (parse(w)[:1], acks(fr)))

    for bit in (100, 50):
        dl0 = [(0x7EF, []), (0x000, []), (0x555, []), (0x2A5, []), (0x123, [])]
        for name, frames in (('6 frames', FRAMES), ('DLC 0 frames', dl0)):
            s, t = sender(frames, gap=3, b=bit)
            r, img, w, bus = run([s], cycles=t + 3000, b=bit)
            log.check('RX %s back to back at the shortest interframe space, %d kbit/s' % (name, 50000 // bit),
                      parse(w) == [want_rx(i, d) for i, d in frames] and acks(decode(r, b=bit)) == [True] * len(frames)
                      and not r.health(), (parse(w)[:1], r.health()))
    for a in (26, 50, 70):
        s, t = sender(FRAMES)
        r, img, w, bus = run([s], cycles=t + 3000, asym=a)
        log.check('RX dominant bits stretched by %d cycles on the RXD path: only falling edges resynchronize (RACQ), '
                  'so the stretch moves no sample' % a, parse(w) == want and acks(decode(r)) == [True] * 6,
                  (parse(w)[:1], acks(decode(r))))
    ok = []
    for a in (72, 76):
        s, t = sender(FRAMES)
        r, img, w, bus = run([s], cycles=t + 3000, asym=a)
        ok.append(parse(w) == want and acks(decode(r)) == [True] * 6)
    log.check('limit: a dominant bit stretched by 72 cycles on the RXD path is clean, by 76 not (SPU = 75: the '
              'next sample lands in the stretch)', ok == [True, False], ok)
    for bit, lams in ((100, (0, 25, 50, 70)), (50, (0, 13, 25, 35))):
        res = {lam: third_node(lam, bit) for lam in lams}
        log.check('the chip as a third node at %d kbit/s: two other nodes arbitrate, the loser %s cycles after the '
                  'winner on the bus, every dominant bit of the arbitration field that much longer at RXD; the '
                  'winner\'s frame reported and ACKed at each' % (50000 // bit, ', '.join(map(str, lams))),
                  all(res.values()), res)
    lim = {}
    for bit, lams in ((100, range(66, 90, 3)), (50, range(30, 46, 2))):
        last = None
        for lam in lams:
            if not third_node(lam, bit):
                break
            last = lam
        lim[50000 // bit] = (last, lam)
    log.info('third node, the last lag clean and the first that fails, in cycles: %s (kbit/s: (clean, fails))' % lim)
    res = [flag_start(i, d) for i, d in ((0x123, [1, 2]), (0x7EF, [0xFF] * 8), (0x000, [0] * 8))]
    log.check('a stuff error: the reception ends at the violating bit (SMODE 2), reported with end flag 1, and the '
              'error flag starts within the next bit (Bosch CAN 2.0, part A, error signalling)',
              all(x is not None and 0 <= x < 1 and rep and rep[-1][:2] == ('rx', 2) for x, rep in res),
              [(x, rep[-1][:2] if rep else None) for x, rep in res])

    s, t = sender(FRAMES[:3], flip=1)
    r, img, w, bus = run([s], cycles=t + 3000)
    fr = decode(r)
    rep = parse(w)
    bad = list(want[:3])
    ws, n = rx_words(*FRAMES[1])
    bits = C.unstuffed(*FRAMES[1])
    cc = crc15_can(bits) ^ 4
    bb = bits + [(cc >> (14 - i)) & 1 for i in range(15)] + [1]
    bad[1] = ('rx', 0, n, [sum(x << (len(bb[i:i + 16]) - 1 - j) for j, x in enumerate(bb[i:i + 16]))
                           for i in range(0, len(bb), 16)])
    fl = flag_starts(r)
    eof1 = [f['end'] - 7 * B for f in fr]
    log.check('RX corrupted CRC: reported with flags 0, no ACK, an error flag from the bit after the ACK delimiter',
              rep == bad and acks(fr)[:2] == [True, False] and len(fl) == 1 and
              abs(fl[0][0] - 10 - (fr[1]['t'] + (len(C.frame_bits(*FRAMES[1])) - 7) * B)) <= 4,
              (rep[1:2], acks(fr), fl, [f['t'] for f in fr]))

    st = C.stuff(C.with_crc(0x123, [1, 2])[0])
    for k in range(5, len(st)):
        if st[k - 5:k] in ([0] * 5, [1] * 5):
            bad6 = st[:k] + [st[k - 1]] + [st[k - 1]] * 2 + [1] * 30
            break
    r, img, w, bus = run([K.Sender([(4000, bad6)], float(B))], cycles=4000 + len(bad6) * B + 3000)
    rep = parse(w)
    fl = flag_starts(r)
    log.check('RX six equal bits at bit %d: the reception ends with end flag 1, no ACK, an error flag' % k,
              len(rep) == (0 if k < 17 else 1) and all(x[0] == 'rx' and x[1] & 2 for x in rep) and len(fl) == 1,
              (rep, fl))

    log.info('stuff errors at every position')
    cover = stuff_cover()
    for bit in (100, 50):
        xs, bad = [], []
        lo, hi = FLAG_AT[bit]
        for i, (k, st) in enumerate(cover):
            res = flag_at(st, k, b=bit, thread=i % 4, align=(i // 4) % 4)
            if res['x'] is not None:
                xs.append(res['x'])
            if not flag_ok(res, st, k, bit, 5) or not lo <= round(res['x'] * bit) <= hi:
                bad.append((k, res['x'], res['len'], res['flags'], res['rep'][:1], res['good']))
        log.check('a stuff error at each of the %d positions that can hold one, k %d to %d (frames of 0 to 8 bytes, '
                  'every thread and alignment), %d kbit/s: one error flag of 6 bit times, starting in the next bit '
                  '(Bosch CAN 2.0, part A, error signalling) and not before the violating bit has ended at RXD, %d to '
                  '%d cycles after it ends at every position; reported as a stuff error from 16 bits on, not before; '
                  'the frame after it received and ACKed' % (len(cover), cover[0][0], cover[-1][0], 50000 // bit, lo,
                                                             hi),
                  not bad and len(xs) == len(cover) and [k for k, _ in cover] == list(range(5, 120)),
                  bad[:3] if bad else span(xs))
    for bit in (100, 50):
        xs, bad, n = [], [], 0
        lo, hi = FLAG_AT[bit]
        for label, st, k in after_dlc_cover():
            for al in range(4):
                res = flag_at(st, k, b=bit, thread=(al + 1) % 4, align=al)
                n += 1
                if res['x'] is not None:
                    xs.append(res['x'])
                if not flag_ok(res, st, k, bit, 5) or not lo <= round(res['x'] * bit) <= hi:
                    bad.append((label, al, res['x'], res['len'], res['flags'], res['good']))
        log.check('a stuff error in the bit right after the 32nd, as the receiver takes the 16 bits that carry the '
                  'DLC\'s bits 2 to 0: frames of 0 to 8 bytes and remote frames of DLC 0, 3 and 8, every alignment, '
                  '%d kbit/s: the error flag in the next bit, as at every other position' % (50000 // bit),
                  not bad and len(xs) == n == 48, bad[:3] if bad else '%d runs, %s' % (n, span(xs)))
    # A sender whose frame an error frame broke sends it again at its first chance: its SOF follows the 6 flag
    # bits, the 8 of the error delimiter and the 3 of the intermission (error frame, interframe space). Here it
    # comes 17 bit times after the latest flag start the program states, plus the 100 ns to the bus, so the hunt
    # must have restarted by then.
    for bit in (100, 50):
        bad = []
        gap = FLAG_AT[bit][1] + 5 + 17 * bit
        for i, (k, st) in enumerate(cover):
            res = flag_at(st, k, b=bit, thread=(i + 1) % 4, align=(i // 4 + 2) % 4, gap=gap)
            if not flag_ok(res, st, k, bit, 5):
                bad.append((k, res['x'], res['len'], res['flags'], res['rep'][:1], res['good']))
        log.check('a stuff error at each of the %d positions, %d kbit/s, then the next frame at the earliest start '
                  'the error frame allows, 17 bit times after the flag starts (6 flag bits, the error delimiter and '
                  'the intermission), as a sender that sends again at once: one error flag of 6 bit times in the '
                  'next bit, and the next frame received and ACKed' % (len(cover), 50000 // bit),
                  not bad, bad[:3] if bad else '%d runs, the next frame %d cycles after the violating bit ends'
                  % (len(cover), gap))
    # The longest loop at each rate whose ACK still reaches the bus before the 72% sample: at 1 Mbit/s a 440 ns loop
    # puts the ACK 37 cycles into its 50-cycle slot, 380 ns 35.
    for bit, (d1, d2) in ((100, (11, 11)), (50, (9, 10))):
        xs, bad = [], []
        for i, (k, st) in enumerate(cover[::4]):
            res = flag_at(st, k, b=bit, thread=(i + 2) % 4, align=i % 4, d1=d1, d2=d2)
            if res['x'] is not None:
                xs.append(res['x'])
            if not flag_ok(res, st, k, bit, d2):
                bad.append((k, res['x'], res['len'], res['flags'], res['good']))
        log.check('a stuff error at every fourth position with a transceiver loop of %d ns, %d kbit/s: the error '
                  'flag in the next bit and not before the violating bit has ended at RXD, no flag after it, the '
                  'frame after it received and ACKed' % (20 * (d1 + d2), 50000 // bit), not bad and xs,
                  bad[:3] if bad else span(xs))
    # Far longer loops than a transceiver's, so that the node's own flag comes back late: the hunt restarts eight bit
    # times after the flag's commit, which must still be after the flag's own edges. The frame after it isn't judged:
    # its ACK comes too late in the slot at these loops.
    for bit, (d1, d2) in ((100, (30, 30)), (50, (20, 20))):
        bad, xs = [], []
        early = [(k, st) for k, st in cover if len(destuffed(st, k)) < 16]
        for i, (k, st) in enumerate(early):
            res = flag_at(st, k, b=bit, thread=i % 4, align=(i + 1) % 4, d1=d1, d2=d2)
            if res['x'] is not None:
                xs.append(res['x'])
            if not (res['flags'] == 1 and res['x'] is not None and d2 / bit <= res['x'] < 1 and res['len'] == 6):
                bad.append((k, res['x'], res['len'], res['flags']))
        log.check('a stuff error in SOF to DLC bit 3 (%d positions) with a transceiver loop of %d ns, %d kbit/s: one '
                  'error flag of 6 bit times in the next bit, and no second one from the node\'s own edge coming '
                  'back' % (len(early), 20 * (d1 + d2), 50000 // bit), not bad and len(xs) == len(early) == 13,
                  bad[:3] if bad else span(xs))
    # After its ACK the node restarts the hunt one bit time after the ACK's commit, which the program states holds
    # for a transceiver loop of up to B - 4 cycles: at that loop its own ACK's edge must come back before the hunt.
    for bit in (100, 50):
        loop = bit - 4
        d1, d2 = loop // 2, loop - loop // 2
        fa, fb = (0x2A5, []), (0x123, [0x55])
        ba, bb = C.frame_bits(*fa), C.frame_bits(*fb)
        t0 = 4000
        t1 = t0 + (len(ba) + 30) * bit
        bad = []
        for th in range(4):
            r, img, w, bus = run([K.Sender([(t0, ba), (t1, bb)], float(bit))], cycles=t1 + len(bb) * bit + 3000,
                                 thread=th, align=(th + 1) % 4, d1=d1, d2=d2, b=bit)
            e = [(t, v) for t, v in r.wave('io', 0).edges(t0)]
            lows = [e[i + 1][0] - e[i][0] for i in range(len(e) - 1) if e[i][1] == 0]
            if lows != [bit, bit] or parse(w) != [want_rx(*fa), want_rx(*fb)] or r.health():
                bad.append((th, lows, parse(w)[:2]))
        log.check('two frames received with a transceiver loop of B - 4 cycles (%d ns), %d kbit/s, every thread: after '
                  'each ACK the hunt starts past the ACK\'s own edge, so the node drives its two ACKs of one bit time '
                  'and nothing else, and reports both frames' % (20 * loop, 50000 // bit), not bad,
                  bad[:2] if bad else '4 runs, 2 ACKs each')
    for bit in (100, 50):
        res = []
        for n, al in ((bit // 10, 0), (bit * 3 // 5, 1), (bit * 7 // 10, 2)):
            gb = C.frame_bits(*GOOD)
            t0, t1 = 4000, 4000 + 30 * bit
            r, img, w, bus = run([Pulse(t0, n), K.Sender([(t1, gb)], float(bit))], cycles=t1 + len(gb) * bit + 2000,
                                 align=al, b=bit)
            fl = [t for t, v in r.wave('io', 0).edges(t0) if v == 0 and t < t1]
            fr = [f for f in decode(r, b=bit) if f['t'] >= t1]
            res.append((n, not fl, parse(w) == [want_rx(*GOOD)], len(fr) == 1 and fr[0]['ack']))
        log.check('a spike of %s cycles on an idle bus, %d kbit/s: the receive half starts on its falling edge and '
                  'samples recessive (the SOF is not sampled dominant, so it is no SOF), and the node sends no flag '
                  'and reports nothing; the next frame is received and ACKed' % (
                      ', '.join(str(x[0]) for x in res), 50000 // bit), all(all(x[1:]) for x in res), res)
    for bit in (100, 50):
        t0, gb = 4000, C.frame_bits(*GOOD)
        t1 = t0 + 30 * bit
        r, img, w, bus = run([Pulse(t0, bit), K.Sender([(t1, gb)], float(bit))], cycles=t1 + len(gb) * bit + 2000,
                             align=3, b=bit)
        e = [(t, v) for t, v in r.wave('io', 0).edges(t0) if t < t1]
        falls = [t for t, v in e if v == 0]
        x = (falls[0] - (t0 + 7 * bit)) / bit if falls else None
        ln = (e[1][0] - e[0][0]) / bit if len(e) > 1 else None
        fr = [f for f in decode(r, b=bit) if f['t'] >= t1]
        log.check('a dominant bit alone on an idle bus, %d kbit/s: a SOF and then recessive bits, so its seventh bit '
                  'is a stuff error (Bosch 5); one error flag of 6 bit times from the next bit, nothing reported, '
                  'the next frame received and ACKed' % (50000 // bit),
                  len(falls) == 1 and x is not None and 5 / bit <= x < 1 and ln == 6 and
                  parse(w) == [want_rx(*GOOD)] and len(fr) == 1 and fr[0]['ack'], (x, ln, len(falls), parse(w)))
    for bit in (100, 50):
        res, bad = [], []
        for i, (k, st) in enumerate(cover[::3]):
            rl = ((5, 5), (11, 11))[i % 2]
            n, ln, nr = local_error(st, k, b=bit, thread=i % 4, rl=rl)
            res.append(n)
            if not (n == 1 and ln == 6 and nr == 1):
                bad.append((k, rl, n, ln, nr))
        log.check('a stuff error that only the chip sees, at every third position, with another node that answers '
                  'error flags (Bosch CAN 2.0, part A, error signalling and error delimiter; its transceiver loop 200 '
                  'or 440 ns), %d kbit/s: the chip sends one flag of 6 bit times and none for the other node\'s, '
                  'which starts after it when the bus carried a recessive stuff bit there' % (50000 // bit),
                  not bad and len(res) == len(cover[::3]), bad[:3] if bad else '%d runs, one flag each' % len(res))

    log.info('transmit')
    for ident, data in FRAMES:
        n = len(C.frame_bits(ident, data)) - 9
        r, img, w, bus = run([K.Acker([n], float(B))], sends=[(ident, data)])
        fr = decode(r)
        f = fr[0] if fr else {}
        log.check('TX id %#x, %d bytes: bus decode, CRC, ACK, report' % (ident, len(data)),
                  len(fr) == 1 and f['id'] == ident and f['data'] == list(data) and f['crc_ok'] and f['ack'] and
                  f['eof_ok'] and parse(w) == [('tx', 0)] and not r.health(),
                  (f.get('id'), f.get('crc_ok'), f.get('ack'), parse(w), f.get('errs'), r.health()))
    for ident, data, last in tails[:2]:
        n = len(C.frame_bits(ident, data)) - 9
        r, img, w, bus = run([K.Acker([n], float(B))], sends=[(ident, data)])
        fr = decode(r)
        log.check('TX id %#x: the CRC sequence ends in five %ds, a stuff bit follows it, ACKed' % (ident, last),
                  len(fr) == 1 and fr[0]['crc_ok'] and fr[0]['ack'] and fr[0]['stuffbits'] >= 1 and
                  parse(w) == [('tx', 0)], (fr[0] if fr else None, parse(w)))
    ident, data = 0x2F1, [0x5A, 0xFF, 0x00]
    n = len(C.frame_bits(ident, data)) - 9
    r, img, w, bus = run([K.Acker([n], 50.0)], sends=[(ident, data)], b=50)
    fr = decode(r, b=50)
    log.check('TX at 1 Mbit/s: bus decode, CRC, ACK, report', len(fr) == 1 and fr[0]['crc_ok'] and fr[0]['ack'] and
              parse(w) == [('tx', 0)] and not r.health(), (parse(w), r.health()))
    bits = C.frame_bits(*FRAMES[1])
    r, img, w, bus = run([K.Acker([len(C.frame_bits(0x555, [0xAA])) - 9], 100.0), K.Sender([(9000, bits)], 100.0)],
                         sends=[(0x555, [0xAA])], cycles=9000 + len(bits) * 100 + 3000)
    fr = decode(r)
    log.check('TX a frame, then receive one that starts 3 bit times after its intermission: the receive half hunts '
              'again with CTX 0',
              parse(w) == [('tx', 0), want_rx(*FRAMES[1])] and acks(fr) == [True, True] and not r.health(),
              (parse(w), acks(fr)))
    r, img, w, bus = run([], sends=[(0x123, [1, 2])])
    fr = decode(r)
    fl = flag_starts(r)
    log.check('TX nobody ACKs: report 1, an error flag from the ACK delimiter on',
              parse(w) == [('tx', 1)] and fr and fr[0]['crc_ok'] and not fr[0]['ack'] and len(fl) == 1 and
              abs(fl[0][0] - (fr[0]['t'] + (len(C.frame_bits(0x123, [1, 2])) - 8) * B)) <= 2 * 5 + 2 and not r.health(),
              (parse(w), acks(fr), fl, [f['t'] for f in fr], r.health()))
    good, runs = 0, 0
    for th in range(4):
        for al in range(4):
            for d in ((2, 2), (11, 11)):
                ident, data = 0x2F1, [0x5A, 0xFF, 0x00]
                n = len(C.frame_bits(ident, data)) - 9
                r, img, w, bus = run([K.Acker([n], float(B))], sends=[(ident, data)], thread=th, align=al,
                                     d1=d[0], d2=d[1])
                fr = decode(r)
                runs += 1
                good += len(fr) == 1 and fr[0]['crc_ok'] and fr[0]['ack'] and parse(w) == [('tx', 0)] and not r.health()
    log.check('TX every thread and alignment x 2 loop delays', good == runs, '%d of %d' % (good, runs))
    frames = FRAMES[:4]
    r, img, w, bus = run([K.Acker([len(C.frame_bits(i, d)) - 9 for i, d in frames], float(B))], sends=frames,
                         cycles=60000)
    fr = decode(r)
    ed = r.wave('ui', RXP).edges(2000)
    runs1 = [(t - ed[k - 1][0]) / B for k, (t, v) in enumerate(ed) if k and v == 0 and ed[k - 1][1] == 1]
    gaps = [g for g in runs1 if g > 6.5]     # inside a frame stuffing keeps a recessive run to 6 bits at most
    log.check('TX %d frames back to back, each posted as the last is reported: all ACKed, and each SOF at least 11 '
              'recessive bit times after the ACK slot before it (ACK delimiter, end of frame, intermission)' % len(frames),
              parse(w) == [('tx', 0)] * len(frames) and acks(fr) == [True] * len(frames) and
              len(gaps) == len(frames) - 1 and min(gaps) >= 11 and not r.health(), (parse(w), acks(fr), gaps))

    log.info('arbitration')
    mine, theirs = (0x246, [1, 2]), (0x244, [9])
    lm, lt = len(C.frame_bits(*mine)) - 9, len(C.frame_bits(*theirs)) - 9
    # Posted at 4000, so that a rival 1000 cycles ahead still starts after 2500, where the Acker begins to look
    # for SOFs: a rival that started before that would have its SOF missed and its ACK put in the end of frame.
    r, img, w, bus = run([K.Acker([lm], float(B))], sends=[mine], post_at=4000)
    sof = [t for t, v in r.wave('io', 0).edges(4000) if v == 0][0]
    for early in (None, 1, 3, 8, 13, 20, 200, 1000):
        riv = K.Rival(C.frame_bits(*theirs), float(B), start=None if early is None else sof - early)
        r, img, w, bus = run([riv, K.Acker([lt, lm], float(B))], sends=[mine], cycles=40000, post_at=4000)
        fr = decode(r)
        rep = parse(w)
        seen = [(f['id'], f['crc_ok'], f['ack'], f['eof_ok']) for f in fr]
        lost = rep == [('tx', 2), want_rx(*theirs), ('tx', 0)]
        after = rep == [want_rx(*theirs), ('tx', 0)]
        log.check('rival 0x244 %s: %s; its frame and then ours intact, ACKed and with a clean end of frame on the '
                  'bus' % ('starts on our SOF' if early is None else 'starts %d cycles before our SOF' % early,
                           'we lose arbitration and receive its frame with a good CRC' if lost else
                           'we receive its frame first'),
                  (lost or (after and early is not None)) and seen == [(0x244, True, True, True),
                                                                       (0x246, True, True, True)],
                  (rep[:3], seen[:3]))
    mine2, theirs2 = (0x244, [7, 8]), (0x246, [9])
    riv2 = K.Rival(C.frame_bits(*theirs2), float(B))
    b2 = C.frame_bits(*mine2)
    r, img, w, bus = run([riv2, K.Acker([len(b2) - 9], float(B))], sends=[mine2], cycles=30000)
    fr = decode(r)
    log.check('we win arbitration: rival drops out, our frame clean, report 0', parse(w) == [('tx', 0)] and len(fr) == 1
              and fr[0]['id'] == 0x244 and fr[0]['crc_ok'] and fr[0]['ack'] and not riv2.alive,
              (parse(w), [(f['id'], f['crc_ok'], f['ack']) for f in fr], riv2.alive))
