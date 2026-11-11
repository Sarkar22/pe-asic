"""CAN node firmware on a wired-AND bus with transceiver delay. Oracles: reference frames built from
can_wire.py (CRC anchored on the published check value) and the pad-level decoder in can_wire.py."""
from collections import deque
from protolib import *  # noqa: F401,F403
PROGRAMS = ('advanced/can.s',)
import can_wire as C

TXP, RXP = 5, 6


class Bus(Device):
    """TX pad goes through d1 cycles to the bus, bus through d2 cycles to the RX pad (transceiver)."""

    def __init__(self, nodes, d1=5, d2=5):
        self.nodes = nodes
        self.ui_mask = 1 << RXP
        self.ui_val = 1 << RXP
        self.q1 = deque([1] * d1)
        self.q2 = deque([1] * d2)
        self.level = 1

    def step(self, c, uo, io):
        tx = self.q1.popleft()
        self.q1.append((uo >> TXP) & 1)
        lv = tx
        for n in self.nodes:
            lv &= n.drive(c, self.level)
        self.level = lv
        for n in self.nodes:
            n.see(c, lv)
        self.q2.append(lv)
        self.ui_val = self.q2.popleft() << RXP


class Sender:
    """Reference transmitter. bits: bus bits with the ACK slot recessive, one bit per bit time."""

    def __init__(self, frames, bit):
        self.frames = frames            # list of (start cycle, bits)
        self.bit = bit
        self.i = 0

    def drive(self, c, bus):
        if self.i >= len(self.frames):
            return 1
        t_start, bits = self.frames[self.i]
        if c < t_start:
            return 1
        k = int((c - t_start) / self.bit)
        if k >= len(bits):
            self.i += 1
            return 1
        return bits[k]

    def see(self, c, lv):
        pass


class Acker:
    """Stands for another node: ACKs a frame whose stuffed length it was told, from the SOF edge seen
    on the bus. It does not decode, so a wrong frame would still get ACKed; the decoder catches that."""

    def __init__(self, lengths, bit):
        self.q = list(lengths)
        self.bit = bit
        self.p = 1
        self.t = None

    def drive(self, c, bus):
        if self.t is None or not self.q:
            return 1
        a = self.t + self.q[0] * self.bit
        if a + self.bit * 0.1 <= c < a + self.bit * 1.05:
            return 0
        if c >= a + self.bit * 1.05:
            self.q.pop(0)
            self.t = None
        return 1

    def see(self, c, lv):
        if self.p and not lv and self.t is None and c > 2500:   # uo pins reset low: skip that dominant
            self.t = c
        self.p = lv


class Rival:
    """A second transmitter that starts on our SOF edge (or `first` cycles before it could), so the
    two are in arbitration. It wins or loses by the wire, and stops driving if it loses."""

    def __init__(self, bits, bit, start=None):
        self.bits, self.bit = bits, bit
        self.t, self.start = None, start
        self.p = 1
        self.alive = True
        self.done = False

    def drive(self, c, bus):
        if self.start is not None and self.t is None and c >= self.start:
            self.t = c
        if self.t is None or not self.alive or self.done:
            return 1
        k = int((c - self.t) / self.bit)
        if k >= len(self.bits):
            self.done = True
            return 1
        return self.bits[k]

    def see(self, c, lv):
        if self.t is None and self.p and not lv and c > 2500:
            self.t = c
        self.p = lv
        if self.t is not None and self.alive and not self.done:
            k = int((c - self.t + self.bit * 0.3) / self.bit)
            f = (c - self.t) / self.bit - int((c - self.t) / self.bit)
            if 0.7 < f < 0.75 and k - 1 < len(self.bits) and self.bits[int((c - self.t) / self.bit)] == 1 and lv == 0:
                self.alive = False


def pack(ident, data):
    bits = C.unstuffed(ident, data)
    bits = bits[1:]                     # SOF comes from the pre-armed wait
    words = []
    for i in range(0, len(bits), 16):
        ch = bits[i:i + 16]
        ch = ch + [0] * (16 - len(ch))
        words.append(sum(b << (15 - k) for k, b in enumerate(ch)))
    return words, len(bits)


def rig(nodes, depth=512, d1=5, d2=5, bit=100, name='can', defines=None, starts=None):
    bus = Bus(nodes, d1, d2)
    r = Rig(devices=[bus], depth=depth)
    img = assemble(src(name), depth=depth, defines=defines)
    r.load(img, starts or {0: 'dec', 1: 'samp', 2: 'tx'})
    return r, img, bus


def rx_words(got):
    return [w for _, w in got]


def parse_reports(words):
    """decoder words to frames. Header bytes: ID10..9, ID8..1, then ID0 RTR IDE r0 DLC."""
    cur = []
    frames = []
    for w in words:
        if w >= 0x100:
            frames.append((cur, w & 255))
            cur = []
        else:
            cur.append(w)
    res = []
    for cur, st in frames:
        if len(cur) < 3:
            res.append((None, None, None, st))
            continue
        a, b, c = cur[0] & 3, cur[1] & 255, cur[2] & 255
        ident = (a << 9) | (b << 1) | (c >> 7)
        dlc = c & 15
        data = [x & 255 for x in cur[3:3 + dlc]]
        res.append((ident, dlc, data, st))
    return res


def tx_case(ident, data, nodes=None, acked=True, bit=100, align=0, d1=5, d2=5, retries=0, run=26000):
    bits = C.frame_bits(ident, data)
    n = len(bits) - 10 + 1              # SOF to the ACK slot
    nodes = nodes if nodes is not None else ([Acker([n], bit)] if acked else [])
    r, img, bus = rig(nodes, d1=d1, d2=d2)
    words, nb = pack(ident, data)
    for i, w in enumerate(words):
        r.chip.host_wrmem(200 + i, w)
    st, got = [], []

    def agent():
        yield 3000
        yield from r.host.post(2, nb + 1)
        yield from r.host.take(5, st)
        for _ in range(retries):
            yield 6000
            yield from r.host.post(2, nb + 1)
            yield from r.host.take(5, st)

    def agent2():
        while True:
            yield from r.host.take(4, got, tries=1)
    r.board.agent(agent())
    r.board.agent(agent2())
    r.start(align)
    r.run(run)
    return r, img, [w for _, w in st], [w for _, w in got]


def rx_case(frames, err=0.0, bit=100, ack=True, gap=30, align=0, flip=None, d1=5, d2=5, t_start=4000):
    sched, lens = [], []
    t = t_start
    actual = bit * (1 + err)
    for fi, (ident, data) in enumerate(frames):
        bits = C.frame_bits(ident, data, flip_crc=(flip == fi))
        sched.append((t, bits))
        lens.append(len(bits))
        t += int(len(bits) * actual) + int(gap * actual)
    s = Sender(sched, actual)
    r, img, bus = rig([s], d1=d1, d2=d2)
    got = []

    def agent():
        while True:
            yield from r.host.take(4, got, tries=1)
    r.board.agent(agent())
    r.start(align)
    r.run(t + 3000)
    return r, img, [w for _, w in got], sched


def main(log):
    log.group('advanced/can.s', title='CAN node')
    for name, v, want in anchors()[2:]:
        log.check('reference ' + name, v == want, hex(v))
    frames = [(0x555, [0xAA]), (0x123, [1, 2, 3, 4, 5, 6, 7, 8]), (0x000, [0, 0, 0, 0, 0, 0, 0, 0]), (0x7EF, [0xFF] * 8), (0x2A5, [])]
    r, img, w, sched = rx_case(frames)
    log.line('INFO  can is %d words in all: sampler %d, decoder %d, transmitter %d' % (img.size, img.labels['dec'] - img.labels['samp'], img.labels['tx'] - img.labels['dec'], img.size - img.labels['tx'] + 0))
    rep = parse_reports(w)
    want = [(i, len(d), list(d), 0) for i, d in frames]
    # thread 1 is the sampler. It times no output, and a late sample shows in the frames, so its LATE isn't counted
    log.check('RX 5 frames incl. all-zero and all-one stuffing, reported and CRC clean', rep == want and not r.health(ignore_late=(1,)), (rep[:2], r.health(ignore_late=(1,))))
    fr = C.decode(r.wave('ui', RXP), 100, lo=2000)
    log.check('ACK on the bus for every frame, pad decoder agrees', [f['ack'] for f in fr] == [True] * 5 and all(f['crc_ok'] and f['eof_ok'] for f in fr), [(f['id'], f['ack'], f['errs']) for f in fr][:2])
    # ACK slot placement, from the pads: chip's pull must cover the slot and only the slot
    txw = r.wave('uo', TXP)
    pulls = [(a, b) for (a, va), (b, vb) in zip(txw.edges(), txw.edges()[1:]) if va == 0 and vb == 1]
    off = []
    for f, (a, b) in zip(fr, pulls[-5:]):
        pass
    good = 0
    for al in range(8):
        r, img, w, sched = rx_case(frames[:3], align=al % 4, t_start=4000 + al)
        good += parse_reports(w) == want[:3] and not r.health(ignore_late=(1,))
    log.check('RX 8 start phases', good == 8, '%d of 8' % good)
    for e in (0.005, -0.005, 0.01, -0.01, 0.02, -0.02):
        r, img, w, sched = rx_case(frames, err=e)
        fr = C.decode(r.wave('ui', RXP), 100 * (1 + e), lo=2000)
        ok = parse_reports(w) == want and not r.health(ignore_late=(1,))
        log.check('RX sender %+.1f%%' % (e * 100), ok, ([f['ack'] for f in fr]))
    for d in ((2, 2), (5, 5), (11, 11)):
        r, img, w, sched = rx_case(frames, d1=d[0], d2=d[1])
        log.check('RX transceiver loop %d ns' % ((d[0] + d[1]) * 20), parse_reports(w) == want and not r.health(ignore_late=(1,)))
    r, img, w, sched = rx_case(frames[:3], flip=1)
    fr = C.decode(r.wave('ui', RXP), 100, lo=2000)
    rep = parse_reports(w)
    log.check('RX corrupted CRC: reported 0x101, no ACK, next frame fine', [x[3] for x in rep] == [0, 1, 0] and [f['ack'] for f in fr] == [True, False, True], ([x[3] for x in rep], [f['ack'] for f in fr]))
    # six equal bits inside a frame
    ident, data = 0x123, [1, 2]
    b, c = C.with_crc(ident, data)
    st = C.stuff(b)
    bad = st[:20] + [1 - st[20]] * 1 + st[21:]
    run = 0
    for k in range(5, len(st)):
        if st[k - 5:k] in ([0] * 5, [1] * 5):
            bad = st[:k] + [st[k - 1]] + st[k + 1:] + [1, 1, 1] + [1] * 7
            break
    s_ = Sender([(4000, bad)], 100.0)
    r, img, bus = rig([s_])
    got = []

    def ag():
        while True:
            yield from r.host.take(4, got, tries=1)
    r.board.agent(ag())
    r.start()
    r.run(4000 + len(bad) * 100 + 3000)
    log.check('RX six equal bits: stuff error reported, no ACK', 0x102 in [x for _, x in got], [hex(x) for _, x in got][-3:])

    log.info('transmit')
    cases = [(0x555, [0xAA]), (0x123, [1, 2, 3, 4, 5, 6, 7, 8]), (0x000, [0] * 8), (0x7EF, [0xFF] * 8), (0x2A5, []), (0x0F0, [0xF0, 0x0F, 0xFF, 0x00])]
    for ident, data in cases:
        r, img, st, got = tx_case(ident, data)
        fr = C.decode(r.wave('ui', RXP), 100, lo=2000)
        f = fr[0] if fr else {}
        log.check('TX id %#x, %d bytes: bus decode, CRC, ACK, status' % (ident, len(data)),
                  len(fr) == 1 and f['id'] == ident and f['data'] == list(data) and f['crc_ok'] and f['ack'] and f['eof_ok'] and st == [0] and not r.health(ignore_late=(1,)),
                  (f.get('id'), f.get('crc_ok'), f.get('ack'), st, f.get('errs'), r.health(ignore_late=(1,))))
    r, img, st, got = tx_case(0x123, [1, 2], acked=False)
    fr = C.decode(r.wave('ui', RXP), 100, lo=2000)
    log.check('TX nobody ACKs: status 1, frame still valid on the bus', st == [1] and fr and fr[0]['crc_ok'] and not fr[0]['ack'], (st, [f['ack'] for f in fr]))
    good = 0
    for al in range(4):
        for d in ((2, 2), (5, 5), (11, 11)):
            r, img, st, got = tx_case(0x2F1, [0x5A, 0xFF, 0x00], align=al, d1=d[0], d2=d[1])
            fr = C.decode(r.wave('ui', RXP), 100, lo=2000)
            good += len(fr) == 1 and fr[0]['crc_ok'] and fr[0]['ack'] and st == [0] and not r.health(ignore_late=(1,))
    log.check('TX 4 alignments x 3 loop delays', good == 12, '%d of 12' % good)

    log.info('arbitration')
    # rival with a lower ID starts on our SOF: chip must stop at the first differing bit, then take the rival's frame
    mine, theirs = (0x246, [1, 2]), (0x244, [9])
    rb = C.frame_bits(*theirs)
    lenb = len(rb) - 10 + 1
    riv = Rival(rb, 100.0)
    ack = Acker([lenb], 100.0)
    r, img, st, got = tx_case(*mine, nodes=[riv, ack], run=30000)
    fr = C.decode(r.wave('ui', RXP), 100, lo=2000)
    rep = parse_reports(got)
    txw = r.wave('uo', TXP)
    log.check('rival wins arbitration: we stop, status 2, its frame intact on the bus', st[:1] == [2] and len(fr) >= 1 and fr[0]['id'] == 0x244 and fr[0]['crc_ok'] and fr[0]['data'] == [9], (st, [(f['id'], f['crc_ok']) for f in fr]))
    log.check('we also received the winner and could ACK it', rep and rep[0][0] == 0x244 and rep[0][2] == [9], rep[:1])
    # chip's ID is the lower one: rival must drop out and our frame goes through
    mine2, theirs2 = (0x244, [7, 8]), (0x246, [9])
    rb2 = C.frame_bits(*theirs2)
    riv2 = Rival(rb2, 100.0)
    b2 = C.frame_bits(*mine2)
    ack2 = Acker([len(b2) - 10 + 1], 100.0)
    r, img, st, got = tx_case(*mine2, nodes=[riv2, ack2], run=30000)
    fr = C.decode(r.wave('ui', RXP), 100, lo=2000)
    log.check('we win arbitration: rival drops out, our frame clean', st == [0] and len(fr) == 1 and fr[0]['id'] == 0x244 and fr[0]['crc_ok'] and fr[0]['ack'] and not riv2.alive, (st, [(f['id'], f['crc_ok'], f['ack']) for f in fr], riv2.alive))
    # rival starts first, in our last idle bit: the pre-armed SOF must follow it in the same bit
    rb3 = C.frame_bits(*theirs)
    for start in (4215, 4250, 4285):
        riv3 = Rival(rb3, 100.0, start=start)
        ack3 = Acker([len(rb3) - 10 + 1], 100.0)
        r, img, st, got = tx_case(*mine, nodes=[riv3, ack3], run=30000)
        fr = C.decode(r.wave('ui', RXP), 100, lo=2000)
        sof = [t for t, v in r.wave('uo', TXP).edges(3000) if v == 0][:1]
        rx_fall = [t for t, v in r.wave('ui', RXP).edges(3000) if v == 0][:1]
        ok = st[:1] == [2] and fr and fr[0]['id'] == 0x244 and fr[0]['crc_ok'] and sof and rx_fall and sof[0] - rx_fall[0] <= 8
        log.check('rival starts first at %d: our SOF lands in the same bit, frame intact' % start, ok, (st, sof, rx_fall, [(f['id'], f['crc_ok']) for f in fr][:2]))

    log.info('each direction alone on the 128-word fallback')
    fx = [(0x123, [1, 2, 3, 4, 5, 6, 7, 8]), (0x7EF, [0xFF] * 8)]
    sc, t = [], 4000
    for i, d in fx:
        bits = C.frame_bits(i, d)
        sc.append((t, bits))
        t += len(bits) * 100 + 3000
    r, img, bus = rig([Sender(sc, 100.0)], depth=128, defines={'WITH_TX': 0}, starts={0: 'dec', 1: 'samp'})
    got = []

    def ag1():
        while True:
            yield from r.host.take(4, got, tries=1)
    r.board.agent(ag1())
    r.start()
    r.run(t + 3000)
    fr = C.decode(r.wave('ui', RXP), 100, lo=2000)
    log.check('RX only, %d words on 128: frames reported, ACK driven' % img.size, parse_reports([w for _, w in got]) == [(i, len(d), list(d), 0) for i, d in fx] and [f['ack'] for f in fr] == [True, True], len(fr))
    i, d = 0x123, [1, 2, 3, 4, 5, 6, 7, 8]
    bits = C.frame_bits(i, d)
    r, img, bus = rig([Acker([len(bits) - 10 + 1], 100.0)], depth=128, defines={'WITH_RX': 0, 'PKT': 100}, starts={2: 'tx'})
    words, nb = pack(i, d)
    for k, w in enumerate(words):
        r.chip.host_wrmem(100 + k, w)
    st = []

    def ag4():
        yield 3000
        yield from r.host.post(2, nb + 1)
        yield from r.host.take(5, st)
    r.board.agent(ag4())
    r.start()
    r.run(30000)
    fr = C.decode(r.wave('ui', RXP), 100, lo=2000)
    log.check('TX only, %d words on 128: frame clean on the bus, ACK seen' % img.size, len(fr) == 1 and fr[0]['crc_ok'] and fr[0]['ack'] and [w for _, w in st] == [0] and not r.health(), (st, r.health()))
