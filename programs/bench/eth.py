"""Bench for advanced/eth_arp.s: a 10BASE-T node at 50 MHz, end to end, against a partner on the other side of the
twisted pairs.

The partner is a line, not a program: its pair is built from eth_check_test's generators, which follow IEEE 802.3
and the RFCs and share no code with the firmware: frames (3.2.8 FCS bit by bit, 4.2.5 preamble, 7.3.1.1 Manchester
with 14.2.1.1's polarity, 14.3.1.2.1 TP_IDL), link test pulses and FLP bursts (28.2.1.1.2), ARP (RFC 826) and ICMP
echo (RFC 791, 792). This bench adds edge jitter, a partner clock off by the 7.3.2 tolerance, short preambles,
TP_IDL of 2.5 and 4.5 bit times, short gaps (96 bit times, and 47 as 4.4.2 note 1 allows at a receiver), and frames
timed against the chip's own replies. frontend.Rs485Eth turns the pair into the chip's RO input (receiver delay
and skew, FN6307 corners), and the chip's DI and DE pads back into its transmit pair. eth_check.check judges the
whole record: the transmit pair against clause 14, every reply field by field, gaps, collisions, link pulses.

What the firmware must do with each partner frame comes from the frame alone (disposition(), 802.3 4.2.4): kept
(DA ours or broadcast, FCS right, 64 octets or more), bad (runt or FCS error) or for another station. The firmware's
counters must match, so every frame is accounted for, and eth_check decides which frames need a reply.

The chip sees RO as frontend's level at each clock edge plus a phase of 0 to 20 ns, so the asynchronous sampling of
section 10, item 2 is swept. With half (the default) the chip also gets RO's level at each falling edge, half a cycle
earlier, and DI and DE are recorded in both halves of every cycle (section 3.1); duty moves every falling edge that
many ns late, a clock duty error (section 7.3). The host posts the MAC and IP, five words on mailbox word 4. Thread placements and
start alignments vary across runs.
"""
import bisect
import math
import random

import eth_check
import eth_check_test as G
import frontend
from report import Chip, assemble, src

PROGRAMS = ('advanced/eth_arp.s',)
PROG = 'advanced/eth_arp.s'
WORDS = 402                     # section 11.2's count for the program
MAC, IP = G.MAC, G.IP
PEER_MAC, PEER_IP = G.PEER_MAC, G.PEER_IP
OTHER_MAC = G.OTHER_MAC
OTHER_IP = bytes([192, 168, 0, 77])
BCAST = eth_check.BROADCAST
BT = 100.0                      # ns, 802.3 7.3.2
CYCLE = 20.0                    # ns at 50 MHz
MCFG = 4                        # the program's mailbox word for MAC and IP
DI, DE, RO = 4, 5, 4            # uo4, uo5, ui4
MEM = dict(HANDED=5, RELEASED=6, NKEEP=7, NBAD=8, NOTHER=9, NBUSY=10)
CODE_FROM = 12                  # words from here on are code and constants that must never change
PPM = 1e-4                      # 7.3.2: 10 Mb/s +/- 0.01%
HALF = True                     # run()'s default: drive and record both halves of every cycle (section 3.1)


def wire_words(octets):
    """Octets as the firmware's words: two per word, the first in the low byte."""
    return [octets[i] | (octets[i + 1] << 8) for i in range(0, len(octets), 2)]


def octets_of(bits):
    return bytes(sum(bits[8 * q + b] << b for b in range(8)) for q in range(len(bits) // 8))


def disposition(bits):
    """What 802.3 4.2.4 asks a receiving station to do with a frame, from its bits alone. 4.2.9's ReceiveLinkMgmt
    first truncates the frame to whole octets (excessBits := frameSize mod 8; 4.2.4.2.1 b), then: 'bad' (under
    minFrameSize of 64 octets, or an FCS error, an alignmentError if bits were truncated: 3.4, 4.2.4.1.2,
    4.2.4.2.2), 'other' (DA neither the station's address nor broadcast: 4.2.4.1.1), else 'keep'."""
    octs = octets_of(bits[:len(bits) - len(bits) % 8])
    if len(octs) < 64:
        return 'bad'
    f = eth_check.parse(octs)
    if not f.get('fcs_ok'):
        return 'bad'
    return 'keep' if f['dst'] in (MAC, BCAST) else 'other'


# ---------------------------------------------------------------------------------------------------------------
# the partner's line

class Line:
    """The partner's pair: changes (ns, +1/-1/0), and each frame sent with its disposition."""

    def __init__(self, seed=1, bt=BT, jitter=None, pre=56, tp_hi=3.0):
        self.rng = random.Random(seed)
        self.bt, self.jitter, self.pre, self.tp_hi = bt, jitter, pre, tp_hi
        self.changes = []
        self.sent = []                  # (t_start, t_end, disposition)
        self.t = 0.0

    def frame(self, bits, t, pre=None, jitter=None, tp_hi=None, tp_lo=None):
        """One frame from t (ns): pre preamble bits, the SFD, the bits, TP_IDL of tp_hi bit times (optionally a
        negative tail of tp_lo). jitter: None, ('uniform', pre_ns, frame_ns), random per transition as 802.3
        7.5.2.2 builds its test signal, ('alt', ns), bit-boundary transitions early and mid-bit ones late (late
        and early for a negative ns), ('step', ns), every transition from a random bit of the frame on (preamble
        to the last bit) moved ns later, TP_IDL with it, ('pwd', ns), every transition to HI ns late and every one
        to LO ns early (the reverse for a negative ns), or ('isi', ns, r), a cable's data-dependent jitter: every
        transition that follows a whole bit with none ns late and every other one ns early, which is how a
        low-pass link moves them (a negative ns is the reverse sign, a transmitter equalized for a long cable on a
        short one, B.4.1), plus random jitter of +/-r. Returns the end of the last bit cell."""
        pre = self.pre if pre is None else pre
        jit = self.jitter if jitter is None else jitter
        allbits = G.preamble(pre) + bits
        jd, s = {}, 0.0
        if jit and jit[0] == 'step':
            j, s = self.rng.randrange(len(allbits)), jit[1]
            jd = {h: (s if h >= 2 * j else 0.0) for h in range(2 * len(allbits))}
        elif jit and jit[0] in ('pwd', 'isi'):
            halves = []
            for b in allbits:
                halves += [-1 if b else 1, 1 if b else -1]       # manchester()'s levels
            last, lh = 0, None
            for h, lv in enumerate(halves):
                if lv == last:
                    continue
                if jit[0] == 'pwd':
                    jd[h] = jit[1] if lv == 1 else -jit[1]
                else:
                    whole = lh is not None and h - lh >= 2      # a whole bit with no transition before it
                    jd[h] = (jit[1] if whole else -jit[1]) + (self.rng.uniform(-jit[2], jit[2]) if jit[2] else 0.0)
                last, lh = lv, h
        elif jit:
            for h in range(2 * len(allbits)):
                if jit[0] == 'uniform':
                    a = jit[1] if h < 2 * (pre + 8) else jit[2]
                    jd[h] = self.rng.uniform(-a, a)
                else:
                    jd[h] = jit[1] if h % 2 else -jit[1]
        ch, end = G.manchester(allbits, t, bt=self.bt, tp_hi=self.tp_hi if tp_hi is None else tp_hi, tp_lo=tp_lo,
                               jitter=jd)
        if s:
            ch = [(tt + s, lv) if tt >= end - 1e-9 else (tt, lv) for tt, lv in ch]
            end += s
        self.changes += ch
        self.sent.append((t, end, disposition(bits)))
        self.t = end
        return end

    def pulse(self, t, width=100.0):
        self.changes += G.link_pulse(t, width)

    def flp(self, t, word=0x01E1):
        self.changes += G.flp(t, word)

    def line(self):
        return G.merge(self.changes)

    def count(self):
        out = dict(keep=0, bad=0, other=0)
        for _, _, d in self.sent:
            out[d] += 1
        return out


def arp_req(tpa=IP, dst=BCAST, sha=PEER_MAC, spa=PEER_IP, op=1, hrd=1, pro=0x0800, **kw):
    return G.frame(dst, PEER_MAC, 0x0806, G.arp(op, sha, spa, bytes(6), tpa, hrd=hrd, pro=pro), **kw)


def echo_ip(n=56, ident=0x1234, seq=1, ipdst=IP, ttl=64, bad_icmp=False):
    data = bytes((i * 7 + seq) & 255 for i in range(n))
    return G.echo(8, PEER_IP, ipdst, ident, seq, data, ttl=ttl, bad_icmp=bad_icmp)


def reheader(ip):
    """An IPv4 datagram with its header checksum computed again (RFC 791 3.1)."""
    ihl = (ip[0] & 15) * 4
    h = bytearray(ip[:ihl])
    h[10] = h[11] = 0
    c = G.csum(bytes(h))
    h[10], h[11] = c >> 8, c & 255
    return bytes(h) + bytes(ip[ihl:])


def ping(n=56, dst=MAC, mutate=None, **kw):
    """An echo request frame from the partner. mutate(datagram) returns a changed IP datagram."""
    ip = echo_ip(n, **kw)
    if mutate:
        ip = mutate(ip)
    return G.frame(dst, PEER_MAC, 0x0800, ip)


def bad_header(ip):
    return ip[:11] + bytes([ip[11] ^ 1]) + ip[12:]


def fragment(ip):
    return reheader(ip[:6] + bytes([ip[6] | 0x20]) + ip[7:])


# ---------------------------------------------------------------------------------------------------------------
# scenarios

def exchange(seed=1, gap=320e3, **kw):
    """Every case a node meets, in about 19 ms: requests to answer, frames to keep but not answer, frames to drop,
    an FLP burst and a partner link pulse, two pairs of requests 96 and 47 bit times apart, full-size frames, and
    TP_IDL at both ends of 14.3.1.2.1's range after a final 0 and a final 1. Returns (line, end of the last frame
    plus a gap)."""
    L = Line(seed, **kw)
    rng = random.Random(seed)
    t = [0.4e6]

    def at(bits, after=gap, **k):
        end = L.frame(bits, t[0], **k)
        t[0] = end + after
        return end
    at(arp_req())                                           # broadcast ARP request: reply
    at(arp_req(dst=MAC, spa=bytes([192, 168, 0, 3])))       # unicast ARP request: reply
    at(ping(56))                                            # Linux's default echo: reply
    at(ping(32, seq=2))                                     # Windows' default: reply
    at(ping(57, seq=3))                                     # the largest that fits, odd length: reply
    at(ping(1, seq=4))                                      # padded to 60 octets, odd: reply
    at(ping(8, seq=5, ttl=0))                               # TTL 0 must not be dropped (RFC 1122 3.2.1.7): reply
    at(arp_req(tpa=OTHER_IP))                               # for another IP (RFC 826): kept, no reply
    at(arp_req(bad_fcs=True))                               # FCS error (3.4): dropped
    at(arp_req(pad=False))                                  # 46 octets, a runt (4.2.4.2.2): dropped
    at(arp_req(dst=OTHER_MAC))                              # for another station (4.2.4.1.1): dropped
    for near in ('020100000001', '020000010001', '020000000003'):
        at(arp_req(dst=bytes.fromhex(near)))                # each one word from ours, all three words compared
    for n in (16, 17):                                      # 62 and 63 octets with a good FCS: runts (4.2.4.2.2)
        at(G.frame(BCAST, PEER_MAC, 0x0806, G.arp(1, PEER_MAC, PEER_IP, bytes(6), IP) + bytes(n), pad=False))
    at(arp_req(op=2))                                       # an ARP reply: kept, no reply
    at(arp_req(hrd=6))                                      # another hardware type: kept, no reply
    at(arp_req(pro=0x86DD))                                 # another protocol: kept, no reply
    at(ping(56, seq=6, bad_icmp=True))                      # ICMP checksum wrong: kept, no reply
    at(ping(56, seq=7, mutate=bad_header))                  # header checksum wrong: kept, no reply
    at(ping(56, seq=8, ipdst=OTHER_IP))                     # for another IP: kept, no reply
    at(ping(56, seq=10, dst=BCAST))                         # to the link broadcast: kept, no reply (RFC 1122 3.3.6)
    at(G.frame(MAC, PEER_MAC, 0x0800, reheader(bytes([0x45, 0, 0, 36, 0, 1, 0, 0, 64, 17, 0, 0]) + PEER_IP + IP +
                                                bytes(16))))                 # UDP: kept, no reply
    at(G.frame(bytes.fromhex('01005e000001'), PEER_MAC, 0x0800, bytes(46)))  # multicast: dropped
    at(G.frame(BCAST, PEER_MAC, 0x86DD, bytes(rng.randrange(256) for _ in range(60))))
    L.flp(t[0])                                             # an auto-negotiating partner's burst (28.2.1.1.2)
    t[0] += 2.6e6
    at(G.frame(OTHER_MAC, PEER_MAC, 0x88B5, bytes(rng.randrange(256) for _ in range(1500))), after=9.6e3)
    at(arp_req(spa=bytes([192, 168, 0, 8])))                # 96 BT after a 1518-octet frame: reply
    at(G.frame(BCAST, PEER_MAC, 0x88B5, bytes(rng.randrange(256) for _ in range(1500))))   # stored in part
    at(arp_req(spa=bytes([192, 168, 0, 9])), after=9.6e3)   # two requests 96 BT apart: both answered
    at(arp_req(spa=bytes([192, 168, 0, 10])))
    at(arp_req(spa=bytes([192, 168, 0, 11])), after=4.7e3)  # 47 BT apart (4.4.2 note 1): both answered
    at(arp_req(spa=bytes([192, 168, 0, 12])))
    L.pulse(t[0])
    t[0] += 150e3
    for hi, lo in ((2.5, None), (4.5, None), (3.0, 1.0)):
        for want in (0, 1):                                 # TP_IDL after a final 0 and after a final 1
            for s in range(20, 120):
                bits = arp_req(spa=bytes([192, 168, 1, s]))
                if bits[-1] == want:
                    break
            at(bits, tp_hi=hi, tp_lo=lo)
    return L, t[0]


def long_run(seed=1, **kw):
    """The exchange, then 34 ms of an idle line, for the link pulse period after frames and between pulses."""
    L, t = exchange(seed, **kw)
    return L, t + 34e6


def frames_only(n, seed, jitter=None, phase_seed=0, octets=46, src=None, **kw):
    """n broadcast frames of 64 octets (data of 46) with random data, 96 bit times apart plus 3 us: R keeps each, T
    drops each. src: the source address (PEER_MAC, locally administered, by default)."""
    L = Line(seed, jitter=jitter, **kw)
    t = 0.3e6
    for _ in range(n):
        t = L.frame(G.frame(BCAST, src or PEER_MAC, 0x88B5, bytes(L.rng.randrange(256) for _ in range(octets))),
                    t) + 12.6e3
    return L, t + 50e3


UAA_MAC = bytes.fromhex('001b213a5c7e')          # a universally administered source (3.2.3 c): a real ARP's shape


PH8 = (1.9, 4.3, 6.1, 8.7, 11.3, 13.9, 16.6, 19.1)     # sampling phases (ns), none a multiple of 2.5
CORNERS = ('typ', 'max', 'min')


def rx_row(jit, seed, n=20, corners=CORNERS, phases=PH8, duty=0.0, bt=BT, octets=46, skew=1, src=None):
    """Frames lost, sent less kept (NKEEP), for each FN6307 corner and sampling phase: runs of n frames, each run
    with its own seed. skew -1 swaps the receiver's skew (the data sheet bounds only its size). Returns ({corner:
    [lost per phase]}, health problems)."""
    lost, health = {}, set()
    for ci, corner in enumerate(corners):
        lost[corner] = []
        over = None if skew > 0 else dict(skew_rx=-frontend.Rs485Eth.CORNERS[corner]['skew_rx'])
        for k, ph in enumerate(phases):
            L, t = frames_only(n, seed=seed + 37 * k + ci, jitter=jit, bt=bt, octets=octets, src=src)
            r = run(L, t, corner=corner, phase=ph, check=False, duty=duty, fe_over=over)
            lost[corner].append(n - r.count['NKEEP'])
            health |= set(r.health)
    return lost, sorted(health)


def total(lost):
    return sum(sum(v) for v in lost.values())


def excess_line(seed=21, extra=range(1, 8)):
    """802.3 frames that end with 1 to 7 bits after their last whole octet: for each count an ARP request and an echo
    request with good FCSs, which 4.2.9 truncates and keeps, and an ARP request with a bad FCS, which it reports as
    an alignmentError and drops. Returns (line, end)."""
    L = Line(seed)
    rng = random.Random(seed)
    t = 0.4e6
    for k in extra:
        for bits in (arp_req(spa=bytes([192, 168, 2, k])), ping(24, seq=100 + k),
                     arp_req(spa=bytes([192, 168, 3, k]), bad_fcs=True)):
            t = L.frame(bits + [rng.getrandbits(1) for _ in range(k)], t) + 150e3
    return L, t


# ---------------------------------------------------------------------------------------------------------------
# a run

class Result:
    pass


def ro_schedule(ro, phase):
    """RO as (cycle, level): the chip's input in cycle c is RO's level at c * 20 ns + phase (frontend.Sampler)."""
    return [(int(math.ceil((t - phase) / CYCLE - 1e-9)), lv) for t, lv in ro]


def run(line, t_end, corner='typ', phase=0.0, align=0, threads=(0, 1), seed=1, text=None, chip_cls=None,
        fe_over=None, check=True, half=HALF, duty=0.0):
    """Runs the program against the line to t_end ns. Returns a Result with the checker's report."""
    img = assemble(text if text is not None else src(PROG), depth=512)
    fe = frontend.Rs485Eth(corner, **(fe_over or {}))
    rx = line.line()
    ro = fe.ro(rx)
    sched = ro_schedule(ro, phase)
    sched_f = ro_schedule(ro, phase - CYCLE / 2 + duty) if half else []
    j, nf, lvf = 0, len(sched_f), 1
    cycles = int(t_end / CYCLE)
    chip = (chip_cls or Chip)(512, seed)
    for a, w in img.words.items():
        chip.host_wrmem(a, w)
    tr, tt = threads
    chip.host_setpc(tr, img.labels['rx'])
    chip.host_setpc(tt, img.labels['tx'])
    cfg = wire_words(MAC) + wire_words(IP)
    rec_t, rec_v = [], []
    last = None
    i, n, lv = 0, len(sched), 1
    started = False
    for c in range(cycles):
        if not started and (c - 1) % 4 == align:
            chip.host_run(0, (1 << tr) | (1 << tt))
            started = True
        if cfg and started and not chip.FULL[MCFG]:
            chip.host_post(MCFG, cfg.pop(0))
        while i < n and sched[i][0] <= c:
            lv = sched[i][1]
            i += 1
        ui = 0x01 | (lv << RO)
        uo = chip.pads(ui)[0]
        if half:
            while j < nf and sched_f[j][0] <= c:
                lvf = sched_f[j][1]
                j += 1
            uo2 = chip.pads2(ui)[0]
            if uo != last:
                rec_t.append(2 * c)
                rec_v.append(uo)
                last = uo
            if uo2 != last:
                rec_t.append(2 * c + 1)
                rec_v.append(uo2)
                last = uo2
            chip.cycle(ui, 0, 0x01 | (lvf << RO))
            continue
        if uo != last:
            rec_t.append(c)
            rec_v.append(uo)
            last = uo
        chip.cycle(ui, 0)
    res = Result()
    res.di = frontend.pad(rec_t, rec_v, 'uo%d' % DI, half=half, duty_ns=duty)
    res.de = frontend.pad(rec_t, rec_v, 'uo%d' % DE, half=half, duty_ns=duty)
    res.fe, res.rx, res.line, res.chip, res.img, res.t_end = fe, rx, line, chip, img, cycles * CYCLE
    res.tx = tx_pair(fe, res.di, res.de)
    res.rep = eth_check.check(res.tx, rx, mac=MAC, ip=IP, t_start=0.0, t_end=res.t_end) if check else None
    res.health = health(chip, img, threads)
    res.count = {k: chip.mem[a] for k, a in MEM.items()}
    return res


def tx_pair(fe, di, de, quiet=2000.0):
    """frontend.Rs485Eth.tx_line, one burst at a time: its cost grows with the square of the changes it is given,
    and a burst's line depends only on the DI and DE changes near it. A cut falls halfway through every stretch of
    DE low lasting `quiet` ns or more, far longer than any transceiver delay (FN6307: at most 60 ns), so cutting
    there changes nothing; the bench compares one call with the pieces."""
    cuts, lo_since, prev = [], None, 0
    for t, lv in de:
        if lv and not prev and lo_since is not None and t - lo_since >= quiet:
            cuts.append((lo_since + t) / 2)
        if not lv and prev:
            lo_since = t
        prev = lv
    td, te = [x[0] for x in di], [x[0] for x in de]
    out, t0, i0, j0 = [], None, 0, 0
    for cut in cuts + [float('inf')]:
        i1, j1 = bisect.bisect_left(td, cut), bisect.bisect_left(te, cut)
        di0 = di[i0 - 1][1] if i0 else 0
        de0 = de[j0 - 1][1] if j0 else 0
        piece = fe.tx_line(di[i0:i1], de[j0:j1], di0=di0, de0=de0)
        if t0 is not None and piece and piece[0][0] == 0.0:
            piece = piece[1:]               # tx_line shows a pair driven from the start at time 0
        out += piece
        t0, i0, j0 = cut, i1, j1
    return out


def health(chip, img, threads):
    """LATE, ILL, a thread stopped, or a code or constant word overwritten."""
    out = []
    for tid in threads:
        t = chip.th[tid]
        if t.LATE:
            out.append('thread %d LATE' % tid)
        if t.ILL:
            out.append('thread %d ILL at %d' % (tid, t.PC))
        if not t.RUN:
            out.append('thread %d stopped' % tid)
    bad = [a for a, w in img.words.items() if a >= CODE_FROM and chip.mem[a] != w]
    if bad:
        out.append('memory overwritten at %s' % bad[:5])
    return out


def accounted(res):
    """The firmware's counters against the frames' dispositions: '' if they agree."""
    want = res.line.count()
    c = res.count
    got = dict(keep=c['NKEEP'], bad=c['NBAD'], other=c['NOTHER'])
    if c['NBUSY'] or got != want or c['HANDED'] != c['RELEASED']:
        return 'counters %s, busy %d, handed %d, released %d; the frames: %s' % (
            got, c['NBUSY'], c['HANDED'], c['RELEASED'], want)
    return ''


def verdict(res, allow=()):
    """Every failing finding, the health and the counters, as one string ('' if all is well). allow: checks whose
    failures this scenario expects."""
    bad = [f for f in res.rep.failed() if f[1] not in allow]
    out = ['%s @%.1f us: %s' % (c, t / 1e3, x) for t, c, x in bad[:3]]
    out += res.health
    a = accounted(res)
    if a:
        out.append(a)
    return '; '.join(out)


def replies(rep):
    """(ARP replies, echo replies) the chip sent."""
    arp = sum(1 for f in rep.tx_frames if f.octets is not None and 'arp' in f.fields)
    icmp = sum(1 for f in rep.tx_frames if f.octets is not None and 'ip' in f.fields)
    return arp, icmp


def wanted(rep):
    """(ARP requests, echo requests) that need a reply, by eth_check's own rule. An echo request to the link
    broadcast address only allows one (RFC 1122 3.2.2.6), and RFC 1122 3.3.6 says a host should discard it."""
    n = [0, 0]
    for rf in rep.rx_frames:
        kind, _ = eth_check._wants_reply(rf, MAC, IP, eth_check.DEFAULTS)
        if kind == 'arp':
            n[0] += 1
        elif kind == 'icmp':
            n[1] += 1
    return tuple(n)


# ---------------------------------------------------------------------------------------------------------------
# margins: what the record measures, against each limit

def tx_margins(rep):
    """Measured transmit values: pulse widths, TP_IDL lengths after a final 0 and a final 1, edge deviation and
    bit rate error, preamble bits, gaps, link pulse periods."""
    m = {}
    m['pulse'] = [p.width for p in rep.tx_pulses]
    idl = {0: [], 1: []}
    for f in rep.tx_frames:
        if f.t_idl is None:
            continue
        k = [q for q, (t, v) in enumerate(f.tail) if t == f.t_idl]
        w = ((f.tail[k[0] + 1][0] if k and k[0] + 1 < len(f.tail) else f.stop) - f.t_idl) / BT
        idl[f.bits[-1]].append(w)
    m['tp_idl'] = idl
    m['jitter'] = [f.jitter for f in rep.tx_frames if f.jitter is not None]
    m['ppm'] = [(f.period / BT - 1) * 1e6 for f in rep.tx_frames if f.period]
    m['pre'] = [f.sfd_at - 7 + f.lead for f in rep.tx_frames if f.sfd_at is not None]
    gaps = []
    ends = sorted([(f.end if f.end is not None else f.stop) for f in rep.tx_frames + rep.rx_frames])
    for f in rep.tx_frames:
        before = [e for e in ends if e <= f.start]
        if before:
            gaps.append((f.start - max(before)) / BT)
    m['ipg'] = gaps
    ev = sorted([(p.start, 'p') for p in rep.tx_pulses] + [(f.t_idl, 'f') for f in rep.tx_frames if f.t_idl] +
                [(f.start, 's') for f in rep.tx_frames])
    per, prev = [], None
    for t, k in ev:
        if k == 's':
            prev = None
            continue
        if k == 'p' and prev is not None:
            per.append((t - prev) / 1e6)
        prev = t
    m['period'] = per
    return m


def span(x, fmt='%.1f'):
    return ('%s to %s' % (fmt % min(x), fmt % max(x))) if x else 'none'


# ---------------------------------------------------------------------------------------------------------------
# broken copies: each must fail somewhere

TEETH = [
    ('the FCS appended uncomplemented (CODING CXOR 0)', 'li    R0, 0x02FF', 'li    R0, 0x02BF'),
    ('the next link pulse 19 TB wraps after the last, 24.9 ms', 'ldi   R3, 12\n        ret',
     'ldi   R3, 19\n        ret'),
    ('the ARP reply not padded: 42 octets', 'ldi   R5, 60\n        goto  tloop\n; IPv4',
     'ldi   R5, 42\n        goto  tloop\n; IPv4'),
    ('an ARP request for another IP answered (tpa unchecked)', 'call  cmpn           ; tpa\n        jf    tdrop',
     'call  cmpn           ; tpa'),
    ('a reply 20 bit times into the gap (deference of 100 cycles)', 'li    R2, 480', 'li    R2, 100'),
    ('TP_IDL of 11 cycles', 'li    R0, 0x800F', 'li    R0, 0x800B'),
    ('DI resting low (IDLEA 0), so a pulse starts with the old DI', 'li    R0, 0x81A1', 'li    R0, 0x8181'),
    ('frames with an FCS error kept', 'srd   R4, end', 'ldi   R4, 1'),
    ('runts kept', 'ldi   R4, BUFW - 30', 'ldi   R4, BUFW'),
    ('frames for another station kept', 'jf    rother', 'jf    rkeep'),
    ('the ICMP checksum updated by 9, not 8', 'li    R4, 0xFFF7', 'li    R4, 0xFFF6'),
    ('tpa left as received (4 words moved, not 5)', 'ldi   R5, 5\n        call  xmv', 'ldi   R5, 4\n        call  xmv'),
    ('no deference: sending while the partner\'s frame is on the line',
     'until RO, 0, R2      ; T 1: the line was active within 96 bit times\n        jt    tloop', 'until RO, 0, R2'),
    ('the receiver\'s polarity wrong (RINV 0)', 'li    R0, 0x002B | (RON << 8)', 'li    R0, 0x0023 | (RON << 8)'),
    ('edges timed to the cycle: RKIND 1, not 3', 'li    R0, 0x002B | (RON << 8)', 'li    R0, 0x0029 | (RON << 8)'),
    ('half bits of 2 and 3 cycles: THALF off', 'li    R0, 0x81A1', 'li    R0, 0x01A1'),
    ('the byte end not asked for: REOP 0 in the receive CODING', 'li    R0, 0x80FF', 'li    R0, 0x00FF'),
]


def broken(old, new):
    text = src(PROG)
    assert text.count(old) == 1, old
    return text.replace(old, new)


# ---------------------------------------------------------------------------------------------------------------

def main(log):
    log.group(PROG, title='10BASE-T node at 50 MHz: ARP and echo replies, link pulses, against a partner line')
    img = assemble(src(PROG), depth=512)
    top = max(img.words)
    log.check('assembles to %d words, the count in section 11.2; code, constants and two buffers fit 512' % WORDS,
              img.size == WORDS and top < 512, '%d words placed, last address %d' % (img.size, top))
    rng = random.Random(3)
    ok = True
    for _ in range(40):
        di, de, t, a, b = [], [], 0.0, 0, 0
        for _ in range(rng.randrange(5, 40)):
            t += rng.choice((20.0, 40.0, 60.0, 3000.0, 50000.0))
            if rng.random() < 0.5:
                a ^= 1
                di.append((t, a))
            else:
                b ^= 1
                de.append((t, b))
        for corner in ('typ', 'max', 'min'):
            fe = frontend.Rs485Eth(corner)
            ok &= fe.tx_line(di, de, di0=0, de0=0) == tx_pair(fe, di, de)
    log.check('the bench\'s burst-by-burst use of frontend.tx_line equals one call, on 120 random records', ok)

    # the main run: every case, then the link pulse period, typical corner
    L, t_end = long_run()
    res = run(L, t_end)
    rep = res.rep
    counts = rep.checks()
    v = verdict(res)
    log.check('main run, %d partner frames over %.0f ms: every eth_check check passes, threads healthy, every frame '
              'accounted for' % (len(L.sent), t_end / 1e6), not v,
              v or ', '.join('%s %d' % (k, counts[k][0]) for k in eth_check.Report.CHECKS))
    arp, icmp = replies(rep)
    want = wanted(rep)
    log.check('main run: one reply per request that needs one, %d ARP and %d echo' % want, (arp, icmp) == want,
              'sent %d ARP and %d echo replies' % (arp, icmp))
    log.check('main run: the firmware kept, dropped and passed over exactly the frames 802.3 4.2.4 says',
              not accounted(res), 'counters %s, frames %s' % (
                  {k: res.count[k] for k in ('NKEEP', 'NBAD', 'NOTHER', 'NBUSY')}, L.count()))
    m = tx_margins(rep)
    log.check('main run: link pulses after 8 to 24 ms of silence (14.2.1.1), the first at start',
              len(m['period']) >= 2 and all(8 <= p <= 24 for p in m['period']) and bool(rep.tx_pulses) and
              rep.tx_pulses[0].start < 10e3, 'periods %s ms, the first pulse at %.1f us' % (
                  ', '.join('%.2f' % p for p in m['period']), rep.tx_pulses[0].start / 1e3 if rep.tx_pulses else -1))
    main_m = m

    # transmit corners: the same pads through each transceiver corner, and the two mixed ones FN6307 allows
    for corner, over in (('typ', {}), ('max', {}), ('min', {}),
                         ('slow enable, fast disable', dict(t_zh=50.0, t_zl=40.0, t_hz=0.0, t_lz=0.0)),
                         ('fast enable, slow disable', dict(t_zh=0.0, t_zl=0.0, t_hz=40.0, t_lz=50.0))):
        fe = frontend.Rs485Eth(corner if not over else 'typ', **over)
        r2 = eth_check.check(tx_pair(fe, res.di, res.de), res.rx, mac=MAC, ip=IP, t_start=0.0, t_end=res.t_end)
        mm = tx_margins(r2)
        fails = sorted({c for _, c, _ in r2.failed()})
        detail = 'pulses %s ns; TP_IDL after 0 %s, after 1 %s BT; fails %s' % (
            span(mm['pulse']), span(mm['tp_idl'][0], '%.2f'), span(mm['tp_idl'][1], '%.2f'), fails or 'none')
        if over:
            log.info('transmit through the %s corner (a board matter: FN6307 gives no minimum enable or '
                     'disable time): %s' % (corner, detail))
        else:
            log.check('transmit through the %s corner of FN6307: every check passes' % corner, r2.ok, detail)
            half = fe.p['skew_dd'] / 2
            log.check('transmit through the %s corner, THALF: every edge within the transceiver\'s tSKEW/2 = %.2f ns '
                      'of the grid of exact 50 ns half bits, so the chip adds nothing at a 50%% clock (7.5.2.1 allows '
                      'an encoder 0.5 ns; section 7.3)' % (corner, half),
                      bool(mm['jitter']) and max(mm['jitter']) <= half + 0.05,
                      'peak deviation %s ns over %d frames' % (span(mm['jitter'], '%.2f'), len(mm['jitter'])))

    # the clock's duty error moves every falling-edge move (section 7.3): ARP replies with the falling edge d ns late
    Ld = Line(seed=13)
    td = 0.4e6
    for k in range(4):
        td = Ld.frame(arp_req(spa=bytes([192, 168, 0, 30 + k])), td) + 200e3
    out = []
    for d in (1.0, 2.0, 3.3):
        rd = run(Ld, td, duty=d)
        md = tx_margins(rd.rep)
        out.append('d %.1f ns: %s ns%s' % (d, span(md['jitter'], '%.2f'), '' if rd.rep.ok else ' (fails %s)' % sorted(
            {c for _, c, _ in rd.rep.failed()})))
    log.info('margins: transmit edge deviation with the falling edge d ns late, typical corner (tSKEW/2 = 0.5 ns; the '
             'arithmetic adds d/2), limit 8.5 (7.5.2.1 + 14.3.1.2.3): %s' % '; '.join(out))

    # receive corners, phases, alignments and threads on the exchange
    configs = [('typ', 5.0, 1, (1, 0)), ('typ', 10.0, 2, (2, 3)), ('typ', 15.0, 3, (3, 2)), ('max', 0.0, 1, (0, 1)),
               ('max', 7.5, 2, (2, 1)), ('max', 12.5, 3, (1, 3)), ('min', 2.5, 0, (3, 0)), ('min', 10.0, 1, (0, 2)),
               ('min', 17.5, 2, (2, 0))]
    L, t = exchange(seed=2)
    for corner, ph, al, th in configs:
        r = run(L, t, corner=corner, phase=ph, align=al, threads=th)
        v = verdict(r)
        a, e = replies(r.rep)
        log.check('exchange, %s corner, sampling phase %.1f ns, alignment %d, R on thread %d, T on %d' % (
            corner, ph, al, th[0], th[1]), not v and (a, e) == wanted(r.rep), v or '%d ARP, %d echo replies' % (a, e))

    # the partner's clock at the edges of 7.3.2, on full-size frames too
    for sign in (1, -1):
        L, t = exchange(seed=3, bt=BT * (1 + sign * PPM))
        r = run(L, t, phase=7.5)
        v = verdict(r)
        log.check('exchange with the partner at 10 Mb/s %+.2f%% (802.3 7.3.2), 1518-octet frames included' % (
            sign * PPM * 100), not v, v or 'counters %s' % r.count)

    # short preambles: 7.5.2.2's 47 bits, then shorter, as a receiver may lose bits while it locks (14.2.1.2)
    for pre in (47, 24, 12):
        L, t = exchange(seed=4, pre=pre)
        r = run(L, t, phase=12.5)
        v = verdict(r)
        log.check('exchange with %d preamble bits before the SFD' % pre, not v, v or 'counters %s' % r.count)

    # jitter on every partner transition (14.3.1.3.1, 7.5.2.2): no frame lost at +/-8 ns on any sampling phase
    lost = []
    for k in range(8):
        L, t = frames_only(40, seed=50 + k, jitter=('uniform', 8.0, 8.0))
        r = run(L, t, phase=2.5 * k, check=False)
        lost.append(40 - r.count['NKEEP'])
    log.check('jitter: 40 frames with +/-8 ns random jitter on every transition at each of 8 sampling phases, none '
              'lost', not any(lost), 'lost per phase %s' % lost)
    # RKIND 3 times every edge to the half cycle (section 5.9.6): every phase and FN6307 corner
    for what, jit in (('+/-12 ns random on every transition', ('uniform', 12.0, 12.0)),
                      ('bit-boundary transitions 10 ns late and mid-bit ones 10 ns early', ('alt', -10.0))):
        lost = {}
        for i, corner in enumerate(('typ', 'max', 'min')):
            lost[corner] = []
            for k in range(8):
                L, t = frames_only(20, seed=600 + 10 * k + i, jitter=jit)
                r = run(L, t, corner=corner, phase=2.5 * k, check=False)
                lost[corner].append(20 - r.count['NKEEP'])
        log.check('jitter: %s, 20 frames at each of 8 sampling phases and 3 FN6307 corners, none lost' % what,
                  not any(sum(v) for v in lost.values()), 'lost per phase: %s' % lost)
    # 802.3's jitter (14.3.1.3.1) at 8 more sampling phases and 3 corners, 480 frames a case: random, and the fixed
    # patterns of section 7.3 that RKIND 3's window must not follow (a bit-boundary edge in the window gives way to
    # the clock edge after it, 5.9.6)
    fixed = [('+/-13.5 ns random on every transition (802.3 14.3.1.3.1)', ('uniform', 13.5, 13.5))]
    fixed += [('bit-boundary transitions %g ns late and mid-bit ones %g ns early' % (a, a), ('alt', -a))
              for a in (10.0, 11.0, 12.0, 13.5)]
    fixed += [('bit-boundary transitions %g ns early and mid-bit ones %g ns late' % (a, a), ('alt', a))
              for a in (12.0, 13.5)]
    for i, (what, jit) in enumerate(fixed):
        lost, hl = rx_row(jit, 81000 + 1009 * i)
        log.check('jitter: %s, 20 frames at each of 8 sampling phases (none a multiple of 2.5 ns) and 3 FN6307 '
                  'corners, none of 480 lost' % what, not total(lost) and not hl, 'lost %s%s' % (
                      lost, ', %s' % hl if hl else ''))
    # the receiver's own rows (sections 5.9.6, 7.3): the receiver's skew either way, transitions to HI late and to
    # LO early and the reverse, a cable's data-dependent jitter in both signs, alone and with random jitter making
    # +/-13.5 ns (B.4.1's budget), and broadcast frames from a universally administered source
    rows = [('+/-13.5 ns random, the receiver\'s skew reversed', ('uniform', 13.5, 13.5), CORNERS, -1, None),
            ('bit-boundary transitions 13.5 ns late and mid-bit ones 13.5 ns early, the receiver\'s skew reversed',
             ('alt', -13.5), CORNERS, -1, None),
            ('bit-boundary transitions 13.5 ns early and mid-bit ones 13.5 ns late, the receiver\'s skew reversed',
             ('alt', 13.5), CORNERS, -1, None),
            ('transitions to HI 13.5 ns late and to LO 13.5 ns early', ('pwd', 13.5), CORNERS, 1, None),
            ('transitions to HI 13.5 ns early and to LO 13.5 ns late', ('pwd', -13.5), CORNERS, 1, None),
            ('transitions to HI 13.5 ns late and to LO 13.5 ns early, the receiver\'s skew reversed', ('pwd', 13.5),
             CORNERS, -1, None),
            ('transitions to HI 13.5 ns early and to LO 13.5 ns late, the receiver\'s skew reversed', ('pwd', -13.5),
             CORNERS, -1, None),
            ('a cable\'s data-dependent jitter of 13.5 ns: transitions after a whole bit late, the others early',
             ('isi', 13.5, 0.0), CORNERS, 1, None),
            ('the same, 13.5 ns, the receiver\'s skew reversed', ('isi', 13.5, 0.0), CORNERS, -1, None),
            ('the same, 6 ns of it with +/-7.5 ns random', ('isi', 6.0, 7.5), CORNERS, 1, None),
            ('the same, 10 ns of it with +/-3.5 ns random', ('isi', 10.0, 3.5), CORNERS, 1, None),
            ('the same, 3.5 ns of it with +/-10 ns random (B.4.1\'s link of the greatest length)', ('isi', 3.5, 10.0),
             CORNERS, 1, None),
            ('the reverse sign, transitions after a whole bit early, 10 ns', ('isi', -10.0, 0.0), CORNERS, 1, None),
            ('the reverse sign, 6 ns of it with +/-6.5 ns random (B.4.1\'s short link)', ('isi', -6.0, 6.5), CORNERS,
             1, None),
            ('the reverse sign, 7 ns of it with +/-5.5 ns random', ('isi', -7.0, 5.5), CORNERS, 1, 'info'),
            ('the reverse sign, 4 ns of it with +/-9.5 ns random', ('isi', -4.0, 9.5), CORNERS, -1, None),
            ('bit-boundary transitions 13.5 ns early and mid-bit ones 13.5 ns late, broadcast frames from a '
             'universally administered source', ('alt', 13.5), CORNERS, 1, UAA_MAC)]
    for i, (what, jit, corners, skew, src_mac) in enumerate(rows):
        info = src_mac == 'info'            # past this receiver's limit (section 7.3): reported, not checked
        lost, hl = rx_row(jit, 87000 + 1009 * i, corners=corners, skew=skew, src=None if info else src_mac)
        if info:
            log.info('receive at this receiver\'s limit (section 7.3): %s, frames lost of %d: %d (%s)%s' % (
                what, 160 * len(corners), total(lost), lost, ', %s' % hl if hl else ''))
            continue
        log.check('jitter: %s, 20 frames at each of 8 sampling phases and %d FN6307 corners, none of %d lost' % (
            what, len(corners), 160 * len(corners)), not total(lost) and not hl, 'lost %s%s' % (
                lost, ', %s' % hl if hl else ''))
    for what, jit in (('the reverse sign of a cable\'s data-dependent jitter, 12 ns', ('isi', -12.0, 0.0)),
                      ('the reverse sign, 8 ns of it with +/-5.5 ns random', ('isi', -8.0, 5.5))):
        lost, hl = rx_row(jit, 88000 + int(10 * abs(jit[1])))
        log.info('receive past this receiver\'s limit (section 7.3): %s, frames lost of 480: %d (%s)%s' % (
            what, total(lost), lost, ', %s' % hl if hl else ''))
    lost, hl = rx_row(('alt', 13.5), 89000, corners=('max',), phases=[0.25 * k for k in range(80)], src=UAA_MAC)
    log.check('jitter: bit-boundary transitions 13.5 ns early and mid-bit ones 13.5 ns late at FN6307\'s slowest '
              'corner, broadcast frames from a universally administered source, 20 frames at each of 80 sampling '
              'phases 0.25 ns apart, none of 1600 lost', not total(lost) and not hl, 'lost at %s%s' % (
                  [(0.25 * k, x) for k, x in enumerate(lost['max']) if x] or 'no phase', ', %s' % hl if hl else ''))
    lost, hl = rx_row(('alt', 13.5), 82000, corners=('max',), phases=[0.25 * k for k in range(80)])
    log.check('jitter: bit-boundary transitions 13.5 ns early and mid-bit ones 13.5 ns late at FN6307\'s slowest '
              'corner, whose 4 ns skew makes pulses of 19 ns, 20 frames at each of 80 sampling phases 0.25 ns apart, '
              'none of 1600 lost', not total(lost) and not hl, 'lost at %s%s' % (
                  [(0.25 * k, x) for k, x in enumerate(lost['max']) if x] or 'no phase', ', %s' % hl if hl else ''))
    # the clock's duty error moves every falling-edge sample d ns late (section 7.3)
    for d in (1.0, 2.0, 3.3):
        res = []
        for i, (what, jit) in enumerate((fixed[4], fixed[6], fixed[0], fixed[1])):
            lost, hl = rx_row(jit, 83000 + 1009 * i + int(10 * d), n=10, duty=d)
            res.append((what, total(lost), hl))
        detail = '; '.join('%s: %d%s' % (w.split(' (')[0], x, ' %s' % h if h else '') for w, x, h in res)
        if d < 3:
            log.check('jitter with the falling clock edge %g ns late: the 13.5 ns fixed pattern, its reverse, '
                      '+/-13.5 ns random and the 10 ns fixed pattern, 10 frames at each of 8 phases and 3 corners, '
                      'none lost of 240 in each' % d, not any(x or h for _, x, h in res), detail)
        else:
            log.info('receive with the falling clock edge 3.3 ns late, the same four cases, frames lost of 240: %s'
                     % detail)
    # the partner's clock at the edges of 7.3.2 on long frames: the sampling phase drifts through every value
    for sign in (1, -1):
        for what, jit in (('clean', None), ('the 13.5 ns fixed pattern', ('alt', -13.5))):
            lost, hl = rx_row(jit, 84000 + 500 * (sign > 0) + 100 * (jit is None), n=4, bt=BT * (1 + sign * PPM),
                              octets=1500)
            log.check('the partner at 10 Mb/s %+.2f%% (7.3.2) on 1518-octet frames, %s, 4 frames at each of 8 '
                      'phases and 3 corners, none of 96 lost' % (sign * PPM * 100, what), not total(lost) and not hl,
                      'lost %s%s' % (lost, ', %s' % hl if hl else ''))
    # phase steps: every transition after a random bit of the frame moved s ns, TP_IDL with it. Two frames whose
    # samples 10 ns apart can't be told apart exist from 20 ns, and two that share every transition from 23 ns
    # (section 7.3), so from 20 ns these are information
    for st in (5.0, 10.0, 13.5, 17.0, 20.0, 23.0):
        res = []
        for sign in (1, -1):
            lost, hl = rx_row(('step', sign * st), 85000 + int(100 * st) + 50 * (sign > 0))
            res.append((sign * st, lost, hl))
        if st < 20:
            for v, lost, hl in res:
                log.check('a phase step of %+g ns after a random bit of each frame, 20 frames at each of 8 phases and '
                          '3 corners, none of 480 lost' % v, not total(lost) and not hl, 'lost %s%s' % (
                              lost, ', %s' % hl if hl else ''))
        else:
            log.info('receive past the limit of section 7.3: phase steps of %+g and %+g ns, frames lost of 480: %d '
                     '(%s) and %d (%s)' % (res[0][0], res[1][0], total(res[0][1]), res[0][1], total(res[1][1]),
                                           res[1][1]))
    lost, hl = rx_row(('uniform', 12.0, 18.0), 86000)
    log.info('receive beyond 14.3.1.3.1: 7.5.2.2\'s test signal, +/-12 ns random on the preamble and +/-18 ns on the '
             'frame, frames lost of 480: %d; per phase %s' % (total(lost), lost))
    # frames that end with 1 to 7 bits after their last whole octet (802.3 4.2.9, 4.2.4.2.1 b): REOP's byte end
    # (5.9.7) truncates them, so the good ones are kept and answered and the bad ones dropped
    Lx, tx_end = excess_line()
    for corner, ph in (('typ', 3.7), ('max', 12.2), ('min', 7.9), ('typ', 17.3), ('max', 1.1), ('min', 14.6)):
        r = run(Lx, tx_end, corner=corner, phase=ph)
        v = verdict(r)
        a, e = replies(r.rep)
        log.check('frames with 1 to 7 bits after their FCS (802.3 4.2.9 truncates them), %s corner, phase %.1f ns: '
                  'the 14 good requests kept and answered, the 7 with a bad FCS dropped' % (corner, ph),
                  not v and (a, e) == wanted(r.rep) == (7, 7) and r.count['NBAD'] == 7,
                  v or '%d ARP, %d echo replies; counters %s' % (a, e, r.count))

    # a frame that arrives while the reply goes out: the program has no collision detection (802.3 4.2.3.2.4), so
    # the reply completes and the collision is reported; the frame that arrived is received and answered after it
    L = Line(seed=6)
    e1 = L.frame(arp_req(), 0.4e6)
    r0 = run(L, e1 + 400e3)
    tx0 = [f for f in r0.rep.tx_frames if f.octets]
    t_hit = tx0[0].start + 20e3 if tx0 else e1 + 100e3
    L.frame(arp_req(spa=bytes([192, 168, 0, 20])), t_hit)
    r = run(L, t_hit + 600e3)
    coll = r.rep.failed('collision')
    v = verdict(r, allow=('collision',))
    a, e = replies(r.rep)
    log.check('a request that arrives while the reply goes out: the reply completes (1 collision reported), the '
              'request is received and answered after it', not v and len(coll) == 1 and a == 2,
              v or '%d replies; %s' % (a, coll[0][2] if coll else 'no collision'))

    # an echo request too big for the buffer: no reply, nothing breaks
    L = Line(seed=7)
    t = L.frame(ping(58, seq=30), 0.4e6) + 300e3
    t = L.frame(arp_req(), t) + 300e3
    r = run(L, t)
    miss = r.rep.failed('icmp_reply')
    v = verdict(r, allow=('icmp_reply',))
    log.check('an echo request of 58 data octets, past the buffer: not answered, the ARP request after it is',
              not v and len(miss) == 1 and replies(r.rep) == (1, 0), v or (miss[0][2] if miss else 'answered'))

    # an ARP request padded to the largest frame: R keeps its first 51 words and takes the rest unstored, which holds
    # all 21 words the reply needs
    L = Line(seed=11)
    t = L.frame(G.frame(BCAST, PEER_MAC, 0x0806, G.arp(1, PEER_MAC, PEER_IP, bytes(6), IP) + bytes(1472)), 0.4e6)
    r = run(L, t + 300e3)
    v = verdict(r)
    log.check('an ARP request padded to 1518 octets: stored in part, answered', not v and replies(r.rep) == (1, 0),
              v or '1 ARP reply')

    # an echo request with an IP option: the firmware takes only 5-word headers (no room for options in 512 words),
    # so it is not answered, though RFC 1122 3.2.2.6 asks a reply, a known limit; the request after it is answered.
    # eth_check reports exactly that one miss, against the option request.
    L = Line(seed=12)
    opt = G.echo(8, PEER_IP, IP, 0x1234, 50, bytes(16))
    opt = reheader(bytes([0x46]) + opt[1:2] + bytes([0, len(opt) + 4]) + opt[4:20] + bytes([1, 1, 1, 0]) + opt[20:])
    t1 = L.frame(G.frame(MAC, PEER_MAC, 0x0800, opt), 0.4e6) + 300e3
    t = L.frame(ping(16, seq=51), t1) + 300e3
    r = run(L, t)
    need = eth_check._wants_reply(r.rep.rx_frames[0], MAC, IP, eth_check.DEFAULTS)[0]
    miss = r.rep.failed('icmp_reply')
    bad = r.health + ([accounted(r)] if accounted(r) else [])
    bad += ['%s: %s' % (c, x) for _, c, x in r.rep.failed() if c != 'icmp_reply']
    log.check('an echo request with an IP option (IHL 6) is not answered, a known limit; the next request is',
              not bad and need == 'icmp' and replies(r.rep) == (0, 1) and len(miss) == 1 and 'seq 50' in miss[0][2],
              '; '.join(bad) or '%s; 1 echo reply, to seq 51' % (miss[0][2] if miss else 'no miss reported'))

    # a lone first fragment of an echo request (MF set): RFC 791 3.2 and RFC 1122 3.3.2 reassemble before the ICMP
    # layer sees a datagram, and the rest never comes, so nothing may answer it, as eth_check rules
    L = Line(seed=10)
    t1 = L.frame(ping(56, seq=40, mutate=fragment), 0.4e6) + 300e3
    t = L.frame(ping(56, seq=41), t1) + 300e3
    r = run(L, t)
    v = verdict(r)
    log.check('a lone first fragment of an echo request (MF set) is not answered; the whole request after it is',
              not v and replies(r.rep) == (0, 1), v or '1 echo reply, to seq 41')

    # margins, for the record
    m = main_m
    log.info('margins (typical corner): link pulse %s ns, Figure 14-12 at 2.8 V asks 85 to 137.5' % span(m['pulse']))
    log.info('margins: TP_IDL HI after a final 0 %s BT, after a final 1 %s BT; Figure 14-10 at 2.8 V asks 2.5 to '
             '4.648' % (span(m['tp_idl'][0], '%.2f'), span(m['tp_idl'][1], '%.2f')))
    log.info('margins: edge deviation from the best grid %s ns, limit 8.5 (7.5.2.1 + 14.3.1.2.3); bit rate %s ppm, '
             'limit +/-100 (7.3.2)' % (span(m['jitter'], '%.2f'), span(m['ppm'], '%.1f')))
    log.info('margins: preamble %s bits on the line, at least 54 (7.2.3.2 less 14.2.1.1\'s 2); gap before each '
             'frame sent %s BT, at least 96 (4.4.2); link pulse periods %s ms, 8 to 24 (14.2.1.1)' % (
                 span(m['pre'], '%d'), span(m['ipg'], '%.1f'), span(m['period'], '%.2f')))

    # broken copies
    L, t = exchange(seed=8)
    Li = Line(seed=9)
    Lj, tj = frames_only(20, seed=761, jitter=('uniform', 13.5, 13.5))
    for what, old, new in TEETH:
        text = broken(old, new)
        if 'RKIND 1' in what:           # edges timed to 20 ns lose frames at 802.3's jitter where RKIND 3 keeps them
            r = run(Lj, tj, corner='max', phase=15.0, check=False, text=text)
            lost = 20 - r.count['NKEEP']
            log.check('a broken copy is caught: %s' % what, lost > 4,
                      '%d of 20 frames lost with +/-13.5 ns random jitter, max corner, phase 15 ns' % lost)
            continue
        if 'byte end' in what:          # frames with bits after their last whole octet fail their check
            r = run(Lx, tx_end, text=text)
            v = verdict(r)
            log.check('a broken copy is caught: %s' % what, bool(v), v[:200] if v else 'every check passed')
            continue
        if 'THALF' in what:             # the transmit check above: half bits of 40 and 60 ns
            r = run(L, t, text=text)
            m = tx_margins(r.rep)
            log.check('a broken copy is caught: %s' % what, bool(m['jitter']) and max(m['jitter']) > 0.55,
                      'peak deviation %s ns, typical corner' % span(m['jitter'], '%.2f'))
            continue
        r = run(Li, 40e6, text=text) if 'link pulse' in what else run(L, t, text=text)
        v = verdict(r)
        log.check('a broken copy is caught: %s' % what, bool(v), v[:200] if v else 'every check passed')
