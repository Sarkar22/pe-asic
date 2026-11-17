"""10BASE-T line checker, written from the standards before any firmware.

Sources: IEEE Std 802.3-2005 clauses 3 (frame), 4 (MAC parameters), 7 (Manchester code, preamble, IDL) and 14
(10BASE-T MAU: TP_IDL, link test pulses, jitter); RFC 826 (ARP), RFC 894 (IP over Ethernet), RFC 791 (IPv4),
RFC 792 and RFC 1122 3.2.2.6 (ICMP echo); RFC 7042 Appendix B.1 (EtherType 0x0806). Every limit below names
its clause. Python 3 standard library only.

The input is the twisted pair as the board model gives it (programs/bench/frontend.py turns the chip's pins
into it): a list of (time_ns, level) changes, sorted by time, with level +1 when TD+ is positive with respect to
TD- (HI), -1 when negative (LO) and 0 for silence (802.3 14.2.1.1 calls silence 0 mV +/- 50 mV). The level
before the first change is 0. A record of the chip's transmit pair is checked against the transmitter
requirements; the partner's pair, if given, is decoded so that replies, gaps and collisions can be judged
against it.

    import eth_check
    rep = eth_check.check(tx_line, rx_line, mac=bytes(6), ip=bytes(4), t_end=..., t_start=...)
    rep.ok                  every finding passed (info findings don't count)
    rep.failed()            the failing findings: (time_ns, check, text)
    rep.checks()            {check: (n_pass, n_fail)}
    rep.tx_frames, rep.rx_frames, rep.tx_pulses, rep.rx_pulses

Checks:
  manchester    every bit of a frame has its mid-bit transition and no transition is off the half-bit grid
                (7.3.1.1)
  edge_timing   bit rate within 10 Mb/s +/- 0.01% (7.3.2); every transition within 8.5 ns of a best-fit grid of
                exact half bits: 0.5 ns for the DTE (7.5.2.1) plus 8 ns for the MAU into 100 ohm (14.3.1.2.3);
                the first bit is left out (14.2.1.1)
  preamble_sfd  at least 54 preamble bits on the line (56 from 7.2.3.2, less the 2 that 14.2.1.1 lets the MAU
                drop), alternating and ending in 0, then the SFD 10101011 (3.2.2, 4.2.5)
  frame_size    whole octets (3.4 b); 64 to 1518 octets from DA to FCS (4.4.2); the pad's content is reported,
                not judged: 4.2.3.3 leaves it unspecified and RFC 894 says it should be zeros
  fcs           the FCS equals CRC-32 of DA to pad (3.2.8), computed with zlib.crc32
  tp_idl        the frame ends with a positive transition, then HI for 2.5 BT to the end of Figure 14-10's
                upper bound at the largest allowed amplitude, then silence; a LO after the HI ends inside the
                lower bound; nothing positive after a negative level (14.3.1.2.1)
  link_pulse    a single positive pulse, HI for 0.85 BT to the end of Figure 14-12's upper bound, measured from
                its rising crossing; a LO tail ends inside the lower bound (14.3.1.2.1)
  link_period   16 ms +/- 8 ms of silence between link pulses, and from the start of a frame's TP_IDL to the
                next pulse; a record that goes on past 24 ms with no pulse fails (14.2.1.1)
  ipg           a transmitted preamble starts at least 96 BT after the end of the last frame on either pair
                (4.4.2, 4.2.3.2.1)
  collision     no transmitted frame overlaps a received one (half duplex, 4.2.3.2.1, 14.2.1.4)
  arp_reply     an ARP request for our IP, with our MAC or broadcast as DA, gets exactly one reply whose fields
                follow RFC 826's swap; nothing else is answered (RFC 826, 802.3 4.2.4.1.1). A request with a
                bad FCS, a runt, or another DA needs no reply
  icmp_reply    an echo request to our IP and MAC gets one echo reply: addresses swapped, type 0, code 0, both
                checksums right, identifier, sequence number and data returned, TTL not 0 (RFC 791, 792, 1122). A
                fragment (MF set or an offset) needs no reply: it reaches ICMP only reassembled (RFC 791 3.2)

The integrity checks are independent: arp_reply and icmp_reply judge a reply by its fields whatever its FCS,
and the FCS is judged only by fcs; a frame that fails to decode is reported by manchester and the checks that
need its octets then have nothing to judge for it.

Template readings (14.3.1.2.1). A digital line has no amplitude, so a level is taken as a rectangle of the
largest allowed peak, 2.8 V (14.3.1.2.1), which is the hardest case for the upper bounds:
  - TP_IDL (Figure 14-10, t = 0 at the last positive crossing): the lower bound is 585 mV from 0.25 to 2.25 BT
    and reaches 0 at 2.5 BT, so HI must last 2.5 BT; the upper bound is 3.1 V to 4.5 BT and falls to +50 mV at
    6.0 BT, which it crosses 2.8 V at 4.648 BT; the lower bound is -3.1 V from 2.5 to 4.5 BT and rises to
    -50 mV at 45.0 BT, crossing -2.8 V at 8.484 BT.
  - Link test pulse (Figure 14-12): the text gives no time origin; it is taken at the rising crossing, as
    14-9 and 14-10 define theirs. The lower bound is a 585 mV hump from 0.25 to 0.85 BT, so HI must last
    0.85 BT; the upper bound is 3.1 V to 1.3 BT and falls to 300 mV at 2.0 BT, crossing 2.8 V at 1.375 BT;
    the lower bound is -3.1 V from 0.85 to 2.0 BT and rises to -200 mV at 4.0 BT, crossing -2.8 V at
    2.207 BT.
"""
import zlib

HI, LO, OFF = 1, -1, 0

DEFAULTS = dict(
    bt=100.0,               # ns per bit, 10 Mb/s (7.3.2)
    rate_tol=1e-4,          # +/- 0.01% (7.3.2)
    jitter_tx=8.5,          # ns: 0.5 (7.5.2.1) + 8 (14.3.1.2.3)
    pre_min=54,             # 56 bits (7.2.3.2) less 2 (14.2.1.1)
    min_frame=64,           # octets, DA to FCS (4.4.2 minFrameSize 512 bits)
    max_frame=1518,         # octets (4.4.2 maxUntaggedFrameSize)
    ipg_bt=96,              # bit times (4.4.2 interFrameGap)
    amp=2.8,                # V, the largest peak of 14.3.1.2.1, for the template bounds
    nlp_min=8e6,            # ns: 16 ms - 8 ms (14.2.1.1)
    nlp_max=24e6,           # ns: 16 ms + 8 ms
    pulse_max_bt=3.0,       # a burst no longer than this is a pulse, not a frame
    pll_acq=0.5,            # decoder phase tracking: fraction of each mid-bit error taken, first 8 bits
    pll_gain=1.0 / 16,      # and after them
)

SFD = [1, 0, 1, 0, 1, 0, 1, 1]          # 3.2.2, in transmission order
ETH_ARP, ETH_IP = 0x0806, 0x0800        # RFC 7042 B.1, RFC 894
BROADCAST = b'\xff' * 6                  # 3.2.3.1


def tp_idl_bounds(amp):
    """(HI min, HI max, LO end max) in BT after the last positive crossing (Figure 14-10)."""
    hi_max = 4.5 + 1.5 * (3.1 - amp) / (3.1 - 0.05)
    lo_end = 4.5 + 40.5 * (3.1 - amp) / (3.1 - 0.05)
    return 2.5, hi_max, lo_end


def link_pulse_bounds(amp):
    """(HI min, HI max, LO end max) in BT after the pulse's rising crossing (Figure 14-12)."""
    hi_max = 1.3 + 0.7 * (3.1 - amp) / (3.1 - 0.3)
    lo_end = 2.0 + 2.0 * (3.1 - amp) / (3.1 - 0.2)
    return 0.85, hi_max, lo_end


# ---------------------------------------------------------------------------------------------------------------
# results

class Finding:
    __slots__ = ('t', 'check', 'ok', 'text')

    def __init__(self, t, check, ok, text):
        self.t, self.check, self.ok, self.text = t, check, ok, text

    def __repr__(self):
        tag = 'INFO' if self.ok is None else 'PASS' if self.ok else 'FAIL'
        return '%s %s @%.1f us: %s' % (tag, self.check, self.t / 1000.0, self.text)


class Report:
    CHECKS = ('manchester', 'edge_timing', 'preamble_sfd', 'frame_size', 'fcs', 'tp_idl', 'link_pulse',
              'link_period', 'ipg', 'collision', 'arp_reply', 'icmp_reply')

    def __init__(self):
        self.findings = []
        self.tx_frames, self.rx_frames, self.tx_pulses, self.rx_pulses = [], [], [], []

    def add(self, t, check, ok, text):
        self.findings.append(Finding(t, check, ok, text))

    @property
    def ok(self):
        return all(f.ok is not False for f in self.findings)

    def failed(self, check=None):
        return [(f.t, f.check, f.text) for f in self.findings
                if f.ok is False and (check is None or f.check == check)]

    def checks(self):
        out = {c: [0, 0] for c in self.CHECKS}
        for f in self.findings:
            if f.ok is not None:
                out.setdefault(f.check, [0, 0])[0 if f.ok else 1] += 1
        return {c: tuple(v) for c, v in out.items()}

    def failing_checks(self):
        return sorted({f.check for f in self.findings if f.ok is False})

    def summary(self):
        lines = ['%-13s %4d pass %4d fail' % (c, p, n) for c, (p, n) in sorted(self.checks().items())]
        lines += [repr(f) for f in self.findings if f.ok is False]
        return '\n'.join(lines)


class Frame:
    """One burst of activity long enough to be a frame. Times in ns."""

    def __init__(self, side):
        self.side = side
        self.start = self.stop = 0.0    # first non-zero level; return to silence
        self.bits = []                  # every decoded bit, preamble and SFD included
        self.mids = []                  # time of each decoded bit's mid-bit transition
        self.bounds = []                # (k, time) of each bit-boundary transition after bit k
        self.inner = []                 # times of every sign change in the burst
        self.lead = 0                   # 1 if the burst began at a mid-bit (first half lost)
        self.violations = []            # (t, text): Manchester code violations
        self.sfd_at = None              # index in bits of the SFD's last bit
        self.octets = None              # DA to FCS, or None if no SFD was found
        self.extra_bits = 0             # bits after the last whole octet
        self.end = None                 # end of the last bit cell (start of idle)
        self.t_idl = None               # last positive crossing (t = 0 of Figure 14-10)
        self.tail = []                  # (t, level) after the last decoded bit
        self.period = self.jitter = None
        self.fields = {}

    def __repr__(self):
        n = len(self.octets) if self.octets is not None else None
        return 'Frame(%s %.1f-%.1f us, %d bits, octets %s)' % (self.side, self.start / 1e3, self.stop / 1e3,
                                                                len(self.bits), n)


class Pulse:
    __slots__ = ('side', 'start', 'stop', 'width', 'levels')

    def __init__(self, side, start, stop, width, levels):
        self.side, self.start, self.stop, self.width, self.levels = side, start, stop, width, levels

    def __repr__(self):
        return 'Pulse(%s %.3f ms, %.1f ns)' % (self.side, self.start / 1e6, self.width)


# ---------------------------------------------------------------------------------------------------------------
# bursts and Manchester decoding

def _norm(line):
    """Sorted changes, with repeated levels dropped."""
    out, last = [], OFF
    for t, v in sorted(line, key=lambda e: e[0]):
        v = HI if v > 0 else LO if v < 0 else OFF
        if v != last:
            if out and out[-1][0] == t:
                out.pop()
                last = out[-1][1] if out else OFF
                if v == last:
                    continue
            out.append((float(t), v))
            last = v
    return out


def bursts(line):
    """[(start, stop, [(t, level), ...])]: each run of non-zero level, with the changes inside it."""
    out, cur = [], None
    for t, v in _norm(line):
        if cur is None:
            if v != OFF:
                cur = [t, None, [(t, v)]]
        elif v == OFF:
            cur[1] = t
            out.append(tuple(cur))
            cur = None
        else:
            cur[2].append((t, v))
    if cur is not None:
        cur[1] = float('inf')
        out.append(tuple(cur))
    return out


class _Levels:
    """The level of one burst at any time."""

    def __init__(self, start, stop, changes):
        self.start, self.stop = start, stop
        self.t = [c[0] for c in changes]
        self.v = [c[1] for c in changes]

    def at(self, t):
        if t < self.start or t >= self.stop:
            return OFF
        lo, hi = 0, len(self.t)
        while hi - lo > 1:
            m = (lo + hi) // 2
            if self.t[m] <= t:
                lo = m
            else:
                hi = m
        return self.v[lo]


def _walk(inner, a0, T, o):
    """Bits and mid-bit times from the anchor inner[a0], taken as a mid-bit transition. Each next mid-bit
    transition is the change nearest the predicted place within a quarter bit, and its direction is the bit
    (to HI: 1). The prediction follows each mid-bit transition by a fraction of its error: pll_acq for the first
    8 bits, then pll_gain. Bit-boundary transitions sit half a bit from both mid-bit ones, outside the window."""
    bits, mids = [], []
    m = inner[a0][0]
    i, n = a0, len(inner)
    while True:
        best = None
        j = i
        while j < n and inner[j][0] <= m + T / 4:
            t, v = inner[j]
            if t >= m - T / 4 and (best is None or abs(t - m) < abs(best[0] - m)):
                best = (t, v)
            j += 1
        if best is None:
            break
        bits.append(1 if best[1] == HI else 0)
        mids.append(best[0])
        g = o['pll_acq'] if len(bits) <= 8 else o['pll_gain']
        m = m + T + g * (best[0] - m)
        i = j
    return bits, mids


def decode(start, stop, changes, side, o):
    """Decodes one burst of 802.3 Manchester (7.3.1.1; on the pair, CD1 is LO then HI, 14.2.1.1). The first bit
    may be corrupt (14.2.1.1), so each of the first three sign changes is tried as the first mid-bit
    transition, and the one that decodes the most bits is kept."""
    T = o['bt']
    fr = Frame(side)
    fr.start, fr.stop = start, stop
    lv = _Levels(start, stop, changes)
    inner = changes[1:]                         # sign changes inside the burst
    fr.inner = [t for t, _ in inner]
    if not inner:
        fr.tail = list(changes)
        return fr
    best = None
    for a0 in range(min(3, len(inner))):
        bits, mids = _walk(inner, a0, T, o)
        if best is None or len(bits) > len(best[0]):
            best = (bits, mids)
    fr.bits, fr.mids = best
    if fr.mids and lv.at(start) == HI and fr.mids[0] - start > 0.75 * T and fr.bits[0] == 0:
        fr.lead = 1                             # the burst began at a CD1's mid-bit, its LO half lost
    if not fr.mids:
        fr.tail = list(changes)
        return fr
    # Between two mid-bit transitions there is exactly one change, near half way, if the bits are equal, and
    # none if they differ. Any other change is a code violation.
    mids = fr.mids
    between = [[] for _ in mids]
    k = 0
    for t, v in inner:
        if t < mids[0]:
            continue                            # inside the first bit, which may be corrupt (14.2.1.1)
        if t > mids[-1]:
            break
        while k + 1 < len(mids) and mids[k + 1] <= t:
            k += 1
        if t != mids[k]:
            between[k].append(t)
    for k in range(len(mids) - 1):
        ts = between[k]
        span = mids[k + 1] - mids[k]
        want = 1 if fr.bits[k] == fr.bits[k + 1] else 0
        keep = None
        if want and ts:
            keep = min(ts, key=lambda t: abs((t - mids[k]) / span - 0.5))
            if abs((keep - mids[k]) / span - 0.5) > 0.25:
                keep = None
        for t in ts:
            if t != keep:
                fr.violations.append((t, 'transition at %.2f of the bit after the mid-bit transition at %.1f ns'
                                      % ((t - mids[k]) / span, mids[k])))
            else:
                fr.bounds.append((k, t))
        if want and keep is None and not ts:
            fr.violations.append((mids[k] + span / 2, 'no bit-boundary transition between two equal bits'))
    last = mids[-1]
    fr.tail = [(last, HI if fr.bits[-1] else LO)] + [(t, v) for t, v in changes if t > last]
    return fr


def _octets(fr):
    """Finds the SFD and packs what follows into octets, least significant bit first (3.3)."""
    bits = fr.bits
    for i in range(len(bits) - 1):
        if bits[i] == 1 and bits[i + 1] == 1:
            fr.sfd_at = i + 1
            break
    if fr.sfd_at is None:
        return
    data = bits[fr.sfd_at + 1:]
    whole = len(data) // 8
    fr.extra_bits = len(data) - 8 * whole
    fr.octets = bytes(sum(data[8 * q + b] << b for b in range(8)) for q in range(whole))


def _fit(points):
    """Least squares t = phi + n * h over (n, t); returns (h, phi, peak deviation from the best grid)."""
    if len(points) < 2:
        return None, None, 0.0
    sn = sum(p[0] for p in points)
    st = sum(p[1] for p in points)
    snn = sum(p[0] * p[0] for p in points)
    snt = sum(p[0] * p[1] for p in points)
    k = len(points)
    den = k * snn - sn * sn
    if den == 0:
        return None, None, 0.0
    h = (k * snt - sn * st) / den
    phi = (st - h * sn) / k
    res = [t - phi - n * h for n, t in points]
    return h, phi, (max(res) - min(res)) / 2


def _timing(fr, skip_first):
    """Half-bit indices of the frame's transitions, then the fit (7.5.2.1, 14.3.1.2.3). Bit 0 is skipped when
    skip_first (14.2.1.1: the first bit sent may contain phase violations)."""
    pts = []
    for k, t in enumerate(fr.mids):
        pts.append((2 * (k + fr.lead) + 1, t))
    for k, t in fr.bounds:
        pts.append((2 * (k + fr.lead) + 2, t))
    if skip_first:
        pts = [p for p in pts if p[0] > 2]
    h, phi, jit = _fit(pts)
    return (2 * h if h else None), jit, len(pts)


# ---------------------------------------------------------------------------------------------------------------
# frame contents

def _u16(b, i):
    return (b[i] << 8) | b[i + 1]


def _csum_ok(data):
    """True if the one's complement sum of the 16-bit words (an odd octet padded with zero) is 0xFFFF
    (RFC 791 3.1 header checksum; RFC 792 checksum)."""
    if len(data) % 2:
        data = data + b'\x00'
    s = 0
    for i in range(0, len(data), 2):
        s += (data[i] << 8) | data[i + 1]
    while s >> 16:
        s = (s & 0xFFFF) + (s >> 16)
    return s == 0xFFFF


def parse(octets):
    """Fields of a frame given DA to FCS (3.1.1), with ARP (RFC 826) and IPv4/ICMP (RFC 791, 792) decoded."""
    f = {}
    if octets is None or len(octets) < 14:
        return f
    f['dst'], f['src'], f['type'] = octets[0:6], octets[6:12], _u16(octets, 12)
    body = octets[14:-4] if len(octets) >= 18 else octets[14:]
    f['fcs_ok'] = len(octets) >= 18 and zlib.crc32(octets[:-4]) == int.from_bytes(octets[-4:], 'little')
    if f['type'] == ETH_ARP and len(body) >= 8:
        a = dict(hrd=_u16(body, 0), pro=_u16(body, 2), hln=body[4], pln=body[5], op=_u16(body, 6))
        hl, pl = a['hln'], a['pln']
        need = 8 + 2 * hl + 2 * pl
        if len(body) >= need:
            p = 8
            a['sha'] = body[p:p + hl]
            p += hl
            a['spa'] = body[p:p + pl]
            p += pl
            a['tha'] = body[p:p + hl]
            p += hl
            a['tpa'] = body[p:p + pl]
            p += pl
            a['pad'] = body[p:]
        f['arp'] = a
    elif f['type'] == ETH_IP and len(body) >= 20:
        ihl = (body[0] & 15) * 4
        ip = dict(version=body[0] >> 4, ihl=ihl, total=_u16(body, 2), ttl=body[8], proto=body[9],
                  src=body[12:16], dst=body[16:20], mf=(body[6] >> 5) & 1, offset=_u16(body, 6) & 0x1FFF)
        ip['hdr_ok'] = ihl >= 20 and len(body) >= ihl and _csum_ok(body[:ihl])
        if ihl >= 20 and ip['total'] <= len(body):
            ip['pad'] = body[ip['total']:]
            msg = body[ihl:ip['total']]
            if ip['proto'] == 1 and len(msg) >= 8:
                ip['icmp'] = dict(type=msg[0], code=msg[1], id=_u16(msg, 4), seq=_u16(msg, 6), data=msg[8:],
                                  csum_ok=_csum_ok(msg))
        f['ip'] = ip
    return f


# ---------------------------------------------------------------------------------------------------------------
# the checker

def _classify(line, side, o, rep):
    T = o['bt']
    frames, pulses = [], []
    for start, stop, changes in bursts(line):
        if stop - start <= o['pulse_max_bt'] * T:
            levels = [v for _, v in changes]
            hi = [(t, (changes[k + 1][0] if k + 1 < len(changes) else stop)) for k, (t, v) in enumerate(changes)
                  if v == HI]
            width = (hi[0][1] - hi[0][0]) if hi and changes[0][1] == HI else 0.0
            pulses.append(Pulse(side, start, stop, width, list(zip([c[0] for c in changes], levels))))
            continue
        fr = decode(start, stop, changes, side, o)
        _octets(fr)
        if fr.octets is not None:
            fr.fields = parse(fr.octets)
            fr.end = fr.mids[-1] + T / 2
        frames.append(fr)
    return frames, pulses


def check(tx, rx=None, mac=None, ip=None, t_end=None, t_start=None, **opts):
    """Checks a record. tx and rx are lists of (time_ns, level). mac (6 bytes) and ip (4 bytes) are the chip's,
    for the reply checks. t_start and t_end bound the record for link_period. opts override DEFAULTS."""
    o = dict(DEFAULTS)
    o.update(opts)
    T = o['bt']
    rep = Report()
    rep.tx_frames, rep.tx_pulses = _classify(tx, 'tx', o, rep)
    if rx is not None:
        rep.rx_frames, rep.rx_pulses = _classify(rx, 'rx', o, rep)
    for fr in rep.tx_frames:
        _check_frame(fr, o, rep)
    for fr in rep.rx_frames:
        _report_rx(fr, o, rep)
    _check_pulses(rep, o)
    _check_period(rep, o, t_start, t_end)
    _check_gaps(rep, o)
    if mac is not None and ip is not None:
        _check_replies(rep, bytes(mac), bytes(ip), o)
    rep.findings.sort(key=lambda f: f.t)
    return rep


def _check_frame(fr, o, rep):
    T = o['bt']
    t0 = fr.start
    # manchester
    if not fr.bits:
        rep.add(t0, 'manchester', False, 'no Manchester bit decodes in a burst of %.1f us' % ((fr.stop - t0) / 1e3))
        return
    for t, text in fr.violations:
        rep.add(t, 'manchester', False, text)
    tail_bad = _tail_shape(fr)
    if tail_bad:
        rep.add(fr.mids[-1], 'manchester', False, tail_bad)
    if not fr.violations and not tail_bad:
        rep.add(t0, 'manchester', True, '%d bits decoded' % len(fr.bits))
    # edge_timing
    period, jit, npts = _timing(fr, True)
    fr.period, fr.jitter = period, jit
    if period is None:
        rep.add(t0, 'edge_timing', False, 'too few transitions to time (%d)' % npts)
    else:
        err = period / T - 1
        ok_rate = abs(err) <= o['rate_tol']
        rep.add(t0, 'edge_timing', ok_rate, 'bit period %.4f ns: %+.0f ppm against +/- %.0f ppm (802.3 7.3.2)'
                % (period, err * 1e6, o['rate_tol'] * 1e6))
        rep.add(t0, 'edge_timing', jit <= o['jitter_tx'] + 1e-9,
                'peak deviation from a grid of exact half bits %.2f ns against %.1f ns (7.5.2.1, 14.3.1.2.3)'
                % (jit, o['jitter_tx']))
    # preamble and SFD
    if fr.sfd_at is None or fr.sfd_at < 7:
        rep.add(t0, 'preamble_sfd', False, 'no SFD: %s' % ('the bits never show 1, 1' if fr.sfd_at is None
                                                            else 'two 1s in a row at bit %d' % (fr.sfd_at - 1)))
        return
    pre = fr.bits[:fr.sfd_at - 7]
    sfd = fr.bits[fr.sfd_at - 7:fr.sfd_at + 1]
    n_pre = len(pre) + fr.lead
    alt = all(pre[q] != pre[q + 1] for q in range(len(pre) - 1)) and (not pre or pre[-1] == 0)
    if fr.lead and pre and pre[0] != 0:
        alt = False
    ok = sfd == SFD and n_pre >= o['pre_min'] and alt
    rep.add(t0, 'preamble_sfd', ok, '%d preamble bits (at least %d), %s, SFD %s' % (
        n_pre, o['pre_min'], 'alternating, ending in 0' if alt else 'NOT alternating or not ending in 0',
        ''.join(map(str, sfd))))
    # frame size and pad
    octs = fr.octets
    n = len(octs)
    ok = fr.extra_bits == 0 and o['min_frame'] <= n <= o['max_frame']
    rep.add(fr.end, 'frame_size', ok, '%d octets%s from DA to FCS (64 to 1518, 802.3 4.4.2)' % (
        n, ' and %d bits' % fr.extra_bits if fr.extra_bits else ''))
    pad = _pad_of(fr.fields)
    if pad is not None:
        # 802.3 4.2.3.3 leaves the pad's content unspecified; RFC 894 says it should be zeros: reported only
        rep.add(fr.end, 'frame_size', None, 'pad of %d octets is %s (802.3 4.2.3.3: unspecified; RFC 894: '
                'should be zeros)' % (len(pad), 'zeros' if not any(pad) else 'NOT zeros: ' + pad.hex()))
    # fcs
    if n >= 18:
        want = zlib.crc32(octs[:-4])
        got = int.from_bytes(octs[-4:], 'little')
        rep.add(fr.end, 'fcs', want == got, 'FCS %08X, CRC-32 of DA to pad %08X (802.3 3.2.8)' % (got, want))
    else:
        rep.add(fr.end, 'fcs', False, 'only %d octets: no FCS to check' % n)
    # TP_IDL
    _check_tp_idl(fr, o, rep)


def _pad_of(f):
    if 'arp' in f and 'pad' in f['arp']:
        return f['arp']['pad']
    if 'ip' in f and 'pad' in f['ip']:
        return f['ip']['pad']
    return None


def _tail_shape(fr):
    """After the last bit only TP_IDL may follow: a positive transition at the boundary after a final 0, HI,
    then optionally LO, then silence. Anything more is a code violation."""
    seq = [v for _, v in fr.tail]
    # seq[0] is the level after the last mid-bit transition
    if seq[:1] == [LO]:
        seq = seq[1:]
    if not seq or seq[0] != HI:
        return 'after the last bit the line goes %s, not HI (TP_IDL)' % (seq[:1] or ['silent'])
    rest = seq[1:]
    if rest not in ([], [LO]):
        return 'transitions after the end of the frame: levels %s' % rest
    return None


def _check_tp_idl(fr, o, rep):
    T = o['bt']
    lo_min, hi_max, lo_end = tp_idl_bounds(o['amp'])
    tail = fr.tail
    # the last positive crossing
    t0, k = None, None
    for q, (t, v) in enumerate(tail):
        if v == HI and (q > 0 or not fr.bits or fr.bits[-1] == 1):
            t0, k = t, q
            break
    if t0 is None:
        rep.add(fr.mids[-1], 'tp_idl', False, 'no positive transition ends the frame (14.3.1.2.1)')
        return
    fr.t_idl = t0
    t_next = tail[k + 1][0] if k + 1 < len(tail) else fr.stop
    width = (t_next - t0) / T
    ok = lo_min - 1e-9 <= width <= hi_max + 1e-9
    text = 'HI for %.2f BT after the last positive crossing (%.2f to %.3f BT, Figure 14-10)' % (width, lo_min,
                                                                                                 hi_max)
    after = [v for _, v in tail[k + 1:]]
    if after and after[0] == LO:
        lo_stop = (tail[k + 2][0] if k + 2 < len(tail) else fr.stop)
        e = (lo_stop - t0) / T
        if e > lo_end + 1e-9:
            ok = False
        text += '; LO until %.2f BT (by %.3f BT)' % (e, lo_end)
        if len(after) > 1 and after[1] == HI:
            ok = False
            text += '; HI again after LO'
    rep.add(t0, 'tp_idl', ok, text)


def _report_rx(fr, o, rep):
    """The partner's frames are stimulus: their decode is reported, not judged."""
    if fr.octets is None:
        rep.add(fr.start, 'manchester', None, 'rx: a burst of %.1f us with no SFD' % ((fr.stop - fr.start) / 1e3))
        return
    period, jit, _ = _timing(fr, False)
    fr.period, fr.jitter = period, jit
    rep.add(fr.start, 'edge_timing', None, 'rx frame of %d octets, period %s ns, peak jitter %.1f ns' % (
        len(fr.octets), '%.3f' % period if period else '?', jit))


def _check_pulses(rep, o):
    T = o['bt']
    w_min, w_max, lo_end = link_pulse_bounds(o['amp'])
    for p in rep.tx_pulses:
        lv = [v for _, v in p.levels]
        if not lv or lv[0] != HI:
            rep.add(p.start, 'link_pulse', False, 'pulse starts %s, not positive (14.3.1.2.1)'
                    % ('LO' if lv and lv[0] == LO else 'silent'))
            continue
        w = p.width / T
        ok = w_min - 1e-9 <= w <= w_max + 1e-9
        text = 'HI for %.1f ns (%.0f to %.1f ns, Figure 14-12)' % (p.width, w_min * T, w_max * T)
        if len(lv) > 1:
            if lv[1] == LO:
                e = ((p.levels[2][0] if len(p.levels) > 2 else p.stop) - p.start) / T
                if e > lo_end + 1e-9:
                    ok = False
                text += '; LO tail until %.2f BT (by %.3f BT)' % (e, lo_end)
            if len(lv) > 2:
                ok = False
                text += '; more than one HI and one LO: %s' % lv
        rep.add(p.start, 'link_pulse', ok, text)


def _check_period(rep, o, t_start, t_end):
    """14.2.1.1: silence of 16 ms +/- 8 ms, then a link test pulse; after a packet, silence first."""
    lo, hi = o['nlp_min'], o['nlp_max']
    ev = [(p.start, 'pulse') for p in rep.tx_pulses]
    ev += [(fr.t_idl if fr.t_idl is not None else fr.stop, 'frame') for fr in rep.tx_frames]
    ev += [(fr.start, 'frame start') for fr in rep.tx_frames]
    ev.sort()
    prev = None                         # (time, kind) of the last pulse or end of frame
    if t_start is not None:
        first = ev[0][0] if ev else t_end
        if first is not None and first - t_start > hi:
            rep.add(t_start, 'link_period', False, 'no link pulse or frame for %.2f ms after the start (16 +/- 8 ms)'
                    % ((first - t_start) / 1e6))
    for t, kind in ev:
        if kind == 'frame start':
            if prev is not None and t - prev[0] > hi:
                rep.add(prev[0], 'link_period', False, '%.2f ms of silence after a %s before a frame (at most 24 ms)'
                        % ((t - prev[0]) / 1e6, prev[1]))
            prev = None                 # a frame may cut TP_IDL anywhere (14.2.1.1)
            continue
        if kind == 'pulse' and prev is not None:
            d = t - prev[0]
            ok = lo - 1e-6 <= d <= hi + 1e-6
            rep.add(t, 'link_period', ok, '%.3f ms of silence after a %s (8 to 24 ms, 802.3 14.2.1.1)' % (
                d / 1e6, prev[1]))
        prev = (t, 'pulse' if kind == 'pulse' else 'frame')
    if t_end is not None and prev is not None and t_end - prev[0] > hi:
        rep.add(prev[0], 'link_period', False, 'no link pulse in the %.2f ms after this %s (at most 24 ms)'
                % ((t_end - prev[0]) / 1e6, prev[1]))


def _data_end(fr):
    return fr.end if fr.end is not None else fr.stop


def _check_gaps(rep, o):
    """ipg: from the end of the last bit cell of any earlier frame to the start of this preamble (4.4.2 Note 1).
    collision: the bit cells of a transmitted frame against those of a received one (TP_IDL is idle)."""
    T = o['bt']
    gap = o['ipg_bt'] * T
    ends = [(_data_end(fr), fr) for fr in rep.tx_frames + rep.rx_frames]
    for fr in rep.tx_frames:
        before = [e for e, f in ends if f is not fr and e <= fr.start]
        if before:
            d = fr.start - max(before)
            rep.add(fr.start, 'ipg', d >= gap - 1e-6, '%.2f us from the end of the last frame to this preamble '
                    '(at least %.1f us, 802.3 4.4.2)' % (d / 1e3, gap / 1e3))
        hit = [rf for rf in rep.rx_frames if rf.start < _data_end(fr) and fr.start < _data_end(rf)]
        for rf in hit:
            who = ('ours started while the partner\'s was on the line: no deference (4.2.3.2.1)'
                   if fr.start >= rf.start else 'the partner started during ours: a collision, which a half-duplex '
                   'MAC must detect, jam and retry (4.2.3.2.4, 4.2.3.2.5)')
            rep.add(max(fr.start, rf.start), 'collision', False,
                    'transmit %.1f to %.1f us overlaps the partner\'s frame %.1f to %.1f us: %s' % (
                        fr.start / 1e3, _data_end(fr) / 1e3, rf.start / 1e3, _data_end(rf) / 1e3, who))
        if not hit and rep.rx_frames:
            rep.add(fr.start, 'collision', True, 'no received frame overlaps this one')


def _wants_reply(rf, mac, ip, o):
    """(kind, why). kind is 'arp' or 'icmp' if the partner's frame must be answered, 'icmp?' if a reply is
    allowed but not required (an echo request to the broadcast MAC: RFC 1122 3.2.2.6 lets a host discard it),
    and None if nothing may answer it."""
    f = rf.fields
    octs = rf.octets
    if octs is None or rf.extra_bits:
        return None, 'not whole octets'
    if len(octs) < o['min_frame']:
        return None, 'a runt of %d octets (802.3 4.2.4.2.2)' % len(octs)
    if not f.get('fcs_ok'):
        return None, 'a bad FCS (802.3 3.4)'
    if f['dst'] not in (mac, BROADCAST):
        return None, 'DA %s is not ours nor broadcast (802.3 4.2.4.1.1)' % f['dst'].hex()
    a = f.get('arp')
    if a is not None:
        if a.get('tpa') is None:
            return None, 'a short ARP packet'
        if (a['hrd'], a['pro'], a['hln'], a['pln']) != (1, ETH_IP, 6, 4):
            return None, 'ARP for hrd %d, pro %04X, hln %d, pln %d (RFC 826)' % (a['hrd'], a['pro'], a['hln'],
                                                                               a['pln'])
        if a['tpa'] != ip:
            return None, 'ARP for %s, not our IP (RFC 826)' % '.'.join(map(str, a['tpa']))
        if a['op'] != 1:
            return None, 'ARP op %d, not a request' % a['op']
        return 'arp', 'ARP request for our IP from %s' % a['sha'].hex()
    p = f.get('ip')
    if p is not None and 'icmp' in p and p['dst'] == ip and p['icmp']['type'] == 8 and p['icmp']['code'] == 0:
        if p['mf'] or p['offset']:
            # RFC 791 3.2 and RFC 1122 3.3.2: a fragment reaches ICMP only once the datagram is reassembled
            return None, 'a fragment of an echo request (MF %d, offset %d): not a whole datagram (RFC 791 3.2)' % (
                p['mf'], p['offset'])
        if not (p['hdr_ok'] and p['icmp']['csum_ok']):
            return None, 'an echo request with a bad checksum (RFC 791, RFC 792)'
        why = 'echo request id %d seq %d' % (p['icmp']['id'], p['icmp']['seq'])
        return ('icmp' if f['dst'] == mac else 'icmp?'), why
    return None, 'nothing to answer'


def _matches(kind, tf, rf):
    """True if the transmitted frame's own fields point back at this request."""
    if kind == 'arp':
        a, q = tf.fields.get('arp', {}), rf.fields['arp']
        return a.get('tha') == q['sha'] and a.get('tpa') == q['spa']
    c, q = tf.fields.get('ip', {}).get('icmp', {}), rf.fields['ip']['icmp']
    return (c.get('id'), c.get('seq')) == (q['id'], q['seq'])


def _check_replies(rep, mac, ip, o):
    """Each request takes the first unused transmitted frame of its kind after it, one that names it if there is
    one; failing that, the first one that names no other request, so that a reply with wrong fields is still judged
    against its request, and a reply that answers another request is never blamed on this one. Whatever is left
    answers nothing."""
    used = set()
    wants = [(rf, _wants_reply(rf, mac, ip, o)) for rf in rep.rx_frames]
    for rf, (kind, why) in wants:
        start = _data_end(rf)
        if kind is None:
            chk = 'icmp_reply' if 'ip' in rf.fields else 'arp_reply'
            rep.add(start, chk, None, 'needs no reply: ' + why)
            continue
        base = 'arp' if kind == 'arp' else 'ip'
        chk = 'arp_reply' if kind == 'arp' else 'icmp_reply'
        cand = [f for f in rep.tx_frames if f.start >= start and id(f) not in used and f.octets is not None
                and base in f.fields]
        named = [f for f in cand if _matches(kind.rstrip('?'), f, rf)]
        others = [q for q, (k, _) in wants if q is not rf and k is not None and k.rstrip('?') == kind.rstrip('?')]
        free = [f for f in cand if not any(_matches(kind.rstrip('?'), f, q) for q in others)]
        pick = named[0] if named else (free[0] if free and kind != 'icmp?' else None)
        if pick is None:
            rep.add(start, chk, None if kind == 'icmp?' else False, 'no reply to the %s' % why)
            continue
        used.add(id(pick))
        if kind == 'arp':
            _judge_arp(pick, rf, mac, ip, rep)
        else:
            _judge_icmp(pick, rf, mac, ip, rep)
    for f in rep.tx_frames:
        if id(f) not in used and f.octets is not None:
            chk = 'icmp_reply' if 'ip' in f.fields else 'arp_reply'
            rep.add(f.start, chk, False, 'a frame that answers no request: type %04X to %s' % (
                f.fields.get('type', 0), f.fields.get('dst', b'').hex()))


def _judge_arp(tf, rf, mac, ip, rep):
    q, f = rf.fields['arp'], tf.fields
    a = f.get('arp', {})
    want = [('DA', f.get('dst'), q['sha']), ('SA', f.get('src'), mac), ('type', f.get('type'), ETH_ARP),
            ('hrd', a.get('hrd'), 1), ('pro', a.get('pro'), ETH_IP), ('hln', a.get('hln'), 6),
            ('pln', a.get('pln'), 4), ('op', a.get('op'), 2), ('sha', a.get('sha'), mac), ('spa', a.get('spa'), ip),
            ('tha', a.get('tha'), q['sha']), ('tpa', a.get('tpa'), q['spa'])]
    bad = ['%s %s, want %s' % (n, _show(g), _show(w)) for n, g, w in want if g != w]
    rep.add(tf.start, 'arp_reply', not bad, 'reply to %s: %s (RFC 826)' % (
        q['sha'].hex(), '; '.join(bad) if bad else 'every field as RFC 826 swaps them'))


def _judge_icmp(tf, rf, mac, ip, rep):
    q, f = rf.fields['ip'], tf.fields
    p = f.get('ip', {})
    c = p.get('icmp', {})
    qi = q['icmp']
    want = [('DA', f.get('dst'), rf.fields['src']), ('SA', f.get('src'), mac), ('type', f.get('type'), ETH_IP),
            ('version', p.get('version'), 4), ('proto', p.get('proto'), 1), ('IP src', p.get('src'), ip),
            ('IP dst', p.get('dst'), q['src']), ('ICMP type', c.get('type'), 0), ('ICMP code', c.get('code'), 0),
            ('id', c.get('id'), qi['id']), ('seq', c.get('seq'), qi['seq']), ('data', c.get('data'), qi['data'])]
    bad = ['%s %s, want %s' % (n, _show(g), _show(w)) for n, g, w in want if g != w]
    if not p.get('hdr_ok'):
        bad.append('IP header checksum wrong (RFC 791)')
    if c and not c.get('csum_ok'):
        bad.append('ICMP checksum wrong (RFC 792)')
    if p.get('ttl') == 0:
        bad.append('TTL 0 (RFC 1122 3.2.1.7)')
    rep.add(tf.start, 'icmp_reply', not bad, 'echo reply id %s seq %s: %s' % (
        c.get('id'), c.get('seq'), '; '.join(bad) if bad else 'as RFC 792 and RFC 1122 3.2.2.6 ask'))


def _show(x):
    if isinstance(x, (bytes, bytearray)):
        return x.hex() if len(x) <= 8 else '%d octets %s...' % (len(x), x[:8].hex())
    if isinstance(x, int):
        return '%d (0x%X)' % (x, x)
    return repr(x)
