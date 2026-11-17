"""Self-test of eth_check.py and the Ethernet front end, on synthetic lines built from the standards.

The generators here share no code with the checker: the FCS is the procedure of IEEE 802.3-2005 3.2.8 done bit
by bit (the checker uses zlib.crc32), the bit order is 3.3 and 3.2.8, the preamble 4.2.5, the SFD 3.2.2, the
Manchester code 7.3.1.1 with 14.2.1.1's polarity, TP_IDL 14.3.1.2.1, link pulses 14.2.1.1 and 14.3.1.2.1, FLP
bursts 28.2.1.1.2, ARP RFC 826, IPv4 RFC 791 and ICMP echo RFC 792.

One good record must pass every check. Each bad record changes one thing and must fail exactly one check.
The front end (frontend.Rs485Eth) is then driven from DI and DE as the chip's pins would carry them, and its
line must pass the checker, and its RO must follow the receive pair.

The generators are also meant for the benches: frame(), arp(), echo(), manchester(), link_pulse(), flp().
"""
import random

import eth_check
import frontend

PROGRAMS = ('bench/eth_check.py',)

BT = 100.0                                  # ns (802.3 7.3.2)
POLY = 0x04C11DB7                           # 3.2.8, without the x^32 term
MAC = bytes.fromhex('020000000001')         # the chip (locally administered, 3.2.3 c)
IP = bytes([192, 168, 0, 2])
PEER_MAC = bytes.fromhex('525400123456')
PEER_IP = bytes([192, 168, 0, 1])
OTHER_MAC = bytes.fromhex('525400abcdef')


# ---------------------------------------------------------------------------------------------------------------
# frames, from the standard

def octet_bits(octets):
    """Each octet least significant bit first (3.3)."""
    return [(o >> b) & 1 for o in octets for b in range(8)]


def fcs_bits(bits):
    """802.3 3.2.8, literally: complement the first 32 bits, divide M(x) x^32 by G(x), complement the remainder,
    send it x^31 first. bits are the frame's bits in transmission order, DA to the end of the pad."""
    m = [b ^ 1 if i < 32 else b for i, b in enumerate(bits)]
    reg = 0
    for b in m + [0] * 32:                  # M(x) x^32: 32 zeros after the message
        top = (reg >> 31) & 1
        reg = ((reg << 1) | b) & 0xFFFFFFFF
        if top:
            reg ^= POLY
    crc = reg ^ 0xFFFFFFFF
    return [(crc >> (31 - i)) & 1 for i in range(32)]


def frame(dst, src, etype, payload, pad=True, bad_fcs=False):
    """DA, SA, Length/Type, data and pad to 46 octets with zeros (RFC 894), as bits with the FCS appended."""
    body = bytes(payload)
    if pad and len(body) < 46:
        body += bytes(46 - len(body))
    octs = bytes(dst) + bytes(src) + bytes([etype >> 8, etype & 255]) + body
    bits = octet_bits(octs)
    f = fcs_bits(bits)
    if bad_fcs:
        f[-1] ^= 1
    return bits + f


def preamble(n=56):
    """n bits of 1010... ending in 0 (4.2.5, 7.2.3.2: the last preamble bit is a CD0; 56 bits start with 1), then
    the SFD 10101011 (3.2.2)."""
    return [(n - 1 - i) % 2 for i in range(n)] + [1, 0, 1, 0, 1, 0, 1, 1]


def arp(op, sha, spa, tha, tpa, hrd=1, pro=0x0800):
    """RFC 826 packet: ar$hrd, ar$pro, ar$hln, ar$pln, ar$op (16-bit fields high byte first), then the addresses."""
    return (bytes([hrd >> 8, hrd & 255, pro >> 8, pro & 255, len(sha), len(spa), op >> 8, op & 255]) +
            bytes(sha) + bytes(spa) + bytes(tha) + bytes(tpa))


def csum(data):
    """RFC 791 / RFC 792: the one's complement of the one's complement sum of 16-bit words."""
    if len(data) % 2:
        data += b'\x00'
    s = sum((data[i] << 8) | data[i + 1] for i in range(0, len(data), 2))
    while s >> 16:
        s = (s & 0xFFFF) + (s >> 16)
    return (~s) & 0xFFFF


def echo(typ, src, dst, ident, seq, data, ttl=64, bad_icmp=False):
    """IPv4 header (RFC 791, protocol 1) and an ICMP echo message (RFC 792, type 8 request or 0 reply)."""
    msg = bytearray([typ, 0, 0, 0, ident >> 8, ident & 255, seq >> 8, seq & 255]) + bytes(data)
    c = csum(bytes(msg))
    msg[2], msg[3] = c >> 8, c & 255
    if bad_icmp:
        msg[3] ^= 1
    total = 20 + len(msg)
    hdr = bytearray([0x45, 0, total >> 8, total & 255, 0, 1, 0, 0, ttl, 1, 0, 0]) + bytes(src) + bytes(dst)
    c = csum(bytes(hdr))
    hdr[10], hdr[11] = c >> 8, c & 255
    return bytes(hdr) + bytes(msg)


def fragment(ip):
    """The datagram as the first fragment of a larger one: MF set (RFC 791 3.1), the header checksum again."""
    h = bytearray(ip[:20])
    h[6] |= 0x20
    h[10] = h[11] = 0
    c = csum(bytes(h))
    h[10], h[11] = c >> 8, c & 255
    return bytes(h) + bytes(ip[20:])


# ---------------------------------------------------------------------------------------------------------------
# lines

def manchester(bits, t0, bt=BT, tp_hi=3.0, tp_lo=None, jitter=None, glitch=None):
    """The pair for one frame: bit k's first half is the bit's complement and its second half the bit
    (7.3.1.1), HI = TD+ positive (14.2.1.1). Then TP_IDL (14.3.1.2.1): a positive transition at the boundary
    after a final 0, HI for tp_hi BT from the last positive crossing, an optional LO for tp_lo BT, silence.
    jitter: {half-bit index: ns} moves single transitions. glitch: (half-bit index, ns from its start, width)
    puts a short excursion to the other level inside that half bit. Returns (changes, end of the last bit)."""
    jitter = jitter or {}
    halves = []
    for b in bits:
        halves += [-1 if b else 1, 1 if b else -1]
    out, last = [], 0
    for h, lv in enumerate(halves):
        t = t0 + h * bt / 2
        if lv != last:
            out.append((t + jitter.get(h, 0.0), lv))
            last = lv
        if glitch and glitch[0] == h:
            g = t + glitch[1]
            out += [(g, -lv), (g + glitch[2], lv)]
    end = t0 + len(bits) * bt
    t_pos = end - bt / 2 if bits[-1] == 1 else end
    if bits[-1] == 0:
        out.append((end, 1))
    out.append((t_pos + tp_hi * bt, -1 if tp_lo else 0))
    if tp_lo:
        out.append((t_pos + (tp_hi + tp_lo) * bt, 0))
    return out, end


def link_pulse(t, width=100.0, lo=None):
    """A link test pulse: positive for width ns, optionally a negative tail of lo ns (14.3.1.2.1)."""
    if lo:
        return [(t, 1), (t + width, -1), (t + width + lo, 0)]
    return [(t, 1), (t + width, 0)]


def flp(t, word=0x01E1, width=100.0):
    """An FLP burst (28.2.1.1.2): 17 clock pulses 125 us apart, and a data pulse 62.5 us after each clock pulse
    for each 1 of the 16-bit link code word, bit 0 first. 0x01E1 advertises 10BASE-T and 100BASE-TX."""
    out = []
    for k in range(17):
        out += link_pulse(t + k * 125e3, width)
        if k < 16 and (word >> k) & 1:
            out += link_pulse(t + k * 125e3 + 62.5e3, width)
    return out


def merge(*parts):
    out = []
    for p in parts:
        out += p
    return sorted(out, key=lambda e: e[0])


# ---------------------------------------------------------------------------------------------------------------
# the scenario

def scenario(mod=None):
    """(tx, rx, t_start, t_end) of a good exchange, with one change named by mod."""
    mod = mod or ''
    tx, rx = [], []
    req = frame(eth_check.BROADCAST, PEER_MAC, 0x0806, arp(1, PEER_MAC, PEER_IP, bytes(6), IP))
    t = 5e6
    line, end = manchester(preamble() + req, t)
    rx += line
    # the reply: RFC 826's swap, padded with zeros, 20 us after the request
    sha, spa = MAC, IP
    if mod == 'arp_reply':
        spa = bytes([192, 168, 0, 3])
    reply = arp(2, sha, spa, PEER_MAC, PEER_IP)
    gap = 5e3 if mod == 'ipg' else 20e3
    kw = dict(pad=mod != 'frame_size', bad_fcs=mod == 'fcs')
    bits = preamble(50 if mod == 'preamble_sfd' else 56) + frame(PEER_MAC, MAC, 0x0806, reply, **kw)
    mk = dict(tp_hi=2.0 if mod == 'tp_idl' else 3.0)
    bt = BT
    if mod == 'edge_timing':
        mk['jitter'] = {401: 20.0}          # the mid-bit transition of bit 200: 10 ns from the best grid
    if mod == 'rate':
        bt = BT * (1 - 2e-4)
    if mod == 'manchester':
        k = next(i for i in range(300, 400) if bits[i] == 1 and bits[i + 1] == 0)
        mk['glitch'] = (2 * k + 1, 30.0, 10.0)
    t_rep = end + gap
    line, rend = manchester(bits, t_rep, bt=bt, **mk)
    tx += line
    t_idl = rend - BT / 2 if bits[-1] == 1 else rend
    if mod == 'collision':
        other = frame(OTHER_MAC, PEER_MAC, 0x0800, bytes(46))
        rx += manchester(preamble() + other, t_rep + 30e3)[0]
    # link pulses: one before the request, then 16 ms after the reply's TP_IDL
    tx += link_pulse(2e6, 60.0 if mod == 'link_pulse' else 100.0)
    p2 = t_idl + 16e6
    tx += link_pulse(p2)
    # frames that need no reply
    t = 25e6
    other_ip = bytes([192, 168, 0, 77])
    frag = echo(8, PEER_IP, IP, 0x1234, 7, bytes(range(24)))
    for f in (frame(MAC, PEER_MAC, 0x0800, fragment(frag)),      # reassembled before ICMP sees it (RFC 791 3.2)
              frame(eth_check.BROADCAST, PEER_MAC, 0x0806, arp(1, PEER_MAC, PEER_IP, bytes(6), other_ip)),
              frame(eth_check.BROADCAST, PEER_MAC, 0x0806, arp(1, PEER_MAC, PEER_IP, bytes(6), IP), bad_fcs=True),
              frame(eth_check.BROADCAST, PEER_MAC, 0x0806, arp(1, PEER_MAC, PEER_IP, bytes(6), IP), pad=False),
              frame(OTHER_MAC, PEER_MAC, 0x0806, arp(1, PEER_MAC, PEER_IP, bytes(6), IP)),
              frame(eth_check.BROADCAST, PEER_MAC, 0x0806, arp(2, PEER_MAC, PEER_IP, bytes(6), IP))):
        line, end = manchester(preamble() + f, t)
        rx += line
        t += 1e6
    if mod == 'fragment':
        # an echo reply to the lone fragment, which nothing may answer
        ans = frame(PEER_MAC, MAC, 0x0800, echo(0, IP, PEER_IP, 0x1234, 7, bytes(range(24))))
        tx += manchester(preamble() + ans, 25e6 + 100e3)[0]
    if mod == 'unsolicited':
        # an answer to the request for another IP, which RFC 826 says to drop
        ans = frame(PEER_MAC, MAC, 0x0806, arp(2, MAC, IP, PEER_MAC, PEER_IP))
        tx += manchester(preamble() + ans, 25e6 + 100e3)[0]
    if mod == 'link_period':
        p3 = p2 + 32e6
    else:
        p3 = p2 + 16e6
    tx += link_pulse(p3)
    # ping: an echo request and its reply
    data = bytes(range(32))
    q = frame(MAC, PEER_MAC, 0x0800, echo(8, PEER_IP, IP, 0x1234, 1, data))
    t = p3 + 4e6
    line, end = manchester(preamble() + q, t)
    rx += line
    r = frame(PEER_MAC, MAC, 0x0800, echo(0, IP, PEER_IP, 0x1234, 1, data, bad_icmp=mod == 'icmp_reply'))
    rb = preamble() + r
    line, rend = manchester(rb, end + 15e3)
    tx += line
    t_idl = rend - BT / 2 if rb[-1] == 1 else rend
    p4 = t_idl + 16e6
    tx += link_pulse(p4)
    # an auto-negotiating partner's FLP burst, then two more pulses
    rx += flp(p4 + 2e6)
    tx += link_pulse(p4 + 16e6) + link_pulse(p4 + 32e6)
    return merge(tx), merge(rx), 0.0, p4 + 40e6


BAD = ('manchester', 'edge_timing', 'rate', 'preamble_sfd', 'frame_size', 'fcs', 'tp_idl', 'link_pulse',
       'link_period', 'ipg', 'collision', 'arp_reply', 'unsolicited', 'icmp_reply', 'fragment')
EXPECT = dict(rate='edge_timing', unsolicited='arp_reply', fragment='icmp_reply')


def run(mod=None, **kw):
    tx, rx, t0, t1 = scenario(mod)
    return eth_check.check(tx, rx, mac=MAC, ip=IP, t_start=t0, t_end=t1, **kw)


# ---------------------------------------------------------------------------------------------------------------
# the front end, driven as the chip's pins would drive it

def pins_for(bits, c0, tail=15):
    """DI and DE changes (ns) for one frame as a shift unit sends it at 50 MHz: half bits of 2 and 3 cycles
    (TPER 2, TFRAC 128), output A (DI) is the 802.3 code, output B (DE) is 1 from the first symbol to the end
    of the tail, during which A is 1 (TP_IDL). Idle is A = 1, B = 0: DI rests high, so that a link pulse, which
    raises DE alone, starts positive (with DI and DE rising together, tZH < tDD shows the old DI first)."""
    di, de = [], [(c0 * 20.0, 1)]
    c = c0
    for b in bits:
        di.append((c * 20.0, 1 - b))
        c += 2
        di.append((c * 20.0, b))
        c += 3
    di.append((c * 20.0, 1))
    c += tail
    de.append((c * 20.0, 0))
    return di, de, c


def frontend_case(corner, **over):
    """A reply frame and two link pulses through the transceiver model; returns the checker's report."""
    fe = frontend.Rs485Eth(corner, **over)
    req = frame(eth_check.BROADCAST, PEER_MAC, 0x0806, arp(1, PEER_MAC, PEER_IP, bytes(6), IP))
    rx, end = manchester(preamble() + req, 1e6)
    reply = preamble() + frame(PEER_MAC, MAC, 0x0806, arp(2, MAC, IP, PEER_MAC, PEER_IP))
    c0 = int((end + 20e3) / 20.0)
    di, de, c_end = pins_for(reply, c0)
    for t in (0.2e6, c_end * 20.0 + 16e6):           # link pulses: B = 1 for 5 cycles (a tail of 5), A stays 1
        de += [(t, 1), (t + 100.0, 0)]
    di.sort()
    de.sort()
    tx = fe.tx_line(di, de, di0=1)
    return eth_check.check(tx, rx, mac=MAC, ip=IP, t_start=0.0, t_end=c_end * 20.0 + 20e6)


def main(log):
    log.group('bench/eth_check.py', title='10BASE-T checker self-test: synthetic lines from IEEE 802.3 and RFC 826')
    # the FCS procedure of 3.2.8, here bit by bit, against zlib's CRC-32 as the checker uses it
    rng = random.Random(1)
    ok = True
    for n in (4, 7, 46, 60, 300):              # 3.2.8 complements the first 32 bits: 4 octets at least
        data = bytes(rng.randrange(256) for _ in range(n))
        f = fcs_bits(octet_bits(data))
        octs = bytes(sum(f[8 * q + b] << b for b in range(8)) for q in range(4))
        ok &= int.from_bytes(octs, 'little') == eth_check.zlib.crc32(data)
    log.check('802.3 3.2.8 FCS, done bit by bit, equals zlib.crc32 on the octets as sent', ok)
    log.check('Figure 14-10 and 14-12 readings at 2.8 V', abs(eth_check.tp_idl_bounds(2.8)[1] - 4.6475) < 1e-3 and
              abs(eth_check.link_pulse_bounds(2.8)[1] - 1.375) < 1e-9, '%s %s' % (
                  eth_check.tp_idl_bounds(2.8), eth_check.link_pulse_bounds(2.8)))
    rep = run()
    counts = rep.checks()
    log.check('good record passes every check', rep.ok and all(counts[c][0] > 0 for c in eth_check.Report.CHECKS),
              ', '.join('%s %d' % (c, counts[c][0]) for c in eth_check.Report.CHECKS) +
              ('' if rep.ok else '; ' + repr(rep.failed()[:3])))
    log.check('good record: 2 frames sent, 8 received, 6 link pulses sent, an FLP burst of 22 pulses',
              (len(rep.tx_frames), len(rep.rx_frames), len(rep.tx_pulses), len(rep.rx_pulses)) == (2, 8, 6, 22),
              'tx %d rx %d tx pulses %d rx pulses %d' % (len(rep.tx_frames), len(rep.rx_frames), len(rep.tx_pulses),
                                                         len(rep.rx_pulses)))
    for mod in BAD:
        rep = run(mod)
        want = [EXPECT.get(mod, mod)]
        got = rep.failing_checks()
        log.check('bad record "%s" fails exactly %s' % (mod, want[0]), got == want,
                  'fails %s: %s' % (got, rep.failed()[:2]))
    # one unanswered echo request is one failure, blamed on it: a reply that names another request is never taken
    # for this one
    q1 = frame(MAC, PEER_MAC, 0x0800, echo(8, PEER_IP, IP, 0x1234, 1, bytes(8)))
    q2 = frame(MAC, PEER_MAC, 0x0800, echo(8, PEER_IP, IP, 0x1234, 2, bytes(8)))
    r2 = frame(PEER_MAC, MAC, 0x0800, echo(0, IP, PEER_IP, 0x1234, 2, bytes(8)))
    rx1, e1 = manchester(preamble() + q1, 1e6)
    rx2, e2 = manchester(preamble() + q2, e1 + 300e3)
    tx2 = manchester(preamble() + r2, e2 + 20e3)[0]
    rep = eth_check.check(tx2, merge(rx1, rx2), mac=MAC, ip=IP)
    f = rep.failed()
    log.check('one of two echo requests left unanswered: exactly one failure, for it, and the other\'s reply passes',
              len(f) == 1 and f[0][1] == 'icmp_reply' and 'seq 1' in f[0][2] and rep.checks()['icmp_reply'] == (1, 1),
              f)
    # 7.5.2.2's receive test signal: 47 preamble bits, random jitter up to 12 ns on preamble transitions and
    # 18 ns on frame transitions. The checker must decode it, since the benches send it to the chip.
    rng = random.Random(7)
    req = frame(eth_check.BROADCAST, PEER_MAC, 0x0806, arp(1, PEER_MAC, PEER_IP, bytes(6), IP))
    bits = preamble(47) + req
    good = 0
    for k in range(20):
        jit = {h: rng.uniform(-12.0, 12.0) if h < 2 * 55 else rng.uniform(-18.0, 18.0) for h in range(2 * len(bits))}
        line, _ = manchester(bits, 1e6, jitter=jit)
        r = eth_check.check([], line, mac=MAC, ip=IP)
        f = r.rx_frames
        good += len(f) == 1 and f[0].octets is not None and f[0].fields.get('fcs_ok') and f[0].jitter > 10
    log.check('20 received frames with 802.3 7.5.2.2 jitter (47-bit preamble, 12 and 18 ns) decode with good FCS',
              good == 20, '%d of 20' % good)
    for corner in ('typ', 'max', 'min'):
        rep = frontend_case(corner)
        log.check('front end, %s corner: DI and DE from a shift unit give a line that passes' % corner, rep.ok,
                  rep.failed()[:3] if not rep.ok else 'TP_IDL %s; pulses %s' % (
                      [round((f.stop - f.t_idl) / BT, 2) for f in rep.tx_frames],
                      [round(p.width, 1) for p in rep.tx_pulses]))
    # FN6307 gives no minimum enable or disable time: a part that enables slowly and disables fast makes a 5-cycle
    # DE pulse 50 ns on the pair, under Figure 14-12's 85 ns
    rep = frontend_case('typ', t_zh=50.0, t_zl=50.0, t_hz=0.0, t_lz=0.0)
    log.check('front end, slow enable and fast disable: 5 cycles of DE give a 50 ns pulse, which link_pulse rejects',
              rep.failing_checks() == ['link_pulse'] and [round(p.width, 1) for p in rep.tx_pulses] == [50.0, 50.0],
              '%s %s' % (rep.failing_checks(), [round(p.width, 1) for p in rep.tx_pulses]))
    line = frontend.Rs485Eth('typ').tx_line([(1000.0, 1), (1100.0, 0)], [(1000.0, 1), (1100.0, 0)])
    log.check('front end: DI and DE rising together show the old DI for tDD - tZH first (a LO glitch)',
              line[:2] == [(1017.0, -1), (1027.5, 1)], line)
    # a record in the benches' layout: DI on uo4 and DE on uo5, as the chip's pads would carry them
    rec_t = [0, 10, 12, 15]
    rec_v = [0, (1 << 4), (1 << 4) | (1 << 5), (1 << 5)]
    di, de = frontend.pad(rec_t, rec_v, 'uo4'), frontend.pad(rec_t, rec_v, 'uo5')
    log.check('front end: pad() reads uo4 and uo5 from a recorded run as DI and DE changes',
              di == [(0.0, 0), (200.0, 1), (300.0, 0)] and de == [(0.0, 0), (240.0, 1)], '%s %s' % (di, de))
    # receive: RO is 1 on an idle line, 0 while the pair is HI, after tPHL and tPLH (FN6307)
    fe = frontend.Rs485Eth('typ')
    line = [(0.0, -1), (50.0, 1), (100.0, -1), (150.0, 1), (450.0, 0)]
    ro = fe.ro(line)
    want = [(50.0 + 33.75, 0), (100.0 + 32.25, 1), (150.0 + 33.75, 0), (450.0 + 32.25, 1)]
    log.check('front end RO: idle 1, low while HI, tPHL 33.75 and tPLH 32.25 ns at typ', ro == want, ro)
    fe2 = frontend.Rs485Eth('typ', rx_invert=False)
    log.check('front end RO with RD+ on A: a positive link pulse doesn\'t show', fe2.ro(link_pulse(0.0)) == [],
              fe2.ro(link_pulse(0.0)))
    s = frontend.Sampler(ro, 1, phase_ns=5.0)
    log.check('Sampler gives the level at each clock edge', [s(c) for c in range(0, 30, 3)] ==
              [1, 1, 0, 0, 0, 0, 0, 0, 1, 1], [s(c) for c in range(0, 30, 3)])
