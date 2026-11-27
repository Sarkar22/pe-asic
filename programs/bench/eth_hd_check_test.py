"""Self-test of eth_hd_check.py on synthetic lines built from the standard with eth_check_test's generators.

One good record must pass every check: an attempt cut by a collision in its preamble, with the header and a 32-bit jam
(4.2.8), a backoff of one slot time (4.2.3.2.5), then the whole frame, answering an ARP request. Each bad record
changes one thing and must fail the check named for it.
"""
import eth_check_test as G
import eth_hd_check as H

PROGRAMS = ('bench/eth_hd_check.py',)

BT = 100.0
MAC, IP = G.MAC, G.IP
JAM = [1, 0] * 16                                   # 32 jam bits; content is the station's choice (4.2.3.2.4)


def reply_bits():
    return G.frame(G.PEER_MAC, MAC, 0x0806, G.arp(2, MAC, IP, G.PEER_MAC, G.PEER_IP))


def request_bits():
    return G.frame(b'\xff' * 6, G.PEER_MAC, 0x0806, G.arp(1, G.PEER_MAC, G.PEER_IP, bytes(6), IP))


def burst(bits, t0):
    ch, end = G.manchester(G.preamble(56) + bits, t0)
    return ch, end


def record(case='good'):
    """(tx, rx, t_end) for a case. Times in ns."""
    tx, rx = [], []
    req, end_req = burst(request_bits(), 100e3)
    rx += req
    rep = reply_bits()
    t0 = end_req + 110 * BT                          # our first attempt, after the gap
    off = 20                                         # the partner's fragment, BT into it
    att = list(JAM)
    if case == 'short_jam':
        att = JAM[:16]
    if case == 'crc_jam':                            # 20 octets, then their own CRC: the jam 4.2.3.2.4 forbids
        body = rep[:160]
        att = body + G.fcs_bits(body)
        off = 200
    if case in ('long_jam', 'not_jammed'):
        off = 200
        att = rep[:int(off - 64 + 80)] if case == 'long_jam' else rep
    if case == 'no_coll':
        off = None
    if case == 'other_frame':                        # cut 40 BT after a collision at 200 BT: 17 octets of the frame
        off = 200
        att = rep[:200 - 64 + 40]
    if case == 'early_end':                          # cut 20 BT after a collision at 200 BT: far more than 32 bits
        off = 200                                    # after the SFD, but under 32 after the collision (4.2.3.2.4)
        att = rep[:200 - 64 + 20]
    ch, end1 = burst(att, t0)
    tx += ch
    if off is not None:
        fr, fend = burst([1, 0] * 16, t0 + off * BT)
        rx += fr
    else:
        rx += G.link_pulse(t0 + 30 * BT)
        fend = end1
    last = max(end1, fend)
    r = 2 if case == 'bad_backoff' else 1
    gap = 50 * BT if case == 'short_gap' else 150 * BT
    t1 = end1 + r * H.SLOT + gap
    if case == 'short_gap':
        t1 = last + gap
    if case == 'busy':                               # our retry starts 60 BT into a partner frame
        other, _ = burst(G.frame(G.OTHER_MAC, G.PEER_MAC, 0x88B5, bytes(46)), t1 - 60 * BT)
        rx += other
    if case == 'other_frame':                        # not the retry: another frame
        rep2 = G.frame(G.OTHER_MAC, MAC, 0x0806, G.arp(2, MAC, IP, G.OTHER_MAC, G.PEER_IP))
        ch, end2 = burst(rep2, t1)
    elif case == 'over_limit':                       # 16 cut attempts, then a 17th
        ch, end2 = [], t1
        tcur = t1
        for n in range(16):
            c2, e2 = burst(list(JAM) if n < 15 else rep, tcur)
            ch += c2
            fr, fe = burst([1, 0] * 16, tcur + 20 * BT)
            if n < 15:
                rx += fr
            tcur = max(e2, fe) + 150 * BT if n < 15 else e2
            end2 = e2
    else:
        ch, end2 = burst(rep, t1)
    tx += ch
    return G.merge(tx), G.merge(rx), end2 + 200e3


CASES = [('short_jam', 'jam'), ('early_end', 'jam'), ('long_jam', 'jam'), ('crc_jam', 'jam'),
         ('not_jammed', 'collision'),
         ('no_coll', 'jam'), ('bad_backoff', 'backoff'), ('short_gap', 'deference'), ('busy', 'deference'),
         ('other_frame', 'retry'), ('over_limit', 'attempts')]


def single_cycle():
    """A whole reply with one cycle of 2.5 MHz on RD in the middle (100 ns HI, 300 ns LO): no RD activity for a
    receiver (14.3.1.3.2 c), so no collision."""
    req, end_req = burst(request_bits(), 100e3)
    tx, end = burst(reply_bits(), end_req + 110 * BT)
    rx = req + [(end_req + 400 * BT, 1), (end_req + 401 * BT, -1), (end_req + 404 * BT, 0)]
    return G.merge(tx), G.merge(rx), end + 200e3


def judge(case):
    tx, rx, t_end = record(case)
    return H.check(tx, rx, mac=MAC, ip=IP, t_start=0.0, t_end=t_end)


def main(log):
    log.group(*PROGRAMS, title='half-duplex checker self-test: synthetic lines from the standard')
    rep = judge('good')
    n = rep.checks()
    log.check('a good record (a jam after a preamble collision, one slot of backoff, the whole frame) passes every '
              'check', rep.ok and n.get('jam', (0, 0))[0] >= 2 and n.get('backoff', (0, 0))[0] == 1,
              rep.summary()[:300] if not rep.ok else ', '.join('%s %d' % (k, v[0]) for k, v in n.items() if v[0]))
    tx, rx, t_end = single_cycle()
    rep = H.check(tx, rx, mac=MAC, ip=IP, t_start=0.0, t_end=t_end)
    log.check('a whole frame with a single cycle on RD during it (14.3.1.3.2 c: not RD activity) passes every check',
              rep.ok and rep.checks()['collision'] == (0, 0), rep.summary()[:300] if not rep.ok else 'no collision')
    for case, want in CASES:
        rep = judge(case)
        fails = rep.failing_checks()
        log.check('the bad record "%s" fails %s' % (case, want), want in fails, 'fails %s' % (fails or 'nothing'))
