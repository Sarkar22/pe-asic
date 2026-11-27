"""Half-duplex 10BASE-T checker: the CSMA/CD rules of IEEE 802.3 clause 4 on a station's transmit pair, judged against
the receive pair, written from the clauses it cites, after the first version of advanced/eth_hd.s (unlike eth_check.py,
which came before any program).

Sources: IEEE Std 802.3 4.2.3.2.1 (deference), 4.2.3.2.3 to 4.2.3.2.5 (collision handling, the jam, truncated binary
exponential backoff), 4.2.8 (TransmitLinkMgmt, WatchForCollision, BackOff, Deference, PhysicalSignalEncap, StartJam),
4.2.9 (a receiver truncates a frame to whole octets before its FCS check), 4.4.2 (slotTime 512 BT, interPacketGap
96 bits, attemptLimit 16, backoffLimit 10, jamSize 32 bits), 14.2.1.1 (DO to TD within 2 BT), 14.2.1.4 (CI within
9 BT of a collision), 14.2.1.7 and 14.2.3.1 (a link test pulse is not RD activity), Table 14-2, and the delay budget of
Annex B.1.2 (D3, D7, D9, D11, D12). Python 3 standard library only.

The lines are those of eth_check: (time_ns, level) changes, +1 HI, -1 LO, 0 silence. eth_check decodes every burst;
this module adds what clause 4 asks of a transmitter that shares the medium, and replaces eth_check's 'ipg' and
'collision' checks, which assume a station that never collides.

    import eth_hd_check
    rep = eth_hd_check.check(tx_line, rx_line, mac=..., ip=..., t_start=..., t_end=...)
    rep.ok, rep.failed(), rep.checks()           as eth_check's Report
    rep.attempts                                  every transmitted burst, in time order, with .t_occ, .jammed, .late
    rep.groups                                    the attempts of each frame: (attempts, outcome)
    rep.backoffs                                  (n, r, slack_ns) before each retransmission

A transmitted burst is a whole frame (64 octets or more of whole octets after the SFD, its FCS right) or an attempt
that a collision cut: its jam. The partner's frames, fragments included, are the RD activity; its link test pulses are
not (14.2.3.1), nor is a single cycle, one HI and one LO, which a receiver rejects (14.3.1.3.2 c). t_occ is when both
pairs are first active together: the later of the two starts.

Checks, besides eth_check's per-frame, pulse, period and reply checks (frame_size and fcs only for whole frames;
replies only from whole frames):
  deference  each attempt starts at least interPacketGap (96 BT) after the end of the last frame on either pair, its
             own pair's included (4.2.3.2.1, 4.4.2); a partner frame in progress when it starts began at most 47 BT
             before: the last third of the gap, 32 BT, which 4.2.3.2.1 says carrier must not restart, plus the MAU's
             receive startup (Table 14-2 M1, 8 BT), the DTE's carrier detect (B.1.2 D3, 3 BT) and the gap's end to
             output (B.1.2 D9, at most 4 BT over 96)
  collision  a whole frame overlapped by partner activity: allowed only if that activity came within 14 BT of the
             frame's end, too late for a station to see in time: CI 9 BT (14.2.1.4), SQE to SIGNAL STATUS 3 BT
             (B.1.2 D7), DO to TD 2 BT (14.2.1.1)
  jam        an attempt cut short: there is partner activity (14.2.1.4); at least 32 bits follow its SFD and it
             ends at least 32 BT after t_occ: the header always goes out, then jamSize bits after a collision that
             detection can't precede (4.2.8 PhysicalSignalEncap and StartJam, 4.2.3.2.4); it ends no later than
             max(SFD + 34 BT, t_occ + 62 BT), the SFD's last bit marking the header's end (14.2.1.1 lets the first
             bits be lost): 9 BT of CI (14.2.1.4), 3 BT (D7), 16 BT to the jam (D11), 32 BT of jam (D12), 2 BT DO
             to TD (14.2.1.1); each time bound allows one edge's 8.5 ns (7.5.2.1, 14.3.1.2.3); and its whole octets
             after the SFD don't end in a valid FCS (4.2.3.2.4, 4.2.9)
  retry      each attempt of a frame carries the same frame: an attempt's octets that went out before its collision
             equal the frame's (4.2.3.2.5: the frame is retried; 4.2.8 StartTransmit starts it from bit 1); an
             attempt that a collision cut is followed by its retry, not another frame, unless that collision was late
             (4.2.8 lets a station at 10 Mb/s end its attempts then)
  backoff    before attempt n + 1, the delay from the end of attempt n is r slot times plus the gap, r =
             floor(delay / 512 BT), with 0 <= r < 2^min(n, 10) (4.2.3.2.5, 4.2.8 BackOff; a station may wait longer,
             never less)
  attempts   no frame has more than attemptLimit (16) attempts (4.2.8 TransmitLinkMgmt, 4.4.2)
"""
import zlib

import eth_check as E

BT = 100.0
HDR = 64 * BT               # 4.2.8 headerSize: preamble and SFD
SLOT = 512 * BT             # 4.4.2 slotTime
IPG = 96 * BT               # 4.4.2 interPacketGap
JAM = 32 * BT               # 4.4.2 jamSize
LATE_JAM = 62 * BT          # 9 + 3 + 16 + 32 + 2 BT: 14.2.1.4, B.1.2 D7, D11, D12, 14.2.1.1
DO_TD = 2 * BT              # 14.2.1.1: DO to TD steady-state delay
SEE = 14 * BT               # 9 + 3 + 2 BT: 14.2.1.4, B.1.2 D7, 14.2.1.1
P2 = 47 * BT                # 32 + 8 + 3 + 4 BT: 4.2.3.2.1, Table 14-2 M1, B.1.2 D3 and D9
ATTEMPT_LIMIT = 16          # 4.4.2
BACKOFF_LIMIT = 10          # 4.4.2
EPS = 1.0                   # ns of slack on every bound, for float times

CHECKS = ('manchester', 'edge_timing', 'preamble_sfd', 'frame_size', 'fcs', 'tp_idl', 'link_pulse', 'link_period',
          'deference', 'collision', 'jam', 'retry', 'backoff', 'attempts', 'arp_reply', 'icmp_reply')


class Report(E.Report):
    CHECKS = CHECKS

    def __init__(self):
        E.Report.__init__(self)
        self.attempts, self.groups, self.backoffs = [], [], []


def fcs_valid(octs):
    """True if whole octets end in the CRC-32 of those before them (3.2.8), as a receiver checks after 4.2.9's
    truncation."""
    return octs is not None and len(octs) >= 5 and zlib.crc32(octs[:-4]) == int.from_bytes(octs[-4:], 'little')


def _end(fr):
    return E._data_end(fr)


def _whole(fr, o):
    return (fr.octets is not None and fr.extra_bits == 0 and o['min_frame'] <= len(fr.octets) <= o['max_frame']
            and fcs_valid(fr.octets))


def _sfd_end(fr):
    """The end of the SFD's last bit cell: the header's end on the bit grid. The first bits on the pair may be lost or
    malformed (14.2.1.1: up to 2 bits), so the header is timed from its end, not from the burst's first level. With no
    SFD decoded, the burst's start plus 64 BT less those 2."""
    if fr.sfd_at is not None and fr.sfd_at < len(fr.mids):
        return fr.mids[fr.sfd_at] + BT / 2
    return fr.start + HDR - 2 * BT


def _safe_octets(fr):
    """The octets of an attempt that went out before its collision began, so before any jam (the jam can't start
    before collisionDetect, which can't precede the collision)."""
    if fr.octets is None:
        return b''
    if fr.t_occ is None:
        return fr.octets
    # bit k after the SFD occupies [sfd_end + k BT, sfd_end + (k + 1) BT)
    n = int((fr.t_occ - _sfd_end(fr)) // (8 * BT))
    return fr.octets[:max(0, min(n, len(fr.octets)))]


def _same_frame(a, b):
    """True if attempts a and b agree on every octet both sent before their collisions."""
    sa, sb = _safe_octets(a), _safe_octets(b)
    n = min(len(sa), len(sb))
    return sa[:n] == sb[:n]


def check(tx, rx, mac=None, ip=None, t_end=None, t_start=None, **opts):
    o = dict(E.DEFAULTS)
    o.update(opts)
    rep = Report()
    rep.tx_frames, rep.tx_pulses = E._classify(tx, 'tx', o, rep)
    rep.rx_frames, rep.rx_pulses = E._classify(rx if rx is not None else [], 'rx', o, rep)
    # RD activity: the partner's frames and fragments, not link pulses (14.2.3.1) nor a single cycle, which a receiver
    # must reject (14.3.1.3.2 c): a burst with fewer than 3 sign changes inside it
    partner = [f for f in rep.rx_frames if len(f.inner) >= 3]
    # each attempt: its collision, if any, and whether a collision cut it
    for fr in rep.tx_frames:
        fr.t_occ, fr.coll = None, None
        for p in partner:
            if p.start < _end(fr) and _end(p) > fr.start:
                t = max(p.start, fr.start)
                if fr.t_occ is None or t < fr.t_occ:
                    fr.t_occ, fr.coll = t, p
        fr.whole = _whole(fr, o)
        fr.jammed = not fr.whole
        fr.late = fr.t_occ is not None and fr.t_occ - fr.start > SLOT
    rep.attempts = list(rep.tx_frames)
    rep.partner = partner
    # eth_check's per-frame checks; an attempt cut by a collision is no frame, so frame_size and fcs judge whole ones
    for fr in rep.tx_frames:
        sub = E.Report()
        E._check_frame(fr, o, sub)
        for f in sub.findings:
            if fr.jammed and f.check in ('frame_size', 'fcs'):
                continue
            rep.findings.append(f)
    for fr in rep.rx_frames:
        E._report_rx(fr, o, rep)
    E._check_pulses(rep, o)
    E._check_period(rep, o, t_start, t_end)
    _check_deference(rep)
    _check_collisions(rep, o)
    _check_attempts(rep)
    if mac is not None and ip is not None:
        allf = rep.tx_frames
        rep.tx_frames = [f for f in allf if f.whole]
        E._check_replies(rep, bytes(mac), bytes(ip), o)
        rep.tx_frames = allf
    rep.findings.sort(key=lambda f: f.t)
    return rep


def _check_deference(rep):
    bursts = rep.tx_frames + rep.partner
    for fr in rep.tx_frames:
        before = [_end(b) for b in bursts if b is not fr and _end(b) <= fr.start]
        if before:
            d = fr.start - max(before)
            rep.add(fr.start, 'deference', d >= IPG - EPS, '%.2f us from the end of the last frame on either pair to '
                    'this attempt (at least %.1f us, 4.2.3.2.1, 4.4.2)' % (d / 1e3, IPG / 1e3))
        busy = [p for p in rep.partner if p.start < fr.start < _end(p)]
        for p in busy:
            d = fr.start - p.start
            rep.add(fr.start, 'deference', d <= P2 + EPS, 'started %.1f BT into a partner frame (at most %.0f BT: the '
                    'last third of the gap, 4.2.3.2.1, and the receive and carrier delays)' % (d / BT, P2 / BT))


def _check_collisions(rep, o):
    for fr in rep.tx_frames:
        end = _end(fr)
        if fr.whole:
            if fr.t_occ is None:
                continue
            ok = fr.t_occ > end - SEE - EPS
            rep.add(fr.t_occ, 'collision', ok, 'a whole frame %.1f to %.1f us with partner activity from %.1f us, '
                    '%.1f BT before its end (%s %.0f BT: 14.2.1.4, B.1.2 D7, 14.2.1.1)' % (
                        fr.start / 1e3, end / 1e3, fr.t_occ / 1e3, (end - fr.t_occ) / BT,
                        'seen too late at under' if ok else 'NOT JAMMED: a station sees it by', SEE / BT))
            continue
        if fr.t_occ is None:
            rep.add(fr.start, 'jam', False, 'an attempt cut short with no partner activity (%s octets, %d bits '
                    'over): a jam with no collision (14.2.1.4, 14.2.1.7)' % (
                        len(fr.octets) if fr.octets is not None else 'no', fr.extra_bits))
            continue
        hdr = _sfd_end(fr)
        nj = len(fr.bits) - fr.sfd_at - 1 if fr.sfd_at is not None else 0
        tol = o['jitter_tx']                    # one edge's allowed deviation (7.5.2.1, 14.3.1.2.3)
        hi = max(hdr + JAM + DO_TD, fr.t_occ + LATE_JAM)
        ok = nj >= 32 and end >= fr.t_occ + JAM - tol and end <= hi + tol
        rep.add(end, 'jam', ok, 'attempt %.1f to %.1f us, collision from %.1f us (%.1f BT in): %d bits after the SFD '
                '(at least 32), ends %.1f BT after the collision (%.0f to %.1f BT: 4.2.3.2.4, 4.2.8, 14.2.1.4, '
                'B.1.2)' % (fr.start / 1e3, end / 1e3, fr.t_occ / 1e3, (fr.t_occ - fr.start) / BT, nj,
                            (end - fr.t_occ) / BT, JAM / BT, (hi - fr.t_occ) / BT))
        bad = fcs_valid(fr.octets)
        rep.add(end, 'jam', not bad, 'the attempt\'s %s whole octets after the SFD %s in a valid FCS (4.2.3.2.4, '
                '4.2.9)' % (len(fr.octets) if fr.octets is not None else 0, 'END' if bad else 'do not end'))


def _check_attempts(rep):
    """Groups the attempts of each frame, then judges the retries, the backoffs and the attempt limit."""
    att = rep.tx_frames
    i = 0
    while i < len(att):
        grp = [att[i]]
        while grp[-1].jammed and i + len(grp) < len(att) and _same_frame(grp[-1], att[i + len(grp)]):
            grp.append(att[i + len(grp)])
        i += len(grp)
        last = grp[-1]
        outcome = 'sent' if last.whole else ('dropped' if len(grp) == ATTEMPT_LIMIT else
                                             'given up after a late collision' if last.late else 'unfinished')
        rep.groups.append((grp, outcome))
        if outcome == 'unfinished' and i < len(att):
            rep.add(att[i].start, 'retry', False, 'an attempt cut by a collision %.1f BT in, then another frame, not '
                    'its retry (4.2.3.2.5: retried until sent or attemptLimit; only a late collision may end it, '
                    '4.2.8)' % ((last.t_occ - last.start) / BT if last.t_occ is not None else -1))
        if len(grp) > 1:
            ok = all(_same_frame(a, grp[-1]) and _same_frame(a, grp[0]) for a in grp)
            rep.add(grp[0].start, 'retry', ok, '%d attempts of one frame: %s' % (
                len(grp), 'each carries the same octets before its collision' if ok else 'THEY DIFFER'))
        rep.add(grp[0].start, 'attempts', len(grp) <= ATTEMPT_LIMIT, '%d attempts, %s (at most %d, 4.4.2)' % (
            len(grp), outcome, ATTEMPT_LIMIT))
        for n in range(1, len(grp)):
            a, b = grp[n - 1], grp[n]
            d = b.start - _end(a)
            r = int(d // SLOT)
            k = min(n, BACKOFF_LIMIT)
            rep.backoffs.append((n, r, d - r * SLOT))
            rep.add(b.start, 'backoff', 0 <= r < (1 << k), 'before attempt %d: %.1f BT after the jam, r = %d, from 0 '
                    'to %d (4.2.3.2.5)' % (n + 1, d / BT, r, (1 << k) - 1))
