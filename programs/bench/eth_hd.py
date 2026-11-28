"""Bench for advanced/eth_hd.s: the half-duplex 10BASE-T node, with collisions, against a partner that reacts to it.

The partner is a line, as in eth.py, plus a plan: for our k-th transmission (a burst whose DE stays high, so not a
link test pulse) it may start a burst of its own a chosen number of bit times after ours began on the pair. A
'frag' is what a CSMA/CD station sends when it collides: 56 preamble bits, the SFD and a 32-bit jam (4.2.8: the
header, then jamSize bits; B.1.5's DTE minimum transmit length, 96 bits). A 'frame' is a whole frame, as from a station
that doesn't defer, such as one set to full duplex (14.2.1.8's mismatch). A partner that collides on every attempt
makes the node drop the frame after attemptLimit attempts (4.4.2).

eth_hd_check judges the record (deference, jam, retries, backoff, attempt limit) and eth_check's per-frame, pulse
and reply checks; the firmware's counters must match what the line shows. The backoff's spread is measured over many
trials: r for the first retransmission should be 0 or 1 with equal odds, for the second 0 to 3, and so on.

The link integrity test and the jabber function (802.3 14.2.1.6, 14.2.1.7) are judged by eth_link_check, held to the
values the program states (LINK_STATED, JAB_STATED, its PICS answers) and, for the link, also to 802.3's ranges: the
checker's own partner cases (eth.link_line) with silences of 95 ms, past this program's link_loss; partners whose
requests and pulses fall just inside and just outside each stated value (eth.probe_line); the end of LINK TEST FAIL
EXTEND against the next frame and against link_test_max running out, and a request whose start T counts as the second
link test pulse; and a broken copy whose feed loop never ends, which the jabber function must cut and hold. A frame
received while the link is down must be taken unstored and counted in NBUSY. These runs are long (up to 1,150 ms of
line), so they go 4 at a time.
"""
import bisect
import concurrent.futures
import random

import eth
import eth_check_test as G
import eth_hd_check as H
import frontend
from report import Chip, assemble, src

PROGRAMS = ('advanced/eth_hd.s',)
PROG = 'advanced/eth_hd.s'
WORDS = 440                     # section 11.2's count for the program
BT = 100.0
CYCLE = 20.0
MAC, IP = eth.MAC, eth.IP
TWIN_MAC = MAC[:4] + bytes([0x5A, 0x3C])   # a second station's MAC: the same but for its last two octets
DI, DE, RO = 4, 5, 4
MCFG = 4
MEM = dict(HANDED=11, RELEASED=12, NKEEP=7, NBAD=8, NOTHER=9, NBUSY=10, NCOLL=17, NLATE=18, NXS=19, LST=13, LW=14)
VARIABLE = set(range(5, 20))    # counters and state words the program writes
MS = 1e6
# the program's own values for the link integrity test and jabber (its header), as the checker takes them (the PICS
# answers LF1, LF5 to LF7, LF16, JF2, JF4): a timer of k TB wraps runs out after (k - 1) to k wraps of 1.31072 ms, give
# or take T's latency in seeing a wrap or a pulse, at most a slot of a backoff or of the unjab wait (2,560 cycles,
# 51.2 us) where a timer can run out; T's longer gaps between samples of TB, up to 6,540 cycles, come in sends, which
# follow a restart of the link timer by under 53 ms
LINK_STATED = dict(link_loss=(86.4 * MS, 87.9 * MS), link_test_min=(3.86 * MS, 5.31 * MS),
                   link_test_max=(86.4 * MS, 87.9 * MS), lc_max=(2, 2))
JAB_STATED = dict(xmit_max=(52.0 * MS, 105.5 * MS), unjab=(499.0 * MS, 501.0 * MS))


def frag_bits(rng):
    """The partner's own collided attempt: the header, then 32 jam bits."""
    return G.preamble(56) + [rng.getrandbits(1) for _ in range(32)]


class Partner:
    """The partner's pair: a static line, and bursts it starts at an offset into our transmissions.

    plan: list indexed by our transmission number (0 first), each None, (offset_bt, kind[, bits]) or a list of them:
    kind 'frag', 'frame' (bits: the frame's bits after the SFD, an ARP request by default) or 'pulse' (bits: width
    and LO tail in ns). jitter as eth.Line's."""

    def __init__(self, static=None, plan=(), seed=1, jitter=None):
        self.changes = list(static or [])
        self.plan = list(plan)
        self.rng = random.Random(seed)
        self.jitter = jitter
        self.sent = []                  # (start, kind) of each reactive burst
        self.ours = []                  # start of each of our transmissions, on the pair

    def burst(self, t0, kind, bits=None):
        L = eth.Line(seed=self.rng.randrange(1 << 30), jitter=self.jitter)
        if kind == 'pulse':                 # a link test pulse: bits is (width ns, LO tail ns or None)
            self.sent.append((t0, kind))
            return G.link_pulse(t0, *(bits or (100.0, None)))
        if kind == 'frag':
            allbits = frag_bits(self.rng)
            ch, end = G.manchester(allbits, t0)
        else:
            L.frame(bits if bits is not None else eth.arp_req(spa=bytes([192, 168, 0, 40 + len(self.sent)])), t0)
            ch = L.changes
        self.sent.append((t0, kind))
        return ch

    def on_tx(self, t_tx):
        """Our transmission number len(self.ours) began on the pair at t_tx: returns the changes to add."""
        k = len(self.ours)
        self.ours.append(t_tx)
        out = []
        if k < len(self.plan) and self.plan[k] is not None:
            for p in (self.plan[k] if isinstance(self.plan[k], list) else [self.plan[k]]):
                out += self.burst(t_tx + p[0] * BT, p[1], p[2] if len(p) > 2 else None)
        return out


class Result:
    pass


def run(partner, t_end, corner='typ', phase=7.5, align=0, threads=(0, 1), seed=1, text=None, chip_cls=None,
        duty=0.0, half=True, check=True, mac=None, link=None, jabber=None):
    """Runs the program against a partner to t_end ns, both halves of every cycle (section 3.1). mac: the station's
    MAC, which the host posts (MAC by default). link and jabber go to the checker (eth_link_check, 802.3 14.2.1.6,
    14.2.1.7)."""
    mac = MAC if mac is None else bytes(mac)
    img = assemble(text if text is not None else src(PROG), depth=512)
    fe = frontend.Rs485Eth(corner)
    t_en = max(fe.p['t_zh'], fe.p['t_zl'])
    line = G.merge(partner.changes)
    sched, sched_f = _sched(fe, line, phase, duty)
    i = j = 0
    lv = lvf = 1
    cycles = int(t_end / CYCLE)
    chip = (chip_cls or Chip)(512, seed)
    for a, w in img.words.items():
        chip.host_wrmem(a, w)
    tr, tt = threads
    chip.host_setpc(tr, img.labels['rx'])
    chip.host_setpc(tt, img.labels['tx'])
    cfg = eth.wire_words(mac) + eth.wire_words(IP)
    rec_t, rec_v = [], []
    last = None
    started = False
    de_rise = None                      # half cycle DE last rose, while it may still be a link pulse
    for c in range(cycles):
        if not started and (c - 1) % 4 == align:
            chip.host_run(0, (1 << tr) | (1 << tt))
            started = True
        if cfg and started and not chip.FULL[MCFG]:
            chip.host_post(MCFG, cfg.pop(0))
        while i < len(sched) and sched[i][0] <= c:
            lv = sched[i][1]
            i += 1
        while j < len(sched_f) and sched_f[j][0] <= c:
            lvf = sched_f[j][1]
            j += 1
        ui = 0x01 | (lv << RO)
        uo = chip.pads(ui)[0]
        uo2 = chip.pads2(ui)[0]
        for h, w in ((2 * c, uo), (2 * c + 1, uo2)):
            if w != last:
                rec_t.append(h)
                rec_v.append(w)
                if (w >> DE) & 1 and not (last is not None and (last >> DE) & 1):
                    de_rise = h
                elif not (w >> DE) & 1:
                    de_rise = None
                last = w
        # a DE high for 8 cycles is a transmission, not a link pulse (5 cycles): tell the partner
        if de_rise is not None and 2 * c - de_rise >= 16:
            t_tx = (de_rise >> 1) * CYCLE + ((CYCLE / 2 + duty) if de_rise & 1 else 0.0) + t_en
            de_rise = None
            add = partner.on_tx(t_tx)
            if add:
                line = G.merge(line, add)
                sched, sched_f = _sched(fe, line, phase, duty)
                i = bisect.bisect_right([s[0] for s in sched], c)
                j = bisect.bisect_right([s[0] for s in sched_f], c)
        chip.cycle(ui, 0, 0x01 | (lvf << RO))
    res = Result()
    res.di = frontend.pad(rec_t, rec_v, 'uo%d' % DI, half=True, duty_ns=duty)
    res.de = frontend.pad(rec_t, rec_v, 'uo%d' % DE, half=True, duty_ns=duty)
    res.rx, res.chip, res.img, res.t_end, res.partner = line, chip, img, cycles * CYCLE, partner
    res.tx = eth.tx_pair(fe, res.di, res.de)
    res.rep = H.check(res.tx, line, mac=mac, ip=IP, t_start=0.0, t_end=res.t_end, link=link,
                      jabber=jabber) if check else None
    res.health = health(chip, img, threads)
    res.count = {k: chip.mem[a] for k, a in MEM.items()}
    return res


def _sched(fe, line, phase, duty):
    ro = fe.ro(line)
    return eth.ro_schedule(ro, phase), eth.ro_schedule(ro, phase - CYCLE / 2 + duty)


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
    bad = [a for a, w in img.words.items() if a not in VARIABLE and chip.mem[a] != w]
    if bad:
        out.append('memory overwritten at %s' % bad[:5])
    return out


def rx_disposition(rep):
    """What 802.3 4.2.4 asks of each partner burst with an SFD, from its decoded octets (eth.disposition's rule)."""
    out = dict(keep=0, bad=0, other=0)
    for f in rep.rx_frames:
        if f.octets is None:
            continue
        octs = f.octets
        if len(octs) < 64 or not H.fcs_valid(octs):
            out['bad'] += 1
        else:
            out['keep' if octs[:6] in (MAC, eth.BCAST) else 'other'] += 1
    return out


def verdict(res, allow=()):
    bad = [f for f in res.rep.failed() if f[1] not in allow]
    out = ['%s @%.1f us: %s' % (c, t / 1e3, x) for t, c, x in bad[:3]]
    out += res.health
    c = res.count
    want = rx_disposition(res.rep)
    got = dict(keep=c['NKEEP'], bad=c['NBAD'], other=c['NOTHER'])
    if c['NBUSY'] or got != want or c['HANDED'] != c['RELEASED']:
        out.append('counters %s, busy %d, handed %d, released %d; the partner\'s bursts: %s' % (
            got, c['NBUSY'], c['HANDED'], c['RELEASED'], want))
    jams = [a for a in res.rep.attempts if a.jammed]
    late = [a for a in jams if a.late]
    drops = [g for g, o in res.rep.groups if o == 'dropped']
    if c['NCOLL'] != len(jams) or c['NXS'] != len(drops):
        out.append('NCOLL %d against %d jams on the line, NXS %d against %d frames dropped' % (
            c['NCOLL'], len(jams), c['NXS'], len(drops)))
    # a collision more than 512 BT into the attempt is late whenever it is seen; one up to 20 BT before 512 may be
    # seen after it (4.3 to 5.3 BT to the detector's word, then up to 8.8 BT to the poll), so may count either way
    near = [a for a in jams if a.t_occ is not None and H.SLOT - 20 * BT < a.t_occ - a.start <= H.SLOT]
    if not len(late) <= c['NLATE'] <= len(late) + len(near):
        out.append('NLATE %d against %d late collisions on the line (%d more just before 512 BT)' % (
            c['NLATE'], len(late), len(near)))
    return '; '.join(out)


def request():
    return eth.arp_req(spa=bytes([192, 168, 0, 9]))


def one(plan, seed=1, req=None, after=0.5e6, **kw):
    """One ARP request at 0.4 ms, the partner reacting to our transmissions by plan; runs until `after` ns past the
    last thing the plan can make happen (the caller sets it for long backoffs)."""
    L = eth.Line(seed)
    t = L.frame(req or request(), 0.4e6)
    P = Partner(L.changes, plan, seed=seed + 1000, jitter=kw.pop('pjit', None))
    return run(P, t + after, seed=seed, **kw)


def carry_run(text=None):
    """Six ARP requests 1 ms apart from one requester (different sender IPs, so the replies differ); the partner
    collides once with each reply's first attempt, 150 BT in."""
    L = eth.Line(61)
    for k in range(6):
        t = L.frame(eth.arp_req(spa=bytes([192, 168, 0, 50 + k])), 0.4e6 + k * 1.0e6)
    return run(Partner(L.changes, [(150, 'frag'), None] * 6, seed=61), t + 3.5e6, seed=61, text=text)


def replies(rep):
    return sum(1 for g, o in rep.groups if o == 'sent' and g[-1].fields.get('arp'))


# ---------------------------------------------------------------------------------------------------------------

BACKOFFS = []


def main(log):
    del BACKOFFS[:]
    log.group(PROG, title='half-duplex 10BASE-T node: CSMA/CD (802.3 clause 4) and ARP, against a reacting partner')
    img = assemble(src(PROG), depth=512)
    top = max(img.words)
    log.check('assembles to %d words, the count in section 11.2; code, data and two buffers fit 512' % WORDS,
              img.size == WORDS and top < 512, '%d words placed, last address %d' % (img.size, top))

    # no collision: requests answered, others dropped and counted, link pulses
    L = eth.Line(seed=21)
    t = 0.4e6
    for k in range(3):
        t = L.frame(eth.arp_req(spa=bytes([192, 168, 0, 20 + k])), t) + 150e3
    for bits in (eth.arp_req(tpa=eth.OTHER_IP), eth.arp_req(bad_fcs=True), eth.arp_req(pad=False),
                 eth.arp_req(dst=eth.OTHER_MAC), eth.arp_req(op=2)):
        t = L.frame(bits, t) + 150e3
    t = L.frame(eth.arp_req(spa=bytes([192, 168, 0, 30])), t) + 9.6e3      # two requests 96 BT apart: both answered
    t = L.frame(eth.arp_req(spa=bytes([192, 168, 0, 31])), t) + 300e3
    t = L.frame(eth.arp_req(spa=bytes([192, 168, 0, 32])), t) + 4.7e3      # and 47 BT apart (4.4.2 note 1)
    t = L.frame(eth.arp_req(spa=bytes([192, 168, 0, 33])), t) + 300e3
    r = run(Partner(L.changes), t + 34e6)
    v = verdict(r)
    log.check('no collision, %.0f ms: every check passes, 7 requests answered, threads healthy, counters right' % (
        r.t_end / 1e6), not v and replies(r.rep) == 7, v or ', '.join('%s %d' % (k, n[0]) for k, n in
                                                                  r.rep.checks().items() if n[0] or n[1]))

    # single collisions: inside the preamble, inside the first 512 BT, after them (late), at FN6307's corners; a
    # CSMA/CD partner's fragment, and a whole frame from a partner that doesn't defer
    rows = [(2, 'frag'), (20, 'frag'), (50, 'frag'), (63, 'frag'), (100, 'frag'), (250, 'frag'), (400, 'frag'),
            (480, 'frag'), (530, 'frag'), (545, 'frag'), (560, 'frag')]
    for corner, ph in (('typ', 7.5), ('max', 13.1), ('min', 2.9)):
        bad, after, hdr = [], [], []
        for off, kind in rows:
            r = one([(off, kind)], seed=int(off) * 7 + len(corner), corner=corner, phase=ph, after=1.2e6)
            v = verdict(r)
            jam = [a for a in r.rep.attempts if a.jammed]
            if v or len(jam) != 1 or replies(r.rep) != 1 or r.count['NCOLL'] != 1:
                bad.append('%g BT: %s' % (off, v or '%d jams, %d replies' % (len(jam), replies(r.rep))))
                continue
            a = jam[0]
            if off <= 50:
                hdr.append(len(a.bits) - a.sfd_at - 1)
            else:
                after.append((H._end(a) - a.t_occ) / BT)
        log.check('one collision at each of %d offsets from 2 to 560 BT, %s corner: jammed as clause 4 says, backed '
                  'off, sent again whole and answered; counters right' % (len(rows), corner), not bad and hdr and
                  set(hdr) == {32}, '; '.join(bad[:3]) or 'at 2 to 50 BT, %s bits after the SFD (the header, then the '
                  'jam); from 63 BT, the jam ends %.1f to %.1f BT after the collision' % (sorted(set(hdr)), min(after),
                                                                                         max(after)))
    bad = []
    for off in (5, 40, 100, 300, 530, 560):
        r = one([(off, 'frame')], seed=off * 11, after=1.5e6)
        v = verdict(r)
        if v or replies(r.rep) != 2 or r.count['NKEEP'] != 2:
            bad.append('%g BT: %s' % (off, v or '%d replies' % replies(r.rep)))
    log.check('a whole ARP request from a partner that does not defer, at 6 offsets from 5 to 560 BT: our reply '
              'jammed and sent again, the partner\'s request received whole (separate pairs) and answered after it',
              not bad,
              '; '.join(bad[:3]) or 'every offset')

    # the end of the frame: a collision too close to the end may go unseen, one before that must be jammed
    seen, unseen, bad = [], [], []
    for off in range(556, 577, 2):
        r = one([(off, 'frag')], seed=off, after=1.2e6)
        v = verdict(r)
        if v:
            bad.append('%d BT: %s' % (off, v))
        (seen if r.count['NCOLL'] else unseen).append(off)
    log.check('collisions 556 to 576 BT into a 576-BT reply: each jammed, or whole with the partner within 14 BT of '
              'its end (14.2.1.4, B.1.2 D7, 14.2.1.1)', not bad, '; '.join(bad[:3]) or 'jammed from %s BT, unseen from '
              '%s BT' % (seen and seen[0], unseen and unseen[0]))

    # several collisions in a row, then the line clears
    for k in (2, 3, 5):
        r = one([(150, 'frag')] * k, seed=40 + k, after=(1 << k) * 51.2e3 + 2e6)
        v = verdict(r)
        g = r.rep.groups
        log.check('the partner collides with %d attempts, then not: %d jams, the frame whole at attempt %d, every r '
                  'in range (4.2.3.2.5)' % (k, k, k + 1), not v and len(g) == 1 and len(g[0][0]) == k + 1 and
                  g[0][1] == 'sent', v or 'r %s' % [b[1] for b in r.rep.backoffs])

    # the attempt count is per frame (4.2.8 TransmitLinkMgmt: attempts starts at 0 for each frame): six requests in one
    # run, the partner colliding once with each reply's first attempt. A count carried from frame to frame would draw
    # the k-th reply's r from 0 to 2^k - 1, so some r of 2 or more with odds 1 - 2^-15
    r = carry_run()
    v = verdict(r)
    g = r.rep.groups
    rs = [x for _, x, _ in r.rep.backoffs]
    log.check('six requests in one run, the partner colliding once with each reply: each reply jammed once and sent at '
              'its 2nd attempt, every r 0 or 1, as the 1st retransmission of its own frame (the count restarts with '
              'each frame, 4.2.8)', not v and len(g) == 6 and all(len(a) == 2 and o == 'sent' for a, o in g) and
              rs and max(rs) <= 1 and r.count['NCOLL'] == 6, v or 'groups %s, r %s' % ([(len(a), o) for a, o in g], rs))

    # a partner that collides with every attempt: dropped after 16 (4.4.2, 4.2.8), counted; the node goes on. The
    # request after it comes from another station, so its reply differs from the first octets on
    for off in (100, 540):
        req2 = eth.arp_req(sha=eth.OTHER_MAC, spa=bytes([192, 168, 0, 77]))
        plan = [(off, 'frag')] * 15 + [[(off, 'frag'), (2000, 'frame', req2)]]
        L = eth.Line(5)
        L.frame(request(), 0.4e6)
        P = Partner(L.changes, plan, seed=6 + off)
        r = run(P, 0.4e6 + 420e6, seed=off)
        v = verdict(r, allow=('arp_reply',))
        miss = r.rep.failed('arp_reply')
        g = r.rep.groups
        rs = ['%d:%d' % (n, x) for n, x, _ in r.rep.backoffs]
        log.check('the partner collides with all 16 attempts at %d BT: no 17th, the frame dropped and counted (NXS 1), '
                  'every r in 0 to 2^min(n, 10) - 1; the dropped reply is the only one missing, and a request after '
                  'it is answered' % off,
                  not v and len(g) == 2 and len(g[0][0]) == 16 and g[0][1] == 'dropped' and g[1][1] == 'sent' and
                  r.count['NXS'] == 1 and r.count['NCOLL'] == 16 and len(miss) == 1 and miss[0][0] < 0.5e6,
                  v or 'n:r %s; missing only the reply to the request at %.1f us' % (
                      ' '.join(rs), miss[0][0] / 1e3 if miss else -1))
        BACKOFFS.extend(r.rep.backoffs)
    # the whole range at every n (4.2.3.2.5, k = min(n, 10); backoffLimit 10, 4.4.2): over the two runs, some r in the
    # upper half of 0 to 2^n - 1 from the 4th to the 9th retransmission, and some r of 512 or more from the 10th. A
    # uniform r misses either with odds 2^-12
    mid = [(n, x) for n, x, _ in BACKOFFS if 4 <= n <= 9]
    top = [(n, x) for n, x, _ in BACKOFFS if n >= 10]
    log.check('the two runs\' draws use the whole range: from the 4th to the 9th retransmission some r in the upper '
              'half of 0 to 2^n - 1, from the 10th some r of 512 or more (4.2.3.2.5: k = min(n, 10), backoffLimit 10)',
              len(mid) == 12 and len(top) == 12 and any(x >= 1 << (n - 1) for n, x in mid) and
              any(x >= 512 for _, x in top), 'n:r %s' % ' '.join('%d:%d' % p for p in mid + top))

    # the spread of r: the first three backoffs of 120 trials, each a reply that collides three times, the request
    # at a random cycle so that TB, which the generator takes in, differs from trial to trial
    rng = random.Random(77)
    tally = {1: [0] * 2, 2: [0] * 4, 3: [0] * 8}
    bad = []
    for k in range(120):
        L = eth.Line(900 + k)
        t0 = 0.4e6 + rng.randrange(0, 15000) * CYCLE
        L.frame(request(), t0)
        P = Partner(L.changes, [(rng.choice((20, 150, 400)), 'frag')] * 3, seed=k)
        r = run(P, t0 + 2.5e6, seed=k, phase=rng.uniform(0, 20))
        v = verdict(r)
        if v or len(r.rep.backoffs) != 3:
            bad.append('trial %d: %s' % (k, v or '%d backoffs' % len(r.rep.backoffs)))
            continue
        for n, x, _ in r.rep.backoffs:
            if x < len(tally[n]):
                tally[n][x] += 1
        BACKOFFS.extend(r.rep.backoffs)
    chi = {n: sum((c - sum(v) / len(v)) ** 2 / (sum(v) / len(v)) for c in v) if sum(v) else float('inf')
           for n, v in tally.items()}                  # no trial passed: an infinite chi-square, a FAIL
    crit = {1: 10.83, 2: 16.27, 3: 24.32}            # chi-square at p = 0.001, 1, 3 and 7 degrees of freedom
    log.check('backoff spread over 120 trials: r before the 1st, 2nd and 3rd retransmission uniform on 0 to 1, 0 to 3 '
              'and 0 to 7 (4.2.3.2.5; chi-square under its p = 0.001 value)', not bad and all(chi[n] < crit[n]
                                                                                          for n in chi),
              '; '.join(bad[:2]) or ' '.join('n=%d %s chi2 %.1f' % (n, tally[n], chi[n]) for n in tally))
    # little correlation between stations (4.2.3.2.5): two stations reset together, so with the same TB at every
    # draw, and MACs that differ; the same partner and timing, five collisions each. Their draws must differ
    plan = [(150, 'frag')] * 5
    ra = one(plan, seed=53, after=(1 << 5) * 51.2e3 + 2e6)
    rb = one(plan, seed=53, after=(1 << 5) * 51.2e3 + 2e6, mac=TWIN_MAC)
    va, vb = verdict(ra), verdict(rb)
    xa, xb = [x for _, x, _ in ra.rep.backoffs], [x for _, x, _ in rb.rep.backoffs]
    log.check('two stations with the same TB and different MACs, five collisions each: they draw different r '
              '(4.2.3.2.5: little correlation between stations)', not va and not vb and len(xa) == len(xb) == 5 and
              xa != xb, va or vb or 'r %s and %s' % (xa, xb))
    slack = [x for _, _, x in BACKOFFS]
    log.check('every backoff, %d of them: r slot times plus a constant wait (the gap and the firmware), %.1f to %.1f '
              'BT, so r is what the firmware drew' % (len(BACKOFFS), min(slack) / BT, max(slack) / BT),
              BACKOFFS and max(slack) - min(slack) < 100 * BT and min(slack) >= 96 * BT)

    # deference into a partner frame (4.2.3.2.1): one that starts in the first two thirds of our gap holds our reply
    # back; one that starts in the last third doesn't, and collides
    ref = one([], seed=31, after=0.4e6)
    t_s = ref.rep.attempts[0].start
    for x, kind in ((90, 'held'), (60, 'held'), (20, 'collides'), (8, 'collides')):
        L = eth.Line(31)
        L.frame(request(), 0.4e6)
        L.frame(G.frame(eth.OTHER_MAC, eth.PEER_MAC, 0x88B5, bytes(46)), t_s - x * BT)
        r = run(Partner(L.changes), t_s + 1.5e6, seed=31)
        v = verdict(r)
        a = r.rep.attempts
        ok = not v and replies(r.rep) == 1 and (len(a) == 1 if kind == 'held' else a[0].jammed)
        log.check('a frame for another station starting %d BT before our reply would have: %s' % (
            x, 'our reply waits for it and the gap after it' if kind == 'held' else
            'in the gap\'s last third, so our reply starts anyway, collides, and is sent again'), ok,
            v or '%d attempts, the first %.1f BT after the reference start' % (len(a), (a[0].start - t_s) / BT))

    # a partner's link test pulses during our frame are not activity on RD (14.2.3.1, 14.2.1.7): no jam
    bad = []
    for k, (off, w, lo) in enumerate(((10, 100.0, None), (65, 85.0, 120.0), (200, 137.5, None), (330, 100.0, 100.0),
                                      (450, 120.0, 50.0), (540, 137.5, 120.0), (300, 100.0, 300.0))):
        for corner in ('typ', 'max'):
            r = one([(off, 'pulse', (w, lo))], seed=60 + k, corner=corner, after=0.6e6)
            v = verdict(r)
            if v or r.count['NCOLL'] or replies(r.rep) != 1:
                bad.append('%g BT, %g ns, %s: %s' % (off, w, corner, v or 'NCOLL %d' % r.count['NCOLL']))
    log.check('a link test pulse from the partner during our reply, 6 offsets, widths 85 to 137.5 ns with and without '
              'a LO tail, and a single cycle (100 ns HI, 300 ns LO: 14.3.1.3.2 c), 2 corners: no jam, the reply '
              'whole', not bad, '; '.join(bad[:3]) or '14 runs')

    # jitter on the partner's colliding burst, thread placements, alignments and sampling phases
    bad = []
    for jit in (('uniform', 13.5, 13.5), ('alt', 13.5), ('alt', -13.5)):
        for off in (20, 100, 540):
            r = one([(off, 'frag')], seed=off + 3, pjit=jit, phase=off % 19 + 0.3, after=1.2e6)
            v = verdict(r)
            if v or r.count['NCOLL'] != 1 or replies(r.rep) != 1:
                bad.append('%s %g BT: %s' % (jit, off, v or 'NCOLL %d' % r.count['NCOLL']))
    log.check('the partner\'s burst with 802.3\'s 13.5 ns of jitter (14.3.1.3.1), random and both fixed patterns, at '
              '20, 100 and 540 BT: each seen and jammed', not bad, '; '.join(bad[:3]) or '9 runs')
    bad = []
    for th, al in (((2, 3), 1), ((3, 1), 2), ((1, 2), 3), ((0, 3), 0)):
        for off in (30, 200):
            r = one([(off, 'frag')] * 2, seed=al * 10 + off, threads=th, align=al, after=1.6e6)
            v = verdict(r)
            if v or r.count['NCOLL'] != 2 or replies(r.rep) != 1:
                bad.append('threads %s align %d, %g BT: %s' % (th, al, off, v or 'NCOLL %d' % r.count['NCOLL']))
    log.check('two collisions, R and T on 4 thread pairs and alignments: jammed, backed off, sent', not bad,
              '; '.join(bad[:3]) or '8 runs')

    # the link integrity test and the jabber function (802.3 14.2.1.6, 14.2.1.7), judged by eth_link_check held to the
    # program's stated values, and broken copies that those rows must catch. Long runs, so up to 4 at a time
    jobs = [('case', c) for c in eth.LINK_CASES] + [('probe', k) for k in eth.PROBE_EXPECT] + [('jab', None),
                                                                                              ('jab_down', None)]
    jobs += [('extend', k) for k in range(len(EXTEND_CFG))]
    jobs += [('extend_timer', k) for k in range(len(EXTEND_TIMER_US))]
    jobs += [('count_trap', k) for k in range(len(COUNT_TRAP_US))]
    jobs += [('tooth', i) for i in range(len(LINK_TEETH))]
    with concurrent.futures.ProcessPoolExecutor(max_workers=4) as ex:
        out = list(ex.map(link_job, jobs))
    for (kind, arg), (ok, detail) in zip(jobs, out):
        if kind == 'case':
            log.check('link integrity, the checker\'s case %s with a silence of 95 ms: every check passes held to the '
                      'program\'s values, and with 802.3\'s ranges; frames received while the link is down taken '
                      'unstored (NBUSY), the rest counted as 4.2.4 says; threads healthy' % arg, ok, detail)
        elif kind == 'probe':
            log.check('link integrity at the edges of the program\'s values, %s (eth.probe_line): every request gets '
                      'the status the values decide, %s, and every check passes' % (arg, eth.PROBE_EXPECT[arg]), ok,
                      detail)
        elif kind == 'jab':
            log.check('jabber: a broken copy that never stops feeding its reply is cut after 65,536 words (at most '
                      '105.5 ms), sends nothing for unjab (at least 499 ms) while its link test pulses go on, counts '
                      'the frame in NXS, and sends the reply it held once the wait is over', ok, detail)
        elif kind == 'extend':
            log.check('the end of EXTEND: a request whose TP_IDL lasts 4.5 BT ends LINK TEST FAIL and is taken '
                      'unstored, and the next, 47 BT after it (4.4.2 note 1), finds the link up and is answered, R and '
                      'T on threads %s at alignment %d' % EXTEND_CFG[arg], ok, detail)
        elif kind == 'extend_timer':
            log.check('EXTEND has no timer (Figure 14-6): a request that arrives in LINK TEST FAIL as link_test_max '
                      'runs out, with no 1, 0, 1 in its last 192 bits, ends it %.0f us after that and is taken '
                      'unstored, and the next, 1 ms after it, finds the link up and is answered' %
                      EXTEND_TIMER_US[arg], ok, detail)
        elif kind == 'count_trap':
            log.check('a request that T first sees as the start of a reception, before its pattern, counts as the '
                      'second link test pulse at a count of 1 (the request %.2f us late, 16 ms after one pulse): it is '
                      'still taken unstored, since the count goes through EXTEND, and the next is answered' %
                      COUNT_TRAP_US[arg], ok, detail)
        elif kind == 'jab_down':
            log.check('jabber and the link together: the link fails during the unjab wait, so the held reply waits '
                      'for it to come back (no transmission while it is down) and for the rest of the wait', ok,
                      detail)
        else:
            log.check('a broken copy is caught: %s' % LINK_TEETH[arg][0], ok, detail)

    # broken copies: each must fail somewhere
    for what, old, new, how in TEETH:
        text = broken(old, new)
        log.check('a broken copy is caught: %s' % what, *tooth(text, how))


TEETH = [
    ('the jam 16 BT short', 'addi  R1, 154        ; R6 must reach', 'addi  R1, 74         ; R6 must reach', 'jam'),
    ('no FCS cut-off at a collision (CODING left on)', 'scfg  R5, CODING     ; CMODE 0',
     'cp    R5, R5         ; CMODE 0', 'tail'),
    ('the append cancel never applied', 'subi  R6, 160        ; the append', 'subi  R6, 0          ; the append',
     'tail'),
    ('no header rule: a preamble collision jams before 32 bits follow the SFD', 'cp    R1, R7         ; seen in the',
     'cp    R1, R1         ; seen in the', 'header'),
    ('the backoff mask one bit wide from the first retransmission', 'ldi   R2, 0\nbm:', 'ldi   R2, 1\nbm:',
     'spread'),
    ('r always 0', 'and   R0, R2\n        ret', 'ldi   R0, 0\n        ret', 'spread'),
    ('the gap\'s first part 16 BT, not 64', 'li    R2, 320', 'li    R2, 80', 'gap'),
    ('a 2-bit pattern that a link pulse matches (SYNCLEN 2, 0, 1)', 'li    R0, 0x1801\n        scfg  R0, RXCNT',
     'li    R0, 0x1001\n        scfg  R0, RXCNT', 'pulse'),
    ('no clean-up of the 16 bits put behind a frame that had ended', 'sstop tx             ; the 16 bits',
     'idle                 ; the 16 bits', 'stray'),
    ('attemptLimit 17', 'cmpi  R0, 16', 'cmpi  R0, 17', 'limit'),
    ('the attempt count carried to the next frame', '        ldi   R1, NATT\n        st    R5, R1',
     '        ldi   R1, NATT\n        cp    R5, R5', 'carry'),
]


def broken(old, new):
    text = src(PROG)
    assert text.count(old) == 1, old
    return text.replace(old, new)


# ---------------------------------------------------------------------------------------------------------------
# the link integrity test and jabber (802.3 14.2.1.6, 14.2.1.7)

LINK_SILENCE = 95 * MS          # the checker's cases with silences past this program's link_loss, 87.9 ms at most
ENDLESS = ('djnz  R2, s3', 'goto  s3')         # a feed loop that never ends: the guard must cut the send


def link_verdict(res):
    """Every failing finding, the health and the counters for a run judged with link=: a frame that no allowed MAU
    hands up is taken unstored (NBUSY), one that some do may be either way, the rest as rx_disposition says."""
    rep = res.rep
    out = ['%s @%.1f us: %s' % (c, t / 1e3, x) for t, c, x in rep.failed()][:3]
    out += res.health
    c = res.count
    want, maybe, down = dict(keep=0, bad=0, other=0), dict(keep=0, bad=0, other=0), 0
    for f in rep.rx_frames:
        if f.octets is None:
            continue
        octs = f.octets
        d = 'bad' if len(octs) < 64 or not H.fcs_valid(octs) else ('keep' if octs[:6] in (MAC, eth.BCAST) else 'other')
        lk = getattr(f, 'link', 'up')
        if lk == 'down':
            down += 1
        elif lk == 'either':
            maybe[d] += 1
        else:
            want[d] += 1
    got = dict(keep=c['NKEEP'], bad=c['NBAD'], other=c['NOTHER'])
    ok = all(want[k] <= got[k] <= want[k] + maybe[k] for k in want) and \
        sum(got.values()) + c['NBUSY'] == sum(want.values()) + sum(maybe.values()) + down and c['NBUSY'] >= down
    if not ok or c['HANDED'] != c['RELEASED']:
        out.append('counters %s, busy %d, handed %d, released %d; frames up %s, either %s, down %d' % (
            got, c['NBUSY'], c['HANDED'], c['RELEASED'], want, maybe, down))
    return '; '.join(out)


def link_run(kind, arg, text=None):
    """(result, statuses with 802.3's ranges, failures with 802.3's ranges) for a case of eth.link_line or a probe of
    eth.probe_line, judged with the program's stated values (res.rep) and again with 802.3's ranges."""
    L, plan, t_end = eth.link_line(arg, silence=LINK_SILENCE) if kind == 'case' else eth.probe_line(arg, LINK_STATED)
    r = run(Partner(L.changes), t_end, text=text, link=dict(LINK_STATED, power_on='pass'), jabber={})
    wide = H.check(r.tx, r.rx, mac=MAC, ip=IP, t_start=0.0, t_end=r.t_end, link=dict(power_on='pass'), jabber={})
    return r, [f.link for f in wide.rx_frames if f.octets is not None], sorted({x for _, x, _ in wide.failed()})


# the checks a cut transmission fails anyway: its frame's size and FCS, the reply it should have been, and
# eth_hd_check's jam and retry, which read a send that ends early as an attempt cut short with no collision, and the
# next frame as not its retry (802.3's MAC would retry the cut frame on the CS0 of 14.2.1.6 b, invisibly at the MDI,
# until attemptLimit). The cut itself ends with TP_IDL, a start of idle (Figure 14-3: xmit=disable sends TD_idle;
# 14.2.1.1), so the checks of a transmission's end are allowed to fail only for one still going when the record ends
JAB_ALLOWED = ('frame_size', 'fcs', 'arp_reply', 'jam', 'retry')
JAB_ALLOWED_OPEN = ('manchester', 'tp_idl', 'edge_timing')


def jab_run(text):
    """Two requests, at 0.4 and 300 ms, and the partner's link test pulses every 16 ms; to 650 ms."""
    L = eth.Line(71)
    L.frame(request(), 0.4e6)
    L.frame(eth.arp_req(spa=bytes([192, 168, 0, 72])), 300e6)
    for q in range(41):
        L.pulse(1e6 + q * eth.NLP)
    return run(Partner(L.changes), 650 * MS, text=text, link=dict(LINK_STATED, power_on='pass'), jabber=JAB_STATED)


# thread pairs and alignments for the end of EXTEND, one per thread pair: over 16 configurations, LINK TEST PASS comes
# 14.2 to 20.4 BT before the next frame
EXTEND_CFG = (((0, 1), 0), ((2, 3), 3), ((1, 0), 2), ((3, 2), 1))


def extend_run(k, text=None):
    """The end of LINK TEST FAIL EXTEND against the next frame: after a silence, a request whose TP_IDL lasts 4.5 BT,
    14.3.1.2.1's longest (it ends LINK TEST FAIL and is dropped), and the next 47 BT after its end, the shortest gap
    a receiver may see (4.4.2 note 1), which must find the link up; on EXTEND_CFG[k]."""
    th, al = EXTEND_CFG[k]
    L = eth.Line(90 + al)
    for q in range(2):
        L.pulse(1e6 + q * eth.NLP)
    e = L.frame(eth.arp_req(spa=bytes([192, 168, 9, 1])), 20e6)
    e = L.frame(eth.arp_req(spa=bytes([192, 168, 9, 2])), e + LINK_SILENCE, tp_hi=4.5)
    e = L.frame(eth.arp_req(spa=bytes([192, 168, 9, 3])), e + 4.7e3)
    return run(Partner(L.changes), e + 1 * MS, threads=th, align=al, text=text, link=dict(LINK_STATED, power_on='pass'),
               jabber={})


# EXTEND has no timer (Figure 14-6): request A ends this many us after link_test_max runs out in LINK TEST FAIL
EXTEND_TIMER_US = (2.0, 6.0)
WRAP = 65536 * CYCLE            # ns between TB wraps


def extend_timer_run(k, text=None):
    """LINK TEST FAIL EXTEND against the link timer: after a request at 20 ms and silence, link_loss runs out on the
    program's LINKW-th TB wrap after that request and link_test_max on the LINKW-th wrap after that (in LINK TEST
    FAIL the timer starts again on a wrap). Request A, whose last 192 bits hold no 1, 0, 1 (sender IP
    192.168.255.237), so that T's receive half gives no word after them, ends EXTEND_TIMER_US[k] us after that second
    expiry: it ends LINK TEST FAIL and is taken unstored. Request B, 1 ms after A, finds the link up (EXTEND ends when
    RD goes idle) and is answered."""
    t = text if text is not None else src(PROG)
    linkw = int(t.split('.equ LINKW', 1)[1].split()[0])
    L = eth.Line(31)
    L.pulse(1e6)
    L.pulse(17e6)
    e = L.frame(eth.arp_req(spa=bytes([192, 168, 9, 1])), 20e6)
    w = int(e // WRAP) + 2 * linkw          # the first wrap after the request is int(e // WRAP) + 1
    bits = eth.arp_req(spa=bytes([192, 168, 255, 237]))
    e = L.frame(bits, w * WRAP + EXTEND_TIMER_US[k] * 1e3 - eth.Line(1).frame(bits, 0.0))
    e = L.frame(eth.arp_req(spa=bytes([192, 168, 9, 3])), e + 1 * MS)
    return run(Partner(L.changes), e + 1 * MS, text=text, link=dict(LINK_STATED, power_on='pass'), jabber={})


# a frame whose start T's poll sees before its pattern (status bit 12 without bit 9) counts as a link test pulse; at a
# count of 1 it is the second, and must still be taken unstored (the count goes through EXTEND, which waits for the
# frame's end). T's poll lands in that window, about 0.3 us of its 2.4 us loop, at these offsets of the request
COUNT_TRAP_US = (0.30, 0.45)


def count_trap_run(k, text=None):
    """LINK TEST FAIL with a count of 1 when a request arrives: after a request at 20 ms and 95 ms of silence, one link
    test pulse, then a request 16 ms + COUNT_TRAP_US[k] us later (taken unstored) and another 0.3 ms after it
    (answered)."""
    L = eth.Line(51)
    L.pulse(1e6)
    L.pulse(17e6)
    e = L.frame(eth.arp_req(spa=bytes([192, 168, 13, 1])), 20e6)
    p = e + LINK_SILENCE
    L.pulse(p)
    e = L.frame(eth.arp_req(spa=bytes([192, 168, 13, 2])), p + eth.NLP + COUNT_TRAP_US[k] * 1e3)
    e = L.frame(eth.arp_req(spa=bytes([192, 168, 13, 3])), e + 0.3 * MS)
    return run(Partner(L.changes), e + 1 * MS, text=text, link=dict(LINK_STATED, power_on='pass'), jabber={})


def jab_down_run(text):
    """The jabber and the link together: a request at 0.4 ms whose reply is cut, the partner's link test pulses to
    145 ms, a request at 160 ms (kept, its reply held by the unjab wait), then silence, so the link fails during the
    wait; from 700 ms pulses again. The reply to the second request waits for the link to come back, then for the
    rest of the wait, which counts only in LINK TEST PASS; to 1,150 ms."""
    L = eth.Line(72)
    L.frame(request(), 0.4e6)
    for q in range(10):
        L.pulse(1e6 + q * eth.NLP)
    L.frame(eth.arp_req(spa=bytes([192, 168, 0, 73])), 160e6)
    for q in range(29):
        L.pulse(700e6 + q * eth.NLP)
    return run(Partner(L.changes), 1150 * MS, text=text, link=dict(LINK_STATED, power_on='pass'), jabber=JAB_STATED)


def jab_verdict(r):
    """The jab row's judgment: the checker's failures but those a cut frame fails anyway, the health, NXS 1, and the
    transmissions: one cut, then the next at least unjab later."""
    tx = sorted(r.rep.tx_frames, key=lambda f: f.start)
    open_from = min([f.start for f in tx if f.stop >= r.t_end - 2 * BT] or [float('inf')])
    bad = ['%s: %s' % (c, x) for t, c, x in r.rep.failed()
           if c not in JAB_ALLOWED and not (c in JAB_ALLOWED_OPEN and t >= open_from)][:3]
    bad += r.health
    cuts = [x for x in r.rep.findings if x.check == 'jabber' and 'after a cut' in x.text]
    if not cuts:
        bad.append('no cut and later transmission found')
    if r.count['NXS'] != 1:
        bad.append('NXS %d' % r.count['NXS'])
    return '; '.join(bad), '%d transmissions, %s; NXS %d' % (len(tx), cuts[0].text if cuts else '-', r.count['NXS'])


# broken copies for the link and jabber rows: (what, old, new, the row that must fail: a case, a probe or 'jab')
LINK_TEETH = [
    ('frames taken while the link is down (R ignores LST)', 'or    R4, R6', 'idle', ('case', 'frame_in_fail')),
    ('lc_max 1: one link test pulse brings the link back', 'cmpi  R4, 4\n        jt    tx6',
     'cmpi  R4, 3\n        jt    tx6', ('case', 'one_pulse')),
    ('an early pulse counted in LINK TEST FAIL (no RESET)', 'ldi   R4, 1          ; LINK TEST FAIL RESET',
     'idle                 ; LINK TEST FAIL RESET', ('probe', 'min')),
    ('EXTEND left without waiting for RD idle', 'jt    t0r            ; RD active', 'idle                 ; RD active',
     ('case', 'close_after')),
    ('link_loss and link_test_max 30 wraps (39 ms, under the stated 86.4)', '.equ LINKW 67', '.equ LINKW 30',
     ('probe', 'loss')),
    ('no jabber guard: the endless feed loop runs on', 'djnz  R3, sp         ; the jabber\'s guard\n        goto  jab',
     'goto  sp             ; the jabber\'s guard\n        idle', ('jab', None)),
    ('an unjab wait of 1,000 slot times, 51 ms', '.equ UNJAB 9766', '.equ UNJAB 1000', ('jab', None)),
    ('no send gate: a reply held through the unjab wait goes out while the link is down',
     'cmpi  R4, 4\n        jf    tloop', 'cmpi  R4, 4\n        idle', ('jab_down', None)),
    ('LINK TEST PASS takes an early pulse: link_test_min not applied there', 'bt    R4, 2\n        jt    t0e',
     'bt    R4, 2\n        jt    tpa', ('probe', 'early')),
    ('the timers not restarted by LINK TEST FAIL COUNT', 'jt    tx6            ; link_count = lc_max: EXTEND\n'
     '        goto  t0r', 'jt    tx6            ; link_count = lc_max: EXTEND\n        goto  t0x', ('probe', 'max')),
    ('EXTEND waits for 250 cycles of quiet RO, not 12, so a frame 47 BT after the one that ended LINK TEST FAIL finds '
     'the link down', '.equ QUIET 12', '.equ QUIET 250', ('extend', 0)),
    ('EXTEND left on the link timer: RD activity there doesn\'t start LW again, so link_test_max running out during '
     'the frame that ends LINK TEST FAIL puts the node back in LINK TEST FAIL', 'jt    t0r            ; RD active',
     'jt    t0x            ; RD active', ('extend_timer', 0)),
    ('the jabber\'s cut by sstop tx: DE drops at once, so the cut send ends with no TP_IDL (start of idle)',
     'jab:    li    R0, UNJAB', 'jab:    sstop tx\n        li    R0, UNJAB', ('jab', None)),
    ('the pulse count reaches LINK TEST PASS without EXTEND, so a request whose start T counts as the second pulse is '
     'handed up', 'cmpi  R4, 4\n        jt    tx6', 'cmpi  R4, 5\n        jt    tx6', ('count_trap', 0)),
]


def link_job(job):
    """One of the link and jabber rows, run in a worker process: (ok, detail)."""
    kind, arg = job
    if kind in ('case', 'probe'):
        r, sts, wide = link_run(kind, arg)
        v = link_verdict(r)
        mine = [f.link for f in r.rep.rx_frames if f.octets is not None]
        if kind == 'probe' and mine != eth.PROBE_EXPECT[arg]:
            v = (v + '; ' if v else '') + 'statuses %s' % mine
        if wide:
            v = (v + '; ' if v else '') + 'with 802.3\'s ranges it fails %s' % wide
        return not v, v or 'statuses %s (802.3\'s ranges: %s); NBUSY %d; %.0f ms' % (mine, sts, r.count['NBUSY'],
                                                                                    r.t_end / MS)
    if kind == 'jab':
        v, d = jab_verdict(jab_run(broken(*ENDLESS)))
        return not v, v or d
    if kind == 'extend':
        r = extend_run(arg)
        v = link_verdict(r)
        sts = [f.link for f in r.rep.rx_frames if f.octets is not None]
        if sts != ['up', 'down', 'up']:
            v = (v + '; ' if v else '') + 'statuses %s' % sts
        return not v, v or 'threads %s, alignment %d: statuses %s; NBUSY %d' % (
            EXTEND_CFG[arg][0], EXTEND_CFG[arg][1], sts, r.count['NBUSY'])
    if kind == 'extend_timer':
        r = extend_timer_run(arg)
        v = link_verdict(r)
        sts = [f.link for f in r.rep.rx_frames if f.octets is not None]
        if sts != ['up', 'down', 'up']:
            v = (v + '; ' if v else '') + 'statuses %s' % sts
        return not v, v or 'A ends %.1f us after link_test_max runs out: statuses %s; NKEEP %d, NBUSY %d' % (
            EXTEND_TIMER_US[arg], sts, r.count['NKEEP'], r.count['NBUSY'])
    if kind == 'count_trap':
        r = count_trap_run(arg)
        v = link_verdict(r)
        sts = [f.link for f in r.rep.rx_frames if f.octets is not None]
        if sts != ['up', 'down', 'up']:
            v = (v + '; ' if v else '') + 'statuses %s' % sts
        return not v, v or 'the request %.2f us late: statuses %s; NKEEP %d, NBUSY %d' % (
            COUNT_TRAP_US[arg], sts, r.count['NKEEP'], r.count['NBUSY'])
    if kind == 'jab_down':
        r = jab_down_run(broken(*ENDLESS))
        v, d = jab_verdict(r)
        v = v.replace('NXS 2', '') if r.count['NXS'] == 2 else v       # both requests' replies are cut here
        v = '; '.join(x for x in v.split('; ') if x)
        tx = sorted(r.rep.tx_frames, key=lambda f: f.start)
        late = [f for f in tx if f.start > 160 * MS]
        if not late or late[0].start < 716 * MS:
            v = (v + '; ' if v else '') + 'the held reply at %s' % ('%.1f ms' % (late[0].start / MS) if late else
                                                                        'no time')
        return not v, v or '%s; the held reply at %.1f ms' % (d, late[0].start / MS)
    what, old, new, (rk, ra) = LINK_TEETH[arg]
    text = broken(old, new)
    if rk in ('jab', 'jab_down'):
        t = text
        assert t.count(ENDLESS[0]) == 1
        r = (jab_run if rk == 'jab' else jab_down_run)(t.replace(*ENDLESS))
        v, d = jab_verdict(r)
        if rk == 'jab_down':
            v = '; '.join(x for x in v.split('; ') if x and x != 'NXS 2')
        return bool(v), v[:200] if v else 'every check passed: ' + d
    if rk == 'extend':
        r = extend_run(ra, text=text)
    elif rk == 'extend_timer':
        r = extend_timer_run(ra, text=text)
    elif rk == 'count_trap':
        r = count_trap_run(ra, text=text)
    else:
        r = link_run(rk, ra, text=text)[0]
    v = link_verdict(r)
    return bool(v), '%s %s: %s' % (rk, ra, v[:200]) if v else '%s %s: every check passed' % (rk, ra)


def tooth(text, how):
    """Runs a broken copy on the scenario named; returns (caught, detail)."""
    if how in ('jam', 'tail', 'header'):
        offs = {'jam': (100, 20), 'tail': (528, 532, 536), 'header': (20, 40)}[how]
        out = []
        for off in offs:
            r = one([(off, 'frag')], seed=off, text=text, after=1.2e6)
            v = verdict(r)
            if v:
                return True, '%g BT: %s' % (off, v[:200])
            out.append(off)
        return False, 'every check passed at %s BT' % (out,)
    if how == 'spread':
        rng = random.Random(5)
        tally = {1: [0, 0], 2: [0] * 4}
        for k in range(40):
            L = eth.Line(700 + k)
            t0 = 0.4e6 + rng.randrange(0, 15000) * CYCLE
            L.frame(request(), t0)
            r = run(Partner(L.changes, [(150, 'frag')] * 2, seed=k), t0 + 1.5e6, seed=k, text=text)
            v = verdict(r)
            if v:
                return True, 'trial %d: %s' % (k, v[:200])
            for n, x, _ in r.rep.backoffs:
                tally[n][x] += 1
        chi = {n: sum((c - sum(v) / len(v)) ** 2 / (sum(v) / len(v)) for c in v) for n, v in tally.items()}
        caught = chi[1] >= 10.83 or chi[2] >= 16.27
        return caught, 'r over 40 trials: %s, chi2 %.1f and %.1f' % (tally, chi[1], chi[2])
    if how == 'gap':
        L = eth.Line(21)
        t = L.frame(eth.arp_req(spa=bytes([192, 168, 0, 30])), 0.4e6) + 9.6e3
        L.frame(eth.arp_req(spa=bytes([192, 168, 0, 31])), t)
        r = run(Partner(L.changes), t + 0.6e6, text=text)
        v = verdict(r)
        return bool(v), v[:200] if v else 'every check passed'
    if how == 'pulse':
        for off in (10, 200, 450):
            r = one([(off, 'pulse', (100.0, None))], seed=off, text=text, after=0.6e6)
            v = verdict(r)
            if v:
                return True, '%g BT: %s' % (off, v[:200])
        return False, 'every check passed'
    if how == 'stray':
        L = eth.Line(8)
        L.frame(request(), 0.4e6)
        L.frame(eth.arp_req(sha=eth.OTHER_MAC, spa=bytes([192, 168, 0, 78])), 1.2e6)
        for off in (574, 576, 578):
            r = run(Partner(L.changes, [(off, 'frag')], seed=off), 2.0e6, text=text)
            v = verdict(r)
            if v:
                return True, '%g BT: %s' % (off, v[:200])
        return False, 'every check passed'
    if how == 'carry':
        r = carry_run(text)
        v = verdict(r)
        return bool(v), v[:200] if v else 'every check passed, r %s' % [x for _, x, _ in r.rep.backoffs]
    if how == 'limit':
        plan = [(100, 'frag')] * 16
        L = eth.Line(5)
        L.frame(request(), 0.4e6)
        r = run(Partner(L.changes, plan, seed=106), 0.4e6 + 420e6, seed=100, text=text)
        v = verdict(r, allow=('arp_reply',))
        return bool(v), v[:200] if v else 'every check passed'
    raise ValueError(how)
