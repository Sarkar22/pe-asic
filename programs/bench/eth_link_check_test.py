"""Self-test of eth_link_check.py on synthetic records: a reference station's transmit pair against the partner lines
of eth.link_line, judged by eth_check.check with link= and jabber=.

The reference station shares no code with the checker. Its MAU runs Figure 14-6 with one exact value for each timer
and lc_max, deterministically, on the partner's pulses and frames as eth.link_line placed them (it decodes nothing);
it answers the ARP requests its MAU hands up, 20 us after each, and sends link test pulses every 16 ms whatever its
link state (TD_idle is TP_IDL, 14.2.2.2). Its jabber records cut a transmission that would go on, and stay silent
but for link test pulses until unjab has passed (Figure 14-5).

Checked:
  - the link status the checker gives each request, for each case, against what 14.2.1.7 decides for every MAU
    802.3 allows (and for stated values);
  - good records: the reference with timer values at the ends and the middle of 802.3's ranges, both power-on
    states, passes every check on every case, and every request it answered was not 'down' and every one it dropped
    not 'up';
  - bad records: a reference with a value outside the ranges or outside its stated values, or a station that sends
    while its link is down, sends before the frame that ends Link Test Fail is over (no EXTEND) or leaves EXTEND
    late, keeps sending past every allowed link_loss, stops its link test pulses while failed, answers a frame its
    MAU dropped, jabbers past 150 ms, or sends again before unjab, each fails the check named for it, in the case
    built to show it;
  - the same through eth_hd_check.check, where every attempt is a transmission;
  - link_loss counted from when RD goes idle at the end of a long frame, not from its start.
"""
import eth
import eth_check
import eth_check_test as G
import eth_hd_check

PROGRAMS = ('bench/eth_link_check.py',)

BT = 100.0
MS = 1e6
MAC, IP, PEER_MAC = G.MAC, G.IP, G.PEER_MAC
GAP = 20e3                      # the reference answers 20 us after a request ends (4.4.2: at least 9.6 us)

# ---------------------------------------------------------------------------------------------------------------
# the reference station


def rd_events(plan):
    """(t0, t1, kind, request index or None): a pulse from its start to its end; a frame from its first level until
    RD goes idle, 1 BT after its last low-to-high transition (14.3.1.3.3 allows up to 2.3 BT)."""
    out, n = [], 0
    for e in plan:
        if e[0] == 'pulse':
            out.append((e[1], e[1] + e[2], 'pulse', None))
        else:
            bits = G.preamble(56) + e[2]
            end = e[1] + len(bits) * BT
            rise = end - BT / 2 if bits[-1] == 1 else end
            out.append((e[1], rise + BT, 'frame', n))
            n += 1
    return out


def ref_mau(rd, L, m, M, c, power_on, t_end, xq=0.0):
    """Figure 14-6 with these values. Returns (handed[i] for each request, LINK TEST PASS intervals). xq: a broken
    EXTEND that waits xq ns more after RD goes idle."""
    st = 'P' if power_on == 'pass' else 'F'
    s, k, x_exit = 0.0, 0, None
    passing, p0 = [], (0.0 if st == 'P' else None)
    handed = {}

    def advance(t):
        nonlocal st, s, k, x_exit, p0
        while True:
            if st == 'X' and x_exit <= t:
                st, s, p0 = 'P', x_exit, x_exit
            elif st == 'P' and s + L < t:
                passing.append((p0, s + L))
                st, k, s = 'F', 0, s + L        # LINK TEST FAIL RESET, then FAIL with its timers started
            elif st == 'F' and s + M < t:
                k, s = 0, s + M                 # link_test_max_timer done: RESET, then FAIL again
            else:
                return
    for t0, t1, kind, i in rd:
        advance(t0)
        if kind == 'frame':
            if st == 'P':
                handed[i] = True
                s = t1                          # both timers restart while RD is active: at its end
            else:
                handed[i] = False               # rcv=disable; RD=active ends LINK TEST FAIL
                st, x_exit = 'X', t1 + xq
        elif st == 'P':
            if t0 - s >= m:
                s = t0                          # accepted; an early pulse is ignored
        elif st == 'F':
            if t0 - s >= m:
                k += 1
                if k >= c:
                    st, x_exit = 'X', t1        # link_count = lc_max: EXTEND, then PASS (RD and DO idle)
                else:
                    s = t1
            else:
                k, s = 0, t1                    # early: RESET
    advance(t_end)
    if st == 'P':
        passing.append((p0, t_end))
    return handed, passing


def in_pass(passing, t):
    return any(a <= t <= b for a, b in passing)


def reply_bits(spa):
    return G.preamble(56) + G.frame(PEER_MAC, MAC, 0x0806, G.arp(2, MAC, IP, PEER_MAC, spa))


def ref_record(case, L=100 * MS, m=4 * MS, M=40 * MS, c=2, power_on='pass', hold=False, delay=0.0,
               answer_dropped=False, pulses_in_fail=True, tx_during=None, xq=0.0):
    """(tx, rx, t_end, handed) for a case of eth.link_line. delay: the first reply is wanted that much later than
    usual (a busy station); hold: a reply is sent only in LINK TEST PASS, held until the link is back; tx_during=k:
    the first reply is sent 20 us after request k begins, while it is still on the line (no EXTEND)."""
    line, plan, t_end = eth.link_line(case)
    rd = rd_events(plan)
    handed, passing = ref_mau(rd, L, m, M, c, power_on, t_end, xq)
    reqs = [e for e in plan if e[0] == 'req']
    sends = []
    for (t0, t1, kind, i) in rd:
        if kind != 'frame' or not (handed[i] or answer_dropped):
            continue
        due = t1 + GAP + (delay if i == 0 else 0.0)
        if hold:
            later = [a for a, b in passing if b >= due]
            if not later:
                continue
            due = max(due, min(later) + GAP)
        if tx_during is not None and i == 0:
            due = reqs[tx_during][1] + GAP
        sends.append((due, reqs[i][3]))
    tx = []
    t_p = 0.5e6
    for due, spa in sorted(sends) + [(t_end + 1, None)]:
        while t_p < due - 1e3 and t_p < t_end:
            if pulses_in_fail or in_pass(passing, t_p):
                tx += G.link_pulse(t_p)
            t_p += eth.NLP
        if spa is None:
            break
        ch, end = G.manchester(reply_bits(spa), due)
        tx += ch
        rise = end - BT / 2 if reply_bits(spa)[-1] == 1 else end
        t_p = rise + eth.NLP
    return G.merge(tx), line.line(), t_end, handed


def judge(tx, rx, t_end, power_on='either', hd=False, **link):
    link = dict(link, power_on=power_on)
    f = eth_hd_check.check if hd else eth_check.check
    return f(tx, rx, mac=MAC, ip=IP, t_start=0.0, t_end=t_end, link=link)


def statuses(rep):
    """The link status of each request the partner sent, in time order."""
    return [rf.link for rf in rep.rx_frames if rf.octets is not None]


# what 14.2.1.7 decides for each request, for every MAU 802.3 allows, powering on in LINK TEST PASS
EXPECT = dict(loss_pulses=['up', 'up'], one_pulse=['up', 'down', 'up'], fast_pulses=['up', 'down', 'up'],
              slow_pulses=['up', 'down', 'up'], frame_in_fail=['up', 'down', 'up'], keep_up=['up'],
              short_silence=['up', 'up', 'either', 'up'], mid_silence=['up', 'either', 'up'],
              flp_in_fail=['up', 'down', 'up'], hold=['up'],
              power_on=['up', 'up'], probe_loss=['up', 'either', 'either', 'up'], probe_count=['up', 'either', 'up'],
              close_after=['up', 'down', 'up'])
# the values a station could state, and what they decide
STATED = dict(link_loss=(64.2 * MS, 65.6 * MS), link_test_min=(3.9 * MS, 5.3 * MS),
              link_test_max=(38.0 * MS, 39.4 * MS), lc_max=(2, 2))
EXPECT_STATED = dict(probe_loss=['up', 'up', 'down', 'up'], probe_count=['up', 'up', 'up'],
                     mid_silence=['up', 'down', 'up'])
INSIDE = dict(L=65.0 * MS, m=4.6 * MS, M=38.7 * MS, c=2)       # a reference inside the stated values

# a sweep inside 802.3's ranges: every corner of (link_loss, link_test_min, link_test_max, lc_max), and the middle
SWEEP = [dict(L=L, m=m, M=M, c=c) for L in (50.01 * MS, 149.99 * MS) for m in (2.01 * MS, 6.99 * MS)
         for M in (25.01 * MS, 149.99 * MS) for c in (2, 10)] + [dict(L=100 * MS, m=4.5 * MS, M=80 * MS, c=5)]

# broken stations: (what, case, reference arguments, the check that must fail, judge arguments)
BAD = [
    ('lc_max 1: one pulse brings the link back', 'one_pulse', dict(c=1), 'link', {}),
    ('link_loss 40 ms: the link fails too soon, a request 45 ms into a silence dropped', 'short_silence',
     dict(L=40 * MS), 'arp_reply', {}),
    ('link_loss 200 ms: the link never fails in 200 ms of silence', 'frame_in_fail', dict(L=200 * MS), 'link', {}),
    ('link_test_min 0.5 ms: pulses 1 ms apart counted', 'fast_pulses', dict(m=0.5 * MS), 'link', {}),
    ('link_test_max 200 ms: pulses 160 ms apart counted as consecutive', 'slow_pulses', dict(M=200 * MS), 'link', {}),
    ('lc_max 13: 12 pulses don\'t bring the link back', 'loss_pulses', dict(c=13), 'arp_reply', {}),
    ('an FLP burst counted as link test pulses (link_test_min 0.05 ms)', 'flp_in_fail', dict(m=0.05 * MS), 'link',
     {}),
    ('powers on in Link Test Pass but states Link Test Fail Reset', 'power_on', dict(power_on='pass'), 'link',
     dict(power_on='fail')),
    ('a reply sent while the link is down (held 170 ms, not until the link is back)', 'hold', dict(delay=170 * MS),
     'link', {}),
    ('link test pulses stop while the link is down', 'loss_pulses', dict(pulses_in_fail=False), 'link_period', {}),
    ('a frame its MAU dropped answered anyway', 'frame_in_fail', dict(answer_dropped=True), 'link', {}),
    ('no EXTEND: a held reply sent while the frame that ends Link Test Fail is still on the line', 'frame_in_fail',
     dict(delay=170 * MS, hold=True, tx_during=1), 'link', {}),
    ('EXTEND left only after 96 BT of quiet: the frame 47 BT after the one that ends Link Test Fail dropped',
     'close_after', dict(xq=9.6e3), 'arp_reply', {}),
    ('stated link_loss 58 ms behaves as 58: a request 60 ms into a silence dropped', 'probe_loss',
     dict(L=58 * MS, m=INSIDE['m'], M=INSIDE['M'], c=2), 'arp_reply', dict(STATED, power_on='pass')),
    ('stated link_loss 64.2 to 65.6 ms, behaves as 72: a request 70 ms into a silence answered', 'probe_loss',
     dict(L=72 * MS, m=INSIDE['m'], M=INSIDE['M'], c=2), 'link', dict(STATED, power_on='pass')),
    ('stated lc_max 2, behaves as 3: two pulses don\'t bring the link back', 'probe_count',
     dict(L=INSIDE['L'], m=INSIDE['m'], M=INSIDE['M'], c=3), 'arp_reply', dict(STATED, power_on='pass')),
    ('a stated link_loss of 40 to 45 ms, outside 802.3\'s 50 to 150', 'keep_up', {}, 'link',
     dict(link_loss=(40 * MS, 45 * MS))),
]

# ---------------------------------------------------------------------------------------------------------------
# jabber records


def jab_record(burst=30 * MS, next_after=600 * MS, pulses_in_unjab=True):
    """A healthy partner (pulses every 16 ms, a request at 1 ms and one more later); the station's reply to the first
    request goes on as alternating bits until it is cut `burst` ns after its start, then TP_IDL; its reply to the
    second request starts `next_after` ns after the cut. Link test pulses every 16 ms all along, but for the
    unjab wait if pulses_in_unjab is False."""
    rx = []
    r1 = G.frame(eth.BCAST, PEER_MAC, 0x0806, G.arp(1, PEER_MAC, G.PEER_IP, bytes(6), IP))
    ch, e1 = G.manchester(G.preamble(56) + r1, 1e6)
    rx += ch
    bits = reply_bits(G.PEER_IP)
    n = int(burst / BT) - len(bits)
    bits = bits + [1, 0] * (n // 2)
    t_cut0 = e1 + GAP
    ch, end = G.manchester(bits, t_cut0)
    tx = list(ch)
    cut = end - BT / 2 if bits[-1] == 1 else end
    spa2 = bytes([192, 168, 0, 99])
    t2 = cut + next_after - 200e3
    r2 = G.frame(eth.BCAST, PEER_MAC, 0x0806, G.arp(1, PEER_MAC, spa2, bytes(6), IP))
    ch, e2 = G.manchester(G.preamble(56) + r2, t2)
    rx += ch
    t_end = e2 + 20 * MS
    for q in range(int(t_end / eth.NLP) + 1):              # the partner's pulses, clear of its frames
        tp = 0.5e6 + q * eth.NLP
        if not (0.9e6 < tp < e1 + 1e3 or t2 - 1e3 < tp < e2 + 1e3):
            rx += G.link_pulse(tp)
    t_p = 0.5e6
    while t_p < t_cut0 - 1e3:
        tx += G.link_pulse(t_p)
        t_p += eth.NLP
    t_p = cut + eth.NLP
    t_r2 = e2 + GAP
    while t_p < t_r2 - 1e3:
        if pulses_in_unjab or t_p > cut + 500 * MS:
            tx += G.link_pulse(t_p)
        t_p += eth.NLP
    rb = reply_bits(spa2)
    ch, end2 = G.manchester(rb, t_r2)
    tx += ch
    rise2 = end2 - BT / 2 if rb[-1] == 1 else end2
    t_p = rise2 + eth.NLP
    while t_p < t_end:
        tx += G.link_pulse(t_p)
        t_p += eth.NLP
    return G.merge(tx), G.merge(rx), t_end


JAB_STATED = dict(xmit_max=(25 * MS, 35 * MS), unjab=(450 * MS, 550 * MS))


def late_record(start=140 * MS, burst=30 * MS):
    """A request, then nothing from the partner; the station answers `start` ns after the request's end with a
    transmission of `burst` ns (its reply, then alternating bits). From 140 ms some allowed MAU is still in Link Test
    Pass, but every one has left it by 150 ms, so the end of the transmission is past xmit=disable for all of them."""
    r1 = G.frame(eth.BCAST, PEER_MAC, 0x0806, G.arp(1, PEER_MAC, G.PEER_IP, bytes(6), IP))
    rx, e1 = G.manchester(G.preamble(56) + r1, 1e6)
    bits = reply_bits(G.PEER_IP)
    bits = bits + [1, 0] * ((int(burst / BT) - len(bits)) // 2)
    t0 = e1 + start
    tx, end = G.manchester(bits, t0)
    t_end = end + 20 * MS
    t_p = 0.5e6
    while t_p < t_end:
        if not t0 - 1e3 < t_p < end + 16 * MS:
            tx += G.link_pulse(t_p)
        t_p += 8 * MS
    return G.merge(tx), G.merge(rx), t_end


def long_frame_record(after=63.7 * MS):
    """A partner link test pulse at 0.5 ms, a request at 1 ms, a frame of 1518 octets for another station at 5 ms,
    then silence and a request `after` ns after that frame's last bit; the station answers each request 20 us after
    it and sends link test pulses every 16 ms. Figure 14-6 restarts link_loss while RD is active, so it counts from
    when RD goes idle at the long frame's end (14.2.3.2), 1.2 ms after its start."""
    rx = list(G.link_pulse(0.5e6))
    r1 = G.frame(eth.BCAST, PEER_MAC, 0x0806, G.arp(1, PEER_MAC, G.PEER_IP, bytes(6), IP))
    ch, e1 = G.manchester(G.preamble(56) + r1, 1e6)
    rx += ch
    big = G.frame(G.OTHER_MAC, PEER_MAC, 0x88B5, bytes(range(256)) * 5 + bytes(220))
    ch, e2 = G.manchester(G.preamble(56) + big, 5e6)
    rx += ch
    spa2 = bytes([192, 168, 0, 98])
    r2 = G.frame(eth.BCAST, PEER_MAC, 0x0806, G.arp(1, PEER_MAC, spa2, bytes(6), IP))
    ch, e3 = G.manchester(G.preamble(56) + r2, e2 + after)
    rx += ch
    t_end = e3 + 20 * MS
    tx, t_p = [], 0.4e6
    for t_r, spa in ((e1 + GAP, G.PEER_IP), (e3 + GAP, spa2), (t_end + 1, None)):
        while t_p < t_r - 1e3 and t_p < t_end:
            tx += G.link_pulse(t_p)
            t_p += eth.NLP
        if spa is None:
            break
        rb = reply_bits(spa)
        ch, end = G.manchester(rb, t_r)
        tx += ch
        t_p = (end - BT / 2 if rb[-1] == 1 else end) + eth.NLP
    return G.merge(tx), G.merge(rx), t_end


def jab_judge(rec, **jab):
    tx, rx, t_end = rec
    return eth_check.check(tx, rx, mac=MAC, ip=IP, t_start=0.0, t_end=t_end, link=dict(power_on='pass'),
                           jabber=jab)


JAB_ALLOWED = ('frame_size', 'fcs', 'arp_reply', 'manchester', 'tp_idl', 'edge_timing')


def jab_fails(rep):
    """Failing checks, less those a jabbering station's garbage frame fails anyway."""
    return sorted({c for _, c, _ in rep.failed() if c not in JAB_ALLOWED})


# ---------------------------------------------------------------------------------------------------------------

def main(log):
    log.group(*PROGRAMS, title='link integrity and jabber checker self-test: a reference station, broken ones')
    # 1. the status of every request, 802.3's ranges and stated values
    bad = []
    for case in eth.LINK_CASES:
        tx, rx, t_end, _ = ref_record(case)
        got = statuses(judge(tx, rx, t_end, power_on='pass'))
        if got != EXPECT[case]:
            bad.append('%s: %s, not %s' % (case, got, EXPECT[case]))
    for po, want in (('fail', ['down', 'up']), ('either', ['either', 'up'])):
        tx, rx, t_end, _ = ref_record('power_on', power_on='fail')
        got = statuses(judge(tx, rx, t_end, power_on=po))
        if got != want:
            bad.append('power_on, %s: %s, not %s' % (po, got, want))
    for case, want in EXPECT_STATED.items():
        tx, rx, t_end, _ = ref_record(case, **INSIDE)
        got = statuses(judge(tx, rx, t_end, power_on='pass', **STATED))
        if got != want:
            bad.append('%s, stated values: %s, not %s' % (case, got, want))
    log.check('every request of the %d partner cases gets the link status 14.2.1.7 decides for every allowed MAU '
              '(and for stated values, and both power-on states)' % len(eth.LINK_CASES), not bad,
              '; '.join(bad[:3]) or 'as expected')
    # 2. good records: the reference inside 802.3's ranges, every case, both power-on states
    bad, n = [], 0
    for case in eth.LINK_CASES:
        for p in SWEEP:
            for po in ('pass', 'fail'):
                kw = dict(p, power_on=po, hold=(case == 'hold'), delay=(170 * MS if case == 'hold' else 0.0))
                tx, rx, t_end, handed = ref_record(case, **kw)
                rep = judge(tx, rx, t_end, power_on='either')
                n += 1
                st = statuses(rep)
                wrong = [i for i, h in handed.items() if (h and st[i] == 'down') or (not h and st[i] == 'up')]
                if not rep.ok or wrong:
                    bad.append('%s %s %s: %s' % (case, p, po, rep.failed()[:1] or 'request %s against %s' % (
                        wrong, st)))
    log.check('a station that follows Figure 14-6 with values at the ends and middle of 802.3\'s ranges, %d records '
              '(every case, %d value sets, both power-on states, judged with either allowed): every check passes, '
              'and no request it answered is "down", none it dropped "up"' % (n, len(SWEEP)), not bad,
              '; '.join(map(str, bad[:2])) or '%d records' % n)
    bad = []
    for case in eth.LINK_CASES:
        tx, rx, t_end, _ = ref_record(case, hold=(case == 'hold'), delay=(170 * MS if case == 'hold' else 0.0),
                                      **INSIDE)
        rep = judge(tx, rx, t_end, power_on='pass', **STATED)
        if not rep.ok:
            bad.append('%s: %s' % (case, rep.failed()[:1]))
    log.check('a station inside the stated values (link_loss 64.2 to 65.6 ms, link_test_min 3.9 to 5.3, link_test_max '
              '38.0 to 39.4, lc_max 2) passes every check on every case when held to them', not bad,
              '; '.join(map(str, bad[:2])) or '%d cases' % len(eth.LINK_CASES))
    # 3. broken stations
    for what, case, kw, want, jkw in BAD:
        kw = dict(kw)
        if case == 'hold':
            kw.setdefault('hold', False)
        tx, rx, t_end, _ = ref_record(case, **kw)
        j = dict(jkw)
        po = j.pop('power_on', 'pass')
        rep = judge(tx, rx, t_end, power_on=po, **j)
        fails = rep.failing_checks()
        log.check('a broken station is caught: %s (case %s) fails %s' % (what, case, want), want in fails,
                  'fails %s%s' % (fails or 'nothing', '; ' + rep.failed(want)[0][2] if want in fails else ''))
    # 4. the half-duplex checker: the same judgment, every attempt a transmission
    bad = []
    for case in ('frame_in_fail', 'loss_pulses', 'hold'):
        tx, rx, t_end, _ = ref_record(case, hold=(case == 'hold'), delay=(170 * MS if case == 'hold' else 0.0))
        rep = judge(tx, rx, t_end, power_on='pass', hd=True)
        if not rep.ok or statuses(rep) != EXPECT[case]:
            bad.append('%s: %s %s' % (case, rep.failed()[:1], statuses(rep)))
    tx, rx, t_end, _ = ref_record('frame_in_fail', answer_dropped=True)
    caught = 'link' in judge(tx, rx, t_end, power_on='pass', hd=True).failing_checks()
    log.check('through eth_hd_check.check: three good cases pass with the expected statuses, and an answer to a frame '
              'its MAU dropped fails link', not bad and caught, '; '.join(bad) or 'caught %s' % caught)
    tx, rx, t_end = late_record()
    rep = eth_check.check(tx, rx, mac=MAC, ip=IP, t_start=0.0, t_end=t_end, link=dict(power_on='pass'))
    late = [x for x in rep.findings if x.check == 'link' and x.ok is False]
    tx, rx, t_end = late_record(start=100 * MS)
    rep2 = eth_check.check(tx, rx, mac=MAC, ip=IP, t_start=0.0, t_end=t_end, link=dict(power_on='pass'))
    early = [x for x in rep2.findings if x.check == 'link' and x.ok is False]
    log.check('a transmission that starts 140 ms after the partner\'s last activity, while some allowed MAU is still '
              'in Link Test Pass, and goes on 30 ms past every allowed link_loss fails link; started at 100 ms, it '
              'passes', late and 'leaves Link Test Pass before it ends' in late[0].text and not early,
              '%s; %s' % (late[0].text if late else 'not caught', early[0].text if early else 'the 100 ms one passes'))
    tx, rx, t_end = long_frame_record()
    rep = judge(tx, rx, t_end, power_on='pass', **STATED)
    st = statuses(rep)
    log.check('link_loss counts from the end of a 1518-octet frame, when RD goes idle (Figure 14-6, 14.2.3.2), not '
              'from its start: a request 63.7 ms after it, under the stated 64.2, is "up" for every allowed MAU, and '
              'the station that answers it passes', rep.ok and st == ['up', 'up', 'up'],
              'statuses %s; fails %s' % (st, rep.failed()[:1] or 'nothing'))
    # 5. jabber
    rep = jab_judge(jab_record(), **JAB_STATED)
    f = jab_fails(rep)
    cut = [x for x in rep.findings if x.check == 'jabber' and 'after a cut' in x.text]
    log.check('jabber: a transmission cut 30 ms after its start (stated xmit_max 25 to 35 ms), TP_IDL and link test '
              'pulses during the unjab wait, the next transmission 600 ms later (stated unjab 450 to 550): jabber, '
              'link and link_period pass', not f and cut and all(x.ok for x in cut), 'fails %s; %s' % (
                  f or 'nothing', cut[0].text if cut else 'no cut found'))
    for what, rec, jab, want in (
            ('a transmission of 160 ms, past 802.3\'s longest xmit_max (no stated values)', jab_record(160 * MS),
             {}, 'jabber'),
            ('the next transmission 100 ms after a cut (stated unjab 450 to 550 ms)', jab_record(next_after=100 * MS),
             JAB_STATED, 'jabber'),
            ('the next transmission 430 ms after a cut (stated unjab 450 to 550 ms)', jab_record(next_after=430 * MS),
             JAB_STATED, 'jabber'),
            ('no link test pulse during the unjab wait', jab_record(pulses_in_unjab=False), JAB_STATED,
             'link_period'),
            ('a stated xmit_max of 10 to 15 ms, outside 802.3\'s 20 to 150, with a cut at 12 ms that fits it',
             jab_record(12 * MS), dict(xmit_max=(10 * MS, 15 * MS)), 'jabber')):
        fails = jab_fails(jab_judge(rec, **jab))
        log.check('jabber: a broken station is caught: %s fails %s' % (what, want), want in fails,
                  'fails %s' % (fails or 'nothing'))
