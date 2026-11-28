"""Link integrity and jabber checker for a 10BASE-T station, written from IEEE Std 802.3 clause 14 before any firmware
that implements those functions: 14.2.1.7 and Figure 14-6 (Link Integrity Test), 14.2.1.6 and Figure 14-5 (Jabber),
with 14.2.1.1 and 14.2.2.2 (TD_idle is TP_IDL, link test pulses included), Figure 14-3 (xmit=disable sends TD_idle,
and ends a transmission under way), 14.2.3.1 (RD is idle during silence and link test pulses), 14.2.3.2 (the timers),
14.3.1.3.2 (what a receiver takes as a link test pulse, and rejects as RD_input) and 14.3.1.3.3 (idle within 2.3 BT
of the last low-to-high transition). Python 3 standard library only.

eth_check.check and eth_hd_check.check call apply() when given link= or jabber=. It adds 'link' and 'jabber'
findings to their report and marks every received frame with what the station's MAU may have done with it, which
their reply check then follows. Without link= and jabber= nothing changes.

The MAU's state isn't on the pairs, and 802.3 gives ranges, not values: link_loss 50 to 150 ms, link_test_min 2 to
7 ms, link_test_max 25 to 150 ms, lc_max 2 to 10, and power on in LINK TEST PASS or, optionally, LINK TEST FAIL
RESET (14.2.1.7). So the checker runs Figure 14-6 over the receive pair's events for every MAU those ranges allow,
every start of a timer with any duration in its range (a timer's tolerance) and lc_max one fixed integer, and fails a
record only where no such MAU could have done what it shows. A station may state narrower values, its answers to
the PICS (LF1, LF5 to LF7 and LF16; JF2 and JF4), which must lie inside 802.3's; the checker then holds it to them.

Checks:
  link     no transmission on TD other than a link test pulse unless some allowed MAU is in LINK TEST PASS from its
           first bit to its last (xmit=disable sends TD_idle: Figures 14-3 and 14-6); no reply to a partner frame
           whose reception began while no allowed MAU was in LINK TEST PASS (rcv=disable puts input_idle on DI, so
           the frame never reaches the MAC; the frame that ends the state is such a frame, 14.2.1.7); a partner frame
           that every allowed MAU hands up is judged by the reply check as before, one that only some hand up may be
           answered or not; and the stated values lie inside 802.3's ranges
  jabber   no transmission on TD lasts longer than xmit_max, 150 ms or the stated upper bound (14.2.1.6, Figure
           14-5); with a stated xmit_max, a transmission that reaches its lower bound was cut by the Jabber function,
           and no transmission starts within unjab of the cut, 0.25 s or the stated lower bound (TD stays TD_idle
           until DO has been idle for unjab, and DO can't go idle before the cut)
Link test pulses on TD go on in every state, Link Test Fail and jabber included (TD_idle is TP_IDL, 14.2.2.2,
14.2.1.1): eth_check's link_period check judges that on every record.

The receive pair's bursts, as 14.2.3.1 and 14.3.1.3.2 sort them: a positive pulse of 0.85 to 2.0 BT, with any
negative tail over by 4.0 BT from its start (Figure 14-13 at any amplitude), is a link test pulse (link_test_rcv);
a burst with 3 or more sign changes inside it is RD activity (RD=active), from its first level until it goes idle,
within 2.3 BT of its last low-to-high transition; any other burst, a single cycle for one, is not RD activity and may
or may not be taken as a link test pulse. A transmission is DO activity from its first level to the start of its
TP_IDL.

    rep = eth_check.check(tx, rx, ..., link=dict(power_on='pass'), jabber={})
    link=  power_on ('pass', 'fail' for LINK TEST FAIL RESET, or 'either'), and optionally link_loss,
           link_test_min, link_test_max as (low, high) in ns, lc_max as (low, high)
    jabber= optionally xmit_max and unjab as (low, high) in ns
    rf.link                 for each received frame: 'up' (every allowed MAU hands it up), 'down' (none does), 'either'
    rep.link_states         (time, what, 'up' / 'down' / 'either') at every received burst and transmission
"""
HI, LO = 1, -1
MS = 1e6
STANDARD = dict(link_loss=(50 * MS, 150 * MS), link_test_min=(2 * MS, 7 * MS), link_test_max=(25 * MS, 150 * MS),
                lc_max=(2, 10), xmit_max=(20 * MS, 150 * MS), unjab=(250 * MS, 750 * MS))
IDLE_BT = 2.3               # 14.3.1.3.3: idle within 2.3 BT of the last low-to-high transition
PULSE_HI = (0.85, 2.0)      # BT: Figure 14-13's HI from its largest amplitude to its smallest
PULSE_LO_END = 4.0          # BT: Figure 14-13's negative tail over by then at any amplitude
EPS = 1e-6                  # ns, for comparing float times


def _ranges(given, names, rep, check):
    """802.3's range for each name, or the stated one, which must lie inside it."""
    out = {}
    for n in names:
        lo, hi = STANDARD[n]
        if given and n in given and given[n] is not None:
            a, b = given[n]
            ok = lo <= a <= b <= hi
            unit = 1 if n == 'lc_max' else MS
            rep.add(0.0, check, ok, 'stated %s %s to %s%s, %s 802.3\'s %s to %s (14.2.1.6, 14.2.1.7)' % (
                n, _fmt(a / unit), _fmt(b / unit), '' if n == 'lc_max' else ' ms', 'inside' if ok else 'OUTSIDE',
                _fmt(lo / unit), _fmt(hi / unit)))
            out[n] = (a, b) if ok else (lo, hi)
        else:
            out[n] = (lo, hi)
    return out


def _fmt(x):
    return ('%d' % x) if x == int(x) else ('%.3f' % x)


# ---------------------------------------------------------------------------------------------------------------
# the pairs' events

def _is_link_pulse(p, bt):
    lv = p.levels
    if not lv or lv[0][1] != HI:
        return False
    w = p.width / bt
    if not PULSE_HI[0] - EPS <= w <= PULSE_HI[1] + EPS:
        return False
    if len(lv) == 1:
        return True
    if len(lv) == 2 and lv[1][1] == LO:
        return (p.stop - p.start) / bt <= PULSE_LO_END + EPS
    return False


def _last_rise(fr):
    """The last low-to-high transition of a burst: TP_IDL's start, or None."""
    for q, (t, v) in enumerate(fr.tail):
        if v == HI and (q > 0 or (fr.bits and fr.bits[-1] == 1)):
            return t
    rises = [t for t, v in fr.tail if v == HI]
    return rises[-1] if rises else None


def _rd_events(rep, bt):
    """(start, kind, (idle_lo, idle_hi), burst) for every burst on the receive pair, in time order. kind: 'pulse',
    'frame' (RD activity) or 'other'."""
    ev = []
    for p in rep.rx_pulses:
        ev.append((p.start, 'pulse' if _is_link_pulse(p, bt) else 'other', (p.stop, p.stop), p))
    for fr in rep.rx_frames:
        if len(fr.inner) >= 3:
            r = _last_rise(fr)
            if r is None:
                r = fr.inner[-1]
            ev.append((fr.start, 'frame', (r, min(r + IDLE_BT * bt, max(fr.stop, r))), fr))
        else:
            ev.append((fr.start, 'other', (fr.stop, fr.stop), fr))
    ev.sort(key=lambda e: e[0])
    return ev


def _tx_end(fr):
    """The end of DO activity: the start of the transmission's TP_IDL, or its last bit, or its return to silence."""
    if getattr(fr, 't_idl', None) is not None:
        return fr.t_idl
    if fr.end is not None:
        return fr.end
    return fr.stop


# ---------------------------------------------------------------------------------------------------------------
# Figure 14-6 for every allowed MAU. A configuration is (state, count, lo, hi, c0, c1, tx):
#   'P'  LINK TEST PASS: link_loss_timer and link_test_min_timer restarted at some time in [lo, hi]
#   'F'  LINK TEST FAIL (RESET and COUNT last only while a pulse does): link_count = count, link_test_min_timer and
#        link_test_max_timer restarted at some time in [lo, hi]
#   'X'  LINK TEST FAIL EXTEND: it enters LINK TEST PASS at some time in [lo, hi] (RD and DO idle)
#   lc_max is some integer in [c0, c1]; tx is 1 while a transmission is under way that this MAU has kept in LINK TEST
#   PASS from its first bit

class _Model:
    def __init__(self, r, power_on, t0):
        self.L, self.m, self.M = r['link_loss'], r['link_test_min'], r['link_test_max']
        c = r['lc_max']
        self.do_until = None                    # DO is active until then (a transmission under way)
        self.cfg = set()
        if power_on in ('pass', 'either'):
            self.cfg.add(('P', 0, t0, t0, c[0], c[1], 0))
        if power_on in ('fail', 'either'):
            self.cfg.add(('F', 0, t0, t0, c[0], c[1], 0))

    # -- the configurations just before time t, before an event at t is applied
    def before(self, t):
        out = []
        for g in self.cfg:
            out += self._before(g, t)
        return _merge(out)

    def _before(self, g, t):
        s = g[0]
        if s == 'X':
            out = []
            if t > g[2] + EPS:                  # it may have entered LINK TEST PASS already
                out += self._before(('P', 0, g[2], min(g[3], t), g[4], g[5], 0), t)
            if t <= g[3] + EPS:                 # it may still be waiting
                out.append(('X', 0, max(g[2], t), g[3], g[4], g[5], 0))
            return out
        if s == 'P':
            lo, hi = g[2], g[3]
            out = []
            if t > lo + self.L[0] + EPS:        # link_loss_timer may have run out: LINK TEST FAIL RESET, then FAIL
                out += self._f_before(('F', 0, lo + self.L[0], min(t, hi + self.L[1]), g[4], g[5], 0), t)
            if t <= hi + self.L[1] + EPS:       # or not yet
                out.append(('P', 0, max(lo, t - self.L[1]), hi, g[4], g[5], g[6]))
            return out
        return self._f_before(g, t)

    def _f_before(self, g, t):
        """LINK TEST FAIL just before t: link_test_max_timer may have run out once or more (each time to LINK TEST FAIL
        RESET, count 0, and back with both timers restarted)."""
        _, k, lo, hi, c0, c1, _ = g
        out = []
        if t <= hi + self.M[1] + EPS:
            out.append(('F', k, max(lo, t - self.M[1]), hi, c0, c1, 0))
        j = 1
        while lo + j * self.M[0] < t - EPS:
            a, b = max(lo + j * self.M[0], t - self.M[1]), min(hi + j * self.M[1], t)
            if a <= b + EPS:
                out.append(('F', 0, a, b, c0, c1, 0))
            j += 1
        return out

    def _exit(self, lo, hi):
        """EXTEND's way out once RD goes idle at some time in [lo, hi]: also not before DO is idle."""
        if self.do_until is not None:
            lo, hi = max(lo, self.do_until), max(hi, self.do_until)
        return lo, hi

    # -- the states at t
    def at(self, t):
        """'up' if every allowed MAU is surely in LINK TEST PASS at t, 'down' if none can be, else 'either'."""
        cur = self.before(t)
        can = [g for g in cur if g[0] == 'P']
        if not can:
            return 'down'
        sure = [g for g in can if t < g[2] + self.L[0] - EPS]
        return 'up' if len(sure) == len(cur) else 'either'

    # -- events
    def pulse(self, t0, t1, maybe=False):
        out = []
        for g in self.before(t0):
            if maybe:
                out.append(g)                   # not taken as a link test pulse
            s = g[0]
            if s == 'P':
                # LINK TEST PASS: accepted if link_test_min_timer is done, restarting both timers at its start
                if t0 - g[2] >= self.m[0] - EPS:
                    out.append(('P', 0, t0, t0, g[4], g[5], g[6]))
                if t0 - g[3] < self.m[1] - EPS:
                    out.append(g)               # ignored (14.2.1.7)
            elif s == 'F':
                out += self._f_pulse(g, t0, t1)
            else:
                out.append(g)                   # EXTEND: a pulse is RD=idle
        self.cfg = set(_merge(out))

    def _f_pulse(self, g, t0, t1):
        _, k, a, b, c0, c1, _ = g
        out = []
        if t0 - a >= self.m[0] - EPS:           # link_test_min_timer may be done: LINK TEST FAIL COUNT
            kk = k + 1
            if c0 <= kk <= c1:                  # link_count = lc_max: EXTEND, LINK TEST PASS once RD and DO are idle
                lo, hi = self._exit(t1, t1)
                out.append(('X', 0, lo, hi, kk, kk, 0))
            if kk < c1:
                out.append(('F', kk, t1, t1, max(c0, kk + 1), c1, 0))
        if t0 - b < self.m[1] - EPS:            # or not: LINK TEST FAIL RESET, count 0
            out.append(('F', 0, t1, t1, c0, c1, 0))
        return out

    def frame(self, t0, idle):
        """RD active from t0 until it goes idle at some time in idle = (lo, hi). Returns the frame's link status."""
        status = self.at(t0)
        out = []
        for g in self.before(t0):
            if g[0] == 'P':                     # both timers restart while RD is active: at its end
                out.append(('P', 0, idle[0], idle[1], g[4], g[5], g[6]))
            else:                               # RD=active: LINK TEST FAIL EXTEND until RD and DO are idle
                lo, hi = self._exit(idle[0], idle[1])
                out.append(('X', 0, lo, hi, g[4], g[5], 0))
        self.cfg = set(_merge(out))
        return status

    def tx_start(self, t0, t1):
        """DO active from t0 to t1. Returns the status at t0; marks the MAUs then in LINK TEST PASS."""
        status = self.at(t0)
        self.do_until = t1
        out = []
        for g in self.before(t0):
            if g[0] == 'P':
                out.append(g[:6] + (1,))
            elif g[0] == 'X':
                out.append(('X', 0, max(g[2], t1), max(g[3], t1), g[4], g[5], 0))
            else:
                out.append(g)
        self.cfg = set(_merge(out))
        return status

    def tx_end(self, t1):
        """Returns 'up' if every allowed MAU stayed in LINK TEST PASS through the transmission, 'down' if none did."""
        cur = self.before(t1)
        kept = [g for g in cur if g[0] == 'P' and g[6]]
        self.cfg = set(_merge([g[:6] + (0,) for g in cur]))
        self.do_until = None
        if not kept:
            return 'down'
        return 'up' if len(kept) == len(cur) else 'either'


def _merge(cfgs):
    """Joins configurations that differ only in overlapping intervals."""
    groups = {}
    for g in cfgs:
        groups.setdefault((g[0], g[1], g[4], g[5], g[6]), []).append((g[2], g[3]))
    out = []
    for (s, k, c0, c1, tx), iv in groups.items():
        iv.sort()
        cur = list(iv[0])
        for a, b in iv[1:]:
            if a <= cur[1] + EPS:
                cur[1] = max(cur[1], b)
            else:
                out.append((s, k, cur[0], cur[1], c0, c1, tx))
                cur = [a, b]
        out.append((s, k, cur[0], cur[1], c0, c1, tx))
    return out


# ---------------------------------------------------------------------------------------------------------------

def apply(rep, o, link=None, jabber=None, t_start=None, t_end=None):
    """Adds the link and jabber findings to rep (eth_check's Report, after the frames are classified and before the
    reply check) and sets .link on every received frame. o: eth_check's options."""
    bt = o['bt']
    rep.link_states = []
    if link is not None:
        _apply_link(rep, bt, link, t_start)
    if jabber is not None:
        _apply_jabber(rep, bt, jabber)


def _apply_link(rep, bt, link, t_start):
    power_on = link.get('power_on', 'either')
    if power_on not in ('pass', 'fail', 'either'):
        raise ValueError('power_on must be pass, fail or either: %r' % power_on)
    r = _ranges(link, ('link_loss', 'link_test_min', 'link_test_max', 'lc_max'), rep, 'link')
    t0 = 0.0 if t_start is None else t_start
    model = _Model(r, power_on, t0)
    ev = [(e[0], 1, 'rd', e) for e in _rd_events(rep, bt)]
    for f in rep.tx_frames:
        t1 = max(_tx_end(f), f.start)
        ev.append((f.start, 2, 'tx', (f, t1)))
        ev.append((t1, 0, 'end', (f, t1)))
    ev.sort(key=lambda e: (e[0], e[1]))
    at_start = {}
    for t, _, what, e in ev:
        if what == 'tx':
            f, t1 = e
            at_start[id(f)] = model.tx_start(f.start, t1)
            continue
        if what == 'end':
            f, t1 = e
            st0, st1 = at_start.pop(id(f)), model.tx_end(t1)
            st = 'down' if 'down' in (st0, st1) else ('up' if st0 == st1 == 'up' else 'either')
            rep.link_states.append((f.start, 'transmission', st))
            if st0 == 'down':
                why = 'NO allowed MAU is in Link Test Pass when it starts (14.2.1.7: xmit=disable sends TD_idle)'
            elif st1 == 'down':
                why = ('every allowed MAU leaves Link Test Pass before it ends (14.2.1.7, Figure 14-3: xmit=disable '
                       'ends a transmission under way)')
            else:
                why = '%s allowed MAU is in Link Test Pass throughout' % ('every' if st == 'up' else 'some')
            rep.add(f.start, 'link', st != 'down', 'transmission %.1f to %.1f us: %s' % (f.start / 1e3, t1 / 1e3, why))
            continue
        start, kind, idle, burst = e
        if kind == 'frame':
            st = model.frame(start, idle)
            burst.link = st
            rep.link_states.append((start, 'frame', st))
            rep.add(start, 'link', None, 'partner frame at %.1f us: %s' % (start / 1e3, {
                'up': 'every allowed MAU hands it up', 'down': 'no allowed MAU hands it up (14.2.1.7: rcv=disable)',
                'either': 'some allowed MAUs hand it up'}[st]))
        else:
            st = model.at(start)
            rep.link_states.append((start, kind, st))
            model.pulse(start, idle[1], maybe=(kind == 'other'))
            if hasattr(burst, 'bits'):
                burst.link = st
    for fr in rep.rx_frames:
        if not hasattr(fr, 'link'):
            fr.link = 'either'


def _apply_jabber(rep, bt, jabber):
    r = _ranges(jabber, ('xmit_max', 'unjab'), rep, 'jabber')
    stated = jabber.get('xmit_max') is not None and r['xmit_max'] == tuple(jabber['xmit_max'])
    xlo, xhi = r['xmit_max']
    ulo = r['unjab'][0]
    tol = 2 * bt                                # the first bit's slack (14.2.1.1) and one bit time of DO to TD
    tx = sorted(rep.tx_frames, key=lambda f: f.start)
    for i, f in enumerate(tx):
        end = max(_tx_end(f), f.start)
        d = end - f.start
        rep.add(f.start, 'jabber', d <= xhi + tol, 'transmission of %.3f ms (at most %.3f ms: 14.2.1.6 xmit_max)' % (
            d / MS, xhi / MS))
        if stated and d >= xlo - tol:
            nxt = tx[i + 1] if i + 1 < len(tx) else None
            if nxt is None:
                rep.add(end, 'jabber', None, 'a transmission cut by the Jabber function at %.3f ms; none after it in '
                        'the record' % (d / MS))
                continue
            gap = nxt.start - end
            rep.add(nxt.start, 'jabber', gap >= ulo - tol, 'the next transmission %.1f ms after a cut at %.3f ms (at '
                    'least unjab, %.0f ms: 14.2.1.6, Figure 14-5)' % (gap / MS, d / MS, ulo / MS))
