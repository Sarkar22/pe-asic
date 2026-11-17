"""Board front ends for the 10BASE-T and USB low-speed demos, as models the benches import.

Python 3 standard library only. Times are in nanoseconds. A signal is a list of (time_ns, level) changes,
sorted, with a stated level before the first change. The chip runs at 50 MHz, so one cycle is 20 ns.

Ethernet: an RS-485-class transceiver with DI, DE and RO, between two shift outputs of the chip (output A to
DI, output B to DE) and an input pin (RO), and the magnetics. Numbers are from the Renesas ISL3170E to
ISL3178E data sheet, FN6307 rev 6.05 (2023-09-29), the 20 Mbps columns (ISL3176E, ISL3177E, ISL3178E).

USB: D+ and D- of a low-speed device on two uio pins of the chip, the device's pull-up on D-, the host's
pull-downs, and a host model driving the other end. Numbers are from USB 2.0 (2000-04-27) chapter 7, with the
pull-up and pull-down resistor ECN.

Assumptions, every one:

Ethernet transmit (Rs485Eth.tx_line)
  E-a  The part has DE. The ISL3177E itself has no DE or RE (FN6307 Table 1; Note 13 says the enable timings
       don't apply to it): its driver is always on, so the pair would never be silent. A board that wants
       silence, TP_IDL and link pulses from DE needs the ISL3176E, the same family with DE and RE. Its enable
       and disable times are used here. (A board with the ISL3177E must make silence another way, such as
       series capacitors, which this model doesn't cover.)
  E-b  RE is tied low, so the receiver is always on and the part never enters shutdown (FN6307 Note 10:
       shutdown needs RE high and DE low for 600 ns). The enable from shutdown, up to 700 ns, never applies.
  E-c  DI high drives TD+ positive (Y high, Z low: the truth table), and Y goes to TD+ through the magnetics.
  E-d  The magnetics are an ideal 1:1 coupler: no droop, no DC blocking, no ringing. The
       line is +1 while the driver drives DI = 1, -1 while it drives DI = 0, and 0 (silence) while it is
       disabled. A real transformer droops and undershoots at the start of TP_IDL and after a link pulse.
  E-e  The differential output is driven from DE high + max(tZH, tZL), and silent from DE low + max(tHZ, tLZ).
       The data sheet measures each output to 50% on enable and to 0.25 V from the driven level on disable,
       into 500 ohm and 50 pF (Figure 6); the real decay into the magnetics is not modelled.
  E-f  A DI change reaches the line after tDD; tSKEW = abs(tPLH - tPHL) is split evenly: rising + tSKEW/2,
       falling - tSKEW/2. Rise and fall time (9 ns typ) is folded into tDD, which the data sheet measures at
       the crossing.
  E-g  Corners: 'typ' takes every typical; 'max' every maximum; 'min' the data sheet's minimum where it gives
       one (receiver 25 ns) and 0 ns where it gives none (tDD, tZH, tZL, tHZ, tLZ, both skews). 'min' is a bound,
       not a part the data sheet promises.
  E-h  A pad driven by the chip changes t_out ns after the clock edge (default 0; the CMOS5L pads have no
       characterized delay). A pad that is not driven (a uio pin with OE 0) reads as the level the board's
       resistor gives it: de_z for DE (default 0, a pull-down, so the driver is off through reset) and di_z for DI.

Ethernet receive (Rs485Eth.ro)
  E-i  RD+ goes to the receiver's B input and RD- to A (rx_invert=True, as in common hobby designs): A - B is
       minus the line, so RO is 0 while the line is HI and 1 while it is LO or silent. The full fail-safe
       threshold (RO high for A - B >= -50 mV, low for <= -200 mV; FN6307 truth table) makes an idle line read 1.
       With rx_invert=False RO is 0 only while the line is LO, and a link pulse or TP_IDL doesn't show on RO.
  E-j  The partner's levels are far beyond both thresholds (802.3 asks a receiver to accept 585 mV), so the
       undetermined band between -200 and -50 mV is crossed only during a transition, taken as instantaneous.
       Hysteresis (15 mV) and noise are not modelled.
  E-k  RO lags the line by tPLH or tPHL (25 / 33 / 60 ns); the receiver skew (1.5 typ, 4 max) is split
       evenly: RO falling + skew/2, RO rising - skew/2. Pulses of any width pass (the part is rated 20 Mbps
       minimum; 10BASE-T's shortest pulse is one half bit, 50 ns).

USB (UsbLs)
  U-a  The chip is a low-speed device: its pull-up (r_pu, default 1.5 kohm) on D- to v_term (3.3 V), and the
       host's pull-downs (r_pd, 15 kohm) on both lines (USB 7.1.5.1, Figure 7-21; ECN ranges 900 to 1575 ohm
       and 14.25 to 24.80 kohm).
  U-b  A line that nobody drives settles at its pulled level: D- at v_term * r_pd / (r_pd + r_pu) (3.0 V), D+ at
       0 V. Released from the other level, it reads the new level once it crosses the single-ended threshold:
       VIH = 2.0 V rising, VIL = 0.8 V falling (Table 7-7), with time constant (r_pu || r_pd) * c_line for D-
       and r_pd * c_line for D+. c_line defaults to 450 pF, the most a low-speed device with its cable may
       have (CLINUA, Table 7-10), so the pull-up is at its slowest: 674 ns from 0 V to VIH.
  U-c  A driven line is at the driver's level at once: the chip's pads and the host's driver are fast enough
       that, at 50 MHz sampling, edge rates don't matter. (USB asks 75 to 300 ns edges of a low-speed driver,
       Table 7-10; that is board work, not modelled.)
  U-d  The model is at the device's pins: the host's changes arrive t_cable ns after the host model makes
       them (default 0; a low-speed cable is under 18 ns, 7.1.1.2). The chip's pad changes come t_out ns after
       its clock edge (default 0). dm_lag delays every change the chip makes on D- by that many ns, to put
       short SE0 or SE1 states on the bus at J-K transitions (USB lets receivers ignore up to 210 ns, TLST).
  U-e  If both ends drive one line, the bus is in contention: the interval is logged, and the line takes the
       chip's level. Series resistors don't change the logic levels and aren't modelled.
"""
import bisect
import heapq
import math

CYCLE_NS = 20.0                 # 50 MHz


# ---------------------------------------------------------------------------------------------------------------
# pins and sampling

def pin_changes(t, v, bit, oe_bit=None, z=None, cycle_ns=CYCLE_NS, t_out=0.0):
    """One pad's changes from a run-length record (lists t of cycles and v of words, as rig.Trace keeps them):
    bit is the bit of the pad's level in the word, oe_bit the bit of its output enable (for a uio pin), and z
    the level to use while OE is 0. Returns [(time_ns, level)], level 0 or 1, from the first record."""
    out = []
    for c, w in zip(t, v):
        lv = (w >> bit) & 1
        if oe_bit is not None and not (w >> oe_bit) & 1:
            lv = z
        if not out or out[-1][1] != lv:
            out.append((c * cycle_ns + t_out, lv))
    return out


# Bit positions in a recorded word, as rig.Trace and programs/sigrok/check.py keep them: uo, uio_out << 8,
# uio_oe << 16, ui << 24, and the uio lines (what the board sees) << 32.
REC_SHIFT = dict(uo=0, ioout=8, iooe=16, ui=24, io=32)


def pad(t, v, name, z=None, cycle_ns=CYCLE_NS, t_out=0.0):
    """One pad's changes from a run-length record in that layout. 'uo4' is an output pad; 'io2' is what the chip
    drives on uio pin 2 (z while its OE is 0); 'ui3' and 'line2' are the levels the board gave the chip on ui pin 3
    and on uio line 2."""
    n = int(name.lstrip('abcdefghijklmnopqrstuvwxyz'))
    if name.startswith('uo'):
        return pin_changes(t, v, REC_SHIFT['uo'] + n, cycle_ns=cycle_ns, t_out=t_out)
    if name.startswith('io'):
        return pin_changes(t, v, REC_SHIFT['ioout'] + n, REC_SHIFT['iooe'] + n, z, cycle_ns, t_out)
    if name.startswith('ui'):
        return pin_changes(t, v, REC_SHIFT['ui'] + n, cycle_ns=cycle_ns)
    if name.startswith('line'):
        return pin_changes(t, v, REC_SHIFT['io'] + n, cycle_ns=cycle_ns)
    raise ValueError('pad name %r' % name)


def level_at(changes, initial, t):
    """The level of a change list at time t."""
    i = bisect.bisect_right([c[0] for c in changes], t) - 1
    return changes[i][1] if i >= 0 else initial


class Sampler:
    """The level of a change list at each clock edge, for driving a chip input cycle by cycle:
    s(c) is the level at c * cycle_ns + phase_ns. Calls should come in rising c for speed."""

    def __init__(self, changes, initial, cycle_ns=CYCLE_NS, phase_ns=0.0):
        self.t = [c[0] for c in changes]
        self.v = [c[1] for c in changes]
        self.initial, self.cycle, self.phase = initial, cycle_ns, phase_ns
        self.i = -1

    def __call__(self, c):
        t = c * self.cycle + self.phase
        i = self.i
        if i >= 0 and self.t[i] > t:
            i = bisect.bisect_right(self.t, t) - 1
        while i + 1 < len(self.t) and self.t[i + 1] <= t:
            i += 1
        self.i = i
        return self.v[i] if i >= 0 else self.initial


# ---------------------------------------------------------------------------------------------------------------
# Ethernet: RS-485 transceiver and magnetics

class Rs485Eth:
    """FN6307, 20 Mbps columns. Values in ns. See assumptions E-a to E-k above."""
    CORNERS = {
        'typ': dict(t_dd=27.0, skew_dd=1.0, t_zh=17.0, t_zl=16.0, t_hz=25.0, t_lz=28.0, t_rx=33.0, skew_rx=1.5),
        'max': dict(t_dd=40.0, skew_dd=3.0, t_zh=50.0, t_zl=40.0, t_hz=40.0, t_lz=50.0, t_rx=60.0, skew_rx=4.0),
        'min': dict(t_dd=0.0, skew_dd=0.0, t_zh=0.0, t_zl=0.0, t_hz=0.0, t_lz=0.0, t_rx=25.0, skew_rx=0.0),
    }

    def __init__(self, corner='typ', rx_invert=True, de_z=0, di_z=0, **over):
        p = dict(self.CORNERS[corner])
        p.update(over)
        self.p = p
        self.rx_invert, self.de_z, self.di_z = rx_invert, de_z, di_z

    def tx_line(self, di, de, di0=0, de0=0):
        """The TD pair, from the changes of DI and DE (levels 0, 1 or None for undriven). di0 and de0 are the
        levels before the first change. Returns [(time_ns, level)], level +1, -1 or 0, silent before."""
        p = self.p
        t_en = max(p['t_zh'], p['t_zl'])
        t_dis = max(p['t_hz'], p['t_lz'])
        rise, fall = p['t_dd'] + p['skew_dd'] / 2, p['t_dd'] - p['skew_dd'] / 2

        def fix(lv, z):
            return z if lv is None else lv
        dd, prev = [], fix(di0, self.di_z)
        for t, lv in di:
            lv = fix(lv, self.di_z)
            if lv != prev:
                dd.append((t + (rise if lv else fall), lv))
                prev = lv
        dd.sort()
        en, prev = [], fix(de0, self.de_z)
        for t, lv in de:
            lv = fix(lv, self.de_z)
            if lv != prev:
                en.append((t + (t_en if lv else t_dis), lv))
                prev = lv
        en.sort()
        d0 = fix(di0, self.di_z)
        e0 = fix(de0, self.de_z)
        times = sorted({t for t, _ in dd} | {t for t, _ in en})
        dd_t, en_t = [c[0] for c in dd], [c[0] for c in en]    # level_at's lookups, built once

        def at(ch, ch_t, initial, t):
            i = bisect.bisect_right(ch_t, t) - 1
            return ch[i][1] if i >= 0 else initial
        last = (1 if d0 else -1) if e0 else 0
        out = [(0.0, last)] if last else []         # driven from the start: shown from time 0
        for t in times:
            d = at(dd, dd_t, d0, t)
            e = at(en, en_t, e0, t)
            lv = (1 if d else -1) if e else 0
            if lv != last:
                out.append((t, lv))
                last = lv
        return out

    def ro(self, line):
        """RO from the RD pair (changes of +1, -1, 0; silent before the first). 1 before the first change."""
        p = self.p
        active = 1 if self.rx_invert else -1      # the line level that pulls RO low
        t_fall, t_rise = p['t_rx'] + p['skew_rx'] / 2, p['t_rx'] - p['skew_rx'] / 2
        out, last = [], 1
        for t, lv in sorted(line):
            ro = 0 if lv == active else 1
            if ro != last:
                out.append((t + (t_fall if ro == 0 else t_rise), ro))
                last = ro
        out.sort()
        # a pulse shorter than the skew could reorder edges: keep the order of the line
        clean, last = [], 1
        for t, lv in out:
            if lv != last:
                clean.append((t, lv))
                last = lv
        return clean


# ---------------------------------------------------------------------------------------------------------------
# USB low speed: the bus between the chip and a host model

class UsbLs:
    """D+ and D- at the device's pins. Feed it the chip's pads with device() and the host's drive with host(),
    in rising time for each side (the host may schedule changes ahead); read the bus with level(t), in rising t;
    get the whole record with record(t_end). See assumptions U-a to U-e above."""

    def __init__(self, r_pu=1500.0, r_pd=15000.0, c_line=450e-12, v_term=3.3, v_ih=2.0, v_il=0.8, t_cable=0.0,
                 t_out=0.0, dm_lag=0.0):
        v_idle = v_term * r_pd / (r_pd + r_pu)
        tau_dm = (r_pu * r_pd / (r_pu + r_pd)) * c_line * 1e9
        tau_dp = r_pd * c_line * 1e9
        self.t_dm_up = tau_dm * math.log(v_idle / (v_idle - v_ih))      # D- from 0 V to VIH
        self.t_dp_down = tau_dp * math.log(v_term / v_il)               # D+ from driven high to VIL
        self.t_cable, self.t_out, self.dm_lag = t_cable, t_out, dm_lag
        self.ev = []                     # heap of (time, sequence, kind, data)
        self.seq = 0
        self.dev = [0, 0, 0, 0]          # dp, dm, oe_dp, oe_dm
        self.hst = [0, 1, 0]             # dp, dm, drive
        self.line = [0, 1]               # resolved D+, D-
        self.rel = [None, None]          # generation of a pending pull for each line
        self.gen = 0
        self.dp_rec, self.dm_rec = [], []
        self.dev_rec, self.host_rec = [], []
        self.contention = []
        self._cont_since = None
        self._drive = [0, 0]             # last recorded: device drives, host drives
        self._last_dev = None

    def _push(self, t, kind, data):
        self.seq += 1
        heapq.heappush(self.ev, (t, self.seq, kind, data))

    def device(self, t, dp, dm, oe_dp, oe_dm):
        """The chip's pads from time t (the clock edge's time; t_out is added here). Repeats are ignored."""
        state = (dp, dm, oe_dp, oe_dm)
        if state == self._last_dev:
            return
        self._last_dev = state
        t = t + self.t_out
        if self.dm_lag:
            self._push(t, 'dev_dp', (dp, oe_dp))
            self._push(t + self.dm_lag, 'dev_dm', (dm, oe_dm))
        else:
            self._push(t, 'dev', state)

    def host(self, t, dp, dm, drive):
        """The host drives (dp, dm) from time t when drive is true, and releases both lines when it is false."""
        self._push(t + self.t_cable, 'host', (dp, dm, drive))

    def _advance(self, t):
        while self.ev and self.ev[0][0] <= t:
            te = self.ev[0][0]
            pulls = []
            while self.ev and self.ev[0][0] == te:
                _, _, kind, data = heapq.heappop(self.ev)
                if kind == 'dev':
                    self.dev = list(data)
                elif kind == 'dev_dp':
                    self.dev[0], self.dev[2] = data
                elif kind == 'dev_dm':
                    self.dev[1], self.dev[3] = data
                elif kind == 'host':
                    self.hst = list(data)
                else:
                    pulls.append(data)
            new = list(self.line)
            for i, g in pulls:
                if self.rel[i] == g:
                    self.rel[i] = None
                    new[i] = 1 if i == 1 else 0
            for i in (0, 1):
                dd, dl, hd, hl = self.dev[2 + i], self.dev[i], self.hst[2], self.hst[i]
                if dd or hd:
                    new[i] = dl if dd else hl
                    self.rel[i] = None
                else:
                    pulled = 1 if i == 1 else 0
                    if new[i] == pulled:
                        self.rel[i] = None
                    elif self.rel[i] is None:
                        self.gen += 1
                        self.rel[i] = self.gen
                        self._push(te + (self.t_dm_up if i == 1 else self.t_dp_down), 'pull', (i, self.gen))
            for i, rec in ((0, self.dp_rec), (1, self.dm_rec)):
                if new[i] != self.line[i]:
                    rec.append((te, new[i]))
            self.line = new
            cont = any(self.dev[2 + i] and self.hst[2] and self.dev[i] != self.hst[i] for i in (0, 1))
            if cont and self._cont_since is None:
                self._cont_since = te
            elif not cont and self._cont_since is not None:
                self.contention.append((self._cont_since, te))
                self._cont_since = None
            drive = [1 if (self.dev[2] or self.dev[3]) else 0, 1 if self.hst[2] else 0]
            for k, rec in ((0, self.dev_rec), (1, self.host_rec)):
                if drive[k] != self._drive[k]:
                    rec.append((te, drive[k]))
            self._drive = drive

    def level(self, t):
        """(D+, D-) at time t."""
        self._advance(t)
        return self.line[0], self.line[1]

    def record(self, t_end):
        """Everything up to t_end, for usb_check: D+ and D- changes (D+ low and D- high before the first), when
        the device and the host drive (0 before the first), and the contention intervals."""
        self._advance(t_end)
        cont = list(self.contention)
        if self._cont_since is not None:
            cont.append((self._cont_since, t_end))
        return dict(dp=list(self.dp_rec), dm=list(self.dm_rec), dev=list(self.dev_rec),
                    host=list(self.host_rec), contention=cont, t_end=t_end)
