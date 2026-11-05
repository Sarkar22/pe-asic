"""Board, host, device models and waveform decoders.

Nothing here knows any program's constants or labels. Device models react to pad
levels only. Decoders take a recorded pin trace plus the protocol's parameters
(baud rate, SPI mode, the spec's timing table) and work from the protocol's rules.

Pin values are recorded for every cycle; the record is run-length stored.
"""
import bisect

CLK = 50_000_000                        # cycles per second, for turning specs into cycles


def us(x):
    return x * CLK / 1e6


def ns(x):
    return x * CLK / 1e9


# ---------------------------------------------------------------------------
# recording

class Trace:
    SHIFT = dict(uo=0, ioout=8, iooe=16, ui=24, io=32)

    def __init__(self):
        self.t, self.v = [], []
        self.end = 0

    def add(self, c, val):
        if not self.v or self.v[-1] != val:
            self.t.append(c)
            self.v.append(val)
        self.end = c + 1

    def wave(self, sig, n):
        s = self.SHIFT[sig] + n
        return self._wave(lambda v: (v >> s) & 1)

    def pull_low(self, n):
        """1 while the chip itself pulls uio pin n low."""
        o, e = self.SHIFT['ioout'] + n, self.SHIFT['iooe'] + n
        return self._wave(lambda v: ((v >> e) & 1) & (1 - ((v >> o) & 1)))

    def drive_high(self, n):
        o, e = self.SHIFT['ioout'] + n, self.SHIFT['iooe'] + n
        return self._wave(lambda v: ((v >> e) & 1) & ((v >> o) & 1))

    def _wave(self, fn):
        ts, vs = [], []
        for t, v in zip(self.t, self.v):
            b = fn(v)
            if not vs or vs[-1] != b:
                ts.append(t)
                vs.append(b)
        return Wave(ts, vs, self.end)

    def projected(self, mask):
        """Run-length record of the chosen bits, for cycle-exact comparison."""
        out = []
        for t, v in zip(self.t, self.v):
            b = v & mask
            if not out or out[-1][1] != b:
                out.append((t, b))
        return out


class Wave:
    def __init__(self, t, v, end):
        self.t, self.v, self.end = t, v, end

    def at(self, c):
        i = bisect.bisect_right(self.t, c) - 1
        return self.v[i] if i >= 0 else 0

    def edges(self, lo=0, hi=None):
        hi = self.end if hi is None else hi
        return [(t, v) for t, v in zip(self.t[1:], self.v[1:]) if lo <= t < hi]


# ---------------------------------------------------------------------------
# board

class Board:
    def __init__(self, chip, devices=(), pullups=0, rise=0, ui_idle=0x01):
        self.chip = chip
        self.devices = list(devices)
        self.pullups = pullups
        self.pu_bits = [b for b in range(8) if (pullups >> b) & 1]
        self.rise = rise                # cycles from release to the line reading high
        self.count = [10 ** 9] * 8
        self.ui_idle = ui_idle          # ui0 is CS of the host link: idle high
        self.trace = Trace()
        self.conflicts = []             # chip drives high while a device pulls low
        self.agents = []

    def agent(self, gen):
        self.agents.append([gen, 0])

    def run(self, cycles, stop=None):
        chip = self.chip
        end = chip.c + cycles
        devs = self.devices
        agents = self.agents
        trace = self.trace
        tt, tv = trace.t, trace.v
        last = tv[-1] if tv else None
        pu_bits = self.pu_bits
        rise = self.rise
        count = self.count
        n_agents, due = len(agents), self._due()
        ran = False
        while chip.c < end:
            ran = True
            c = chip.c
            ui = self.ui_idle
            low = 0
            for d in devs:
                ui = (ui & ~d.ui_mask) | d.ui_val
                low |= d.io_low
            uo, ioo, oe = chip.pads(ui)
            dlow = oe & ~ioo & 0xFF
            dhigh = oe & ioo
            pulled = dlow | low
            if dhigh & low:
                self.conflicts.append((c, dhigh & low))
            io = dhigh & ~pulled
            for b in pu_bits:
                if (pulled >> b) & 1:
                    count[b] = 0
                else:
                    count[b] += 1
                    if count[b] > rise or (dhigh >> b) & 1:
                        io |= 1 << b
            val = uo | (ioo << 8) | (oe << 16) | (ui << 24) | (io << 32)
            if val != last:                     # the trace is run-length stored
                tt.append(c)
                tv.append(val)
                last = val
            chip.cycle(ui, io)
            for d in devs:
                d.step(c, uo, io)
            if agents and (c >= due or len(agents) != n_agents):
                self._agents(c)
                n_agents, due = len(agents), self._due()
            if stop is not None and stop():
                break
        if ran:
            trace.end = chip.c

    def _due(self):
        """The first cycle at which an agent wants to run again."""
        return min((a[1] for a in self.agents if a[0] is not None), default=1 << 62)

    def _agents(self, c):
        for a in self.agents:
            if a[0] is not None and c >= a[1]:
                try:
                    a[1] = c + next(a[0])
                except StopIteration:
                    a[0] = None


class Host:
    """Byte-level host. A command takes the SCK time of its bytes at clk/6 and acts
    at its decision point. With tdm=(period, offsets) a command starts only in this
    agent's windows, so several agents can share one link without colliding, and
    the schedule doesn't depend on what the other agents are doing.
    """
    SCK = 6

    def __init__(self, chip, tdm=None):
        self.chip, self.tdm = chip, tdm

    def window(self):
        if self.tdm:
            period, offs = self.tdm
            c = self.chip.c
            yield min((o - c) % period for o in offs) + 1

    def post(self, n, data):
        while True:
            yield from self.window()
            yield 24 * self.SCK
            if self.chip.host_post(n, data) == 0:
                return

    def take(self, n, into, tries=10 ** 9):
        """Appends (cycle, word) to into. A TAKE that finds nothing costs the same."""
        for _ in range(tries):
            yield from self.window()
            yield 8 * self.SCK
            full, data = self.chip.host_take(n)
            yield 24 * self.SCK
            if full:
                into.append((self.chip.c, data))
                return

    def delay(self, n):
        yield n


def run_mask(chip, stop=0, start=0):
    chip.host_run(stop, start)


# ---------------------------------------------------------------------------
# device models

class Device:
    ui_mask = 0
    ui_val = 0
    io_low = 0

    def step(self, c, uo, io):
        pass


class UartSender(Device):
    """Drives a ui pin. Bit time is a float, so baud error and phase are exact."""

    def __init__(self, ui_bit, frames, bit, start, stop_bits=1.0, gap=0.0):
        self.ui_mask = 1 << ui_bit
        self.ui_val = self.ui_mask
        self.sent = []
        self.sched = []
        t = float(start)
        for fr in frames:
            kind, val = fr if isinstance(fr, tuple) else ('byte', fr)
            if kind == 'byte':
                bits = [0] + [(val >> i) & 1 for i in range(8)] + [1]
            elif kind == 'badstop':
                bits = [0] + [(val >> i) & 1 for i in range(8)] + [0, 1]
            elif kind == 'glitch':      # short low pulse, val cycles wide
                self.sched.append((int(t), 0))
                self.sched.append((int(t) + val, 1))
                t += val + bit
                continue
            elif kind == 'gap':
                t += val
                continue
            for k, b in enumerate(bits):
                self.sched.append((int(round(t)), b))
                t += bit
            t += (stop_bits - 1.0) * bit + gap * bit
        self.sched.sort(key=lambda x: x[0])
        self.i = 0
        self.last = int(t)

    def step(self, c, uo, io):
        while self.i < len(self.sched) and self.sched[self.i][0] <= c + 1:
            self.ui_val = self.ui_mask * self.sched[self.i][1]
            self.i += 1


class SpiTarget(Device):
    """SPI target for one mode. MISO changes tco cycles after the shifting edge."""

    def __init__(self, sck, mosi, cs, miso, cpol, cpha, replies, tco=2):
        self.sck, self.mosi, self.cs = sck, mosi, cs
        self.ui_mask = 1 << miso
        self.miso_bit = miso
        self.cpol, self.cpha = cpol, cpha
        self.replies = list(replies)
        self.tco = tco
        self.p_sck = 0                  # reset state of the pads
        self.p_cs = 0
        self.in_frame = False
        self.frames = []
        self.count = 0                  # bits sampled in this frame
        self.acc = 0
        self.pending = []
        self.stream = 0                 # bits already consumed from replies

    def _bit(self, k):
        idx = k // 8
        if idx >= len(self.replies):
            return 0
        return (self.replies[idx] >> (7 - k % 8)) & 1

    def step(self, c, uo, io):
        sck = (uo >> self.sck) & 1
        cs = (uo >> self.cs) & 1
        mosi = (uo >> self.mosi) & 1
        if self.p_cs and not cs:
            self.in_frame = True
            self.frames.append([])
            self.count = 0
            self.acc = 0
            if self.cpha == 0:
                self.pending.append((c + self.tco, self._bit(self.stream)))
        elif cs and not self.p_cs:
            self.in_frame = False
            self.pending.append((c + self.tco, 0))
        elif self.in_frame and sck != self.p_sck:
            leading = self.p_sck == self.cpol
            if leading == (self.cpha == 0):
                self.acc = (self.acc << 1) | mosi
                self.count += 1
                self.stream += 1
                if self.count % 8 == 0:
                    self.frames[-1].append(self.acc & 255)
                    self.acc = 0
            else:
                self.pending.append((c + self.tco, self._bit(self.stream)))
        self.p_sck, self.p_cs = sck, cs
        while self.pending and self.pending[0][0] <= c + 1:
            self.ui_val = self.pending.pop(0)[1] << self.miso_bit


class I2cTarget(Device):
    """I2C target on uio pins, reacting to bus levels only.

    addr: 7-bit address. nack: set of byte numbers in a transaction to NACK, with
    0 the address byte. stretch(byte_no, bit_no): cycles to hold SCL low after the
    SCL fall that ends bit bit_no (8 is the ACK clock); 0 means no stretch.
    """

    def __init__(self, scl, sda, addr, nack=(), stretch=None, hold=15):
        self.scl, self.sda = scl, sda
        self.addr = addr
        self.nack = set(nack)
        self.stretch = stretch
        self.hold = hold                # SCL fall to our SDA change
        self.p_scl = self.p_sda = 1
        self.io_low = 0
        self.state = 'idle'
        self.nbit = 0
        self.acc = 0
        self.nbyte = 0
        self.ack_this = False
        self.sda_low = False
        self.scl_until = -1
        self.sda_at = None
        self.tx = []                    # one list per transaction
        self.starts = self.stops = 0
        self.stretched = 0

    def step(self, c, uo, io):
        scl = (io >> self.scl) & 1
        sda = (io >> self.sda) & 1
        if scl and self.p_scl:
            if self.p_sda and not sda:
                self.starts += 1
                self.state = 'byte'
                self.nbit = self.acc = self.nbyte = 0
                self.tx.append([])
            elif not self.p_sda and sda:
                self.stops += 1
                self.state = 'idle'
        elif self.state in ('byte', 'ack'):
            if scl and not self.p_scl and self.state == 'byte':
                self.acc = (self.acc << 1) | sda
                self.nbit += 1
            elif self.p_scl and not scl:
                if self.state == 'byte' and self.nbit == 8:
                    self.state = 'ack'
                    if self.nbyte == 0:
                        ok = (self.acc >> 1) == self.addr
                    else:
                        ok = True
                    if self.nbyte in self.nack:
                        ok = False
                    self.ack_this = ok
                    if ok:
                        self.sda_at = (c + self.hold, True)
                    self.tx[-1].append(self.acc)
                    self._stretch(c, 7)
                elif self.state == 'ack':
                    self.sda_at = (c + self.hold, False)
                    self.nbyte += 1
                    self.nbit = self.acc = 0
                    self.state = 'byte' if self.ack_this else 'ignore'
                    self._stretch(c, 8)
                elif self.state == 'byte':
                    self._stretch(c, self.nbit - 1)
        self.p_scl, self.p_sda = scl, sda
        if self.sda_at and c + 1 >= self.sda_at[0]:
            self.sda_low = self.sda_at[1]
            self.sda_at = None
        low = 0
        if self.sda_low:
            low |= 1 << self.sda
        if c + 1 < self.scl_until:
            low |= 1 << self.scl
        self.io_low = low

    def _stretch(self, c, bit_no):
        if self.stretch:
            n = self.stretch(self.nbyte, bit_no)
            if n:
                self.scl_until = c + 1 + n
                self.stretched += 1


class OneWireTarget(Device):
    """1-Wire slave at standard speed. Reset is a low of 480 us or more; a shorter
    low is a time slot, sampled sample_us after its falling edge."""

    def __init__(self, pin, sample_us=30, pd_wait_us=30, pd_low_us=120, present=True):
        self.pin = pin
        self.sample = us(sample_us)
        self.pd_wait, self.pd_low = us(pd_wait_us), us(pd_low_us)
        self.present = present
        self.p = 1
        self.fall = None
        self.sample_at = None
        self.pd_on = self.pd_off = None
        self.io_low = 0
        self.resets = 0
        self.bits = []
        self.bytes = []

    def step(self, c, uo, io):
        line = (io >> self.pin) & 1
        own = self.pd_on is not None and self.pd_on <= c + 1 < self.pd_off
        if self.p and not line and not (self.pd_on is not None and c + 1 >= self.pd_on
                                        and c <= self.pd_off):
            self.fall = c
            self.sample_at = c + self.sample
        if not self.p and line and self.fall is not None:
            if c - self.fall >= us(480):
                self.resets += 1
                self.bits = []
                if self.present:
                    self.pd_on = c + self.pd_wait
                    self.pd_off = self.pd_on + self.pd_low
            self.fall = None
        if self.sample_at is not None and c >= self.sample_at:
            self.sample_at = None
            if self.fall is not None and c - self.fall >= us(480):
                pass
            elif self.resets:
                self.bits.append(line)
                if len(self.bits) == 8:
                    self.bytes.append(sum(b << i for i, b in enumerate(self.bits)))
                    self.bits = []
        self.p = line
        self.io_low = (1 << self.pin) if own else 0


# ---------------------------------------------------------------------------
# decoders: pin trace in, protocol events and rule violations out

def uart_decode(w, baud, lo=0, hi=None, tol=0.02):
    """8N1, LSB first, sampled at bit centres. Every edge in a frame must lie on the
    nominal bit grid of that frame, within tol of a bit. Returns (bytes, errors)."""
    bit = CLK / baud
    hi = w.end if hi is None else hi
    t_start = lo
    while w.at(t_start) == 0 and t_start < hi:    # a line that starts low hasn't reached idle
        later = [t for t, v in w.edges(t_start, hi) if v == 1]
        if not later:
            return [], ['line never went high']
        t_start = later[0]
    out, errs, edges = [], [], [t for t, v in w.edges(t_start, hi)]
    falls = [t for t, v in w.edges(t_start, hi) if v == 0]
    i = 0
    next_ok = t_start
    for f in falls:
        if f < next_ok:
            continue
        if f + 10 * bit > hi:
            errs.append('frame cut off at the end of the run: %d' % f)
            break
        s = [w.at(int(f + (k + 0.5) * bit)) for k in range(10)]
        if s[0] != 0:
            errs.append('no start bit at %d' % f)
            next_ok = f + 1
            continue
        if s[9] != 1:
            errs.append('stop bit low at %d' % f)
        val = sum(b << k for k, b in enumerate(s[1:9]))
        out.append(val)
        for e in edges:
            if f < e < f + 9.5 * bit:
                k = (e - f) / bit
                if abs(k - round(k)) > tol:
                    errs.append('edge %d is %.3f bits into frame at %d' % (e, k, f))
        next_ok = f + 9.5 * bit
    return out, errs


def spi_decode(sck, mosi, miso, cs, cpol, cpha, tsu=1, thd=1):
    """Frames are CS low. Returns (list of (mosi bytes, miso bytes), errors)."""
    errs, frames = [], []
    cs_e = cs.edges()
    falls = [t for t, v in cs_e if v == 0]
    rises = [t for t, v in cs_e if v == 1]
    begin = 0
    if cs.at(0) == 0 and rises:         # reset leaves CS low until firmware raises it
        begin = rises[0]
        rises = rises[1:]
    for k, f in enumerate(falls):
        if k >= len(rises):
            errs.append('CS never rose after %d' % f)
            break
        r = rises[k]
        if sck.at(f) != cpol or sck.at(f + 1) != cpol:
            errs.append('SCK not idle at CS fall %d' % f)
        if sck.at(r) != cpol:
            errs.append('SCK not idle at CS rise %d' % r)
        ed = [t for t, v in sck.edges(f, r + 1)]
        if len(ed) % 2:
            errs.append('odd SCK edge count in frame at %d' % f)
        if not ed:
            frames.append(([], []))
            continue
        if ed[0] - f < 1:
            errs.append('no CS setup before first SCK edge at %d' % f)
        lead = ed[0::2]
        trail = ed[1::2]
        sample = lead if cpha == 0 else trail
        m_edges = [t for t, v in mosi.edges(f, r + 1)]
        bits_o, bits_i = [], []
        for s in sample:
            bits_o.append(mosi.at(s - 1))
            bits_i.append(miso.at(s - 1))
            before = [t for t in m_edges if t <= s]
            after = [t for t in m_edges if t > s]
            if before and s - before[-1] < tsu:
                errs.append('MOSI setup %d at SCK sample %d' % (s - before[-1], s))
            if after and after[0] - s < thd:
                errs.append('MOSI hold %d at SCK sample %d' % (after[0] - s, s))
        if len(bits_o) % 8:
            errs.append('frame at %d has %d bits' % (f, len(bits_o)))
        tob = lambda b: [sum(x << (7 - i) for i, x in enumerate(b[j:j + 8]))
                         for j in range(0, len(b) - len(b) % 8, 8)]
        frames.append((tob(bits_o), tob(bits_i)))
    all_edges = [t for t, v in sck.edges()]
    spans = list(zip(falls, rises))
    for t in all_edges:
        if t > begin and not any(a <= t <= b for a, b in spans):
            errs.append('SCK edge outside CS at %d' % t)
    return frames, errs


# UM10204, the bus timing table for Standard-mode and Fast-mode, in nanoseconds.
I2C_SPEC = {
    'Sm': dict(f_max=100e3, t_low=4700, t_high=4000, t_su_sta=4700, t_hd_sta=4000, t_su_sto=4000,
               t_buf=4700, t_su_dat=250, t_hd_dat=0, t_vd_dat=3450),
    'Fm': dict(f_max=400e3, t_low=1300, t_high=600, t_su_sta=600, t_hd_sta=600, t_su_sto=600,
               t_buf=1300, t_su_dat=100, t_hd_dat=0, t_vd_dat=900),
}


def i2c_decode(scl, sda, mode, lo=0, hi=None):
    """Walks bus levels. Returns (transactions, errors, measured). A transaction is
    dict(bytes=[...], acks=[...], stop=bool). Rises are VIH crossings at cycle level."""
    sp = {k: (ns(v) if k != 'f_max' else v) for k, v in I2C_SPEC[mode].items()}
    hi = scl.end if hi is None else hi
    ev = [(t, 'c', v) for t, v in scl.edges(lo, hi)] + [(t, 'd', v) for t, v in sda.edges(lo, hi)]
    ev.sort(key=lambda e: e[0])
    txs, errs = [], []
    m = {}

    def seen(name, val):
        a = m.setdefault(name, [val, val])
        a[0], a[1] = min(a[0], val), max(a[1], val)

    tx = None
    bits = []
    cl, da = scl.at(lo), sda.at(lo)
    t_fall = t_rise = t_rise_prev = t_d = t_start = t_stop = vd = None
    first_fall = False
    for i, (t, k, v) in enumerate(ev):
        same = i + 1 < len(ev) and ev[i + 1][0] == t
        if same:
            errs.append('SCL and SDA changed in one cycle at %d' % t)
        if k == 'd':
            if cl:                      # SDA moves while SCL is high: START or STOP
                if v == 0:
                    if tx is not None and len(bits) > 1:
                        errs.append('START in the middle of a byte at %d' % t)
                    if tx is None:
                        if t_stop is not None:
                            seen('t_buf', t - t_stop)
                            if t - t_stop < sp['t_buf']:
                                errs.append('tBUF %d at %d' % (t - t_stop, t))
                    else:
                        if t_rise is not None:
                            seen('t_su_sta', t - t_rise)
                            if t - t_rise < sp['t_su_sta']:
                                errs.append('tSU;STA %d at %d' % (t - t_rise, t))
                        txs.append(tx)
                    tx = dict(bytes=[], acks=[], stop=False, start=t)
                    bits = []
                    t_start = t
                    first_fall = True
                else:
                    if tx is None:
                        errs.append('STOP outside a transaction at %d' % t)
                    else:
                        if len(bits) > 1:   # the SCL rise of a STOP counts as one
                            errs.append('STOP in the middle of a byte at %d' % t)
                        seen('t_su_sto', t - t_rise)
                        if t - t_rise < sp['t_su_sto']:
                            errs.append('tSU;STO %d at %d' % (t - t_rise, t))
                        tx['stop'] = True
                        txs.append(tx)
                        tx = None
                    t_stop = t
            else:                       # data change while SCL is low
                if t_fall is not None:
                    seen('t_hd_dat', t - t_fall)
                    if t - t_fall < sp['t_hd_dat']:
                        errs.append('tHD;DAT %d at %d' % (t - t_fall, t))
                    vd = t - t_fall     # measured at the SCL rise, see below
            t_d = t
            da = v
        else:
            if v == 1:                  # SCL rises
                if t_fall is not None:
                    seen('t_low', t - t_fall)
                    if t - t_fall < sp['t_low']:
                        errs.append('tLOW %d at %d' % (t - t_fall, t))
                    # UM10204 applies the tVD;DAT maximum only when SCL's low period
                    # isn't stretched; a stretched low needs only tSU;DAT.
                    if vd is not None and t - t_fall <= 2 * sp['t_low']:
                        seen('t_vd_dat', vd)
                        if vd > sp['t_vd_dat']:
                            errs.append('tVD;DAT %d at %d' % (vd, t))
                    vd = None
                if t_d is not None and t_fall is not None and t_d > t_fall:
                    seen('t_su_dat', t - t_d)
                    if t - t_d < sp['t_su_dat']:
                        errs.append('tSU;DAT %d at %d' % (t - t_d, t))
                if t_rise is not None:
                    if t - t_rise < CLK / sp['f_max']:
                        errs.append('fSCL too high: period %d at %d' % (t - t_rise, t))
                    seen('period', t - t_rise)
                t_rise_prev, t_rise = t_rise, t
                if tx is not None:
                    bits.append(da)
                    if len(bits) == 9:
                        tx['bytes'].append(sum(b << (7 - j) for j, b in enumerate(bits[:8])))
                        tx['acks'].append(1 - bits[8])
                        bits = []
            else:                       # SCL falls
                if t_rise is not None:
                    seen('t_high', t - t_rise)
                    if t - t_rise < sp['t_high']:
                        errs.append('tHIGH %d at %d' % (t - t_rise, t))
                if first_fall:
                    seen('t_hd_sta', t - t_start)
                    if t - t_start < sp['t_hd_sta']:
                        errs.append('tHD;STA %d at %d' % (t - t_start, t))
                    first_fall = False
                t_fall = t
            cl = v
    if tx is not None:
        txs.append(tx)
        errs.append('transaction not closed by a STOP')
    return txs, errs, m


WS2812 = dict(t0h=(200, 500), t1h=(550, 850), t0l=(650, 950), t1l=(450, 750), reset=280_000)


def ws2812_decode(w, lo=0, hi=None):
    """Bits from pulse widths, using the WS2812B +-150 ns windows. A low of at least
    the reset time ends a frame. Returns (frames of bit lists, errors, measured)."""
    hi = w.end if hi is None else hi
    ed = w.edges(lo, hi)
    frames, cur, errs = [], [], []
    m = dict(t0h=set(), t1h=set(), t0l=set(), t1l=set(), period=set())
    rises = [t for t, v in ed if v == 1]
    for i, r in enumerate(rises):
        f = next((t for t, v in ed if v == 0 and t > r), None)
        if f is None:
            errs.append('line left high at %d' % r)
            break
        high = f - r
        nextr = rises[i + 1] if i + 1 < len(rises) else None
        low = (nextr - f) if nextr is not None else (hi - f)
        ns_h = high * 1e9 / CLK
        ns_l = low * 1e9 / CLK
        if WS2812['t0h'][0] <= ns_h <= WS2812['t0h'][1]:
            bit = 0
        elif WS2812['t1h'][0] <= ns_h <= WS2812['t1h'][1]:
            bit = 1
        else:
            errs.append('high time %d cycles at %d fits neither symbol' % (high, r))
            bit = None
        m['t%dh' % (bit if bit is not None else 0)].add(high)
        if bit is not None:
            cur.append(bit)
        if ns_l >= WS2812['reset']:
            frames.append(cur)
            cur = []
        else:
            key = 't0l' if bit == 0 else 't1l'
            if bit is not None:
                m[key].add(low)
                if not WS2812[key][0] <= ns_l <= WS2812[key][1]:
                    errs.append('low time %d cycles after bit %d at %d' % (low, bit, r))
                m['period'].add(high + low)
    if cur:
        errs.append('frame not closed by a reset gap (%d bits)' % len(cur))
        frames.append(cur)
    return frames, errs, m


def onewire_decode(line, master, lo=0, hi=None):
    """line: bus level. master: 1 while the chip pulls the bus low.
    Returns (events, errors, measured). Rules are the DS18B20 standard-speed table."""
    hi = hi if hi is not None else line.end
    ev, errs = [], []
    m = {}
    ed = master.edges(lo, hi)
    pulses = []
    pend = None
    for t, v in ed:
        if v == 1:
            pend = t
        elif pend is not None:
            pulses.append((pend, t))
            pend = None
    if pend is not None:
        errs.append('master still holds the bus low at the end of the run')
    bits = []
    prev_end = None
    after_reset = None
    for i, (a, b) in enumerate(pulses):
        width = b - a
        gap = a - prev_end if prev_end is not None else None
        if width >= us(480):
            m.setdefault('t_rstl', []).append(width)
            if width > us(960):
                errs.append('reset low %d cycles at %d, above 960 us' % (width, a))
            if bits:
                errs.append('reset in the middle of a byte: %d bits' % len(bits))
                bits = []
            nxt = pulses[i + 1][0] if i + 1 < len(pulses) else hi
            m.setdefault('t_rsth', []).append(nxt - b)
            if nxt - b < us(480):
                errs.append('reset high %d cycles at %d, below 480 us' % (nxt - b, b))
            low = [t for t, v in line.edges(b, nxt) if v == 0]
            ev.append(('reset', a, bool(low)))
            after_reset = True
        elif us(1) <= width <= us(15):
            m.setdefault('t_low1', []).append(width)
            bits.append(1)
        elif us(60) <= width <= us(120):
            m.setdefault('t_low0', []).append(width)
            bits.append(0)
        else:
            errs.append('low pulse of %d cycles at %d is no valid slot' % (width, a))
            prev_end = b
            continue
        if width < us(480):
            if after_reset is None:
                errs.append('slot before any reset at %d' % a)
            if gap is not None and gap < us(1):
                errs.append('recovery %d cycles at %d, below 1 us' % (gap, a))
            if gap is not None:
                m.setdefault('t_rec', []).append(gap)
            nxt = pulses[i + 1][0] if i + 1 < len(pulses) else None
            if nxt is not None:
                m.setdefault('t_slot', []).append(nxt - a)
                if nxt - a < us(60):
                    errs.append('slot of %d cycles at %d, below 60 us' % (nxt - a, a))
            if len(bits) == 8:
                ev.append(('byte', a, sum(x << k for k, x in enumerate(bits))))
                bits = []
        prev_end = b
    if bits:
        errs.append('%d bits left over' % len(bits))
    return ev, errs, m
