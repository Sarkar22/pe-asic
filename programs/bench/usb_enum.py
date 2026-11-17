"""Bench for advanced/usb_enum.s: a USB low-speed device on endpoint 0 at 50 MHz, enumerated end to end by a host
model on the other side of the cable.

The host is a model of a host controller and its USB core, at the device's pins, through frontend.UsbLs (the
device's 1.5 kohm pull-up on D-, the host's 15 kohm pull-downs, 450 pF, contention log). Its packets are built
field by field by usb_check_test's builders, which follow USB 2.0 8.3 and 8.4 and share no code with the firmware
or the checker: PID and check nibble, ADDR and ENDP, CRC5 and CRC16 as 8.3.5 words them, stuffing from the SYNC
(7.1.9), NRZI from idle J (7.1.8), EOP. It sends at a bit time it is given (USB 2.0 Table 7-10: TLDRATHS is
1.5 Mb/s +/- 0.05% for a host), with edge jitter, EOPs of a given SE0 width (TLEOPT 1.25 to 1.50 us; receivers
accept 670 ns, TLEOPR), and inter-packet delays of a given length (7.1.18.1: at least 2 bit times). It reacts to
the device as a host does: it waits 17 bit times for an answer (7.1.19.1: a host times out after 16 to 18), ACKs a
good data packet, discards one whose toggle it didn't expect but ACKs it (8.6.4), retries after a NAK, a time-out
or a corrupted packet, and sends a keep-alive, a low-speed EOP, at the start of every 1 ms frame (7.1.7.6, Table
7-2 note 4). Faults can be planted: lost ACKs, corrupted packets, tokens for other addresses or endpoints, a reset
or a SETUP in the middle of a transfer, dribble bits before an EOP.

usb_check.check, written from USB 2.0 before this firmware, judges the whole record of D+ and D-:
the device's bit rate, jitter, SYNC, stuffing, PIDs, CRCs, packet formats, EOP and the J after it, inter-packet
delay and response time, keep-alive, reset, the control transfers with their toggles, retries and the address
change, the descriptor, and the line state. The bench adds what the host itself received, the device's counters,
and the threads' health (LATE, ILL, a stopped thread, an overwritten code word).

The chip samples the bus at each clock edge plus a phase of 0 to 20 ns (section 10, item 2), and its pads drive
the bus from the clock edge. Threads and start alignments vary across runs.
"""
import bisect
import random

import frontend
import usb_check
import usb_check_test as G
from report import Chip, assemble, src

PROGRAMS = ('advanced/usb_enum.s',)
PROG = 'advanced/usb_enum.s'
WORDS = 493                     # section 11.2's count for the program
BIT = 1e9 / 1.5e6               # ns, USB 2.0 7.1.11
CYCLE = 20.0                    # ns at 50 MHz
DP, DM = 0, 1                   # io0, io1
MEM = dict(STAGE=0, ADDRV=13, NSETUP=14, NREQ=15, RSTN=11)
CODE_FROM = 23                  # the descriptor and the code: never written by the program
DESC = bytes([18, 1, 0x10, 0x01, 0, 0, 0, 8, 0x09, 0x12, 0x01, 0x00, 0x00, 0x01, 0, 0, 0, 1])
DESC_AT = 23                    # where the image keeps it, two bytes a word
J, K, SE0, SE1 = 'J', 'K', 'SE0', 'SE1'
LEVELS = {J: (0, 1), K: (1, 0), SE0: (0, 0), SE1: (1, 1)}       # (D+, D-) at low speed (Table 7-2)
PIDS = {0b0011: 'DATA0', 0b1011: 'DATA1', 0b0010: 'ACK', 0b1010: 'NAK', 0b1110: 'STALL'}
GET_DEV = lambda n: [0x80, 6, 0, 1, 0, 0, n & 255, n >> 8]                  # GET_DESCRIPTOR(DEVICE), 9.4.3
SET_ADDR = lambda a: [0, 5, a, 0, 0, 0, 0, 0]                                 # SET_ADDRESS, 9.4.6


def desc_words(d):
    return [d[i] | (d[i + 1] << 8) for i in range(0, len(d), 2)]


# ---------------------------------------------------------------------------------------------------------------
# the line, as the host sees it

class Watch:
    """Line states from frontend.UsbLs's record, kept incrementally: (t, state). An SE0 or SE1 no longer than TLST
    (210 ns, Table 7-10) between J and K is a transition, as 7.1.4.1 asks of a receiver."""

    def __init__(self, bus):
        self.bus = bus
        self.ip = self.im = 0
        self.lv = [0, 1]
        self.raw = [(-1e18, J)]

    def update(self):
        dp, dm = self.bus.dp_rec, self.bus.dm_rec
        new = [(t, 0, v) for t, v in dp[self.ip:]] + [(t, 1, v) for t, v in dm[self.im:]]
        self.ip, self.im = len(dp), len(dm)
        for t, i, v in sorted(new):
            self.lv[i] = v
            s = {(0, 1): J, (1, 0): K, (0, 0): SE0, (1, 1): SE1}[tuple(self.lv)]
            if self.raw[-1][0] == t:
                self.raw[-1] = (t, s)
            elif self.raw[-1][1] != s:
                self.raw.append((t, s))

    def since(self, t0):
        """States from t0 on, transients folded in; the first is the state at t0."""
        k = bisect.bisect_right(self.raw, (t0, 'ZZZ')) - 1
        seq = self.raw[max(k, 0):]
        out = []
        q = 0
        while q < len(seq):
            t, s = seq[q]
            if s in (SE0, SE1) and out and q + 1 < len(seq) and seq[q + 1][0] - t <= 210.0 and \
                    out[-1][1] in (J, K) and seq[q + 1][1] in (J, K) and seq[q + 1][1] != out[-1][1]:
                out.append(((t + seq[q + 1][0]) / 2, seq[q + 1][1]))
                q += 2
                continue
            if not out or out[-1][1] != s:
                out.append((max(t, t0), s))
            q += 1
        return out


class Rx:
    """A packet the host received: from the J-to-K of its SOP to the SE0-to-J of its EOP."""

    def __init__(self, sop, se0, end, raw):
        self.sop, self.se0, self.end = sop, se0, end
        self.raw = raw
        self.ok = False
        self.name = None
        self.payload = b''
        bits, run, k = [], 1, 8
        if raw[:8] != [0] * 7 + [1]:
            return
        while k < len(raw):                 # 7.1.9: drop the 0 after six 1s
            b = raw[k]
            if run == 6:
                if b:
                    return
                run = 0
                k += 1
                continue
            bits.append(b)
            run = run + 1 if b else 0
            k += 1
        if len(bits) % 8 or not bits:
            return
        octs = bytes(sum(bits[8 * q + b] << b for b in range(8)) for q in range(len(bits) // 8))
        pid = octs[0]
        if (pid >> 4) != (~pid & 15) or (pid & 15) not in PIDS:
            return
        self.name = PIDS[pid & 15]
        if self.name.startswith('DATA'):
            if len(octs) < 3:
                return
            reg = 0xFFFF                    # 8.3.5.2: the residual over data and CRC16
            for b in bits[8:]:
                f = ((reg >> 15) & 1) ^ b
                reg = (reg << 1) & 0xFFFF
                if f:
                    reg ^= 0x8005
            if reg != 0x800D:
                return
            self.payload = octs[1:-2]
        elif len(octs) != 1:
            return
        self.ok = True

    def __repr__(self):
        return '%s %s @%.1f us' % (self.name, self.payload.hex(), self.sop / 1e3)


def decode_device(states, period=BIT):
    """The first whole packet in states (from idle J), or None: its NRZI bits from its transitions."""
    k = 0
    while k < len(states) and not (states[k][1] == K and (k == 0 or states[k - 1][1] == J)):
        k += 1
    if k >= len(states):
        return None
    sop = states[k][0]
    trans = [sop]
    m = k + 1
    while m < len(states) and states[m][1] in (J, K):
        trans.append(states[m][0])
        m += 1
    if m >= len(states) or states[m][1] != SE0 or m + 1 >= len(states):
        return None
    se0, end = states[m][0], states[m + 1][0]
    raw = []
    for q, t in enumerate(trans):
        nxt = trans[q + 1] if q + 1 < len(trans) else se0
        n = max(1, int(round((nxt - t) / period)))
        raw += [0] + [1] * (n - 1)
    return Rx(sop, se0, end, raw)


# ---------------------------------------------------------------------------------------------------------------
# the host

class Host:
    """A low-speed host at the device's pins. Times in ns. A script is a generator of Host methods (yield from);
    each yield is the time to resume at. period: its bit time; jitter: each transition moved by a uniform amount up
    to that many ns; eop: its SE0 in bit times; gap_data, gap_hs, gap_tr: inter-packet delays in bit times before a
    token's data packet, before its handshake to the device's data, and between transactions."""

    def __init__(self, bus, period=BIT, jitter=0.0, eop=2.0, gap_data=3.0, gap_hs=3.0, gap_tr=4.0, seed=1,
                 frame=1e6, timeout=17.0, dribble=0, skew=0.0):
        self.bus, self.period, self.jitter, self.eop = bus, period, jitter, eop
        self.gap_data, self.gap_hs, self.gap_tr = gap_data, gap_hs, gap_tr
        self.rng = random.Random(seed)
        self.frame, self.timeout, self.dribble, self.skew = frame, timeout, dribble, skew
        self.watch = Watch(bus)
        self.t = 0.0
        self.last = 0.0                 # the SE0-to-J of the last packet on the bus
        self.next_frame = None
        self.log = []                   # what happened, for the checks: (kind, ...)
        self.got = []                   # the device's packets as the host decoded them
        self.answers = []               # (host packet's SE0-to-J, device's SOP) for each answer

    # -- the wire
    def tx(self, bits, t0, eop=None, jitter=None, dribble=None):
        """Schedules one packet from J at t0: SYNC and stuffing (usb_check_test.stuff), NRZI, then the EOP: SE0 for
        eop bit times, J driven for one bit, released. dribble: bit times of the last level added before the SE0
        (7.1.9.1). skew: D- moves that many ns after D+ at each J-K transition, so the line passes through SE1 or
        SE0 (7.1.4.1: up to TLST). Returns the SE0-to-J time."""
        eop = self.eop if eop is None else eop
        jit = self.jitter if jitter is None else jitter
        drib = self.dribble if dribble is None else dribble
        raw = G.stuff(bits)
        st = J
        b = self.bus
        p = self.period
        for i, x in enumerate(raw):
            if x == 0:
                old, st = st, K if st == J else J
                d = self.rng.uniform(-jit, jit) if jit and i else 0.0
                if self.skew and i:
                    b.host(t0 + i * p + d, LEVELS[st][0], LEVELS[old][1], 1)
                    b.host(t0 + i * p + d + self.skew, *LEVELS[st], 1)
                else:
                    b.host(t0 + i * p + d, *LEVELS[st], 1)
            elif i == 0:
                b.host(t0, *LEVELS[J], 1)
        t_se0 = t0 + (len(raw) + drib) * p
        b.host(t_se0, *LEVELS[SE0], 1)
        t_j = t_se0 + eop * p
        b.host(t_j, *LEVELS[J], 1)
        b.host(t_j + p, *LEVELS[J], 0)
        return t_j

    def at(self, gap):
        return max(self.last + gap * self.period, self.t + 2 * CYCLE)

    def send(self, bits, gap, **kw):
        """One packet, gap bit times after the last one on the bus. Waits until it is out and released."""
        t0 = self.at(gap)
        self.last = self.tx(bits, t0, **kw)
        yield self.last + self.period + CYCLE
        return self.last

    def answer(self, after):
        """The device's packet that starts within the time-out after `after` (the SE0-to-J of the host's packet),
        decoded, or None."""
        t_lim = after + self.timeout * self.period
        while True:
            yield self.t + 10 * CYCLE
            self.watch.update()
            st = self.watch.since(after + 1.0)
            pk = decode_device(st)
            if pk is not None:
                self.last = pk.end
                self.got.append(pk)
                self.answers.append((after, pk.sop))
                return pk
            started = any(s != J for _, s in st)
            if not started and self.t > t_lim:
                return None
            if self.t > t_lim + 200 * self.period:
                return None

    def wait(self, ns):
        """Idles ns, with a keep-alive at every frame start."""
        t_end = self.t + ns
        while True:
            if self.next_frame is not None and self.next_frame < t_end:
                yield max(self.next_frame, self.t + CYCLE)
                yield from self.keepalive()
                continue
            yield t_end
            return

    def keepalive(self):
        """A low-speed EOP: SE0 for two bit times, then J (Table 7-2 note 4, 7.1.7.6)."""
        t0 = self.at(2.0)
        b = self.bus
        b.host(t0, *LEVELS[SE0], 1)
        b.host(t0 + 2 * self.period, *LEVELS[J], 1)
        b.host(t0 + 3 * self.period, *LEVELS[J], 0)
        self.last = t0 + 2 * self.period
        if self.next_frame is not None:
            while self.next_frame <= self.t + CYCLE:
                self.next_frame += self.frame
        self.log.append(('keepalive', t0))
        yield self.last + self.period + CYCLE

    def frame_check(self):
        """Before a transaction: the keep-alive of a frame that has started (a host starts each frame with one)."""
        if self.next_frame is not None and self.t >= self.next_frame:
            yield from self.keepalive()

    def reset(self, ns=10.2e6):
        """SE0 for ns (7.1.7.5: TDRST at least 10 ms), then J, then keep-alives from the next frame on."""
        t0 = self.at(4.0)
        b = self.bus
        b.host(t0, *LEVELS[SE0], 1)
        b.host(t0 + ns, *LEVELS[J], 1)
        b.host(t0 + ns + self.period, *LEVELS[J], 0)
        self.last = t0 + ns
        self.log.append(('reset', t0, ns))
        yield self.last + self.period + CYCLE
        self.next_frame = self.t + 0.1e6

    # -- transactions (8.4.6, 8.5.3)
    def setup(self, addr, req, endp=0, bad=False, tries=3, dribble=None, bad_token=False):
        """SETUP and DATA0 with the 8 bytes of req. True once the device ACKs. dribble: bit times added before the
        data packet's EOP. bad_token: a CRC5 bit of the SETUP token flipped."""
        for _ in range(tries):
            yield from self.frame_check()
            tokb = G.token('SETUP', addr, endp)
            if bad_token:
                tokb = list(tokb)
                tokb[-1] ^= 1               # a CRC5 bit
            yield from self.send(tokb, self.gap_tr)
            t = yield from self.send(G.data('DATA0', req, bad_crc=bad), self.gap_data, dribble=dribble)
            pk = yield from self.answer(t)
            self.log.append(('setup', addr, bytes(req), pk.name if pk else None))
            if pk is not None and pk.ok and pk.name == 'ACK':
                return True
            if bad or bad_token:
                return False
        return False

    def in_(self, addr, endp=0, ack=True, bad_token=False, dribble=None):
        """One IN transaction: ('data', Rx), ('NAK', Rx), ('STALL', Rx), ('bad', Rx) or ('none', None). The host
        ACKs a good data packet unless ack is False (its ACK lost)."""
        yield from self.frame_check()
        tokb = G.token('IN', addr, endp)
        if bad_token:
            tokb = list(tokb)
            tokb[-1] ^= 1                   # a CRC5 bit
        t = yield from self.send(tokb, self.gap_tr, dribble=dribble)
        pk = yield from self.answer(t)
        if pk is None:
            self.log.append(('in', addr, 'none'))
            return 'none', None
        if not pk.ok:
            self.log.append(('in', addr, 'bad'))
            return 'bad', pk
        if pk.name.startswith('DATA'):
            if ack:
                yield from self.send(G.handshake('ACK'), self.gap_hs)
            self.log.append(('in', addr, pk.name, pk.payload, ack))
            return 'data', pk
        self.log.append(('in', addr, pk.name))
        return pk.name, pk

    def out(self, addr, payload, toggle='DATA1', endp=0, bad=False, bad_token=False):
        """OUT and its data packet: the device's handshake name, or None. bad_token: a CRC5 bit of the token
        flipped."""
        yield from self.frame_check()
        tokb = G.token('OUT', addr, endp)
        if bad_token:
            tokb = list(tokb)
            tokb[-1] ^= 1                   # a CRC5 bit
        yield from self.send(tokb, self.gap_tr)
        t = yield from self.send(G.data(toggle, payload, bad_crc=bad), self.gap_data)
        pk = yield from self.answer(t)
        name = pk.name if pk is not None and pk.ok else None
        self.log.append(('out', addr, name))
        return name

    def control_read(self, addr, req, lose=(), tries=8):
        """A control read (8.5.3): SETUP, IN data stage from DATA1, then the status OUT with a zero-length DATA1.
        lose: data packets (0, 1, ...) whose first ACK the host loses. Returns the bytes, or None on a STALL."""
        if not (yield from self.setup(addr, req)):
            return None
        want = req[6] | (req[7] << 8)
        data, toggle, n, lost = b'', 'DATA1', 0, set()
        miss = 0
        while len(data) < want:
            kind, pk = yield from self.in_(addr, ack=not (n in lose and n not in lost))
            if kind == 'STALL':
                return None
            if kind != 'data':
                miss += 1
                if miss > tries:
                    return None
                continue
            if n in lose and n not in lost:
                lost.add(n)
                continue                        # the ACK was lost: the device must send it again
            if pk.name != toggle:
                continue                        # a retry the host already has (8.6.4): ACKed and dropped
            data += pk.payload
            toggle = 'DATA0' if toggle == 'DATA1' else 'DATA1'
            n += 1
            if len(pk.payload) < 8:
                break
        for _ in range(tries):
            hs = yield from self.out(addr, [], 'DATA1')
            if hs == 'ACK':
                return data
            if hs == 'STALL':
                return None
        return None

    def control_nodata(self, addr, req, lose_status=0, tries=8):
        """A request with no data stage: SETUP, then the status IN, a zero-length DATA1 the host ACKs.
        lose_status: how many of the device's status packets the host doesn't ACK first. True when done."""
        if not (yield from self.setup(addr, req)):
            return False
        lost = 0
        for _ in range(tries):
            kind, pk = yield from self.in_(addr, ack=lost >= lose_status)
            if kind == 'STALL':
                return False
            if kind == 'data':
                if lost < lose_status:
                    lost += 1
                    continue
                return pk.name == 'DATA1' and pk.payload == b''
        return False


# ---------------------------------------------------------------------------------------------------------------
# a run

class Result:
    pass


def run(script, t_end, phase=0.0, align=0, threads=(0, 1, 2), seed=1, text=None, chip_cls=None, host_kw=None,
        bus_kw=None, desc=None, check=True):
    """Runs the program against the host script until the script ends plus 0.2 ms, or t_end ns."""
    img = assemble(text if text is not None else src(PROG), depth=512)
    chip = (chip_cls or Chip)(512, seed)
    for a, w in img.words.items():
        chip.host_wrmem(a, w)
    if desc is not None:
        for i, w in enumerate(desc_words(desc)):
            chip.host_wrmem(DESC_AT + i, w)
    ts, ta, tw = threads
    chip.host_setpc(ts, img.labels['dev'])
    chip.host_setpc(ta, img.labels['tok'])
    chip.host_setpc(tw, img.labels['rst'])
    bus = frontend.UsbLs(**(bus_kw or {}))
    host = Host(bus, seed=seed, **(host_kw or {}))
    out = {}

    def wrap():
        yield 30e3                              # the threads configure their units first
        v = yield from script(host)
        out['value'] = v
    gen = wrap()
    wake = 0.0
    done_at = None
    cycles = int(t_end / CYCLE)
    started = False
    c = 0
    for c in range(cycles):
        if not started and (c - 1) % 4 == align:
            chip.host_run(0, (1 << ts) | (1 << ta) | (1 << tw))
            started = True
        t = c * CYCLE
        uo, io, oe = chip.pads(0x01)
        bus.device(t, io & 1, (io >> 1) & 1, oe & 1, (oe >> 1) & 1)
        if gen is not None and t >= wake:
            host.t = t
            try:
                wake = gen.send(None)
            except StopIteration:
                gen = None
                done_at = t
        if done_at is not None and t > done_at + 0.2e6:
            break
        dp, dm = bus.level(t + phase)
        chip.cycle(0x01, dp | (dm << 1) | 0xFC)
    res = Result()
    res.t_end = (c + 1) * CYCLE
    res.rec = bus.record(res.t_end)
    res.host, res.chip, res.img, res.bus = host, chip, img, bus
    res.value = out.get('value')
    res.finished = gen is None
    res.desc = desc if desc is not None else DESC
    res.rep = usb_check.check(res.rec, descriptor=res.desc) if check else None
    res.health = health(chip, img, threads)
    res.count = {k: chip.mem[a] for k, a in MEM.items()}
    return res


def health(chip, img, threads):
    """LATE, ILL, a thread stopped, or a code or descriptor word overwritten."""
    out = []
    for tid in threads:
        t = chip.th[tid]
        if t.LATE:
            out.append('thread %d LATE' % tid)
        if t.ILL:
            out.append('thread %d ILL at %d' % (tid, t.PC))
        if not t.RUN:
            out.append('thread %d stopped' % tid)
    bad = [a for a, w in img.words.items() if a >= CODE_FROM and chip.mem[a] != w and not DESC_AT <= a < DESC_AT + 9]
    if bad:
        out.append('memory overwritten at %s' % bad[:5])
    return out


def verdict(res, allow=()):
    """Every failing finding, the health and the script's end, as one string ('' if all is well)."""
    bad = [f for f in res.rep.failed() if f[1] not in allow]
    out = ['%s @%.1f us: %s' % (c, t / 1e3, x) for t, c, x in bad[:3]]
    out += res.health
    if not res.finished:
        out.append('the host script did not finish')
    return '; '.join(out)


def gaps(res):
    """(SOP - SE0-to-J) / bit time of every device answer, from the checker's packets: the response times."""
    out = []
    pks = res.rep.packets
    for i, pk in enumerate(pks):
        if i and pk.sender == 'dev' and pks[i - 1].sender == 'host' and pks[i - 1].end != float('inf'):
            out.append((pk.sop - pks[i - 1].end) / BIT)
    return out


def span(x, fmt='%.2f'):
    return ('%s to %s' % (fmt % min(x), fmt % max(x))) if x else 'none'


# ---------------------------------------------------------------------------------------------------------------
# scripts

def start(h, ka=0.5e6):
    """The bus idle at power-up, then keep-alives every frame."""
    yield from h.wait(0.1e6)
    h.next_frame = h.t + ka


def enumerate_(h, addr=5, second_reset=True, lose=()):
    """What a host does first (9.1.2): reset; GET_DESCRIPTOR(DEVICE, 64) at address 0, which gets all 18 bytes;
    reset again; SET_ADDRESS; the 2 ms recovery (9.2.6.3); GET_DESCRIPTOR(DEVICE, 18) at the new address; then a
    SETUP to the old address, which must go unanswered."""
    yield from h.wait(0.2e6)
    yield from h.reset()
    yield from h.wait(1.0e6)
    d1 = yield from h.control_read(0, GET_DEV(64), lose=lose)
    if second_reset:
        yield from h.reset()
        yield from h.wait(1.0e6)
    ok = yield from h.control_nodata(0, SET_ADDR(addr))
    yield from h.wait(2.2e6)
    d2 = yield from h.control_read(addr, GET_DEV(18))
    old = yield from h.setup(0, GET_DEV(8), tries=1)
    yield from h.wait(0.3e6)
    return dict(d1=d1, set=ok, d2=d2, old=old)


def quick(h, addr=9, wl=18):
    """Power-up enumeration without resets, the device at address 0 from reset: about 6 ms."""
    yield from start(h)
    d1 = yield from h.control_read(0, GET_DEV(wl))
    ok = yield from h.control_nodata(0, SET_ADDR(addr))
    d2 = yield from h.control_read(addr, GET_DEV(18))
    yield from h.wait(0.1e6)
    return dict(d1=d1, set=ok, d2=d2)


def faults(h):
    """NAK when nothing is due; tokens for another address and for endpoints 1 and 2; a SETUP whose data has a bad
    CRC16; a SETUP, an IN and an OUT whose token CRC5 is bad; an IN after the data stage (NAK); unsupported requests
    (STALL until the next SETUP); then a whole transfer."""
    yield from start(h)
    r = {}
    r['idle'] = (yield from h.in_(0))[0]                            # NAK: no transfer open
    r['other_setup'] = yield from h.setup(3, GET_DEV(18), tries=1)  # another address: no answer
    r['other_in'] = (yield from h.in_(3))[0]
    r['ep1_setup'] = yield from h.setup(0, GET_DEV(18), endp=1, tries=1)
    r['ep2_in'] = (yield from h.in_(0, endp=2))[0]
    r['bad_setup'] = yield from h.setup(0, GET_DEV(18), bad=True)   # corrupted data: no handshake
    r['bad_setup_tok'] = yield from h.setup(0, GET_DEV(18), bad_token=True)  # corrupted token: its data unanswered
    r['setup'] = yield from h.setup(0, GET_DEV(18))
    r['bad_in'] = (yield from h.in_(0, bad_token=True))[0]          # corrupted token: no answer
    data = b''
    for _ in range(3):
        kind, pk = yield from h.in_(0)
        data += pk.payload if kind == 'data' else b''
    r['data'] = data
    r['late_in'] = (yield from h.in_(0))[0]                         # the data stage is over: NAK
    r['bad_out_tok'] = yield from h.out(0, [], 'DATA1', bad_token=True)  # corrupted token: no handshake
    r['status'] = yield from h.out(0, [], 'DATA1')
    r['config'] = yield from h.setup(0, [0x80, 6, 0, 2, 0, 0, 9, 0])  # GET_DESCRIPTOR(CONFIGURATION): unsupported
    r['config_in'] = (yield from h.in_(0))[0]
    r['config_out'] = yield from h.out(0, [], 'DATA1')
    r['setconf'] = yield from h.setup(0, [0, 9, 1, 0, 0, 0, 0, 0])   # SET_CONFIGURATION: unsupported
    r['setconf_in'] = (yield from h.in_(0))[0]
    r['after'] = yield from h.control_read(0, GET_DEV(18))
    yield from h.wait(0.1e6)
    return r


FAULTS_WANT = dict(idle='NAK', other_setup=False, other_in='none', ep1_setup=False, ep2_in='none', bad_setup=False,
                   bad_setup_tok=False, setup=True, bad_in='none', data=DESC, late_in='NAK', bad_out_tok=None,
                   status='ACK', config=True, config_in='STALL',
                   config_out='STALL', setconf=True, setconf_in='STALL', after=DESC)


def lost(h):
    """Lost ACKs: the host loses its ACK of the second data packet, of the first status packet of SET_ADDRESS, and
    of the first and third data packets at the new address."""
    yield from start(h)
    d1 = yield from h.control_read(0, GET_DEV(18), lose=(1,))
    ok = yield from h.control_nodata(0, SET_ADDR(6), lose_status=1)
    yield from h.wait(2.2e6)
    d2 = yield from h.control_read(6, GET_DEV(18), lose=(0, 2))
    yield from h.wait(0.1e6)
    return dict(d1=d1, set=ok, d2=d2)


def interrupted(h):
    """A SETUP that interrupts a transfer, and a reset in the middle of one."""
    yield from start(h)
    r = {}
    r['setup1'] = yield from h.setup(0, GET_DEV(64))
    r['first'] = (yield from h.in_(0))[0]                           # one packet of the data stage, then a new SETUP
    r['d1'] = yield from h.control_read(0, GET_DEV(18))
    r['set'] = yield from h.control_nodata(0, SET_ADDR(7))
    yield from h.wait(2.2e6)
    r['setup2'] = yield from h.setup(7, GET_DEV(18))
    r['second'] = (yield from h.in_(7))[0]                          # then a reset in the data stage
    yield from h.reset()
    yield from h.wait(1.0e6)
    r['d2'] = yield from h.control_read(0, GET_DEV(18))              # the Default state: address 0
    r['old'] = yield from h.setup(7, GET_DEV(8), tries=1)
    yield from h.wait(0.1e6)
    return r


INTERRUPTED_WANT = dict(setup1=True, first='data', d1=DESC, set=True, setup2=True, second='data', d2=DESC, old=False)


def dribble(h):
    """One dribble bit (7.1.9.1: a receiver must accept up to six bit times with no transition before the EOP) on
    the token of an IN in a data stage and on the data packet of a SETUP, each next to the same transaction sent
    clean."""
    yield from start(h)
    r = {}
    r['setup'] = yield from h.setup(0, GET_DEV(18))
    r['in_dribble'] = (yield from h.in_(0, dribble=1))[0]
    r['in'] = (yield from h.in_(0))[0]
    r['setup_dribble'] = yield from h.setup(0, GET_DEV(18), dribble=1, tries=1)
    r['setup2'] = yield from h.setup(0, GET_DEV(18))
    yield from h.wait(0.1e6)
    return r


def stale(h):
    """A SET_ADDRESS whose status packet the host never ACKs, so its status stage never completes, then a request with
    no data stage, GET_DESCRIPTOR(DEVICE) with wLength 0, done to its status ACK: a SETUP ends the transfer before it
    (8.5.3), so the address stays 0 (9.4.6), and the device then answers at 0 and not at the address it was given."""
    yield from start(h)
    r = {}
    r['set'] = yield from h.setup(0, SET_ADDR(5))
    r['status'] = (yield from h.in_(0, ack=False))[0]              # the zero-length status packet, never ACKed
    r['nodata'] = yield from h.control_nodata(0, GET_DEV(0))
    yield from h.wait(2.2e6)
    r['at0'] = yield from h.control_read(0, GET_DEV(18))
    r['at5'] = yield from h.setup(5, GET_DEV(8), tries=1)
    yield from h.wait(0.1e6)
    return r


STALE_WANT = dict(set=True, status='data', nodata=True, at0=DESC, at5=False)


def good_enum(r, addr=5):
    v = r.value or {}
    return v.get('d1') == r.desc and v.get('set') is True and v.get('d2') == r.desc[:18] and v.get('old') is False and \
        r.count['ADDRV'] == addr


def good_quick(r, addr=9):
    v = r.value or {}
    return v.get('d1') == r.desc and v.get('set') is True and v.get('d2') == r.desc and r.count['ADDRV'] == addr


def summary(rep, keys=('response_time', 'retry', 'address', 'reset', 'toggle', 'sequence', 'descriptor')):
    c = rep.checks()
    return ', '.join('%s %d' % (k, c[k][0]) for k in keys)


def device_stats(reps):
    """Over the device's packets of the given reports: bit rate (Mb/s), worst jitter (ns, against the packet's own
    period: next, paired), SE0 width (ns), SE0 offset from the bit grid (ns), J driven (ns)."""
    rate, nxt, par, se0, off, jd = [], [], [], [], [], []
    for rep in reps:
        for pk in rep.packets:
            if pk.sender != 'dev' or pk.period is None:
                continue
            rate.append(1e3 / pk.period)
            times = [x for x, _ in pk.trans]
            w = [0.0, 0.0]
            for k in range(1, len(times)):
                for gap, i in ((1, 0), (2, 1)):
                    if k - gap >= 0:
                        w[i] = max(w[i], abs(times[k] - times[k - gap] - (pk.cells[k] - pk.cells[k - gap]) *
                                             pk.period))
            nxt.append(w[0])
            par.append(w[1])
            se0.append(pk.end - pk.se0)
            off.append(pk.se0 - (pk.sop + pk.n_raw * pk.period))
            if pk.j_drive is not None:
                jd.append(pk.j_drive)
    return rate, nxt, par, se0, off, jd


def critical_path(text=None):
    """The cycles from S's time reference (its `now` after a packet's last word) to the `tick` that starts the
    answer, on the simulator's execute log of a power-up enumeration: the worst case per kind of answer."""
    img = assemble(text if text is not None else src(PROG), depth=512)
    ref = img.labels['rend']
    ticks = {img.labels['shs'] + 4: 'handshake', img.labels['szlp'] + 6: 'zero-length', img.labels['sdata'] + 7: 'data'}

    class Logged(Chip):
        def __init__(self, *a, **k):
            Chip.__init__(self, *a, **k)
            self.log = []
    r = run(quick, 12e6, chip_cls=Logged, check=False)
    worst, at = {}, None
    for c, tid, pc, word in r.chip.log:
        if tid != 0:
            continue
        if pc == ref:
            at = c
        elif pc in ticks and at is not None:
            k = ticks[pc]
            worst[k] = max(worst.get(k, 0), c - at)
            at = None
    return worst


# ---------------------------------------------------------------------------------------------------------------
# broken copies: each must fail somewhere

TEETH = [
    ('the toggle not flipped on the host\'s ACK', 'li    R4, WDATA0 ^ WDATA1', 'li    R4, 0'),
    ('the new address taken at the SETUP, before the status stage', 'st    R1, R7         ; taken when',
     'st    R1, R7\n        ldi   R7, ADDRV\n        st    R1, R7\n        cp    R6, R1\n        swap  R6\n'
     '        ori   R6, 0x69\n        ldi   R7, PADDR      ; taken when'),
    ('the answer 7 bit times after the EOP (NRSP 260)', '.equ NRSP 176', '.equ NRSP 260'),
    ('the host\'s data ACKed without its CRC-16 check', 'srd   R4, end\n        cmpi  R4, 1',
     'ldi   R4, 1\n        cmpi  R4, 1'),
    ('the CRC-5 verdict ignored on IN', 'xor   R7, R3\n        cmpi  R7, 1          ; A\'s verdict',
     'ldi   R7, 1\n        cmpi  R7, 1          ; A\'s verdict'),
    ('a fraction on the bit time (TFRAC 85): edges off the cycle grid', 'ldi   R0, 67\n        scfg  R0, TXTAIL',
     'li    R0, 0x5543\n        scfg  R0, TXTAIL'),
    ('an EOP of 50 cycles, 1.0 us', 'ldi   R0, 67\n        scfg  R0, TXTAIL', 'ldi   R0, 50\n        scfg  R0, TXTAIL'),
    ('J released 3 cycles after the EOP (NREL 0)', '.equ NREL 31', '.equ NREL 0'),
    ('resets not counted', 'st    R0, R4         ; a reset: S acts on it before the next packet', 'idle'),
    ('min(wLength, 18) not applied', 'ldi   R0, 18         ; min(wLength, 18) bytes (9.4.3)', 'idle'),
    ('the status packet sent as DATA0', 'li    R0, WDATA1     ; status packets are DATA1 (8.5.3)', 'li    R0, WDATA0'),
    ('no stuffing on send (SMODE 0)', '.equ CODTX 0xE6EF', '.equ CODTX 0xE2EF'),
]


def broken(old, new):
    text = src(PROG)
    assert text.count(old) == 1, old
    return text.replace(old, new)


# ---------------------------------------------------------------------------------------------------------------

def main(log):
    log.group(PROG, title='USB low-speed device on endpoint 0, enumerated by a host model at 1.5 Mb/s, 50 MHz')
    img = assemble(src(PROG), depth=512)
    log.check('assembles to %d words, the count in section 11.2; the image ends below the 512-word memory' % WORDS,
              img.size == WORDS and max(img.words) < 512, '%d words placed, last address %d' % (
                  img.size, max(img.words)))
    reps, resp = [], []

    def note(r):
        reps.append(r.rep)
        resp.extend(gaps(r))

    # the enumeration a host does after attach, with its two resets
    r = run(enumerate_, 40e6)
    note(r)
    v = verdict(r)
    log.check('enumeration with two 10 ms resets: every usb_check check passes, threads healthy', not v,
              v or summary(r.rep))
    log.check('enumeration: the host reads the 18-byte descriptor at address 0 and at 5, SET_ADDRESS completes, '
              'address 0 then goes unanswered', good_enum(r), '%s, counters %s' % (r.value, r.count))
    log.check('enumeration: the device counts 2 resets, 3 SETUPs and 3 completed transfers', r.count['RSTN'] == 2 and
              r.count['NSETUP'] == 3 and r.count['NREQ'] == 3, r.count)

    # placements, alignments and sampling phases
    configs = [((0, 1, 2), 0, 0.0), ((1, 2, 3), 1, 2.5), ((2, 3, 0), 2, 5.0), ((3, 0, 1), 3, 7.5),
               ((3, 2, 1), 0, 10.0), ((1, 0, 3), 1, 12.5), ((2, 1, 0), 2, 15.0), ((0, 3, 2), 3, 17.5)]
    for th, al, ph in configs:
        r = run(quick, 12e6, threads=th, align=al, phase=ph)
        note(r)
        v = verdict(r)
        log.check('power-up enumeration, S on thread %d, A on %d, W on %d, alignment %d, sampling phase %.1f ns' % (
            th + (al, ph)), not v and good_quick(r), v or summary(r.rep))

    # the host's bit rate at both ends of TLDRATHS (Table 7-10), edge jitter, the shortest inter-packet delays,
    # and EOPs from TLEOPT's 1.25 to 1.50 us down to the 670 ns a receiver must accept (TLEOPR)
    cases = [('host 0.05% fast (TLDRATHS)', dict(period=BIT / 1.0005)),
             ('host 0.05% slow (TLDRATHS)', dict(period=BIT / 0.9995)),
             ('edges moved +/-22 ns at random: 44 ns between transitions, the jitter of a host attached directly '
              '(Table 7-5)', dict(jitter=22.0)),
             ('edges moved +/-70 ns at random: 140 ns, the function receiver\'s budget through hubs (Table 7-5)',
              dict(jitter=70.0)),
             ('every inter-packet delay at 2 bit times (7.1.18.1)', dict(gap_data=2.0, gap_hs=2.0, gap_tr=2.0)),
             ('the host\'s D- 100 ns behind its D+: SE0 and SE1 inside TLST at each transition (7.1.4.1)',
              dict(skew=100.0)),
             ('the host\'s D- 200 ns behind its D+ (TLST 210 ns)', dict(skew=200.0)),
             ('host EOPs of 1.25 us (TLEOPT)', dict(eop=1.25e3 / BIT)),
             ('host EOPs of 1.50 us (TLEOPT)', dict(eop=1.5e3 / BIT)),
             ('host EOPs of 670 ns (TLEOPR)', dict(eop=670.0 / BIT))]
    for i, (what, kw) in enumerate(cases):
        r = run(quick, 12e6, host_kw=kw, phase=2.5 * i, align=i % 4)
        note(r)
        v = verdict(r)
        log.check('power-up enumeration, %s' % what, not v and good_quick(r),
                  v or 'response %s bit times' % span(gaps(r)))
    for ph in (0.0, 5.0, 10.0, 15.0):
        r = run(quick, 12e6, host_kw=dict(eop=670.0 / BIT), phase=ph, align=1)
        note(r)
        v = verdict(r)
        log.check('power-up enumeration, host EOPs of 670 ns at sampling phase %.0f ns' % ph, not v and good_quick(r),
                  v or 'response %s bit times' % span(gaps(r)))
    r = run(quick, 12e6, host_kw=dict(gap_data=2.0, gap_hs=2.0, gap_tr=2.0, period=BIT / 1.0005, eop=670.0 / BIT),
            phase=17.5)
    note(r)
    v = verdict(r)
    log.check('power-up enumeration, all three at once: 2-bit delays, 0.05% fast, 670 ns EOPs', not v and good_quick(r),
              v or 'response %s bit times' % span(gaps(r)))

    # the board: the device's D- lagging its D+ by 40 ns (SE0 and SE1 inside TLST at its transitions, 7.1.4.1),
    # an 18 ns cable each way (7.1.1.2), and a descriptor whose packet ends in six 1s, so a stuffed 0 comes last
    for what, kw in (('the device\'s D- 40 ns behind its D+', dict(bus_kw=dict(dm_lag=40.0))),
                     ('an 18 ns cable each way', dict(bus_kw=dict(t_cable=18.0))),
                     ('a descriptor whose data packet ends in a stuffed 0 before the EOP (7.1.9)',
                      dict(desc=G.descriptor()))):
        r = run(quick, 12e6, **kw)
        note(r)
        v = verdict(r)
        log.check('power-up enumeration, %s' % what, not v and good_quick(r), v or summary(r.rep, ('stuffing',
                                                                                              'line_state')))

    # faults, lost ACKs, interruptions
    r = run(faults, 30e6)
    note(r)
    v = verdict(r)
    diff = {k: (r.value or {}).get(k) for k in FAULTS_WANT if (r.value or {}).get(k) != FAULTS_WANT[k]}
    log.check('NAK with nothing due, other addresses and endpoints ignored, corrupted SETUP data and corrupted '
              'SETUP, IN and OUT tokens unanswered, NAK after the data stage, STALL for unsupported requests until the '
              'next SETUP',
              not v and not diff, v or diff or summary(r.rep))
    r = run(lost, 20e6)
    note(r)
    v = verdict(r)
    c = r.rep.checks()
    log.check('lost ACKs: the same data packet sent again with the same toggle, and the old address kept until the '
              'status ACK (8.6.4, 8.5.3.1)', not v and (r.value or {}).get('d1') == DESC and
              (r.value or {}).get('d2') == DESC and (r.value or {}).get('set') is True and c['retry'][0] >= 3 and
              r.count['ADDRV'] == 6, v or '%s, %s' % (summary(r.rep), r.value))
    r = run(stale, 20e6)
    note(r)
    v = verdict(r)
    diff = {k: (r.value or {}).get(k) for k in STALE_WANT if (r.value or {}).get(k) != STALE_WANT[k]}
    log.check('a SET_ADDRESS left in its status stage, then a request with no data stage: the address stays 0 (8.5.3, '
              '9.4.6)', not v and not diff and r.count['ADDRV'] == 0, v or diff or 'address %d' % r.count['ADDRV'])
    r = run(interrupted, 30e6)
    note(r)
    v = verdict(r)
    diff = {k: (r.value or {}).get(k) for k in INTERRUPTED_WANT if (r.value or {}).get(k) != INTERRUPTED_WANT[k]}
    log.check('a SETUP in the middle of a data stage starts the new transfer; a reset in the middle of one returns '
              'the device to address 0 and the old address goes unanswered', not v and not diff and
              r.count['ADDRV'] == 0, v or diff or summary(r.rep))

    # margins
    rate, nxt, par, se0, off, jd = device_stats(reps)
    log.check('response time over %d answers in these runs: %s bit times after the host\'s SE0-to-J, inside 2 to '
              '6.5 (7.1.18.1, TRSPIPD1)' % (len(resp), span(resp)), resp and 2.0 <= min(resp) and max(resp) <= 6.5)
    log.info('margins, device packets: bit rate %s Mb/s (TLDRATE 1.4775 to 1.5225); jitter against the packet\'s own '
             'period %s ns next and %s paired (25 and 10, 7.1.13.1.1); SE0 %s ns (TLEOPT 1250 to 1500), %s ns from the '
             'bit grid (TLDEOP -40 to +100); J driven %s ns (one bit, %.0f to %.0f)' % (
                 span(rate, '%.4f'), span(nxt, '%.1f'), span(par, '%.1f'), span(se0, '%.0f'), span(off, '%+.0f'),
                 span(jd, '%.0f'), 1e3 / 1.5152 - 25, 2e3 / 1.5152))
    cp = critical_path()
    log.info('margins: from the time reference to the start, worst %s cycles, against %d (NRSP): %s' % (
        max(cp.values()) if cp else '?', 176, ', '.join('%s %d' % kv for kv in sorted(cp.items()))))

    # past the standard, for the record: the host's bit rate and jitter the device still follows
    for what, kws in (('host bit rate error', [('%+.0f%%' % (100 * e), dict(period=BIT / (1 + e)))
                                               for e in (0.03, -0.03, 0.06, -0.06, 0.08, -0.08, 0.10, -0.10)]),
                      ('host edge jitter', [('+/-%d ns' % j, dict(jitter=float(j)))
                                            for j in (100, 160, 190, 220, 250)])):
        out = []
        for name, kw in kws:
            r = run(quick, 12e6, host_kw=kw, seed=3, phase=5.0)
            out.append('%s %s' % (name, 'enumerates' if good_quick(r) and not r.health else 'FAILS'))
        log.info('past the standard, %s: %s' % (what, '; '.join(out)))

    # a dribble bit before the host's EOP (7.1.9.1): the units' end test covers every bit up to the SE0, so the CRC
    # checks fail and the packet goes unanswered, a limit of the shift units (section 5.9.7, 11.5)
    r = run(dribble, 12e6)
    d = r.value or {}
    fails = r.rep.failed()
    log.info('receive limit: one dribble bit before an EOP (7.1.9.1): an IN token with it is %s and a SETUP\'s data '
             'packet with it is %s; usb_check, which takes both as good packets, fails %s' % (
                 'unanswered' if d.get('in_dribble') == 'none' else 'answered %s' % d.get('in_dribble'),
                 'not ACKed' if d.get('setup_dribble') is False else 'ACKed', sorted({c for _, c, _ in fails})))
    log.check('after a dribbled IN and a dribbled SETUP, the same transactions sent clean are answered; threads '
              'healthy', d.get('setup') is True and d.get('in') == 'data' and d.get('setup2') is True and
              not r.health, '%s %s' % (d, r.health))

    # broken copies
    for what, old, new in TEETH:
        text = broken(old, new)
        bad = []
        for script, t_end in ((quick, 12e6), (faults, 30e6), (lost, 20e6), (interrupted, 30e6), (enumerate_, 40e6)):
            r = run(script, t_end, text=text)
            v = verdict(r)
            want = {quick: good_quick, faults: lambda x: not {k for k in FAULTS_WANT if (x.value or {}).get(k) !=
                                                               FAULTS_WANT[k]},
                    lost: lambda x: (x.value or {}).get('d2') == DESC,
                    interrupted: lambda x: not {k for k in INTERRUPTED_WANT if (x.value or {}).get(k) !=
                                                INTERRUPTED_WANT[k]}, enumerate_: good_enum}[script]
            if v or not want(r):
                bad.append('%s: %s' % (script.__name__, v[:120] or 'wrong result'))
                break
        log.check('a broken copy is caught: %s' % what, bool(bad), bad[0] if bad else 'every check passed')
