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
MEM = dict(STAGE=0, ADDRV=13, NSETUP=14, NREQ=15, RSTN=11, CFGV=3)
CODE_FROM = 25                  # the descriptor and the code: never written by the program
DESC = bytes([18, 1, 0x10, 0x01, 0, 0, 0, 8, 0x09, 0x12, 0x01, 0x00, 0x00, 0x01, 0, 0, 0, 1])
DESC_AT = 25                    # where the image keeps it, two bytes a word
J, K, SE0, SE1 = 'J', 'K', 'SE0', 'SE1'
LEVELS = {J: (0, 1), K: (1, 0), SE0: (0, 0), SE1: (1, 1)}       # (D+, D-) at low speed (Table 7-2)
PIDS = {0b0011: 'DATA0', 0b1011: 'DATA1', 0b0010: 'ACK', 0b1010: 'NAK', 0b1110: 'STALL'}
GET_DEV = lambda n: [0x80, 6, 0, 1, 0, 0, n & 255, n >> 8]                  # GET_DESCRIPTOR(DEVICE), 9.4.3
SET_ADDR = lambda a: [0, 5, a, 0, 0, 0, 0, 0]                                 # SET_ADDRESS, 9.4.6
GET_CONF = lambda n: [0x80, 6, 0, 2, 0, 0, n & 255, n >> 8]                 # GET_DESCRIPTOR(CONFIGURATION), 9.4.3
SET_CONF = lambda v: [0, 9, v & 255, v >> 8, 0, 0, 0, 0]                      # SET_CONFIGURATION, 9.4.7
GET_CFG = [0x80, 8, 0, 0, 0, 0, 1, 0]                                           # GET_CONFIGURATION, 9.4.2
GET_STATUS = lambda bm, i: [bm, 0, 0, 0, i, 0, 2, 0]                            # GET_STATUS, 9.4.5: 0x80 the
                                                                                # device, 0x81 an interface, 0x82
                                                                                # an endpoint
GET_IF = lambda i: [0x81, 10, 0, 0, i, 0, 1, 0]                                 # GET_INTERFACE, 9.4.4
SET_IF = lambda i, alt: [1, 11, alt, 0, i, 0, 0, 0]                             # SET_INTERFACE, 9.4.10
FEATURE = lambda op, bm, f, i: [bm, op, f, 0, i, 0, 0, 0]                       # CLEAR_FEATURE (1), SET_FEATURE (3)
# the configuration descriptor (9.6.3, Table 9-10: wTotalLength 18, one interface, value 1, bus powered, 100 mA) and its
# interface descriptor (9.6.5, Table 9-12: interface 0, no endpoints but endpoint 0, vendor-specific class), field by
# field from the tables
CONF = bytes([9, 2, 18, 0, 1, 1, 0, 0x80, 50]) + bytes([9, 4, 0, 0, 0, 0xFF, 0, 0, 0])


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

def stuff_bits(raw):
    """Where usb_check_test.stuff put its stuff bits in a packet's line bits: after six 1s in a row, the SYNC's
    last 1 counted (7.1.9)."""
    pos, run, i = [], 1, 8
    while i < len(raw):
        if run == 6:
            pos.append(i)
            run = 0
        else:
            run = run + 1 if raw[i] else 0
        i += 1
    return pos


class Host:
    """A low-speed host at the device's pins. Times in ns. A script is a generator of Host methods (yield from);
    each yield is the time to resume at. period: its bit time; jitter: each transition moved by a uniform amount up
    to that many ns; eop: its SE0 in bit times; gap_data, gap_hs, gap_tr: inter-packet delays in bit times before a
    token's data packet, before its handshake to the device's data, and between transactions."""

    def __init__(self, bus, period=BIT, jitter=0.0, eop=2.0, gap_data=3.0, gap_hs=3.0, gap_tr=4.0, seed=1,
                 frame=1e6, timeout=17.0, dribble=0, skew=0.0, dribble_fit=False):
        self.dribble_fit = dribble_fit  # dribble at most 6 bit times with no transition with the packet's own 1s
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
    def tx(self, bits, t0, eop=None, jitter=None, dribble=None, nostuff=None):
        """Schedules one packet from J at t0: SYNC and stuffing (usb_check_test.stuff), NRZI, then the EOP: SE0 for
        eop bit times, J driven for one bit, released. dribble: bit times of the last level added before the SE0
        (7.1.9.1). skew: D- moves that many ns after D+ at each J-K transition, so the line passes through SE1 or
        SE0 (7.1.4.1: up to TLST). nostuff: leaves out the packet's stuff bit of that number (0 the first), so
        seven or more 1s go out in a row, a bit-stuff error (7.1.9). Returns the SE0-to-J time."""
        eop = self.eop if eop is None else eop
        jit = self.jitter if jitter is None else jitter
        drib = self.dribble if dribble is None else dribble
        raw = G.stuff(bits)
        if nostuff is not None:
            del raw[stuff_bits(raw)[nostuff]]
        if self.dribble_fit and drib:   # 7.1.9.1: up to six bit times with no transition before the EOP, counting
            run1 = 0                    # the 1s the packet ends with (no transition in NRZI)
            while run1 < len(raw) and raw[-1 - run1] == 1:
                run1 += 1
            drib = max(0, min(drib, 6 - run1))
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
    def setup(self, addr, req, endp=0, bad=False, tries=3, dribble=None, bad_token=False, nostuff=None, extra=()):
        """SETUP and DATA0 with the 8 bytes of req. True once the device ACKs. dribble: bit times added before the
        data packet's EOP. bad_token: a CRC5 bit of the SETUP token flipped. nostuff: the data packet's stuff bit of
        that number left out (Host.tx). extra: bits sent after the data packet's CRC16, before its EOP."""
        for _ in range(tries):
            yield from self.frame_check()
            tokb = G.token('SETUP', addr, endp)
            if bad_token:
                tokb = list(tokb)
                tokb[-1] ^= 1               # a CRC5 bit
            yield from self.send(tokb, self.gap_tr)
            t = yield from self.send(G.data('DATA0', req, bad_crc=bad) + list(extra), self.gap_data,
                                     dribble=dribble, nostuff=nostuff)
            pk = yield from self.answer(t)
            self.log.append(('setup', addr, bytes(req), pk.name if pk else None))
            if pk is not None and pk.ok and pk.name == 'ACK':
                return True
            if bad or bad_token or nostuff is not None or extra:
                return False
        return False

    def in_(self, addr, endp=0, ack=True, bad_token=False, dribble=None, ack_dribble=None, token=None):
        """One IN transaction: ('data', Rx), ('NAK', Rx), ('STALL', Rx), ('bad', Rx) or ('none', None). The host
        ACKs a good data packet unless ack is False (its ACK lost). dribble, ack_dribble: the token's and that ACK's
        dribble. token: the token's bits instead of a good IN token."""
        yield from self.frame_check()
        tokb = G.token('IN', addr, endp) if token is None else token
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
                yield from self.send(G.handshake('ACK'), self.gap_hs, dribble=ack_dribble)
            self.log.append(('in', addr, pk.name, pk.payload, ack))
            return 'data', pk
        self.log.append(('in', addr, pk.name))
        return pk.name, pk

    def out(self, addr, payload, toggle='DATA1', endp=0, bad=False, bad_token=False, dribble=None, tok_dribble=None):
        """OUT and its data packet: the device's handshake name, or None. bad_token: a CRC5 bit of the token
        flipped. dribble, tok_dribble: the data packet's and the token's dribble."""
        yield from self.frame_check()
        tokb = G.token('OUT', addr, endp)
        if bad_token:
            tokb = list(tokb)
            tokb[-1] ^= 1                   # a CRC5 bit
        yield from self.send(tokb, self.gap_tr, dribble=tok_dribble)
        t = yield from self.send(G.data(toggle, payload, bad_crc=bad), self.gap_data, dribble=dribble)
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
    res.rep = usb_check.check(res.rec, descriptor=res.desc, config=CONF) if check else None
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
    r['string'] = yield from h.setup(0, [0x80, 6, 0, 3, 0, 0, 9, 0])  # GET_DESCRIPTOR(STRING 0): unsupported, no strings
    r['string_in'] = (yield from h.in_(0))[0]
    r['string_out'] = yield from h.out(0, [], 'DATA1')
    r['setconf'] = yield from h.setup(0, SET_CONF(2))   # SET_CONFIGURATION(2): no such configuration (9.4.7)
    r['setconf_in'] = (yield from h.in_(0))[0]
    r['after'] = yield from h.control_read(0, GET_DEV(18))
    yield from h.wait(0.1e6)
    return r


FAULTS_WANT = dict(idle='NAK', other_setup=False, other_in='none', ep1_setup=False, ep2_in='none', bad_setup=False,
                   bad_setup_tok=False, setup=True, bad_in='none', data=DESC, late_in='NAK', bad_out_tok=None,
                   status='ACK', string=True, string_in='STALL',
                   string_out='STALL', setconf=True, setconf_in='STALL', after=DESC)


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
    """One dribble bit (7.1.9.1: the last bit before the EOP stretched by a bit time, which a receiver must accept)
    on one packet at a time, the others clean: the data packet of a SETUP; the token of an IN in the data stage; the
    host's ACK of that data, which must move the device on to the next packet; the token and the zero-length data
    packet of the status OUT; and the ACK of SET_ADDRESS's status packet, which must make the new address take
    effect."""
    yield from start(h)
    r = {}
    r['setup'] = yield from h.setup(0, GET_DEV(18), dribble=1, tries=1)
    kind, pk = yield from h.in_(0, dribble=1, ack_dribble=1)
    r['in1'] = (kind, pk.name if pk else None, pk.payload if pk else None)
    kind, pk = yield from h.in_(0)
    r['in2'] = (kind, pk.name if pk else None, pk.payload if pk else None)
    kind, pk = yield from h.in_(0, ack_dribble=1)
    r['in3'] = (kind, pk.name if pk else None, pk.payload if pk else None)
    r['status_tok'] = yield from h.out(0, [], 'DATA1', tok_dribble=1)
    r['setup2'] = yield from h.setup(0, GET_DEV(18))
    for _ in range(3):
        yield from h.in_(0)
    r['status_data'] = yield from h.out(0, [], 'DATA1', dribble=1)
    r['set'] = yield from h.setup(0, SET_ADDR(12))
    kind, pk = yield from h.in_(0, ack_dribble=1)
    r['set_status'] = (kind, pk.name if pk else None, pk.payload if pk else None)
    yield from h.wait(2.2e6)
    r['at12'] = yield from h.control_read(12, GET_DEV(18))
    yield from h.wait(0.1e6)
    return r


DRIBBLE_WANT = dict(setup=True, in1=('data', 'DATA1', DESC[:8]), in2=('data', 'DATA0', DESC[8:16]),
                    in3=('data', 'DATA1', DESC[16:]), status_tok='ACK', setup2=True, status_data='ACK', set=True,
                    set_status=('data', 'DATA1', b''), at12=DESC)


def crc_reach(state, width, poly, target, n, fixed=()):
    """The first n bits, after the bits `fixed`, that take a CRC register of `width` bits from `state` to `target`:
    the register as USB 2.0 8.3.5 defines it (the x^(width-1) coefficient first, each bit XORed with it as it
    enters, the generator's lower terms in poly), searched exhaustively. None if no n bits do it."""
    mask = (1 << width) - 1

    def feed(s, bits):
        for b in bits:
            f = ((s >> (width - 1)) & 1) ^ b
            s = (s << 1) & mask
            if f:
                s ^= poly
        return s
    s0 = feed(state, fixed)
    for v in range(1 << n):
        bits = [(v >> i) & 1 for i in range(n)]
        if feed(s0, bits) == target:
            return list(fixed) + bits
    return None


# A good packet's register ends at the residue, 01100 for CRC5 (8.3.5.1) and 1000000000001101 for CRC16 (8.3.5.2);
# one more 1 bit, a dribble bit, takes it to 11101 and 0000000000011010. These extra bits reach those values too,
# but are not a dribble bit: 6 bits after a token, the first a 1 as a dribble bit would be, and 19 after a data
# packet's CRC16.
EXTRA_TOK = crc_reach(0b01100, 5, 0b00101, 0b11101, 5, fixed=(1,))
# 5 bits after a token that take its CRC5 register back to the residue itself, with 0s among them, so the byte end
# keeps them (section 5.9.7): a 29-bit token whose check passes, which only its length refuses (8.4.1: 3 bytes)
LEN_TOK = crc_reach(0b01100, 5, 0b00101, 0b01100, 5)
EXTRA_DATA = crc_reach(0x800D, 16, 0x8005, 0x001A, 16, fixed=(0, 1, 0))
# 7 0s, then 16 bits that take the CRC16 register from its residue back to the residue: 23 bits after a data packet's
# CRC16, so its check passes, but its data field is no longer whole bytes (8.3.4), and with 7 bits past the last byte
# boundary the byte end keeps them (section 5.9.7). Only the length refuses it. No suffix of 1 to 7 bits returns a
# 16-bit register to its residue, so a shorter one would fail the check itself.
LEN_DATA = crc_reach(0x800D, 16, 0x8005, 0x800D, 16, fixed=(0,) * 7)


def wrong_crc5_then_0(tok):
    """The token tok (wire order: PID, 11 field bits, CRC5) with its CRC5 replaced by the one wrong value after which a
    0 bit takes the register to the dribble value 11101, then that 0: a corrupted token (8.3.5.1) that a check of the
    register and the bit count alone would take for a good token with a dribble bit. A dribble bit has no transition,
    so in NRZI it is a 1 (7.1.9.1), and a 0 there is no dribble bit."""
    field = list(tok[8:19])
    mask, out = 0b11111, []
    for v in range(32):
        bits = [(v >> i) & 1 for i in range(5)]
        s = 0b11111
        for b in field + bits + [0]:
            f = ((s >> 4) & 1) ^ b
            s = (s << 1) & mask
            if f:
                s ^= 0b00101
        if s == 0b11101 and bits != list(tok[19:24]):
            out.append(bits)
    assert len(out) == 1, out
    return list(tok[:19]) + out[0] + [0]


BAD_TOK0 = wrong_crc5_then_0(G.token('IN', 0, 0))


def near_misses(h):
    """Packets with bits after their last whole byte that are not a dribble bit (7.1.9.1 allows only bits with no
    transition): a 0 after an IN token and after a SETUP's data packet; and bits that take the CRC register to the
    value a dribble bit gives, but are too many (EXTRA_TOK, EXTRA_DATA), or are a 0 after a token whose CRC5 is wrong
    (BAD_TOK0). All must go unanswered (8.7.1: the CRC
    fails); the IN after them gets the first data packet; then a whole transfer. A handshake has no CRC, only its
    PID check (8.7.1), so an ACK with one more bit of either value is taken as an ACK, as bench/usb_check.py also
    reads it."""
    yield from start(h)
    r = {}
    r['setup'] = yield from h.setup(0, GET_DEV(18))
    r['in0'] = (yield from h.in_(0, token=G.token('IN', 0, 0) + [0]))[0]
    r['in_crc'] = (yield from h.in_(0, token=G.token('IN', 0, 0) + EXTRA_TOK))[0]
    r['in_bad0'] = (yield from h.in_(0, token=BAD_TOK0))[0]
    r['in_len'] = (yield from h.in_(0, token=G.token('IN', 0, 0) + LEN_TOK))[0]
    kind, pk = yield from h.in_(0)
    r['in1'] = (kind, pk.name if pk else None, pk.payload if pk else None)
    r['setup0'] = yield from h.setup(0, GET_DEV(18), extra=[0], tries=1)
    r['setup_crc'] = yield from h.setup(0, GET_DEV(18), extra=EXTRA_DATA, tries=1)
    r['after'] = yield from h.control_read(0, GET_DEV(18))
    yield from h.wait(0.1e6)
    return r


NEAR_WANT = dict(setup=True, in0='none', in_crc='none', in_bad0='none', in_len='none', in1=('data', 'DATA1', DESC[:8]), setup0=False,
                 setup_crc=False, after=DESC)


def len_data(h):
    """A data packet whose CRC16 passes but whose data field is not whole bytes (8.3.4): the status stage of a
    control read, a zero-length DATA1 with the 23 bits of LEN_DATA after its CRC16. It must get no handshake (8.4.6.3,
    corrupted); the host sends the status again, clean, which is ACKed; then a whole transfer."""
    yield from start(h)
    r = {}
    r['setup'] = yield from h.setup(0, GET_DEV(8))
    kind, pk = yield from h.in_(0)
    r['in'] = (kind, pk.name if pk else None, pk.payload if pk else None)
    yield from h.frame_check()
    yield from h.send(G.token('OUT', 0, 0), h.gap_tr)
    t = yield from h.send(G.data('DATA1', []) + LEN_DATA, h.gap_data)
    pk = yield from h.answer(t)
    r['status_len'] = pk.name if pk is not None and pk.ok else None
    h.log.append(('out', 0, r['status_len']))
    r['status'] = yield from h.out(0, [], 'DATA1')
    r['after'] = yield from h.control_read(0, GET_DEV(18))
    yield from h.wait(0.1e6)
    return r


LEN_DATA_WANT = dict(setup=True, **{'in': ('data', 'DATA1', DESC[:8])}, status_len=None, status='ACK', after=DESC)


def conf(h):
    """GET_DESCRIPTOR(CONFIGURATION) in the Default state with wLength 9, what a host asks first, then 255, which
    gives all of wTotalLength (9.4.3); SET_ADDRESS; the same request at the new address with wLength 18; a
    SET_CONFIGURATION(1), which a host sends next (9.1.2 step 8); and the device descriptor after it."""
    yield from start(h)
    r = {}
    r['c9'] = yield from h.control_read(0, GET_CONF(9))
    r['c255'] = yield from h.control_read(0, GET_CONF(255))
    r['set'] = yield from h.control_nodata(0, SET_ADDR(3))
    yield from h.wait(2.2e6)
    r['c18'] = yield from h.control_read(3, GET_CONF(18))
    r['setconf'] = yield from h.control_nodata(3, SET_CONF(1))
    r['dev'] = yield from h.control_read(3, GET_DEV(18))
    yield from h.wait(0.1e6)
    return r


CONF_WANT = dict(c9=CONF[:9], c255=CONF, set=True, c18=CONF, setconf=True, dev=DESC)


def configure(h, addr=4):
    """An enumeration to the Configured state as a host does it (9.1.2): GET_DESCRIPTOR(DEVICE, 64) at address 0,
    SET_ADDRESS, the 2 ms recovery (9.2.6.3), the device descriptor, the configuration descriptor with wLength 9 and
    then wTotalLength, GET_CONFIGURATION (0: the Address state, 9.4.2), SET_CONFIGURATION(1), GET_CONFIGURATION (1);
    then SET_CONFIGURATION(0), back to the Address state (9.4.7), GET_CONFIGURATION (0), and values no
    configuration has, 2 and 255, each a request error (9.2.7), with GET_CONFIGURATION after them (0), and
    SET_CONFIGURATION(1) again."""
    yield from start(h)
    r = {}
    r['d1'] = yield from h.control_read(0, GET_DEV(64))
    r['set'] = yield from h.control_nodata(0, SET_ADDR(addr))
    yield from h.wait(2.2e6)
    r['d2'] = yield from h.control_read(addr, GET_DEV(18))
    r['c9'] = yield from h.control_read(addr, GET_CONF(9))
    r['c18'] = yield from h.control_read(addr, GET_CONF(18))
    r['cfg0'] = yield from h.control_read(addr, GET_CFG)
    r['setconf'] = yield from h.control_nodata(addr, SET_CONF(1))
    r['cfg1'] = yield from h.control_read(addr, GET_CFG)
    r['unconf'] = yield from h.control_nodata(addr, SET_CONF(0))
    r['cfg2'] = yield from h.control_read(addr, GET_CFG)
    r['bad2'] = yield from h.control_nodata(addr, SET_CONF(2))
    r['bad255'] = yield from h.control_nodata(addr, SET_CONF(255))
    r['cfg3'] = yield from h.control_read(addr, GET_CFG)
    r['again'] = yield from h.control_nodata(addr, SET_CONF(1))
    r['cfg4'] = yield from h.control_read(addr, GET_CFG)
    yield from h.wait(0.1e6)
    return r


CONFIGURE_WANT = dict(d1=DESC, set=True, d2=DESC, c9=CONF[:9], c18=CONF, cfg0=b'\x00', setconf=True, cfg1=b'\x01',
                      unconf=True, cfg2=b'\x00', bad2=False, bad255=False, cfg3=b'\x00', again=True, cfg4=b'\x01')
CONFIGURE_COUNT = dict(NSETUP=15, NREQ=13)     # the counters after it: 15 SETUPs ACKed, 13 transfers completed


def good_configure(r):
    return not {k for k in CONFIGURE_WANT if (r.value or {}).get(k) != CONFIGURE_WANT[k]} and \
        all(r.count[k] == n for k, n in CONFIGURE_COUNT.items())

# requests to the device, endpoint 0 and the interface, with what this device answers in the Address and the
# Configured state (9.4): bytes for a request to the host, True for one with no data stage that completes, None or
# False for a STALL (a request error, 9.2.7)
REQUESTS = (
    ('cfg', GET_CFG, b'\x00', b'\x01'),
    ('st_dev', GET_STATUS(0x80, 0), b'\x00\x00', b'\x00\x00'),        # bus powered, no remote wakeup (Figure 9-4)
    ('st_ep0out', GET_STATUS(0x82, 0x00), b'\x00\x00', b'\x00\x00'),  # endpoint 0, not halted (Figure 9-6)
    ('st_ep0in', GET_STATUS(0x82, 0x80), b'\x00\x00', b'\x00\x00'),
    ('st_ep1', GET_STATUS(0x82, 0x01), None, None),                     # no such endpoint
    ('st_ep81', GET_STATUS(0x82, 0x81), None, None),
    ('st_ep0x10', GET_STATUS(0x82, 0x10), None, None),                  # a reserved bit of the endpoint field
    ('st_if0', GET_STATUS(0x81, 0), None, b'\x00\x00'),                # interface 0: when configured (Figure 9-5)
    ('st_if1', GET_STATUS(0x81, 1), None, None),
    ('if0', GET_IF(0), None, b'\x00'),                                  # its alternate setting, 0 (9.4.4)
    ('if1', GET_IF(1), None, None),
    ('setif', SET_IF(0, 0), False, False),                              # the default setting alone: STALL (9.4.10)
    ('setif1', SET_IF(0, 1), False, False),
    ('wake', FEATURE(3, 0, 1, 0), False, False),                        # remote wakeup: not offered (Table 9-6)
    ('unwake', FEATURE(1, 0, 1, 0), False, False),
    ('halt0', FEATURE(3, 2, 0, 0x80), False, False),                    # endpoint 0's halt: not kept (9.4.5)
    ('iffeat', FEATURE(3, 1, 0, 0), False, False),                      # no interface feature
    ('c512', GET_CONF(0x200), CONF, CONF),                              # wLength 512: wTotalLength's 18 bytes
    ('string', [0x80, 6, 0, 3, 0, 0, 255, 0], None, None),              # no strings (9.6.7)
    ('conf1', [0x80, 6, 1, 2, 0, 0, 9, 0], None, None),                 # configuration index 1: none
    ('qual', [0x80, 6, 0, 6, 0, 0, 10, 0], None, None),                 # DEVICE_QUALIFIER: bcdUSB 1.10 (9.2.6.6)
    ('set3', SET_CONF(3), False, False))


def requests(h, addr=7):
    """Every request of REQUESTS in the Address state, then SET_CONFIGURATION(1) and every one again in the
    Configured state, then SET_CONFIGURATION(1) once more (the Configured state takes it again, 9.4.7)."""
    yield from start(h)
    r = {}
    r['set'] = yield from h.control_nodata(0, SET_ADDR(addr))
    yield from h.wait(2.2e6)
    for tag, phase in (('a_', 2), ('c_', 3)):
        if tag == 'c_':
            r['setconf'] = yield from h.control_nodata(addr, SET_CONF(1))
        for name, req, *_ in REQUESTS:
            if req[0] & 0x80:
                r[tag + name] = yield from h.control_read(addr, req)
            else:
                r[tag + name] = yield from h.control_nodata(addr, req)
    r['again'] = yield from h.control_nodata(addr, SET_CONF(1))
    r['cfg'] = yield from h.control_read(addr, GET_CFG)
    yield from h.wait(0.1e6)
    return r


REQUESTS_WANT = dict([('a_' + n, a) for n, _, a, _ in REQUESTS] + [('c_' + n, c) for n, _, _, c in REQUESTS] +
                     [('set', True), ('setconf', True), ('again', True), ('cfg', b'\x01')])


def states(h):
    """The Default state: SET_CONFIGURATION(1) at address 0, which 9.4.7 leaves open (this device takes it), then
    SET_ADDRESS, after which the device is in the Address state, not configured (9.4.6): GET_CONFIGURATION 0. Then
    configured, a reset (7.1.7.5) returns it to the Default state, where this device answers GET_CONFIGURATION with 0
    (9.4.2 leaves it open), and after SET_ADDRESS again GET_CONFIGURATION is 0 (9.4.2 asks it). A
    SET_CONFIGURATION(1) whose status packet the host ACKs only the second time is taken once (8.5.3.1); one that a
    new SETUP interrupts before its status stage (8.5.3) has taken effect all the same (9.2.6: the request is done
    before its status stage), which 9.4 doesn't judge. After SET_CONFIGURATION(0) the device is in the Address
    state again, where GET_STATUS(interface) and GET_INTERFACE are Request Errors (9.4.4, 9.4.5)."""
    yield from start(h)
    r = {}
    r['d_setconf'] = yield from h.control_nodata(0, SET_CONF(1))
    r['set'] = yield from h.control_nodata(0, SET_ADDR(8))
    yield from h.wait(2.2e6)
    r['cfg_a'] = yield from h.control_read(8, GET_CFG)
    r['setconf'] = yield from h.control_nodata(8, SET_CONF(1), lose_status=1)
    r['cfg_c'] = yield from h.control_read(8, GET_CFG)
    yield from h.reset()
    yield from h.wait(1.0e6)
    r['d_cfg0'] = yield from h.control_read(0, GET_CFG)                 # the Default state: 9.4.2 leaves it open
    r['set2'] = yield from h.control_nodata(0, SET_ADDR(9))
    yield from h.wait(2.2e6)
    r['cfg_r'] = yield from h.control_read(9, GET_CFG)
    r['cut'] = yield from h.setup(9, SET_CONF(1))                       # no status stage: a new SETUP follows
    r['cfg_cut'] = yield from h.control_read(9, GET_CFG)
    r['unconf'] = yield from h.control_nodata(9, SET_CONF(0))
    r['if_unconf'] = yield from h.control_read(9, GET_STATUS(0x81, 0))  # the Address state again: a Request Error
    r['getif_unconf'] = yield from h.control_read(9, GET_IF(0))          # for both (9.4.4, 9.4.5)
    yield from h.reset()
    yield from h.wait(1.0e6)
    r['d_cfg'] = yield from h.control_read(0, GET_CFG)                  # the Default state: 9.4.2 leaves it open
    yield from h.wait(0.1e6)
    return r


STATES_WANT = dict(d_setconf=True, set=True, cfg_a=b'\x00', setconf=True, cfg_c=b'\x01', d_cfg0=b'\x00', set2=True,
                   cfg_r=b'\x00', cut=True, cfg_cut=b'\x01', unconf=True, if_unconf=None, getif_unconf=None,
                   d_cfg=b'\x00')


def endpoints(h):
    """Tokens at the device's address for endpoints 1, 2, 4 and 8, one ENDP bit each (8.3.2.2): SETUP, IN and OUT,
    all unanswered, before and during a control read at address 0 and at a new address; the transfers then
    complete."""
    yield from start(h)
    r = {}
    for addr in (0, 10):
        for ep in (1, 2, 4, 8):
            r['setup%d_%d' % (addr, ep)] = yield from h.setup(addr, GET_DEV(18), endp=ep, tries=1)
        r['setup%d' % addr] = yield from h.setup(addr, GET_DEV(18))     # the data stage open: an IN gets data
        for ep in (1, 2, 4, 8):
            r['in%d_%d' % (addr, ep)] = (yield from h.in_(addr, endp=ep))[0]
            r['out%d_%d' % (addr, ep)] = yield from h.out(addr, [], 'DATA1', endp=ep)
        data = b''
        for _ in range(3):
            kind, pk = yield from h.in_(addr)
            data += pk.payload if kind == 'data' else b''
        r['data%d' % addr] = data
        r['status%d' % addr] = yield from h.out(addr, [], 'DATA1')
        if addr == 0:
            r['set'] = yield from h.control_nodata(0, SET_ADDR(10))
            yield from h.wait(2.2e6)
    yield from h.wait(0.1e6)
    return r


ENDPOINTS_WANT = dict([('setup%d_%d' % (a, e), False) for a in (0, 10) for e in (1, 2, 4, 8)] +
                      [('in%d_%d' % (a, e), 'none') for a in (0, 10) for e in (1, 2, 4, 8)] +
                      [('out%d_%d' % (a, e), None) for a in (0, 10) for e in (1, 2, 4, 8)] +
                      [('setup0', True), ('setup10', True), ('data0', DESC), ('data10', DESC), ('status0', 'ACK'),
                       ('status10', 'ACK'), ('set', True)])


REQ7 = [0x80, 6, 0, 1, 0, 0, 0x7F, 0]     # GET_DESCRIPTOR(DEVICE), wLength 127: seven 1s, then a 0
REQ8 = [0x80, 6, 0, 1, 0, 0, 0xFF, 0]     # wLength 255: eight 1s


def stuff_errors(h):
    """Bit-stuff errors from the host (7.1.9: seven 1s in a row is a bit-stuff error, and the packet is to be
    ignored, so no handshake answers it): a SETUP whose DATA0 packet leaves out its first stuff bit, in wLength
    0x7F, so seven 1s and then a 0 go out, and one in wLength 0xFF, eight 1s; after each, the same SETUP sent
    clean, and the descriptor read."""
    yield from start(h)
    r = {}
    r['err7'] = yield from h.setup(0, REQ7, nostuff=0, tries=1)
    r['clean7'] = yield from h.setup(0, REQ7)
    r['err8'] = yield from h.setup(0, REQ8, nostuff=0, tries=1)
    r['read8'] = yield from h.control_read(0, REQ8)
    yield from h.wait(0.1e6)
    return r


STUFF_WANT = dict(err7=False, clean7=True, err8=False, read8=DESC)


def flag_chip(*a, **k):
    """The chip with a probe on every unit's receive buffer: each word that arrives marked last, with its end flags
    (section 5.9.7: bit 1 a stuff violation, bit 2 the bit after it continued the run), in chip.ends. A probe only:
    it reads the simulator's state and changes nothing."""
    chip = Chip(*a, **k)
    chip.ends, full = [], [0] * 4
    base = chip.cycle

    def cycle(ui, uio):
        base(ui, uio)
        for t in chip.th:
            u = t.u
            if u.RXFULL and not full[t.TID] and u.RXBL:
                chip.ends.append((chip.c, t.TID, u.RXBE, u.RXBN))
            full[t.TID] = u.RXFULL
    chip.cycle = cycle
    return chip


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


def critical_path(text=None, host_kw=None):
    """On the simulator's execute log of a power-up enumeration (S on thread 0, A on 1): the cycles from S's time
    reference (its `now` after a packet's last word) to the `tick` that starts the answer, the worst case per kind
    of answer; and the cycles from A's store of a verdict (label vst) to S's load of it (vin for an IN, vtok for a
    SETUP or OUT), the least per kind of token."""
    img = assemble(text if text is not None else src(PROG), depth=512)
    L = img.labels
    ref = L['rend']
    kinds = {L['shs']: 'handshake', L['szlp']: 'zero-length', L['sdata']: 'data'}   # the senders' entries, which
    tick = L['stick']                                                               # share the tick that starts
    loads = {L['vin']: 'IN', L['vtok']: 'SETUP/OUT'}

    class Logged(Chip):
        def __init__(self, *a, **k):
            Chip.__init__(self, *a, **k)
            self.log = []
    r = run(quick, 12e6, chip_cls=Logged, check=False, host_kw=host_kw)
    worst, lead, at, st, kind = {}, {}, None, None, None
    for c, tid, pc, word in r.chip.log:
        if tid == 1 and pc == L['vst']:
            st = c
        if tid != 0:
            continue
        if pc in loads and st is not None:
            lead[loads[pc]] = min(lead.get(loads[pc], 1 << 30), c - st)
        if pc == ref:
            at, kind = c, None
        elif pc in kinds and at is not None and kind is None:
            kind = kinds[pc]                    # the first entry: the zero-length packet goes on through shs
        elif pc == tick and at is not None and kind is not None:
            worst[kind] = max(worst.get(kind, 0), c - at)
            at = None
    return worst, lead


# ---------------------------------------------------------------------------------------------------------------
# broken copies: each must fail somewhere

TEETH = [
    ('the toggle not flipped on the host\'s ACK', 'li    R4, WDATA0 ^ WDATA1', 'li    R4, 0'),
    ('the new address taken at the SETUP, before the status stage',
     'ldi   R7, PADDR      ; the address, taken when the host ACKs the status packet\nsset:',
     'cp    R6, R1\n        swap  R6\n        ori   R6, 0x69\n        ldi   R7, ADDRV\nsset:'),
    ('the answer 7 bit times after the EOP (NRSP 260)', '.equ NRSP 176', '.equ NRSP 260'),
    ('the host\'s data ACKed without its CRC-16 check', 'srd   R4, end\n        cmpi  R4, 1',
     'ldi   R4, 1\n        cmpi  R4, 1'),
    ('the CRC-5 verdict ignored on IN', 'xor   R2, R3\n        cmpi  R2, 1          ; A\'s verdict',
     'ldi   R2, 1\n        cmpi  R2, 1          ; A\'s verdict'),
    ('a fraction on the bit time (TFRAC 85): edges off the cycle grid', 'ldi   R0, 67\n        scfg  R0, TXTAIL',
     'li    R0, 0x5543\n        scfg  R0, TXTAIL'),
    ('an EOP of 50 cycles, 1.0 us', 'ldi   R0, 67\n        scfg  R0, TXTAIL', 'ldi   R0, 50\n        scfg  R0, TXTAIL'),
    ('J released 3 cycles after the EOP (NREL 0)', '.equ NREL 31', '.equ NREL 0'),
    ('resets not counted', 'st    R0, R4         ; a reset: S acts on it before the next packet', 'idle'),
    ('min(wLength, the answer\'s length) not applied', 'lt    R0, R4\n        jt    sg1', 'sett\n        jt    sg1'),
    ('the status packet sent as DATA0', 'lui   R0, WDATA1 >> 8    ; status packets are DATA1 (8.5.3)',
     'lui   R0, WDATA0 >> 8'),
    ('no stuffing on send (SMODE 0)', '.equ CODTX 0xE6EF', '.equ CODTX 0xE2EF'),
    ('IN: endpoint bit 3 unchecked', 'tsti  R0, 7          ; endpoint bits 3:1 clear',
     'tsti  R0, 3          ; endpoint bits 3:1 clear'),
    ('SETUP and OUT: endpoint bit 3 unchecked', 'tsti  R0, 7\n        jt    tbad', 'tsti  R0, 3\n        jt    tbad'),
    ('A: a token of any length taken (the bit count unchecked)', 'cmpi  R2, 8          ; 24 bits',
     'sett                 ; 24 bits'),
    ('GET_DESCRIPTOR(CONFIGURATION) answered with the device descriptor', 'ldi   R7, CONF', 'ldi   R7, DESC'),
    ('GET_DESCRIPTOR(CONFIGURATION) STALLed', 'cmpi  R2, 2          ; or CONFIGURATION',
     'cmpi  R2, 7          ; or CONFIGURATION'),
    ('S: a data packet taken whatever its length (the whole-byte test never fires)',
     'tsti  R2, 7          ; whole bytes', 'tsti  R2, 0          ; whole bytes'),
    # the configuration requests (9.4.2, 9.4.4, 9.4.5, 9.4.7) and the code reworked to make room for them
    ('SET_CONFIGURATION takes any value', 'sconf:  tsti  R1, 254', 'sconf:  tsti  R1, 0'),
    ('SET_CONFIGURATION completes but the value is not kept',
     'ldi   R7, CFGV       ; taken now: the request is done before its status stage (9.2.6)\n        goto  sset',
     'ldi   R0, 4\n        goto  snew'),
    ('GET_CONFIGURATION answers from the zero word', 'ldi   R7, CFGV\n        cmpi  R1, 8',
     'ldi   R7, ZERO\n        cmpi  R1, 8'),
    ('the interface answers in the Address state', 'tsti  R7, 1          ; interface 0 exists when configured',
     'sett                 ; interface 0 exists when configured'),
    ('GET_STATUS takes endpoint 1 for endpoint 0', 'tsti  R2, 0x7F       ; wIndex low: endpoint 0',
     'tsti  R2, 0x70       ; wIndex low: endpoint 0'),
    ('GET_STATUS answers from the configuration word: self powered when configured',
     'ldi   R4, 2\n        ldi   R7, ZERO', 'ldi   R4, 2\n        ldi   R7, CFGV'),
    ('SET_ADDRESS leaves the configuration as it was', 'st    R2, R7         ; state the standard leaves it open)',
     'idle                 ; state the standard leaves it open)'),
    ('a reset leaves the configuration as it was', 'stp   R0, R2         ; CFGV: not configured', 'idle'),
    ('wLength\'s high byte ignored', 'tsti  R2, 255        ; wLength 256 or more', 'tsti  R2, 0          ; wLength'),
    ('a handshake goes on into the data loop (T 0 at the shared tail)', 'sett\nsgo1:', 'clrt\nsgo1:'),
    ('a stalled transfer\'s OUT ACKed (STAGE 8 missed by the OUTW test)', 'tsti  R2, 12', 'tsti  R2, 4'),
    ('the status stage of a control read never completed by its OUT (STAGE 2 missed)',
     'tsti  R2, 3          ; the data stage (1) or the status stage (2)',
     'tsti  R2, 1          ; the data stage (1) or the status stage (2)'),
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

    # the configuration descriptor (9.4.3, 9.6.3, 9.6.5)
    r = run(conf, 20e6)
    note(r)
    v = verdict(r)
    diff = {k: (r.value or {}).get(k) for k in CONF_WANT if (r.value or {}).get(k) != CONF_WANT[k]}
    log.check('GET_DESCRIPTOR(CONFIGURATION): wLength 9 gives the 9-byte configuration descriptor, 255 and 18 give it '
              'with its interface descriptor, 18 bytes, at address 0 and after SET_ADDRESS; SET_CONFIGURATION(1) '
              'completes; every usb_check check passes', not v and not diff, v or diff or 'as wanted')

    # the requests of 9.4 in the Address and Configured states, judged by usb_check's request check
    def request_checks(r):
        c = r.rep.checks().get('request', (0, 0))
        info = [f.text for f in r.rep.findings if f.check == 'request' and f.ok is None]
        return c, info
    r = run(configure, 30e6)
    note(r)
    v = verdict(r)
    diff = {k: (r.value or {}).get(k) for k in CONFIGURE_WANT if (r.value or {}).get(k) != CONFIGURE_WANT[k]}
    c, info = request_checks(r)
    log.check('to the Configured state as a host goes there (9.1.2): the device and configuration descriptors, '
              'GET_CONFIGURATION 0, SET_CONFIGURATION(1), GET_CONFIGURATION 1; SET_CONFIGURATION(0) back to the Address '
              'state; the values 2 and 255 STALLed; every usb_check check passes, its request check on every '
              'transfer; the counters show 15 SETUPs and 13 completed transfers', not v and good_configure(r) and
              c == (15, 0) and not info and r.count['CFGV'] & 255 == 1,
              v or diff or 'request check %s, %s; counters %s' % (c, info, r.count))
    r = run(requests, 50e6)
    note(r)
    v = verdict(r)
    diff = {k: (r.value or {}).get(k) for k in REQUESTS_WANT if (r.value or {}).get(k) != REQUESTS_WANT[k]}
    c, info = request_checks(r)
    log.check('%d requests in the Address state and again in the Configured state: GET_CONFIGURATION, GET_STATUS of '
              'the device, endpoint 0 both ways, endpoints 1, 0x81 and a reserved bit, interfaces 0 and 1, '
              'GET_INTERFACE, SET_INTERFACE, SET_FEATURE and CLEAR_FEATURE, wLength 512, strings, configuration 1, '
              'the device qualifier, SET_CONFIGURATION(3); every usb_check check passes' % len(REQUESTS),
              not v and not diff and c == (41, 0) and len(info) == 7,
              v or diff or 'request check %s; open: %d' % (c, len(info)))
    log.info('request check on that run: %d judged right, %d left open by 9.4 or not judged: %s' % (
        c[0], len(info), '; '.join(sorted({x.split(' (')[1].rstrip(')') for x in info}))))
    r = run(states, 50e6)
    note(r)
    v = verdict(r)
    diff = {k: (r.value or {}).get(k) for k in STATES_WANT if (r.value or {}).get(k) != STATES_WANT[k]}
    log.check('states (9.1.1): SET_CONFIGURATION(1) at address 0 then SET_ADDRESS leaves the device not configured; '
              'a reset when configured returns it to the Default state; a lost status ACK; a SET_CONFIGURATION cut '
              'short by a new SETUP has taken effect (9.2.6); after SET_CONFIGURATION(0) the interface is a Request Error '
              'again (9.4.4, 9.4.5); every usb_check check passes', not v and not diff,
              v or diff or 'request check %s' % (request_checks(r)[0],))

    # a dribble bit before the host's EOP (7.1.9.1). With REOP the units' end test drops 1 to 6 ones after the last
    # whole byte (section 5.9.7, the byte end), so a dribbled packet reaches the firmware as a clean one
    r = run(dribble, 20e6)
    note(r)
    v = verdict(r)
    diff = {k: (r.value or {}).get(k) for k in DRIBBLE_WANT if (r.value or {}).get(k) != DRIBBLE_WANT[k]}
    log.check('one dribble bit on one packet at a time: a SETUP\'s data packet ACKed, an IN token answered with the '
              'next data packet, the host\'s ACK of it taken, the status OUT\'s token and data packet ACKed, '
              'SET_ADDRESS\'s status ACK taken (address 12); every usb_check check passes', not v and not diff and
              r.count['ADDRV'] == 12, v or diff or 'address %d' % r.count['ADDRV'])
    for what, script, t_end, want, kw in (
            ('enumeration with two 10 ms resets', enumerate_, 40e6, good_enum, {}),
            ('power-up enumeration, S on 1, A on 2, W on 3, alignment 1, phase 2.5 ns', quick, 12e6, good_quick,
             dict(threads=(1, 2, 3), align=1, phase=2.5)),
            ('power-up enumeration, S on 3, A on 0, W on 1, alignment 3, phase 7.5 ns', quick, 12e6, good_quick,
             dict(threads=(3, 0, 1), align=3, phase=7.5)),
            ('power-up enumeration, S on 2, A on 1, W on 0, alignment 2, 670 ns EOPs, 2-bit delays', quick, 12e6,
             good_quick, dict(threads=(2, 1, 0), align=2, phase=15.0,
                              host_kw=dict(eop=670.0 / BIT, gap_data=2.0, gap_hs=2.0, gap_tr=2.0))),
            ('faults', faults, 30e6, lambda x: not {k for k in FAULTS_WANT if (x.value or {}).get(k) !=
                                                     FAULTS_WANT[k]}, dict(threads=(0, 3, 2), align=1, phase=12.5)),
            ('lost ACKs', lost, 20e6, lambda x: (x.value or {}).get('d2') == DESC and x.count['ADDRV'] == 6,
             dict(threads=(3, 2, 1))),
            ('a SETUP and a reset in the middle of transfers', interrupted, 30e6,
             lambda x: not {k for k in INTERRUPTED_WANT if (x.value or {}).get(k) != INTERRUPTED_WANT[k]},
             dict(threads=(1, 0, 3), align=3, phase=17.5))):
        kw = dict(kw)
        kw['host_kw'] = dict(kw.get('host_kw', {}), dribble=1)
        r = run(script, t_end, **kw)
        note(r)
        v = verdict(r)
        log.check('every host packet with one dribble bit (tokens, data, handshakes): %s, the same results as clean, '
                  'every usb_check check passes' % what, not v and want(r), v or summary(r.rep))
    # 2 to 6 dribble bit times on every host packet: each packet is stretched by that many, or by fewer where its own
    # final 1s would make more than the six bit times with no transition that 7.1.9.1 asks a receiver to take
    for n in range(2, 7):
        r = run(quick, 12e6, host_kw=dict(dribble=n, dribble_fit=True), phase=2.5 * n, align=n % 4)
        note(r)
        v = verdict(r)
        log.check('power-up enumeration with %d dribble bit times on every host packet (fewer where its final 1s '
                  'leave less room, 7.1.9.1): the same results as clean, every usb_check check passes' % n,
                  not v and good_quick(r), v or summary(r.rep))
    cp, ld = critical_path(host_kw=dict(dribble=1))
    log.info('margins, every host packet dribbled: from the time reference to the start, worst %s cycles of %d: %s; '
             'A\'s store of its verdict to S\'s load of it, at least %s cycles for an IN and %s for a SETUP or OUT' % (
                 max(cp.values()) if cp else '?', 176, ', '.join('%s %d' % kv for kv in sorted(cp.items())),
                 ld.get('IN', '?'), ld.get('SETUP/OUT', '?')))
    log.check('every host packet dribbled: A stores each verdict before S loads it (section 4.2) and every answer '
              'starts on time', ld and min(ld.values()) >= 1 and cp and max(cp.values()) < 176 - 8,
              'lead %s, worst path %s cycles' % (ld, max(cp.values()) if cp else None))

    # bits after the last whole byte that are not a dribble bit, and tokens to other endpoints
    r = run(near_misses, 16e6)
    note(r)
    v = verdict(r)
    diff = {k: (r.value or {}).get(k) for k in NEAR_WANT if (r.value or {}).get(k) != NEAR_WANT[k]}
    log.check('bits that are not a dribble bit: a 0 after an IN token and after a SETUP\'s data packet, too many '
              'bits that reach the dribble value of the CRC5 and CRC16 registers, and a 0 that takes an IN token with a '
              'wrong CRC5 there: no answer (8.7.1); every usb_check check passes', not v and not diff, v or diff or summary(r.rep))
    r = run(len_data, 16e6)
    note(r)
    v = verdict(r)
    diff = {k: (r.value or {}).get(k) for k in LEN_DATA_WANT if (r.value or {}).get(k) != LEN_DATA_WANT[k]}
    log.check('a data packet whose CRC16 passes but whose data field is not whole bytes (8.3.4), 23 bits past its '
              'CRC16: no handshake (8.4.6.3); the clean status after it is ACKed; every usb_check check passes',
              not v and not diff, v or diff or summary(r.rep))
    r = run(endpoints, 24e6)
    note(r)
    v = verdict(r)
    diff = {k: (r.value or {}).get(k) for k in ENDPOINTS_WANT if (r.value or {}).get(k) != ENDPOINTS_WANT[k]}
    log.check('SETUP, IN and OUT tokens to endpoints 1, 2, 4 and 8 (each ENDP bit, 8.3.2.2) at address 0 and 10, idle '
              'and inside a data stage: none answered, and the transfers complete', not v and not diff and
              r.count['ADDRV'] == 10, v or diff or summary(r.rep))

    # margins
    rate, nxt, par, se0, off, jd = device_stats(reps)
    log.check('response time over %d answers in these runs: %s bit times after the host\'s SE0-to-J, inside 2 to '
              '6.5 (7.1.18.1, TRSPIPD1)' % (len(resp), span(resp)), resp and 2.0 <= min(resp) and max(resp) <= 6.5)
    log.info('margins, device packets: bit rate %s Mb/s (TLDRATE 1.4775 to 1.5225); jitter against the packet\'s own '
             'period %s ns next and %s paired (25 and 10, 7.1.13.1.1); SE0 %s ns (TLEOPT 1250 to 1500), %s ns from the '
             'bit grid (TLDEOP -40 to +100); J driven %s ns (one bit, %.0f to %.0f)' % (
                 span(rate, '%.4f'), span(nxt, '%.1f'), span(par, '%.1f'), span(se0, '%.0f'), span(off, '%+.0f'),
                 span(jd, '%.0f'), 1e3 / 1.5152 - 25, 2e3 / 1.5152))
    cp, _ = critical_path()
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

    # bit-stuff errors from the host (7.1.9): the destuffer's violation, end flags 1 and 2 (section 5.9.7)
    r = run(stuff_errors, 16e6, chip_cls=flag_chip)
    d = r.value or {}
    flags = [e for e in r.chip.ends if e[2] & 6]
    host_errs = sorted({c for _, c, _ in r.rep.failed()})
    fl = r.rep.failed()
    log.check('bit-stuff errors: a SETUP whose data packet carries seven 1s and then a 0, and one with eight 1s, go '
              'unanswered; the same SETUPs sent clean are ACKed and the descriptor read; threads healthy; usb_check, '
              'which reads the host\'s broken packets as good, finds only their missing ACKs',
              d == STUFF_WANT and not r.health and len(fl) == 2 and all(
                  c == 'sequence' and 'no answer' in x for _, c, x in fl),
              '%s %s; usb_check: %s' % ({k: v for k, v in d.items() if k != 'read8'}, r.health,
                                        [x[:60] for _, _, x in fl[:3]]))
    log.check('bit-stuff errors: the broken packets end their receptions with end flag 1, and with end flag 2 too '
              'where the eighth bit continues the run (section 5.9.7), on both units that hunt on D+; no clean '
              'packet sets either', sorted({(c, f & 6) for c, _, f, _ in flags}, key=lambda x: x[0]) and
              [f for _, f in sorted({(c, f & 6) for c, _, f, _ in flags})] == [2, 6],
              'flagged ends (cycle, thread, flags, bits): %s; usb_check findings %s' % (flags, host_errs))

    # broken copies
    def same(want):
        return lambda x: not {k for k in want if (x.value or {}).get(k) != want[k]}
    scripts = (('quick', quick, 12e6, {}, good_quick), ('quick, dribbled', quick, 12e6, dict(host_kw=dict(dribble=1)),
                                                        good_quick),
               ('dribble', dribble, 20e6, {}, same(DRIBBLE_WANT)), ('near_misses', near_misses, 16e6, {},
                                                                   same(NEAR_WANT)),
               ('len_data', len_data, 16e6, {}, same(LEN_DATA_WANT)),
               ('endpoints', endpoints, 24e6, {}, same(ENDPOINTS_WANT)), ('faults', faults, 30e6, {},
                                                                         same(FAULTS_WANT)),
               ('lost', lost, 20e6, {}, lambda x: (x.value or {}).get('d2') == DESC),
               ('interrupted', interrupted, 30e6, {}, same(INTERRUPTED_WANT)),
               ('conf', conf, 20e6, {}, same(CONF_WANT)), ('configure', configure, 30e6, {}, good_configure),
               ('requests', requests, 50e6, {}, same(REQUESTS_WANT)), ('states', states, 50e6, {}, same(STATES_WANT)),
               ('enumerate_', enumerate_, 40e6, {}, good_enum))
    for what, old, new in TEETH:
        text = broken(old, new)
        bad = []
        for name, script, t_end, kw, want in scripts:
            r = run(script, t_end, text=text, **kw)
            v = verdict(r)
            if v or not want(r):
                bad.append('%s: %s' % (name, v[:120] or 'wrong result'))
                break
        log.check('a broken copy is caught: %s' % what, bool(bad), bad[0] if bad else 'every check passed')
