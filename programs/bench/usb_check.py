"""USB low-speed line and protocol checker, written from USB 2.0 before any firmware.

Source: Universal Serial Bus Specification Revision 2.0 (2000-04-27): chapter 7 (signalling, NRZI, stuffing,
SYNC, EOP, jitter, inter-packet delay, reset, keep-alive, suspend), chapter 8 (PIDs, CRCs, packets,
transactions, control transfers, data toggle) and chapter 9 (requests, SetAddress, the device descriptor).
Every limit names its clause or table. Python 3 standard library only.

It checks what sigrok's usb_signalling, usb_packet and usb_request decoders don't: bit rate and its tolerance,
SYNC, stuffing and EOP width, inter-packet delay and the device's response time, keep-alive and reset, and the
sequence of the control transfers with toggles, NAK and retry, and the address change after the status stage.
sigrok_requests() runs sigrok's usb_request on the same record as a second opinion when sigrok-cli is installed.

The input is the bus at the device's pins, as programs/bench/frontend.UsbLs records it: a dict with
  dp, dm   lists of (time_ns, level) changes of D+ and D-, level 0 or 1; before the first change D+ is 0 and D-
           is 1, the idle J of a low-speed device (USB Table 7-2)
  dev      optional (time_ns, 0 or 1) changes of "the device drives the bus"; 0 before the first. With it each
           packet is assigned to its sender by who drives at its start; without it, by its place in the
           transaction
  host     optional, the same for the host
  contention   optional list of (start, end) when both ends drive one line differently
  t_end    optional end of the record, in ns

    import usb_check
    rep = usb_check.check(rec, descriptor=bytes(18), config=bytes(18))
    rep.ok, rep.failed(), rep.checks(), rep.packets, rep.transfers

descriptor is the device descriptor the device must return; config, its configuration descriptor with the
interface and endpoint descriptors after it, as GET_DESCRIPTOR(CONFIGURATION) returns them (9.4.3).

Checks:
  bit_rate       device packets at 1.4775 to 1.5225 Mb/s (TLDRATE, 7.1.11, Table 7-10); host packets against
                 TLDRATHS 1.49925 to 1.50075 Mb/s, as information unless strict_host
  jitter         device packets: consecutive transitions within +/-25 ns and paired ones within +/-10 ns of
                 N periods of the packet's own rate (7.1.13.1.1), and within TUDJ1 +/-95 ns and TUDJ2 +/-150 ns
                 of N nominal bit times (Table 7-10)
  sync           SYNC of 3 KJ pairs and 2 K's, 8 symbols (7.1.10)
  stuffing       a 0 after every six 1s, from the SYNC's last bit, including before the EOP (7.1.9, 7.1.9.1)
  pid            check nibble is the complement of the type (8.3.1); device PIDs only DATA0, DATA1, ACK, NAK,
                 STALL (Table 8-1, 8.6.5)
  crc            token CRC5 residual 01100, data CRC16 residual 1000000000001101 (8.3.5)
  packet_format  whole bytes; token 3 bytes, handshake 1, data with CRC16 and at most 8 payload bytes at low
                 speed (8.4.1, 8.4.4, 8.4.5); a device never sends a token
  eop            device EOP: SE0 for 1.25 to 1.50 us (TLEOPT), starting -40 to +100 ns from the bit grid
                 (TLDEOP, Table 7-10), then J, driven for one bit before the device lets go (7.1.7.4.1)
  ipd            at least 2 bit times from SE0-to-J to the next SOP (7.1.18.1): device packets fail, host packets
                 inform unless strict_host. Delays count nominal low-speed bit times, 666.67 ns (7.1.11), as the
                 host and hubs count them
  response_time  a device response at most 6.5 bit times after the host's SE0-to-J (TRSPIPD1, 7.1.18.1; the 7.5
                 of a captive cable is reported too)
  keepalive      the bus is never idle for more than 3.0 ms between the first and last event (7.1.7.6: the device
                 would suspend); keep-alives counted
  reset          the device is silent during an SE0 longer than TDETRST (2.5 us) and, after a reset (SE0 of 10 ms
                 or more, TDRST), answers address 0 and not its old address (7.1.7.5, 9.1.1.3). An SE0 between the
                 two may be taken as a reset or not (7.1.7.5): both readings are tried
  sequence       transactions: a SETUP with good data is always ACKed, never NAKed or STALLed; no answer to a
                 corrupted token or data packet, to another address or endpoint, or to nothing; a host packet with
                 dribble before its EOP (all 1s, at most six bit times with no transition) is good (7.1.9.1); IN in a
                 data stage gets DATA, NAK or STALL; status stages; a protocol STALL lasts to the next SETUP; data
                 stage packets full except the last, never past wLength; request times (8.4.6, 8.5.3, 9.2.6.4)
  toggle         the device's first data-stage packet is DATA1 and its toggle flips only on the host's ACK; its
                 status packet is DATA1 (8.5.3, 8.6)
  retry          a data packet sent again after a lost ACK is the same length and content (8.6.4)
  address        after SetAddress the device answers the old address until it sees the ACK of its status packet,
                 keeps answering it if that ACK is lost, and after the 2 ms recovery answers the new address and
                 not the old (9.4.6, 9.2.6.3, 8.5.3.1)
  descriptor     GET_DESCRIPTOR(DEVICE) returns min(wLength, 18) bytes of a device descriptor: bLength 18,
                 bDescriptorType 1, bMaxPacketSize0 8, BCD bcdUSB, bNumConfigurations at least 1, or exactly the
                 descriptor given (9.4.3, 9.6.1, 5.5.3)
  request        each standard request against 9.4 in the device state of its SETUP (9.1.1: Default at address 0,
                 Address, Configured): whether it must complete, must be a Request Error (a STALL in its data or
                 status stage, 9.2.7), or is left open by the standard, and what it returns. GET_DESCRIPTOR(DEVICE)
                 and SET_ADDRESS(1 to 127) complete in the Default and Address states (9.4.3, 9.4.6);
                 GET_DESCRIPTOR(CONFIGURATION) returns min(wLength, wTotalLength) bytes of the configuration given
                 (9.4.3) and an index past bNumConfigurations is a Request Error; SET_CONFIGURATION takes 0 or a
                 bConfigurationValue and refuses any other value (9.4.7); GET_CONFIGURATION returns 0 in the Address
                 state and the value when configured (9.4.2); GET_STATUS of the device returns two bytes with the
                 reserved bits 0, remote wakeup as last set, self powered 0 for a bus-powered configuration; of
                 endpoint 0 the halt bit as last set; of another endpoint or of an interface, a Request Error in the
                 Address state, and when configured two bytes for one the configuration has, a Request Error
                 otherwise (9.4.5, Figures 9-4 to 9-6); GET_INTERFACE and SET_INTERFACE are Request Errors in the
                 Address state, and when configured return or select an alternate setting the configuration has;
                 SET_INTERFACE may be STALLed for an interface with the default setting alone (9.4.4, 9.4.10);
                 CLEAR_FEATURE and SET_FEATURE of remote wakeup are Request Errors when bmAttributes doesn't offer
                 it, of an interface always (Table 9-6), of endpoint 0's halt either way (9.4.5). The Default
                 state leaves most of these open, and so do values the requests don't define: those are INFO. A
                 reset returns the device to the Default state, not configured; SET_ADDRESS from the Default or
                 Address state leaves it not configured (9.4.6). Needs config= for the rules that depend on the
                 configuration. This check came later than the rest of the module: it was written from 9.4 after
                 a first draft of these requests in advanced/usb_enum.s, none of which ran before it
  line_state     no SE1 longer than TLST (210 ns); no contention; SE0 shorter than TLST between J and K is a
                 transition, not an EOP (Table 7-2, 7.1.4.1)

Each check judges one thing: integrity checks (sync, stuffing, pid, crc, packet_format) judge a device packet's
form, and the protocol checks (sequence, toggle, retry, address, descriptor) judge it by its PID's type nibble and
payload whatever its form. A host packet's integrity decides only whether the device should answer it.
"""
import os
import re
import shutil
import subprocess

BIT = 1e9 / 1.5e6               # ns, low speed (7.1.11)

DEFAULTS = dict(
    bit=BIT,
    dev_rate=(1.4775e6, 1.5225e6),      # TLDRATE (Table 7-10)
    host_rate=(1.49925e6, 1.50075e6),   # TLDRATHS (Table 7-10)
    jit_next=25.0, jit_paired=10.0,     # 7.1.13.1.1, against the packet's own period
    tudj1=95.0, tudj2=150.0,            # Table 7-10, device, against the nominal bit time
    tddj1=25.0, tddj2=14.0,             # Table 7-10, host
    eop_t=(1250.0, 1500.0),             # TLEOPT
    eop_r=670.0,                        # TLEOPR: a receiver accepts this short an SE0
    tlst=210.0,                         # TLST: SE0 during a transition
    tldeop=(-40.0, 100.0),              # TLDEOP
    ipd=2.0,                            # bit times (7.1.18.1)
    rsp=6.5, rsp_captive=7.5,           # TRSPIPD1, TRSPIPD2
    timeout=16.0,                       # bit times: a host waits at least this long (7.1.19.1)
    tdetrst=2500.0,                     # TDETRST minimum
    tdrst=10e6,                         # TDRST minimum
    suspend=3e6,                        # 7.1.7.6
    recovery=2e6,                       # SetAddress recovery (9.2.6.3)
    max_packet=8,                       # low-speed EP0 (5.5.3, 8.4.4)
    strict_host=False,
)

PIDS = {1: 'OUT', 9: 'IN', 5: 'SOF', 13: 'SETUP', 3: 'DATA0', 11: 'DATA1', 7: 'DATA2', 15: 'MDATA', 2: 'ACK',
        10: 'NAK', 14: 'STALL', 6: 'NYET', 12: 'PRE', 8: 'SPLIT', 4: 'PING', 0: 'reserved'}   # Table 8-1
TOKENS = ('OUT', 'IN', 'SOF', 'SETUP')
DATA = ('DATA0', 'DATA1')
HANDSHAKES = ('ACK', 'NAK', 'STALL')
DEV_PIDS = ('DATA0', 'DATA1', 'ACK', 'NAK', 'STALL')


# ---------------------------------------------------------------------------------------------------------------
# results (the same shape as eth_check's)

class Finding:
    __slots__ = ('t', 'check', 'ok', 'text')

    def __init__(self, t, check, ok, text):
        self.t, self.check, self.ok, self.text = t, check, ok, text

    def __repr__(self):
        tag = 'INFO' if self.ok is None else 'PASS' if self.ok else 'FAIL'
        return '%s %s @%.3f us: %s' % (tag, self.check, self.t / 1000.0, self.text)


class Report:
    CHECKS = ('bit_rate', 'jitter', 'sync', 'stuffing', 'pid', 'crc', 'packet_format', 'eop', 'ipd',
              'response_time', 'keepalive', 'reset', 'sequence', 'toggle', 'retry', 'address', 'descriptor',
              'request', 'line_state')

    def __init__(self):
        self.findings = []
        self.packets, self.transfers, self.keepalives, self.resets = [], [], [], []

    def add(self, t, check, ok, text):
        self.findings.append(Finding(t, check, ok, text))

    @property
    def ok(self):
        return all(f.ok is not False for f in self.findings)

    def failed(self, check=None):
        return [(f.t, f.check, f.text) for f in self.findings
                if f.ok is False and (check is None or f.check == check)]

    def checks(self):
        out = {c: [0, 0] for c in self.CHECKS}
        for f in self.findings:
            if f.ok is not None:
                out.setdefault(f.check, [0, 0])[0 if f.ok else 1] += 1
        return {c: tuple(v) for c, v in out.items()}

    def failing_checks(self):
        return sorted({f.check for f in self.findings if f.ok is False})

    def summary(self):
        lines = ['%-14s %4d pass %4d fail' % (c, p, n) for c, (p, n) in sorted(self.checks().items())]
        lines += [repr(f) for f in self.findings if f.ok is False]
        return '\n'.join(lines)


class Packet:
    def __init__(self):
        self.sop = self.se0 = self.end = 0.0     # J-to-K; start of the EOP's SE0; its SE0-to-J
        self.trans = []                          # (time, new state) of each differential transition
        self.raw = []                            # NRZI-decoded bits, SYNC included, stuff bits included
        self.period = None
        self.sync_len = 0
        self.stuff_errors = []                   # raw bit index of each 7th one
        self.final_stuff_missing = False
        self.bits = []                           # after the SYNC, unstuffed
        self.octets = b''
        self.pid = None
        self.name = None
        self.sender = None
        self.eop_state = None                    # the state after the SE0
        self.addr = self.endp = None
        self.crc_ok = None
        self.pid_ok = False
        self.extra = 0
        self.cells = []
        self.n_raw = 0
        self.eop_kind = None
        self.j_drive = None

    @property
    def type(self):
        return self.pid & 15 if self.pid is not None else None

    @property
    def payload(self):
        return self.octets[1:-2] if self.name in DATA and len(self.octets) >= 3 else b''

    def __repr__(self):
        return '%s %s %.3f us %s' % (self.sender, self.name, self.sop / 1e3, self.octets.hex())


# ---------------------------------------------------------------------------------------------------------------
# line states

def _states(dp, dm, tlst):
    """[(t, state)] from the two lines, states J K SE0 SE1 (Table 7-2, low speed: J is D- high). An SE0 or SE1
    no longer than TLST between J and K is a transition, put at its middle (7.1.4.1). Also returns the
    transient SE1s and SE0s so dropped."""
    ev = sorted([(t, 0, v) for t, v in dp] + [(t, 1, v) for t, v in dm])
    lv = [0, 1]
    raw = [(float('-inf'), 'J')]
    names = {(0, 1): 'J', (1, 0): 'K', (0, 0): 'SE0', (1, 1): 'SE1'}
    for t, i, v in ev:
        lv[i] = v
        s = names[tuple(lv)]
        if raw[-1][0] == t:
            raw[-1] = (t, s)
            if len(raw) > 1 and raw[-2][1] == s:
                raw.pop()
        elif raw[-1][1] != s:
            raw.append((t, s))
    out, short = [], []
    k = 0
    while k < len(raw):
        t, s = raw[k]
        if s in ('SE0', 'SE1') and 0 < k < len(raw) - 1:
            t_next = raw[k + 1][0]
            before, after = raw[k - 1][1], raw[k + 1][1]
            if t_next - t <= tlst and before in ('J', 'K') and after in ('J', 'K') and before != after:
                short.append((t, s, t_next - t))
                out.append(((t + t_next) / 2, after))
                k += 2
                continue
        if out and out[-1][1] == s:
            k += 1
            continue
        out.append((t, s))
        k += 1
    return out, short


# ---------------------------------------------------------------------------------------------------------------
# packets

def crc5_ok(bits):
    """8.3.5 and 8.3.5.1: seeded with ones; for each bit the remainder's top bit XOR the bit decides whether to
    XOR G after the shift; over the 11 token bits and the 5 CRC bits the remainder is 01100."""
    reg = 0b11111
    for b in bits:
        f = ((reg >> 4) & 1) ^ b
        reg = (reg << 1) & 0b11111
        if f:
            reg ^= 0b00101
    return reg == 0b01100


def crc16_ok(bits):
    """8.3.5.2: G = x^16 + x^15 + x^2 + 1 (1000000000000101B), residual 1000000000001101B."""
    reg = 0xFFFF
    for b in bits:
        f = ((reg >> 15) & 1) ^ b
        reg = (reg << 1) & 0xFFFF
        if f:
            reg ^= 0x8005
    return reg == 0x800D


def _decode(pk, T0):
    """NRZI bits from the differential transitions (7.1.8: a 0 is a change), the period refined by a least
    squares fit of transition times against bit numbers, then the SYNC and the destuffing (7.1.9)."""
    times = [t for t, _ in pk.trans]
    T = T0
    cells = []
    for _ in range(3):
        cells = [0]
        for k in range(1, len(times)):
            cells.append(cells[-1] + max(1, round((times[k] - times[k - 1]) / T)))
        if len(times) >= 2 and cells[-1] > 0:
            n = len(times)
            sx, sy = sum(cells), sum(times)
            sxx = sum(c * c for c in cells)
            sxy = sum(c * t for c, t in zip(cells, times))
            den = n * sxx - sx * sx
            if den:
                T = (n * sxy - sx * sy) / den
    pk.period = T
    pk.cells = cells
    last = max(1, round((pk.se0 - times[-1]) / T)) if times else 0
    raw = []
    for k in range(len(times)):
        n = (cells[k + 1] - cells[k]) if k + 1 < len(times) else last
        raw += [0] + [1] * (n - 1)
    pk.raw = raw
    pk.n_raw = (cells[-1] if cells else 0) + last
    # SYNC: zeros up to the first 1 (8.2: its last two bits, KK, mark the end)
    try:
        first_one = raw.index(1)
    except ValueError:
        pk.sync_len = len(raw)
        return
    pk.sync_len = first_one + 1
    run, bits = 1, []
    k = first_one + 1
    while k < len(raw):
        b = raw[k]
        if run == 6:
            if b == 1:
                pk.stuff_errors.append(k)
                bits.append(1)
                run = 7
            else:
                run = 0
            k += 1
            continue
        bits.append(b)
        run = run + 1 if b else 0
        k += 1
    if run == 6:
        pk.final_stuff_missing = True
    pk.bits = bits
    whole = len(bits) // 8
    pk.extra = len(bits) - 8 * whole
    pk.octets = bytes(sum(bits[8 * q + b] << b for b in range(8)) for q in range(whole))
    if pk.octets:
        pk.pid = pk.octets[0]
        pk.name = PIDS[pk.pid & 15]
        pk.pid_ok = (pk.pid >> 4) == ((~pk.pid) & 15)
        body = bits[8:8 * whole]
        if pk.name in TOKENS and len(pk.octets) == 3:
            pk.addr = pk.octets[1] & 0x7F
            pk.endp = (pk.octets[1] >> 7) | ((pk.octets[2] & 7) << 1)
            pk.crc_ok = crc5_ok(body)
        elif pk.name in DATA and len(pk.octets) >= 3:
            pk.crc_ok = crc16_ok(body)


def _dribble_ok(pk):
    """7.1.9.1: a receiver must accept a packet with up to six full bit times with no transition before its EOP
    (dribble, from hub switching skews). Bits after the last whole byte are dribble, and the packet is taken without
    them, when they are all 1s (no transition) and the run of raw 1s that ends the packet is at most 6 long."""
    if not pk.extra:
        return True
    if any(b == 0 for b in pk.bits[-pk.extra:]):
        return False
    run = 0
    for b in reversed(pk.raw):
        if b != 1:
            break
        run += 1
    return run <= 6 and not pk.stuff_errors


def packets(states, o):
    """Splits the state timeline into packets, keep-alives and resets."""
    T0 = o['bit']
    pks, kas, rsts, odd = [], [], [], []
    k = 0
    n = len(states)
    while k < n:
        t, s = states[k]
        prev = states[k - 1][1] if k else 'J'
        if s == 'K' and prev == 'J':
            pk = Packet()
            pk.sop = t
            pk.trans.append((t, 'K'))
            k += 1
            while k < n and states[k][1] in ('J', 'K'):
                pk.trans.append(states[k])
                k += 1
            if k >= n:
                pk.se0 = pk.end = float('inf')
                odd.append((t, 'a packet with no EOP'))
                break
            pk.se0 = states[k][0]
            pk.eop_kind = states[k][1]
            if k + 1 < n:
                pk.end = states[k + 1][0]
                pk.eop_state = states[k + 1][1]
            else:
                pk.end = float('inf')
            _decode(pk, T0)
            pks.append(pk)
            k += 1
            continue
        if s == 'SE0' and prev == 'J':
            t_end = states[k + 1][0] if k + 1 < n else float('inf')
            nxt = states[k + 1][1] if k + 1 < n else None
            if t_end - t >= o['tdetrst']:
                rsts.append((t, t_end))
            else:
                kas.append((t, t_end, nxt))
        k += 1
    return pks, kas, rsts, odd


def _level_at(changes, t, initial=0):
    v = initial
    for tc, lv in changes:
        if tc > t:
            break
        v = lv
    return v


# ---------------------------------------------------------------------------------------------------------------
# the checker

def check(rec, descriptor=None, config=None, **opts):
    """Checks a record (a dict as described in the module's docstring). descriptor, if given, is the device
    descriptor the device must return (18 bytes); config, its configuration descriptor with the interface and
    endpoint descriptors that follow it (9.4.3). opts override DEFAULTS."""
    o = dict(DEFAULTS)
    o.update(opts)
    rep = Report()
    rep.expected = bytes(descriptor) if descriptor is not None else None
    rep.config = bytes(config) if config is not None else None
    rep.descriptor = None
    states, short = _states(rec['dp'], rec['dm'], o['tlst'])
    pks, kas, rsts, odd = packets(states, o)
    rep.packets, rep.keepalives, rep.resets = pks, kas, rsts
    for t, text in odd:
        rep.add(t, 'line_state', False, text)
    _line_state(rep, states, short, rec, o)
    _assign(rep, pks, rec, o)
    _j_drive(pks, rec)
    for i, pk in enumerate(pks):
        _packet_checks(rep, pk, pks[i - 1] if i else None, o)
    _bus_events(rep, pks, kas, rsts, rec, o)
    _protocol(rep, pks, rsts, o)
    rep.findings.sort(key=lambda f: f.t)
    return rep


def _line_state(rep, states, short, rec, o):
    bad = False
    for k, (t, s) in enumerate(states):
        if s == 'SE1':
            t_next = states[k + 1][0] if k + 1 < len(states) else rec.get('t_end', t)
            rep.add(t, 'line_state', False, 'SE1 for %.0f ns (Table 7-2: not a bus state)' % (t_next - t))
            bad = True
    for a, b in rec.get('contention', []):
        rep.add(a, 'line_state', False, 'both ends drive one line differently for %.0f ns' % (b - a))
        bad = True
    for t, s, w in short:
        rep.add(t, 'line_state', None, '%s of %.0f ns between J and K taken as a transition (TLST %.0f ns)'
                % (s, w, o['tlst']))
    if not bad:
        rep.add(0.0, 'line_state', True, 'no SE1 and no contention in %d state changes' % len(states))


def _assign(rep, pks, rec, o):
    """Who sent each packet: who drives at its SOP when the record says; else by the packet before it: a host IN
    or host data packet is answered by the device, anything else comes from the host."""
    dev = rec.get('dev')
    prev = None
    for pk in pks:
        if dev is not None:
            pk.sender = 'dev' if _level_at(dev, pk.sop + 1e-6) else 'host'
        elif pk.name in TOKENS or pk.name in ('PRE', 'SPLIT', 'PING'):
            pk.sender = 'host'
        elif prev is not None and prev.sender == 'host' and (prev.name == 'IN' or prev.name in DATA):
            pk.sender = 'dev'
        else:
            pk.sender = 'host'
        prev = pk


def _j_drive(pks, rec):
    """How long the device drives J after its EOP's SE0 (7.1.7.4.1: one bit time, then high impedance)."""
    dev = rec.get('dev')
    if dev is None:
        return
    for pk in pks:
        if pk.sender == 'dev' and pk.end != float('inf'):
            off = [t for t, v in dev if t >= pk.end - 1e-6 and v == 0]
            if off:
                pk.j_drive = off[0] - pk.end


def _packet_checks(rep, pk, prev, o):
    T0 = o['bit']
    dev = pk.sender == 'dev'
    strict = dev or o['strict_host']
    who = 'device' if dev else 'host'
    t = pk.sop
    if pk.period is None or len(pk.trans) < 2:
        rep.add(t, 'packet_format', False if dev else None, '%s packet with %d transitions' % (who, len(pk.trans)))
        return
    # bit_rate
    rate = 1e9 / pk.period
    lo, hi = o['dev_rate'] if dev else o['host_rate']
    ok = lo <= rate <= hi
    rep.add(t, 'bit_rate', ok if strict else (None if not ok else True),
            '%s %s at %.4f Mb/s (%.4f to %.4f, %s)' % (who, pk.name, rate / 1e6, lo / 1e6, hi / 1e6,
                                                     'TLDRATE' if dev else 'TLDRATHS'))
    # jitter: consecutive and paired transitions (7.1.13.1.1, Table 7-10)
    times = [x for x, _ in pk.trans]
    cells = pk.cells
    worst = [0.0, 0.0, 0.0, 0.0]
    for k in range(1, len(times)):
        for gap, idx in ((1, 0), (2, 1)):
            if k - gap < 0:
                continue
            nb = cells[k] - cells[k - gap]
            dt = times[k] - times[k - gap]
            worst[idx] = max(worst[idx], abs(dt - nb * pk.period))
            worst[idx + 2] = max(worst[idx + 2], abs(dt - nb * T0))
    if dev:
        ok = worst[0] <= o['jit_next'] + 1e-6 and worst[1] <= o['jit_paired'] + 1e-6 and \
            worst[2] <= o['tudj1'] + 1e-6 and worst[3] <= o['tudj2'] + 1e-6
        rep.add(t, 'jitter', ok, 'device %s: next %.1f, paired %.1f ns against the packet\'s period (25, 10); '
                'next %.1f, paired %.1f ns against 666.67 ns (TUDJ1 95, TUDJ2 150)' % (pk.name, *worst))
    else:
        ok = worst[2] <= o['tddj1'] + 1e-6 and worst[3] <= o['tddj2'] + 1e-6
        rep.add(t, 'jitter', ok if o['strict_host'] else (None if not ok else True),
                'host %s: next %.1f, paired %.1f ns against 666.67 ns (TDDJ1 25, TDDJ2 14)' % (pk.name, worst[2],
                                                                                              worst[3]))
    # sync
    ok = pk.sync_len == 8 and pk.raw[:8] == [0] * 7 + [1]
    rep.add(t, 'sync', ok if strict else (None if not ok else True), '%s SYNC of %d symbols (8: KJKJKJKK, '
            '7.1.10)' % (who, pk.sync_len))
    # stuffing
    ok = not pk.stuff_errors and not pk.final_stuff_missing
    text = '%s %s: %s' % (who, pk.name, 'stuffed as 7.1.9 asks' if ok else
                          'seven 1s at raw bit %s' % pk.stuff_errors if pk.stuff_errors else
                          'six 1s end the packet with no stuffed 0 before the EOP')
    rep.add(t, 'stuffing', ok if strict else (None if not ok else True), text)
    if not pk.octets:
        rep.add(t, 'packet_format', False if dev else None, '%s packet of %d bits, no PID' % (who, len(pk.bits)))
        return
    # pid
    ok = pk.pid_ok and pk.name != 'reserved' and (pk.name in DEV_PIDS if dev else True)
    rep.add(t, 'pid', ok if strict else (None if not ok else True), '%s PID %02X (%s)%s' % (
        who, pk.pid, pk.name, '' if pk.pid_ok else ': check bits not the complement (8.3.1)'))
    # crc
    if pk.crc_ok is not None:
        rep.add(t, 'crc', pk.crc_ok if strict else (None if not pk.crc_ok else True),
                '%s %s CRC%s %s' % (who, pk.name, '5' if pk.name in TOKENS else '16',
                                    'residual right' if pk.crc_ok else 'WRONG (8.3.5)'))
    # packet_format
    n = len(pk.octets)
    if pk.name in TOKENS:
        ok = n == 3 and not pk.extra and not dev
    elif pk.name in HANDSHAKES:
        ok = n == 1 and not pk.extra
    elif pk.name in DATA:
        ok = 3 <= n <= 3 + o['max_packet'] and not pk.extra
    else:
        ok = not dev
    rep.add(t, 'packet_format', ok if strict else (None if not ok else True), '%s %s: %d bytes%s' % (
        who, pk.name, n, ' and %d bits' % pk.extra if pk.extra else ''))
    # eop
    w = pk.se0_width = pk.end - pk.se0
    if dev:
        grid = pk.sop + pk.n_raw * pk.period
        dev_off = pk.se0 - grid
        ok = o['eop_t'][0] <= w <= o['eop_t'][1] and o['tldeop'][0] <= dev_off <= o['tldeop'][1] and \
            pk.eop_state == 'J'
        text = 'device EOP: SE0 %.0f ns (1250 to 1500, TLEOPT), %+.0f ns from the bit grid (TLDEOP -40 to +100), ' \
               'then %s' % (w, dev_off, pk.eop_state)
        j = pk.j_drive
        if j is not None:
            okj = pk.period - 25.0 <= j < 2 * pk.period
            ok = ok and okj
            text += ', driven %.0f ns (one bit, released before two)' % j
        rep.add(pk.se0, 'eop', ok, text)
    else:
        ok = w >= o['eop_r'] and pk.eop_state == 'J'
        rep.add(pk.se0, 'eop', ok if o['strict_host'] else (None if not ok else True),
                'host EOP: SE0 %.0f ns (receivers accept 670, TLEOPR), then %s' % (w, pk.eop_state))
    # ipd and response time, in low-speed bit times (7.1.11), as the host and hubs count them
    if prev is not None and prev.end != float('inf'):
        gap = (pk.sop - prev.end) / T0
        pk.gap = gap
        ok = gap >= o['ipd'] - 1e-9
        rep.add(t, 'ipd', ok if strict else (None if not ok else True), '%.2f bit times from SE0-to-J to the '
                '%s %s (at least 2, 7.1.18.1)' % (gap, who, pk.name))
        if dev and prev.sender == 'host' and gap <= o['timeout']:
            ok = gap <= o['rsp'] + 1e-9
            rep.add(t, 'response_time', ok, 'device %s %.2f bit times after the host\'s %s (at most 6.5, '
                    'TRSPIPD1; %s 7.5 of a captive cable)' % (pk.name, gap, prev.name,
                                                             'within' if gap <= o['rsp_captive'] else 'past'))


def _bus_events(rep, pks, kas, rsts, rec, o):
    dev = rec.get('dev')
    # keep-alive: the device must not idle past 3 ms (7.1.7.6)
    marks = sorted([(p.sop, p.end) for p in pks] + [(a, b) for a, b, _ in kas] + [(a, b) for a, b in rsts])
    worst, at = 0.0, 0.0
    for (a0, b0), (a1, b1) in zip(marks, marks[1:]):
        if a1 - b0 > worst:
            worst, at = a1 - b0, b0
    if marks:
        rep.add(at, 'keepalive', worst <= o['suspend'], 'longest idle J %.3f ms between bus events (at most 3.0 ms, '
                '7.1.7.6); %d keep-alives' % (worst / 1e6, len(kas)))
    for a, b, nxt in kas:
        if nxt != 'J':
            rep.add(a, 'keepalive', False, 'a keep-alive SE0 ends in %s, not J' % nxt)
    # resets
    for a, b in rsts:
        w = b - a
        if dev is not None:
            drv = any(a < t < b and v for t, v in dev) or _level_at(dev, a + 1e-6)
            rep.add(a, 'reset', not drv, 'SE0 of %.3f ms: the device %s' % (w / 1e6, 'drives during it' if drv
                                                                             else 'is silent'))
        kind = 'a reset (TDRST 10 ms)' if w >= o['tdrst'] else 'longer than TDETRST: the device may treat it as a reset'
        rep.add(a, 'reset', None, 'SE0 of %.3f ms is %s' % (w / 1e6, kind))


# ---------------------------------------------------------------------------------------------------------------
# transactions and control transfers

class _Dev:
    """What a correct device must be doing, from what has happened on the bus."""

    def __init__(self):
        self.addr = 0
        self.old = None             # (old address, valid until): during the SetAddress recovery
        self.reset_old = None       # the address before the last reset
        self.stage = 'idle'
        self.req = None
        self.tx_toggle = 1
        self.rx_toggle = 1
        self.stall = False
        self.pending_addr = None
        self.last_data = None       # (pid name, payload) of the device's last data packet not yet ACKed
        self.xfer = None
        self.new_seen = False
        self.conf = 0               # the configuration value, 0 not configured; None where 9.4 leaves it open
        self.alt = {}               # interface -> its alternate setting, when configured
        self.rwake = False          # remote wakeup enabled (9.4.5, Figure 9-4)
        self.halt0 = False          # endpoint 0's halt feature


def _transactions(pks, o):
    """Groups packets: (token, data, handshake) from the host's token. Packets that don't fit stand alone."""
    out, k = [], 0
    while k < len(pks):
        pk = pks[k]
        if pk.sender == 'host' and pk.name in ('SETUP', 'OUT', 'IN'):
            tr = dict(token=pk, data=None, hs=None, t=pk.sop)
            k += 1
            if pk.name in ('SETUP', 'OUT'):
                if k < len(pks) and pks[k].sender == 'host' and pks[k].name in DATA:
                    tr['data'] = pks[k]
                    k += 1
                    if k < len(pks) and pks[k].sender == 'dev':
                        tr['hs'] = pks[k]
                        k += 1
            else:
                if k < len(pks) and pks[k].sender == 'dev':
                    tr['data'] = pks[k]
                    k += 1
                    if tr['data'].name in DATA and k < len(pks) and pks[k].sender == 'host' and \
                            pks[k].name in HANDSHAKES:
                        tr['hs'] = pks[k]
                        k += 1
            out.append(tr)
        else:
            out.append(dict(token=None, data=None, hs=None, t=pk.sop, lone=pk))
            k += 1
    return out


def _late(dev_pk, host_pk, o):
    """A device packet that starts after the host's time-out doesn't answer."""
    return dev_pk is not None and (dev_pk.sop - host_pk.end) / o['bit'] > o['timeout']


def _protocol(rep, pks, rsts, o):
    """The transfers against a model of a correct device. An SE0 longer than TDETRST and shorter than TDRST may or
    may not reset the device (7.1.7.5: a device may treat an SE0 longer than TDETRST as a reset), so each such SE0
    is read both ways, up to 8 of them, and the reading with the fewest failures is kept: no reset first."""
    amb = [r for r in sorted(rsts) if r[1] - r[0] < o['tdrst']][:8]
    if not amb:
        _protocol_run(rep, pks, rsts, o, set())
        return
    best = None
    for mask in range(1 << len(amb)):
        taken = {amb[i] for i in range(len(amb)) if mask >> i & 1}
        sub = Report()
        sub.expected = rep.expected
        sub.config = rep.config
        sub.descriptor = None
        _protocol_run(sub, pks, rsts, o, taken)
        n_bad = sum(1 for f in sub.findings if f.ok is False)
        if best is None or n_bad < best[0]:
            best = (n_bad, taken, sub)
        if n_bad == 0:
            break
    _, taken, sub = best
    rep.findings += sub.findings
    rep.transfers += sub.transfers
    rep.descriptor = sub.descriptor
    for a, b in amb:
        rep.add(a, 'reset', None, 'SE0 of %.3f ms, between TDETRST and TDRST, read as %s (7.1.7.5 allows both)' % (
            (b - a) / 1e6, 'a reset' if (a, b) in taken else 'no reset'))


def _protocol_run(rep, pks, rsts, o, taken):
    d = _Dev()
    resets = sorted(rsts)
    ri = 0
    for tr in _transactions(pks, o):
        while ri < len(resets) and resets[ri][1] <= tr['t']:
            a, b = resets[ri]
            if b - a >= o['tdrst'] or (a, b) in taken:
                if d.addr != 0:
                    d.reset_old = d.addr
                d.addr, d.old, d.stage, d.stall, d.pending_addr = 0, None, 'idle', False, None
                _close(rep, d, o)
                d.conf, d.alt, d.rwake, d.halt0 = 0, {}, False, False     # 9.1.1.3, 9.1.2 step 4
            ri += 1
        if 'lone' in tr:
            pk = tr['lone']
            if pk.sender == 'dev':
                rep.add(pk.sop, 'sequence', False, 'the device sends %s, which answers no host packet' % pk.name)
            continue
        _transaction(rep, d, tr, o)
    _close(rep, d, o)


def _addressed(d, tok, o):
    """(us, why): True if a correct device must answer this token, False if it must not, 'either' inside the
    2 ms after a SetAddress status stage, when the old and the new address may both answer or not (9.2.6.3)."""
    if d.old is not None and tok.sop < d.old[1] and tok.addr in (d.old[0], d.addr):
        return 'either', 'address %d inside the 2 ms SetAddress recovery' % tok.addr
    if d.addr == tok.addr:
        return True, ''
    return False, 'address %d' % tok.addr


def _transaction(rep, d, tr, o):
    tok, data, hs = tr['token'], tr['data'], tr['hs']
    t = tok.sop
    valid_tok = tok.pid_ok and tok.crc_ok and len(tok.octets) == 3 and _dribble_ok(tok)
    us, why = _addressed(d, tok, o) if valid_tok else (False, 'a corrupted token')
    if us is not False and tok.endp != 0:
        us, why = False, 'endpoint %d, which a device with only EP0 ignores' % tok.endp
    reply = hs if tok.name in ('SETUP', 'OUT') else data
    if us == 'either':
        if reply is None:
            rep.add(t, 'address', None, 'no answer to %s %s' % (tok.name, why))
            return
        us = True
    if us is False:
        old_reset = d.reset_old is not None and tok.addr == d.reset_old
        old_set = d.old is not None and tok.addr == d.old[0]
        chk = 'reset' if old_reset else 'address' if old_set else 'sequence'
        if reply is not None:
            rep.add(reply.sop, chk, False, 'the device answers %s to %s %s' % (reply.name, tok.name, why))
        elif chk != 'sequence':
            rep.add(t, chk, True, 'the device ignores %s to its old address %d' % (tok.name, tok.addr))
        return
    if d.old is not None and tok.addr == d.addr and tok.sop >= d.old[1] and not d.new_seen:
        d.new_seen = True
        rep.add(t, 'address', True, 'the device takes %s at its new address %d' % (tok.name, d.addr))
    if tok.name in ('SETUP', 'OUT'):
        host_ok = data is not None and data.pid_ok and data.crc_ok and _dribble_ok(data)
        if not host_ok:
            rep.add(t, 'sequence', reply is None, 'no handshake to %s with a corrupted or missing data packet '
                    '(8.5.3, 8.4.6.3)' % tok.name if reply is None else 'the device answers %s to %s with a '
                    'corrupted data packet' % (reply.name, tok.name))
            return
        if _late(reply, data, o):
            rep.add(reply.sop, 'sequence', False, 'the device answers after the host\'s time-out')
            reply = None
        if tok.name == 'SETUP':
            _setup(rep, d, tok, data, reply, o)
        else:
            _out(rep, d, tok, data, reply, o)
    else:
        if _late(reply, tok, o):
            rep.add(reply.sop, 'sequence', False, 'the device answers after the host\'s time-out')
            reply = None
        _in(rep, d, tok, reply, hs, o)


def _setup(rep, d, tok, data, reply, o):
    t = tok.sop
    ok = reply is not None and reply.type == 2
    rep.add(t, 'sequence', ok, 'SETUP %s: %s (8.4.6.4: always ACK, never NAK or STALL)' % (
        data.payload.hex(), 'ACKed' if ok else 'answered %s' % reply.name if reply else 'no answer'))
    if data.name != 'DATA0':
        rep.add(t, 'toggle', None if not o['strict_host'] else False, 'the host sends SETUP data as %s, not DATA0'
                % data.name)
    if not ok:
        return
    _close(rep, d, o)                   # an ACKed SETUP ends the transfer before it (8.5.3)
    p = data.payload
    if len(p) != 8:
        rep.add(t, 'sequence', None, 'setup data of %d bytes (9.3: always 8)' % len(p))
        return
    req = dict(bm=p[0], b=p[1], wv=p[2] | p[3] << 8, wi=p[4] | p[5] << 8, wl=p[6] | p[7] << 8, t=t, data=[],
               first={}, sent=[], state=(d.addr, d.conf))
    d.req = req
    d.stall = False
    d.tx_toggle, d.rx_toggle = 1, 1
    d.last_data = None
    d.pending_addr = None
    d.stage = ('data_in' if req['bm'] & 0x80 else 'data_out') if req['wl'] else 'status_in'
    if req['bm'] == 0 and req['b'] == 5:
        d.pending_addr = req['wv'] & 0x7F
    rep.transfers.append(req)


def _out(rep, d, tok, data, reply, o):
    t = tok.sop
    if d.stall:
        ok = reply is not None and reply.name == 'STALL'
        rep.add(t, 'sequence', ok, 'OUT while stalled: %s (8.5.3.4)' % (reply.name if reply else 'no answer'))
        return
    if d.stage in ('data_in', 'status_out'):
        # the status stage of a control read (8.5.3.1, 8.5.3.3)
        if reply is None:
            rep.add(t, 'sequence', False, 'no handshake to the status OUT')
            return
        rep.add(t, 'sequence', reply.name in HANDSHAKES, 'status OUT answered %s (Table 8-7)' % reply.name)
        if reply.name == 'ACK':
            _finish(rep, d, t, o)
        elif reply.name == 'STALL':
            d.stall = True
            _stalled(d)
        return
    if d.stage == 'data_out':
        if reply is None:
            rep.add(t, 'sequence', False, 'no handshake to OUT data')
            return
        rep.add(t, 'sequence', reply.name in HANDSHAKES, 'OUT data answered %s' % reply.name)
        if reply.name == 'STALL':
            _stalled(d)
        if reply.name == 'ACK' and data.name == ('DATA1' if d.rx_toggle else 'DATA0'):
            d.rx_toggle ^= 1
        return
    rep.add(t, 'sequence', None, 'OUT with no control transfer open: answered %s' % (
        reply.name if reply else 'nothing'))


def _in(rep, d, tok, reply, hs, o):
    t = tok.sop
    if d.stall:
        ok = reply is not None and reply.name == 'STALL'
        rep.add(t, 'sequence', ok, 'IN while stalled: %s (8.5.3.4)' % (reply.name if reply else 'no answer'))
        return
    if reply is None:
        if d.stage == 'status_in' and d.pending_addr is not None and d.req.get('zlp_sent'):
            rep.add(t, 'address', False, 'no answer to IN at the old address %d: the status stage is not done '
                    'until the host ACKs the status packet (9.2.6.3, 8.5.3.1)' % tok.addr)
        else:
            rep.add(t, 'sequence', False, 'no answer to IN in stage %s' % d.stage)
        return
    if reply.name in ('NAK', 'STALL'):
        if d.stage == 'idle':
            rep.add(t, 'sequence', None, 'IN with no control transfer open: %s' % reply.name)
            return
        rep.add(t, 'sequence', True, 'IN in stage %s answered %s' % (d.stage, reply.name))
        if reply.name == 'STALL':
            d.stall = True
            _stalled(d)
        return
    if reply.type not in (3, 11):
        rep.add(t, 'sequence', False, 'IN answered %s' % reply.name)
        return
    toggle = 1 if reply.type == 11 else 0
    acked = hs is not None and hs.name == 'ACK'
    if d.stage == 'status_in':
        ok = len(reply.payload) == 0
        rep.add(t, 'sequence', ok, 'status IN answered with %d bytes (Table 8-7: zero-length)' % len(reply.payload))
        rep.add(t, 'toggle', toggle == 1, 'status packet %s (8.5.3: DATA1)' % reply.name)
        d.req['zlp_sent'] = True
        if acked:
            _finish(rep, d, t, o)
        return
    if d.stage != 'data_in':
        rep.add(t, 'sequence', False, 'IN answered with data in stage %s' % d.stage)
        return
    req = d.req
    p = reply.payload
    # toggle: the expected one, which only an ACK flips
    rep.add(t, 'toggle', toggle == d.tx_toggle, 'data stage packet %s, expected DATA%d (8.6)' % (reply.name,
                                                                                               d.tx_toggle))
    # retry: the same packet again after a lost ACK
    idx = len(req['data'])
    if idx in req['first']:
        same = req['first'][idx] == p
        rep.add(t, 'retry', same, 'a packet sent again after a lost ACK is %s (8.6.4)' % (
            'identical' if same else 'DIFFERENT: %s, first %s' % (p.hex(), req['first'][idx].hex())))
    else:
        req['first'][idx] = bytes(p)
    left = req['wl'] - sum(len(x) for x in req['data'])
    ok = len(p) <= o['max_packet'] and len(p) <= left
    rep.add(t, 'sequence', ok, 'IN data of %d bytes (at most %d, %d left of wLength %d)' % (
        len(p), o['max_packet'], left, req['wl']))
    if acked:
        req['data'].append(bytes(p))
        req.setdefault('t_first', t)
        req['t_last'] = t
        d.tx_toggle ^= 1
        if len(p) < o['max_packet'] or len(p) >= left:
            d.stage = 'status_out'


def _finish(rep, d, t, o):
    req = d.req
    if req is not None and req['bm'] == 0x80 and req['b'] == 6 and req['wv'] >> 8 == 1:
        _descriptor(rep, req, t, o)
    if d.pending_addr is not None and d.stage == 'status_in':
        old = d.addr
        d.addr = d.pending_addr
        d.old = (old, t + o['recovery'])
        d.new_seen = False
        d.reset_old = None
        rep.add(t, 'address', True, 'SetAddress(%d): status stage done at %.3f ms; the old address %d may answer '
                'until %.3f ms (9.2.6.3)' % (d.addr, t / 1e6, old, (t + o['recovery']) / 1e6))
        d.pending_addr = None
    d.stage = 'idle'
    if req is not None:
        req['done'] = t
        # 9.2.6.3, 9.2.6.4, Table 7-14: no data stage, status within 50 ms of the request; with data to the host,
        # the first data within 500 ms and the status within 50 ms of the last data
        if req['wl'] == 0:
            ok = t - req['t'] <= 50e6
            text = 'request done %.3f ms after its SETUP (at most 50 ms, TDSETADDR, TDRQCMPLTND)' % (
                (t - req['t']) / 1e6)
        elif 't_first' in req:
            ok = req['t_first'] - req['t'] <= 500e6 and t - req['t_last'] <= 50e6
            text = 'first data %.3f ms after the SETUP (at most 500, TDRETDATA1), status %.3f ms after the last ' \
                   '(at most 50, TDRETDATAN)' % ((req['t_first'] - req['t']) / 1e6, (t - req['t_last']) / 1e6)
        else:
            ok, text = True, 'request done with no data'
        rep.add(t, 'sequence', ok, text)


def _descriptor(rep, req, t, o):
    """The first transmission of each data packet is judged; a retry that differs is the retry check's."""
    got = b''.join(req['first'][k] for k in sorted(req['first']))
    want_n = min(req['wl'], 18)
    rep.descriptor = got
    bad = []
    if len(got) != want_n:
        bad.append('%d bytes, want %d' % (len(got), want_n))
    if rep.expected is not None:
        if got != rep.expected[:want_n]:
            bad.append('differs from the expected %s' % rep.expected[:want_n].hex())
    else:
        for i, v, name in ((0, 18, 'bLength'), (1, 1, 'bDescriptorType'), (7, o['max_packet'], 'bMaxPacketSize0')):
            if i < len(got) and got[i] != v:
                bad.append('%s %d, want %d' % (name, got[i], v))
        if len(got) >= 4:
            bcd = got[2] | got[3] << 8
            if any(((bcd >> s) & 15) > 9 for s in (0, 4, 8, 12)):
                bad.append('bcdUSB %04X is not BCD' % bcd)
        if len(got) >= 18 and got[17] < 1:
            bad.append('bNumConfigurations 0 (9.4.3: at least one configuration)')
    rep.add(req['t'], 'descriptor', not bad, 'GET_DESCRIPTOR(DEVICE, %d): %s%s' % (
        req['wl'], got.hex(), ': ' + '; '.join(bad) if bad else ' (9.6.1, Table 9-8)'))


# ---------------------------------------------------------------------------------------------------------------
# standard requests (9.4) in the device's state (9.1.1)

def _stalled(d):
    """The device answered the open request's data or status stage with STALL: a Request Error (9.2.7)."""
    if d.req is not None:
        d.req['stalled'] = True


def parse_config(cfg):
    """A configuration descriptor and the descriptors after it (9.6.3, 9.6.5, 9.6.6): (bConfigurationValue,
    bmAttributes, {bInterfaceNumber: {bAlternateSetting}}, {(interface, alternate setting): {bEndpointAddress}})."""
    ifs, eps, cur = {}, {}, None
    k = cfg[0]
    while k + 1 < len(cfg) and cfg[k] >= 2:
        if cfg[k + 1] == 4 and k + 3 < len(cfg):
            cur = (cfg[k + 2], cfg[k + 3])
            ifs.setdefault(cur[0], set()).add(cur[1])
            eps.setdefault(cur, set())
        elif cfg[k + 1] == 5 and cur is not None and k + 2 < len(cfg):
            eps[cur].add(cfg[k + 2])
        k += cfg[k]
    return cfg[5], cfg[7], ifs, eps


def _close(rep, d, o):
    """The open request is over: a new SETUP was ACKed, a reset came, or the record ended. Judge it once."""
    req = d.req
    if req is None or req.get('closed') or 'state' not in req:
        return
    req['closed'] = True
    _request(rep, d, req, 'done' if 'done' in req else 'stall' if req.get('stalled') else 'open', o)


def _state_name(addr, conf):
    if addr == 0:
        return 'Default'
    if conf is None:
        return None                     # Address or Configured: a request 9.4 leaves open came before
    return 'Configured' if conf else 'Address'


def _request(rep, d, req, outcome, o):
    """Judges a request against 9.4 for the state at its SETUP, and applies what it does to the configuration.
    outcome: 'done' (its status stage completed), 'stall' (a STALL in its data or status stage) or 'open'."""
    addr, conf = req['state']
    st = _state_name(addr, conf)
    bm, b, wv, wi, wl = req['bm'], req['b'], req['wv'], req['wi'], req['wl']
    cfg = parse_config(rep.config) if rep.config else None
    got = b''.join(req['first'][k] for k in sorted(req['first']))
    want, data, why = None, None, ''
    open_text = '9.4 leaves it open'

    def need(kind, text, value=None):
        return kind, value, text

    if bm == 0x80 and b == 6:
        kind, index = wv >> 8, wv & 255
        name = 'GET_DESCRIPTOR(%s, %d)' % ({1: 'DEVICE', 2: 'CONFIGURATION', 3: 'STRING'}.get(kind, kind), index)
        n_conf = rep.expected[17] if rep.expected is not None and len(rep.expected) >= 18 else None
        if kind == 1 and index == 0:
            want, data, why = need('done', '9.4.3: every device has a device descriptor, in every state')
        elif kind == 2 and n_conf is not None and index >= n_conf:
            want, data, why = need('stall', '9.4.3: no such descriptor, a Request Error (bNumConfigurations %d)'
                                   % n_conf)
        elif kind == 2 and index == 0 and cfg is not None:
            total = rep.config[2] | rep.config[3] << 8
            want, data, why = need('done', '9.4.3: the configuration with its interfaces, min(wLength, '
                                   'wTotalLength %d) bytes' % total, rep.config[:min(wl, total)])
        else:
            why = 'not judged here'
    elif bm == 0x80 and b == 8:
        name = 'GET_CONFIGURATION'
        if wv or wi or wl != 1:
            why = '9.4.2: wValue, wIndex or wLength not as specified'
        elif st is None or st == 'Default':
            why = open_text if st else 'the state is not known'
        else:
            want, data, why = need('done', '9.4.2: %s state' % st, bytes([conf]))
    elif b == 0 and bm in (0x80, 0x81, 0x82):
        who = {0x80: 'device', 0x81: 'interface %d' % (wi & 255), 0x82: 'endpoint 0x%02X' % (wi & 255)}[bm]
        name = 'GET_STATUS(%s)' % who
        if wv or wl != 2 or (bm == 0x80 and wi) or wi >> 8:
            why = '9.4.5: wValue, wIndex or wLength not as specified'
        elif st == 'Default':
            why = open_text
        elif bm == 0x80:
            bus_only = cfg is not None and not cfg[1] & 0x40

            def data(x, rw=d.rwake, bus_only=bus_only):
                v = x[0] | x[1] << 8 if len(x) == 2 else None
                return v is not None and not v & 0xFFFC and (v >> 1 & 1) == rw and not (bus_only and v & 1)
            want, why = 'done', '9.4.5, Figure 9-4: reserved bits 0, remote wakeup %d%s' % (
                d.rwake, ', self powered 0: bmAttributes offers bus power only' if bus_only else '')
        elif bm == 0x82 and not wi & 0x7F:
            want, data, why = need('done', '9.4.5, Figure 9-6: endpoint 0, halt %d' % d.halt0, bytes([d.halt0, 0]))
        elif st == 'Address':
            want, data, why = need('stall', '9.4.5: Address state, a Request Error for an interface or an endpoint '
                                   'other than endpoint 0')
        elif st is None or cfg is None:
            why = 'the state or the configuration is not known'
        elif bm == 0x81:
            have = (wi & 255) in cfg[2]
            want, data, why = need('done' if have else 'stall', '9.4.5, Figure 9-5: %s' % (
                'two bytes 0' if have else 'no such interface, a Request Error'), b'\x00\x00' if have else None)
        else:
            have = any((wi & 255) in e for (i, a), e in cfg[3].items() if d.alt.get(i, 0) == a)
            want, data, why = need('done' if have else 'stall', '9.4.5: %s' % (
                'an endpoint of the configuration' if have else 'no such endpoint, a Request Error'))
    elif bm == 0x81 and b == 10:
        name = 'GET_INTERFACE(%d)' % (wi & 255)
        if wv or wl != 1:
            why = '9.4.4: wValue or wLength not as specified'
        elif st == 'Default' or st is None:
            why = open_text if st else 'the state is not known'
        elif st == 'Address':
            want, data, why = need('stall', '9.4.4: Address state, a Request Error')
        elif cfg is None:
            why = 'the configuration is not known'
        elif (wi & 255) in cfg[2]:
            want, data, why = need('done', '9.4.4: the alternate setting', bytes([d.alt.get(wi & 255, 0)]))
        else:
            want, data, why = need('stall', '9.4.4: no such interface, a Request Error')
    elif bm == 0x01 and b == 11:
        i, alt = wi & 255, wv
        name = 'SET_INTERFACE(%d, %d)' % (i, alt)
        if wl:
            why = '9.4.10: wLength not 0'
        elif st == 'Default' or st is None:
            why = open_text if st else 'the state is not known'
        elif st == 'Address':
            want, data, why = need('stall', '9.4.10: Address state, a Request Error')
        elif cfg is None:
            why = 'the configuration is not known'
        elif alt not in cfg[2].get(i, ()):
            want, data, why = need('stall', '9.4.10: no such interface or setting, a Request Error')
        elif cfg[2][i] == {0}:
            why = '9.4.10: the default setting alone, which a STALL may refuse'
            if outcome == 'done':
                d.alt[i] = alt
        else:
            want, data, why = need('done', '9.4.10: an alternate setting of the configuration')
            if outcome == 'done':
                d.alt[i] = alt
    elif bm == 0x00 and b == 9:
        v = wv & 255
        name = 'SET_CONFIGURATION(%d)' % v
        valid = v == 0 or (cfg is not None and v == cfg[0])
        if wi or wl or wv >> 8:
            why = '9.4.7: wIndex, wLength or wValue\'s upper byte not 0'
        elif st == 'Default':
            why = '9.4.7: Default state, ' + open_text
        elif v and cfg is None:
            why = 'the configuration is not known'
        elif valid:
            want, data, why = need('done', '9.4.7: %s' % ('0, not configured' if not v else
                                                          'the configuration\'s bConfigurationValue'))
        else:
            want, data, why = need('stall', '9.4.7: no configuration has the value %d, a Request Error' % v)
        # the value takes effect before the status stage completes (9.2.6), so a transfer cut short leaves either
        if outcome == 'done' and valid and want == 'done':
            d.conf, d.alt, d.halt0 = v, {}, False
        elif outcome != 'stall' and d.conf != (v if valid else -1):
            d.conf = None
    elif bm == 0x00 and b == 5:
        name = 'SET_ADDRESS(%d)' % wv
        if wi or wl or not 0 < wv <= 127:
            why = '9.4.6: address 0, above 127, or wIndex or wLength not 0'
        elif st == 'Configured' or st is None:
            why = ('9.4.6: Configured state, ' + open_text) if st else 'the state is not known'
        else:
            want, data, why = need('done', '9.4.6: %s state' % st)
        if outcome == 'done' and st in ('Default', 'Address'):
            d.conf = 0                  # 9.4.6: the device is then in the Address state, not configured
        elif st == 'Configured':
            d.conf = None
    elif b in (1, 3) and bm in (0x00, 0x01, 0x02):
        op = 'SET_FEATURE' if b == 3 else 'CLEAR_FEATURE'
        name = '%s(%s, %d)' % (op, {0: 'device', 1: 'interface %d' % (wi & 255), 2: 'endpoint 0x%02X' % (
            wi & 255)}[bm], wv)
        if wl:
            why = '9.4.1, 9.4.9: wLength not 0'
        elif st == 'Default':
            why = open_text
        elif bm == 0x00 and wv == 1 and cfg is not None:
            has = bool(cfg[1] & 0x20)
            want, data, why = need('done' if has else 'stall', 'Table 9-6: remote wakeup, %s' % (
                'which bmAttributes offers' if has else 'which bmAttributes does not offer: a Request Error'))
            if outcome == 'done' and has:
                d.rwake = b == 3
        elif bm == 0x00 and wv == 2:
            if b == 1:
                want, data, why = need('stall', '9.4.1: TEST_MODE cannot be cleared, a Request Error')
            else:
                why = '9.4.9: TEST_MODE, a high-speed feature'
        elif bm == 0x00 and wv != 1:
            want, data, why = need('stall', 'Table 9-6: no such device feature, a Request Error')
        elif bm == 0x01:
            want, data, why = need('stall', 'Table 9-6: no interface feature, a Request Error')
        elif bm == 0x02 and wv == 0 and not wi & 0x7F:
            why = '9.4.5: the default pipe\'s halt is optional'
            if outcome == 'done':
                d.halt0 = b == 3
        elif bm == 0x02 and wv != 0:
            want, data, why = need('stall', 'Table 9-6: no such endpoint feature, a Request Error')
        elif bm == 0x02 and st == 'Address':
            want, data, why = need('stall', '9.4.1, 9.4.9: Address state, an endpoint other than endpoint 0')
        else:
            why = 'the state or the configuration is not known'
    else:
        return                          # class, vendor and the other standard requests: not judged here
    text = '%s in the %s state: %s' % (name, st or 'Address or Configured', {
        'done': 'completed', 'stall': 'STALLed', 'open': 'neither completed nor STALLed'}[outcome])
    if outcome == 'done' and req['bm'] & 0x80:
        text += ', %s' % (got.hex() or 'no data')
    if want is None:
        rep.add(req['t'], 'request', None, '%s (%s)' % (text, why))
        return
    if outcome == 'open':
        rep.add(req['t'], 'request', None, '%s; wanted %s (%s): the transfer ended first' % (
            text, 'completion' if want == 'done' else 'a STALL', why))
        return
    ok = outcome == want
    if ok and want == 'done' and data is not None:
        ok = data(got) if callable(data) else got == data
        if not ok:
            why += '; wanted %s' % (data.hex() if isinstance(data, bytes) else 'other bytes')
    rep.add(req['t'], 'request', ok, '%s (%s)' % (text, why))


# ---------------------------------------------------------------------------------------------------------------
# sigrok's usb_request as a second opinion

SIGROK = shutil.which('sigrok-cli')
TICK_NS = 10                    # VCD timescale: sigrok takes 1, 10 or 100 units


def write_vcd(path, rec, t_end=None):
    """D+ and D- of the record as a VCD file with a 10 ns tick."""
    t_end = t_end or rec.get('t_end') or max([t for t, _ in rec['dp'] + rec['dm']] + [0.0]) + 20000.0
    ev = sorted([(round(t / TICK_NS), '!', v) for t, v in rec['dp']] + [(round(t / TICK_NS), '"', v)
                                                                         for t, v in rec['dm']])
    out = ['$timescale 10 ns $end', '$scope module usb $end', '$var wire 1 ! dp $end', '$var wire 1 " dm $end',
           '$upscope $end', '$enddefinitions $end', '#0', '0!', '1"']
    last = None
    for tick, ch, v in ev:
        if tick != last:
            out.append('#%d' % max(tick, 1))
            last = tick
        out.append('%d%s' % (v, ch))
    out.append('#%d' % round(t_end / TICK_NS))
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w') as fh:
        fh.write('\n'.join(out) + '\n')
    return path


def sigrok_requests(rec, path):
    """sigrok's usb_request annotations for the record, as strings like 'SETUP in: [ ... ][ ... ] : ACK', or
    None if sigrok-cli isn't installed. path is where the VCD file goes."""
    if SIGROK is None:
        return None
    write_vcd(path, rec)
    stack = 'usb_signalling:dp=dp:dm=dm:signalling=low-speed,usb_packet:signalling=low-speed,usb_request'
    p = subprocess.run([SIGROK, '-i', path, '-P', stack, '-A', 'usb_request'], capture_output=True, text=True)
    out = []
    for line in p.stdout.splitlines():
        m = re.match(r'^usb_request-\d+: (.*)$', line)
        if m:
            out.append(m.group(1).strip())
    return out


def requests_as_sigrok(rep):
    """This checker's control transfers in usb_request's summary form, for comparing with sigrok_requests()."""
    out = []
    for req in rep.transfers:
        if 'done' not in req:
            continue
        setup = bytes([req['bm'], req['b'], req['wv'] & 255, req['wv'] >> 8, req['wi'] & 255, req['wi'] >> 8,
                       req['wl'] & 255, req['wl'] >> 8])
        kind = 'SETUP in' if req['bm'] & 0x80 else 'SETUP out'
        data = b''.join(req['data'])
        out.append('%s: [%s ][%s ] : ACK' % (kind, ''.join(' %02X' % b for b in setup),
                                             ''.join(' %02X' % b for b in data)))
    return out


def compare_with_sigrok(srk, rep):
    """(ok, text): every control transfer that sigrok's usb_request completes is one of this checker's, with the
    same setup packet, data and handshake, in the same order. usb_request has no time-out after a data packet: a
    data packet with no handshake (a lost ACK, a corrupted or unanswered SETUP) leaves it waiting, so it drops the
    next SETUP and reports that transfer's data stage as BULK. Its ERR and BULK lines are counted, not compared."""
    mine = requests_as_sigrok(rep)
    theirs = [s for s in srk if s.startswith('SETUP ')]
    i = 0
    for s in theirs:
        while i < len(mine) and mine[i] != s:
            i += 1
        if i == len(mine):
            return False, 'sigrok\'s %r is not among the checker\'s transfers %s' % (s, mine)
        i += 1
    n_err = sum(1 for s in srk if s.startswith('ERR'))
    n_bulk = sum(1 for s in srk if s.startswith('BULK'))
    return True, '%d of the checker\'s %d transfers confirmed by sigrok; it also printed %d ERR and %d BULK lines' % (
        len(theirs), len(mine), n_err, n_bulk)
