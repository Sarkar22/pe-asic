"""Self-test of usb_check.py and the USB bus model, on synthetic records built from USB 2.0.

The generators share no code with the checker: packets are built field by field from 8.3 and 8.4 (PID and its
check nibble, ADDR, ENDP, CRC5 and CRC16 computed as 8.3.5 describes and sent MSb first), stuffed from the SYNC
(7.1.9), NRZI-coded (7.1.8) from idle J (Table 7-2), and ended with SE0 for two bit times and J for one
(7.1.7.4.1). Host and device drive frontend.UsbLs, which adds the pull-up and pull-downs and records the bus.

One good enumeration must pass every check: reset, keep-alives every 1 ms, GET_DESCRIPTOR(DEVICE, 64) at
address 0 with a NAK and a lost ACK, SET_ADDRESS(5) with a lost status ACK, the old address ignored after the
2 ms recovery, GET_DESCRIPTOR(DEVICE, 18) at address 5, a token for another address, a corrupted SETUP, a
second reset after which the old address is ignored and address 0 answers. Each bad record changes one thing
and must fail exactly one check.
"""
import os

import frontend
import usb_check

PROGRAMS = ('bench/usb_check.py',)

BIT = 1e9 / 1.5e6
J, K, SE0, SE1 = (0, 1), (1, 0), (0, 0), (1, 1)          # (D+, D-), low speed (Table 7-2)
PID = dict(OUT=0b0001, IN=0b1001, SETUP=0b1101, DATA0=0b0011, DATA1=0b1011, ACK=0b0010, NAK=0b1010,
           STALL=0b1110)                                  # Table 8-1


# ---------------------------------------------------------------------------------------------------------------
# packets, from the standard

def lsb(value, n):
    return [(value >> i) & 1 for i in range(n)]


def pid_bits(name, check=None):
    t = PID[name]
    c = (~t) & 15 if check is None else check
    return lsb(t, 4) + lsb(c, 4)                          # 8.3.1: type, then its one's complement


def crc_bits(bits, width, poly):
    """8.3.5: seeded with ones; per bit, the remainder's top bit XOR the data bit; shift left; XOR the polynomial
    if that was 1. At the end the remainder is inverted and sent MSb first."""
    reg = (1 << width) - 1
    for b in bits:
        f = ((reg >> (width - 1)) & 1) ^ b
        reg = (reg << 1) & ((1 << width) - 1)
        if f:
            reg ^= poly
    reg ^= (1 << width) - 1
    return [(reg >> (width - 1 - i)) & 1 for i in range(width)]


def token(name, addr, endp):
    body = lsb(addr, 7) + lsb(endp, 4)                    # 8.3.2
    return pid_bits(name) + body + crc_bits(body, 5, 0b00101)


def data(name, payload, bad_crc=False):
    body = [b for x in payload for b in lsb(x, 8)]
    c = crc_bits(body, 16, 0x8005)
    if bad_crc:
        c[-1] ^= 1
    return pid_bits(name) + body + c


def handshake(name, check=None):
    return pid_bits(name, check)


def stuff(bits, final=True):
    """SYNC 00000001 then the packet, a 0 after every six 1s counting the SYNC's last 1 (7.1.9). final=False
    leaves out a stuff bit due after the last bit, which 7.1.9 forbids."""
    out = [0] * 7 + [1]
    run = 1
    for b in bits:
        out.append(b)
        run = run + 1 if b else 0
        if run == 6:
            out.append(0)
            run = 0
    if not final and run == 0 and out[-1] == 0 and len(out) >= 7 and out[-7:-1] == [1] * 6:
        out.pop()
    return out


def ends_in_six(bits):
    """True if the stuffed packet ends with a stuff bit after its last bit."""
    s = stuff(bits)
    return s[-1] == 0 and s[-7:-1] == [1] * 6 and len(stuff(bits, final=False)) == len(s) - 1


# ---------------------------------------------------------------------------------------------------------------
# driving the bus

class Bus:
    """A host and a device on frontend.UsbLs. Times in ns. Each send() is one packet from idle J."""

    def __init__(self, **kw):
        self.u = frontend.UsbLs(**kw)
        self.t = 0.0

    def drive(self, who, t, state, on=1):
        if who == 'host':
            self.u.host(t, state[0], state[1], on)
        else:
            self.u.device(t, state[0], state[1], on, on)

    def send(self, who, bits, gap=3.0, period=BIT, se0=2.0, j=1.0, final=True, jit=None, sync=None, dribble=0):
        """Starts gap bit times after the last packet's SE0-to-J (7.1.18.1); returns the end of its EOP's J.
        dribble: bit times the last level is held before the EOP, as a hub's switching skew stretches it (7.1.9.1)."""
        raw = stuff(bits, final)
        if sync is not None:
            raw = sync + raw[8:]
        t0 = self.t + gap * period
        st = J
        for i, b in enumerate(raw):
            if b == 0:
                st = K if st == J else J
                self.drive(who, t0 + i * period + (jit or {}).get(i, 0.0), st)
        t_eop = t0 + (len(raw) + dribble) * period
        self.drive(who, t_eop, SE0)
        self.drive(who, t_eop + se0 * period, J)
        self.drive(who, t_eop + (se0 + j) * period, J, 0)
        self.t = t_eop + se0 * period
        return self.t

    def idle(self, ns):
        self.t += ns

    def se0(self, ns, who='host'):
        """A keep-alive (a low-speed EOP: Table 7-2 note 4) or, held 10 ms, a reset (7.1.7.5)."""
        self.drive(who, self.t, SE0)
        self.drive(who, self.t + ns, J)
        self.drive(who, self.t + ns + BIT, J, 0)
        self.t += ns + BIT

    def record(self):
        return self.u.record(self.t + 50e3)


DESC = bytes([18, 1, 0x10, 0x01, 0, 0, 0, 8, 0x09, 0x12, 0x00, 0x00, 0x00, 0x01, 0, 0, 0, 1])


def descriptor():
    """A device descriptor (9.6.1, Table 9-8) one of whose packets at address 5 ends in a stuff bit, so that the
    stuffing check has a final stuff bit to judge: idProduct is picked for it."""
    for pid in range(1, 65536):
        d = bytearray(DESC)
        d[10], d[11] = pid & 255, pid >> 8
        names = ('DATA1', 'DATA0', 'DATA1')
        if any(ends_in_six(data(names[k], bytes(d[8 * k:8 * k + 8]))) for k in range(3)):
            return bytes(d)
    raise RuntimeError('no idProduct gives a final stuff bit')


def scenario(mod=None, desc=None):
    """The enumeration, with one change named by mod. Returns the record."""
    mod = mod or ''
    desc = desc or descriptor()
    b = Bus()
    # 1.55% fast: past TLDRATE's 1.5%, still inside TUDJ2 (150 ns over 14 bits), which counts the tolerance
    dev_period = BIT / 1.0155 if mod == 'bit_rate' else BIT

    def dev(bits, gap=3.0, **kw):
        return b.send('dev', bits, gap=gap, period=dev_period, **kw)

    def host(bits, gap=3.0, dribble=0):
        return b.send('host', bits, gap=gap, dribble=dribble)

    def frame_gap():
        b.idle(0.25e6)
        b.se0(2 * BIT)                    # keep-alive
        b.idle(0.25e6)

    b.idle(100e3)
    b.se0(10.2e6)                         # reset
    for _ in range(2):
        b.idle(0.5e6)
        b.se0(2 * BIT)

    def get_desc(addr, wl, nak=False, lost=None, toggles=('DATA1', 'DATA0', 'DATA1'), retry_change=False,
                 setup_nak=False):
        host(token('SETUP', addr, 0))
        host(data('DATA0', [0x80, 6, 0, 1, 0, 0, wl, 0]), gap=2.5)
        if setup_nak:
            dev(handshake('NAK'), gap=3.0)
            frame_gap()
            host(token('SETUP', addr, 0))
            host(data('DATA0', [0x80, 6, 0, 1, 0, 0, wl, 0]), gap=2.5)
        if mod == 'pid' and addr == 5:
            dev(handshake('ACK', check=0b0101))
        elif mod == 'packet_format' and addr == 5:
            dev(handshake('ACK') + lsb(0x55, 8))
        elif mod == 'sync' and addr == 5:
            dev(handshake('ACK'), sync=[0, 0, 0, 0, 0, 1])
        elif mod == 'ipd' and addr == 5:
            dev(handshake('ACK'), gap=1.5)
        elif mod == 'response_time' and addr == 5:
            dev(handshake('ACK'), gap=7.0)
        elif mod == 'eop' and addr == 5:
            dev(handshake('ACK'), se0=1.5)
        else:
            dev(handshake('ACK'))
        frame_gap()
        n = min(wl, len(desc))
        k = 0
        sent = 0
        while sent < n:
            chunk = desc[sent:sent + 8][:n - sent]
            host(token('IN', addr, 0))
            if nak and k == 0:
                dev(handshake('NAK'))
                frame_gap()
                host(token('IN', addr, 0))
            kw = {}
            if mod == 'jitter' and addr == 5 and k == 1:
                raw = stuff(data(toggles[k % 3], chunk))
                kw['jit'] = {next(i for i in range(40, 80) if raw[i] == 0): 30.0}
            if mod == 'stuffing' and addr == 5:
                kw['final'] = False
            dev(data(toggles[k % 3], chunk, bad_crc=mod == 'crc' and addr == 5 and k == 0), **kw)
            if lost == k:
                frame_gap()                   # the host's ACK is lost: it asks again
                host(token('IN', addr, 0))
                again = bytes(chunk[:-1]) + bytes([chunk[-1] ^ 1]) if retry_change else chunk
                dev(data(toggles[k % 3], again))
            host(handshake('ACK'))
            frame_gap()
            sent += len(chunk)
            k += 1
        host(token('OUT', addr, 0))
        host(data('DATA1', []), gap=2.5)
        dev(handshake('ACK'))
        frame_gap()

    get_desc(0, 64, nak=True, lost=1, retry_change=mod == 'retry')
    # SET_ADDRESS(5); the first status ACK is lost (8.5.3.1)
    host(token('SETUP', 0, 0))
    host(data('DATA0', [0, 5, 5, 0, 0, 0, 0, 0]), gap=2.5)
    dev(handshake('NAK') if mod == 'sequence' else handshake('ACK'))
    if mod == 'sequence':
        frame_gap()
        host(token('SETUP', 0, 0))
        host(data('DATA0', [0, 5, 5, 0, 0, 0, 0, 0]), gap=2.5)
        dev(handshake('ACK'))
    frame_gap()
    host(token('IN', 0, 0))
    dev(data('DATA1', []))
    frame_gap()
    host(token('IN', 0, 0))
    if mod == 'address_early':
        frame_gap()                       # the device moved to address 5 before the ACK: silence
        host(token('IN', 0, 0))
    dev(data('DATA1', []))
    host(handshake('ACK'))
    for _ in range(4 if mod != 'keepalive' else 0):      # past the 2 ms SetAddress recovery (9.2.6.3)
        frame_gap()
    if mod == 'keepalive':
        b.idle(4e6)
    b.idle(0.2e6)
    # the old address after the recovery
    host(token('SETUP', 0, 0))
    host(data('DATA0', [0x80, 6, 0, 1, 0, 0, 8, 0]), gap=2.5)
    if mod == 'address':
        dev(handshake('ACK'))
    frame_gap()
    toggles = ('DATA0', 'DATA0', 'DATA1') if mod == 'toggle' else ('DATA1', 'DATA0', 'DATA1')
    get_desc(5, 18, toggles=toggles)
    # another address, and a corrupted SETUP: no answer
    host(token('IN', 9, 0))
    frame_gap()
    host(token('SETUP', 5, 0))
    host(data('DATA0', [0x80, 6, 0, 1, 0, 0, 18, 0], bad_crc=True), gap=2.5)
    frame_gap()
    if mod == 'line_state':
        b.drive('host', b.t, SE1)
        b.drive('host', b.t + 1000.0, J)
        b.drive('host', b.t + 1000.0 + BIT, J, 0)
        b.idle(1000.0 + BIT)
        frame_gap()
    # a reset: the old address is ignored, address 0 answers. With short_*, an SE0 of 50 us instead, which a device
    # may take as a reset or not (7.1.7.5): short_reset takes it, short_noreset keeps address 5, short_both answers
    # both addresses, which neither reading allows
    short = mod in ('short_reset', 'short_noreset', 'short_both')
    b.se0(50e3 if short else 10.1e6)
    b.idle(0.5e6)
    b.se0(2 * BIT)
    b.idle(0.5e6)
    get8 = [0x80, 6, 0, 1, 0, 0, 8, 0]
    host(token('SETUP', 5, 0))
    host(data('DATA0', get8), gap=2.5)
    if mod in ('reset', 'short_both', 'short_noreset'):
        dev(handshake('ACK'))
    frame_gap()
    if mod == 'short_noreset':
        host(token('IN', 5, 0))
        dev(data('DATA1', desc[:8]))
        host(handshake('ACK'))
        frame_gap()
        host(token('OUT', 5, 0))
        host(data('DATA1', []), gap=2.5)
        dev(handshake('ACK'))
        frame_gap()
        host(token('SETUP', 0, 0))
        host(data('DATA0', get8), gap=2.5)
        frame_gap()
        return b.record()
    # the SETUP's data with a dribble bit before its EOP, which the device must take (7.1.9.1)
    assert stuff(data('DATA0', get8))[-6:] != [1] * 6
    host(token('SETUP', 0, 0))
    host(data('DATA0', get8), gap=2.5, dribble=1)
    if mod != 'dribble':
        dev(handshake('ACK'))
    frame_gap()
    host(token('IN', 0, 0))
    dev(data('DATA1', desc[:8]))
    host(handshake('ACK'))
    frame_gap()
    host(token('OUT', 0, 0))
    host(data('DATA1', []), gap=2.5)
    dev(handshake('ACK'))
    frame_gap()
    return b.record()


BAD = ('bit_rate', 'jitter', 'sync', 'stuffing', 'pid', 'crc', 'packet_format', 'eop', 'ipd', 'response_time',
       'keepalive', 'reset', 'sequence', 'toggle', 'retry', 'address', 'address_early', 'descriptor', 'line_state',
       'dribble', 'short_both')
EXPECT = dict(address_early='address', dribble='sequence', short_both='reset')


def run(mod=None, desc=None):
    good = descriptor()
    rec = scenario(mod, desc=bytes([18, 1, 0x10, 0x01, 0, 0, 0, 16]) + good[8:] if mod == 'descriptor' else good)
    return usb_check.check(rec, descriptor=good), rec


def main(log):
    log.group('bench/usb_check.py', title='USB low-speed checker self-test: synthetic records from USB 2.0')
    rep, rec = run()
    counts = rep.checks()
    log.check('good enumeration passes every check', rep.ok and all(counts[c][0] > 0 for c in usb_check.Report.CHECKS),
              ', '.join('%s %d' % (c, counts[c][0]) for c in usb_check.Report.CHECKS) +
              ('' if rep.ok else '; ' + repr(rep.failed()[:3])))
    names = [p.name for p in rep.packets]
    log.check('good enumeration: packets, keep-alives and resets found', len(rep.resets) == 2 and len(rep.keepalives)
              > 20 and len(rep.transfers) == 4, '%d packets, %d keep-alives, %d resets, %d control transfers' % (
                  len(names), len(rep.keepalives), len(rep.resets), len(rep.transfers)))
    log.check('good enumeration: the descriptor and the address as the device gave them',
              rep.descriptor == descriptor()[:8] and any(f.check == 'address' and f.ok for f in rep.findings),
              rep.descriptor.hex() if rep.descriptor else None)
    for mod in BAD:
        r, _ = run(mod)
        got = r.failing_checks()
        want = EXPECT.get(mod, mod)
        log.check('bad record "%s" fails exactly %s' % (mod, want), got == [want], 'fails %s: %s' % (
            got, r.failed()[:2]))
    # an SE0 of 50 us, past TDETRST and short of TDRST: a device may take it as a reset or not (7.1.7.5)
    for mod, what in (('short_reset', 'taken as a reset: address 0 answers, 5 doesn\'t'),
                      ('short_noreset', 'not taken: address 5 answers, 0 doesn\'t')):
        r, _ = run(mod)
        info = [f.text for f in r.findings if f.check == 'reset' and f.ok is None and 'TDETRST and TDRST' in f.text]
        log.check('an SE0 of 50 us %s; either passes' % what, r.ok, info or r.failed()[:2])
    # the bus model: the pull-up brings D- back in 674 ns at 450 pF; a short D- lag makes TLST-short SE0s
    u = frontend.UsbLs()
    u.device(0.0, 0, 0, 1, 1)
    u.device(1000.0, 0, 0, 0, 0)
    r = u.record(3000.0)
    log.check('bus model: D- released from SE0 reads J after the pull-up time (USB 7.1.5.1)',
              r['dm'] and abs(r['dm'][-1][0] - 1674.1) < 1.0, r['dm'])
    b = Bus(dm_lag=40.0)
    b.idle(1000.0)
    b.send('dev', handshake('ACK'))
    b.idle(2000.0)
    rr = b.record()
    short = [f for f in usb_check.check(rr).findings if f.check == 'line_state' and f.ok is None]
    log.check('bus model: a 40 ns D- lag shows as SE0 and SE1 shorter than TLST, taken as transitions',
              len(short) >= 4, '%d transient states' % len(short))
    # sigrok's usb_request on the same record, when sigrok-cli is installed
    if usb_check.SIGROK is None:
        log.info('sigrok-cli is not installed: the usb_request comparison is skipped')
        return
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'sigrok', 'work', 'usb_check_test.vcd')
    srk = usb_check.sigrok_requests(rec, path)
    ok, text = usb_check.compare_with_sigrok(srk or [], rep)
    log.check('sigrok usb_request confirms the control transfers it completes', srk is not None and ok and
              text.startswith('3 of'), text)
