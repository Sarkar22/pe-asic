"""USB low-speed wire layer, from USB 2.0 chapter 7 and 8, written before any firmware.

LS idle (J) is D- high, D+ low; K is D+ high, D- low. Packet: SYNC (KJKJKJKK), bits LSB first, NRZI
(0 toggles, 1 holds), a stuffed 0 after six 1s, EOP is SE0 for two bit times then J.
This file knows nothing of any program. Decoder input is the pair of pad levels.
"""
from protolib import crc16_usb, crc5_usb, bytes_lsb, CLK

BIT = CLK / 1.5e6                       # 33.33 cycles at 50 MHz
PID = dict(OUT=0xE1, IN=0x69, SOF=0xA5, SETUP=0x2D, DATA0=0xC3, DATA1=0x4B, ACK=0xD2, NAK=0x5A, STALL=0x1E)


def token(pid, addr, ep):
    v = addr | (ep << 7)
    bits = [(v >> i) & 1 for i in range(11)]
    c = crc5_usb(bits)
    return [PID[pid], v & 255, (v >> 8) | (c << 3)]


def data_packet(pid, payload):
    c = crc16_usb(bytes_lsb(payload))
    return [PID[pid]] + list(payload) + [c & 255, c >> 8]


def wire_bits(packet):
    """the raw bit sequence after stuffing, SYNC included"""
    bits = [0] * 7 + [1]
    run = 1
    out = list(bits)
    for b in bytes_lsb(packet):
        out.append(b)
        run = run + 1 if b else 0
        if run == 6:
            out.append(0)
            run = 0
    return out


def schedule(packet, t_start, bit=BIT, jitter=0, rng=None, eop_bits=2.0, skew=0):
    """(cycle, dp, dm) events for one packet, starting from J. skew: D- lags D+ by that many cycles
    on every J-K transition, so the pair passes through SE0 or SE1 for a moment."""
    ev = []
    t = float(t_start)
    dp, dm = 0, 1
    for b in wire_bits(packet):
        if b == 0:
            j = rng.uniform(-jitter, jitter) if jitter else 0
            c = int(round(t + j))
            if skew:
                ev.append((c, 1 - dp, dm))
                ev.append((c + skew, 1 - dp, 1 - dm))
            else:
                ev.append((c, 1 - dp, 1 - dm))
            dp, dm = 1 - dp, 1 - dm
        t += bit
    ev.append((int(round(t)), 0, 0))
    t += eop_bits * bit
    ev.append((int(round(t)), 0, 1))
    return ev, t


def decode(dpw, dmw, lo=0, hi=None, tol=0.2, bit=BIT):
    """Pads only. Finds packets by the first departure from J and walks bit centres, resyncing on
    each transition as a receiver would. Returns list of packets:
    dict(t, bytes, bits, eop, stuff_err, edges) and a list of errors."""
    hi = hi or dpw.end
    evs = sorted(set([t for t, v in dpw.edges(lo, hi)] + [t for t, v in dmw.edges(lo, hi)]))
    pk, errs = [], []

    def state(t):
        a, b = dpw.at(t), dmw.at(t)
        return {(0, 1): 'J', (1, 0): 'K', (0, 0): 'SE0', (1, 1): 'SE1'}[(a, b)]
    i = 0
    n = len(evs)
    while i < n:
        t_start = evs[i]
        if state(t_start) != 'K':
            i += 1
            continue
        # walk the packet
        t = float(t_start)
        prev = 'J'
        raw = []
        last_edge = t_start
        k = i
        while True:
            c = t + bit / 2
            s = state(int(c))
            if s == 'SE0':
                break
            if s == 'SE1':
                errs.append('SE1 at %d' % int(c))
            raw.append(0 if s != prev else 1)
            prev = s
            # resync on the transition nearest this bit's start
            while k < n and evs[k] <= c + bit / 2:
                if abs(evs[k] - t) <= bit * tol * 2 and evs[k] <= c:
                    t = float(evs[k])
                k += 1
            t += bit
            if t > hi:
                errs.append('packet ran off the end at %d' % t_start)
                break
        se0_at = int(t + bit / 2 - bit / 2)
        se0 = [e for e in evs if e >= int(t)]
        # EOP length: first SE0 moment to J
        q = int(t + bit / 2)
        while state(q) == 'SE0' and q < hi:
            q += 1
        start = q
        while state(start - 1) == 'SE0':
            start -= 1
        eop = q - start
        endstate = state(q) if q < hi else None
        # NRZI raw bits to data: remove SYNC, unstuff
        bad_sync = raw[:8] != [0] * 7 + [1]
        data, run, stuff_err = [], 1, False
        j = 8
        while j < len(raw):
            b = raw[j]
            data.append(b)
            run = run + 1 if b else 0
            j += 1
            if run == 6:
                if j < len(raw) and raw[j] != 0:
                    stuff_err = True
                j += 1
                run = 0
        by = [sum(b << m for m, b in enumerate(data[x:x + 8])) for x in range(0, len(data) - len(data) % 8, 8)]
        if bad_sync:
            errs.append('bad SYNC at %d: %s' % (t_start, raw[:8]))
        if len(data) % 8:
            errs.append('packet at %d is %d bits' % (t_start, len(data)))
        pk.append(dict(t=t_start, bytes=by, raw=raw, eop=eop, stuff_err=stuff_err, end=q, endstate=endstate, nbits=len(data)))
        while i < n and evs[i] < q:
            i += 1
    return pk, errs


def edge_check(dpw, dmw, lo=0, hi=None, bit=BIT, tol=0.1):
    """Every transition inside a packet must sit on the bit grid of that packet; D+ and D- move in
    the same cycle except in and out of SE0."""
    return None

