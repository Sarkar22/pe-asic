"""Classical CAN 2.0A wire layer, from the Bosch 2.0 spec, written before any firmware.

Dominant is 0. SOF, 11-bit ID, RTR, IDE, r0, DLC(4), data, CRC-15 (0x4599), all stuffed with a
complement bit after five equal bits; then CRC delimiter (1), ACK slot, ACK delimiter (1), EOF (7 ones),
intermission (3 ones). Nothing here knows any program.
"""
from protolib import crc15_can


def unstuffed(ident, data, rtr=0):
    b = [0]
    b += [(ident >> (10 - i)) & 1 for i in range(11)]
    b += [rtr, 0, 0]
    n = len(data)
    b += [(n >> (3 - i)) & 1 for i in range(4)]
    for x in data:
        b += [(x >> (7 - i)) & 1 for i in range(8)]
    return b


def with_crc(ident, data):
    b = unstuffed(ident, data)
    c = crc15_can(b)
    return b + [(c >> (14 - i)) & 1 for i in range(15)], c


def stuff(bits):
    out, run, last = [], 0, None
    for x in bits:
        out.append(x)
        run = run + 1 if x == last else 1
        last = x
        if run == 5:
            s = 1 - x
            out.append(s)
            last, run = s, 1
    return out


def frame_bits(ident, data, flip_crc=False):
    """bus bits for a transmitter, ACK slot recessive"""
    b, c = with_crc(ident, data)
    if flip_crc:
        b[-3] ^= 1
    return stuff(b) + [1, 1, 1] + [1] * 7


class Trouble(Exception):
    pass


def decode(w, bit, lo=0, hi=None, sp=0.72):
    """Pad level in. Hard sync on the SOF edge, resync on recessive-to-dominant edges, sample at sp.
    Returns frames: dict(t, id, dlc, data, crc_ok, ack, eof_ok, errs, stuffbits)."""
    hi = hi or w.end
    frames, t_start = [], lo
    falls = [t for t, v in w.edges(lo, hi) if v == 0]
    next_ok = lo
    for f in falls:
        if f < next_ok:
            continue
        fr = walk(w, f, bit, sp, falls, hi)
        frames.append(fr)
        next_ok = fr['end']
    return frames


def walk(w, f, bit, sp, falls, hi):
    errs = []
    t = float(f)                        # start of the current bit
    import bisect
    raw, run, last = [], 0, None
    dest = []
    stuffs = 0
    need = None
    n = 0
    state = 0
    while True:
        # resync on the first falling edge inside this bit's first part, as a real node would
        i = bisect.bisect_left(falls, t - bit * 0.3)
        if i < len(falls) and falls[i] <= t + bit * 0.4 and falls[i] != f:
            t = float(falls[i])
        s = int(t + sp * bit)
        if s >= hi:
            errs.append('ran off the end')
            return dict(t=f, id=None, dlc=None, data=[], crc_ok=False, ack=None, eof_ok=False, errs=errs, end=hi, stuffbits=stuffs)
        v = w.at(s)
        t += bit
        if last is not None and run == 5:
            if v == last:
                errs.append('stuff error at bit %d' % len(dest))
                return dict(t=f, id=None, dlc=None, data=[], crc_ok=False, ack=None, eof_ok=False, errs=errs, end=int(t), stuffbits=stuffs)
            stuffs += 1
            last, run = v, 1
            if need and len(dest) == need:      # the stuff bit after the CRC sequence's last bit
                break
            continue
        run = run + 1 if v == last else 1
        last = v
        dest.append(v)
        if len(dest) == 19:
            dlc = sum(b << (3 - k) for k, b in enumerate(dest[15:19]))
            need = 19 + 8 * min(dlc, 8) + 15
        if need and len(dest) == need and run < 5:  # the CRC sequence is stuffed too: five equal bits at its end
            break                                   # are followed by a stuff bit (Bosch CAN 2.0 part A, 5)
    ident = sum(b << (10 - k) for k, b in enumerate(dest[1:12]))
    data = [sum(b << (7 - k) for k, b in enumerate(dest[19 + 8 * j:27 + 8 * j])) for j in range(min(dlc, 8))]
    crc = sum(b << (14 - k) for k, b in enumerate(dest[-15:]))
    ok = crc15_can(dest[:-15]) == crc
    tail = []
    for _ in range(10):
        s = int(t + sp * bit)
        tail.append(w.at(s) if s < hi else None)
        t += bit
    if tail[0] != 1:
        errs.append('CRC delimiter not recessive')
    ack = tail[1] == 0
    if tail[2] != 1:
        errs.append('ACK delimiter not recessive')
    eof = tail[3:10] == [1] * 7
    return dict(t=f, id=ident, dlc=dlc, data=data, crc_ok=ok, ack=ack, eof_ok=eof, errs=errs, end=int(t), stuffbits=stuffs)

