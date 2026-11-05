"""DMX512 (ANSI E1.11 numbers: transmit break >= 92 us, MAB >= 12 us, receive break >= 88 us,
MAB >= 8 us, 250 kbaud 8N2, up to 512 slots, frame period >= 1204 us if the full 512 are sent)."""
import random
from protolib import *  # noqa: F401,F403
PROGRAMS = ('advanced/dmx_tx.s', 'advanced/dmx_rx.s')


def dmx_decode(w, lo=0, hi=None):
    """Pad level in. Returns (frames, errors, measured). A frame is dict(start, slots, brk, mab).
    Works from edges only: walks characters at 250 kbaud sampling at bit centres."""
    hi = hi or w.end
    bit = CLK / 250000
    errs, frames, m = [], [], dict(brk=[], mab=[], interslot=[])
    falls = [t for t, v in w.edges(lo, hi) if v == 0]
    rises = [t for t, v in w.edges(lo, hi) if v == 1]
    cur = None
    t = next((f for f in falls), None)
    i = 0
    next_ok = 0
    for f in falls:
        if f < next_ok:
            continue
        r = next((x for x in rises if x > f), None)
        if r is None:
            errs.append('line ends low at %d' % f)
            break
        low = r - f
        if low >= 88 * 50:              # a break
            if low < 92 * 50:
                errs.append('break %.1f us at %d, below 92' % (low / 50, f))
            if cur is not None:
                frames.append(cur)
            nf = next((x for x in falls if x > r), None)
            mab = (nf - r) if nf else hi - r
            m['brk'].append(low)
            m['mab'].append(mab)
            if mab < 12 * 50:
                errs.append('MAB %.1f us at %d, below 12' % (mab / 50, r))
            cur = dict(start=None, slots=[], brk=low, mab=mab, t=f)
            next_ok = r
            continue
        s = [w.at(int(f + (k + 0.5) * bit)) for k in range(11)]
        if s[0] != 0 or s[9] != 1 or s[10] != 1:
            errs.append('bad framing at %d: %s' % (f, s))
            next_ok = f + 1
            continue
        for e, _ in w.edges(f + 1, int(f + 9.5 * bit)):
            k = (e - f) / bit
            if abs(k - round(k)) > 0.02:
                errs.append('edge off the grid at %d' % e)
        v = sum(b << k for k, b in enumerate(s[1:9]))
        if cur is None:
            errs.append('character before a break at %d' % f)
        elif cur['start'] is None:
            cur['start'] = v
        else:
            cur['slots'].append(v)
        next_ok = f + 10.5 * bit
    if cur is not None:
        frames.append(cur)
    return frames, errs, m


def tx_case(frames, thread=0, align=0, slow=0):
    img = assemble(src('dmx_tx'), depth=128)
    r = Rig()
    r.load(img, {thread: 'start'})

    def script():
        for f in frames:
            for i, v in enumerate(f):
                if slow and i % 7 == 3:
                    yield slow
                yield from r.host.post(0, v | (0x100 if i == len(f) - 1 else 0))
    r.board.agent(script())
    r.start(align)
    r.run(1600 * 50 * len(frames) + 3000000, stop=r.agents_done)
    r.run(9000)
    return r, img


def rx_schedule(frames, rng, brk=4400, mab=400, t_start=2000, bit=200.0, stop=2.0, mbb=0.0, mbreak=1000, err=0.0):
    bit = bit * (1 + err)
    ev, t = [], float(t_start)

    def lvl(v):
        ev.append((int(round(t)), v))
    for f in frames:
        lvl(0)
        t += brk
        lvl(1)
        t += mab
        for x in f:
            bits = [0] + [(x >> i) & 1 for i in range(8)]
            for b in bits:
                lvl(b)
                t += bit
            lvl(1)
            t += stop * bit + (mbb() if callable(mbb) else mbb)
        t += mbreak
    return ev, t


def rx_case(frames, first, count, thread=0, depth=128, **kw):
    rng = random.Random(1)
    ev, t = rx_schedule(frames, rng, **kw)
    snd = Sched(4, ev, 1)
    img = assemble(src('dmx_rx'), depth=depth)
    r = Rig(devices=[snd], depth=depth)
    r.load(img, {thread: 'start'})
    got = []

    def script():
        yield from r.host.post(7, first)
        yield from r.host.post(6, count)
        while True:
            yield from r.host.take(1, got, tries=1)
    r.board.agent(script())
    r.start()
    r.run(int(t) + 8000)
    return [w for _, w in got], r, img


def main(log):
    log.group('advanced/dmx_tx.s', title='DMX512 transmitter')
    rng = random.Random(7)
    uni = [0] + [rng.randrange(256) for _ in range(512)]
    r, img = tx_case([uni])
    log.line('INFO  dmx_tx is %d words' % img.size)
    fr, er, m = dmx_decode(r.wave('uo', 5))
    log.check('TX full 512-slot universe, decoded from the pad', len(fr) == 1 and fr[0]['start'] == 0 and fr[0]['slots'] == uni[1:] and not er and not r.health(),
              'slots %s, break %s us, MAB %s us, errs %s %s' % (len(fr[0]['slots']) if fr else None, head([x / 50 for x in m['brk']]),
                                                                 head([x / 50 for x in m['mab']]), er[:2], r.health()))
    short = [[0, 1, 2, 3, 4, 5], [0x17, 9, 8], [0] + list(range(64))]
    short16 = [[0, 1, 2, 3, 4, 5], [0x17, 9, 8], [0] + list(range(16))]
    ok = 0
    for th in range(4):
        for al in range(4):
            r, img = tx_case(short16, th, al)
            fr, er, m = dmx_decode(r.wave('uo', 5))
            ok += [([f['start']] + f['slots']) for f in fr] == short16 and not er and not r.health()
    log.check('TX 3 short frames, one alternate start code, 4 threads x 4 alignments', ok == 16, '%d of 16' % ok)
    r, img = tx_case(short, slow=70000)
    fr, er, m = dmx_decode(r.wave('uo', 5))
    log.check('TX with a host that stalls 1.4 ms mid-frame (stale D path)', [([f['start']] + f['slots']) for f in fr] == short and not er and not r.health(), '%s %s' % (er[:2], r.health()))

    log.group('advanced/dmx_rx.s', title='DMX512 receiver')
    img = assemble(src('dmx_rx'), depth=128)
    log.line('INFO  dmx_rx is %d words' % img.size)
    u = [0] + [rng.randrange(256) for _ in range(512)]
    for first, cnt in ((1, 4), (509, 4), (250, 8)):
        w, r, img = rx_case([u], first, cnt)
        log.check('RX 512 slots, window %d..%d' % (first, first + cnt - 1), w == u[first:first + cnt] and not r.health(), '%s %s' % ([hex(x) for x in w[:8]], r.health()))
    cases = [('shortest legal timing: break 88 us, MAB 8 us', dict(brk=4400, mab=400)),
             ('long break 1 ms, MAB 100 us', dict(brk=50000, mab=5000)),
             ('sender +2%', dict(err=0.02)), ('sender -2%', dict(err=-0.02)),
             ('mark between slots up to 1 ms', dict(mbb=lambda: rng.choice((0, 0, 400.0, 2000.0, 50000.0))))]
    for name, kw in cases:
        f = [[0] + [rng.randrange(256) for _ in range(24)] for _ in range(3)]
        want = sum((x[3:7] for x in f), [])
        w, r, img = rx_case(f, 3, 4, **kw)
        log.check('RX ' + name + ', 3 frames', w == want and not r.health(), '%s %s' % (len(w), r.health()))
    f = [[0] + [1] * 10, [0x55] + [2] * 10, [0] + [3] * 10]
    w, r, img = rx_case(f, 2, 2)
    log.check('RX ignores a non-zero start code', w == [1, 1, 3, 3], w)
    w, r, img = rx_case([[0] + [7] * 3, [0] + [8] * 3], 2, 8)
    log.check('RX frame shorter than the window, next frame still found', w == [7, 7, 8, 8], w)

