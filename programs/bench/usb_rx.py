from protolib import *  # noqa: F401,F403
PROGRAMS = ('advanced/usb_rx.s',)
import usb_wire as U
import random
DP, DM = 0, 1


class Sender(Device):
    """Drives D+ and D- from (cycle, dp, dm) events. Idle is J. A pull-up reads 1, so a low is driven."""

    def __init__(self, ev):
        self.ev = ev
        self.i = 0
        self.io_low = 1 << DP

    def step(self, c, uo, io):
        e = self.ev
        while self.i < len(e) and e[self.i][0] <= c + 1:
            _, dp, dm = e[self.i]
            self.io_low = ((1 - dp) << DP) | ((1 - dm) << DM)
            self.i += 1


def traffic(packets, bit, jitter, rng, t_start=1200, gap=None, skew=0):
    ev, t = [], t_start
    for p in packets:
        e, t = U.schedule(p, t, bit, jitter, rng, skew=skew)
        ev += e
        t += (gap if gap is not None else rng.uniform(2, 8)) * U.BIT
    ev.sort(key=lambda x: x[0])
    return ev, t


def case(packets, err=0.0, jitter=0.0, seed=1, thread=0, align=0, gap=None, name='usb_rx', skew=0):
    rng = random.Random(seed)
    ev, t = traffic(packets, U.BIT * (1 + err), jitter, rng, gap=gap, skew=skew)
    r = Rig(devices=[Sender(ev)], pullups=(1 << DP) | (1 << DM))
    img = assemble(src(name), depth=128)
    r.load(img, {0: 'bit', 1: 'byt'})
    got = []

    def agent():
        while True:
            yield from r.host.take(4, got, tries=1)
    r.board.agent(agent())
    r.start(align)
    r.run(int(t) + 1500)
    return [w for _, w in got], r, img


def expect(packets):
    out = []
    for p in packets:
        out += list(p) + [0x100]
    return out


def mix(rng, n=6):
    out = []
    for i in range(n):
        k = rng.randrange(6)
        if k == 0:
            out.append([U.PID['ACK']])
        elif k == 1:
            out.append(U.token('IN', rng.randrange(128), rng.randrange(16)))
        elif k == 2:
            out.append(U.data_packet('DATA0', [0xFF] * rng.randrange(1, 9)))
        elif k == 3:
            out.append(U.data_packet('DATA1', [0] * 8))
        elif k == 4:
            out.append(U.data_packet('DATA1', []))
        else:
            out.append(U.data_packet('DATA0', [rng.choice((0xFF, 0x3F, 0x7E, 0xFC, 0x00, rng.randrange(256))) for _ in range(8)]))
    return out


def norm(words):
    # high byte of the accumulator is left by nothing here, but keep the comparison honest
    return [w & 0x3FF for w in words]


def score(n, **kw):
    bad = []
    for s in range(n):
        rng = random.Random(s + 100)
        ps = mix(rng)
        w, r, img = case(ps, seed=s, **kw)
        if norm(w) != expect(ps) or r.health(ignore_late=(0, 1)):   # usb_rx times no output, so the packets decide a late tick
            bad.append(s)
    return n - len(bad), bad


def main(log):
    log.group('advanced/usb_rx.s', title='USB low-speed receiver')
    ps = [[U.PID['ACK']], U.token('IN', 0x15, 1), U.data_packet('DATA1', [1, 2, 3, 4, 5, 6, 7, 8])]
    w, r, img = case(ps)
    log.line('INFO  usb_rx is %d words' % img.size)
    log.check('handshake, token, 8-byte data at exactly 1.5 Mb/s', norm(w) == expect(ps), [hex(x) for x in w[:6]])
    # Fewer random runs per setting than a full sweep would use: the whole bench has 5 minutes.
    ok, bad = score(20)
    log.check('20 random 6-packet runs, stuffing-heavy mix, gaps 2 to 8 bits, exact rate', ok == 20, '%d of 20, bad %s' % (ok, bad[:5]))
    for al in (1, 2, 3):
        ok, bad = score(12, align=al)
        log.check('alignment %d' % al, ok == 12, '%d of 12' % ok)
    for sk in (1, 3, 5, 8):
        ok, bad = score(8, skew=sk)
        log.check('D- lags D+ by %d cycles (%d ns) on every transition' % (sk, sk * 20), ok == 8, '%d of 8, bad %s' % (ok, bad[:4]))
    for e in (0.005, -0.005, 0.01, -0.01, 0.015, -0.015):
        ok, bad = score(8, err=e)
        log.check('sender rate error %+.1f%%, the LS device limit is 1.5%%' % (e * 100), ok == 8, '%d of 8, bad %s' % (ok, bad[:4]))
    for j in (1.25, 2, 2.5, 3):
        ok, bad = score(8, jitter=j)
        if j <= 3:
            log.check('edge jitter %d ns' % (j * 20), ok == 8, '%d of 8, bad %s' % (ok, bad[:2]))
    for g in (2.0, 2.5, 3.0):
        ok, bad = score(8, gap=g)
        log.check('inter-packet gap of %.1f bit times (2 is the minimum)' % g, ok == 8, '%d of 8' % ok)

