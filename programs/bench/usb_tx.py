from protolib import *  # noqa: F401,F403
PROGRAMS = ('advanced/usb_tx.s',)
import usb_wire as U
import random
DP, DM = 0, 1


def load_packet(r, packet):
    """SYNC plus PID word, then packed data, as the transmitter wants them."""
    words = [0x80 | (packet[0] << 8)]
    rest = packet[1:]
    for i in range(0, len(rest), 2):
        lo = rest[i]
        hi = rest[i + 1] if i + 1 < len(rest) else 0
        words.append(lo | (hi << 8))
    for i, w in enumerate(words):
        r.chip.host_wrmem(96 + i, w)
    return 8 + 8 * len(packet)


def case(packets, step=33, thread=0, align=0, depth=128):
    r = Rig(pullups=(1 << DM), depth=depth)
    img = assemble(src('usb_tx'), depth=depth, defines={'STEP': step, 'PKT': 96})
    r.load(img, {thread: 'start'})
    done = []

    def agent():
        for p in packets:
            n = load_packet(r, p)
            yield from r.host.post(7, n)
            yield from r.host.take(1, done)
            yield 400
    r.board.agent(agent())
    r.start(align)
    r.run(sum(len(p) for p in packets) * 12 * 60 + 8000 * len(packets) + 8000, stop=r.agents_done)
    r.run(200)
    return r, img


def waves(r):
    return r.wave('io', DP), r.wave('io', DM)


def main(log):
    log.group('advanced/usb_tx.s', title='USB low-speed transmitter')
    rng = random.Random(3)
    cases = [('handshake ACK', [U.PID['ACK']]),
             ('DATA1, 8 bytes, random', U.data_packet('DATA1', [rng.randrange(256) for _ in range(8)])),
             ('DATA0, 8 bytes 0xFF, heaviest stuffing', U.data_packet('DATA0', [0xFF] * 8)),
             ('DATA0, 8 bytes 0x00, a toggle on every bit', U.data_packet('DATA0', [0] * 8)),
             ('DATA1, zero length', U.data_packet('DATA1', [])),
             ('DATA1, 7 bytes, odd length', U.data_packet('DATA1', [1, 2, 3, 4, 5, 6, 7])),
             ('token IN', U.token('IN', 0x15, 1))]
    for name, p in cases:
        r, img = case([p])
        pk, er = U.decode(*waves(r), bit=33)
        ok = len(pk) == 1 and pk[0]['bytes'] == list(p) and not pk[0]['stuff_err'] and not er and not r.health() and pk[0]['eop'] in (65, 66, 67) and pk[0]['endstate'] == 'J'
        log.check(name, ok, '%s eop %s %s %s' % ([hex(x) for x in pk[0]['bytes']][:4] if pk else None, pk[0]['eop'] if pk else None, er[:2], r.health()))
    log.line('INFO  usb_tx is %d words' % img.size)
    ok = 0
    n = 0
    for th in range(4):
        for al in range(4):
            for s in range(3):
                p = U.data_packet('DATA1', [rng.choice((0xFF, 0xFF, 0x3F, 0x7E, 0xFC, rng.randrange(256))) for _ in range(8)])
                r, img = case([p], thread=th, align=al)
                pk, er = U.decode(*waves(r), bit=33)
                n += 1
                ok += len(pk) == 1 and pk[0]['bytes'] == list(p) and not er and not r.health()
    log.check('stuffing-heavy packets, 4 threads x 4 alignments x 3', ok == n, '%d of %d' % (ok, n))
    # grid exactness: every transition on an integer multiple of the step from the first K
    p = U.data_packet('DATA0', [0xFF, 0x0F, 0xF0, 0xAA, 0x55, 0x00, 0x7E, 0x81])
    r, img = case([p])
    dpw, dmw = waves(r)
    ed = dpw.edges()
    t_start = ed[0][0]
    off = sorted(set((t - t_start) % 33 for t, v in ed))
    log.check('every D+ edge on the 33-cycle grid, no jitter at all', off == [0], off)
