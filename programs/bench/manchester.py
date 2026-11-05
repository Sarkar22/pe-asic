"""Manchester RX with clock recovery. The sender knows nothing of the firmware."""
import random
from protolib import *  # noqa: F401,F403
PROGRAMS = ('advanced/manch_rx.s',)


def manch_bits(payload, pre=32):
    bits = [(i + 1) % 2 for i in range(pre)]    # 1 0 1 0 ...
    if bits[-1] == 1:
        bits.append(0)
    bits += [1, 1]
    for x in payload:
        bits += [(x >> i) & 1 for i in range(8)]
    return bits


def schedule(bits, B, t_start, jitter=0.0, seed=0, idle=0):
    """Levels: first half is the complement of the bit, second half the bit."""
    rng = random.Random(seed)
    ev, t, lvl = [], float(t_start), idle
    for b in bits:
        for half, want in ((0, 1 - b), (1, b)):
            if want != lvl:
                j = rng.uniform(-jitter, jitter) if jitter else 0
                ev.append((int(round(t + j)), want))
                lvl = want
            t += B / 2
    return sorted(ev), t


def run(B_nom, err=0.0, jitter=0.0, payload=None, phase=0, thread=0, seed=1, idle=0, frames=1):
    B = B_nom * (1 + err)
    payload = payload if payload is not None else [random.Random(seed).randrange(256) for _ in range(64)]
    ev, t = [], 400.0 + phase
    for f in range(frames):
        e, t = schedule(manch_bits(payload), B, t, jitter, seed + f, idle if not ev else ev[-1][1])
        ev += e
        t += B * 8
    snd = Sched(6, ev, idle)
    r = Rig(devices=[snd])
    img = assemble(src('manch_rx'), depth=128)
    r.load(img, {thread: 'start'})
    got = []

    def script():
        yield from r.host.post(7, round(B_nom * 3 / 4))
        yield from r.host.post(6, round(B_nom / 2))
        while True:
            yield from r.host.take(1, got, tries=1)
    r.board.agent(script())
    r.start()
    r.run(int(t) + 3000)
    words = [w for _, w in got]
    return words, payload, r, img


def good(words, payload, frames=1):
    want = (list(payload) + [0x100 + 8]) * frames
    # bits-left word is 8 when the line stops on a byte boundary
    return words == want


def main(log):
    log.group('advanced/manch_rx.s', title='Manchester receiver with clock recovery')
    w, p, r, img = run(100)
    log.line('INFO  manch_rx is %d words' % img.size)
    # Fewer bytes and phases at the slow rates, where a run is mostly waiting: the bench has 5 minutes in all.
    for B, nbytes, phases in ((5000, 1, 1), (500, 4, 8), (100, 16, 8), (60, 16, 8)):
        for th in range(4):
            ok = 0
            for ph in range(phases):
                payload = [random.Random(ph + 8 * th).randrange(256) for _ in range(nbytes)]
                w, p, r, img = run(B, phase=ph, thread=th, payload=payload)
                if good(w, p) and not r.health():
                    ok += 1
            log.check('B=%d cycles (%.1f kb/s) thread %d, %d phase%s, %d byte%s' % (
                B, 50000 / B, th, phases, 's' * (phases > 1), nbytes, 's' * (nbytes > 1)), ok == phases, '%d of %d' % (ok, phases))
    w, p, r, img = run(100, payload=[random.Random(5).randrange(256) for _ in range(600)], err=0.02)
    log.check('600 bytes, 4800 bits, sender +2%', good(w, p) and not r.health(), '%d words' % len(w))
    w, p, r, img = run(100, payload=[0] * 32 + [255] * 32)
    log.check('runs of 0x00 and 0xFF (boundary edge on every bit)', good(w, p) and not r.health())
    w, p, r, img = run(100, frames=3)
    log.check('3 frames with silence between', good(w, p, 3) and not r.health(), '%d words' % len(w))
    w, p, r, img = run(100, idle=1)
    log.check('line idles high', good(w, p) and not r.health())
