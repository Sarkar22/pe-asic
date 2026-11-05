"""USB LS device link layer against a host model. Host drives bit times of 33.33 cycles and listens.
Device traffic and timing are checked from the pads with usb_wire.decode."""
import random
from protolib import *  # noqa: F401,F403
PROGRAMS = ('advanced/usb_dev.s',)
import usb_wire as U
DP, DM = 0, 1
ADDR = 0x15


class UHost(Device):
    def __init__(self, chip, script, bit=U.BIT):
        self.chip, self.bit = chip, bit
        self.gen = script(self)
        self.drv = None                 # (dp, dm) when driving, None when released
        self.dp, self.dm = 0, 1
        self.io_low = 1 << DP
        self.c = 0
        self.turn = []
        self.quiet = []
        self.done = False

    def step(self, c, uo, io):
        self.c = c
        self.dp, self.dm = io & 1, (io >> 1) & 1
        if self.gen is not None:
            try:
                next(self.gen)
            except StopIteration:
                self.gen = None
                self.done = True
        if self.drv is not None:
            self.io_low = ((1 - self.drv[0]) << DP) | ((1 - self.drv[1]) << DM)
        else:                           # 15k pull-down on D+, pull-up on D-: the device wins when it drives
            self.io_low = (0 if self.chip.OE[DP] else 1 << DP)

    def wait(self, n):
        for _ in range(int(n)):
            yield

    def send(self, packet):
        ev, end = U.schedule(packet, self.c + 1, self.bit)
        self.drv = (0, 1)
        i = 0
        while self.c < end or i < len(ev):
            while i < len(ev) and ev[i][0] <= self.c + 1:
                self.drv = (ev[i][1], ev[i][2])
                i += 1
            yield
        self.drv = None
        self.eop_end = self.c

    def expect(self, limit_bits=12):
        """waits for a K from the device. Records turnaround from our EOP's end, in bit times."""
        t_start = self.eop_end
        while not self.dp:
            if self.c - t_start > limit_bits * self.bit:
                self.turn.append(None)
                return False
            yield
        self.turn.append((self.c - t_start) / self.bit)
        while self.dp or self.dm:       # device's EOP
            yield
        while not self.dm:
            yield
        yield from self.wait(self.bit * 2)
        return True


def case(script, posts=None, align=0, depth=512, packet=None, run=None):
    bus = {}

    class Late:
        pass
    # the host needs the chip, so build the rig then add the device
    r = Rig(pullups=3, depth=depth)
    h = UHost(r.chip, script)
    r.board.devices.append(h)
    img = assemble(src('usb_dev'), depth=depth)
    r.load(img, {0: 'bit', 1: 'byt', 2: 'wire'})
    got = []
    if packet is not None:
        words = [0x80 | (packet[0] << 8)]
        rest = packet[1:]
        for i in range(0, len(rest), 2):
            words.append(rest[i] | ((rest[i + 1] if i + 1 < len(rest) else 0) << 8))
        for i, w in enumerate(words):
            r.chip.host_wrmem(202 + i, w)

    def agent():
        if packet is not None:
            yield from r.host.post(4, ((8 + 8 * len(packet)) << 8) | 202)
        while True:
            yield from r.host.take(6, got, tries=1)
    r.board.agent(agent())
    r.start(align)
    r.run(run or 40000, stop=lambda: h.done)
    r.run(1500)
    return r, h, [w for _, w in got], img


def main(log):
    log.group('advanced/usb_dev.s', title='USB low-speed device link')
    rng = random.Random(9)
    payload = [rng.randrange(256) for _ in range(8)]

    def s_setup(h):
        yield from h.wait(1500)
        yield from h.send(U.token('SETUP', ADDR, 0))
        yield from h.wait(3 * h.bit)
        yield from h.send(U.data_packet('DATA0', payload))
        h.ok = yield from h.expect()
    r, h, w, img = case(s_setup)
    log.line('INFO  usb_dev is %d words' % img.size)
    pk, er = U.decode(r.wave('io', DP), r.wave('io', DM), bit=33)
    want = list(payload) + [(U.crc16_usb(bytes_lsb(payload))) & 255, U.crc16_usb(bytes_lsb(payload)) >> 8, 0x100]
    log.check('SETUP + DATA0: bytes reach the host side, CRC accepted, device ACKs', w == want and [p['bytes'] for p in pk][-1:] == [[U.PID['ACK']]] and not r.health(), ([hex(x) for x in w[-3:]], [[hex(b) for b in p['bytes'][:2]] for p in pk], r.health()))
    log.line('INFO  turnaround, end of our EOP to the device first K: %s bit times, limit 6.5' % (['%.2f' % x if x else x for x in h.turn]))

    def s_in_nak(h):
        yield from h.wait(1500)
        yield from h.send(U.token('IN', ADDR, 0))
        h.ok = yield from h.expect()
    r, h, w, img = case(s_in_nak)
    pk, er = U.decode(r.wave('io', DP), r.wave('io', DM), bit=33)
    log.check('IN with nothing ready: NAK', [p['bytes'] for p in pk][-1:] == [[U.PID['NAK']]] and not r.health(), ([p['bytes'] for p in pk], h.turn))

    dpk = U.data_packet('DATA1', [0x12, 0x01, 0x10, 0x01, 0xFF, 0x00, 0xFF, 0x08])

    def s_in_data(h):
        yield from h.wait(4000)
        yield from h.send(U.token('IN', ADDR, 0))
        h.ok = yield from h.expect()
        yield from h.send([U.PID['ACK']])
        yield from h.wait(1000)
    r, h, w, img = case(s_in_data, packet=dpk)
    pk, er = U.decode(r.wave('io', DP), r.wave('io', DM), bit=33)
    log.check('IN with a packet ready: DATA1 with its CRC-16 on the wire, host ACK seen by the device',
              len(pk) == 3 and pk[1]['bytes'] == dpk and 0x1D2 in w and not pk[1]['stuff_err'] and not r.health(), ([hex(b) for b in pk[1]['bytes']] if len(pk) > 1 else pk, [hex(x) for x in w], h.turn, r.health()))
    log.line('INFO  turnaround for DATA1: %s bit times' % (['%.2f' % x if x else x for x in h.turn]))

    def s_other(h):
        yield from h.wait(1500)
        yield from h.send(U.token('IN', 0x16, 0))
        h.ok = yield from h.expect(limit_bits=16)
        yield from h.wait(800)
        yield from h.send(U.token('OUT', 0x14, 0))
        yield from h.wait(3 * h.bit)
        yield from h.send(U.data_packet('DATA0', [1, 2]))
        h.ok2 = yield from h.expect(limit_bits=16)
    r, h, w, img = case(s_other)
    log.check('tokens for other addresses: silent, data ignored', h.turn == [None, None] and not [x for x in w if x >= 0x100], (h.turn, [hex(x) for x in w]))

    bad = U.data_packet('DATA0', [1, 2, 3, 4])
    bad[3] ^= 1

    def s_bad(h):
        yield from h.wait(1500)
        yield from h.send(U.token('OUT', ADDR, 0))
        yield from h.wait(3 * h.bit)
        yield from h.send(bad)
        h.ok = yield from h.expect(limit_bits=14)
    r, h, w, img = case(s_bad)
    log.check('bad CRC: no ACK, flagged 0x101', h.turn == [None] and 0x101 in w, (h.turn, [hex(x) for x in w]))

    def s_seq(h):
        yield from h.wait(4000)
        for k in range(3):
            yield from h.send(U.token('SETUP', ADDR, 0))
            yield from h.wait(2.5 * h.bit)
            yield from h.send(U.data_packet('DATA0', [0xFF] * 8 if k == 1 else [k] * 8))
            yield from h.expect()
            yield from h.wait(3 * h.bit)
            yield from h.send(U.token('IN', ADDR, 0))
            yield from h.expect()
            yield from h.wait(3 * h.bit)
    good = 0
    worst = 0
    for al in range(4):
        r, h, w, img = case(s_seq, align=al, run=60000)
        pk, er = U.decode(r.wave('io', DP), r.wave('io', DM), bit=33)
        hs = [p['bytes'][0] for p in pk if len(p['bytes']) == 1]
        good += hs == [U.PID['ACK'], U.PID['NAK']] * 3 and len(h.turn) == 6 and None not in h.turn and not r.health()
        worst = max([worst] + [x for x in h.turn if x])
        turns = ['%.2f' % x for x in h.turn if x]
    log.check('three SETUP/IN rounds back to back, 4 alignments, stuffing-heavy data', good == 4, '%d of 4, worst turnaround %.2f bit times, last run %s' % (good, worst, turns))
    log.check('turnaround inside 6.5 bit times on every reply', worst <= 6.5, '%.2f' % worst)

