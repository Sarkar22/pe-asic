"""Shared rig for the benches of the programs in advanced/.

Firmware sees only pads and the mailbox. Device models and decoders in these benches see only pad
levels and protocol parameters, never a label or constant of a program.
"""
import report
import rig
from report import Chip, assemble                       # noqa: F401
from rig import Board, Device, Host, Trace, Wave        # noqa: F401

CLK = rig.CLK


def src(name):
    return report.src('advanced/%s.s' % name)


class Rig:
    def __init__(self, depth=128, devices=(), pullups=0, rise=0, seed=1, ui_idle=0x01):
        self.chip = Chip(depth, seed)
        self.board = Board(self.chip, devices, pullups, rise, ui_idle)
        self.board.run(8)
        self.host = Host(self.chip)
        self.depth = depth

    def load(self, img, starts):
        """starts: {thread: address or label}"""
        for a, w in sorted(img.words.items()):
            if not self.chip.host_wrmem(a, w):
                raise RuntimeError('memory write refused')
        mask = 0
        for th, pc in starts.items():
            self.chip.host_setpc(th, img.labels[pc] if isinstance(pc, str) else pc)
            mask |= 1 << th
        self.mask = mask

    def start(self, align=0):
        while (self.chip.c - 1) % 4 != align:
            self.board.run(1)
        self.chip.host_run(0, self.mask)

    def run(self, cycles, stop=None):
        self.board.run(cycles, stop)

    def agents_done(self):
        return all(a[0] is None for a in self.board.agents)

    def health(self, ignore_late=()):
        out = []
        for t in self.chip.th:
            if t.LATE and t.TID not in ignore_late:
                out.append('thread %d LATE at cycle %s' % (t.TID, t.late_at))
            if t.ILL:
                out.append('thread %d ILL at PC %d' % (t.TID, t.PC))
        out.extend('bus conflict at %d' % c for c, m in self.board.conflicts[:3])
        return out

    def wave(self, sig, n):
        return self.board.trace.wave(sig, n)


# ---------------------------------------------------------------------------
# reference CRCs, written from the catalogue parameters, anchored on published check values

def crc_generic(data_bits, width, poly, init, xorout):
    """MSB-first LFSR over a list of bits. Returns the register, xored."""
    reg, top, mask = init, 1 << (width - 1), (1 << width) - 1
    for b in data_bits:
        fb = ((reg & top) != 0) ^ bool(b)
        reg = (reg << 1) & mask
        if fb:
            reg ^= poly
    return reg ^ xorout


def bytes_lsb(data):
    return [(x >> i) & 1 for x in data for i in range(8)]


def bytes_msb(data):
    return [(x >> (7 - i)) & 1 for x in data for i in range(8)]


def rev(v, n):
    return sum(((v >> i) & 1) << (n - 1 - i) for i in range(n))


def crc5_usb(bits):
    """bits in wire order. CRC-5/USB: refin/refout true, so the register is read reversed."""
    return rev(crc_generic(bits, 5, 0x05, 0x1F, 0x1F), 5)


def crc16_usb(bits):
    return rev(crc_generic(bits, 16, 0x8005, 0xFFFF, 0xFFFF), 16)


def crc15_can(bits):
    return crc_generic(bits, 15, 0x4599, 0, 0)


def anchors():
    """Published check values for the ASCII string 123456789 (catalogue of CRC algorithms)."""
    msg = b'123456789'
    return [
        ('CRC-5/USB', crc5_usb(bytes_lsb(msg)), 0x19),
        ('CRC-16/USB', crc16_usb(bytes_lsb(msg)), 0xB4C8),
        ('CRC-15/CAN', crc15_can(bytes_msb(msg)), 0x059E),
    ]


class Sched(Device):
    """Drives one ui pin from a sorted list of (cycle, level)."""

    def __init__(self, pin, sched, idle=1):
        self.ui_mask = 1 << pin
        self.ui_val = self.ui_mask * idle
        self.pin = pin
        self.sched = sched
        self.i = 0

    def step(self, c, uo, io):
        s = self.sched
        while self.i < len(s) and s[self.i][0] <= c + 1:
            self.ui_val = s[self.i][1] << self.pin
            self.i += 1


def span(x):
    """'lo..hi' of the values in x, or '-' if there are none, so that a detail can't crash on a broken
    program."""
    return '%s..%s' % (min(x), max(x)) if x else '-'


def head(x):
    return x[0] if x else None


def late_site(img, r, cycles, tid=0):
    """Runs until the thread's LATE flag rises, then names the source line it stopped at."""
    r.run(cycles, stop=lambda: r.chip.th[tid].LATE)
    pc = r.chip.th[tid].PC
    near = [(a, t) for a, w, t in img.listing if pc - 3 <= a <= pc + 1 and t]
    return r.chip.c, pc, near
