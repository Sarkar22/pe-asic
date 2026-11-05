"""A small per-cycle bench on sim.Chip, used by the benches of standard/, trimmed/ and extra/.

Chip wraps sim.Chip with start/post/take/run and calls a board each cycle as
board.step(chip, cycle, uo, uio_oe, uio_out), which returns (ui, uio_in). The board sees the pads
before the cycle, and the bypass lanes follow the ui of the cycle before (section 10 allows that lag).
Board is a wired-AND uio bus with pull-ups that logs every pad change; device models see only
pad levels. Run is the same thing for one program and one callback.
"""
from report import Chip as SimChip
from report import assemble, path


def read(rel):
    with open(path(rel)) as fh:
        return fh.read()


class ThreadView:
    """Per-thread flags as the benches use them. latecount counts the rises of LATE."""

    def __init__(self, th):
        self.th = th
        self.latecount = 0
        self.was = 0

    @property
    def id(self):
        return self.th.TID

    @property
    def late(self):
        return self.th.LATE

    @property
    def run(self):
        return self.th.RUN

    @property
    def r(self):
        return self.th.R


LIMIT = 10_000_000      # cycles one chip may run: the longest bench run is under 4 million


class Chip:
    def __init__(self, words=None, size=512, seed=1):
        self.sim = SimChip(size, seed)
        for a, w in (words or {}).items():
            if not self.sim.host_wrmem(a, w):
                raise RuntimeError('memory write refused')
        self.t = [ThreadView(th) for th in self.sim.th]
        self.prev_ui = 0

    @property
    def c(self):
        return self.sim.c

    @property
    def full(self):
        return self.sim.FULL

    @property
    def od(self):
        return sum(v << i for i, v in enumerate(self.sim.OD))

    def pads(self):
        return self.sim.pads(self.prev_ui)

    def cycle(self, board):
        if self.sim.c >= LIMIT:     # a bench loop that waits on a stuck program becomes a failure, not a hang
            raise RuntimeError('chip ran %d cycles: the program or the bench is stuck' % LIMIT)
        uo, uio_out, uio_oe = self.sim.pads(self.prev_ui)
        ui, uio_in = board.step(self, self.sim.c, uo, uio_oe, uio_out)
        self.prev_ui = ui
        self.sim.cycle(ui & 255, uio_in & 255)
        for v in self.t:
            late = v.th.LATE
            if late and not v.was:
                v.latecount += 1
            v.was = late

    def run(self, cycles, board):
        for _ in range(cycles):
            self.cycle(board)

    def start(self, tid, pc=0):
        """SETPC, then RUN for one thread."""
        self.sim.host_setpc(tid, pc)
        self.sim.host_run(0, 1 << tid)

    def post(self, n, v):
        """True if the mailbox word took it."""
        return self.sim.host_post(n, v) == 0

    def take(self, n):
        full, word = self.sim.host_take(n)
        return word if full else None

    def mem(self, addr):
        """The simulator's own copy of a memory word. A real host can't read it while threads run."""
        return self.sim.mem[addr % self.sim.DEPTH]


def boot(rel, tid=0, defs=None, size=128):
    """Assembles programs/<rel>, starts it on one thread at address 0, and gives it an empty board."""
    img = assemble(read(rel), defines=dict(defs) if defs else None, depth=size)
    chip = Chip(img.words, size)
    chip.start(tid)
    return chip, Board(), img.size


class Board:
    """Wired-AND uio bus with pull-ups, a log of every pad change, and device hooks."""

    def __init__(self, ui=0xFF, pullups=0xFF):
        self.ui, self.pullups = ui, pullups
        self.io_low = 0                 # bits a device is pulling low
        self.devs = []
        self.log = []                   # (cycle, 'uoN' or 'ioN', level)
        self.uo = self.io = None
        self.contention = 0
        self.chip_high_on_od = 0        # chip driving 1 on a pin in OD mode (must stay 0)

    def step(self, chip, c, uo, en, val):
        chip_low = en & ~val & 0xFF
        chip_high = en & val & 0xFF
        ext_low = self.io_low & 0xFF
        if chip_high:
            self.contention += bin(chip_high & ext_low).count('1')
            self.chip_high_on_od += bin(chip_high & chip.od).count('1')
        lvl = ~(chip_low | ext_low) & (chip_high | self.pullups) & 0xFF
        if self.uo is None:
            self.uo, self.io = uo, lvl
        for name, old, new in (('uo', self.uo, uo), ('io', self.io, lvl)):
            if old != new:
                for i in range(8):
                    if (old ^ new) >> i & 1:
                        self.log.append((c, '%s%d' % (name, i), (new >> i) & 1))
        self.uo, self.io, self.chip_en = uo, lvl, en
        for d in self.devs:
            d.step(c, self)
        return self.ui, lvl

    def edges(self, name, since=0):
        return [(c, v) for c, n, v in self.log if n == name and c >= since]


def set_ui(board, bit, v):
    board.ui = (board.ui & ~(1 << bit)) | (v & 1) << bit


class Bus:
    """Wired-AND uio with pull-ups. ext_low is a bitmask a device pulls low; ext_oe and ext_high let a
    device drive. Counts contention: the chip driving against a device."""

    def __init__(self, pull=0xFF):
        self.pull, self.ext_low, self.contention = pull, 0, 0
        self.ext_high, self.ext_oe = 0, 0

    def level(self, o, oe):
        lvl = 0
        for i in range(8):
            b = 1 << i
            chip_low = bool(oe & b and not o & b)
            chip_high = bool(oe & b and o & b)
            ext_l = bool(self.ext_low & b)
            ext_h = bool(self.ext_oe & b and self.ext_high & b)
            ext_d0 = bool(self.ext_oe & b and not self.ext_high & b)
            if (chip_high and (ext_l or ext_d0)) or (chip_low and ext_h):
                self.contention += 1
            high = not (chip_low or ext_l or ext_d0) and (chip_high or ext_h or bool(self.pull & b))
            lvl |= b if high else 0
        return lvl


class _Callback:
    def __init__(self):
        self.w = None

    def step(self, chip, c, uo, en, val):
        return self.w(c, uo, val, en)


class Run:
    """One program on thread 0, started at address 0. step(w) runs one cycle, where
    w(cycle, uo, uio_out, uio_oe) returns (ui, uio_in)."""

    def __init__(self, text, defs=None, depth=128):
        img = assemble(text, defines=dict(defs) if defs else None, depth=depth)
        self.img, self.words = img, img.size
        self.chip = Chip(img.words, depth)
        self.chip.start(0, 0)
        self.env = _Callback()

    def post(self, v):
        return self.chip.post(0, v)

    def take(self):
        return self.chip.take(1)

    def step(self, w):
        self.env.w = w
        self.chip.cycle(self.env)

    @property
    def c(self):
        return self.chip.c

    def late(self):
        return sum(t.late << t.id for t in self.chip.t)
