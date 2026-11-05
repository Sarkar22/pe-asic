"""Device models for the benches built on simple.py. Each sees only pad levels through the Board."""
from simple import set_ui


class UartSource:
    """Drives ui[bit]. With window=w, data bits are correct only within +-w cycles of mid-bit
    and inverted elsewhere, so a read that lands away from mid-bit returns the wrong byte."""

    def __init__(self, bit, period, data, start, window=None, gap=20):
        self.bit, self.p, self.window, self.frames = bit, period, window, []
        t = start
        for byte in data:
            self.frames.append((t, byte))
            t += 10 * period + gap * period // 10
        self.end = t

    def step(self, c, b):
        v = 1
        for t, byte in self.frames:
            if t <= c < t + 10 * self.p:
                i, off = (c - t) // self.p, (c - t) % self.p
                if i == 0:
                    v = 0
                elif i <= 8:
                    v = (byte >> (i - 1)) & 1
                    if self.window is not None and abs(off - self.p // 2) > self.window:
                        v ^= 1
                break
        set_ui(b, self.bit, v)


class Force:
    """Overrides ui[bit] over [start, end). Must come after the source in board.devs."""

    def __init__(self, bit, start, end, level):
        self.bit, self.start, self.end, self.level = bit, start, end, level

    def step(self, c, b):
        if self.start <= c < self.end:
            set_ui(b, self.bit, self.level)


class SpiTarget:
    """Spec-driven mode logic. tco = cycles from SCK or CS at the pad to MISO at the pad."""

    def __init__(self, cpol, cpha, reply, sck=1, mosi=2, cs=5, miso=4, tco=0):
        self.cpol, self.cpha, self.reply = cpol, cpha, reply
        self.sck, self.mosi, self.cs, self.miso, self.tco = sck, mosi, cs, miso, tco
        self.got, self.rx, self.n, self.tx, self.prev, self.prevcs = [], 0, 0, 0, cpol, 1
        self.pipe = []

    def present(self, c):
        v = (self.tx >> 7) & 1
        self.tx = (self.tx << 1) & 0xFF
        self.pipe.append((c + self.tco, v))

    def step(self, c, b):
        sck, mosi, cs = (b.uo >> self.sck) & 1, (b.uo >> self.mosi) & 1, (b.uo >> self.cs) & 1
        if self.prevcs and not cs:
            self.tx, self.n, self.rx, self.prev = self.reply, 0, 0, sck
            if not self.cpha:
                self.present(c)
        elif not cs and sck != self.prev:
            leading = sck != self.cpol
            if leading == (self.cpha == 0):                 # sample edge
                self.rx = (self.rx << 1 | mosi) & 0xFF
                self.n += 1
                if self.n == 8:
                    self.got.append(self.rx)
                    self.n = 0
            else:
                self.present(c)
            self.prev = sck
        elif cs:
            self.prev = sck
        self.prevcs = cs
        while self.pipe and self.pipe[0][0] <= c:
            set_ui(b, self.miso, self.pipe.pop(0)[1])


class I2cTarget:
    """7-bit target. stretch = {bit_index: cycles}: hold SCL low after that bit's falling edge
    (bit 0 is the first SCL fall after START). ack = which bytes get an ACK."""

    def __init__(self, sda=0, scl=1, stretch=None, ack=(True, True)):
        self.sda, self.scl, self.stretch, self.ack = sda, scl, stretch or {}, ack
        self.prev, self.state = None, 'idle'
        self.bytes, self.sh, self.n, self.fall_count, self.hold_until = [], 0, 0, 0, 0
        self.starts = self.stops = 0

    def lvl(self, b, pin):
        return (b.io >> pin) & 1

    def pull(self, b, pin, low):
        b.io_low = (b.io_low | 1 << pin) if low else (b.io_low & ~(1 << pin))

    def step(self, c, b):
        sda, scl = self.lvl(b, self.sda), self.lvl(b, self.scl)
        if self.prev is None:
            self.prev = (sda, scl)
        psda, pscl = self.prev
        if self.hold_until and c >= self.hold_until:
            self.pull(b, self.scl, False)
            self.hold_until = 0
        if pscl and scl and psda and not sda:
            self.starts += 1
            self.state, self.sh, self.n, self.fall_count = 'data', 0, 0, 0
        elif pscl and scl and sda and not psda:
            self.stops += 1
            self.state = 'idle'
            self.pull(b, self.sda, False)
        elif self.state in ('data', 'ack') and not pscl and scl and self.state == 'data':
            self.sh = (self.sh << 1 | sda) & 0xFF
            self.n += 1
        elif self.state in ('data', 'ack') and pscl and not scl:
            k = self.fall_count
            self.fall_count += 1
            if k in self.stretch:
                self.pull(b, self.scl, True)
                self.hold_until = c + self.stretch[k]
            if k >= 1 and self.state == 'ack':
                self.pull(b, self.sda, False)
                self.state, self.n = 'data', 0
            elif self.n == 8:
                self.bytes.append(self.sh)
                give = self.ack[min(len(self.bytes) - 1, len(self.ack) - 1)]
                self.pull(b, self.sda, give)
                self.state = 'ack'
        self.prev = (sda, scl)


class OneWireSlave:
    def __init__(self, pin=2, present=True):
        self.pin, self.present = pin, present
        self.prev, self.fall, self.byte, self.n, self.got = 1, None, 0, 0, []
        self.pres_at = self.pres_end = None
        self.resets, self.sample_at = 0, None

    def step(self, c, b):
        lvl = (b.io >> self.pin) & 1
        own_low = (b.io_low >> self.pin) & 1
        if self.pres_at and c == self.pres_at:
            b.io_low |= 1 << self.pin
        if self.pres_end and c == self.pres_end:
            b.io_low &= ~(1 << self.pin)
            self.pres_at = self.pres_end = None
        if self.prev and not lvl and not own_low:
            self.fall, self.sample_at = c, c + 1500   # read 30 us after the fall
        if not self.prev and lvl and self.fall is not None:
            if c - self.fall >= 23500:           # 470 us, the part's minimum
                self.resets += 1
                self.n, self.byte = 0, 0
                if self.present:
                    self.pres_at, self.pres_end = c + 1500, c + 7500  # 30 us, 120 us long
            self.fall = None
        if c == self.sample_at:
            self.byte |= lvl << self.n
            self.n += 1
            if self.n == 8:
                self.got.append(self.byte)
                self.n, self.byte = 0, 0
        self.prev = lvl
