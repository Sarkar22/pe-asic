# SPDX-License-Identifier: Apache-2.0
"""Host side of the chip's SPI link: every command of ISA section 9.2, and loading programs.

The same file runs under MicroPython on the demo board's RP2350 and under CPython. It never touches hardware: it
talks to a transport with two methods.

    xfer(out)      One transaction, and returns the bytes read: CS low, len(out) bytes out on MOSI in mode 0, MSB
                   first, the same number in from MISO, CS high, and then CS stays high for at least the 3 cycles
                   of section 9.1 before the next transaction can start.
    pause_us(us)   Lets at least us microseconds pass. Polling waits this long between tries.

board.py has the demo board's transport, and test/test.py has one that drives the chip's pins in simulation.
"""

VERSION = 0x01          # CAPS: ISA version 0.1
THREADS = 4
LOG2_DEPTH = 9
DEPTH = 1 << LOG2_DEPTH

# Command codes, section 9.2
CAPS = 0x00
WRMEM = 0x01
RDMEM = 0x02
RUN = 0x10
SETPC = 0x11
STATUS = 0x12
STEP = 0x13
RDREG = 0x14
POST = 0x20             # + mailbox word
TAKE = 0x28             # + mailbox word
ABORT = 0x3F

# RDREG reg8, section 9.2. 16 + k is configuration slot k, also by its name in section 5.9.2.
SLOTS = ('TXMODE', 'TXMAP', 'TXPER', 'TXTAIL', 'RXMODE', 'RXPER', 'RXFIRST', 'RXEND', 'RXCNT', 'RXSYNC', 'RXOPT',
         'POLYHI', 'POLYLO', 'CHKHI', 'CHKLO', 'CODING')
REGS = {'D': 8, 'S': 9, 'PC': 10, 'FLAGS': 11, 'SSTAT': 12, 'CRC_LO': 13, 'CRC_HI': 14, 'ENDF': 15}
for _k in range(8):
    REGS['R%d' % _k] = _k
for _k in range(16):
    REGS['CFG%d' % _k] = 16 + _k
    REGS[SLOTS[_k]] = 16 + _k


class LinkError(Exception):
    """The chip's replies don't make sense, or the transport failed."""


class CapsError(LinkError):
    """CAPS didn't read version 0x01, 4 threads, 512 words."""


class Busy(LinkError):
    """A command found BUSY set: it named a running thread, or a memory command found all four running."""


class Timeout(LinkError):
    """Polling ran out of tries."""


class VerifyError(LinkError):
    """Memory read back differs from what was written."""


class ImageError(ValueError):
    """An image file that can't be loaded."""


def _check(ok, what):
    if not ok:
        raise ValueError(what)


class Status:
    """The four STATUS bytes (section 9.2) as fields: run, ill, late and stale are 4-bit thread masks, bit i for
    thread i; full is the 8-bit mask of full mailbox words; busy is BUSY. raw keeps the bytes, so the reserved bits
    7..1 of byte 3 can still be seen."""

    def __init__(self, raw):
        self.raw = bytes(raw)
        b0, b1, b2, b3 = self.raw
        self.run = b0 >> 4
        self.ill = b0 & 15
        self.late = b1 >> 4
        self.stale = b1 & 15
        self.full = b2
        self.busy = b3 & 1

    def __repr__(self):
        return 'run %X ill %X late %X stale %X full %02X busy %d' % (
            self.run, self.ill, self.late, self.stale, self.full, self.busy)


class Flags:
    """RDREG 11: T in bit 0, LATE in bit 1, STALE in bit 2, WAITING in bit 3."""

    def __init__(self, v):
        self.value = v
        self.t = v & 1
        self.late = v >> 1 & 1
        self.stale = v >> 2 & 1
        self.waiting = v >> 3 & 1

    def __repr__(self):
        return 'T %d LATE %d STALE %d WAITING %d' % (self.t, self.late, self.stale, self.waiting)


def runs(words, most=64):
    """(address, [words]) for each run of consecutive addresses in the dict words, split so that no run is longer
    than most words. Each piece starts at its own address."""
    out = []
    for a in sorted(words):
        if out and a == out[-1][0] + len(out[-1][1]) and len(out[-1][1]) < most:
            out[-1][1].append(words[a])
        else:
            out.append((a, [words[a]]))
    return out


class Host:
    """One method per command of section 9.2, then loading, starting and polling. chunk is the most words one
    WRMEM or RDMEM carries, so the board's buffers stay small."""

    def __init__(self, link, depth=DEPTH, chunk=64):
        self.link = link
        self.depth = depth
        self.chunk = chunk

    # ---- one transaction per command -----------------------------------------------------------------------
    def xfer(self, out):
        got = self.link.xfer(bytes(out))
        if len(got) != len(out):
            raise LinkError('the transport returned %d bytes for %d' % (len(got), len(out)))
        return bytes(got)

    def _thread(self, t):
        # The chip would take thread 5 as thread 1 (section 9.2), so anything but 0 to 3 is refused here.
        _check(0 <= t < THREADS, 'thread %r is not 0 to 3' % (t,))

    def _span(self, addr, n):
        _check(0 <= addr and 1 <= n and addr + n <= self.depth,
               '%d words from address %r are not inside the %d-word memory' % (n, addr, self.depth))

    @staticmethod
    def _caps(r):
        return r[1], r[2] >> 4, r[2] & 15

    def caps(self):
        """(version, threads, log2 of the memory depth)."""
        return self._caps(self.xfer(bytes((CAPS, 0, 0))))

    def check_caps(self):
        """CAPS, which must read version 0x01, 4 threads and 512 words. It is the first thing to read after reset,
        and its known bytes show a link that reads MISO one bit early or late."""
        r = self.xfer(bytes((CAPS, 0, 0)))
        want = (VERSION, THREADS << 4 | LOG2_DEPTH)
        if self._caps(r) == (VERSION, THREADS, LOG2_DEPTH):
            return VERSION, THREADS, LOG2_DEPTH
        if (r[1], r[2]) == (0x02, 0x92):
            why = 'every bit arrived one place early: MISO is sampled after the chip has moved it (a hold problem)'
        elif (r[1], r[2]) in ((0x00, 0xA4), (0x00, 0x24)):
            # 00 24 when a gap between bytes, as the RP2350's SPI block makes, gives each byte's first bit more time
            why = 'bits arrived one place late: MISO is sampled before the chip has moved it (a setup problem)'
        elif r == b'\x00\x00\x00':
            why = 'MISO stays low: is the project selected, clocked and out of reset?'
        elif r == b'\xff\xff\xff':
            why = 'MISO stays high'
        else:
            why = 'not a reply this chip gives'
        raise CapsError('CAPS read %02X %02X %02X, want 00 %02X %02X: %s' % (r[0], r[1], r[2], want[0], want[1], why))

    def wrmem(self, addr, words):
        self._span(addr, len(words))
        out = bytearray(3 + 2 * len(words))
        out[0] = WRMEM
        out[1] = addr >> 8
        out[2] = addr & 255
        i = 3
        for w in words:
            _check(0 <= w <= 0xFFFF, 'word %r is not 16 bits' % (w,))
            out[i] = w >> 8
            out[i + 1] = w & 255
            i += 2
        self.xfer(out)

    def rdmem(self, addr, n):
        """n words from addr. They read 0 if all four threads run (BUSY sets)."""
        self._span(addr, n)
        out = bytearray(5 + 2 * n)
        out[0] = RDMEM
        out[1] = addr >> 8
        out[2] = addr & 255
        r = self.xfer(out)
        if r[1] or r[2] or r[3] or r[4]:
            raise LinkError('RDMEM: the 4 bytes before the data read %02X %02X %02X %02X, not 0' % (r[1], r[2], r[3], r[4]))
        return [r[5 + 2 * i] << 8 | r[6 + 2 * i] for i in range(n)]

    def run(self, stop, start):
        """Stop the threads in the mask stop and start those in start (bit i is thread i), in one command, so all
        the threads it starts share one D (section 9.3)."""
        _check(0 <= stop < 16 and 0 <= start < 16, 'masks are 4 bits')
        self.xfer(bytes((RUN, stop, start)))

    def setpc(self, t, pc):
        self._thread(t)
        _check(0 <= pc < self.depth, 'PC %r is past the memory' % (pc,))
        self.xfer(bytes((SETPC, t, pc >> 8, pc & 255)))

    def status(self):
        """A Status. Reading it clears BUSY."""
        r = self.xfer(bytes((STATUS, 0, 0, 0, 0)))
        return Status(r[1:5])

    def step(self, t):
        self._thread(t)
        self.xfer(bytes((STEP, t)))

    def rdreg(self, t, reg):
        """Register reg of stopped thread t, by number (0 to 31) or name: R0..R7, D, S, PC, FLAGS, SSTAT, CRC_LO,
        CRC_HI, ENDF, CFG0..CFG15 or a slot name such as TXPER. It reads 0 if the thread runs (BUSY sets)."""
        self._thread(t)
        n = REGS.get(reg) if isinstance(reg, str) else reg
        _check(n is not None and 0 <= n < 32, 'no register %r' % (reg,))
        r = self.xfer(bytes((RDREG, t, n, 0, 0, 0)))
        return r[4] << 8 | r[5]

    def post(self, n, data):
        """Fill mailbox word n with data if it is empty. True if it took the data, False if the word was full
        (the result bit 0 of section 9.2 is 1 then)."""
        _check(0 <= n < 8, 'mailbox word %r is not 0 to 7' % (n,))
        _check(0 <= data <= 0xFFFF, 'data %r is not 16 bits' % (data,))
        r = self.xfer(bytes((POST + n, data >> 8, data & 255, 0)))
        if r[1] or r[2] or r[3] & 0xFE:
            raise LinkError('POST: reply %02X %02X %02X, want 00 00 then 00 or 01' % (r[1], r[2], r[3]))
        return not r[3] & 1

    def take(self, n):
        """(full, data): if mailbox word n was full, (True, its word), and the word is now empty. Otherwise
        (False, 0)."""
        _check(0 <= n < 8, 'mailbox word %r is not 0 to 7' % (n,))
        r = self.xfer(bytes((TAKE + n, 0, 0, 0)))
        if r[1] & 0xFE or (not r[1] and (r[2] or r[3])):
            raise LinkError('TAKE: reply %02X %02X %02X: a word that is not full reads all 0' % (r[1], r[2], r[3]))
        return bool(r[1]), r[2] << 8 | r[3]

    def abort(self):
        """Release every uio pad and stop every thread (section 9.3)."""
        self.xfer(bytes((ABORT,)))

    # ---- on top of the commands ----------------------------------------------------------------------------
    def load(self, image):
        """Write image.words, a dict from address to word, with one WRMEM per run of consecutive addresses."""
        self.status()                               # clears BUSY
        for a, ws in runs(image.words, self.chunk):
            self.wrmem(a, ws)
        if self.status().busy:
            raise Busy('WRMEM found all four threads running (section 9.4): the image is not loaded')

    def verify(self, image):
        """Read the image back. A list of (address, wanted, read) for every word that differs."""
        self.status()
        bad = []
        for a, ws in runs(image.words, self.chunk):
            got = self.rdmem(a, len(ws))
            for i in range(len(ws)):
                if got[i] != ws[i]:
                    bad.append((a + i, ws[i], got[i]))
        if self.status().busy:
            raise Busy('RDMEM found all four threads running (section 9.4): nothing was read')
        return bad

    def load_and_verify(self, image):
        self.load(image)
        bad = self.verify(image)
        if bad:
            a, w, g = bad[0]
            raise VerifyError('%d words differ, the first at %d: wrote %04X, read %04X' % (len(bad), a, w, g))

    def start(self, pcs):
        """Set the PC of each thread in the dict pcs and start them all with one RUN."""
        self.status()
        mask = 0
        for t in sorted(pcs):
            self.setpc(t, pcs[t])
            mask |= 1 << t
        if self.status().busy:
            raise Busy('SETPC named a running thread')
        self.run(0, mask)

    def stop(self, mask=0xF):
        self.run(mask, 0)

    def wait_stopped(self, mask=0xF, tries=100, pause_us=100):
        """Poll STATUS until no thread in mask runs. Returns the last Status."""
        for _ in range(tries):
            s = self.status()
            if not s.run & mask:
                return s
            self.link.pause_us(pause_us)
        raise Timeout('threads %X still run after %d tries' % (s.run & mask, tries))

    def take_wait(self, n, tries=200, pause_us=50):
        """The next word a thread sends to mailbox word n."""
        for _ in range(tries):
            full, data = self.take(n)
            if full:
                return data
            self.link.pause_us(pause_us)
        raise Timeout('nothing in mailbox word %d after %d tries' % (n, tries))

    def post_wait(self, n, data, tries=200, pause_us=50):
        """POST, again while the word is still full."""
        for _ in range(tries):
            if self.post(n, data):
                return
            self.link.pause_us(pause_us)
        raise Timeout('mailbox word %d still full after %d tries' % (n, tries))


# ---- program images ----------------------------------------------------------------------------------------
# A text file that the board can load without the assembler. host/mkimage.py writes it from the assembler's output:
#
#   pe-image 1
#   # comments
#   @0000 90E8 9020 0100      up to 16 words, from the hex address after @
#   label byte 0004           a label of the source and its address
#   start 0 0000              thread 0 starts at 0 (mkimage.py --start)
#   end 15 6F3A               the number of words, and the CRC below over every word in address order
#
# The end line comes last, so a file cut short in copying is refused.

class Image:
    def __init__(self, words=None, labels=None, starts=None):
        self.words = {} if words is None else words       # address -> word
        self.labels = {} if labels is None else labels    # name -> address
        self.starts = {} if starts is None else starts    # thread -> address


def crc16(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE: polynomial 0x1021, MSB first, no reflection, starting from crc."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = (crc << 1 ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc


def image_crc(words):
    crc = 0xFFFF
    for a in sorted(words):
        w = words[a]
        crc = crc16(bytes((a >> 8, a & 255, w >> 8, w & 255)), crc)
    return crc


def _hex(s, no, digits=None):
    if not s or (digits and len(s) != digits):
        raise ImageError('line %d: %r is not %s hex digits' % (no, s, digits or 'some'))
    for c in s:
        if c not in '0123456789abcdefABCDEF':
            raise ImageError('line %d: %r is not hex' % (no, s))
    return int(s, 16)


def read_image(lines, depth=DEPTH):
    """An Image from the lines of an image file (any iterable of strings, such as an open file)."""
    words, labels, starts = {}, {}, {}
    seen_header = False
    end = None
    no = 0
    for line in lines:
        no += 1
        line = line.strip()
        if not line or line[0] == '#':
            continue
        f = line.split()
        if not seen_header:
            if f != ['pe-image', '1']:
                raise ImageError('line %d: not a "pe-image 1" file' % no)
            seen_header = True
            continue
        if end is not None:
            raise ImageError('line %d: text after the end line' % no)
        if f[0][0] == '@':
            if not 2 <= len(f) <= 17:
                raise ImageError('line %d: 1 to 16 words after the address' % no)
            a = _hex(f[0][1:], no)
            for w in f[1:]:
                if a >= depth:
                    raise ImageError('line %d: address %d is past the %d-word memory' % (no, a, depth))
                if a in words:
                    raise ImageError('line %d: address %d is given twice' % (no, a))
                words[a] = _hex(w, no, 4)
                a += 1
        elif f[0] == 'label' and len(f) == 3:
            labels[f[1]] = _hex(f[2], no)
        elif f[0] == 'start' and len(f) == 3:
            if f[1] not in ('0', '1', '2', '3'):
                raise ImageError('line %d: thread %r is not 0 to 3' % (no, f[1]))
            starts[int(f[1])] = _hex(f[2], no)
        elif f[0] == 'end' and len(f) == 3:
            end = (f[1], _hex(f[2], no, 4))
        else:
            raise ImageError('line %d: %r is not an image line' % (no, line))
    if not seen_header:
        raise ImageError('empty file')
    if end is None:
        raise ImageError('no end line: the file is cut short')
    if end[0] != str(len(words)):
        raise ImageError('the end line counts %s words, the file has %d' % (end[0], len(words)))
    if end[1] != image_crc(words):
        raise ImageError('CRC %04X, the end line says %04X: the words are damaged' % (image_crc(words), end[1]))
    for a in list(labels.values()) + list(starts.values()):
        if a >= depth:
            raise ImageError('address %d is past the %d-word memory' % (a, depth))
    return Image(words, labels, starts)
