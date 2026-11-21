"""An SD memory card in SD mode at the pads, for bench/sd_unit.py: the reference, written from the SD Physical Layer
Simplified Specification before the firmware, and used by no program.

What it follows, by section of that specification:
- 3.6.1, Figures 3-7 to 3-9: a command token is 48 bits on CMD, start bit 0, transmitter bit 1, 6-bit index, 32-bit
  argument, CRC7, end bit 1, most significant bit first. Responses R1, R3, R6 and R7 are 48 bits, R2 136 bits, with
  transmitter bit 0. A data block on DAT0 (1-bit bus) is a start bit 0, the bytes in order, each most significant
  bit first, the CRC16 and an end bit 1.
- 4.5: CRC7 is the remainder of M(x) x^7 / (x^7 + x^3 + 1) over the first 40 bits of a command or response (the
  120 bits of a CID or CSD for R2), and CRC16 the remainder of M(x) x^16 / (x^16 + x^12 + x^5 + 1) over the block.
  Both are computed here by polynomial long division, and self_test() checks the section's own examples: CMD0,
  CMD17 and CMD17's response, and 512 bytes of 0xFF giving 0x7FA1.
- 4.3.13, 4.9.6: CMD8 carries VHS in argument bits 11:8 and a check pattern in 7:0; a card that accepts the voltage
  (VHS 0001b) answers R7, echoing both, and otherwise does not answer.
- 4.9.1 (R1), 4.9.3 (R2: bits 133:128 are 111111, then CID bits 127:1, whose own CRC7 is bits 7:1, then the end bit).
- 4.6.1: a command with a CRC error is not answered and not executed.
- 6.4.1: the host gives at least 74 clocks with CMD high before the first command. 4.4: the host may stop the clock.
- 6.6.6 (default speed): the card takes CMD at the rising edge of CLK and changes its outputs after the falling edge.

Not in the Simplified Specification's text: section 4.12 ("Timings") is blank there, so NCR (command end bit to
response start bit, 2 to 64 clocks in the full specification), NAC (command end bit to data start bit) and the 8 idle
clocks before a next command (NRC, NCC) are parameters here, and the bench sweeps them.

States (4.8, reduced to the commands the bench sends): idle at power-up and after CMD0; CMD8 in idle answers R7;
CMD55 answers R1 with APP_CMD, and the ACMD41 after it R3 with the OCR (power-up done, CCS 1) and moves to ready; CMD2
in ready answers R2 with the CID and moves to ident; CMD3 in ident answers R6 with a new RCA and moves to stand-by;
CMD7 with that RCA answers R1 and moves to transfer; CMD17 (READ_SINGLE_BLOCK, a block number as SDHC takes it) in
transfer answers R1 with status 0x00000900 (READY_FOR_DATA, CURRENT_STATE tran), as in 4.5's example, and sends the
block. R1's CURRENT_STATE is the state the command found (4.10.1). Anything else gets no answer, and nothing models
busy, errors in the status or the other states.

Time: the card sees CLK and CMD as the chip's pads show them in a cycle, and its outputs change d cycles after the
falling edge it acts on (d may be fractional, and stands for the card's output delay plus the board's). The chip reads
a change in the first cycle at or after that time.
"""

POLY7 = 0x09        # x^7 + x^3 + 1 without its top term
POLY16 = 0x1021     # x^16 + x^12 + x^5 + 1 without its top term


def remainder(bits, width, poly):
    """The remainder of M(x) x^width / G(x), M's first bit the highest term (4.5). Plain long division."""
    top = 1 << width
    g = top | poly
    r = 0
    for b in list(bits) + [0] * width:          # M(x) x^width
        r = (r << 1) | b
        if r & top:
            r ^= g
    return r


def crc7(bits):
    return remainder(bits, 7, POLY7)


def crc16(bits):
    return remainder(bits, 16, POLY16)


def bits_of(v, n):
    return [(v >> (n - 1 - i)) & 1 for i in range(n)]


def value(bits):
    v = 0
    for b in bits:
        v = (v << 1) | b
    return v


def command(index, arg):
    """The 48 bits of a command token, start bit first."""
    head = [0, 1] + bits_of(index, 6) + bits_of(arg, 32)
    return head + bits_of(crc7(head), 7) + [1]


def response48(index, arg, crc=True):
    """R1, R6 or R7 (crc) or R3 (index field 111111, CRC field 1111111), start bit first."""
    head = [0, 0] + bits_of(index, 6) + bits_of(arg, 32)
    return head + (bits_of(crc7(head), 7) if crc else [1] * 7) + [1]


def cid_register(fields):
    """A 128-bit CID with its CRC7 in bits 7:1 over bits 127:8, and bit 0 = 1 (5.2)."""
    body = bits_of(fields, 120)
    return body + bits_of(crc7(body), 7) + [1]


def response_r2(reg128):
    """R2: start, transmitter bit 0, 111111, register bits 127:1, end bit (4.9.3)."""
    return [0, 0] + [1] * 6 + list(reg128[:127]) + [1]


def data_block(data):
    """A data block on DAT0 in 1-bit mode: start bit, bytes MSB first, CRC16, end bit (3.6.1, Figure 3-9)."""
    bits = [b for byte in data for b in bits_of(byte, 8)]
    return [0] + bits + bits_of(crc16(bits), 16) + [1]


def self_test():
    """4.5's examples, and the familiar CMD8 with argument 0x1AA, whose last byte is 0x87. Returns the failures."""
    bad = []
    for name, bits, want in (
            ('CMD0, argument 0', command(0, 0), 0b1001010),
            ('CMD17, argument 0', command(17, 0), 0b0101010),
            ('response of CMD17, status 0x00000900', response48(17, 0x00000900), 0b0110011),
            ('CMD8, argument 0x1AA', command(8, 0x1AA), 0x87 >> 1)):
        got = value(bits[40:47])
        if got != want:
            bad.append('%s: CRC7 %s, the specification gives %s' % (name, bin(got), bin(want)))
    if value(command(0, 0)[40:48]) != 0x95:
        bad.append('CMD0 does not end in 0x95')
    if crc16([1] * 4096) != 0x7FA1:
        bad.append('512 bytes of 0xFF: CRC16 0x%04X, the specification gives 0x7FA1' % crc16([1] * 4096))
    return bad


STATUS_TRAN = 0x00000900        # READY_FOR_DATA, CURRENT_STATE 4 (tran), 4.10.1


class Card:
    """The card. ncr: clocks from a command's end bit to its response's start bit (2..64). nac: clocks from a read
    command's end bit to the data block's start bit. d: the output delay in cycles. state: 'idle' or 'tran'.
    v2: answers CMD8. blocks: {block number: 512 bytes}; others read as a pattern of the block number.
    flip: {(block, bit): 1} to corrupt a block bit on the wire after its CRC is made; bad_end sends the block's end
    bit as 0; resp_flip: {index: bit} corrupts that bit of the response to a command. cid: 120 bits of CID fields."""

    def __init__(self, ncr=2, nac=8, d=0.0, state='idle', v2=True, blocks=None, flip=None, cid=0x0353445344433136F0,
                 bad_end=False, resp_flip=None):
        self.ncr, self.nac, self.d, self.state, self.v2 = ncr, nac, d, state, v2
        self.blocks = dict(blocks or {})
        self.flip = dict(flip or {})
        self.cid = cid_register(cid)
        self.bad_end = bad_end
        self.resp_flip = dict(resp_flip or {})
        self.app = False
        self.rca = 0xB368
        self.ocr = 0xC0FF8000           # power-up done, CCS 1, 2.7 to 3.6 V
        self.prev_clk = 0
        self.k = 0                      # rising edges so far
        self.rx = None                  # bits of the command being received
        self.cmd_q = []                 # [(first clock, bits)] scheduled on CMD
        self.dat_q = []                 # on DAT0
        self.cmd_out = (0, 1)           # (driving, level) now
        self.dat_out = (0, 1)
        self.pending = []               # (time, 'cmd' or 'dat', (driving, level))
        self.log = []                   # (cycle, kind, detail): commands taken, responses and blocks sent, faults
        self.faults = []
        self.high_clocks = 0            # clocks with CMD high before the first command
        self.first_cmd = None
        self.cmd_last = None            # clock of the last end bit on CMD, the host's or the card's
        self.dat_last = -1              # clock of the last block's end bit
        self.last_rise = None
        self.periods = []               # cycles between rising edges
        self.highs, self.lows = [], []
        self.last_edge = None
        self.cmd_seen = []              # (index, arg, crc ok)
        self.answered = []              # (kind, bits)
        self.per_cmd = []               # one record per good command: index, arg, the response bits, the block
        self.gaps = []                  # clocks between the last end bit on CMD and each later command's start bit

    def block_data(self, n):
        if n in self.blocks:
            return list(self.blocks[n])
        x = (n * 2654435761 + 12345) & 0xFFFFFFFF
        out = []
        for _ in range(512):
            x = (x * 1103515245 + 12345) & 0x7FFFFFFF
            out.append((x >> 16) & 0xFF)
        return out

    def schedule(self, q, first, bits):
        q.append((first, list(bits)))

    STATES = {'idle': 0, 'ready': 1, 'ident': 2, 'stby': 3, 'tran': 4}

    def r1(self, index, extra=0):
        return response48(index, 0x100 | self.STATES[self.state] << 9 | extra)

    def act(self, c, index, arg):
        k = self.k
        app, self.app = self.app, False
        r = None
        rec = {'index': index, 'arg': arg, 'resp': None, 'block': None, 'clock': k}
        self.per_cmd.append(rec)
        if index == 0:
            self.state = 'idle'
            self.log.append((c, 'cmd0', 'go idle, no response'))
        elif app and index == 41 and self.state == 'idle':
            r = response48(0x3F, self.ocr, crc=False)
            self.state = 'ready'
        elif index == 8 and self.state == 'idle' and self.v2 and ((arg >> 8) & 15) == 1:
            r = response48(8, arg & 0xFFF)
        elif index == 55 and self.state != 'ready':
            r = self.r1(55, 0x20)
            self.app = True
        elif index == 2 and self.state == 'ready':
            r = response_r2(self.cid)
            self.state = 'ident'
        elif index == 3 and self.state == 'ident':
            r = response48(3, self.rca << 16 | 0x100 | self.STATES[self.state] << 9)
            self.state = 'stby'
        elif index == 7 and self.state == 'stby' and arg >> 16 == self.rca:
            r = self.r1(7)
            self.state = 'tran'
        elif index == 17 and self.state == 'tran':
            r = self.r1(17)
            data = self.block_data(arg)
            blk = data_block(data)
            for (bn, bit), _ in self.flip.items():
                if bn == arg:
                    blk[1 + bit] ^= 1
            if self.bad_end:
                blk[-1] = 0
            self.schedule(self.dat_q, k + self.nac, blk)
            self.answered.append(('block', arg, data))
            rec['block'] = data
            self.dat_last = k + self.nac + len(blk) - 1
        if r is None:
            if index:
                self.log.append((c, 'ignored', 'CMD%d arg 0x%08X in state %s' % (index, arg, self.state)))
            self.cmd_last = k
            return
        if index in self.resp_flip:                 # a bit of the response corrupted on the wire
            r = list(r)
            r[self.resp_flip[index]] ^= 1
        self.schedule(self.cmd_q, k + self.ncr, r)
        self.answered.append(('response', index, r))
        rec['resp'] = r
        self.cmd_last = k + self.ncr + len(r) - 1

    def drive_for(self, q, clock):
        """What a queue drives in the clock period that ends at rising edge `clock`."""
        for first, bits in q:
            if first <= clock < first + len(bits):
                return (1, bits[clock - first])
        return (0, 1)

    def step(self, c, clk, cmd):
        """clk and cmd: the levels in cycle c as the chip's pads and the bus give them. Returns nothing; read
        levels(c) for this cycle's outputs."""
        rise = clk and not self.prev_clk
        fall = self.prev_clk and not clk
        if rise or fall:
            if self.last_edge is not None:
                (self.lows if rise else self.highs).append(c - self.last_edge)
            self.last_edge = c
        self.prev_clk = clk
        if rise:
            self.k += 1
            if self.last_rise is not None:
                self.periods.append(c - self.last_rise)
            self.last_rise = c
            driving = self.cmd_out[0]
            if self.rx is None and not driving:
                if cmd == 0:
                    if self.first_cmd is None:
                        self.first_cmd = (c, self.high_clocks)
                    if self.cmd_last is not None:
                        self.gaps.append(self.k - self.cmd_last - 1)
                    if self.cmd_last is not None and self.k - self.cmd_last - 1 < 8:
                        self.faults.append('cycle %d: a command starts %d clocks after the last end bit on CMD, 8 '
                                           'needed (NRC, NCC)' % (c, self.k - self.cmd_last - 1))
                    if self.k <= self.dat_last:
                        self.faults.append('cycle %d: a command starts during a data block' % c)
                    self.rx = [0]
                elif self.first_cmd is None:
                    self.high_clocks += 1
            elif self.rx is not None:
                if driving:
                    self.faults.append('cycle %d: the card drives CMD while a command comes in' % c)
                self.rx.append(cmd)
                if len(self.rx) == 48:
                    bits, self.rx = self.rx, None
                    index, arg = value(bits[2:8]), value(bits[8:40])
                    ok = bits[1] == 1 and bits[47] == 1 and value(bits[40:47]) == crc7(bits[:40])
                    self.cmd_seen.append((index, arg, ok))
                    if ok:
                        self.act(c, index, arg)
                    else:
                        self.log.append((c, 'crc', 'CMD%d: bad CRC, transmitter or end bit; not answered'
                                         % index))
        if fall:
            nxt = self.k + 1                    # the period after this fall ends at rising edge k + 1
            for name, q in (('cmd', self.cmd_q), ('dat', self.dat_q)):
                self.pending.append((c + self.d, name, self.drive_for(q, nxt)))

    def levels(self, c):
        """The card's (CMD drive, CMD level, DAT0 drive, DAT0 level) in cycle c."""
        keep = []
        for t, name, v in self.pending:
            if t <= c:
                if name == 'cmd':
                    self.cmd_out = v
                else:
                    self.dat_out = v
            else:
                keep.append((t, name, v))
        self.pending = keep
        return self.cmd_out + self.dat_out


CLK_PIN, CMD_PIN, DAT0_PIN = 0, 1, 2            # io0, io1, io2


class Bus:
    """The SD bus between the chip's uio pads and the card. CMD and DAT0..DAT3 have pull-ups (6.5), CLK is driven
    by the chip only and reads 0 undriven. Logs every level change and counts cycles where the chip and the card
    drive one line at once."""

    def __init__(self, card):
        self.card = card
        self.both = 0                   # cycles where chip and card drive CMD together
        self.both_dat = 0
        self.first_both = None
        self.log = []                   # (cycle, 'clk' / 'cmd' / 'dat', level)
        self.lv = None
        self.chip_cmd = []              # (cycle, chip drives CMD) changes

    def step(self, chip, c, uo, en, val):
        card = self.card
        ce, cl, de, dl = card.levels(c)
        clk = (val >> CLK_PIN) & 1 if (en >> CLK_PIN) & 1 else 0
        chip_cmd = (en >> CMD_PIN) & 1
        if chip_cmd:
            cmd = (val >> CMD_PIN) & 1
        else:
            cmd = cl if ce else 1
        if chip_cmd and ce:
            self.both += 1
            if self.first_both is None:
                self.first_both = c
        chip_dat = (en >> DAT0_PIN) & 1
        dat = ((val >> DAT0_PIN) & 1) if chip_dat else (dl if de else 1)
        if chip_dat and de:
            self.both_dat += 1
        card.step(c, clk, cmd)
        ce2, cl2, de2, dl2 = card.levels(c)          # a change due in this very cycle (d = 0) shows at once
        if not chip_cmd:
            cmd = cl2 if ce2 else 1
        if not chip_dat:
            dat = dl2 if de2 else 1
        lv = (clk, cmd, dat)
        if lv != self.lv:
            for i, name in enumerate(('clk', 'cmd', 'dat')):
                if self.lv is None or self.lv[i] != lv[i]:
                    self.log.append((c, name, lv[i]))
            self.lv = lv
        if not self.chip_cmd or self.chip_cmd[-1][1] != chip_cmd:
            self.chip_cmd.append((c, chip_cmd))
        uio = 0xFF & ~7 | clk << CLK_PIN | cmd << CMD_PIN | dat << DAT0_PIN
        return 0xFF, uio
