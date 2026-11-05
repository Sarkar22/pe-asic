"""Benches for the programs in trimmed/, cut down so that four programs share one 128-word memory.

First the trimmed programs alone, on the same benches as the full ones in standard/, then all four in one
128-word image on four threads: each protocol's pad trace and mailbox results must match its solo run to
the cycle, with no LATE. Then the same scenario for the four full programs of standard/ (135 words) on a
512-word memory.
"""
import i2c_eeprom
from devices import SpiTarget, UartSource
from report import assemble, check
from simple import Board, Chip, read

PROGRAMS = ('trimmed/uart_tx.s', 'trimmed/uart_rx.s', 'trimmed/i2c.s', 'standard/uart_tx.s', 'standard/uart_rx.s',
            'standard/spi.s', 'standard/i2c.s')


def uart_tx_alone(rel, period, tid=0):
    a = assemble(read(rel), depth=128)
    chip, b = Chip(a.words, 128), Board()
    chip.start(tid)
    chip.post(7, period)
    chip.run(60, b)
    got, exact = [], True
    for byte in (0x55, 0xA3, 0x00, 0xFF):
        while not chip.post(0, byte):
            chip.run(4, b)
        since = chip.c
        chip.run(12 * period + 60, b)
        e = b.edges('uo0', since)
        if not e:                       # 0xFF with the line already high still has a start edge
            return None
        level = lambda t: ([1] + [v for c, v in e if c <= t])[-1]
        bits = [level(e[0][0] + period // 2 + i * period) for i in range(10)]
        got.append(sum(x << i for i, x in enumerate(bits[1:9])))
        exact &= bits[0] == 0 and bits[9] == 1 and all((c - e[0][0]) % period == 0 for c, _ in e)
    return got == [0x55, 0xA3, 0x00, 0xFF] and exact and chip.t[tid].latecount == 0, len(a.words)


def uart_rx_alone(rel, period, two_cfg, tid=0):
    a = assemble(read(rel), depth=128)
    chip, b = Chip(a.words, 128), Board()
    chip.start(tid)
    data = [0x00, 0xFF, 0x5A, 0xC3]
    chip.post(7, period)
    if two_cfg:
        chip.run(40, b)
        chip.post(7, period // 2)
    src = UartSource(3, period, data, 200)
    b.devs.append(src)
    got = []
    for _ in range(src.end + 300):
        chip.cycle(b)
        v = chip.take(1)
        if v is not None:
            got.append(v)
    return got == data, len(a.words)


class BoardBus:
    """Lets the pad-level EEPROM of i2c_eeprom.py sit on the Board's wired-AND bus."""
    def __init__(self, board):
        self.b = board

    @property
    def ext_low(self):
        return self.b.io_low

    @ext_low.setter
    def ext_low(self, v):
        self.b.io_low = v


class Eeprom:
    def __init__(self, stretch=0):
        self.d = i2c_eeprom.Eeprom(1, 0, stretch=stretch)
        self.d.mem[0x10] = 0xC3
        self.d.mem[0x20] = 0x3C
        self.bus = None

    def step(self, c, board):
        self.bus = self.bus or BoardBus(board)
        self.d.step(c, self.bus, board.io)


ORG = dict(tx=0, rx=14, spi=34, i2c=56)
DEFS = dict(tx={'MCFG': 7, 'MIN': 0}, rx={'MCFG': 7, 'MOUT': 1},
            spi={'MCFG': 7, 'MIN': 2, 'MOUT': 3}, i2c={'MCFG': 7, 'MIN': 4, 'MOUT': 5})
FILES = dict(tx='trimmed/uart_tx.s', rx='trimmed/uart_rx.s', spi='standard/spi.s', i2c='trimmed/i2c.s')
THREAD = dict(tx=0, rx=1, spi=2, i2c=3)


def image(files=FILES, org=ORG, size=128):
    words = {}
    for k in org:
        a = assemble(read(files[k]), defines=DEFS[k], base=org[k], depth=size)
        assert not set(a.words) & set(words), k
        words.update(a.words)
    return words


def scenario(which, words, org, size, rx_cfg=2, cycles=40000):
    """A thread starts at the cycle of its first config word: all threads read config from one
    mailbox word, so they must not be waiting on it at the same time."""
    chip, b = Chip(words, size), Board()
    b.ui &= ~(1 << 4)
    spi = SpiTarget(0, 0, 0x96)
    rom = Eeprom()
    b.devs += [UartSource(3, 100, [0x31, 0xC7, 0x05], 400), spi, rom]
    cfg = [(0, 'tx', 7, 434), (40, 'rx', 7, 100), (120, 'spi', 7, 10),
           (160, 'i2c', 7, 45), (200, 'i2c', 7, 37)]
    if rx_cfg == 2:
        cfg.append((80, 'rx', 7, 50))
    load = [(300, 'tx', 0, 0x55), (500, 'spi', 2, 0xD2), (600, 'i2c', 4, 0xA010), (900, 'i2c', 4, 0x5A00),
            (9000, 'tx', 0, 0xA3), (9100, 'spi', 2, 0x3C), (12000, 'i2c', 4, 0xA120)]
    start = {0: 'tx', 40: 'rx', 120: 'spi', 160: 'i2c'}
    posts = {t: (m, v) for t, k, m, v in cfg + load if k in which}
    rx, spi_in, i2c_in = [], [], []
    for c in range(cycles):
        if c in start and start[c] in which:
            k = start[c]
            chip.start(THREAD[k], org[k])
        if c in posts:
            chip.post(*posts[c])
        chip.cycle(b)
        v = chip.take(1)
        if v is not None:
            rx.append((c, v))
        v = chip.take(3)
        if v is not None:
            spi_in.append((c, v))
        v = chip.take(5)
        if v is not None:
            i2c_in.append((c, v))
    return chip, b, rx, spi_in, i2c_in, rom.d


def same_run(words, org, size, label, rx_cfg=2):
    full = scenario(('tx', 'rx', 'spi', 'i2c'), words, org, size, rx_cfg)
    chip, b, rx, sp, ic, rom = full
    pins = dict(tx={'uo0'}, spi={'uo1', 'uo2', 'uo5'}, i2c={'io0', 'io1'})
    for k, names in pins.items():
        one = scenario((k,), words, org, size, rx_cfg)
        a = [e for e in b.log if e[1] in names]
        c = [e for e in one[1].log if e[1] in names]
        check(f'{label}: {k} pad trace identical to the cycle, alone vs all four running', a == c and len(a) > 10, f'{len(a)} edges')
    one = scenario(('rx',), words, org, size, rx_cfg)
    check(f'{label}: uart rx results identical to the cycle', one[2] == rx and [v for _, v in rx] == [0x31, 0xC7, 0x05], rx)
    check(f'{label}: spi bytes in', [v for _, v in sp] == [0x96, 0x96], sp)
    check(f'{label}: i2c write then random read gives status 0, 0x5A stored, 0x3C read back',
          [v & 0xFF for _, v in ic][0] == 0 and rom.mem[0x10] == 0x5A and [v & 0xFF for _, v in ic][2:4] == [0, 0x3C],
          [(hex(v)) for _, v in ic])
    late = [t.latecount for t in chip.t]
    check(f'{label}: no LATE on any thread, no OD pin ever driven high', not any(late) and b.chip_high_on_od == 0, late)


def main(log):
    log.group('trimmed/uart_tx.s', title='UART transmitter, trimmed')
    for rel, per in (('trimmed/uart_tx.s', 434), ('trimmed/uart_tx.s', 13)):
        r = uart_tx_alone(rel, per)
        check(f'bit {per} cycles: four bytes on the bit grid, no LATE', r and r[0], r)
    log.group('standard/uart_tx.s', title='UART transmitter')
    r = uart_tx_alone('standard/uart_tx.s', 13)
    check('bit 13 cycles: four bytes on the bit grid, no LATE', r and r[0], r)
    log.group('trimmed/uart_rx.s', title='UART receiver, trimmed: the host posts the half bit too')
    for rel, per, two in (('trimmed/uart_rx.s', 434, True), ('trimmed/uart_rx.s', 50, True)):
        r = uart_rx_alone(rel, per, two)
        check(f'bit {per} cycles: bytes right', r[0], r)
    log.group('trimmed/i2c.s', title='I2C controller, trimmed, against an EEPROM model')
    rel = 'trimmed/i2c.s'
    W, RD, BAD = (0xA0, 0x10, 0x5A), (0xA1, 0x20, 0), (0xB0, 0x10, 0x5A)
    res, d, r, tr, bus = i2c_eeprom.run(rel, [W, (0xA1, 0x10, 0), RD])
    t = i2c_eeprom.timing(tr)
    check('write, then two random reads', res == [0, 0x5A, 0, 0x5A, 0, 0x3C] and d.mem[0x10] == 0x5A and r.late() == 0 and bus.contention == 0, (res, t))
    check('100 kHz intervals inside UM10204', t['low'] >= 4700 and t['high'] >= 4000 and t['su_sta'] >= 4700 and t['su_sto'] >= 4000 and t['hd_sta'] >= 4000, t)
    res, d, r, tr, bus = i2c_eeprom.run(rel, [BAD])
    check('wrong address NACK, STOP sent', res[0] == 1 and d.stops == 1, res)
    res, d, r, tr, bus = i2c_eeprom.run(rel, [W], nack_data=True)
    check('data NACK, STOP sent', res[0] == 1 and d.stops == 1, res)
    res, d, r, tr, bus = i2c_eeprom.run(rel, [(0xA1, 0x20, 0)], stretch=400)
    check('read with 8 us stretch after every SCL fall', res == [0, 0x3C] and r.late() == 0, res)

    def stuck(c, b, dv, lvl):
        b.ext_low |= 1
    res, d, r, tr, bus = i2c_eeprom.run(rel, [W], hook=stuck)
    check('SDA stuck low, status 2 and no clocks', res[0] == 2 and d.falls == 0, (res, d.falls))

    def stuck2(c, b, dv, lvl):
        if dv.falls >= 27:
            b.ext_low |= 2
    res, d, r, tr, bus = i2c_eeprom.run(rel, [W], hook=stuck2)
    check('SCL stuck low at STOP, status 2', res[0] == 2, res)
    res, d, r, tr, bus = i2c_eeprom.run(rel, [W, (0xA1, 0x20, 0)], fast=True)
    t = i2c_eeprom.timing(tr)
    check('400 kHz write and read, intervals inside UM10204', res == [0, 0x5A, 0, 0x3C] and r.late() == 0 and t['low'] >= 1300 and t['high'] >= 600 and t['su_sta'] >= 600 and t['su_sto'] >= 600 and t['hd_sta'] >= 600, (res, t))

    log.group('trimmed/uart_tx.s', 'trimmed/uart_rx.s', 'standard/spi.s', 'trimmed/i2c.s',
              title='the four in one 128-word image, one thread each')
    words = image()
    n = len(words)
    check('four-protocol image fits the 128-word memory', max(words) < 128 and n <= 128, f'{n} words, last address {max(words)}')
    same_run(words, ORG, 128, '128 words')
    log.group('standard/uart_tx.s', 'standard/uart_rx.s', 'standard/spi.s', 'standard/i2c.s',
              title='the four full programs in one 512-word image, one thread each')
    org = dict(tx=0, rx=15, spi=37, i2c=59)
    files = dict(tx='standard/uart_tx.s', rx='standard/uart_rx.s', spi='standard/spi.s', i2c='standard/i2c.s')
    words = image(files, org, 512)
    check('full 135-word set assembled for a 512-word memory', max(words) < 512, f'{len(words)} words')
    same_run(words, org, 512, '512 words', rx_cfg=1)
