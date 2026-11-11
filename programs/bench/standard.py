"""Benches for the programs in standard/.

UART, SPI, I2C write, WS2812 and 1-Wire against device models; CRC check values; an HDLC frame,
Manchester, JTAG and HDMI CEC decoded from the pads; the pre-armed reaction; four programs in one
128-word image; a thread hammering memory; and the I2C controller talking to the I2C target firmware
on another thread of the same chip. Everything is checked from pad traces and mailbox words.
"""
import binascii

import i2c_eeprom
from devices import Force, I2cTarget, OneWireSlave, SpiTarget, UartSource
from report import LOG, assemble, check
from simple import Board, Chip, read, set_ui
import simple

NAMES = ('uart_tx', 'uart_rx', 'spi', 'i2c', 'i2c_write', 'i2c_target', 'ws2812', 'onewire', 'jtag', 'cec',
         'crc16', 'crc8', 'crc32', 'hdlc_tx', 'manch_rx', 'reflex', 'memstress')
PROGRAMS = tuple('standard/%s.s' % n for n in NAMES)


def P(*names):
    return tuple('standard/%s.s' % n for n in names)


def boot(name, tid=0, defs=None, size=128):
    return simple.boot('standard/%s.s' % name, tid, defs, size)


def decode_uart(edges, period, frame_start):
    """Level of the line at each mid-bit, taken from the edge list only."""
    def level_at(t):
        v = 1
        for c, x in edges:
            if c > t:
                break
            v = x
        return v
    bits = [level_at(frame_start + period // 2 + i * period) for i in range(10)]
    return bits[0], sum(b << i for i, b in enumerate(bits[1:9])), bits[9]


def test_uart_tx():
    LOG.group(*P('uart_tx'), title='UART transmitter')
    for period in (434, 5208, 12):
        for tid in range(4):
            chip, b, n = boot('uart_tx', tid)
            chip.post(7, period)
            chip.run(60, b)
            data = [0x55, 0xA3]
            got, exact, gaps = [], True, []
            for byte in data:
                while not chip.post(0, byte):
                    chip.run(4, b)
                since = chip.c
                chip.run(12 * period + 60, b)
                e = b.edges('uo0', since)
                st, val, sp = decode_uart(e, period, e[0][0])
                got.append(val)
                exact &= st == 0 and sp == 1
                exact &= all((c - e[0][0]) % period == 0 for c, _ in e)
            t = chip.t[tid]
            check(f'uart_tx period={period} thread={tid}',
                  got == data and exact and t.late == 0, f'{n} words')


def test_uart_rx():
    LOG.group(*P('uart_rx'), title='UART receiver')
    for period in (434, 50):
        for tid in range(4):
            chip, b, n = boot('uart_rx', tid)
            data = [0x00, 0xFF, 0x5A, 0xC3]
            chip.post(7, period)
            b.devs.append(UartSource(3, period, data, 200))
            got = []
            for _ in range(b.devs[0].end + 300):
                chip.cycle(b)
                v = chip.take(1)
                if v is not None:
                    got.append(v)
            check(f'uart_rx period={period} thread={tid}', got == data, f'{n} words')
    # mid-bit proof: data bits are right only near mid-bit, at every start phase
    data = [0x5A, 0xC3, 0x0F]
    for w in (2, 1, 0):
        ok = []
        for phase in range(8):
            chip, b, _ = boot('uart_rx', 0)
            chip.post(7, 434)
            src = UartSource(3, 434, data, 200 + phase, window=w)
            b.devs.append(src)
            got = []
            for _ in range(src.end + 300):
                chip.cycle(b)
                v = chip.take(1)
                if v is not None:
                    got.append(v)
            ok.append(got == data)
        check(f'uart_rx reads inside +-{w} cycles of mid-bit, 8 start phases', all(ok), ok)
    # glitch rejection: a 10-cycle low pulse before the real frame
    chip, b, _ = boot('uart_rx', 0)
    chip.post(7, 100)
    b.devs += [UartSource(3, 100, [0x41], 300), Force(3, 100, 110, 0)]
    got = []
    for c in range(1600):
        chip.cycle(b)
        v = chip.take(1)
        if v is not None:
            got.append(v)
    check('uart_rx rejects a short glitch', got == [0x41], got)
    # framing error: stop bit forced low
    chip, b, _ = boot('uart_rx', 0)
    chip.post(7, 100)
    b.devs += [UartSource(3, 100, [0x41], 300), Force(3, 1190, 1300, 0)]
    got = []
    for c in range(1700):
        chip.cycle(b)
        v = chip.take(1)
        if v is not None:
            got.append(v)
    check('uart_rx flags a framing error', len(got) == 1 and got[0] >> 8 == 1, [hex(x) for x in got])


def spi_once(cpol, cpha, half, tid, skew=0, tco=0):
    chip, b, n = boot('spi', tid, {'CPOL': cpol, 'CPHA': cpha, 'SKEW': skew})
    dev = SpiTarget(cpol, cpha, 0xB7, tco=tco)
    b.ui &= ~(1 << 4)
    b.devs.append(dev)
    chip.post(7, half)
    chip.run(100, b)
    chip.post(0, 0xD2)
    since = chip.c
    got = None
    for _ in range(40 * half + 400):
        chip.cycle(b)
        v = chip.take(1)
        if v is not None:
            got = v
            break
    return chip, b, dev, got, n, since


def test_spi():
    LOG.group(*P('spi'), title='SPI controller, all four modes')
    for cpol in (0, 1):
        for cpha in (0, 1):
            for half in (25, 10, 9):
                bad = []
                for tid in range(4):
                    for skew in range(4):
                        chip, b, dev, got, n, _ = spi_once(cpol, cpha, half, tid, skew)
                        if dev.got != [0xD2] or got != 0xB7 or chip.t[tid].latecount:
                            bad.append((tid, skew, dev.got, got, chip.t[tid].latecount))
                check(f'spi mode={cpol}{cpha} half={half} all 16 alignments', not bad,
                      bad[:2] if bad else f'{n} words')


NS = 20
SPEC = {   # UM10204 bus timing table, ns: tHD;STA tLOW tHIGH tSU;DAT tSU;STO tBUF
    'Sm': dict(hd_sta=4000, low=4700, high=4000, su_dat=250, su_sto=4000, buf=4700),
    'Fm': dict(hd_sta=600, low=1300, high=600, su_dat=100, su_sto=600, buf=1300),
}
MODES = {'Sm': (250, 210), 'Fm': (45, 37)}      # tHIGH, tLOW-QHOLD in cycles


def i2c_run(mode, word, dev, tid=0, limit=200000):
    chip, b, n = boot('i2c_write', tid)
    b.devs.append(dev)
    high, low = MODES[mode]
    chip.post(7, high)
    while chip.full[7]:
        chip.cycle(b)
    chip.post(7, low)
    chip.run(100, b)
    since = chip.c
    chip.post(0, word)
    res = None
    for _ in range(limit):
        chip.cycle(b)
        res = chip.take(1)
        if res is not None:
            break
    chip.run(400, b)
    return chip, b, n, res, since


def i2c_timing(b, since):
    """Minimum of each interval seen on the pads, in ns."""
    ev = [(c, n, v) for c, n, v in b.log if c >= since and n in ('io0', 'io1')]
    sda, scl, last = 1, 1, {}
    m = dict(hd_sta=1e9, low=1e9, high=1e9, su_dat=1e9, su_sto=1e9, buf=1e9)
    rate = []
    for c, n, v in ev:
        if n == 'io1':
            if v == 0 and 'scl_rise' in last:
                m['high'] = min(m['high'], c - last['scl_rise'])
                if 'scl_fall' in last:
                    rate.append(c - last['scl_fall'])
            if v == 0 and 'start' in last:
                m['hd_sta'] = min(m['hd_sta'], c - last['start'])
                del last['start']
            if v == 1 and 'scl_fall' in last:
                m['low'] = min(m['low'], c - last['scl_fall'])
            if v == 1 and 'sda' in last:
                m['su_dat'] = min(m['su_dat'], c - last['sda'])
            last['scl_rise' if v else 'scl_fall'] = c
            scl = v
        else:
            if scl and v == 0:
                last['start'] = c
                if 'stop' in last:
                    m['buf'] = min(m['buf'], c - last['stop'])
            elif scl and v == 1:
                m['su_sto'] = min(m['su_sto'], c - last['scl_rise'])
                last['stop'] = c
            else:
                last['sda'] = c
            sda = v
    return {k: x * NS for k, x in m.items()}, rate


def test_i2c():
    LOG.group(*P('i2c_write'), title='I2C controller, write only')
    word = 0xA0 << 8 | 0x3C
    for mode in ('Sm', 'Fm'):
        for tid in range(4):
            dev = I2cTarget()
            chip, b, n, res, since = i2c_run(mode, word, dev, tid)
            t, rate = i2c_timing(b, since)
            bad = {k: v for k, v in t.items() if k != 'buf' and v < SPEC[mode][k]}
            ok = dev.bytes == [0xA0, 0x3C] and res == 0 and not bad and dev.stops == 1 \
                and b.chip_high_on_od == 0 and chip.t[tid].latecount == 0
            check(f'i2c {mode} write thread={tid}', ok, f'{n} words, res={res}, bad={bad}')
        LOG.info(f'i2c {mode} intervals ns: {t}; SCL period {min(rate)}..{max(rate)} cycles')
    # stretching: first data bit, the ACK clock, and on the second byte
    for stretch in ({0: 3000}, {7: 5000}, {8: 1234}, {12: 777, 17: 20000}):
        dev = I2cTarget(stretch=stretch)
        chip, b, n, res, since = i2c_run('Sm', word, dev)
        t, _ = i2c_timing(b, since)
        check(f'i2c stretch {stretch}', dev.bytes == [0xA0, 0x3C] and res == 0
              and t['high'] >= SPEC['Sm']['high'] and b.chip_high_on_od == 0,
              f"res={res} tHIGH min {t['high']} ns")
    # NACK on the address and on the data
    for ack, name in (((False,), 'address'), ((True, False), 'data')):
        dev = I2cTarget(ack=ack)
        chip, b, n, res, since = i2c_run('Sm', word, dev)
        check(f'i2c NACK on {name} reported, STOP sent', res == 1 and dev.stops == 1, f'res={res} bytes={dev.bytes}')
    # target holds SCL past the limit: must give up and report, not hang
    dev = I2cTarget(stretch={3: 10**7})
    chip, b, n, res, since = i2c_run('Sm', word, dev, limit=70000)
    check('i2c stretch timeout reported', res == 1, f'res={res}')


def test_ws2812():
    LOG.group(*P('ws2812'), title='WS2812 driver, one byte at a time')
    for tid in range(4):
        chip, b, n = boot('ws2812', tid)
        chip.run(40, b)
        byte = 0b10110010
        chip.post(0, byte)
        since = chip.c
        done = None
        for _ in range(20000):
            chip.cycle(b)
            done = chip.take(1)
            if done:
                break
        e = b.edges('uo6', since)
        rises = [c for c, v in e if v]
        falls = [c for c, v in e if not v]
        highs = [f - r for r, f in zip(rises, falls)]
        periods = [b2 - a for a, b2 in zip(rises, rises[1:])]
        bits = [1 if h > 26 else 0 for h in highs]
        val = sum(x << (7 - i) for i, x in enumerate(bits))
        h0 = {h * NS for h, x in zip(highs, bits) if not x}
        h1 = {h * NS for h, x in zip(highs, bits) if x}
        ok = val == byte and len(rises) == 8 and set(periods) == {62} \
            and all(200 <= h <= 500 for h in h0) and all(550 <= h <= 850 for h in h1) \
            and chip.t[tid].latecount == 0
        check(f'ws2812 thread={tid}', ok, f'{n} words, T0H {h0} ns, T1H {h1} ns, period {set(periods)} cycles')


def test_onewire():
    LOG.group(*P('onewire'), title='1-Wire controller')
    for tid in range(4):
        chip, b, n = boot('onewire', tid)
        dev = OneWireSlave()
        b.devs.append(dev)
        chip.run(40, b)
        chip.post(0, 0xCC)
        res = None
        since = chip.c
        for _ in range(200000):
            chip.cycle(b)
            res = chip.take(1)
            if res is not None:
                break
        e = b.edges('io2', since)
        low0 = e[1][0] - e[0][0]
        ok = dev.got == [0xCC] and res == 1 and dev.resets == 1 and b.chip_high_on_od == 0 \
            and b.contention == 0 and low0 == 24000 and chip.t[tid].latecount == 0
        check(f'onewire reset, presence, write 0xCC thread={tid}', ok,
              f'{n} words, reset low {low0 * NS / 1000} us, got {dev.got}, res={res}')
    chip, b, n = boot('onewire', 0)
    dev = OneWireSlave(present=False)
    b.devs.append(dev)
    chip.run(40, b)
    chip.post(0, 0xCC)
    res = None
    for _ in range(100000):
        chip.cycle(b)
        res = chip.take(1)
        if res is not None:
            break
    check('onewire no device: reports absent, writes nothing', res == 0 and dev.got == [], f'res={res}')


def feed(chip, b, data, end=0x100, answers=1, budget=200000):
    out = []
    for x in list(data) + [end]:
        while not chip.post(0, x):
            chip.cycle(b)
    for _ in range(budget):
        chip.cycle(b)
        v = chip.take(1)
        if v is not None:
            out.append(v)
            if len(out) == answers:
                break
    return out


def test_crc():
    msg = b'123456789'
    for name, want, ans in (('crc16', [0x29B1], 1), ('crc8', [0xA1], 1)):
        LOG.group(*P(name), title={'crc16': 'CRC-16/CCITT-FALSE', 'crc8': 'CRC-8/MAXIM'}[name])
        chip, b, n = boot(name)
        got = feed(chip, b, msg, answers=ans)
        check(f'{name} check value of "123456789"', got == want, f'{n} words, {[hex(x) for x in got]}')
    LOG.group(*P('crc32'), title='CRC-32')
    chip, b, n = boot('crc32')
    got = feed(chip, b, msg, answers=2)
    v = got[0] | got[1] << 16 if len(got) == 2 else None
    check('crc32 check value of "123456789"', v == 0xCBF43926 == binascii.crc32(msg),
          f'{n} words, {hex(v or 0)}')


def x25(data):
    crc = 0xFFFF
    for x in data:
        crc ^= x
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc ^ 0xFFFF


def test_hdlc():
    LOG.group(*P('hdlc_tx'), title='HDLC-style frame transmitter')
    payload = [0xFF, 0x7E, 0x00, 0xF8, 0x1F, 0xFF, 0xFF, 0x41]    # forces stuffing
    period = 64
    chip, b, n = boot('hdlc_tx')
    chip.post(7, period)
    chip.run(60, b)
    got = feed(chip, b, payload, budget=20000)
    chip.run(period * 4, b)
    e = b.edges('uo7')[1:]      # the first edge is the line going to idle high
    t_start, lvl, i, bits, prev = e[0][0], 1, 0, [], 1
    end = e[-1][0]
    while t_start + i * period + period // 2 <= end:
        t = t_start + i * period + period // 2
        cur = 1
        for c, v in e:
            if c > t:
                break
            cur = v
        bits.append(int(cur == prev))
        prev = cur
        i += 1
    s = ''.join(map(str, bits)).replace('', '')
    flag = '01111110'
    first = s.find(flag)
    last = s.rfind(flag)
    body = s[first + 8:last]
    unstuffed, run, k = [], 0, 0
    while k < len(body):
        x = body[k]
        unstuffed.append(int(x))
        run = run + 1 if x == '1' else 0
        k += 1
        if run == 5:
            k += 1
            run = 0
    data = [sum(unstuffed[j + m] << m for m in range(8)) for j in range(0, len(unstuffed) - 7, 8)]
    fcs_ok = len(data) == len(payload) + 2 and data[:-2] == payload \
        and data[-2] | data[-1] << 8 == x25(payload)
    uneven = [c for c, _ in e if (c - e[0][0]) % period]
    check('hdlc frame: NRZI, stuffing, CRC-16/X-25 all in firmware', fcs_ok and not uneven
          and chip.t[0].latecount == 0, f'{n} words, {len(body)} line bits for {len(data)} bytes')
    # the host posts the bit time long after the line went idle: section 3.5, no LATE
    for wait in (5000, 40000):
        chip, b, _ = boot('hdlc_tx')
        chip.run(wait, b)
        chip.post(7, period)
        chip.run(60, b)
        feed(chip, b, payload, budget=20000)
        check(f'hdlc: bit time posted {wait} cycles after start, no LATE', chip.t[0].latecount == 0, chip.t[0].latecount)


class Manchester:
    def __init__(self, bit, period, byte, start):
        self.bit, self.p, self.start = bit, period, start
        self.bits = [1] + [(byte >> i) & 1 for i in range(8)]

    def step(self, c, b):
        t = (c - self.start) / self.p
        v = 0
        if 0 <= t < len(self.bits):
            i, f = int(t), t - int(t)
            v = self.bits[i] if f >= 0.5 else 1 - self.bits[i]
        set_ui(b, self.bit, v)


def test_manchester():
    LOG.group(*P('manch_rx'), title='Manchester receiver, fixed format')
    for err in (0.0, 0.05, -0.05, 0.10, -0.10):
        res = []
        for byte in (0x00, 0xFF, 0xA5, 0x3C, 0x81):
            chip, b, n = boot('manch_rx')
            b.ui &= ~(1 << 6)
            chip.post(7, 150)                 # 3/4 of a nominal 200-cycle bit
            b.devs.append(Manchester(6, 200 * (1 + err), byte, 300))
            got = None
            for _ in range(3000):
                chip.cycle(b)
                v = chip.take(1)
                if v is not None:
                    got = v
                    break
            res.append(got == byte)
        check(f'manchester RX, sender clock {err:+.0%}', all(res), f'{n} words')


class Pulses:
    def __init__(self, bit, times):
        self.bit, self.times, self.v = bit, sorted(times), 0

    def step(self, c, b):
        if self.times and c == self.times[0]:
            self.times.pop(0)
            self.v ^= 1
            set_ui(b, self.bit, self.v)


def test_reaction():
    LOG.group(*P('reflex'), title='reaction time, pad to pad')
    best = {}
    for tid in range(4):
        chip, b, n = boot('reflex', tid)
        b.ui &= ~(1 << 5)
        rises = [1000 + 100 * i + i % 7 for i in range(1, 40, 2)]
        p = Pulses(5, sorted(rises + [r + 50 for r in rises]))
        b.devs.append(p)
        chip.run(4000, b)
        a = [c for c, v in b.edges('uo4') if v]
        s = [c for c, v in b.edges('uo5') if v]
        best[tid] = ({x - r for x, r in zip(a, rises)}, {x - r for x, r in zip(s, rises)})
    pre = set().union(*[v[0] for v in best.values()])
    fw = set().union(*[v[1] for v in best.values()])
    check('pre-armed reaction is exactly 3 cycles pad to pad, every thread and phase', pre == {3}, pre)
    LOG.info(f'firmware reaction (next instruction after the wait): {min(fw)}..{max(fw)} cycles')


ORG = dict(tx=0, rx=15, spi=37, i2c=60)
DEFS = dict(tx={'MCFG': 7, 'MIN': 0}, rx={'MCFG': 7, 'MOUT': 1},
            spi={'MCFG': 7, 'MIN': 2, 'MOUT': 3}, i2c={'MCFG': 7, 'MIN': 4, 'MOUT': 5})
NAME = dict(tx='uart_tx', rx='uart_rx', spi='spi', i2c='i2c_write')


def image():
    words = {}
    for k in ORG:
        a = assemble(read('standard/%s.s' % NAME[k]), defines=DEFS[k], base=ORG[k], depth=128)
        assert not set(a.words) & set(words), k
        words.update(a.words)
    return words


def scenario(which, cycles=30000):
    words = image()
    chip, b = Chip(words, 128), Board()
    b.devs += [UartSource(3, 100, [0x31, 0xC7, 0x05], 210), SpiTarget(0, 0, 0x96), I2cTarget()]
    b.ui &= ~(1 << 4)
    seq = [(0, 'tx', 0, 7, 434), (40, 'rx', 1, 7, 100), (80, 'spi', 2, 7, 10),
           (120, 'i2c', 3, 7, 45), (160, 'i2c', None, 7, 37)]
    posts = {t: (k, tid, m, v) for t, k, tid, m, v in seq}
    bytes_at = {300: ('tx', 0, 0x55), 400: ('spi', 2, 0xD2), 500: ('i2c', 4, 0xA03C),
                9000: ('tx', 0, 0xA3), 9100: ('spi', 2, 0x3C), 12000: ('i2c', 4, 0x7E81)}
    rx = []
    for c in range(cycles):
        if c in posts and posts[c][0] in which:
            k, tid, m, v = posts[c]
            if tid is not None:
                chip.start(tid, ORG[k])
            chip.post(m, v)
        if c in bytes_at and bytes_at[c][0] in which:
            chip.post(bytes_at[c][1], bytes_at[c][2])
        chip.cycle(b)
        v = chip.take(1)
        if v is not None:
            rx.append((c, v))
    return chip, b, rx


def test_noninterference():
    LOG.group(*P('uart_tx', 'uart_rx', 'spi', 'i2c_write'), title='four programs in one 128-word image, one thread each')
    words = image()
    check('four-protocol image fits the 128-word fallback', max(words) < 128, f'{len(words)} words')
    full, bf, rxf = scenario(('tx', 'rx', 'spi', 'i2c'))
    late = [t.latecount for t in full.t]
    pins = dict(tx={'uo0'}, spi={'uo1', 'uo2', 'uo5'}, i2c={'io0', 'io1'})
    for k, names in pins.items():
        _, b1, _ = scenario((k,))
        a = [e for e in bf.log if e[1] in names]
        c = [e for e in b1.log if e[1] in names]
        check(f'{k} pad trace identical to the cycle, alone vs all four running', a == c and len(a) > 10,
              f'{len(a)} edges')
    _, _, rx1 = scenario(('rx',))
    check('rx mailbox writes identical to the cycle, alone vs all four running', rx1 == rxf and len(rxf) == 3, rxf)
    check('no late ticks with four threads running', not any(late), late)
    LOG.group(*P('memstress', 'uart_tx'), title='a thread hammering memory next to the UART transmitter')
    words_mem = dict(words)
    a = assemble(read('standard/memstress.s'), base=102, depth=128)
    words_mem.update(a.words)
    chip, b = Chip(words_mem, 128), Board()
    chip.start(0, ORG['tx'])
    chip.start(3, 102)
    chip.post(7, 434)
    chip.run(300, b)
    chip.post(0, 0x55)
    chip.run(6000, b)
    chip2, b2 = Chip(words, 128), Board()
    chip2.start(0, ORG['tx'])
    chip2.post(7, 434)
    chip2.run(300, b2)
    chip2.post(0, 0x55)
    chip2.run(6000, b2)
    check('thread hammering ld/st changes nothing on another thread', b.log == b2.log and chip.t[3].r[5] == 0,
          f'{len(b.log)} edges, data errors {chip.t[3].r[5]}, loops {chip.t[3].r[6]}')


# IEEE 1149.1 TAP: state -> (next if TMS=0, next if TMS=1)
TAP = {
    'TLR': ('IDLE', 'TLR'), 'IDLE': ('IDLE', 'SELDR'), 'SELDR': ('CAPDR', 'SELIR'),
    'CAPDR': ('SHDR', 'EX1DR'), 'SHDR': ('SHDR', 'EX1DR'), 'EX1DR': ('PAUDR', 'UPDR'),
    'PAUDR': ('PAUDR', 'EX2DR'), 'EX2DR': ('SHDR', 'UPDR'), 'UPDR': ('IDLE', 'SELDR'),
    'SELIR': ('CAPIR', 'TLR'), 'CAPIR': ('SHIR', 'EX1IR'), 'SHIR': ('SHIR', 'EX1IR'),
    'EX1IR': ('PAUIR', 'UPIR'), 'PAUIR': ('PAUIR', 'EX2IR'), 'EX2IR': ('SHIR', 'UPIR'),
    'UPIR': ('IDLE', 'SELDR'),
}
IDCODE, BYPASS = 0b0001, 0b1111


class Tap:
    def __init__(self, idcode, tck=1, tms=2, tdi=5, tdo=4):
        self.id, self.tck, self.tms, self.tdi, self.tdo = idcode, tck, tms, tdi, tdo
        self.state, self.ir, self.ir_sh, self.dr, self.prev, self.visited = 'IDLE', 0xA, 0, 0, 0, set()

    def step(self, c, b):
        tck, tms, tdi = (b.uo >> self.tck) & 1, (b.uo >> self.tms) & 1, (b.uo >> self.tdi) & 1
        if tck and not self.prev:
            s = self.state
            if s == 'CAPDR':
                self.dr = self.id if self.ir == IDCODE else 0
            elif s == 'SHDR':
                n = 32 if self.ir == IDCODE else 1
                self.dr = (self.dr >> 1) | tdi << (n - 1)
            elif s == 'CAPIR':
                self.ir_sh = 0b0001
            elif s == 'SHIR':
                self.ir_sh = (self.ir_sh >> 1) | tdi << 3
            elif s == 'UPIR':
                self.ir = self.ir_sh
            self.state = TAP[s][tms]
            if self.state == 'TLR':
                self.ir = IDCODE
            self.visited.add(self.state)
        if self.prev and not tck:
            if self.state == 'SHDR':
                set_ui(b, self.tdo, self.dr & 1)
            elif self.state == 'SHIR':
                set_ui(b, self.tdo, self.ir_sh & 1)
        self.prev = tck


def jtag_once(half, tid, ident, pattern):
    chip, b, n = boot('jtag', tid)
    dev = Tap(ident)
    b.devs.append(dev)
    b.ui &= ~(1 << 4)
    chip.post(7, half)
    posted = False
    out = []
    for _ in range(200 * half + 4000):
        chip.cycle(b)
        v = chip.take(1)
        if v is not None:
            out.append(v)
        if len(out) == 2 and not posted:
            posted = chip.post(0, pattern)
        if not chip.t[tid].run:
            chip.run(8, b)
            break
    return chip, dev, out, n


def test_jtag():
    LOG.group(*P('jtag'), title='JTAG controller against a TAP model')
    ident = 0x4BA00477
    pattern = 0xB3C5
    for half in (25, 16):
        for tid in range(4):
            chip, dev, out, n = jtag_once(half, tid, ident, pattern)
            want = [ident & 0xFFFF, ident >> 16, (pattern << 1) & 0xFFFF]
            ok = out == want and dev.state == 'IDLE' and dev.ir == BYPASS and chip.t[tid].latecount == 0
            check(f'jtag half={half} thread={tid}: IDCODE, IR load, BYPASS loop', ok,
                  f'{n} words, {[hex(x) for x in out]}, state {dev.state}, late {chip.t[tid].latecount}')


MS = 50000      # cycles per ms at 50 MHz
BIT = int(2.4 * MS)     # HDMI CEC nominal data bit period
FREE7 = 7 * BIT         # HDMI CEC 1.4, CEC 9.1: signal free time before an initiator's own next frame
FREE5 = 5 * BIT         # and before a new initiator's frame, after another initiator's


class CecFollower:
    """Times every bit from the line and checks the HDMI CEC windows. Acknowledges blocks if the
    destination matches: pulls low from the fall of the ACK bit to 1.5 ms."""

    def __init__(self, pin, addr):
        self.pin, self.addr = pin, addr
        self.prev, self.fall, self.bits, self.blocks, self.errors = 1, None, [], [], []
        self.lowtime, self.started, self.release_at, self.mine = None, False, None, False
        self.first_of_block = True

    def step(self, c, b):
        lvl = (b.io >> self.pin) & 1
        own = (b.io_low >> self.pin) & 1
        if self.release_at == c:
            b.io_low &= ~(1 << self.pin)
            self.release_at = None
        if self.prev and not lvl and not own:
            if self.fall is not None:
                self.bit_done(c)
            self.fall = c
            if self.started and len(self.bits) == 9 and self.mine:
                b.io_low |= 1 << self.pin           # our ACK
                self.release_at = c + int(1.5 * MS)
        if not self.prev and lvl and not own and self.fall is not None and self.lowtime is None:
            self.lowtime = c - self.fall
        self.prev = lvl

    def finish(self):
        """The last ACK slot has no following edge to close it."""
        if len(self.bits) == 9:
            byte = sum(x << (7 - i) for i, x in enumerate(self.bits[:8]))
            self.blocks.append((byte, self.bits[8]))

    def bit_done(self, c):
        total, low = (c - self.fall) / MS, (self.lowtime or 0) / MS
        self.lowtime = None
        if not self.started:
            if not (3.5 <= low <= 3.9 and 4.3 <= total <= 4.7):
                self.errors.append(('start', low, total))
            self.started = True
            return
        if len(self.bits) == 9:             # ACK slot: only total time is the initiator's
            if not 2.05 <= total <= 2.75:
                self.errors.append(('ack', low, total))
            byte = sum(x << (7 - i) for i, x in enumerate(self.bits[:8]))
            self.blocks.append((byte, self.bits[8]))
            self.bits = []
            return
        val = 1 if low < 1.0 else 0
        lim = (0.4, 0.8) if val else (1.3, 1.7)
        if not lim[0] <= low <= lim[1] or not 2.05 <= total <= 2.75:
            self.errors.append((val, low, total))
        self.bits.append(val)
        if len(self.bits) == 8 and len(self.blocks) == 0:
            byte = sum(x << (7 - i) for i, x in enumerate(self.bits))
            self.mine = (byte & 15) == self.addr


def cec_run(addr, tid):
    chip, b, n = boot('cec', tid)
    dev = CecFollower(3, addr)
    b.devs.append(dev)
    header, data = 0x10 | 4, 0x36           # from device 1 to device 4: <Standby>
    words = [header << 8 | 0x40, data << 8 | 0xC0]
    chip.run(40, b)
    res, i = None, 0
    for _ in range(100 * MS):               # the frame waits out 7 bit periods of free line first
        chip.cycle(b)
        if i < 2 and chip.post(0, words[i]):
            i += 1
        res = chip.take(1)
        if res is not None:
            break
    chip.run(3 * MS, b)
    dev.finish()
    return chip, b, dev, res, n


def cec_four(addr):
    """Four copies at once, one per thread, each with its own pin (io0..io3), mailbox words and
    follower. A CEC frame lasts 50 ms, so this runs the four threads in the time of one."""
    words, n = {}, 0
    for tid in range(4):
        img = assemble(read('standard/cec.s'), defines={'CEC': 'io%d' % tid, 'MIN': tid, 'MOUT': 4 + tid},
                       base=64 * tid, depth=512)
        words.update(img.words)
        n = img.size
    chip, b = Chip(words, 512), Board()
    devs = [CecFollower(tid, addr) for tid in range(4)]
    b.devs += devs
    for tid in range(4):
        chip.start(tid, 64 * tid)
    header, data = 0x10 | 4, 0x36           # from device 1 to device 4: <Standby>
    blocks = [header << 8 | 0x40, data << 8 | 0xC0]
    chip.run(40, b)
    sent, res = [0] * 4, [None] * 4
    for _ in range(100 * MS):               # the frame waits out 7 bit periods of free line first
        chip.cycle(b)
        for tid in range(4):
            if sent[tid] < 2 and chip.post(tid, blocks[sent[tid]]):
                sent[tid] += 1
            if res[tid] is None:
                res[tid] = chip.take(4 + tid)
        if None not in res:
            break
    chip.run(3 * MS, b)
    for d in devs:
        d.finish()
    return chip, b, devs, res, n


class CecNode:
    """A follower at addr that reads every frame on its line, whoever sends it. A start bit is a low of 3.5 to
    3.9 ms in a pulse of 4.3 to 4.7 ms; every bit after it must meet the windows CecFollower checks. It holds the
    line low from an ACK bit's fall to 1.5 ms to acknowledge each block of a frame addressed to it, except the
    blocks named in refuse as (frame, block), counted from 0. A frame to destination 15 is a broadcast, where
    HDMI CEC 1.4 inverts the ACK bit: there it pulls the slot low only to reject, the blocks in refuse. frames: one
    dict per frame with the start bit's fall, how long the line had been high before it (from cycle 0 if it never
    fell), the nominal end of its last bit, and its blocks as (byte, EOM, ACK bit), the ACK bit being the line's
    level: 0 acknowledges a direct block and rejects a broadcast one."""

    def __init__(self, pin, addr, refuse=()):
        self.pin, self.addr, self.refuse = pin, addr, set(refuse)
        self.prev, self.rise, self.fall, self.low, self.free = 1, 0, None, None, None
        self.frames, self.errors = [], []
        self.cur, self.bits, self.release_at = None, [], None

    def step(self, c, b):
        lvl = (b.io >> self.pin) & 1
        if self.release_at == c:
            b.io_low &= ~(1 << self.pin)
            self.release_at = None
        if self.prev and not lvl:
            self.pulse(c)
            self.fall, self.low, self.free = c, None, c - self.rise
            if self.cur is not None and len(self.bits) == 9 and self.pulls():
                b.io_low |= 1 << self.pin
                self.release_at = c + int(1.5 * MS)
        elif not self.prev and lvl:
            self.rise = c
            if self.fall is not None and self.low is None:
                self.low = c - self.fall
        self.prev = lvl

    def pulls(self):
        blocks = self.cur['blocks']
        head = blocks[0][0] if blocks else sum(x << (7 - i) for i, x in enumerate(self.bits[:8]))
        refused = (len(self.frames) - 1, len(blocks)) in self.refuse
        return refused if head & 15 == 15 else (head & 15) == self.addr and not refused

    def pulse(self, nxt):
        """Reads the pulse from the last fall to nxt, the next fall (None at the end of the run)."""
        if self.fall is None or self.low is None:
            return
        low, total = self.low / MS, None if nxt is None else (nxt - self.fall) / MS
        if 3.5 <= low <= 3.9 and (total is None or 4.3 <= total <= 4.7):
            if self.cur is not None:
                self.errors.append(('start bit inside a frame', self.fall))
            self.cur = dict(start=self.fall, free=self.free, end=None, blocks=[])
            self.frames.append(self.cur)
            self.bits = []
            return
        if self.cur is None:
            self.errors.append(('pulse outside a frame', self.fall, low, total))
            return
        val = 1 if low < 1.0 else 0
        blocks = self.cur['blocks']
        head = blocks[0][0] if blocks else sum(x << (7 - i) for i, x in enumerate(self.bits[:8]))
        last = len(self.bits) == 9 and (self.bits[8] or val != (head & 15 == 15))  # EOM, or the block failed
        if len(self.bits) < 9:                                      # the ACK slot's low time is the follower's
            lim = (0.4, 0.8) if val else (1.3, 1.7)
            if not lim[0] <= low <= lim[1]:
                self.errors.append(('low', self.fall, val, low))
        if total is not None and not last and not 2.05 <= total <= 2.75:
            self.errors.append(('total', self.fall, total))
        self.bits.append(val)
        if len(self.bits) == 10:
            byte = sum(x << (7 - i) for i, x in enumerate(self.bits[:8]))
            self.cur['blocks'].append((byte, self.bits[8], self.bits[9]))
            self.bits = []
            if last:
                self.cur['end'] = self.fall + BIT
                self.cur = None

    def finish(self):
        self.pulse(None)


class CecSender:
    """Another initiator on the same line: one frame of blocks [(byte, EOM)] from cycle t0 at the nominal HDMI
    CEC timing, with its ACK slots released. It doesn't look at the line, and moves it only at its own edges."""

    def __init__(self, pin, t0, blocks):
        self.bit, self.edges, t = 1 << pin, {}, t0
        bits = [None] + [x for byte, eom in blocks for x in [(byte >> (7 - i)) & 1 for i in range(8)] + [eom, 1]]
        for v in bits:
            self.edges[t] = 1
            self.edges[t + int((3.7 if v is None else 0.6 if v else 1.5) * MS)] = 0
            t += int(4.5 * MS) if v is None else BIT

    def step(self, c, b):
        e = self.edges.get(c)
        if e == 1:
            b.io_low |= self.bit
        elif e == 0:
            b.io_low &= ~self.bit


def cec_frames(frames, refuse=(), other=None, tail=6 * MS, more=()):
    """The program on all four threads at once, thread t on io<t> with MIN t and MOUT 4 + t, and a CecNode at 4
    on each line, refusing the blocks in refuse; more: (address, refuse) of further followers on each line. The
    host posts each thread the blocks of frames in order, each as soon as MIN is empty, and takes the answers.
    other: (cycle, blocks) of another initiator's frame on every line. Runs until each thread has answered once
    per frame, then tail cycles more. nodes[t] is the follower at 4 on line t."""
    words, n = {}, 0
    for tid in range(4):
        img = assemble(read('standard/cec.s'), defines={'CEC': 'io%d' % tid, 'MIN': tid, 'MOUT': 4 + tid},
                       base=64 * tid, depth=512)
        words.update(img.words)
        n = img.size
    chip, b = Chip(words, 512), Board()
    nodes = [CecNode(tid, 4, refuse) for tid in range(4)]
    b.devs += nodes
    b.devs += [CecNode(tid, a, r) for a, r in more for tid in range(4)]
    if other:
        b.devs += [CecSender(tid, *other) for tid in range(4)]
    for tid in range(4):
        chip.start(tid, 64 * tid)
    queue = [[w for fr in frames for w in fr] for _ in range(4)]
    answers = [[] for _ in range(4)]
    for _ in range(180 * MS):
        chip.cycle(b)
        for tid in range(4):
            if queue[tid] and chip.post(tid, queue[tid][0]):
                queue[tid].pop(0)
            v = chip.take(4 + tid)
            if v is not None:
                answers[tid].append(v)
        if all(len(a) >= len(frames) for a in answers):
            break
    chip.run(tail, b)
    for nd in nodes:
        nd.finish()
    return chip, b, nodes, answers, queue, n


def free_before(b, pin, t):
    """How long the line had been high before the fall at cycle t, from cycle 0 if it never fell before."""
    rises = [c for c, v in b.edges('io%d' % pin) if v and c < t]
    return t - (rises[-1] if rises else 0)


def test_cec():
    LOG.group(*P('cec'), title='HDMI CEC initiator')
    chip, b, devs, res, n = cec_four(4)
    for tid in range(4):
        dev = devs[tid]
        ok = res[tid] == 0 and dev.blocks == [(0x14, 0), (0x36, 1)] \
            and not dev.errors and chip.t[tid].latecount == 0 and b.chip_high_on_od == 0
        falls = [c for c, v in b.edges('io%d' % tid) if not v]
        spans = {y - x for x, y in zip(falls[1:], falls[2:])}
        check(f'cec frame acknowledged, all bit windows met, thread={tid} on io{tid}, all four threads at once',
              ok and spans == {int(2.4 * MS)},
              f'{n} words, res={res[tid]}, blocks={dev.blocks[:2]}, bit spans {spans} cycles, late {chip.t[tid].latecount}')
    free = [free_before(b, tid, ([c for c, v in b.edges('io%d' % tid) if not v] or [0])[0]) for tid in range(4)]
    check('cec signal free time: the start bit follows 7 bit periods (16.8 ms) of high line, on all four threads',
          all(f >= FREE7 for f in free), 'line high %s ms before each start bit' % [round(f / MS, 2) for f in free])
    chip, b, dev, res, n = cec_run(9, 0)
    check('cec no follower: reports not acknowledged, stops after the header', res == 1 and len(dev.blocks) >= 1,
          f'res={res}, blocks={dev.blocks}')

    # A NACK in the middle of a frame: the follower refuses the frame's second block. The third, with EOM, must
    # leave MIN unsent, and the next frame must go out whole, after the signal free time.
    a = [0x14 << 8 | 0x40, 0x36 << 8 | 0x40, 0x55 << 8 | 0xC0]
    b2 = [0x14 << 8 | 0x40, 0x83 << 8 | 0xC0]
    chip, b, nodes, ans, queue, n = cec_frames([a, b2], refuse={(0, 1)})
    want = [[(0x14, 0, 0), (0x36, 0, 1)], [(0x14, 0, 0), (0x83, 1, 0)]]
    for tid in range(4):
        nd = nodes[tid]
        fr = [f['blocks'] for f in nd.frames]
        gap = nd.frames[1]['start'] - nd.frames[0]['end'] if len(nd.frames) == 2 and nd.frames[0]['end'] else None
        ok = fr == want and ans[tid] == [1, 0] and not nd.errors and not queue[tid] and not chip.full[tid] \
            and chip.t[tid].latecount == 0 and b.chip_high_on_od == 0
        check(f'cec block refused mid-frame, thread={tid}: answer 1, the frame\'s last block taken unsent, '
              'then the next frame whole and acknowledged', ok,
              f'frames {fr}, answers {ans[tid]}, errors {nd.errors[:2]}, late {chip.t[tid].latecount}')
        check(f'cec signal free time, thread={tid}: 7 bit periods of high line before both start bits, and 7 '
              'after the end of the refused frame', gap is not None and gap >= FREE7
              and all(f['free'] >= FREE7 for f in nd.frames),
              'line high %s ms; %s ms after the first frame' % ([round(f['free'] / MS, 2) for f in nd.frames],
                                                                 gap and round(gap / MS, 2)))

    # Broadcast (destination 15): HDMI CEC 1.4 inverts the ACK bit, and a follower pulls the slot low to reject.
    # Two followers, at 4 and 5, on each line. First a frame both accept, then one that 5 rejects in its second
    # block, whose last block must leave MIN unsent, then a direct frame, which 4 acknowledges by pulling low.
    bc = [0x1F << 8 | 0x40, 0x36 << 8 | 0xC0]
    chip, b, nodes, ans, queue, n = cec_frames([bc], more=[(5, ())])
    for tid in range(4):
        nd = nodes[tid]
        fr = [f['blocks'] for f in nd.frames]
        ok = fr == [[(0x1F, 0, 1), (0x36, 1, 1)]] and ans[tid] == [0] and not nd.errors and not chip.full[tid] \
            and all(f['free'] >= FREE7 for f in nd.frames) and chip.t[tid].latecount == 0 and b.chip_high_on_od == 0
        check(f'cec broadcast that every follower accepts (slots left high), thread={tid}: answer 0, the frame whole',
              ok, f'frames {fr}, answers {ans[tid]}, errors {nd.errors[:2]}, late {chip.t[tid].latecount}')
    a = [0x1F << 8 | 0x40, 0x36 << 8 | 0x40, 0x55 << 8 | 0xC0]
    chip, b, nodes, ans, queue, n = cec_frames([a, [0x14 << 8 | 0x40, 0x83 << 8 | 0xC0]], more=[(5, {(0, 1)})])
    for tid in range(4):
        nd = nodes[tid]
        fr = [f['blocks'] for f in nd.frames]
        ok = fr == [[(0x1F, 0, 1), (0x36, 0, 0)], [(0x14, 0, 0), (0x83, 1, 0)]] and ans[tid] == [1, 0] \
            and not nd.errors and not queue[tid] and not chip.full[tid] and all(f['free'] >= FREE7 for f in nd.frames) \
            and chip.t[tid].latecount == 0 and b.chip_high_on_od == 0
        check(f'cec broadcast that a follower rejects in its second block, thread={tid}: answer 1, the last block '
              'taken unsent; then a direct frame whole', ok,
              f'frames {fr}, answers {ans[tid]}, errors {nd.errors[:2]}, late {chip.t[tid].latecount}')

    # Another initiator's frame (from 3 to 5, nobody acknowledges) starts 5 ms into the program's count: the
    # program's start bit must follow 7 bit periods of high line after it, and 5 after its end (CEC 9.1).
    other = (5 * MS, [(0x35, 1)])
    chip, b, nodes, ans, queue, n = cec_frames([[0x14 << 8 | 0x40, 0x36 << 8 | 0xC0]], other=other)
    for tid in range(4):
        nd = nodes[tid]
        fr = [f['blocks'] for f in nd.frames]
        ok = fr == [[(0x35, 1, 1)], [(0x14, 0, 0), (0x36, 1, 0)]] and ans[tid] == [0] and not nd.errors \
            and chip.t[tid].latecount == 0 and b.chip_high_on_od == 0
        mine = nd.frames[1] if len(nd.frames) == 2 and nd.frames[0]['end'] else None
        after = mine and mine['start'] - nd.frames[0]['end']
        check(f'cec another initiator\'s frame during the count, thread={tid}: the program waits it out, then its '
              'frame goes out whole', ok and mine and mine['free'] >= FREE7 and after >= FREE5,
              f'frames {fr}, answers {ans[tid]}, errors {nd.errors[:2]}, line high '
              f'{mine and round(mine["free"] / MS, 2)} ms, {mine and round(after / MS, 2)} ms after the other frame')


class Wire:
    """io0 tied to io2 and io1 tied to io3, with pull-ups. Logs bus changes for the timing checks."""
    def __init__(self):
        self.trace, self.highs = [], 0

    def step(self, chip, c, uo, en, val):
        low = en & ~val & 0xFF
        self.highs += bin(en & val & 0x0F).count('1')
        sda = 0 if low & 0b0101 else 1
        scl = 0 if low & 0b1010 else 1
        lvl = 0xF0 | sda | sda << 2 | scl << 1 | scl << 3
        s = (scl, sda)
        if not self.trace or self.trace[-1][1] != s:
            self.trace.append((c, s))
        return 0xFF, lvl


def build():
    words = {}
    ctl = assemble(read('standard/i2c.s'), defines={'MCFG': 7, 'MIN': 0, 'MOUT': 1}, depth=512)
    words.update(ctl.words)
    tgt = assemble(read('standard/i2c_target.s'), base=100, depth=512)
    assert not set(ctl.words) & set(tgt.words)
    words.update(tgt.words)
    return words, len(ctl.words), len(tgt.words)


def run(cfg, ops, seed=1):
    """ops: list of mailbox words for the controller, grouped per transaction."""
    words, _, n = build()
    chip, env = Chip(words, 512, seed), Wire()
    for i in range(256):
        chip.sim.host_wrmem(0x100 + i, 0xEE)
    chip.sim.host_wrmem(0x120, 0x3C)
    chip.start(1, 100)
    chip.run(100, env)
    chip.start(0, 0)
    for v in cfg:
        while not chip.post(7, v):
            chip.cycle(env)
    res = []
    for op in ops:
        for w in op:
            for _ in range(60000):          # a program that no longer takes words: the op gets no replies
                if chip.post(0, w):
                    break
                chip.cycle(env)
        got = []
        for _ in range(60000):
            chip.cycle(env)
            v = chip.take(1)
            if v is not None:
                got.append(v & 0xFF)
                if len(got) == 2:
                    break
        res.append(got)
    chip.run(2000, env)
    return chip, env, res


def check_target(log):
    log.group(*P('i2c', 'i2c_target'), title='I2C controller and I2C target firmware on two threads of one chip')
    words, nc, nt = build()
    log.info(f'controller {nc} words, target {nt} words, storage 256 words, total {nc + nt + 256} of 512')
    W = [0xA010, 0x5A00]
    R = [0xA110]
    R2 = [0xA120]
    BAD = [0xB010, 0x5A00]
    for name, cfg, limits in (('100 kHz', (250, 210), dict(low=4700, high=4000, su_sta=4700, su_sto=4000, hd_sta=4000)),
                              ('400 kHz', (45, 37), dict(low=1300, high=600, su_sta=600, su_sto=600, hd_sta=600))):
        chip, env, res = run(cfg, [W, R, R2, BAD])
        ok = res[0][0] == 0 and chip.mem(0x110) == 0x5A and res[1] == [0, 0x5A] and res[2] == [0, 0x3C] and res[3][0] == 1
        late = [t.latecount for t in chip.t]
        t = i2c_eeprom.timing(env.trace)
        check(f'{name}: write, random read of it, random read of a stored byte, wrong address NACK',
              ok and not any(late) and env.highs == 0, (res, late))
        check(f'{name}: bus intervals inside UM10204', all(t[k] >= v for k, v in limits.items()), t)
        chip, env, res = run(cfg, [BAD, W, R])
        ok = res[0][:1] == [1] and res[1][:1] == [0] and res[2] == [0, 0x5A] and chip.mem(0x110) == 0x5A \
            and not chip.full[0]
        late = [t.latecount for t in chip.t]
        check(f'{name}: wrong address, then a write and a random read of it: the NACKed write\'s data word is '
              'taken, not run', ok and not any(late) and env.highs == 0, (res, late))


def test_i2c_words():
    LOG.group(*P('i2c'), title='I2C controller against bench/i2c_eeprom.py: the host\'s words after a NACK or a '
              'bus fault')
    check('wrong address, then a write and a random read: the NACKed write\'s data word is taken, not run',
          *i2c_eeprom.after_nack('standard/i2c.s'))
    check('bus fault on a write (SDA held low past the timeout), then a write: the first one\'s data word is taken',
          *i2c_eeprom.after_fault('standard/i2c.s'))


def main(log):
    test_uart_tx()
    test_uart_rx()
    test_spi()
    test_i2c()
    test_ws2812()
    test_onewire()
    test_crc()
    test_hdlc()
    test_manchester()
    test_reaction()
    test_noninterference()
    test_jtag()
    test_cec()
    check_target(log)
    test_i2c_words()
