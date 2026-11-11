"""Benches for the programs in extra/: an SPI target, 8-bit parallel capture, an SWD read, the I2C
controller against an EEPROM model, a LIN responder header and an NEC IR transmitter.

Each limit below says where it comes from. Where a protocol sets no number (SPI has no clock rate),
the bench uses the rate section 11.2 of docs/ISA.md states for the program.
"""
import i2c_eeprom
from report import check
from simple import Bus, Run, read

PROGRAMS = ('extra/spi_target.s', 'extra/parallel.s', 'extra/swd.s', 'extra/i2c.s', 'extra/lin.s',
            'extra/nec_tx.s')
CLK = 50_000_000


# ---- SPI target, mode 0 ---------------------------------------------------------------------
# Mode 0 as SPI defines it (Motorola SPI Block Guide): SCK idles low, both sides sample on the rising
# edge and change on the falling edge, CS low frames each byte. The controller model here samples
# MISO in the very cycle SCK rises at the pad, so the chip gets no setup margin at all.

def spi_target(half, phase, data):
    r = Run(read('extra/spi_target.s'))
    MISO = 2
    ev, t, samples = [], 200 + phase, []         # (cycle, cs, sck, mosi)
    for b in data:
        ev.append((t, 0, 0, (b >> 7) & 1))
        t += 2 * half + 20                      # CS falls, MOSI bit 7 set
        for k in range(8):
            ev.append((t, 0, 1, (b >> (7 - k)) & 1))
            samples.append(t)
            t += half                           # rising
            nxt = (b >> (6 - k)) & 1 if k < 7 else 0
            ev.append((t, 0, 0, nxt))
            t += half                           # falling, MOSI changes
        ev.append((t, 1, 0, 0))
        t += 3 * half + 60
    end = t + 400
    st = [0, (1, 0, 0), 0]                      # next event, (cs, sck, mosi), next sample
    miso_seen, out = [], []

    def w(c, uo, o, oe):
        while st[0] < len(ev) and ev[st[0]][0] <= c:
            st[1] = ev[st[0]][1:]
            st[0] += 1
        if st[2] < len(samples) and samples[st[2]] == c:
            miso_seen.append((uo >> MISO) & 1)
            st[2] += 1
        cs, sck, mosi = st[1]
        return 0xFF & ~(7 << 3) | cs << 3 | sck << 4 | mosi << 5, 0xFF
    while r.c < end:
        r.step(w)
        v = r.take()
        if v is not None:
            out.append(v & 0xFF)
    miso = [int(''.join(map(str, miso_seen[j:j + 8])), 2) for j in range(0, len(miso_seen), 8)]
    return out, miso, r.late()


def check_spi_target(log):
    log.group('extra/spi_target.s', title='SPI target, mode 0; MISO returns the previous byte')
    data = [0xA5, 0x3C, 0xFF, 0x00, 0x81, 0x7E]
    want_miso = [0] + data[:-1]
    for half in (10, 25, 250):
        bad = []
        for ph in range(12):
            out, miso, late = spi_target(half, ph, data)
            if out != data or miso != want_miso or late:
                bad.append((ph, out, miso, late))
        check('half period %d cycles (%.2f MHz SCK), 6 bytes, 12 start phases: bytes in, MISO at each rising edge'
              % (half, CLK / 2e6 / half), not bad, bad[:1] or '12 of 12')


# ---- 8-bit parallel capture ---------------------------------------------------------------------
# The program takes the byte on io at the rising edge of ui4. Against IEEE 1284 compatibility mode
# (Centronics) timing, ui4 plays nStrobe: data is set up at least 0.5 us (25 cycles) before nStrobe
# falls, nStrobe stays low at least 0.5 us, and data is held at least 0.5 us after nStrobe rises.

def par_events(data, strobe_at, valid_from, valid_to, low_from):
    """Times are offsets from each strobe rise. Outside its window the bus shows the inverted byte."""
    ev = []
    for b, t in zip(data, strobe_at):
        ev += [(t + valid_from, 'd', b), (t + valid_to + 1, 'd', ~b & 0xFF), (t + low_from, 's', 0), (t, 's', 1)]
    ev.sort(key=lambda e: (e[0], e[1] == 'd'))
    return ev


def par_run(ev, end):
    r = Run(read('extra/parallel.s'))
    st = [0, 0, 1]                              # next event, data, strobe
    out = []

    def w(c, uo, o, oe):
        while st[0] < len(ev) and ev[st[0]][0] <= c:
            _, kind, v = ev[st[0]]
            st[1 if kind == 'd' else 2] = v
            st[0] += 1
        return 0xEF | st[2] << 4, st[1]
    while r.c < end:
        r.step(w)
        v = r.take()
        if v is not None:
            out.append(v & 0xFF)
    return out, r.late()


def check_parallel(log):
    log.group('extra/parallel.s', title='8-bit parallel capture on the rising edge of ui4')
    data = [0xA5, 0x3C, 0xFF, 0x00, 0x81, 0x7E, 0x55, 0xAA]
    # IEEE 1284 minimums back to back: one byte every 1.5 us (75 cycles). The data windows tile exactly,
    # so the byte changes 25 cycles after one rise and is good until 25 cycles after the next.
    bad = []
    for ph in range(12):
        rises = [400 + ph + 75 * i for i in range(len(data))]
        out, late = par_run(par_events(data, rises, -50, 24, -25), rises[-1] + 1000)
        if out != data or late:
            bad.append((ph, out, late))
    check('IEEE 1284 minimums: setup 0.5 us, strobe low 0.5 us, hold 0.5 us, a byte every 1.5 us, 12 phases',
          not bad, bad[:1] or '12 of 12')
    # Section 11.2 claims: data may change the cycle after the strobe rises, and 28 cycles per byte work.
    bad = []
    for ph in range(12):
        rises = [300 + ph + 2000 * i for i in range(len(data))]
        out, late = par_run(par_events(data, rises, -20, 0, -1000), rises[-1] + 2000)
        if out != data or late:
            bad.append((ph, out, late))
    check('data good only from 20 cycles before the strobe rise to the rise itself (hold 0), 12 phases',
          not bad, bad[:1] or '12 of 12')
    bad = []
    for ph in range(8):
        rises = [300 + ph + 28 * i for i in range(len(data))]
        out, late = par_run(par_events(data, rises, -14, 13, -14), rises[-1] + 2000)
        if out != data or late:
            bad.append((ph, out, late))
    check('a byte every 28 cycles, strobe high 14 and low 14, data stable for the whole period, 8 phases',
          not bad, bad[:1] or '8 of 8')


# ---- SWD read of DPIDR -----------------------------------------------------------------------
# ARM Debug Interface v5 (ADIv5): a line reset is at least 50 clocks with SWDIO high; a request is
# Start 1, APnDP, RnW, A[2:3], even parity over those four, Stop 0, Park 1, sent LSB first, so a read
# of DPIDR is 0xA5; one turnaround cycle comes before the target's ACK (OK 001, WAIT 010, LSB first)
# and another after the read data and parity. The target model drives SWDIO 5 cycles after SWCLK rises.

DPIDR = 0x2BA01477


class SwdTarget:
    def __init__(s, mode='ok', tco=5):
        s.mode, s.tco = mode, tco
        s.prev = 0
        s.ones = 0
        s.state = 'wait'
        s.bits, s.pend, s.seq = [], [], []
        s.req = None
        s.idle_after = s.reset_seen = 0

    def rising(s, c, dio):
        if s.state == 'wait':
            s.ones = s.ones + 1 if dio else 0
            if s.ones >= 50:
                s.state = 'reset'
        elif s.state == 'reset':
            if not dio:
                s.state = 'idle'
                s.reset_seen += 1
        elif s.state == 'idle':
            if dio:
                s.state, s.bits = 'req', [1]
        elif s.state == 'req':
            s.bits.append(dio)
            if len(s.bits) == 8:
                s.req = sum(b << i for i, b in enumerate(s.bits))
                par = sum(s.bits[1:5]) & 1
                ok = s.bits[0] == 1 and s.bits[6] == 0 and s.bits[7] == 1 and par == s.bits[5]
                s.state = 'trn' if ok else 'wait'
        elif s.state == 'trn':                  # this rising is the turnaround clock: ACK follows
            ack = {'ok': [1, 0, 0], 'wait': [0, 1, 0], 'badpar': [1, 0, 0]}[s.mode]
            if s.mode == 'wait':
                s.seq = ack + [None]            # release after the ACK
            else:
                d = [(DPIDR >> i) & 1 for i in range(32)]
                p = sum(d) & 1
                if s.mode == 'badpar':
                    p ^= 1
                s.seq = ack + d + [p, None]
            s.state = 'data'
            s.pend.append((c + s.tco, s.seq.pop(0)))
        elif s.state == 'data':
            if s.seq:
                s.pend.append((c + s.tco, s.seq.pop(0)))
            else:
                s.state = 'after'
        elif s.state == 'after':
            if not dio:
                s.idle_after += 1

    def step(s, c, clk, dio, bus):
        if clk and not s.prev:
            s.rising(c, dio)
        s.prev = clk
        while s.pend and s.pend[0][0] <= c:
            v = s.pend.pop(0)[1]
            if v is None:
                bus.ext_oe &= ~1
            else:
                bus.ext_oe |= 1
                bus.ext_high = (bus.ext_high & ~1) | v


def swd_run(mode):
    r = Run(read('extra/swd.s'))
    bus, t, out = Bus(pull=0xFF), SwdTarget(mode), []

    def w(c, uo, o, oe):
        lvl = bus.level(o, oe)
        t.step(c, (uo >> 1) & 1, lvl & 1, bus)
        return 0xFF, bus.level(o, oe)
    while r.c < 30000:
        r.step(w)
        v = r.take()
        if v is not None:
            out.append(v)
    data = (out[0] | out[1] << 16) if len(out) >= 3 else None
    return dict(data=data, status=out[-1] if out else None, req=t.req, resets=t.reset_seen,
                contention=bus.contention, idle=t.idle_after, late=r.late())


def show(d):
    return ', '.join('%s %s' % (k, hex(v) if k in ('data', 'req') and v is not None else v) for k, v in d.items())


def check_swd(log):
    log.group('extra/swd.s', title='SWD host: line reset, read DPIDR')
    ok = swd_run('ok')
    check('line reset of at least 50 high clocks, then the request 0xA5 (read DPIDR), as ADIv5 frames it',
          ok['resets'] == 1 and ok['req'] == 0xA5, 'resets %s, request %s' % (ok['resets'], hex(ok['req'] or 0)))
    check('ACK OK: DPIDR read back, parity accepted, status 0, no contention at either turnaround, no LATE',
          ok['data'] == DPIDR and ok['status'] == 0 and ok['contention'] == 0 and ok['late'] == 0,
          show(ok))
    wt = swd_run('wait')
    check('ACK WAIT: reported as status 1, the bus released, no contention',
          wt['status'] == 1 and wt['contention'] == 0 and wt['late'] == 0, show(wt))
    bp = swd_run('badpar')
    check('read parity wrong: reported as status 2, no contention',
          bp['status'] == 2 and bp['contention'] == 0 and bp['late'] == 0, show(bp))


# ---- I2C controller against an EEPROM ----------------------------------------------------------
# Limits: the UM10204 minimums in i2c_eeprom.UM10204.

def check_i2c(log):
    log.group('extra/i2c.s', title='I2C controller against an EEPROM model (same code as standard/i2c.s)')
    rel = 'extra/i2c.s'
    W, RD, BAD = (0xA0, 0x10, 0x5A), (0xA1, 0x20, 0), (0xB0, 0x10, 0x5A)
    res, d, r, tr, bus = i2c_eeprom.run(rel, [W, (0xA1, 0x10, 0), RD])
    t = i2c_eeprom.timing(tr)
    check('100 kHz: write, then two random reads with repeated START',
          res == [0, 0x5A, 0, 0x5A, 0, 0x3C] and d.mem[0x10] == 0x5A and d.starts == 5 and d.stops == 3
          and r.late() == 0 and bus.contention == 0,
          '%s, starts %d, stops %d' % ([hex(x) for x in res], d.starts, d.stops))
    check('100 kHz intervals at the pad inside UM10204 Standard-mode', i2c_eeprom.within(t, 'Sm'), t)
    res, d, r, tr, bus = i2c_eeprom.run(rel, [BAD])
    check('wrong address: NACK reported, STOP sent', res[0] == 1 and d.stops == 1 and r.late() == 0, res)
    check('wrong address, then a write and a random read: the NACKed write\'s data word is taken, not run',
          *i2c_eeprom.after_nack(rel))
    check('bus fault on a write (SDA held low past the timeout), then a write: the first one\'s data word is taken',
          *i2c_eeprom.after_fault(rel))
    res, d, r, tr, bus = i2c_eeprom.run(rel, [W], nack_data=True)
    check('data NACK: reported, STOP sent', res[0] == 1 and d.stops == 1 and r.late() == 0, res)
    res, d, r, tr, bus = i2c_eeprom.run(rel, [(0xA1, 0x20, 0)], stretch=400)
    check('read with the EEPROM stretching 8 us after every SCL fall', res == [0, 0x3C] and r.late() == 0, res)

    def stuck(c, b, dv, lvl):
        b.ext_low |= 1 << i2c_eeprom.SDA
    res, d, r, tr, bus = i2c_eeprom.run(rel, [W], hook=stuck)
    check('SDA stuck low from reset: bus fault (status 2), no clocks sent', res[:1] == [2] and d.falls == 0,
          (res[:1], d.falls))

    def stuck2(c, b, dv, lvl):
        if dv.falls >= 27:                      # after the last ACK clock of a 3-byte write
            b.ext_low |= 1 << i2c_eeprom.SCL
    res, d, r, tr, bus = i2c_eeprom.run(rel, [W], hook=stuck2)
    check('SCL stuck low at STOP: bus fault (status 2)', res[:1] == [2], res[:1])
    res, d, r, tr, bus = i2c_eeprom.run(rel, [W, (0xA1, 0x20, 0)], fast=True)
    t = i2c_eeprom.timing(tr)
    check('400 kHz: write and random read, intervals inside UM10204 Fast-mode',
          res == [0, 0x5A, 0, 0x3C] and r.late() == 0 and i2c_eeprom.within(t, 'Fm'), (res, t))


# ---- LIN responder header ---------------------------------------------------------------------
# LIN 2.1: break at least 13 nominal bits dominant, a delimiter of at least 1 bit, the sync byte 0x55,
# then the protected identifier with P0 = ID0^ID1^ID2^ID4 and P1 = not(ID1^ID3^ID4^ID5). A responder
# that synchronises on the sync byte must work with its clock up to 14% off before synchronisation
# (F_TOL_UNSYNCH). Here the commander's bit time is off instead, by the same amounts, against the
# nominal 19200 baud (2604 cycles). Bad parity is reported as 0xFF.

def lin_pid(id6):
    b = [(id6 >> i) & 1 for i in range(6)]
    p0 = b[0] ^ b[1] ^ b[2] ^ b[4]
    p1 = 1 ^ (b[1] ^ b[3] ^ b[4] ^ b[5])
    return id6 | p0 << 6 | p1 << 7


def lin_frame(p, bits, t_start):
    e = [(t_start, 0)]
    t = t_start + 13 * bits
    e.append((round(t), 1))
    t += bits
    for byte in (0x55, p):
        for i, lvl in enumerate([0] + [(byte >> k) & 1 for k in range(8)] + [1]):
            e.append((round(t + i * bits), lvl))
        t += 10 * bits
    return e, t


def lin_run(frames, bits):
    r = Run(read('extra/lin.s'))
    e, t = [], 3000
    for p in frames:
        f, t = lin_frame(p, bits, round(t))
        e += f
        t += 5 * bits
    st = [0, 1]
    out = []

    def w(c, uo, o, oe):
        while st[0] < len(e) and e[st[0]][0] <= c:
            st[1] = e[st[0]][1]
            st[0] += 1
        return (0xFF & ~(1 << 3)) | st[1] << 3, 0xFF
    while r.c < t + 4000:
        r.step(w)
        v = r.take()
        if v is not None:
            out.append(v)
    return out, r.late()


def check_lin(log):
    log.group('extra/lin.s', title='LIN responder header with auto-baud')
    ids = [0x00, 0x3C, 0x3D, 0x15, 0x2A, 0x3F, 0x01, 0x20]
    frames = [lin_pid(x) for x in ids] + [lin_pid(0x15) ^ 0x40, lin_pid(0x2A) ^ 0x80]
    want = [lin_pid(x) for x in ids] + [0xFF, 0xFF]
    for err in (-0.14, 0.0, 0.14):
        out, late = lin_run(frames, 2604 * (1 + err))
        check('commander bit time %+d%% from 19200 baud: 8 identifiers right, 2 with bad parity rejected, no LATE'
              % round(err * 100), out == want and not late, [hex(x) for x in out] if out != want else '')


# ---- NEC IR transmitter ------------------------------------------------------------------------
# NEC format (uPD6121 datasheet): carrier 38 kHz at 1/3 duty; a 9 ms burst and a 4.5 ms space, then
# 32 bits LSB first, address, inverted address, command, inverted command; a bit is a 562.5 us burst
# and a 562.5 us space for 0 or a 1687.5 us space for 1; a final 562.5 us burst ends the frame.
# The bench allows 5% on every time, 2.5% on the carrier frequency and 3 points on the duty.

NEC_US = dict(leader=9000, lspace=4500, burst=562.5, zero=562.5, one=1687.5)


def nec_run(adr, cmd):
    r = Run(read('extra/nec_tx.s'))
    q = [adr, cmd]
    prev, edges = 0, []

    def w(c, uo, o, oe):
        return 0xFF, 0xFF
    while r.c < 3_600_000:
        if q and r.post(q[0]):
            q.pop(0)
        r.step(w)
        v = (r.chip.pads()[0] >> 2) & 1
        if v != prev:
            edges.append((r.c, v))
            prev = v
    return edges, r.late()


def near(x, want, tol=0.05):
    return abs(x - want) <= tol * want


def check_nec(log):
    log.group('extra/nec_tx.s', title='NEC IR transmitter, checked at the uo2 pad')
    edges, late = nec_run(0x59, 0x16)
    rises = [c for c, v in edges if v]
    falls = [c for c, v in edges if not v]
    period = sorted({b - a for a, b in zip(rises, rises[1:]) if b - a < 2000})
    highs = sorted({f - a for a, f in zip(rises, falls)})
    per = period[0] if len(period) == 1 else None
    khz = CLK / per / 1000 if per else 0
    duty = highs[0] / per if per and len(highs) == 1 else 0
    check('carrier 38 kHz at 1/3 duty', per and near(khz, 38.0, 0.025) and abs(duty - 1 / 3) <= 0.03 and not late,
          'periods %s cycles (%.2f kHz), high %s cycles (duty %.3f)' % (period, khz, highs, duty))
    bursts, s, last = [], rises[0], rises[0]    # a burst runs from its first rise to one period after its last rise
    for a in rises[1:]:
        if a - last > 2000:
            bursts.append((s, last + (per or 0)))
            s = a
        last = a
    bursts.append((s, last + (per or 0)))
    us = lambda n: n * 1e6 / CLK
    marks = [us(e - b) for b, e in bursts]
    spaces = [us(bursts[i + 1][0] - bursts[i][1]) for i in range(len(bursts) - 1)]
    ok = len(bursts) == 34 and near(marks[0], NEC_US['leader']) and near(spaces[0], NEC_US['lspace'])
    check('leader: 9 ms burst, 4.5 ms space', ok, 'burst %.1f us, space %.1f us' % (marks[0], spaces[0]))
    bits = [1 if sp > 1125 else 0 for sp in spaces[1:33]]
    ok = (len(bursts) == 34 and all(near(m, NEC_US['burst']) for m in marks[1:])
          and all(near(sp, NEC_US['one'] if b else NEC_US['zero']) for sp, b in zip(spaces[1:33], bits)))
    check('32 bit bursts and the final burst 562.5 us, spaces 562.5 or 1687.5 us', ok,
          'bursts %.1f..%.1f us, spaces %s us' % (min(marks[1:]), max(marks[1:]),
                                                  sorted({round(x, 1) for x in spaces[1:33]})))
    val = sum(b << i for i, b in enumerate(bits))
    want = 0x59 | (0xA6 << 8) | (0x16 << 16) | (0xE9 << 24)
    check('frame: address 0x59, its inverse, command 0x16, its inverse, LSB first', val == want and len(bits) == 32,
          hex(val))


def main(log):
    check_spi_target(log)
    check_parallel(log)
    check_swd(log)
    check_i2c(log)
    check_lin(log)
    check_nec(log)
