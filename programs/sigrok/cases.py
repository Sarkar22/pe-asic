"""The scenarios, one group per bench module, and what sigrok must decode from each.

Each scenario runs a bench's own case function or scenario code, with its device models, on the recording chip of
check.py. The expected payload is built from the scenario's stimulus: the words the host posts, the replies a
device model is given, a target's memory, the frames a sender model is told to send. The benches' decoders run on
the same record only as a cross-check. sigrok's options come from the protocol and from the rate the scenario
sets on the wire, never from a program's constants.
"""
import os

import check
from check import CLK, LOG, State, decode, trace_of

NOBENCH = object()


# ---------------------------------------------------------------------------------------------------------------
# a batch of runs reported as one line per kind of check

class Batch:
    """Runs of one setting. close() prints one line for sigrok against the stimulus, one for the bench's decoder
    against sigrok, and under --rtl one for the RTL's record against the simulator's."""

    def __init__(self, runner, title, unit='bytes'):
        self.runner, self.title, self.unit = runner, title, unit
        self.n = self.items = 0
        self.bad, self.xn, self.xbad, self.pn, self.pbad = [], 0, [], 0, []
        self.known = []
        self.changes = self.cycles = 0

    def run(self, name, fn, *a, **k):
        """One bench scenario. Under --rtl, also its records against the simulator's."""
        ctx = self.runner.run(fn, *a, **k)
        ctx.name = name
        ctx.same = True
        for p in LOG.progs:
            LOG.row(p).scenarios += 1
        if self.runner.rtl:
            self.pn += 1
            a = [check.record(c) for c in ctx.sim_chips]
            b = [check.record(c) for c in ctx.chips]
            if a != b:
                ctx.same = False
                self.pbad.append('%s: %s' % (name, check.first_difference(a, b)))
            self.changes += sum(len(r[0]) for r in b)
            self.cycles += sum(r[1] for r in b)
        return ctx

    def judge(self, d, extract, want, known=None):
        got, errs = extract(d)
        if callable(want):
            ok, detail = want(got, errs)
            ok = ok and not d.problems
        else:
            ok = got == want and not errs and not d.problems
            detail = 'decoded %s, wanted %s' % (short(got), short(want)) if got != want else ''
            if errs:
                detail += ' sigrok errors %s' % short(errs)
        if d.problems:
            detail += ' sigrok-cli: %s' % short(d.problems)
        reason = known(got, errs) if known and not ok and not d.problems else None
        return got, errs, ok, detail.strip(), reason

    def sigrok(self, ctx, stack, extract, want, bench=NOBENCH, chip=None, tag='', lo=0, hi=None, known=None,
               prog=None):
        """Decodes one record with one decoder stack. want is the expected payload, or a function of (payload,
        errors) giving (ok, detail) for runs where the stimulus plants a fault that sigrok must see. known, for a
        documented difference between sigrok and the protocol document, is a function of (payload, errors) that
        gives the reason when the run differs in exactly that way; such a run is reported apart, not failed. prog
        names the one program of a shared image whose bus this is (default: every program of the group)."""
        progs = ((prog,) if isinstance(prog, str) else tuple(prog)) if prog else LOG.progs
        name = ctx.name + (('_' + tag) if tag else '')
        chip = chip or ctx.chip
        vcd = self.runner.vcd(ctx, name, chip, lo, hi)
        d = decode(vcd, stack)
        got, errs, ok, detail, reason = self.judge(d, extract, want, known)
        if reason:
            self.known.append((os.path.basename(vcd), reason))
            LOG.known(os.path.relpath(vcd, self.runner.work), reason, detail, progs)
            ok = True
        mode = 'rtl' if self.runner.rtl else 'sim'
        LOG.outcome(mode, ok, progs)
        if self.runner.rtl:             # the simulator's run: the same file if the records are equal
            if ctx.same:
                LOG.outcome('sim', ok, progs)
            else:
                sc = ctx.sim_chips[ctx.chips.index(chip)] if len(ctx.sim_chips) == len(ctx.chips) else None
                if sc is None:
                    LOG.outcome('sim', False, progs)
                else:
                    ds = decode(self.runner.vcd(ctx, name, sc, lo, hi, sim_side=True), stack)
                    _, _, ok_s, det_s, why = self.judge(ds, extract, want, known)
                    ok_s = ok_s or bool(why)
                    LOG.outcome('sim', ok_s, progs)
                    if not ok_s:
                        self.bad.append('%s on the simulator: %s' % (name, det_s))
        for p in progs:
            r = LOG.row(p)
            r.stacks.append(stack)
            r.unit = self.unit
            r.items += count(got)
        self.n += 1
        self.items += count(got)
        if not ok:
            self.bad.append('%s: %s' % (name, detail))
        if bench is not NOBENCH:
            self.xn += 1
            if bench != got:
                self.xbad.append('%s: bench decoder %s, sigrok %s' % (name, short(bench), short(got)))
        return got, errs, d

    def cross(self, name, bench, got, known=None):
        """The bench's own decoder, run on the same record, against sigrok. known(bench, got) names a documented
        sigrok difference."""
        self.xn += 1
        if bench != got and not (known and known(bench, got)):     # a known difference is reported with the run
            self.xbad.append('%s: bench decoder %s, sigrok %s' % (name, short(bench), short(got)))

    def fail(self, name, detail):
        """A failure found outside sigrok, such as the firmware's own result."""
        self.bad.append('%s: %s' % (name, detail))

    def close(self):
        if self.known:
            reasons = list(dict.fromkeys(r for _, r in self.known))
            LOG.info('%s: %d run(s) differ from the stimulus where sigrok reads the protocol document differently, '
                     'not counted as failures: %s; files %s' % (self.title, len(self.known), ' / '.join(reasons),
                                                                ', '.join(v for v, _ in self.known[:4])))
        if self.n:
            LOG.check('%s: sigrok decodes what the stimulus says, no error annotations' % self.title, not self.bad,
                      '%d of %d runs, %d %s' % (self.n - len(self.bad), self.n, self.items, self.unit)
                      + ('; ' + ' | '.join(self.bad[:3]) if self.bad else ''))
        if self.xn:
            LOG.check('%s: the bench\'s decoder agrees with sigrok' % self.title, not self.xbad,
                      '%d of %d' % (self.xn - len(self.xbad), self.xn) + ('; ' + ' | '.join(self.xbad[:3])
                                                                            if self.xbad else ''))
        if self.pn:
            ok = LOG.check('%s: RTL pads and board levels equal the simulator\'s, cycle for cycle' % self.title,
                           not self.pbad, '%d of %d runs, %d changes over %d cycles' % (
                               self.pn - len(self.pbad), self.pn, self.changes, self.cycles)
                           + ('; ' + ' | '.join(self.pbad[:2]) if self.pbad else ''))
            if not ok:
                LOG.outcome('rtl', False)


def count(got):
    if isinstance(got, (list, tuple)):
        return sum(count(x) if isinstance(x, (list, tuple)) else 1 for x in got)
    return 1 if got is not None else 0


def short(x, n=160):
    s = repr(x)
    return s if len(s) <= n else s[:n] + '...'


def hexs(seq):
    return [('%02X' % x) if isinstance(x, int) else x for x in seq]


# ---------------------------------------------------------------------------------------------------------------
# decoder stacks and what to take from them

def uart(line, baud, **o):
    opt = dict(baudrate=int(round(baud)), format='hex')
    opt.update(o)
    return [('uart', {'rx': line}, opt)]


def uart_bytes(d):
    data = [int(a.text, 16) for a in d.of('uart', 'rx-data')]
    errs = ['%s at %d' % (a.text, a.cycle) for a in d.of('uart', 'rx-warnings', 'rx-parity-err', 'rx-break')]
    return data, errs


def uart_tokens(d):
    """Data bytes and error annotations in time order, for runs that plant faults: a byte is an int, a framing
    error 'FE', a parity error 'PE', a break 'BRK'."""
    tag = {'rx-warnings': 'FE', 'rx-parity-err': 'PE', 'rx-break': 'BRK'}
    ev = [(a.ss, int(a.text, 16) if a.cls == 'rx-data' else tag[a.cls])
          for a in d.of('uart', 'rx-data', 'rx-warnings', 'rx-parity-err', 'rx-break')]
    return [x for _, x in sorted(ev, key=lambda e: e[0])], []


def spi(clk, mosi, miso, cs, mode):
    return [('spi', {'clk': clk, 'mosi': mosi, 'miso': miso, 'cs': cs}, dict(cpol=mode >> 1, cpha=mode & 1))]


def spi_transfers(d):
    """[(MOSI bytes, MISO bytes)] per CS-low transfer. A transfer with no whole byte (CS low from reset until the
    program raises it) is left out."""
    mo = [a for a in d.of('spi', 'mosi-transfer')]
    mi = [a for a in d.of('spi', 'miso-transfer')]
    out = []
    for a, b in zip(mo, mi):
        if a.text or b.text:
            out.append((list(bytes.fromhex(a.text)), list(bytes.fromhex(b.text))))
    errs = ['%s at %d' % (a.text, a.cycle) for a in d.of('spi', 'warnings')]
    return out, errs


I2C_TOKEN = {'start': 'S', 'repeat-start': 'Sr', 'stop': 'P', 'ack': 'ACK', 'nack': 'NACK'}
I2C_BYTE = {'address-write': 'AW', 'address-read': 'AR', 'data-write': 'DW', 'data-read': 'DR'}


def i2c(scl, sda, *above):
    return [('i2c', {'scl': scl, 'sda': sda}, {})] + list(above)


def i2c_events(d):
    """The bus as sigrok's i2c decoder reads it: S, Sr, P, ACK, NACK, AW:aa and AR:aa (7-bit address), DW:dd and
    DR:dd. sigrok 0.5.3's i2c decoder gives no warnings, so a missing STOP shows as Sr in place of P S."""
    out = []
    for a in sorted(d.of('i2c'), key=lambda a: a.ss):
        if a.cls in I2C_TOKEN:
            out.append(I2C_TOKEN[a.cls])
        elif a.cls in I2C_BYTE and ':' in a.text:
            out.append('%s:%s' % (I2C_BYTE[a.cls], a.text.split(':')[1].strip().upper()))
    return out, ['%s at %d' % (a.text, a.cycle) for a in d.of('i2c', 'warnings')]


def i2c_expect(txs, addr, nack=(), last_nack=False):
    """Bus events for transactions given as byte lists (the first byte is the address byte), against a target at
    addr (None: one that answers any address) that ACKs its address and every data byte except those numbered in
    nack (0 is the address byte). The controller ends each transaction with a STOP; after a NACK it stops at
    once."""
    out = []
    for tx in txs:
        out.append('S')
        for i, b in enumerate(tx):
            out.append(('AR:%02X' if b & 1 else 'AW:%02X') % (b >> 1) if i == 0 else 'DW:%02X' % b)
            acked = (i > 0 or addr is None or (b >> 1) == addr) and i not in nack
            out.append('ACK' if acked else 'NACK')
            if not acked:
                break
        out.append('P')
    return out


def bench_i2c_events(txs):
    """rig.i2c_decode's transactions as i2c_events tokens. That decoder reads every ninth bit as the ACK and doesn't
    know read from write, so only write transactions compare."""
    out = []
    for k, tx in enumerate(txs):
        out.append('Sr' if k and not txs[k - 1]['stop'] else 'S')
        for i, (b, ack) in enumerate(zip(tx['bytes'], tx['acks'])):
            out.append(('AR:%02X' if b & 1 else 'AW:%02X') % (b >> 1) if i == 0 else 'DW:%02X' % b)
            out.append('ACK' if ack else 'NACK')
        if tx['stop']:
            out.append('P')
    return out


def ws281x(din):
    return [('rgb_led_ws281x', {'din': din}, {})]


def ws281x_frames(d):
    """Bits per frame, frames split at sigrok's RESET; and the #rrggbb colour of every 24 bits."""
    frames, cur = [], []
    for a in sorted(d.of('rgb_led_ws281x', 'bit', 'reset'), key=lambda a: a.ss):
        if a.cls == 'bit':
            cur.append(int(a.text))
        else:
            frames.append(cur)
            cur = []
    if cur:
        frames.append(cur)
    rgb = [a.text for a in sorted(d.of('rgb_led_ws281x', 'rgb'), key=lambda a: a.ss)]
    return (frames, rgb), []


def ws281x_expect(frames):
    """GRB bytes on the wire, MSB first; sigrok prints each LED as #rrggbb."""
    bits = [[(b >> (7 - i)) & 1 for b in fr for i in range(8)] for fr in frames]
    rgb = ['#%02x%02x%02x' % (fr[k + 1], fr[k], fr[k + 2]) for fr in frames for k in range(0, len(fr) - 2, 3)]
    return bits, rgb


def onewire(owr):
    return [('onewire_link', {'owr': owr}, {}), ('onewire_network', {}, {})]


def onewire_events(d):
    """The network layer's view: RP:1 or RP:0 (reset, presence), ROM:cc (a ROM command), D:dd (data); and the link
    layer's bytes, LSB first, eight bits after each reset."""
    net = []
    for a in sorted(d.of('onewire_network'), key=lambda a: a.ss):
        t = a.text
        if t.startswith('Reset/presence'):
            net.append('RP:%d' % (t.endswith('true')))
        elif t.startswith('ROM command'):
            net.append('ROM:%s' % t.split()[2][2:])
        elif t.startswith('Data'):
            net.append('D:%s' % t.split()[1][2:])
        else:
            net.append(t)
    errs = ['%s at %d' % (a.text, a.cycle) for a in d.of('onewire_link', 'warnings')]
    return net, errs


def onewire_expect(data, present=True):
    out = ['RP:%d' % present]
    for i, b in enumerate(data):
        out.append(('ROM:%02x' if i == 0 else 'D:%02x') % b)
    return out


# ---------------------------------------------------------------------------------------------------------------
# basic/: bench/basic.py

def jtag(tck, tms, tdi, tdo):
    return [('jtag', {'tck': tck, 'tms': tms, 'tdi': tdi, 'tdo': tdo}, {})]


def jtag_scans(d):
    """[(IR or DR, bits, TDI value, TDO value)] per shift, the first bit shifted as bit 0."""
    tdi = sorted(d.of('jtag', 'bitstring-tdi'), key=lambda a: a.ss)
    tdo = sorted(d.of('jtag', 'bitstring-tdo'), key=lambda a: a.ss)
    out = []
    for a, b in zip(tdi, tdo):
        kind = a.text.split()[0]
        n = int(a.text.split(',')[-1].split()[0])
        out.append((kind, n, int(a.text.split('(')[1].split(')')[0], 16), int(b.text.split('(')[1].split(')')[0], 16)))
    return out, []


def jtag_match(want):
    """want: [(kind, bits, TDI or None for don't care, TDO or None)]."""
    def f(got, errs):
        ok = len(got) == len(want) and all(g[0] == w[0] and g[1] == w[1] and (w[2] is None or g[2] == w[2]) and
                                           (w[3] is None or g[3] == w[3]) for g, w in zip(got, want))
        return ok, '' if ok else 'decoded %s, wanted %s' % (short(got), short(want))
    return f


def cec(line):
    return [('cec', {'cec': line}, {})]


def cec_frames(d):
    """[[(byte, EOM, ACK or NACK)] per frame], from sigrok's bytes, EOM and ACK annotations."""
    frames, cur, last = [], [], None
    for a in sorted(d.of('cec', 'st', 'bytes', 'eom-0', 'eom-1', 'ack', 'nack'), key=lambda a: a.ss):
        if a.cls == 'st':
            if cur:
                frames.append(cur)
            cur = []
        elif a.cls == 'bytes':
            last = [int(a.text, 16), None, None]
            cur.append(last)
        elif a.cls in ('eom-0', 'eom-1') and last:
            last[1] = int(a.cls == 'eom-1')
        elif last:
            last[2] = a.text
    if cur:
        frames.append(cur)
    frames = [[tuple(x) for x in fr] for fr in frames]
    errs = ['%s at %d' % (a.text, a.cycle) for a in d.of('cec', 'warnings')]
    return frames, errs


def eeprom_ops(d):
    """sigrok's 24xx EEPROM operations in time order, such as 'Byte write (addr=10, 1 byte): 5A', with its warnings
    among them: a 'No reply from slave!' is what the stimulus wants where it addresses a device that isn't there."""
    ops = [a.text for a in sorted(d.of('eeprom24xx'), key=lambda a: a.ss)
           if a.cls in ('byte-write', 'page-write', 'cur-addr-read', 'random-read', 'seq-random-read',
                        'seq-cur-addr-read', 'ack-polling', 'warnings')]
    return ops, []


def i2c_eeprom_events(d):
    """i2c_events and eeprom_ops together, for a stack of i2c and eeprom24xx."""
    ev, e1 = i2c_events(d)
    ops, e2 = eeprom_ops(d)
    return (ev, ops), e1 + e2


def eeprom_expect(ops, mem, dev=0x50):
    """Bus events and sigrok's 24xx operation texts for (device byte, register, data) operations of the
    host-driven controllers (a write of one byte, or a random read of one byte with a repeated START), against
    an EEPROM model holding mem (a dict) at device address dev."""
    ev, texts = [], []
    mem = dict(mem)
    for devb, reg, data in ops:
        a = devb >> 1
        ev += ['S', 'AW:%02X' % a]
        if a != dev:
            ev += ['NACK', 'P']
            texts.append('Warning: No reply from slave!')
            continue
        ev += ['ACK', 'DW:%02X' % reg, 'ACK']
        if devb & 1:
            v = mem.get(reg, 0xFF)
            ev += ['Sr', 'AR:%02X' % a, 'ACK', 'DR:%02X' % v, 'NACK', 'P']
            texts.append('Random access read (addr=%02X, 1 byte): %02X' % (reg, v))
        else:
            mem[reg] = data
            ev += ['DW:%02X' % data, 'ACK', 'P']
            texts.append('Byte write (addr=%02X, 1 byte): %02X' % (reg, data))
    return ev, texts


def swd(swclk, swdio):
    return [('swd', {'swclk': swclk, 'swdio': swdio}, {})]


def swd_events(d):
    """RESET, JTAG->SWD, R:<register> and W:<register> requests, ACK:OK/WAIT/FAULT/NOREPLY, D:0x........ and
    PE:<computed><received> for a data parity error, in time order."""
    tag = {'reset': lambda t: 'RESET', 'enable': lambda t: t, 'read': lambda t: 'R:' + t, 'write': lambda t: 'W:' + t,
           'ack': lambda t: 'ACK:' + t, 'data': lambda t: 'D:' + t, 'parity': lambda t: 'PE:' + t}
    out = [tag[a.cls](a.text) for a in sorted(d.of('swd'), key=lambda a: a.ss) if a.cls in tag]
    return out, []


def lin_stack(line, baud):
    return [('uart', {'rx': line}, dict(baudrate=int(round(baud)), format='hex')), ('lin', {}, {})]


def lin_events(d):
    """BRK, SYNC and ID:ii/ok or ID:ii/bad (sigrok's LIN 2 parity check), in time order. The UART's framing error
    on the break's own null character is part of the break, not an error."""
    out = []
    for a in sorted(d.of('lin'), key=lambda a: a.ss):
        t = a.text
        if a.cls == 'control' and t.startswith('Break'):
            out.append('BRK')
        elif t == 'Sync':
            out.append('SYNC')
        elif t.startswith('ID:'):
            out.append('ID:%s/%s' % (t.split()[1], 'ok' if t.endswith('(ok)') else 'bad'))
        elif t.startswith('Sync is not'):
            out.append('SYNC?')
    brk = [(a.ss, a.es) for a in d.of('uart', 'rx-break')]
    errs = ['%s at %d' % (a.text, a.cycle) for a in d.of('uart', 'rx-warnings', 'rx-parity-err')
            if not any(lo <= a.ss <= hi for lo, hi in brk)]
    errs += ['%s at %d' % (a.text, a.cycle) for a in d.of('lin', 'error') if not a.text.startswith('P != ')]
    return out, errs


def nec(line):
    return [('ir_nec', {'ir': line}, {'polarity': 'active-high', 'cd_freq': 38000})]


def nec_events(d):
    keep = ('leader-code', 'addr', 'addr-inv', 'cmd', 'cmd-inv', 'repeat-code')
    out = [a.text for a in sorted(d.of('ir_nec'), key=lambda a: a.ss) if a.cls in keep]
    return out, ['%s at %d' % (a.text, a.cycle) for a in d.of('ir_nec', 'warnings')]


def parallel(clk, data):
    ch = {'clk': clk}
    ch.update({'d%d' % i: n for i, n in enumerate(data)})
    return [('parallel', ch, {'clock_edge': 'rising'})]


def parallel_items(d):
    return [int(a.text, 16) for a in sorted(d.of('parallel', 'items'), key=lambda a: a.ss)], []


def dmx_stack(line):
    return [('dmx512', {'dmx': line}, {})]


def dmx_packets(d):
    """[[start code, slot 1, ...] per packet], from sigrok's break, start code and data annotations."""
    out, cur = [], None
    for a in sorted(d.of('dmx512', 'break', 'data'), key=lambda a: a.ss):
        if a.cls == 'break':
            cur = []
            out.append(cur)
        elif cur is not None:
            cur.append(int(a.text.split('/')[0]))
    return out, ['%s at %d' % (a.text, a.cycle) for a in d.of('dmx512', 'error')]


def long_lows(chip, sig, n, at_least):
    """Cycles where a line falls into a low lasting at least at_least cycles (DMX breaks, CAN idle ends)."""
    w = trace_of(chip).wave(sig, n)
    out = []
    ed = w.edges()
    for (t, v), nxt in zip(ed, ed[1:] + [(w.end, None)]):
        if v == 0 and nxt[0] - t >= at_least:
            out.append(t)
    return out


def mdio_stack(mdc, line):
    return [('mdio', {'mdc': mdc, 'mdio': line}, {})]


def mdio_frames(d):
    """sigrok's frame summaries, such as 'READ: 0141 PHYAD: 17 REGAD: 02', spaces folded."""
    out = [' '.join(a.text.split()) for a in sorted(d.of('mdio', 'decode'), key=lambda a: a.ss)]
    return out, ['%s at %d' % (a.text, a.cycle) for a in d.of('mdio', 'frame-error')]


def bench_mdio(chip):
    import mdio
    tr = trace_of(chip)
    frames, errs, m = mdio.mdio_decode(tr.wave('uo', 1), tr.wave('io', 0), tr.wave('iooe', 0))
    return ['%s: %04X PHYAD: %02d REGAD: %02d' % ('READ' if op == 2 else 'WRITE', data, pa, ra)
            for op, pa, ra, data in frames]


def mdio_expect(ops, regs, phy=0x11):
    """Frame summaries for (header word, [data]) operations of the MDIO station program."""
    out, regs = [], dict(regs)
    for op in ops:
        h = op[0]
        kind, pa, ra = (h >> 12) & 3, (h >> 7) & 31, (h >> 2) & 31
        if kind == 1:
            regs[ra] = op[1]
            out.append('WRITE: %04X PHYAD: %02d REGAD: %02d' % (op[1], pa, ra))
        else:
            out.append('READ: %04X PHYAD: %02d REGAD: %02d' % (regs.get(ra, 0) if pa == phy else 0xFFFF, pa, ra))
    return out


def ps2(clk, data):
    return [('ps2', {'clk': clk, 'data': data}, {})]


def ps2_words(d):
    """(byte, parity ok) per device-to-host frame. sigrok's ps2 0.5.3 reports a frame at the twelfth clock fall
    after its start bit, so it is run on one window per frame (see ps2_windows)."""
    words = sorted(d.of('ps2', 'word'), key=lambda a: a.ss)
    par = sorted(d.of('ps2', 'parity-ok', 'parity-err'), key=lambda a: a.ss)
    return [(int(w.text.split()[1], 16), p.cls == 'parity-ok') for w, p in zip(words, par)], []


def can_stack(line, bitrate, sp):
    return [('can', {'can_rx': line}, {'nominal_bitrate': bitrate, 'sample_point': sp})]


def crc15(bits):
    """CRC-15/CAN, polynomial 0x4599, init 0 (Bosch CAN 2.0 part A, 3.1.1); check value 0x059E for '123456789'."""
    reg = 0
    for b in bits:
        fb = ((reg >> 14) & 1) ^ b
        reg = (reg << 1) & 0x7FFF
        if fb:
            reg ^= 0x4599
    return reg


assert crc15([(c >> (7 - i)) & 1 for c in b'123456789' for i in range(8)]) == 0x059E


def can_frames(d):
    """(id, data, CRC field, ACK) per frame from sigrok's fields; sigrok's warnings are the errors (0.5.3 does
    not check the CRC itself: the expected CRC field is computed from the frame, see can_expect)."""
    out, cur = [], None
    for a in sorted(d.of('can'), key=lambda a: a.ss):
        t = a.text
        if a.cls == 'sof':
            cur = dict(id=None, data=[], crc=None, ack=None)
            out.append(cur)
        elif cur is None:
            continue
        elif a.cls == 'id':
            cur['id'] = int(t.split('(')[1].split(')')[0], 16)
        elif a.cls == 'data':
            cur['data'].append(int(t.split(':')[1], 16))
        elif a.cls == 'crc-sequence':
            cur['crc'] = int(t.split(':')[1], 16)
        elif a.cls == 'ack-slot':
            cur['ack'] = t.split(':')[1].strip() == 'ACK'
    frames = [(f['id'], f['data'], f['crc'], f['ack']) for f in out]
    return frames, ['%s at %d' % (a.text, a.cycle) for a in d.of('can', 'warnings')]


def can_expect(ident, data, ack=True):
    bits = [0] + [(ident >> (10 - i)) & 1 for i in range(11)] + [0, 0, 0] + \
        [(len(data) >> (3 - i)) & 1 for i in range(4)] + [(x >> (7 - i)) & 1 for x in data for i in range(8)]
    return (ident, list(data), crc15(bits), ack)


def bus_integrated(chip, sig, n, bit):
    """The first cycle after which a CAN line has been recessive for 11 bit times: where a node joins the bus
    (Bosch CAN 2.0 part B, 9.1, bus integration). Before it the TX pad is still low from reset."""
    w = trace_of(chip).wave(sig, n)
    t_hi = 0 if w.at(0) else None
    for t, v in w.edges():
        if v == 1:
            t_hi = t
        elif t_hi is not None and t - t_hi >= 11 * bit:
            return t_hi + 11 * bit
        else:
            t_hi = None
    return t_hi + 11 * bit if t_hi is not None else 0


USB_PID = {0xE1: 'OUT', 0x69: 'IN', 0xA5: 'SOF', 0x2D: 'SETUP', 0xC3: 'DATA0', 0x4B: 'DATA1', 0xD2: 'ACK',
           0x5A: 'NAK', 0x1E: 'STALL'}


def usb_stack(dp, dm):
    return [('usb_signalling', {'dp': dp, 'dm': dm}, {'signalling': 'low-speed'}),
            ('usb_packet', {}, {'signalling': 'low-speed'})]


def usb_packets(d):
    """sigrok's packet summaries ('DATA1 [ 01 02 ]', 'IN ADDR 21 EP 1', 'ACK'). Errors: a CRC5 or CRC16 error, a
    bad SYNC, an invalid or reserved PID, and usb_signalling's errors."""
    out = [a.text for a in sorted(d.of('usb_packet'), key=lambda a: a.ss) if a.cls.startswith('packet-')]
    bad = ('crc5-err', 'crc16-err', 'sync-err', 'packet-invalid', 'packet-reserved')
    errs = ['%s %s at %d' % (a.cls, a.text, a.cycle) for a in d.of('usb_packet', *bad)]
    errs += ['%s at %d' % (a.text, a.cycle) for a in d.of('usb_signalling', 'error')]
    return out, errs


def usb_expect(packet):
    """sigrok's summary for a packet given as bytes, PID first (USB 2.0, 8.3 and 8.4)."""
    name = USB_PID[packet[0]]
    if name in ('OUT', 'IN', 'SETUP'):
        v = packet[1] | (packet[2] & 7) << 8
        return '%s ADDR %d EP %d' % (name, v & 0x7F, v >> 7)
    if name.startswith('DATA'):
        return '%s [%s ]' % (name, ''.join(' %02X' % x for x in packet[1:-2]))
    return name


# Firmware findings: runs where sigrok shows the bus doing something the stimulus doesn't ask for. Reported on
# DIFF lines and in the README, not failed, until the programs or their mailbox contracts change.
FINDING = {
    'i2c_left': 'firmware: after the address byte of a write is NACKed, its data word stays in MIN and the program '
                'runs it as the next transaction (device byte 0x5A: address 0x2D)',
    'cec_rest': 'firmware: after a NACKed block the rest of the frame stays in MIN and goes out as a new frame, '
                'with no signal free time before its start bit',
}
# Decoder findings: where sigrok 0.5.3 reads a bus differently from the protocol document.
DECODER = {
    'jtag_pause': 'sigrok 0.5.3 jtag takes a bit on the Exit1-DR to Pause-DR clock, where IEEE 1149.1 shifts '
                  'nothing: the scan through Pause-DR shows one bit too many',
    'can_7ff': 'bench stimulus: identifier 0x7FF, whose bits ID-10 to ID-4 are all recessive, which Bosch CAN 2.0 '
               'part A 3.1.1 forbids; sigrok warns, and decodes the frame as sent',
    'dmx_88': 'ANSI E1.11 has receivers take a break of 88 us and a mark between slots of 0; sigrok 0.5.3 dmx512 '
              'wants a break of more than 88 us and at least one sample of mark between slots',
}


def left_data_word(want):
    """For a controller whose stimulus ends with a write to an absent device (words 0xB010, 0x5A00): the stray
    transaction, or as much of it as the bench's run lasts."""
    ev, ops = want
    stray = ['S', 'AW:2D', 'NACK', 'P']

    def f(got, errs):
        g_ev, g_ops = got
        extra = g_ev[len(ev):]
        if g_ev[:len(ev)] == ev and extra and extra == stray[:len(extra)] and \
                g_ops in (ops, ops + ['Warning: No reply from slave!']) and not errs:
            return FINDING['i2c_left']
    return f


def presence_at_limit(want):
    """The DS18B20 data sheet gives the presence pulse 15 to 60 us after the reset pulse ends (tPDHIGH). A device
    model that starts it at exactly 60 us is inside that range; sigrok's onewire_link reports no presence when the
    fall comes at its 60 us timeout."""
    def f(got, errs):
        if got == ['RP:0'] + want[1:] and not errs:
            return 'presence pulse starts at exactly 60 us, the tPDHIGH maximum: sigrok reports no presence'
    return f


def g_basic_uart(runner, log):
    import basic
    log.group('basic/uart_tx.s', title='UART 8N1 transmit on uo0, bench/basic.py uart_tx_case')
    t = basic.src('uart_tx')
    data = [0x55, 0xAA, 0x00, 0xFF, 0x31, 0x80]
    for baud in (115200, 1_000_000):
        b = Batch(runner, '%d baud, 4 threads x 4 alignments, 6 bytes' % baud)
        for th in range(4):
            for al in range(4):
                ctx = b.run('basic_uart_tx_%d_t%d_a%d' % (baud, th, al), basic.uart_tx_case, t, baud, th, al)
                b.sigrok(ctx, uart('uo0', baud), uart_bytes, data, bench=ctx.value[1])
        b.close()
    for name, baud, d in (('12 cycles per bit (4.17 Mbaud)', CLK / 12, (0x55, 0xAA, 0x00, 0xFF, 0x31, 0x80)),
                          ('9600 baud', 9600, (0x41, 0x7E)), ('32767 cycles per bit', CLK / 32767, (0x5A,))):
        b = Batch(runner, name)
        ctx = b.run('basic_uart_tx_%d' % round(baud), basic.uart_tx_case, t, baud, data=d)
        b.sigrok(ctx, uart('uo0', baud), uart_bytes, list(d), bench=ctx.value[1])
        b.close()

    log.group('basic/uart_rx.s', title='UART 8N1 receive on ui3, bench/basic.py uart_rx_case: the sender model')
    t = basic.src('uart_rx')
    data = [0x55, 0xAA, 0x00, 0xFF, 0x31, 0x80, 0x0F, 0xF0]
    long_data = [(i * 37 + 11) & 255 for i in range(64)]
    for baud in (115200, 1_000_000):
        b = Batch(runner, '%d baud, abutting frames, 8 start phases over the 4 threads' % baud)
        for ph in range(8):
            ctx = b.run('basic_uart_rx_%d_p%d' % (baud, ph), basic.uart_rx_case, t, baud, ph, data, thread=ph % 4)
            got, r, snd = ctx.value
            tr = trace_of(ctx.chip)
            bench, _ = rig_uart_decode(tr.wave('ui', 3), baud)
            b.sigrok(ctx, uart('ui3', baud), uart_bytes, data, bench=bench)
            if got != data:
                b.fail(ctx.name, 'the firmware read %s' % short(got))
        b.close()
        for err in (-0.02, 0.02):
            b = Batch(runner, '%d baud, sender %+.0f%%, 64 abutting bytes' % (baud, err * 100))
            ctx = b.run('basic_uart_rx_%d_e%s2' % (baud, 'm' if err < 0 else 'p'), basic.uart_rx_case, t, baud, 3,
                        long_data, err=err)
            b.sigrok(ctx, uart('ui3', baud), uart_bytes, long_data)
            b.close()
    b = Batch(runner, 'a 10-cycle glitch, a byte, a byte with its stop bit low, a byte')
    ctx = b.run('basic_uart_rx_faults', basic.uart_rx_case, t, 115200, 0, None,
                frames=[('glitch', 10), 0x5A, ('badstop', 0xC3), ('gap', 5000), 0x3C])
    want = ['FE', 0x5A, 0xC3, 'FE', 0x3C]
    b.sigrok(ctx, uart('ui3', 115200), uart_tokens,
             lambda got, errs: (got == want, 'sigrok %s, wanted %s (the glitch and the low stop bit as framing '
                                'errors)' % (hexs(got), hexs(want))))
    b.close()
    State.thread = 0


def g_basic_spi(runner, log):
    import basic
    import random
    import rig
    log.group('basic/spi.s', title='SPI controller, SCK uo1, MOSI uo2, MISO ui4, CS uo5, bench/basic.py spi_case')
    t = basic.src('spi')
    frames = ((0xA5,), (0x3C, 0xFF, 0x00))
    for half in (9, 25, 250):
        b = Batch(runner, 'half period %d, modes 0 to 3 x 4 threads x 2 alignments' % half)
        for mode in range(4):
            rng = random.Random(7 + mode)                       # spi_case's target replies
            replies = [rng.getrandbits(8) for _ in range(4)]
            want = [(list(frames[0]), replies[:1]), (list(frames[1]), replies[1:])]
            for th in range(4):
                for al in ((th + mode) % 4, (th + mode + 2) % 4):
                    ctx = b.run('basic_spi_h%d_m%d_t%d_a%d' % (half, mode, th, al), basic.spi_case, t, mode, half,
                                th, al)
                    tr = trace_of(ctx.chip)
                    fr, _ = rig.spi_decode(tr.wave('uo', 1), tr.wave('uo', 2), tr.wave('ui', 4), tr.wave('uo', 5),
                                           mode >> 1, mode & 1, tsu=half // 2, thd=half // 2)
                    b.sigrok(ctx, spi('uo1', 'uo2', 'ui4', 'uo5', mode), spi_transfers, want,
                             bench=[(list(x), list(y)) for x, y in fr])
                    if not ctx.value[0]:
                        b.fail(ctx.name, 'the bench fails it: %s' % short(ctx.value[1]))
        b.close()


def g_basic_i2c(runner, log):
    import basic
    import rig
    log.group('basic/i2c_write.s', title='I2C controller, write only, SCL io2, SDA io1, bench/basic.py i2c_case')
    t = basic.src('i2c_write')
    write = [0xA0, 0x10, 0x5A, 0xC3]

    def one(b, name, mode, txs, want, **kw):
        ctx = b.run(name, basic.i2c_case, t, mode, txs, **kw)
        dec, st, tgt, m, p = ctx.value
        tr = trace_of(ctx.chip)
        bdec, _, _ = rig.i2c_decode(tr.wave('io', 2), tr.wave('io', 1), mode)
        b.sigrok(ctx, i2c('uio2', 'uio1'), i2c_events, want, bench=bench_i2c_events(bdec))
        if p:
            b.fail(name, 'the bench fails it: %s' % short(p))
    for mode in ('Sm', 'Fm'):
        b = Batch(runner, '%s: write and a short write, rise 0 and %d cycles' % (mode, 50 if mode == 'Sm' else 15),
                  'events')
        for rise in (0, 50 if mode == 'Sm' else 15):
            one(b, 'basic_i2c_%s_rise%d' % (mode, rise), mode, [write, write[:2]],
                i2c_expect([write, write[:2]], 0x50), rise=rise)
        b.close()
        b = Batch(runner, '%s: 4 threads x 2 alignments' % mode, 'events')
        for th in range(4):
            for al in (th, (th + 2) % 4):
                one(b, 'basic_i2c_%s_t%d_a%d' % (mode, th, al), mode, [write], i2c_expect([write], 0x50),
                    thread=th, align=al)
        b.close()
        b = Batch(runner, '%s: target stretches every clock; after each ACK' % mode, 'events')
        one(b, 'basic_i2c_%s_stretch_all' % mode, mode, [write], i2c_expect([write], 0x50),
            target=dict(stretch=lambda byte, n: 1000 + 37 * (n + byte)))
        one(b, 'basic_i2c_%s_stretch_ack' % mode, mode, [write], i2c_expect([write], 0x50),
            target=dict(stretch=lambda byte, n: 5000 if n == 8 else 0))
        b.close()
        b = Batch(runner, '%s: NACK of the second data byte; wrong address; three transactions' % mode, 'events')
        one(b, 'basic_i2c_%s_nack' % mode, mode, [write], i2c_expect([write], 0x50, nack={2}), target=dict(nack={2}))
        one(b, 'basic_i2c_%s_addr51' % mode, mode, [write], i2c_expect([write], 0x51), addr=0x51)
        one(b, 'basic_i2c_%s_three' % mode, mode, [write[:2]] * 3, i2c_expect([write[:2]] * 3, 0x50))
        b.close()


def g_basic_ws(runner, log):
    import basic
    import rig
    log.group('basic/ws2812.s', title='WS2812 on uo6, bench/basic.py ws_case')
    t = basic.src('ws2812')
    frames = [[0x12, 0x34, 0x56, 0xFF, 0x00, 0xA5, 0x80, 0x01, 0x7E]]
    b = Batch(runner, '3 LEDs, 72 bits, 4 threads x 2 alignments', 'bits')
    for th in range(4):
        for al in (th, (th + 2) % 4):
            ctx = b.run('basic_ws_t%d_a%d' % (th, al), basic.ws_case, t, frames, th, al)
            dec, errs, m = rig.ws2812_decode(trace_of(ctx.chip).wave('uo', 6))
            got, _, _ = b.sigrok(ctx, ws281x('uo6'), ws281x_frames, ws281x_expect(frames))
            b.xn += 1
            if dec != got[0]:
                b.xbad.append('%s: bench %s, sigrok %s' % (ctx.name, short(dec), short(got[0])))
    b.close()
    b = Batch(runner, 'two frames with the latch gap', 'bits')
    two = [[0xF0, 0x0F, 0x55], [0xAA, 0x01, 0x80]]
    ctx = b.run('basic_ws_two', basic.ws_case, t, two)
    b.sigrok(ctx, ws281x('uo6'), ws281x_frames, ws281x_expect(two))
    b.close()


def g_basic_ow(runner, log):
    import basic
    log.group('basic/onewire.s', title='1-Wire controller on io4, bench/basic.py ow_case')
    t = basic.src('onewire')
    data = [0xCC, 0x44, 0xA5, 0x00, 0xFF]
    b = Batch(runner, 'reset, presence, 5 bytes; no device', 'events')
    ctx = b.run('basic_ow_5', basic.ow_case, t, data)
    b.sigrok(ctx, onewire('uio4'), onewire_events, onewire_expect(data),
             bench=onewire_expect([e[2] for e in ctx.value[0][1:]], ctx.value[0][0][2]))
    ctx = b.run('basic_ow_absent', basic.ow_case, t, [0x33], present=False)
    b.sigrok(ctx, onewire('uio4'), onewire_events, onewire_expect([0x33], False))
    b.close()
    b = Batch(runner, 'device timing corners: sample 15/30/60 us, presence wait and length', 'events')
    for s_ in (15, 30, 60):
        for wait, low in ((15, 60), (30, 120), (60, 60), (60, 240)):
            ctx = b.run('basic_ow_s%d_w%d_l%d' % (s_, wait, low), basic.ow_case, t, data[:2], sample_us=s_,
                        pd_wait=wait, pd_low=low)
            want = onewire_expect(data[:2])
            b.sigrok(ctx, onewire('uio4'), onewire_events, want,
                     known=presence_at_limit(want) if wait == 60 else None)
    b.close()
    b = Batch(runner, '4 threads x 2 alignments', 'events')
    for th in range(4):
        for al in (th, (th + 2) % 4):
            ctx = b.run('basic_ow_t%d_a%d' % (th, al), basic.ow_case, t, [0x5A], thread=th, align=al)
            b.sigrok(ctx, onewire('uio4'), onewire_events, onewire_expect([0x5A]))
    b.close()


# ---------------------------------------------------------------------------------------------------------------
# standard/: bench/standard.py

BAUD = {434: 115200, 5208: 9600, 100: 500000, 50: 1000000}     # cycles per bit at 50 MHz to the nominal rate


def baud_of(period):
    return BAUD.get(period, CLK / period)


def std_uart_tx(period, tid, data=(0x55, 0xA3)):
    """standard.py test_uart_tx's run: post the bit time, then each byte when the word is free."""
    import standard
    chip, b, n = standard.boot('uart_tx', tid)
    chip.post(7, period)
    chip.run(60, b)
    starts = []
    for byte in data:
        while not chip.post(0, byte):
            chip.run(4, b)
        starts.append(chip.c)
        chip.run(12 * period + 60, b)
    bench = []
    for since in starts:                    # the bench's decoder, per byte, as test_uart_tx runs it
        e = b.edges('uo0', since)
        bench.append(standard.decode_uart(e, period, e[0][0])[1] if e else None)
    return bench


def std_uart_rx(period, tid, data, faults=()):
    import standard
    from devices import UartSource
    chip, b, n = standard.boot('uart_rx', tid)
    chip.post(7, period)
    src = UartSource(3, period, data, 200 if not faults else 300)
    b.devs.append(src)
    b.devs += list(faults)
    got = []
    for _ in range(src.end + 300 if not faults else 1700):
        chip.cycle(b)
        v = chip.take(1)
        if v is not None:
            got.append(v)
    return got


def std_spi(cpol, cpha, half, tid, skew):
    import standard
    chip, b, dev, got, n, since = standard.spi_once(cpol, cpha, half, tid, skew)
    chip.run(4 * half + 40, b)
    return dev.got, got


def std_i2c(mode, word, tid=0, **kw):
    import standard
    from devices import I2cTarget
    dev = I2cTarget(**kw)
    chip, b, n, res, since = standard.i2c_run(mode, word, dev, tid)
    return res, dev.bytes


def std_ws(tid, byte):
    import standard
    chip, b, n = standard.boot('ws2812', tid)
    chip.run(40, b)
    chip.post(0, byte)
    for _ in range(20000):
        chip.cycle(b)
        if chip.take(1):
            break
    chip.run(5000, b)                       # the line idles past sigrok's 50 us reset before the capture ends
    return None


def std_ow(tid, present=True):
    import standard
    from devices import OneWireSlave
    chip, b, n = standard.boot('onewire', tid)
    dev = OneWireSlave(present=present)
    b.devs.append(dev)
    chip.run(40, b)
    chip.post(0, 0xCC)
    res = None
    for _ in range(200000):
        chip.cycle(b)
        res = chip.take(1)
        if res is not None:
            break
    chip.run(5000, b)                       # the time slot of the last bit ends inside the capture
    return res, dev.got


def std_cec_four():
    import standard
    r = standard.cec_four(4)
    r[0].run(5 * standard.MS, r[1])         # sigrok closes the last ACK bit after 4.1 ms of idle line
    return r[3]


def std_cec_alone():
    """cec_run(9, 0): no follower at 4. The bench's host posts both blocks at once, so after the NACKed header the
    second block goes out as a frame of its own; run until the program reports that one too."""
    import standard
    chip, b, dev, res, n = standard.cec_run(9, 0)
    got = [res]
    for _ in range(80 * standard.MS):
        chip.cycle(b)
        v = chip.take(1)
        if v is not None:
            got.append(v)
            break
    chip.run(5 * standard.MS, b)
    return got


def std_feed(name, msg, answers, tid=0):
    import standard
    chip, b, n = standard.boot(name, tid)
    return standard.feed(chip, b, msg, answers=answers)


def g_standard(runner, log):
    import binascii
    import random
    import standard
    import zlib
    log.group('standard/uart_tx.s', title='UART 8N1 transmit on uo0, bench/standard.py test_uart_tx')
    for period in (434, 5208, 12):
        b = Batch(runner, '%d cycles per bit, 4 threads' % period)
        for tid in range(4):
            ctx = b.run('std_uart_tx_%d_t%d' % (period, tid), std_uart_tx, period, tid)
            b.sigrok(ctx, uart('uo0', baud_of(period)), uart_bytes, [0x55, 0xA3], bench=ctx.value)
        b.close()

    log.group('standard/uart_rx.s', title='UART 8N1 receive on ui3, bench/standard.py test_uart_rx: the sender')
    data = [0x00, 0xFF, 0x5A, 0xC3]
    for period in (434, 50):
        b = Batch(runner, '%d cycles per bit, 4 threads' % period)
        for tid in range(4):
            ctx = b.run('std_uart_rx_%d_t%d' % (period, tid), std_uart_rx, period, tid, data)
            b.sigrok(ctx, uart('ui3', baud_of(period)), uart_bytes, data)
            if ctx.value != data:
                b.fail(ctx.name, 'firmware read %s' % short(ctx.value))
        b.close()
    from devices import Force
    b = Batch(runner, 'a 10-cycle glitch before a frame; a frame with its stop bit forced low')
    ctx = b.run('std_uart_rx_glitch', std_uart_rx, 100, 0, [0x41], faults=[Force(3, 100, 110, 0)])
    b.sigrok(ctx, uart('ui3', 500000), uart_tokens,
             lambda got, errs: (got == ['FE', 0x41], 'sigrok %s, wanted the glitch as a framing error, then 41'
                                % hexs(got)))
    ctx = b.run('std_uart_rx_badstop', std_uart_rx, 100, 0, [0x41], faults=[Force(3, 1190, 1300, 0)])
    b.sigrok(ctx, uart('ui3', 500000), uart_tokens,
             lambda got, errs: (got[:2] == [0x41, 'FE'], 'sigrok %s, wanted 41 with a framing error' % hexs(got)))
    b.close()

    log.group('standard/spi.s', title='SPI controller, SCK uo1, MOSI uo2, MISO ui4, CS uo5, bench/standard.py '
              'spi_once')
    for half in (25, 10, 9):
        b = Batch(runner, 'half period %d, modes 0 to 3 x 4 threads' % half)
        for mode in range(4):
            for tid in range(4):
                skew = (tid + mode) % 4
                ctx = b.run('std_spi_h%d_m%d_t%d' % (half, mode, tid), std_spi, mode >> 1, mode & 1, half, tid,
                            skew)
                b.sigrok(ctx, spi('uo1', 'uo2', 'ui4', 'uo5', mode), spi_transfers, [([0xD2], [0xB7])])
                if ctx.value != ([0xD2], 0xB7):
                    b.fail(ctx.name, 'target model %s, firmware read %s' % ctx.value)
        b.close()

    log.group('standard/i2c_write.s', title='I2C controller, write only, SDA io0, SCL io1, bench/standard.py i2c_run')
    w = [0xA0, 0x3C]
    for mode in ('Sm', 'Fm'):
        b = Batch(runner, '%s, 4 threads' % mode, 'events')
        for tid in range(4):
            ctx = b.run('std_i2c_%s_t%d' % (mode, tid), std_i2c, mode, 0xA03C, tid)
            b.sigrok(ctx, i2c('uio1', 'uio0'), i2c_events, i2c_expect([w], 0x50))
        b.close()
    b = Batch(runner, 'Sm, stretching at four points; NACK on the address and on the data', 'events')
    for k, st in enumerate(({0: 3000}, {7: 5000}, {8: 1234}, {12: 777, 17: 20000})):
        ctx = b.run('std_i2c_stretch%d' % k, std_i2c, 'Sm', 0xA03C, stretch=st)
        b.sigrok(ctx, i2c('uio1', 'uio0'), i2c_events, i2c_expect([w], 0x50))
    ctx = b.run('std_i2c_nack_addr', std_i2c, 'Sm', 0xA03C, ack=(False,))
    b.sigrok(ctx, i2c('uio1', 'uio0'), i2c_events, i2c_expect([w], 0x50, nack={0}))
    ctx = b.run('std_i2c_nack_data', std_i2c, 'Sm', 0xA03C, ack=(True, False))
    b.sigrok(ctx, i2c('uio1', 'uio0'), i2c_events, i2c_expect([w], 0x50, nack={1}))
    b.close()

    log.group('standard/ws2812.s', title='WS2812 on uo6, one byte, bench/standard.py test_ws2812')
    b = Batch(runner, '0xB2 on 4 threads', 'bits')
    for tid in range(4):
        ctx = b.run('std_ws_t%d' % tid, std_ws, tid, 0b10110010)
        b.sigrok(ctx, ws281x('uo6'), ws281x_frames, ([[1, 0, 1, 1, 0, 0, 1, 0]], []))
    b.close()

    log.group('standard/onewire.s', title='1-Wire controller on io2, bench/standard.py test_onewire')
    b = Batch(runner, 'reset, presence, 0xCC on 4 threads; no device', 'events')
    for tid in range(4):
        ctx = b.run('std_ow_t%d' % tid, std_ow, tid)
        b.sigrok(ctx, onewire('uio2'), onewire_events, onewire_expect([0xCC]))
    ctx = b.run('std_ow_absent', std_ow, 0, False)
    b.sigrok(ctx, onewire('uio2'), onewire_events, ['RP:0'])
    b.close()

    log.group('standard/jtag.s', title='JTAG controller, TCK uo1, TMS uo2, TDI uo5, TDO ui4, bench/standard.py '
              'jtag_once')
    ident, pattern = 0x4BA00477, 0xB3C5
    want = [('DR', 32, None, ident), ('IR', 4, 0xF, 0x1), ('DR', 16, pattern, (pattern << 1) & 0xFFFF)]
    for half in (25, 16):
        b = Batch(runner, 'half period %d, 4 threads: IDCODE, IR to BYPASS, a pattern through BYPASS' % half,
                  'scans')
        for tid in range(4):
            ctx = b.run('std_jtag_h%d_t%d' % (half, tid), standard.jtag_once, half, tid, ident, pattern)
            b.sigrok(ctx, jtag('uo1', 'uo2', 'uo5', 'ui4'), jtag_scans, jtag_match(want))
        b.close()

    log.group('standard/cec.s', title='HDMI CEC initiator, bench/standard.py cec_four and cec_run')
    b = Batch(runner, 'four threads at once, each on its own pin io0 to io3, follower at 4 on each', 'frames')
    ctx = b.run('std_cec_four', std_cec_four)
    one = [[(0x14, 0, 'ACK'), (0x36, 1, 'ACK')]]
    for tid in range(4):
        b.sigrok(ctx, cec('uio%d' % tid), cec_frames, one, tag='io%d' % tid)
    b.close()
    b = Batch(runner, 'no follower at the destination: the header is not acknowledged', 'frames')
    ctx = b.run('std_cec_alone', std_cec_alone)
    want = [[(0x14, 0, 'NACK')]]
    b.sigrok(ctx, cec('uio3'), cec_frames, want,
             known=lambda got, errs: FINDING['cec_rest'] if got == want + [[(0x36, 1, 'NACK')]] else None)
    b.close()

    log.group('standard/crc16.s', title='CRC-16/CCITT-FALSE: no sigrok decoder, checked against binascii.crc_hqx')
    LOG.row('standard/crc16.s').note = 'no decoder: binascii.crc_hqx'
    rng = random.Random(16)
    msgs = [b'123456789', bytes(rng.getrandbits(8) for _ in range(40)), b'\x00' * 8, b'\xff' * 8]
    crc_runs(runner, 'crc16', msgs, 1, lambda m: [binascii.crc_hqx(m, 0xFFFF)], 'binascii.crc_hqx(msg, 0xFFFF)')
    log.group('standard/crc8.s', title='CRC-8/MAXIM: no sigrok decoder, no standard library entry')
    LOG.row('standard/crc8.s').note = 'no decoder: catalogue check value 0xA1'
    crc_runs(runner, 'crc8', [b'123456789'], 1, lambda m: [0xA1], 'the catalogue\'s check value 0xA1')
    log.group('standard/crc32.s', title='CRC-32: no sigrok decoder, checked against zlib.crc32')
    LOG.row('standard/crc32.s').note = 'no decoder: zlib.crc32'
    crc_runs(runner, 'crc32', msgs, 2, lambda m: [zlib.crc32(m) & 0xFFFF, zlib.crc32(m) >> 16], 'zlib.crc32(msg)')


def crc_runs(runner, name, msgs, answers, want, how):
    ok, bad = 0, []
    for k, m in enumerate(msgs):
        for tid in range(4):
            ctx = runner.run(std_feed, name, m, answers, tid)
            LOG.row(LOG.progs[0]).scenarios += 1
            same = True
            if runner.rtl:
                same = [check.record(c) for c in ctx.sim_chips] == [check.record(c) for c in ctx.chips]
            good = ctx.value == want(m) and same
            LOG.outcome('rtl' if runner.rtl else 'sim', good)
            if runner.rtl:
                LOG.outcome('sim', good)
            ok += good
            if not good:
                bad.append('message %d thread %d: %s, wanted %s%s' % (k, tid, ctx.value, want(m),
                                                                       '' if same else ', RTL differs'))
    LOG.check('%d messages x 4 threads: the result equals %s%s' % (len(msgs), how,
                                                                  ', RTL and simulator equal' if runner.rtl else ''),
              not bad, '%d of %d' % (ok, 4 * len(msgs)) + ('; ' + ' | '.join(bad[:3]) if bad else ''))


def g_standard_pairs(runner, log):
    """standard/i2c.s against standard/i2c_target.s on another thread, and the shared-image scenarios."""
    import standard
    log.group('standard/i2c.s', 'standard/i2c_target.s',
              title='I2C controller (io0, io1) and target (io2, io3) on two threads, wired together, '
                    'bench/standard.py run')
    ops = [[0xA010, 0x5A00], [0xA110], [0xA120], [0xB010, 0x5A00]]
    mem = {r: 0xEE for r in range(256)}
    mem[0x20] = 0x3C
    want = eeprom_expect([(0xA0, 0x10, 0x5A), (0xA1, 0x10, 0), (0xA1, 0x20, 0), (0xB0, 0x10, 0x5A)], mem)
    for name, cfg in (('100 kHz', (250, 210)), ('400 kHz', (45, 37))):
        b = Batch(runner, '%s: write, random read of it, random read of a stored byte, wrong address' % name,
                  'events')
        ctx = b.run('std_i2c_pair_%s' % name.split()[0], std_i2c_pair, cfg, ops)
        b.sigrok(ctx, i2c('uio1', 'uio0', ('eeprom24xx', {}, {})), i2c_eeprom_events, want,
                 known=left_data_word(want))
        b.close()

    log.group('standard/uart_tx.s', 'standard/uart_rx.s', 'standard/spi.s', 'standard/i2c_write.s',
              title='the four in one 128-word image, one thread each, bench/standard.py scenario')
    b = Batch(runner, 'UART TX, UART RX, SPI mode 0 and I2C on four threads at once', 'items')
    ctx = b.run('std_four', standard.scenario, ('tx', 'rx', 'spi', 'i2c'))
    b.sigrok(ctx, uart('uo0', 115200), uart_bytes, [0x55, 0xA3], tag='uart_tx', prog='standard/uart_tx.s')
    b.sigrok(ctx, uart('ui3', 500000), uart_bytes, [0x31, 0xC7, 0x05], tag='uart_rx', prog='standard/uart_rx.s')
    b.sigrok(ctx, spi('uo1', 'uo2', 'ui4', 'uo5', 0), spi_transfers, [([0xD2], [0x96]), ([0x3C], [0x96])],
             tag='spi', prog='standard/spi.s')
    b.sigrok(ctx, i2c('uio1', 'uio0'), i2c_events, i2c_expect([[0xA0, 0x3C], [0x7E, 0x81]], None),
             tag='i2c', prog='standard/i2c_write.s')
    b.close()

    log.group('standard/memstress.s', 'standard/uart_tx.s',
              title='a thread hammering memory next to the UART transmitter: no decoder for the memory load, '
                    'the UART decoded')
    LOG.row('standard/memstress.s').note = 'no decoder for itself; uart for its neighbour'
    b = Batch(runner, 'UART at 115200 next to ld/st on thread 3')
    ctx = b.run('std_memstress', std_memstress)
    b.sigrok(ctx, uart('uo0', 115200), uart_bytes, [0x55])
    b.close()


def std_i2c_pair(cfg, ops):
    import standard
    chip, env, res = standard.run(cfg, ops)
    chip.run(20000, env)                    # until the program has taken every word the host posted
    return res


def std_memstress():
    import standard
    from simple import Board, Chip
    words = standard.image()
    a = standard.assemble(standard.read('standard/memstress.s'), base=102, depth=128)
    words.update(a.words)
    chip, b = Chip(words, 128), Board()
    chip.start(0, standard.ORG['tx'])
    chip.start(3, 102)
    chip.post(7, 434)
    chip.run(300, b)
    chip.post(0, 0x55)
    chip.run(6000, b)
    return None


# ---------------------------------------------------------------------------------------------------------------
# trimmed/ and extra/: bench/trimmed.py, bench/extra.py, bench/i2c_eeprom.py

EEPROM = {0x10: 0xC3, 0x20: 0x3C}           # what the EEPROM models hold before a run


def eeprom_run(rel, ops, **kw):
    import i2c_eeprom
    res, dev, r, tr, bus = i2c_eeprom.run(rel, ops, **kw)
    return res


def g_eeprom_controller(runner, log, rel):
    """The host-driven I2C controllers against bench/i2c_eeprom.py's EEPROM: SDA io0, SCL io1."""
    W, RD, BAD = (0xA0, 0x10, 0x5A), (0xA1, 0x20, 0), (0xB0, 0x10, 0x5A)
    st = i2c('uio1', 'uio0', ('eeprom24xx', {}, {}))
    b = Batch(runner, '100 kHz: write, then two random reads with repeated START, 4 threads', 'events')
    for tid in range(4):
        State.thread = tid
        ops = [W, (0xA1, 0x10, 0), RD]
        ctx = b.run('%s_wrr_t%d' % (rel.split('/')[0], tid), eeprom_run, rel, ops)
        b.sigrok(ctx, st, i2c_eeprom_events, eeprom_expect(ops, EEPROM))
    State.thread = 0
    b.close()
    b = Batch(runner, '100 kHz: wrong address; data NACKed; 8 us stretch after every SCL fall; SDA stuck low', 'events')
    ctx = b.run('%s_bad' % rel.split('/')[0], eeprom_run, rel, [BAD])
    want = eeprom_expect([BAD], EEPROM)
    b.sigrok(ctx, st, i2c_eeprom_events, want, known=left_data_word(want))
    ctx = b.run('%s_nack_data' % rel.split('/')[0], eeprom_run, rel, [W], nack_data=True)
    b.sigrok(ctx, st, i2c_eeprom_events, (['S', 'AW:50', 'ACK', 'DW:10', 'ACK', 'DW:5A', 'NACK', 'P'], []))
    ctx = b.run('%s_stretch' % rel.split('/')[0], eeprom_run, rel, [RD], stretch=400)
    b.sigrok(ctx, st, i2c_eeprom_events, eeprom_expect([RD], EEPROM))

    def stuck(c, bus, dv, lvl):
        bus.ext_low |= 1
    ctx = b.run('%s_sda_stuck' % rel.split('/')[0], eeprom_run, rel, [W], hook=stuck)
    b.sigrok(ctx, st, i2c_eeprom_events, ([], []))
    b.close()
    b = Batch(runner, '400 kHz: write and random read', 'events')
    ops = [W, RD]
    ctx = b.run('%s_fast' % rel.split('/')[0], eeprom_run, rel, ops, fast=True)
    b.sigrok(ctx, st, i2c_eeprom_events, eeprom_expect(ops, EEPROM))
    b.close()


def trimmed_four(files, org, size, rx_cfg):
    import trimmed
    words = trimmed.image(files, org, size)
    return trimmed.scenario(('tx', 'rx', 'spi', 'i2c'), words, org, size, rx_cfg)[2:5]


def g_trimmed(runner, log):
    import trimmed
    log.group('trimmed/uart_tx.s', title='UART 8N1 transmit on uo0, bench/trimmed.py uart_tx_alone')
    for rel, per in (('trimmed/uart_tx.s', 434), ('trimmed/uart_tx.s', 13)):
        b = Batch(runner, '%d cycles per bit, 4 threads' % per)
        for tid in range(4):
            ctx = b.run('trim_uart_tx_%d_t%d' % (per, tid), trimmed.uart_tx_alone, rel, per, tid)
            b.sigrok(ctx, uart('uo0', baud_of(per)), uart_bytes, [0x55, 0xA3, 0x00, 0xFF])
        b.close()
    log.group('standard/uart_tx.s', title='UART 8N1 transmit, bench/trimmed.py uart_tx_alone at 13 cycles per bit')
    b = Batch(runner, '13 cycles per bit (3.85 Mbaud), 4 threads')
    for tid in range(4):
        ctx = b.run('trim_std_uart_tx_13_t%d' % tid, trimmed.uart_tx_alone, 'standard/uart_tx.s', 13, tid)
        b.sigrok(ctx, uart('uo0', baud_of(13)), uart_bytes, [0x55, 0xA3, 0x00, 0xFF])
    b.close()
    log.group('trimmed/uart_rx.s', title='UART 8N1 receive on ui3, bench/trimmed.py uart_rx_alone: the sender')
    for per in (434, 50):
        b = Batch(runner, '%d cycles per bit, 4 threads' % per)
        for tid in range(4):
            ctx = b.run('trim_uart_rx_%d_t%d' % (per, tid), trimmed.uart_rx_alone, 'trimmed/uart_rx.s', per, True, tid)
            b.sigrok(ctx, uart('ui3', baud_of(per)), uart_bytes, [0x00, 0xFF, 0x5A, 0xC3])
            if not ctx.value[0]:
                b.fail(ctx.name, 'the firmware read the bytes wrong')
        b.close()
    log.group('trimmed/i2c.s', title='I2C controller, trimmed, against bench/i2c_eeprom.py')
    g_eeprom_controller(runner, log, 'trimmed/i2c.s')
    for files, org, size, rx_cfg, progs, tag in (
            (trimmed.FILES, trimmed.ORG, 128, 2, ('trimmed/uart_tx.s', 'trimmed/uart_rx.s', 'standard/spi.s',
                                                  'trimmed/i2c.s'), '128'),
            (dict(tx='standard/uart_tx.s', rx='standard/uart_rx.s', spi='standard/spi.s', i2c='standard/i2c.s'),
             dict(tx=0, rx=15, spi=37, i2c=59), 512, 1,
             ('standard/uart_tx.s', 'standard/uart_rx.s', 'standard/spi.s', 'standard/i2c.s'), '512')):
        log.group(*progs, title='the four in one %d-word image, one thread each, bench/trimmed.py scenario' % size)
        b = Batch(runner, 'UART TX, UART RX, SPI mode 0 and I2C to the EEPROM at once', 'items')
        ctx = b.run('trim_four_%s' % tag, trimmed_four, files, org, size, rx_cfg)
        b.sigrok(ctx, uart('uo0', 115200), uart_bytes, [0x55, 0xA3], tag='uart_tx', prog=progs[0])
        b.sigrok(ctx, uart('ui3', 500000), uart_bytes, [0x31, 0xC7, 0x05], tag='uart_rx', prog=progs[1])
        b.sigrok(ctx, spi('uo1', 'uo2', 'ui4', 'uo5', 0), spi_transfers, [([0xD2], [0x96]), ([0x3C], [0x96])],
                 tag='spi', prog=progs[2])
        b.sigrok(ctx, i2c('uio1', 'uio0', ('eeprom24xx', {}, {})), i2c_eeprom_events,
                 eeprom_expect([(0xA0, 0x10, 0x5A), (0xA1, 0x20, 0)], EEPROM), tag='i2c', prog=progs[3])
        b.close()


def ex_lin(frames, bits, tail=60000):
    """extra.py lin_run, run on past the last header so that sigrok's lin decoder closes it (it reports a header
    at the next break, or after two idle character times)."""
    import extra
    r = extra.Run(extra.read('extra/lin.s'))
    e, t = [], 3000
    for p in frames:
        f, t = extra.lin_frame(p, bits, round(t))
        e += f
        t += 5 * bits
    st = [0, 1]
    out = []

    def w(c, uo, o, oe):
        while st[0] < len(e) and e[st[0]][0] <= c:
            st[1] = e[st[0]][1]
            st[0] += 1
        return (0xFF & ~(1 << 3)) | st[1] << 3, 0xFF
    while r.c < t + 4000 + tail:
        r.step(w)
        v = r.take()
        if v is not None:
            out.append(v)
    return out


def g_extra(runner, log):
    import extra
    log.group('extra/spi_target.s', title='SPI target, mode 0: CS ui3, SCK ui4, MOSI ui5, MISO uo2, '
              'bench/extra.py spi_target')
    data = [0xA5, 0x3C, 0xFF, 0x00, 0x81, 0x7E]
    want = [([x], [y]) for x, y in zip(data, [0] + data[:-1])]
    for half in (10, 25, 250):
        b = Batch(runner, 'half period %d, 6 bytes, 4 threads at 3 start phases each' % half)
        for tid in range(4):
            for ph in (tid, tid + 4, tid + 8):
                State.thread = tid
                ctx = b.run('ex_spi_target_h%d_t%d_p%d' % (half, tid, ph), extra.spi_target, half, ph, data)
                b.sigrok(ctx, spi('ui4', 'ui5', 'uo2', 'ui3', 0), spi_transfers, want)
                if ctx.value[0] != data:
                    b.fail(ctx.name, 'the firmware read %s' % short(ctx.value[0]))
        b.close()
    State.thread = 0

    log.group('extra/parallel.s', title='8-bit parallel capture: strobe ui4, data io0-7, bench/extra.py par_run')
    LOG.row('extra/parallel.s').note = 'parallel'
    data = [0xA5, 0x3C, 0xFF, 0x00, 0x81, 0x7E, 0x55, 0xAA]
    st = parallel('ui4', ['uio%d' % i for i in range(8)])
    for title, step, args in (('IEEE 1284 minimums, a byte every 75 cycles', 75, (-50, 24, -25)),
                              ('data good from 20 cycles before the strobe rise to the rise', 2000, (-20, 0, -1000)),
                              ('a byte every 28 cycles', 28, (-14, 13, -14))):
        b = Batch(runner, title + ', 4 threads (sigrok shows each byte at the next strobe edge: the last of a run '
                  'is not shown)')
        for tid in range(4):
            State.thread = tid
            rises = [400 + tid + step * i for i in range(len(data))]
            ctx = b.run('ex_parallel_%d_t%d' % (step, tid), extra.par_run,
                        extra.par_events(data, rises, *args), rises[-1] + 2000)
            b.sigrok(ctx, st, parallel_items, data[:-1])
            if ctx.value[0] != data:
                b.fail(ctx.name, 'the firmware captured %s' % short(ctx.value[0]))
        b.close()
    State.thread = 0

    log.group('extra/swd.s', title='SWD read of DPIDR: SWCLK uo1, SWDIO io0, bench/extra.py swd_run')
    b = Batch(runner, 'ACK OK, ACK WAIT, bad read parity, each on 4 threads', 'events')
    for mode, want in (('ok', ['RESET', 'R:IDCODE', 'ACK:OK', 'D:0x2ba01477']),
                       ('wait', ['RESET', 'R:IDCODE', 'ACK:WAIT']),
                       ('badpar', ['RESET', 'R:IDCODE', 'ACK:OK', 'D:0x2ba01477', 'PE:01'])):
        for tid in range(4):
            State.thread = tid
            ctx = b.run('ex_swd_%s_t%d' % (mode, tid), extra.swd_run, mode)
            b.sigrok(ctx, swd('uo1', 'uio0'), swd_events, want)
    State.thread = 0
    b.close()

    log.group('extra/i2c.s', title='I2C controller against bench/i2c_eeprom.py')
    g_eeprom_controller(runner, log, 'extra/i2c.s')

    log.group('extra/lin.s', title='LIN responder header on ui3: the commander model, bench/extra.py lin_run')
    ids = [0x00, 0x3C, 0x3D, 0x15, 0x2A, 0x3F, 0x01, 0x20]
    frames = [extra.lin_pid(x) for x in ids] + [extra.lin_pid(0x15) ^ 0x40, extra.lin_pid(0x2A) ^ 0x80]
    want = []
    for k, p in enumerate(frames):
        want += ['BRK', 'SYNC', 'ID:%02X/%s' % (p & 0x3F, 'ok' if k < len(ids) else 'bad')]
    reply = [extra.lin_pid(x) for x in ids] + [0xFF, 0xFF]
    for err, threads in ((-0.14, (0,)), (0.0, (0, 1, 2, 3)), (0.14, (3,))):
        b = Batch(runner, 'commander bit time %+d%% from 19200 baud: 8 headers, 2 with bad parity (sigrok at the '
                  'commander\'s rate), thread%s %s' % (round(err * 100), 's' if len(threads) > 1 else '',
                                                       ', '.join(map(str, threads))), 'events')
        for tid in threads:
            State.thread = tid
            ctx = b.run('ex_lin_%s%d_t%d' % ('m' if err < 0 else 'p', abs(round(err * 100)), tid), ex_lin, frames,
                        2604 * (1 + err))
            b.sigrok(ctx, lin_stack('ui3', CLK / (2604 * (1 + err))), lin_events, want)
            if ctx.value != reply:
                b.fail(ctx.name, 'the firmware answered %s' % hexs(ctx.value))
        State.thread = 0
        b.close()

    log.group('extra/nec_tx.s', title='NEC IR transmit on uo2, 38 kHz carrier, bench/extra.py nec_run')
    b = Batch(runner, 'address 0x59, command 0x16, threads 0 and 3', 'fields')
    want = ['Leader code', 'Address: 0x59', 'Address#: 0xA6', 'Command: 0x16', 'Command#: 0xE9']
    for tid in (0, 3):
        State.thread = tid
        ctx = b.run('ex_nec_t%d' % tid, extra.nec_run, 0x59, 0x16)
        b.sigrok(ctx, nec('uo2'), nec_events, want)
    State.thread = 0
    b.close()


# ---------------------------------------------------------------------------------------------------------------
# advanced/: bench/dmx.py, ps2.py, mdio.py, jtag_chain.py, swd_host.py, i2c_target.py, usb_*.py, can.py

def windows(chip, starts, back=2):
    """[(lo, hi)]: from just before each start to just before the next one, the last to the end of the run."""
    out = []
    for k, t in enumerate(starts):
        hi = starts[k + 1] - back if k + 1 < len(starts) else chip.c
        out.append((max(0, t - back), hi))
    return out


def dmx_by_packet(b, ctx, line, want, known=None):
    """sigrok's dmx512 0.5.3 goes back to its break search only after slot 512 (or a bad second stop bit), so a
    run with shorter packets is decoded one window per packet, each from just before its break."""
    sig, n = line[:-1], int(line[-1])
    brk = long_lows(ctx.chip, sig, n, 88 * CLK // 1000000)
    got = []
    for k, (lo, hi) in enumerate(windows(ctx.chip, brk)):
        g, _, _ = b.sigrok(ctx, dmx_stack(line), dmx_packets, want[k:k + 1] if k < len(want) else [], tag='p%d' % k,
                           lo=lo, hi=hi, known=known)
        got += g
    if len(brk) != len(want):
        b.fail(ctx.name, '%d breaks on the line, %d packets wanted' % (len(brk), len(want)))
    if not known:
        b.cross(ctx.name, bench_dmx(ctx.chip, sig, n), got)
    return got


def bench_dmx(chip, sig, n):
    import dmx
    frames, errs, m = dmx.dmx_decode(trace_of(chip).wave(sig, n))
    return [[f['start']] + f['slots'] for f in frames]


def dmx_tx(frames, thread=0, align=0, slow=0):
    import dmx
    r, img = dmx.tx_case(frames, thread, align, slow)
    return None


def dmx_rx(frames, first, count, marks=None, **kw):
    """marks: (seed, choices) for random marks between slots, drawn afresh on each call so that the simulator's
    run and the RTL's see the same line."""
    import dmx
    import random
    if marks:
        rr = random.Random(marks[0])
        kw['mbb'] = lambda: rr.choice(marks[1])
    w, r, img = dmx.rx_case(frames, first, count, **kw)
    return w


def g_dmx(runner, log):
    import random
    log.group('advanced/dmx_tx.s', title='DMX512 transmit on uo5, bench/dmx.py tx_case')
    rng = random.Random(7)                  # dmx.py main's draws, in its order
    uni = [0] + [rng.randrange(256) for _ in range(512)]
    b = Batch(runner, 'a full universe: start code and 512 slots', 'slots')
    ctx = b.run('dmx_tx_universe', dmx_tx, [uni])
    b.sigrok(ctx, dmx_stack('uo5'), dmx_packets, [uni], bench=bench_dmx(ctx.chip, 'uo', 5))
    b.close()
    short16 = [[0, 1, 2, 3, 4, 5], [0x17, 9, 8], [0] + list(range(16))]
    b = Batch(runner, '3 short packets, one with start code 0x17, 4 threads x 2 alignments', 'slots')
    for th in range(4):
        for al in (th, (th + 2) % 4):
            ctx = b.run('dmx_tx_short_t%d_a%d' % (th, al), dmx_tx, short16, th, al)
            dmx_by_packet(b, ctx, 'uo5', short16)
    b.close()
    short = [[0, 1, 2, 3, 4, 5], [0x17, 9, 8], [0] + list(range(64))]
    b = Batch(runner, 'a host that stalls 1.4 ms in the middle of a packet', 'slots')
    ctx = b.run('dmx_tx_stall', dmx_tx, short, slow=70000)
    dmx_by_packet(b, ctx, 'uo5', short)
    b.close()

    log.group('advanced/dmx_rx.s', title='DMX512 receive on ui4: the sender model, bench/dmx.py rx_case, with '
              'a 92 us break and 40 ns of mark between slots, which sigrok 0.5.3 needs')
    u = [0] + [rng.randrange(256) for _ in range(512)]
    b = Batch(runner, 'a full universe, windows 1..4, 509..512, 250..257', 'slots')
    for first, cnt in ((1, 4), (509, 4), (250, 8)):
        ctx = b.run('dmx_rx_universe_%d' % first, dmx_rx, [u], first, cnt, brk=4600, mbb=2.0)
        b.sigrok(ctx, dmx_stack('ui4'), dmx_packets, [u], bench=bench_dmx(ctx.chip, 'ui', 4))
        if ctx.value != u[first:first + cnt]:
            b.fail(ctx.name, 'the firmware read %s' % short(ctx.value))
    b.close()
    cases = [('MAB 8 us, the shortest', dict(mab=400)), ('long break 1 ms, MAB 100 us', dict(brk=50000, mab=5000)),
             ('sender +2%', dict(err=0.02)), ('sender -2% with a quarter bit of mark between slots', dict(err=-0.02,
                                                                                                           mbb=50.0)),
             ('marks between slots up to 1 ms', dict(marks=(11, (2.0, 2.0, 400.0, 2000.0, 50000.0))))]
    b = Batch(runner, '3 packets of 24 slots: ' + '; '.join(n for n, _ in cases), 'slots')
    for k, (name, kw) in enumerate(cases):
        f = [[0] + [rng.randrange(256) for _ in range(24)] for _ in range(3)]
        kw = dict(kw)
        kw.setdefault('brk', 4600)
        if 'marks' not in kw:
            kw.setdefault('mbb', 2.0)
        ctx = b.run('dmx_rx_case%d' % k, dmx_rx, f, 3, 4, **kw)
        dmx_by_packet(b, ctx, 'ui4', f)
        if ctx.value != sum((x[3:7] for x in f), []):
            b.fail(ctx.name, 'the firmware read %s' % short(ctx.value))
    f = [[0] + [1] * 10, [0x55] + [2] * 10, [0] + [3] * 10]
    ctx = b.run('dmx_rx_startcode', dmx_rx, f, 2, 2, brk=4600, mbb=2.0)
    dmx_by_packet(b, ctx, 'ui4', f)
    b.close()
    b = Batch(runner, '3 packets of 24 slots on threads 1, 2 and 3', 'slots')
    for tid in (1, 2, 3):
        f = [[0] + [rng.randrange(256) for _ in range(24)] for _ in range(3)]
        ctx = b.run('dmx_rx_t%d' % tid, dmx_rx, f, 3, 4, thread=tid, brk=4600, mbb=2.0)
        dmx_by_packet(b, ctx, 'ui4', f)
        if ctx.value != sum((x[3:7] for x in f), []):
            b.fail(ctx.name, 'the firmware read %s' % short(ctx.value))
    b.close()
    b = Batch(runner, 'the bench\'s own timing: an 88 us break, no mark between slots', 'slots')
    f = [[0] + [rng.randrange(256) for _ in range(24)] for _ in range(3)]
    ctx = b.run('dmx_rx_brk88', dmx_rx, f, 3, 4)
    dmx_by_packet(b, ctx, 'ui4', f, known=lambda got, errs: DECODER['dmx_88'] if got != f else None)
    b.close()


def ps2_run(script, posts=(), thread=0, align=0, cycles=2000000, rise=100):
    """ps2.py case(), then the host inhibits (holds clock low 200 us, Chapweske), which gives sigrok the clock fall
    it needs after the last frame."""
    import ps2
    h = ps2.Ps2Host(script)
    r = ps2.Rig(devices=[h], pullups=(1 << ps2.CK) | (1 << ps2.DT), rise=rise)
    img = ps2.assemble(ps2.src('ps2_dev'), defines={'MIN': thread, 'MOUT': 5}, depth=128)
    r.load(img, {thread: 'start'})
    got = []

    def agent():
        for p in posts:
            if isinstance(p, tuple):
                yield p[0]
                yield from r.host.post(thread, p[1])
            else:
                yield from r.host.post(thread, p)
        while True:
            yield from r.host.take(5, got, tries=1)
    r.board.agent(agent())
    r.start(align)
    r.run(cycles)
    h.script = None
    h.lck = 1
    r.run(200 * ps2.US)
    h.lck = 0
    r.run(100 * ps2.US)
    return [w for _, w in got]


def ps2_device_frames(chip):
    """Device-to-host frames: bursts of 11 clock falls the chip makes, split where the clock rests longer than
    110 us (a PS/2 clock period is 60 to 100 us), with the chip pulling data low at the first (the start bit; in a
    host-to-device frame the host pulls it). Each frame's window runs to the next clock fall on the bus, whoever
    makes it."""
    tr = trace_of(chip)
    ck_chip = [t for t, v in tr.pull_low(3).edges() if v == 1]
    dt_chip = tr.pull_low(2)
    bus = [t for t, v in tr.wave('io', 3).edges() if v == 0]
    bursts, cur = [], []
    for t in ck_chip:
        if cur and t - cur[-1] > 110 * 50:
            bursts.append(cur)
            cur = []
        cur.append(t)
    if cur:
        bursts.append(cur)
    out = []
    for g in bursts:
        if len(g) == 11 and dt_chip.at(g[0]):
            nxt = [t for t in bus if t > g[-1]]
            out.append((g[0], nxt[0] if nxt else None))
    return out


def ps2_check(b, ctx, want):
    got = []
    for k, (t0, t12) in enumerate(ps2_device_frames(ctx.chip)):
        if t12 is None:
            b.fail(ctx.name, 'no clock fall after frame %d' % k)
            continue
        g, _, _ = b.sigrok(ctx, ps2('uio3', 'uio2'), ps2_words, [want[k]] if k < len(want) else [],
                           tag='f%d' % k, lo=t0 - 2, hi=t12 + 2)
        got += g
    if len(got) != len(want):
        b.fail(ctx.name, 'sigrok saw %d device frames, wanted %d' % (len(got), len(want)))


def g_ps2(runner, log):
    log.group('advanced/ps2_dev.s', title='PS/2 device, clock io3, data io2: sigrok 0.5.3 decodes device-to-host '
              'frames only, one window per frame')

    def idle(h):
        yield
    b = Batch(runner, 'the device sends 1C F0 1C', 'frames')
    ctx = b.run('ps2_1c_f0_1c', ps2_run, idle, [0x1C, 0xF0, 0x1C], cycles=300000)
    ps2_check(b, ctx, [(0x1C, True), (0xF0, True), (0x1C, True)])
    b.close()
    b = Batch(runner, '55 80 (parity bits 1 and 0), 4 threads x 2 alignments', 'frames')
    for th in range(4):
        for al in (th, (th + 2) % 4):
            ctx = b.run('ps2_55_80_t%d_a%d' % (th, al), ps2_run, idle, [0x55, 0x80], th, al, 220000)
            ps2_check(b, ctx, [(0x55, True), (0x80, True)])
    b.close()

    import ps2

    def cmds(*seq):
        def s(h):
            yield from h.wait(1000 * ps2.US)
            for c in seq:
                if isinstance(c, tuple):
                    yield from h.tx(c[0], flip_parity=True)
                else:
                    yield from h.tx(c)
                yield from h.wait(6000 * ps2.US)
        return s
    b = Batch(runner, 'the device\'s answers to host commands FF, F2, ED 02, EE F4 FE, and to bad parity', 'frames')
    for name, seq, cyc, want in (('ff', (0xFF,), 350000, [0xFA, 0xAA]), ('f2', (0xF2,), 400000, [0xFA, 0xAB, 0x83]),
                                 ('ed02', (0xED, 0x02), 650000, [0xFA, 0xFA]),
                                 ('ee_f4_fe', (0xEE, 0xF4, 0xFE), 1000000, [0xEE, 0xFA, 0xFA]),
                                 ('badpar', ((0x42,),), 300000, [0xFE])):
        ctx = b.run('ps2_cmd_%s' % name, ps2_run, cmds(*seq), cycles=cyc)
        ps2_check(b, ctx, [(x, True) for x in want])
    b.close()
    b = Batch(runner, 'slow bus, 10 us clock and data rise', 'frames')
    ctx = b.run('ps2_slow', ps2_run, idle, [0xA5, 0x5A], cycles=260000, rise=500)
    ps2_check(b, ctx, [(0xA5, True), (0x5A, True)])
    b.close()


def mdio_sta(half, ops, tco, thread=0, align=0, regs=None):
    import mdio
    r, phy, got, img = mdio.sta_case(half, ops, tco, thread, align, regs)
    return got


def mdio_phy(half, ops, regs, align=0, thread=0):
    import mdio
    r, st, img = mdio.phy_case(half, ops, regs, thread=thread, align=align)
    return st.got


def g_mdio(runner, log):
    import mdio
    log.group('advanced/mdio_sta.s', title='MDIO station: MDC uo1, MDIO io0, bench/mdio.py sta_case')
    ops = [(mdio.hdr(2, 0x11, 2),), (mdio.hdr(1, 0x11, 4), 0xA5C3), (mdio.hdr(2, 0x11, 4),), (mdio.hdr(2, 0x11, 3),)]
    regs = {2: 0x0141, 3: 0x0CC2}
    want = mdio_expect(ops, regs)
    b = Batch(runner, 'read, write, read back, read: half periods 20, 50, 250, PHY output 0 to 300 ns', 'frames')
    for half, tco in ((20, 15), (20, 0), (50, 15), (250, 15)):
        ctx = b.run('mdio_sta_h%d_tco%d' % (half, tco), mdio_sta, half, ops, tco, regs=regs)
        b.sigrok(ctx, mdio_stack('uo1', 'uio0'), mdio_frames, want, bench=bench_mdio(ctx.chip))
    b.close()
    b = Batch(runner, '1.25 MHz, 4 threads x 2 alignments', 'frames')
    for th in range(4):
        for al in (th, (th + 2) % 4):
            ctx = b.run('mdio_sta_t%d_a%d' % (th, al), mdio_sta, 20, ops, 15, th, al, regs=regs)
            b.sigrok(ctx, mdio_stack('uo1', 'uio0'), mdio_frames, want, bench=bench_mdio(ctx.chip))
    b.close()
    log.group('advanced/mdio_phy.s', title='MDIO PHY: MDC ui5 from the station model, MDIO io0, '
              'bench/mdio.py phy_case')
    regs = {2: 0x0141, 3: 0x0CC2, 31: 0xFFFF, 0: 0x8000}
    ops2 = [(mdio.hdr(2, 0x11, 2),), (mdio.hdr(1, 0x11, 5), 0x1234), (mdio.hdr(2, 0x11, 5),),
            (mdio.hdr(2, 0x11, 31),), (mdio.hdr(2, 0x11, 0),)]
    want = mdio_expect(ops2, regs)
    b = Batch(runner, 'half periods 100, 50, 30, 20, 2 alignments each, a station that drives garbage after its hold',
              'frames')
    for half in (100, 50, 30, 20):
        for al in (half % 4, (half + 2) % 4):
            ctx = b.run('mdio_phy_h%d_a%d' % (half, al), mdio_phy, half, ops2, regs, al)
            b.sigrok(ctx, mdio_stack('ui5', 'uio0'), mdio_frames, want)
    b.close()
    b = Batch(runner, 'half period 20 on threads 1, 2 and 3', 'frames')
    for tid in (1, 2, 3):
        ctx = b.run('mdio_phy_t%d' % tid, mdio_phy, 20, ops2, regs, tid, tid)
        b.sigrok(ctx, mdio_stack('ui5', 'uio0'), mdio_frames, want)
    b.close()


def jtag_chain_run(half, thread=0, align=0):
    import jtag_chain
    r, res, pat, img, taps = jtag_chain.case(half, thread, align)
    return pat


def jtag_chain_want(pat):
    """Two TAPs, IR 4 and 5 bits (1149.1: IR capture ends in 01), IDCODE 0x4BA00477 and 0x0BB11477, the second
    nearest TDO; BYPASS in both after the IR scan, so data comes out two bits late."""
    val = lambda b: sum(x << i for i, x in enumerate(b))
    m = lambda n: (1 << n) - 1
    return [('DR', 64, 0, 0x0BB11477 | 0x4BA00477 << 32), ('IR', 9, 0x1FF, 0b000100001),
            ('DR', 40, val(pat), (val(pat) << 2) & m(40)),
            ('DR', 20, val(pat[:20]), (val(pat[:18]) << 2) & m(20))]


def pause_extra_bit(want):
    """sigrok's view of the last scan, which goes through Pause-DR after 10 bits: one more bit after the first 10."""
    def f(got, errs):
        if len(got) != len(want) or got[:-1] != want[:-1]:
            return None
        k, n, tdi, tdo = got[-1]
        drop = lambda v: (v & 0x3FF) | (v >> 11) << 10
        if (k, n) == ('DR', 21) and (drop(tdi), drop(tdo)) == want[-1][2:]:
            return DECODER['jtag_pause']
    return f


def bench_jtag(chip):
    import jtag_chain
    tr = trace_of(chip)
    scans, errs, st, sh = jtag_chain.tap_decode(tr.wave('uo', 1), tr.wave('uo', 2), tr.wave('uo', 5),
                                                tr.wave('ui', 4))
    val = lambda bits: sum(x << i for i, x in enumerate(bits))
    return [('DR' if k == 'D' else 'IR', len(di), val(di), val(do)) for k, di, do in scans]


def g_jtag_chain(runner, log):
    log.group('advanced/jtag.s', title='JTAG scan engine against a two-TAP chain, bench/jtag_chain.py case')
    for title, runs in (('half periods 250, 25, 12, 10, 9', [(h, 0, 0) for h in (250, 25, 12, 10, 9)]),
                        ('half 12, 4 threads x 2 alignments', [(12, th, al) for th in range(4)
                                                                for al in (th, (th + 2) % 4)])):
        b = Batch(runner, title + ': IDCODEs, IR to BYPASS, 40 bits through BYPASS x2, a scan through Pause-DR',
                  'scans')
        for half, th, al in runs:
            ctx = b.run('jtag_chain_h%d_t%d_a%d' % (half, th, al), jtag_chain_run, half, th, al)
            want = jtag_chain_want(ctx.value)
            got, _, _ = b.sigrok(ctx, jtag('uo1', 'uo2', 'uo5', 'ui4'), jtag_scans, jtag_match(want),
                                 known=pause_extra_bit(want))
            b.cross(ctx.name, bench_jtag(ctx.chip), got,
                    known=lambda bench, got: pause_extra_bit(bench)(got, []))
        b.close()


def swd_name(kind, apdp, a):
    """The register names sigrok's swd decoder prints (ADIv5.2 tables 2-4 and 2-5; AP registers by address)."""
    if apdp:
        return '%s:%s AP%d' % ('R' if kind == 'r' else 'W', 'R' if kind == 'r' else 'W', a)
    if kind == 'r':
        return 'R:' + {0: 'IDCODE', 4: 'R CTRL/STAT', 8: 'RESEND', 12: 'RDBUFF'}[a]
    return 'W:' + {0: 'W ABORT', 4: 'W CTRL/STAT', 8: 'W SELECT', 12: 'W RDBUFF'}[a]


def swd_want(ops, results, waits=0):
    """Line reset, JTAG-to-SWD, line reset, then each operation with its ACK and data. results: the host's
    (status, value) per op from the stimulus (the target model's registers)."""
    out = ['RESET', 'JTAG->SWD', 'RESET']
    for op, (st, val) in zip(ops, results):
        req = swd_name(op[0], op[1], op[2])
        for _ in range(waits):
            out += [req, 'ACK:WAIT']
            waits = 0
        out += [req, 'ACK:OK', 'D:0x%08x' % (val if op[0] == 'r' else op[3])]
    return out


def bench_swd(chip):
    """bench/swd_host.py's swd_decode, in swd_events' tokens."""
    import swd_host
    tr = trace_of(chip)
    tx, errs = swd_host.swd_decode(tr.wave('uo', 1), tr.wave('io', 0), tr.wave('iooe', 0))
    acks = {1: 'OK', 2: 'WAIT', 4: 'FAULT', 7: 'NOREPLY'}
    out = []
    for t in tx:
        if t[0] == 'reset':
            out.append('RESET')
        elif t[0] == 'jtag2swd':
            out.append('JTAG->SWD')
        elif t[0] == 'ack':
            out += [swd_name('r' if t[3] else 'w', t[2], t[4]), 'ACK:' + acks.get(t[1], 'ERROR')]
        elif t[0] == 'r':
            out += [swd_name('r', t[1], t[2]), 'ACK:OK', 'D:0x%08x' % t[3]]
            if t[4]:
                p = bin(t[3]).count('1') & 1
                out.append('PE:%d%d' % (p, p ^ 1))
        else:
            out += [swd_name('w', t[1], t[2]), 'ACK:OK', 'D:0x%08x' % t[3]]
    return out


def swd_host_run(half, ops=None, inject=(), tco=3, thread=0, align=0):
    import swd_host
    r, t, got, tx, er, img = swd_host.case(half, ops or swd_host.OPS, inject, tco, thread, align)
    return got


def g_swd_host(runner, log):
    import swd_host
    log.group('advanced/swd.s', title='SWD host: SWCLK uo1, SWDIO io0, bench/swd_host.py case')
    want = swd_want(swd_host.OPS, swd_host.WANT)
    st = swd('uo1', 'uio0')
    b = Batch(runner, 'reset, JTAG-to-SWD, DPIDR, writes, posted AP reads: half 250, 25, 24; target clock-to-out '
              '0, 3, 15', 'events')
    for half, tco in ((250, 3), (25, 3), (24, 3), (25, 0), (25, 15)):
        ctx = b.run('swd_host_h%d_tco%d' % (half, tco), swd_host_run, half, tco=tco)
        b.sigrok(ctx, st, swd_events, want, bench=bench_swd(ctx.chip))
    b.close()
    b = Batch(runner, '1 MHz, 4 threads x 2 alignments', 'events')
    for th in range(4):
        for al in (th, (th + 2) % 4):
            ctx = b.run('swd_host_t%d_a%d' % (th, al), swd_host_run, 25, thread=th, align=al)
            b.sigrok(ctx, st, swd_events, want, bench=bench_swd(ctx.chip))
    b.close()
    b = Batch(runner, 'WAIT 3 times then OK; FAULT then OK; bad read parity then OK', 'events')
    one = [('r', 0, 0)]
    ctx = b.run('swd_host_wait3', swd_host_run, 25, one, [('wait', 3)])
    b.sigrok(ctx, st, swd_events, ['RESET', 'JTAG->SWD', 'RESET'] + ['R:IDCODE', 'ACK:WAIT'] * 3 +
             ['R:IDCODE', 'ACK:OK', 'D:0x2ba01477'])
    ctx = b.run('swd_host_fault', swd_host_run, 25, one * 2, ['fault'])
    b.sigrok(ctx, st, swd_events, ['RESET', 'JTAG->SWD', 'RESET', 'R:IDCODE', 'ACK:FAULT', 'R:IDCODE', 'ACK:OK',
                                   'D:0x2ba01477'])
    ctx = b.run('swd_host_parity', swd_host_run, 25, one * 2, ['parity'])
    b.sigrok(ctx, st, swd_events, ['RESET', 'JTAG->SWD', 'RESET', 'R:IDCODE', 'ACK:OK', 'D:0x2ba01477', 'PE:01',
                                   'R:IDCODE', 'ACK:OK', 'D:0x2ba01477'])
    b.close()


def i2c_tgt_run(mode, script, rise=0, thread=0, align=0, name='i2c_tgt', cycles=None):
    import i2c_target
    r, c, img = i2c_target.run(mode, script, rise, thread, align, name=name, cycles=cycles)
    return c.log


def g_i2c_target(runner, log):
    import i2c_target as T
    data = [0xA5, 0x3C, 0xFF, 0x00, 0x81]

    def s1(c):
        yield from T.write_read(c, 0x50, 4, data, len(data))
    # The controller model writes 5 bytes at 4, then reads them back with a repeated START: the target's memory
    # and its ACKs are the stimulus.
    want = (['S', 'AW:50', 'ACK', 'DW:04', 'ACK'] + sum((['DW:%02X' % x, 'ACK'] for x in data), []) + ['P'] +
            ['S', 'AW:50', 'ACK', 'DW:04', 'ACK', 'Sr', 'AR:50', 'ACK'] +
            sum((['DR:%02X' % x, 'ACK' if i < len(data) - 1 else 'NACK'] for i, x in enumerate(data)), []) + ['P'])
    st = i2c('uio3', 'uio2')
    log.group('advanced/i2c_tgt.s', title='I2C target, 32 bytes: SDA io2, SCL io3, bench/i2c_target.py run')
    b = Batch(runner, 'write 5 bytes, stop, random read with repeated START: Sm, SmH, Fm, FmH, with rise times',
              'events')
    for mode, rises in (('Sm', (0, 50)), ('SmH', (0,)), ('Fm', (0, 15)), ('FmH', (0, 15))):
        for rise in rises:
            ctx = b.run('i2c_tgt_%s_r%d' % (mode, rise), i2c_tgt_run, mode, s1, rise, cycles=3000000)
            b.sigrok(ctx, st, i2c_events, want)
    b.close()
    b = Batch(runner, 'Fm, 4 threads x 2 alignments', 'events')
    for th in range(4):
        for al in (th, (th + 2) % 4):
            ctx = b.run('i2c_tgt_Fm_t%d_a%d' % (th, al), i2c_tgt_run, 'Fm', s1, 10, th, al, cycles=3000000)
            b.sigrok(ctx, st, i2c_events, want)
    b.close()
    log.group('advanced/i2c_fp.s', title='I2C target, 1 MHz build, bench/i2c_target.py run')
    b = Batch(runner, 'Fast-mode Plus minimums Fp and FpH, rise 0 to 120 ns, 2 alignments; Fp on threads 1 to 3',
              'events')
    for mode in ('Fp', 'FpH'):
        for rise in (0, 3, 6):
            for al in (0, 2):
                ctx = b.run('i2c_fp_%s_r%d_a%d' % (mode, rise, al), i2c_tgt_run, mode, s1, rise, 0, al, 'i2c_fp',
                            500000)
                b.sigrok(ctx, st, i2c_events, want)
    for tid in (1, 2, 3):
        ctx = b.run('i2c_fp_t%d' % tid, i2c_tgt_run, 'Fp', s1, 3, tid, tid, 'i2c_fp', 500000)
        b.sigrok(ctx, st, i2c_events, want)
    b.close()


def bench_usb(chip, bit):
    import usb_wire as U
    tr = trace_of(chip)
    pk, errs = U.decode(tr.wave('io', 0), tr.wave('io', 1), bit=bit)
    return [usb_expect(p['bytes']) if p['bytes'] and p['bytes'][0] in USB_PID else 'bytes %s' % hexs(p['bytes'])
            for p in pk]


def usb_tx_run(packets, thread=0, align=0):
    import usb_tx
    r, img = usb_tx.case(packets, thread=thread, align=align)
    return None


def usb_rx_run(packets, **kw):
    import usb_rx
    w, r, img = usb_rx.case(packets, **kw)
    return w


def usb_dev_run(script, packet=None, align=0, run=None):
    import usb_dev
    r, h, w, img = usb_dev.case(script, packet=packet, align=align, run=run)
    return w


def g_usb(runner, log):
    import random
    import usb_rx
    import usb_wire as U
    st = usb_stack('uio0', 'uio1')
    log.group('advanced/usb_tx.s', title='USB low-speed transmit: D+ io0, D- io1, bench/usb_tx.py case')
    rng = random.Random(3)                  # usb_tx.py main's draws, in its order
    cases = [[U.PID['ACK']], U.data_packet('DATA1', [rng.randrange(256) for _ in range(8)]),
             U.data_packet('DATA0', [0xFF] * 8), U.data_packet('DATA0', [0] * 8), U.data_packet('DATA1', []),
             U.data_packet('DATA1', [1, 2, 3, 4, 5, 6, 7]), U.token('IN', 0x15, 1)]
    b = Batch(runner, 'ACK, DATA1 random, DATA0 all ones and all zeros, zero length, odd length, IN', 'packets')
    for k, p in enumerate(cases):
        ctx = b.run('usb_tx_%d' % k, usb_tx_run, [p])
        b.sigrok(ctx, st, usb_packets, [usb_expect(p)], bench=bench_usb(ctx.chip, 33))
    b.close()
    b = Batch(runner, 'stuffing-heavy DATA1, 4 threads x 2 alignments', 'packets')
    for th in range(4):
        for al in (th, (th + 2) % 4):
            p = U.data_packet('DATA1', [rng.choice((0xFF, 0xFF, 0x3F, 0x7E, 0xFC, rng.randrange(256)))
                                        for _ in range(8)])
            ctx = b.run('usb_tx_t%d_a%d' % (th, al), usb_tx_run, [p], th, al)
            b.sigrok(ctx, st, usb_packets, [usb_expect(p)], bench=bench_usb(ctx.chip, 33))
    b.close()

    log.group('advanced/usb_rx.s', title='USB low-speed receive: the sender model on io0 and io1, bench/usb_rx.py')
    b = Batch(runner, 'handshake, token, data at 1.5 Mb/s; random 6-packet runs; skew, rate error, jitter, gaps',
              'packets')
    ps = [[U.PID['ACK']], U.token('IN', 0x15, 1), U.data_packet('DATA1', [1, 2, 3, 4, 5, 6, 7, 8])]
    ctx = b.run('usb_rx_three', usb_rx_run, ps)
    b.sigrok(ctx, st, usb_packets, [usb_expect(p) for p in ps])
    for k, kw in enumerate((dict(), dict(align=2), dict(skew=3), dict(skew=8), dict(err=0.015), dict(err=-0.015),
                            dict(jitter=3), dict(gap=2.0))):
        rr = random.Random(k + 100)
        ps = usb_rx.mix(rr)
        ctx = b.run('usb_rx_mix%d' % k, usb_rx_run, ps, seed=k, **kw)
        b.sigrok(ctx, st, usb_packets, [usb_expect(p) for p in ps])
        if usb_rx.norm(ctx.value) != usb_rx.expect(ps):
            b.fail(ctx.name, 'the firmware read %s' % short(ctx.value))
    b.close()

    log.group('advanced/usb_dev.s', title='USB low-speed device link: the host model and the device on io0, io1, '
              'bench/usb_dev.py')
    rng = random.Random(9)
    payload = [rng.randrange(256) for _ in range(8)]
    ADDR = 0x15

    def s_setup(h):
        yield from h.wait(1500)
        yield from h.send(U.token('SETUP', ADDR, 0))
        yield from h.wait(3 * h.bit)
        yield from h.send(U.data_packet('DATA0', payload))
        yield from h.expect()

    def s_in_nak(h):
        yield from h.wait(1500)
        yield from h.send(U.token('IN', ADDR, 0))
        yield from h.expect()
    dpk = U.data_packet('DATA1', [0x12, 0x01, 0x10, 0x01, 0xFF, 0x00, 0xFF, 0x08])

    def s_in_data(h):
        yield from h.wait(4000)
        yield from h.send(U.token('IN', ADDR, 0))
        yield from h.expect()
        yield from h.send([U.PID['ACK']])
        yield from h.wait(1000)

    def s_other(h):
        yield from h.wait(1500)
        yield from h.send(U.token('IN', 0x16, 0))
        yield from h.expect(limit_bits=16)
        yield from h.wait(800)
        yield from h.send(U.token('OUT', 0x14, 0))
        yield from h.wait(3 * h.bit)
        yield from h.send(U.data_packet('DATA0', [1, 2]))
        yield from h.expect(limit_bits=16)
    bad = U.data_packet('DATA0', [1, 2, 3, 4])
    bad[3] ^= 1

    def s_bad(h):
        yield from h.wait(1500)
        yield from h.send(U.token('OUT', ADDR, 0))
        yield from h.wait(3 * h.bit)
        yield from h.send(bad)
        yield from h.expect(limit_bits=14)

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
    E = usb_expect
    b = Batch(runner, 'SETUP and DATA0 then ACK; IN then NAK; IN then DATA1 and ACK; other addresses; a bad CRC',
              'packets')
    for name, script, packet, want in (
            ('setup', s_setup, None, [E(U.token('SETUP', ADDR, 0)), E(U.data_packet('DATA0', payload)), 'ACK']),
            ('in_nak', s_in_nak, None, [E(U.token('IN', ADDR, 0)), 'NAK']),
            ('in_data', s_in_data, dpk, [E(U.token('IN', ADDR, 0)), E(dpk), 'ACK']),
            ('other', s_other, None, [E(U.token('IN', 0x16, 0)), E(U.token('OUT', 0x14, 0)),
                                      E(U.data_packet('DATA0', [1, 2]))])):
        ctx = b.run('usb_dev_%s' % name, usb_dev_run, script, packet)
        b.sigrok(ctx, st, usb_packets, want, bench=bench_usb(ctx.chip, 33))
    ctx = b.run('usb_dev_bad_crc', usb_dev_run, s_bad)
    wb = [E(U.token('OUT', ADDR, 0)), 'DATA0 [ 01 02 02 04 ]']
    b.sigrok(ctx, st, usb_packets, lambda got, errs: (
        got == wb and len(errs) == 1 and errs[0].startswith('crc16-err'),
        'sigrok %s %s, wanted %s with a CRC16 error and no reply' % (got, errs, wb)))
    b.close()
    b = Batch(runner, 'three SETUP/IN rounds back to back, 4 alignments', 'packets')
    want = []
    for k in range(3):
        want += [E(U.token('SETUP', ADDR, 0)), E(U.data_packet('DATA0', [0xFF] * 8 if k == 1 else [k] * 8)), 'ACK',
                 E(U.token('IN', ADDR, 0)), 'NAK']
    for al in range(4):
        ctx = b.run('usb_dev_seq_a%d' % al, usb_dev_run, s_seq, align=al, run=60000)
        b.sigrok(ctx, st, usb_packets, want, bench=bench_usb(ctx.chip, 33))
    b.close()


def can_rx_run(frames, **kw):
    import can
    r, img, w, sched = can.rx_case(frames, **kw)
    return w


def can_tx_run(ident, data, nodes=None, **kw):
    """nodes: a function that makes the other nodes, called afresh on each run: they keep state."""
    import can
    r, img, st, got = can.tx_case(ident, data, nodes=nodes() if nodes else None, **kw)
    return st


def id_7ff(want):
    def f(got, errs):
        if (got == want if not callable(want) else want(got, [])[0]) and errs and \
                all(e.startswith('Identifier bits 10..4 must not be all recessive') for e in errs):
            return DECODER['can_7ff']
    return f


def can_check(b, ctx, want, bit=100, err=0.0, tag=''):
    """sigrok from bus integration on, and bench/can_wire.py's decoder on the same line as a cross-check: id, data,
    ACK, and whether the CRC field is right (sigrok's field against the frame's CRC-15)."""
    import can_wire as C
    lo = bus_integrated(ctx.chip, 'ui', 6, bit)
    got, errs, d = b.sigrok(ctx, can_stack('ui6', round(CLK / (bit * (1 + err))), 75.0), can_frames, want, lo=lo,
                            tag=tag, known=id_7ff(want))
    fr = C.decode(trace_of(ctx.chip).wave('ui', 6), bit * (1 + err), lo=2000)
    bench = [(f['id'], f['data'], f['ack'], f['crc_ok']) for f in fr]
    mine = [(i, dd, a, c == can_expect(i, dd)[2]) for i, dd, c, a in got]
    b.cross(ctx.name, bench, mine)
    return got, errs, d


def g_can(runner, log):
    import can
    import can_wire as C
    log.group('advanced/can.s', title='CAN 2.0A node: TX uo5, RX ui6 (the bus through the transceiver model), '
              'bench/can.py; sigrok from bus integration on')
    frames = [(0x555, [0xAA]), (0x123, [1, 2, 3, 4, 5, 6, 7, 8]), (0x000, [0] * 8), (0x7FF, [0xFF] * 8), (0x2A5, [])]
    want = [can_expect(i, d) for i, d in frames]
    b = Batch(runner, 'receive 5 frames, all-zero and all-one stuffing: the sender model\'s frames and the chip\'s '
              'ACK', 'frames')
    ctx = b.run('can_rx5', can_rx_run, frames)
    can_check(b, ctx, want)
    b.close()
    b = Batch(runner, 'receive: 8 start phases; sender +-0.5, 1, 2%; transceiver loops 80 to 440 ns', 'frames')
    for al in range(8):
        ctx = b.run('can_rx_phase%d' % al, can_rx_run, frames[:3], align=al % 4, t_start=4000 + al)
        can_check(b, ctx, want[:3])
    for e in (0.005, -0.005, 0.01, -0.01, 0.02, -0.02):
        ctx = b.run('can_rx_err%s%d' % ('m' if e < 0 else 'p', abs(e * 1000)), can_rx_run, frames, err=e)
        can_check(b, ctx, want, err=e)
    for d in ((2, 2), (11, 11)):
        ctx = b.run('can_rx_loop%d' % (d[0] + d[1]), can_rx_run, frames, d1=d[0], d2=d[1])
        can_check(b, ctx, want)
    b.close()
    b = Batch(runner, 'receive a frame with a corrupted CRC: no ACK from the chip', 'frames')
    ctx = b.run('can_rx_badcrc', can_rx_run, frames[:3], flip=1)
    bits, crc = C.with_crc(*frames[1])
    w2 = list(want[:3])
    w2[1] = (frames[1][0], frames[1][1], crc ^ 4, False)
    can_check(b, ctx, w2)
    b.close()
    b = Batch(runner, 'transmit 6 frames, ACKed by another node; 4 alignments x 3 loop delays', 'frames')
    for ident, data in [(0x555, [0xAA]), (0x123, [1, 2, 3, 4, 5, 6, 7, 8]), (0x000, [0] * 8), (0x7FF, [0xFF] * 8),
                        (0x2A5, []), (0x0F0, [0xF0, 0x0F, 0xFF, 0x00])]:
        ctx = b.run('can_tx_%03x_%d' % (ident, len(data)), can_tx_run, ident, data)
        can_check(b, ctx, [can_expect(ident, data)])
        if ctx.value != [0]:
            b.fail(ctx.name, 'the firmware reported %s' % ctx.value)
    for al in range(4):
        for d in ((2, 2), (5, 5), (11, 11)):
            ctx = b.run('can_tx_a%d_loop%d' % (al, d[0] + d[1]), can_tx_run, 0x2F1, [0x5A, 0xFF, 0x00], align=al,
                        d1=d[0], d2=d[1])
            can_check(b, ctx, [can_expect(0x2F1, [0x5A, 0xFF, 0x00])])
    b.close()
    b = Batch(runner, 'transmit with nobody to ACK; arbitration lost and won', 'frames')
    ctx = b.run('can_tx_noack', can_tx_run, 0x123, [1, 2], acked=False)
    can_check(b, ctx, lambda got, errs: (got[:1] == [can_expect(0x123, [1, 2], ack=False)],
                                         'sigrok %s %s' % (short(got), short(errs))))
    mine, theirs = (0x246, [1, 2]), (0x244, [9])
    rb = C.frame_bits(*theirs)
    ctx = b.run('can_arb_lost', can_tx_run, *mine, nodes=lambda: [can.Rival(rb, 100.0), can.Acker([len(rb) - 9],
                                                                                                  100.0)], run=30000)
    can_check(b, ctx, [can_expect(*theirs)])
    mine2, theirs2 = (0x244, [7, 8]), (0x246, [9])
    b2 = C.frame_bits(*mine2)
    ctx = b.run('can_arb_won', can_tx_run, *mine2, nodes=lambda: [can.Rival(C.frame_bits(*theirs2), 100.0),
                                                               can.Acker([len(b2) - 9], 100.0)], run=30000)
    can_check(b, ctx, [can_expect(*mine2)])
    b.close()


# ---------------------------------------------------------------------------------------------------------------
# the shift units of section 5.9: test/test.py's UNIT_UART and UNIT_USB programs on a board like its own (a uio pin
# reads its own drive, or 1 through a pull-up)

def unit_programs():
    """The assembly sources of test/test.py, read as text: that file needs cocotb to import."""
    import ast
    path = os.path.join(check.ROOT, 'test', 'test.py')
    if not os.path.exists(path):
        return {}
    tree = ast.parse(open(path).read())
    out = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) and \
                isinstance(node.value, ast.Constant) and isinstance(node.value.value, str):
            out[node.targets[0].id] = node.value.value
    return out


def unit_uart(src, bursts, bit=40):
    import protolib
    img = protolib.assemble(src, depth=512)
    r = protolib.Rig(depth=512, pullups=0xFF)
    r.load(img, {0: 'start'})
    seen = []

    def agent():
        yield 60
        yield from r.host.post(7, bit)
        for burst in bursts:
            for i, x in enumerate(burst):
                yield from r.host.post(0, x | (0x100 if i == len(burst) - 1 else 0))
            yield from r.host.take(4, seen)
            yield 4 * bit
    r.board.agent(agent())
    r.start()
    r.run(400000, stop=r.agents_done)
    r.run(20 * bit)
    return [w for _, w in seen]


def unit_usb(src, labels):
    import protolib
    img = protolib.assemble(src, depth=512)
    r = protolib.Rig(depth=512, pullups=0xFF)
    r.load(img, {0: 'start', 1: 'rx'})
    got = []

    def agent():
        yield 600
        for lab in labels:
            yield from r.host.post(0, img.labels[lab])
            for n in (4, 5, 6, 7):
                yield from r.host.take(n, got)
            yield 200
    r.board.agent(agent())
    r.start()
    r.run(400000, stop=r.agents_done)
    r.run(400)
    return img


def g_units(runner, log):
    progs = unit_programs()
    log.group('test/test.py', title='the shift units (ISA section 5.9): UNIT_UART and UNIT_USB of test/test.py')
    if 'UNIT_UART' not in progs or 'UNIT_USB' not in progs:
        LOG.check('test/test.py has UNIT_UART and UNIT_USB', False, 'not found')
        return
    bursts = [[0x55, 0xA3, 0x00, 0xFF], [0x81], [0x0F, 0xF0]]
    b = Batch(runner, 'UART 8N1 from thread 0\'s unit at 40 cycles per bit (1.25 Mbaud): on uo0, and on io0')
    ctx = b.run('unit_uart_uo0', unit_uart, progs['UNIT_UART'], bursts)
    b.sigrok(ctx, uart('uo0', CLK / 40), uart_bytes, sum(bursts, []))
    if any(w & 0x80 for w in ctx.value):
        b.fail(ctx.name, 'an underrun was reported: %s' % hexs(ctx.value))
    ctx = b.run('unit_uart_io0', unit_uart, progs['UNIT_UART'].replace('.equ TX uo0', '.equ TX io0'), bursts)
    b.sigrok(ctx, uart('uio0', CLK / 40), uart_bytes, sum(bursts, []))
    b.close()
    b = Batch(runner, 'USB low-speed packets from thread 0\'s unit (NRZI, stuffing, CRC-16 appended) on io4 and '
              'io5, the last with a wrong CRC on purpose', 'packets')
    labels = ('P1', 'P2', 'P3', 'P4', 'P5')
    ctx = b.run('unit_usb', unit_usb, progs['UNIT_USB'], labels)
    img = ctx.value
    want = []
    for lab in labels:
        a = img.labels[lab]
        pid, n = img.words[a] & 255, img.words[a + 1] & 0x7FFF
        data = b''.join(bytes((w & 255, w >> 8)) for w in (img.words[a + 2 + i] for i in range((n + 1) // 2)))[:n]
        want.append('%s [%s ]' % (USB_PID[pid], ''.join(' %02X' % x for x in data)))
    b.sigrok(ctx, usb_stack('uio4', 'uio5'), usb_packets, lambda got, errs: (
        got == want and len(errs) == 1 and errs[0].startswith('crc16-err'),
        'sigrok %s %s, wanted %s with one CRC16 error, on the last' % (got, errs, want)))
    b.close()


# ---------------------------------------------------------------------------------------------------------------
# basic/'s shared images: bench/basic.py conc_setup and conc_run, every program's bus decoded at once

def conc(names, rotate, depth):
    import basic
    pieces, used = basic.conc_setup(names, rotate)
    r, res, targets = basic.conc_run(pieces, list(names), depth)
    r.board.run(20000)                      # the line idles past the last WS2812 frame and 1-Wire slot
    return res


def g_basic_images(runner, log):
    import basic
    full = dict(tx='uart_tx', rx='uart_rx', spi='spi', i2c='i2c_write', ws='ws2812', ow='onewire', mem='memhog')
    for names, rotate, depth, tag in ((('tx', 'rx', 'spi', 'i2c'), 0, 128, '128-word image, UART RX'),
                                      (('tx', 'spi', 'i2c', 'ws'), 1, 128, '128-word image, WS2812'),
                                      (('tx', 'spi', 'i2c', 'ow'), 2, 128, '128-word image, 1-Wire'),
                                      (('tx', 'i2c', 'ow', 'mem'), 3, 512, '512 words with a memory hog')):
        log.group(*['basic/%s.s' % full[n] for n in names], title='%s, one thread each, bench/basic.py conc_run'
                  % tag)
        b = Batch(runner, 'every bus of the image at once', 'items')
        ctx = b.run('basic_image_%d' % rotate, conc, names, rotate, depth)
        # each bus counts for its own program, and for the memory hog that runs beside it
        P = lambda n: ('basic/%s.s' % full[n],) + (('basic/memhog.s',) if 'mem' in names else ())
        if 'tx' in names:
            b.sigrok(ctx, uart('uo0', 115200), uart_bytes, [0x4A, 0x53, 0x00, 0xFF], tag='uart_tx', prog=P('tx'))
        if 'rx' in names:
            b.sigrok(ctx, uart('ui3', 115200), uart_bytes, basic.RX_BYTES, tag='uart_rx', prog=P('rx'))
            if [w for _, w in ctx.value['rx']] != basic.RX_BYTES:
                b.fail(ctx.name, 'the firmware read %s' % short(ctx.value['rx']))
        if 'spi' in names:
            b.sigrok(ctx, spi('uo1', 'uo2', 'ui4', 'uo5', 3), spi_transfers,
                     [([1], [0x96]), ([2, 3], [0x69, 0x3C]), ([4], [0xC3]), ([5, 6], [0x11, 0x22])], tag='spi',
                     prog=P('spi'))
        if 'i2c' in names:
            b.sigrok(ctx, i2c('uio2', 'uio1'), i2c_events, i2c_expect([[0xA0, 0x10, 0x5A], [0xA0, 0x11, 0x5A]],
                                                                         0x50), tag='i2c', prog=P('i2c'))
        if 'ws' in names:
            fr = [[0x10 + k, 0x20, 0x30, 0x40, 0x50, 0x60] for k in range(4)]
            b.sigrok(ctx, ws281x('uo6'), ws281x_frames, ws281x_expect(fr), tag='ws2812', prog=P('ws'))
        if 'ow' in names:
            b.sigrok(ctx, onewire('uio4'), onewire_events, onewire_expect([0xCC, 0x44]), tag='onewire',
                     prog=P('ow'))
        b.close()
    LOG.row('basic/memhog.s').note = 'no decoder for itself; its neighbours decoded'
    log.group('standard/hdlc_tx.s', 'standard/manch_rx.s', 'standard/reflex.s', 'advanced/manch_rx.s',
              title='no sigrok decoder')
    for p, why in (('standard/hdlc_tx.s', 'no decoder: libsigrokdecode has no HDLC'),
                   ('standard/manch_rx.s', 'no decoder: no plain Manchester decoder'),
                   ('standard/reflex.s', 'no decoder: a reaction time, not a protocol'),
                   ('advanced/manch_rx.s', 'no decoder: no plain Manchester decoder')):
        LOG.row(p).note = why
    LOG.info('these four have no sigrok decoder; their benches stay their only check')


# ---------------------------------------------------------------------------------------------------------------
# teeth: broken copies of programs, made in memory from the source text as bench/basic.py check_bugs does, never
# written over the originals. sigrok must catch each one: its payload differs from the stimulus, or it reports an
# error.

class patched_src:
    """Has a bench module's src(name) give text for one program."""

    def __init__(self, module, name, text):
        self.m, self.name, self.text = module, name, text

    def __enter__(self):
        self.old = self.m.src
        self.m.src = lambda n: self.text if n == self.name else self.old(n)

    def __exit__(self, *a):
        self.m.src = self.old


def mutate(text, old, new):
    if old not in text:
        raise ValueError('not in the source: %r' % old)
    return text.replace(old, new, 1)


def g_teeth(runner, log):
    import basic
    import can
    import dmx
    import i2c_target as T
    import protolib
    import swd_host
    import usb_dev
    import usb_wire as U
    log.group('teeth', title='broken copies of programs, each of which sigrok must catch')
    found = []

    def tooth(name, what, fn, stack, extract, want, lo=lambda c: 0, hi=lambda c: None):
        try:
            ctx = runner.run(fn)
        except Exception as e:          # a copy that crashes its bench is not a catch by sigrok
            LOG.check('%s: %s' % (name, what), False, 'the bench crashed on it: %r' % e)
            LOG.outcome('rtl' if runner.rtl else 'sim', False)
            found.append((name, False))
            return
        chip = ctx.chip
        d = decode(runner.vcd(ctx, 'tooth_' + name, chip, lo(chip), hi(chip)), stack)
        got, errs = extract(d)
        caught = got != want or bool(errs) or bool(d.problems)
        said = ('errors %s' % short(errs[:3], 220)) if errs else ('decoded %s, the stimulus wants %s' % (
            short(got, 160), short(want, 160)))
        same = True
        if runner.rtl:
            same = [check.record(c) for c in ctx.sim_chips] == [check.record(c) for c in ctx.chips]
        LOG.check('%s: %s' % (name, what), caught and same, ('caught: ' if caught else 'NOT caught: ') + said +
                  ('' if same else '; the RTL differs from the simulator'))
        LOG.outcome('rtl' if runner.rtl else 'sim', caught and same)
        if runner.rtl:
            LOG.outcome('sim', caught)
        found.append((name, caught))

    t = basic.src('uart_tx')
    data = (0x55, 0xAA, 0x00, 0xFF, 0x31, 0x80)
    bad = mutate(t, 'recv  R2, MCFG      ; R2 = cycles per bit, sent by the host once',
                 'recv  R2, MCFG\n        addi  R2, 3')
    tooth('uart_6pct', 'basic/uart_tx.s with its bit 3 cycles long, 53 for 50: 6% slow at 1 Mbaud',
          lambda: basic.uart_tx_case(bad, 1_000_000, data=data), uart('uo0', 1_000_000), uart_bytes, list(data))

    short16 = [[0, 1, 2, 3, 4, 5], [0x17, 9, 8], [0] + list(range(16))]
    bad = mutate(dmx.src('dmx_tx'), 'li    R5, 400       ; two stop bits', 'li    R5, 200')

    def dmx_one():
        with patched_src(dmx, 'dmx_tx', bad):
            return dmx_tx(short16)
    tooth('dmx_stop', 'advanced/dmx_tx.s with one stop bit, where DMX512 has two', dmx_one, dmx_stack('uo5'),
          dmx_packets, short16[:1], hi=lambda c: long_lows(c, 'uo', 5, 4400)[1] - 2)

    frame = (0x123, [1, 2, 3, 4, 5, 6, 7, 8])
    for name, what, old, new in (
            ('can_crc', 'advanced/can.s sending with the CRC polynomial 0x4598 for 0x4599',
             'tx:     li    R7, 0x8B32', 'tx:     li    R7, 0x8B30'),
            ('can_stuff', 'advanced/can.s stuffing after six equal bits, not five',
             'jf    same\n        ldi   R4, 4', 'jf    same\n        ldi   R4, 5')):
        badc = mutate(can.src('can'), old, new)
        ident, dat = frame if name == 'can_crc' else (0x000, [0] * 8)

        def can_one(badc=badc, ident=ident, dat=dat):
            with patched_src(can, 'can', badc):
                return can_tx_run(ident, dat)
        tooth(name, what, can_one, can_stack('ui6', 500000, 75.0), can_frames, [can_expect(ident, dat)],
              lo=lambda c: bus_integrated(c, 'ui', 6, 100))

    progs = unit_programs()
    if 'UNIT_USB' in progs:
        badu = mutate(progs['UNIT_USB'], 'li    R0, 0x8005\n        scfg  R0, POLYHI     ; CRC-16',
                      'li    R0, 0x8004\n        scfg  R0, POLYHI')
        tooth('usb_crc16', 'test.py UNIT_USB with the send half\'s CRC polynomial 0x8004 for 0x8005',
              lambda: unit_usb(badu, ('P1', 'P2')), usb_stack('uio4', 'uio5'), usb_packets,
              ['DATA0 [ 00 01 02 03 ]', 'DATA1 [ FF FF FF 3F 7E 01 80 00 ]'])
    rng = __import__('random').Random(9)
    payload = [rng.randrange(256) for _ in range(8)]

    def s_setup(h):
        yield from h.wait(1500)
        yield from h.send(U.token('SETUP', 0x15, 0))
        yield from h.wait(3 * h.bit)
        yield from h.send(U.data_packet('DATA0', payload))
        yield from h.expect()
    badd = mutate(usb_dev.src('usb_dev'), 'HACK:   .data 0xD280', 'HACK:   .data 0xD2C0')

    def usb_sync():
        with patched_src(usb_dev, 'usb_dev', badd):
            return usb_dev_run(s_setup)
    tooth('usb_sync', 'advanced/usb_dev.s answering with SYNC 0xC0 for 0x80', usb_sync, usb_stack('uio0', 'uio1'),
          usb_packets, [usb_expect(U.token('SETUP', 0x15, 0)), usb_expect(U.data_packet('DATA0', payload)), 'ACK'])

    write = [0xA0, 0x10, 0x5A, 0xC3]
    bad = mutate(basic.src('i2c_write'), '        edge  SDA, 1, R4    ; STOP: SDA rises while SCL is high',
                 '        tick  R4')
    tooth('i2c_stop', 'basic/i2c_write.s without the STOP\'s SDA rise, two transactions',
          lambda: basic.i2c_case(bad, 'Sm', [write, write[:2]]), i2c('uio2', 'uio1'), i2c_events,
          i2c_expect([write, write[:2]], 0x50))
    data5 = [0xA5, 0x3C, 0xFF, 0x00, 0x81]

    def s1(c):
        yield from T.write_read(c, 0x50, 4, data5, len(data5))
    bad = mutate(T.src('i2c_tgt'), 'ackdata: until SCL, 1\n        shl8  R0, SDA, -\n        put   SDA, 0',
                 'ackdata: until SCL, 1\n        shl8  R0, SDA, -\n        put   SDA, 1')

    def i2c_ack():
        with patched_src(T, 'i2c_tgt', bad):
            return i2c_tgt_run('Fm', s1, 10, cycles=3000000)
    tooth('i2c_ack', 'advanced/i2c_tgt.s leaving SDA high in the ACK slot of a data byte', i2c_ack,
          i2c('uio3', 'uio2'), i2c_events,
          ['S', 'AW:50', 'ACK', 'DW:04', 'ACK'] + sum((['DW:%02X' % x, 'ACK'] for x in data5), []) + ['P'] +
          ['S', 'AW:50', 'ACK', 'DW:04', 'ACK', 'Sr', 'AR:50', 'ACK'] +
          sum((['DR:%02X' % x, 'ACK' if i < 4 else 'NACK'] for i, x in enumerate(data5)), []) + ['P'])

    import random
    # In a zero-delay record a line that changes in the very cycle of the sampling edge reads as its new value, so
    # a copy that moves MOSI on the sampling edge (bench/basic.py's own swapped-edge copy) decodes as if right;
    # sigrok's spi decoder has no setup check (the bench's decoder has). A copy that ignores CPHA, run against a
    # mode 1 target, shifts on the wrong edge, and that sigrok sees.
    bad = mutate(basic.src('spi'), '.if CPOL ^ CPHA', '.if CPOL')
    rr = random.Random(7 + 1)
    rep = [rr.getrandbits(8) for _ in range(4)]
    tooth('spi_mode', 'basic/spi.s ignoring CPHA, so mode 1 runs as mode 0, against a mode 1 target',
          lambda: basic.spi_case(bad, 1, 25), spi('uo1', 'uo2', 'ui4', 'uo5', 1), spi_transfers,
          [([0xA5], rep[:1]), ([0x3C, 0xFF, 0x00], rep[1:])])

    bad = mutate(swd_host.src('swd'), '        oe    IO, 0         ; turnaround: let go on this falling edge',
                 '        put   IO, 0')

    def swd_trn():
        with patched_src(swd_host, 'swd', bad):
            return swd_host_run(25, [('r', 0, 0)])
    tooth('swd_turnaround', 'advanced/swd.s holding SWDIO low through the turnaround instead of letting go',
          swd_trn, swd('uo1', 'uio0'), swd_events, ['RESET', 'JTAG->SWD', 'RESET', 'R:IDCODE', 'ACK:OK',
                                                    'D:0x2ba01477'])

    bad = mutate(basic.src('onewire'), '.equ TRSTL 24500', '.equ TRSTL 20000')
    tooth('onewire_reset', 'basic/onewire.s with a reset low of 400 us, under the 480 us minimum',
          lambda: basic.ow_case(bad, [0xA5]), onewire('uio4'), onewire_events, onewire_expect([0xA5]))
    LOG.info('teeth: %d of %d caught' % (sum(c for _, c in found), len(found)))


def rig_uart_decode(w, baud):
    import rig
    return rig.uart_decode(w, baud)


GROUPS = [
    ('basic_uart', g_basic_uart),
    ('basic_spi', g_basic_spi),
    ('basic_i2c', g_basic_i2c),
    ('basic_ws2812', g_basic_ws),
    ('basic_onewire', g_basic_ow),
    ('standard', g_standard),
    ('standard_pairs', g_standard_pairs),
    ('trimmed', g_trimmed),
    ('extra', g_extra),
    ('dmx', g_dmx),
    ('ps2', g_ps2),
    ('mdio', g_mdio),
    ('jtag_chain', g_jtag_chain),
    ('swd_host', g_swd_host),
    ('i2c_target', g_i2c_target),
    ('usb', g_usb),
    ('can', g_can),
    ('units', g_units),
    ('basic_images', g_basic_images),
    ('teeth', g_teeth),
]
