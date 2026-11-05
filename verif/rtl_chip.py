#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""The RTL behind the class interface of a Python ISA simulator.

Chip has the interface of the cycle simulator in sim/sim.py: cycle(ui, uio), pads(ui) and byte-level
host_* calls between cycles. ChipV2 has a second interface, which takes an env object and speaks
xfer(). ChipV2.xfer is Python over the flops, so it never runs pe_spi or the byte logic of pe_host;
ChipV2Pads runs the same xfer() through the real SPI block at the pads, and is the one that tests the
link. All run the Verilator model in build/ through verif/shim.cpp.

A host call between cycles means h = c - 1. The model holds the clock edge that ends a cycle until
something needs it, so the first host call after cycle(c) goes through the real command block in
cycle c. Anything that cannot, such as a second command in the same gap, is applied to the flops
directly with the same effect. On silicon commands are at least 48 cycles apart, so that case exists
only in zero-time benches.
"""
import ctypes
import os
import random

HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.environ.get('PE_BUILD', os.path.join(HERE, 'build'))

# Command codes of pe_host.sv.
C_NONE, C_CAPS, C_WRMEM, C_RDMEM, C_RUN, C_SETPC, C_STATUS, C_STEP, C_RDREG, C_POST, C_TAKE = range(11)
ISA_VERSION = 0x01
THREADS = 4
LOG2 = {128: 7, 512: 9}
LANES = (0, 4, 5, 6, 7)

FIELDS = ('pc_q', 't_q', 'd_q', 's_q', 'late_q', 'stale_q', 'ill_q', 'run_q', 'p_q', 'kind_q', 'due_q', 'waiting_q',
          'mask_q', 'pat_q', 'bank_q', 'ne_q', 'timed_q', 'inbox_q', 'stopreq_q', 'stepping_q', 'mem_q', 'mem_done_q',
          'mem_kind_q', 'mem_reg_q', 'run_post', 'tb_q', 'sync1_q', 'sync2_q', 'valid2_q', 'valid3_q', 'valid4_q',
          'ir3_q', 'ir4_q', 'ls_addr_q', 'ls_data_q', 'dr_r', 'out_q', 'shd_q', 'oe_q', 'shoe_q', 'od_q', 'byp_q',
          'shbyp_q', 'own_q', 'full_q', 'idx_q', 'cmd_q', 'n_q', 'arg_q', 'thread_q', 'addr_q', 'hold_q', 'mem_req_q',
          'mem_write_q', 'mem_read_q', 'rd_req_q', 'rd_cap_q', 'busy_q', 'tx_byte', 'reg0', 'm0')

_libs = {}


def _load(kind, depth):
    key = (kind, depth)
    if key not in _libs:
        path = os.path.join(BUILD, '%s_%d' % (kind, depth), 'libpe.so')
        lib = ctypes.CDLL(path)
        u, i, vp = ctypes.c_uint, ctypes.c_int, ctypes.c_void_p
        for name, res, args in (
                ('pe_new', vp, []), ('pe_free', None, [vp]), ('pe_field', i, [vp, ctypes.c_char_p]),
                ('pe_cycle', None, [vp, u, u]), ('pe_pads', u, [vp, u]), ('pe_flush', None, [vp]),
                ('pe_get', u, [vp, i]), ('pe_set', None, [vp, i, u]), ('pe_peek', u, [vp, i]),
                ('pe_poke', None, [vp, i, u]),
                ('pe_snapshot', None, [vp, vp]), ('pe_field_count', i, [vp]),
                ('pe_mem_get', u, [vp, u]), ('pe_mem_set', None, [vp, u, u]),
                ('pe_mem_fill', None, [vp, vp, u]), ('pe_mem_dump', None, [vp, vp, u]),
                ('pe_host', i, [vp, u, u, u, u, u, u]), ('pe_log_enable', None, [vp, i]),
                ('pe_log_drain', i, [vp, vp, i]), ('pe_late_events', ctypes.c_uint64, [vp, i]),
                ('pe_late_at', ctypes.c_int64, [vp, i]),
                ('pe_host_mem', None, [vp, vp]), ('pe_rst', None, [vp, u]), ('pe_raw_edge', None, [vp])):
            f = getattr(lib, name)
            f.restype, f.argtypes = res, args
        _libs[key] = lib
    return _libs[key]


class Bits:
    """A list view of the bits of one flop vector."""

    def __init__(self, chip, field, width):
        self.chip, self.field, self.width = chip, field, width

    def __len__(self):
        return self.width

    def __getitem__(self, i):
        if isinstance(i, slice):
            return list(self)[i]
        return (self.chip._get(self.chip._h, self.field) >> i) & 1

    def __setitem__(self, i, v):
        x = self.chip._get(self.chip._h, self.field)
        self.chip._set(self.chip._h, self.field, (x & ~(1 << i)) | ((v & 1) << i))

    def __iter__(self):
        x = self.chip._get(self.chip._h, self.field)
        return iter([(x >> i) & 1 for i in range(self.width)])

    def __eq__(self, other):
        return list(self) == list(other)

    def __repr__(self):
        return repr(list(self))


class Own(Bits):
    """OWN: two bits per pin."""

    def __getitem__(self, i):
        return (self.chip._get(self.chip._h, self.field) >> (2 * i)) & 3

    def __setitem__(self, i, v):
        x = self.chip._get(self.chip._h, self.field)
        self.chip._set(self.chip._h, self.field, (x & ~(3 << (2 * i))) | ((v & 3) << (2 * i)))

    def __iter__(self):
        x = self.chip._get(self.chip._h, self.field)
        return iter([(x >> (2 * i)) & 3 for i in range(self.width)])


class Words:
    """A list view of consecutive 16-bit fields: a thread's registers, or the mailbox."""

    def __init__(self, chip, base, n):
        self.chip, self.base, self.n = chip, base, n

    def __len__(self):
        return self.n

    def __getitem__(self, i):
        if isinstance(i, slice):
            return list(self)[i]
        return self.chip._get(self.chip._h, self.base + i)

    def __setitem__(self, i, v):
        self.chip._set(self.chip._h, self.base + i, v & 0xFFFF)

    def __iter__(self):
        return iter([self.chip._get(self.chip._h, self.base + i) for i in range(self.n)])

    def __eq__(self, other):
        return list(self) == list(other)

    def __repr__(self):
        return repr(list(self))


class Memory:
    """A list view of the SRAM model, DEPTH words long."""

    def __init__(self, chip):
        self.chip = chip

    def __len__(self):
        return self.chip.DEPTH

    def __getitem__(self, a):
        if isinstance(a, slice):
            return list(self)[a]
        return self.chip._lib.pe_mem_get(self.chip._h, a)

    def __setitem__(self, a, v):
        self.chip._lib.pe_mem_set(self.chip._h, a, v & 0xFFFF)

    def __iter__(self):
        n = self.chip.DEPTH
        buf = (ctypes.c_uint16 * n)()
        self.chip._lib.pe_mem_dump(self.chip._h, buf, n)
        return iter(list(buf))


def _flop(field):
    def get(self):
        return self._c._get(self._c._h, self._c._f[field] + self.TID)

    def put(self, v):
        self._c._set(self._c._h, self._c._f[field] + self.TID, int(v))
    return property(get, put)


class Thread:
    """One thread's flops, by the names the simulators use."""

    def __init__(self, chip, tid):
        self._c = chip
        self.TID = tid
        self._R = Words(chip, chip._f['reg0'] + 8 * tid, 8)

    R = property(lambda s: s._R, lambda s, v: [s._R.__setitem__(i, x) for i, x in enumerate(v)])
    PC = _flop('pc_q')
    T = _flop('t_q')
    D = _flop('d_q')
    S = _flop('s_q')
    LATE = _flop('late_q')
    STALE = _flop('stale_q')
    ILL = _flop('ill_q')
    RUN = _flop('run_q')
    P = _flop('p_q')
    KIND = _flop('kind_q')
    DUE = _flop('due_q')
    WAITING = _flop('waiting_q')
    MASK = _flop('mask_q')
    PAT = _flop('pat_q')
    BANK = _flop('bank_q')
    NE = _flop('ne_q')
    TIMED = _flop('timed_q')
    INBOX = _flop('inbox_q')
    STOPREQ = _flop('stopreq_q')
    STEPPING = _flop('stepping_q')
    MEM = _flop('mem_q')
    MEM_DONE = _flop('mem_done_q')
    MEM_KIND = _flop('mem_kind_q')
    MEM_REG = _flop('mem_reg_q')

    @property
    def IV(self):
        """There is no IV flop: the valid bit travels with the word, so it is read from the stage this
        thread is in. Once the word has executed it is gone, and IV can't change anything there."""
        c = self._c
        stage = (self.TID - c._get(c._h, c._f['tb_q'])) & 3
        if stage == 3:
            raise AttributeError('thread %d has no word in the pipeline in this cycle' % self.TID)
        return c._get(c._h, c._f[('valid4_q', 'valid3_q', 'valid2_q')[stage]])

    @property
    def latecount(self):
        return self._c._lib.pe_late_events(self._c._h, self.TID)

    @property
    def late_at(self):
        v = self._c._lib.pe_late_at(self._c._h, self.TID)
        return None if v < 0 else v


class Chip:
    """Same interface as a cycle simulator: pads(ui), cycle(ui, uio) and host_* between cycles."""

    def __init__(self, depth=128, seed=1, kind='stub'):
        self.DEPTH = depth
        self._lib = lib = _load(kind, depth)
        self._get, self._set = lib.pe_get, lib.pe_set
        self._h = lib.pe_new()
        n = lib.pe_field_count(self._h)
        self._f = {name: lib.pe_field(self._h, name.encode()) for name in FIELDS}
        self._n_fields = n
        self._cycle, self._pads = lib.pe_cycle, lib.pe_pads
        rng = random.Random(seed)       # unwritten memory is arbitrary, and the same as a simulator's
        self.mem = [rng.getrandbits(16) for _ in range(depth)]
        self.c = 0
        self.th = [Thread(self, i) for i in range(THREADS)]
        f = self._f
        self._M = Words(self, f['m0'], 8)
        self._FULL = Bits(self, f['full_q'], 8)
        self._OUT = Bits(self, f['out_q'], 16)
        self._SHD = Bits(self, f['shd_q'], 16)
        self._OE = Bits(self, f['oe_q'], 8)
        self._SHOE = Bits(self, f['shoe_q'], 8)
        self._OD = Bits(self, f['od_q'], 8)
        self._BYP = Bits(self, f['byp_q'], 8)
        self._SHBYP = Bits(self, f['shbyp_q'], 8)
        self._OWN = Own(self, f['own_q'], 16)
        self._log = None
        self.host_miso = 0

    def __del__(self):
        h = getattr(self, '_h', None)
        if h:
            self._lib.pe_free(h)
            self._h = None

    def _view(name):
        def put(self, values):
            view = getattr(self, name)
            for i, v in enumerate(values):
                view[i] = v
        return property(lambda self: getattr(self, name), put)

    M = _view('_M')
    FULL = _view('_FULL')
    OUT = _view('_OUT')
    SHD = _view('_SHD')
    OE = _view('_OE')
    SHOE = _view('_SHOE')
    OD = _view('_OD')
    BYP = _view('_BYP')
    SHBYP = _view('_SHBYP')
    OWN = _view('_OWN')
    del _view

    @property
    def mem(self):
        return Memory(self)

    @mem.setter
    def mem(self, words):
        words = list(words)
        buf = (ctypes.c_uint16 * len(words))(*words)
        self._lib.pe_mem_fill(self._h, buf, len(words))

    @property
    def SYNC1(self):
        return self._get(self._h, self._f['sync1_q'])

    @property
    def SYNC2(self):
        return self._get(self._h, self._f['sync2_q'])

    @property
    def BUSY(self):
        return self._get(self._h, self._f['busy_q'])

    @BUSY.setter
    def BUSY(self, v):
        self._set(self._h, self._f['busy_q'], v)

    @property
    def log(self):
        if self._log is None:
            return None
        lib, h = self._lib, self._h
        buf = (ctypes.c_uint64 * 4096)()
        while True:
            n = lib.pe_log_drain(h, buf, 1024)
            for i in range(n):
                self._log.append((buf[4 * i], buf[4 * i + 1], buf[4 * i + 2], buf[4 * i + 3]))
            if n < 1024:
                return self._log

    @log.setter
    def log(self, value):
        self._log = value
        self._lib.pe_log_enable(self._h, 0 if value is None else 1)

    def next(self, t):
        return (t.PC + 1) % self.DEPTH

    def pads(self, ui_in):
        v = self._pads(self._h, ui_in)
        return v & 255, (v >> 8) & 255, (v >> 16) & 255

    def cycle(self, ui_in, uio_in):
        self._cycle(self._h, ui_in, uio_in)
        self.c += 1

    def snapshot(self):
        """Every field of the shim's table as a list, indexed by the ids in self._f."""
        buf = (ctypes.c_uint32 * self._n_fields)()
        self._lib.pe_snapshot(self._h, buf)
        return list(buf)

    # ---- host commands. Between cycles: h = c - 1 ------------------------------------------------
    def _host(self, cmd, idx, byte, arg=0, n=0, thread=0):
        return self._lib.pe_host(self._h, cmd, idx, arg, n, thread, byte)

    # The slow commands on their real path. They land in a free memory cycle or a thread's slot, a
    # few cycles after h. A byte-level simulator has them land at h, so a lockstep bench lines the
    # two up with host_mem() and hold().
    def start_wrmem(self, addr, data):
        self._lib.pe_poke(self._h, self._f['addr_q'], addr % self.DEPTH)
        return self._host(C_WRMEM, 4, data & 255, arg=(data >> 8) & 255)

    def start_rdmem(self, addr):
        return self._host(C_RDMEM, 2, addr & 255, arg=(addr >> 8) & 255)

    def start_rdreg(self, tid, reg):
        return self._host(C_RDREG, 2, reg & 255, thread=tid & 3)

    def host_mem(self):
        """The last memory access granted to the host: (cycle, addr, data, write)."""
        buf = (ctypes.c_int64 * 4)()
        self._lib.pe_host_mem(self._h, buf)
        return tuple(buf)

    def hold(self):
        return self._get(self._h, self._f['hold_q'])

    def host_wrmem(self, addr, data):
        if all(t.RUN for t in self.th):
            self.BUSY = 1
            return False
        self.mem[addr % self.DEPTH] = data & 0xFFFF
        return True

    def host_rdmem(self, addr):
        if all(t.RUN for t in self.th):
            self.BUSY = 1
            return 0
        return self.mem[addr % self.DEPTH]

    def host_setpc(self, tid, pc):
        if self._host(C_SETPC, 3, pc & 255, arg=(pc >> 8) & 255, thread=tid & 3) >= 0:
            return
        t = self.th[tid & 3]
        if t.RUN:
            self.BUSY = 1
            return
        t.PC = pc % self.DEPTH

    def host_run(self, stop, start):
        if self._host(C_RUN, 2, start & 255, arg=stop & 255) >= 0:
            return
        h = (self.c - 1) & 0xFFFF
        for t in self.th:
            s, g = (stop >> t.TID) & 1, (start >> t.TID) & 1
            if s:
                if t.RUN:
                    t.STOPREQ = 1
                else:
                    t.P = 0
            elif g and not t.RUN:
                t.RUN = 1
                t.D = h
                t.P = 0
                t.WAITING = 0
                t.STOPREQ = 0
                t.LATE = 0
                t.ILL = 0
                t.STALE = 0

    def host_step(self, tid):
        if self._host(C_STEP, 1, tid & 3) >= 0:
            return
        t = self.th[tid & 3]
        if t.RUN:
            self.BUSY = 1
            return
        t.RUN = 1
        t.STEPPING = 1
        t.WAITING = 0

    def host_rdreg(self, tid, reg):
        real = self._host(C_RDREG, 2, reg & 255, thread=tid & 3) >= 0
        t = self.th[tid & 3]
        if real:                        # the value comes from the flops, at once; drop the slower path
            self._set(self._h, self._f['rd_req_q'], 0)
        if t.RUN:
            if not real:
                self.BUSY = 1
            return 0
        if reg < 8:
            return t.R[reg]
        return {8: t.D, 9: t.S, 10: t.PC,
                11: t.T | (t.LATE << 1) | (t.STALE << 2) | (t.WAITING << 3)}.get(reg, 0)

    def host_post(self, n, data):
        """Result bit 0 is 1 if refused."""
        r = self._host(C_POST, 2, data & 255, arg=(data >> 8) & 255, n=n)
        if r >= 0:
            return r & 1
        if self.FULL[n]:
            return 1
        self.M[n] = data & 0xFFFF
        self.FULL[n] = 1
        return 0

    def host_take(self, n):
        r = self._host(C_NONE, 0, 0x28 + n)
        if r >= 0:
            if r & 1:
                return 1, self._get(self._h, self._f['hold_q'])
            return 0, 0
        if self.FULL[n]:
            self.FULL[n] = 0
            return 1, self.M[n]
        return 0, 0

    def host_status(self):
        busy = self._lib.pe_peek(self._h, self._f['busy_q'])
        real = self._host(C_STATUS, 4, 0) >= 0
        if not real:
            busy = self.BUSY
            self.BUSY = 0

        def nib(name):
            return sum(getattr(t, name) << t.TID for t in self.th)
        full = sum(f << i for i, f in enumerate(self.FULL))
        return [(nib('RUN') << 4) | nib('ILL'), (nib('LATE') << 4) | nib('STALE'), full, busy]

    def host_abort(self):
        if self._host(C_NONE, 0, 0x3F) >= 0:
            return
        for t in self.th:
            t.P = 0
            t.WAITING = 0
            if t.RUN:
                t.STOPREQ = 1
        for i in range(8):
            self.OE[i] = 0
            self.SHOE[i] = 0
            if self.OD[i]:
                self.OUT[8 + i] = 1
                self.SHD[8 + i] = 1


def _bits(view):
    return sum(v << i for i, v in enumerate(view))


class ThreadV2:
    """One thread under the lower-case names of the ChipV2 interface."""

    def __init__(self, t):
        self._t = t
        self.id = t.TID

    r = property(lambda s: s._t.R, lambda s, v: setattr(s._t, 'R', v))
    pc = property(lambda s: s._t.PC, lambda s, v: setattr(s._t, 'PC', v))
    T = property(lambda s: s._t.T, lambda s, v: setattr(s._t, 'T', v))
    D = property(lambda s: s._t.D, lambda s, v: setattr(s._t, 'D', v))
    S = property(lambda s: s._t.S, lambda s, v: setattr(s._t, 'S', v))
    P = property(lambda s: s._t.P)
    late = property(lambda s: s._t.LATE)
    stale = property(lambda s: s._t.STALE)
    ill = property(lambda s: s._t.ILL)
    run = property(lambda s: s._t.RUN)
    waiting = property(lambda s: s._t.WAITING)
    stopreq = property(lambda s: s._t.STOPREQ)
    stepping = property(lambda s: s._t.STEPPING)
    latecount = property(lambda s: s._t.latecount)


class ChipV2:
    """The env-object interface: cycle(env), start, post, take, xfer."""

    def __init__(self, words=None, size=512, seed=1, kind='stub'):
        self.size = size
        self.chip = Chip(size, seed, kind)
        self.mem = self.chip.mem
        self.written = set()
        for a, w in (words or {}).items():
            self.mem[a] = w
            self.written.add(a)
        self.t = [ThreadV2(t) for t in self.chip.th]
        self.ui_last = 0

    c = property(lambda s: s.chip.c)
    full = property(lambda s: s.chip.FULL)
    mail = property(lambda s: s.chip.M)
    owner = property(lambda s: s.chip.OWN)
    out = property(lambda s: _bits(s.chip.OUT))
    sh_out = property(lambda s: _bits(s.chip.SHD))
    oe = property(lambda s: _bits(s.chip.OE))
    sh_oe = property(lambda s: _bits(s.chip.SHOE))
    od = property(lambda s: _bits(s.chip.OD))
    byp = property(lambda s: _bits(s.chip.BYP))
    sh_byp = property(lambda s: _bits(s.chip.SHBYP))
    busy = property(lambda s: s.chip.BUSY)
    sync2 = property(lambda s: s.chip.SYNC2)

    @property
    def illegal(self):
        return int(any(t.ILL for t in self.chip.th))

    def pads(self):
        """Bypass shows ui of the cycle before, as simulators with this interface do."""
        uo, val, en = self.chip.pads(self.ui_last)
        return uo, en, val

    def cycle(self, env):
        uo, en, val = self.pads()
        ui, lvl = env.step(self, self.chip.c, uo, en, val)
        self.ui_last = ui
        self.chip.cycle(ui & 255, lvl & 255)

    def run(self, cycles, env):
        for _ in range(cycles):
            self.cycle(env)

    def start(self, tid, pc=0):
        self.chip.host_setpc(tid, pc)
        self.chip.host_run(0, 1 << tid)

    def host_run(self, stop, start):
        self.chip.host_run(stop, start)

    def post(self, n, v):
        return self.chip.host_post(n, v) == 0

    def take(self, n):
        full, v = self.chip.host_take(n)
        return v if full else None

    def stopped(self):
        return [t for t in self.t if not t.run]

    def abort(self):
        self.chip.host_abort()

    def xfer(self, mosi):
        """One CS-low transaction at byte level, applied to the flops from Python. It does not run pe_spi or
        pe_host's byte logic: use ChipV2Pads for that."""
        cmd, n = mosi[0], len(mosi)
        miso = [0] * n
        chip = self.chip
        arg = lambda i: mosi[i]
        word = lambda i: arg(i) << 8 | arg(i + 1)

        def reply(start, data):
            for i, x in enumerate(data):
                if start + i < n:
                    miso[start + i] = x
        if n < (3 if 0x20 <= cmd < 0x28 else {0x01: 3, 0x02: 3, 0x10: 3, 0x11: 4, 0x13: 2, 0x14: 3}.get(cmd, 1)):
            return miso                 # cut before its last argument byte: nothing happens
        if cmd == 0x00:
            reply(1, [ISA_VERSION, THREADS << 4 | LOG2.get(self.size, 0)])
        elif cmd == 0x01:
            a = word(1)
            for i in range(3, n - 1, 2):
                if not self.stopped():
                    chip.BUSY = 1
                    break
                self.mem[a % self.size] = word(i)
                self.written.add(a % self.size)
                a += 1
        elif cmd == 0x02:               # the BUSY test runs when addr16 arrives
            if not self.stopped():
                chip.BUSY = 1
            else:
                a = word(1)
                for i in range(5, n, 2):
                    v = self.mem[a % self.size]
                    reply(i, [v >> 8, v & 255])
                    a += 1
        elif cmd == 0x10:
            chip.host_run(arg(1), arg(2))
        elif cmd == 0x11:
            chip.host_setpc(arg(1) & 3, word(2))
        elif cmd == 0x12:
            s = chip.host_status() if n >= 5 else None
            if s is None:
                t = chip.th
                run = sum(x.RUN << x.TID for x in t)
                ill = sum(x.ILL << x.TID for x in t)
                late = sum(x.LATE << x.TID for x in t)
                stale = sum(x.STALE << x.TID for x in t)
                s = [run << 4 | ill, late << 4 | stale, _bits(chip.FULL), chip.BUSY]
            reply(1, s)
        elif cmd == 0x13:
            chip.host_step(arg(1) & 3)
        elif cmd == 0x14:
            th = chip.th[arg(1) & 3]
            v = chip.host_rdreg(arg(1) & 3, arg(2))
            if not th.RUN:
                reply(4, [v >> 8, v & 255])
        elif 0x20 <= cmd < 0x28:
            reply(3, [int(not self.post(cmd & 7, word(1)))])
        elif 0x28 <= cmd < 0x30:
            v = self.take(cmd & 7)
            reply(1, [0, 0, 0] if v is None else [1, v >> 8, v & 255])
        elif cmd == 0x3F:
            self.abort()
        return miso


class ChipV2Pads(ChipV2):
    """ChipV2 with xfer() on the real link: the pins build, with SPI at the pads, so pe_spi and pe_host run
    as they do on silicon and cycles pass during a transaction. ui0 to ui2 belong to the link here, so
    between transactions cycle() holds CS high whatever env asks for."""

    def __init__(self, words=None, size=512, seed=1):
        super().__init__(words, size, seed, 'pins')
        self.ui_last = 1
        self.io_last = 0
        self.ran = 0                    # threads that showed RUN in any cycle of the last transaction

    def cycle(self, env):
        uo, en, val = self.pads()
        ui, self.io_last = env.step(self, self.chip.c, uo, en, val)
        self.ui_last = (ui & 0xF8) | 1
        self.chip.cycle(self.ui_last, self.io_last & 255)

    def _link(self, cs, sck, mosi):
        ui = (self.ui_last & 0xF8) | cs | sck << 1 | mosi << 2
        self.ui_last = ui
        miso = (self.chip.pads(ui)[0] >> 3) & 1
        self.chip.cycle(ui, self.io_last & 255)
        self.ran |= sum(t.RUN << t.TID for t in self.chip.th)
        return miso

    def xfer(self, mosi, half=3):
        """One CS-low transaction, half cycles per SCK phase. half = 3 is the top rate, clk/6. Returns one
        byte per byte sent, read from MISO at each rising SCK, as a host would."""
        self.ran = 0
        for _ in range(half):
            self._link(0, 0, 0)
        got = []
        for byte in mosi:
            v = 0
            for i in range(7, -1, -1):
                b = (byte >> i) & 1
                for _ in range(half):
                    self._link(0, 0, b)
                v = v << 1 | self._link(0, 1, b)
                for _ in range(half - 1):
                    self._link(0, 1, b)
            got.append(v)
        for _ in range(half):
            self._link(0, 0, 0)
        for _ in range(5):              # MISO is back at 0 and the link has seen CS high
            self._link(1, 0, 0)
        return got


K = THREADS
