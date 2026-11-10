#!/usr/bin/env python3
"""Cycle model of ISA v0.1, written by hand from sections 2-5, 6.3, 8 and 9.

Names follow the document so each function can be read against its code block.
The order inside one cycle is section 3.2. Row semantics are hand-written here.

Each thread's shift unit (section 5.9) is a Unit, run by the Chip methods under
5.9.4 to 5.9.7, one per code block. Step 2b runs every unit, then step 3 runs on
the state it left, so the instruction wins where both write. The rows that read
what step 2b writes read the unit as it was before step 2b instead (3.2).
"""
import random

from asm import ROWS, decode

TICK, WAIT = 0, 1
LOAD, STORE = 0, 1
LANES = (0, 4, 5, 6, 7)
SRC = {0: 3, 4: 4, 5: 5, 6: 6, 7: 7}
HOLD, PCSET = 1, 2                      # run_row outcomes; None retires with PC := next

# 5.9.3: the fields of the 16 configuration slots, as (name, slot, low bit, width)
FIELDS = (
    ('SYMS', 0, 0, 2), ('TMSB', 0, 2, 1), ('TGLA', 0, 3, 1), ('TGLB', 0, 4, 1), ('IDLEA', 0, 5, 1),
    ('IDLEB', 0, 6, 1), ('ENDA', 0, 7, 1), ('ENDB', 0, 8, 1), ('TEXT', 0, 9, 1), ('TFALL', 0, 10, 1),
    ('TCLK', 0, 11, 4),
    ('MAPA', 1, 0, 8), ('MAPB', 1, 8, 8),
    ('TPER', 2, 0, 16),
    ('TAIL', 3, 0, 8), ('TFRAC', 3, 8, 8),
    ('RKIND', 4, 0, 2), ('RMSB', 4, 2, 1), ('RINV', 4, 3, 1), ('RDIFF', 4, 4, 1), ('HUNT', 4, 5, 1),
    ('HLVL', 4, 6, 1), ('RFALL', 4, 7, 1), ('RPIN', 4, 8, 4), ('RCLK', 4, 12, 4),
    ('RPER', 5, 0, 16),
    ('FIRST', 6, 0, 16),
    ('REND', 7, 0, 16),
    ('RCNT', 8, 0, 11), ('SYNCLEN', 8, 11, 5),
    ('SYNC', 9, 0, 16),
    ('RFRAC', 10, 0, 8), ('RESYNC', 10, 8, 1), ('RGAIN', 10, 9, 2), ('RACQ', 10, 11, 1), ('CSKIP', 10, 12, 4),
    ('POLYH', 11, 0, 16),
    ('POLYL', 12, 0, 16),
    ('CHKH', 13, 0, 16),
    ('CHKL', 14, 0, 16),
    ('CW', 15, 0, 5), ('CINIT', 15, 5, 1), ('CXOR', 15, 6, 1), ('CMODE', 15, 7, 2), ('CTX', 15, 9, 1),
    ('SMODE', 15, 10, 2), ('SN', 15, 12, 3), ('REOP', 15, 15, 1),
)
SLOT_FIELDS = [[(n, lo, (1 << w) - 1) for n, s, lo, w in FIELDS if s == k] for k in range(16)]
KEEP = [sum(m << lo for _, lo, m in SLOT_FIELDS[k]) for k in range(16)]     # 5.9.4; TXMODE bit 15 is reserved
OUTPUT_CODE = ((0, 1, 0, 1), (1, 0, 0, 1))      # 5.9.3 output codes: OUTPUT_CODE[b][code] = b, 1 - b, 0, 1

# 5.9.2: the state of one unit. CFG, CRC and RXPE can be seen; the rest is sequencing state.
UNIT_STATE = ('TXARM', 'TXRUN', 'TXTL', 'TXSR', 'TXN', 'TXLW', 'TXBUF', 'TXBN', 'TXBL', 'TXFULL', 'TXSYM', 'TXCD',
              'TXACC', 'LA', 'LB', 'TXUR', 'TXK', 'TXBK', 'TXB', 'TXSTF', 'TXAN', 'TXRL', 'TXRV',
              'RXARM', 'RXHUNT', 'RXRUN', 'RXEP', 'RXSR', 'RXN', 'RXTOT', 'RXSYN', 'RXKIND', 'RXPH', 'RXG',
              'RXOPEN', 'RXCAND', 'RXIC', 'RXLAST', 'RXWIN', 'RXBUF', 'RXBN', 'RXBL', 'RXFULL', 'RXPN', 'RXOVR',
              'RXSEEN', 'RXRL', 'RXRV', 'RXVIO', 'RXCOV', 'RXEF', 'RXBE', 'CMT', 'CRC', 'RXPE')


class Unit:
    """One thread's shift unit. Every name of 5.9.2 is 0 at reset (section 8), and so is every slot."""

    def __init__(self):
        for n in UNIT_STATE:
            setattr(self, n, 0)
        self.CFG = [0] * 16
        for n, _, _, _ in FIELDS:       # each field reads its slot: set_cfg keeps them equal
            setattr(self, n, 0)

    def status(self):
        """5.9.4: the status word that sstat and RDREG 12 read."""
        tb = 1 if self.TXARM or self.TXRUN or self.TXTL else 0
        rw = 1 if self.RXARM or self.RXHUNT or self.RXEP else 0
        f = self.RXPN | (tb << 5) | (self.TXFULL << 6) | (self.TXUR << 7) | (self.RXRUN << 8) | (self.RXFULL << 9)
        return f | (self.RXOVR << 10) | (self.RXSYN << 11) | (self.RXSEEN << 12) | (self.LA << 13) | (self.LB << 14) \
            | (rw << 15)


class Thread:
    def __init__(self, tid):
        self.TID = tid
        self.PC = 0
        self.R = [0] * 8
        self.T = 0
        self.D = 0
        self.S = 0
        self.LATE = self.STALE = self.ILL = self.RUN = 0
        self.P = self.KIND = self.DUE = self.WAITING = 0
        self.MASK = self.PAT = self.BANK = self.NE = self.TIMED = self.INBOX = 0
        self.MEM = self.MEM_DONE = self.MEM_KIND = self.MEM_REG = 0
        self.MEM_ADDR = self.MEM_DATA = 0
        self.STOPREQ = self.STEPPING = 0
        self.IR = self.IV = 0
        self.HIT = self.D_WRITTEN = 0
        self.u = Unit()                 # 5.9: this thread's shift unit
        self.pre = self.u               # what step 3 reads of it (3.2): see Chip.units_and_slot
        self.u0 = Unit()                # room for the copy taken before step 2b
        self.retired = 0                # not architectural: instruction count
        self.late_at = None             # not architectural: first cycle LATE rose


class Chip:
    def __init__(self, depth=128, seed=1):
        self.DEPTH = depth
        rng = random.Random(seed)       # section 10: unwritten memory is arbitrary
        self.mem = [rng.getrandbits(16) for _ in range(depth)]
        self.th = [Thread(i) for i in range(4)]
        self.c = 0
        self.M = [0] * 8
        self.FULL = [0] * 8
        self.SYNC1 = self.SYNC2 = self.SYNC3 = 0
        self.SEL = [0] * 16             # 5.9.2: 0 is OUT, 1 shift output A, 2 shift output B of the pin's owner
        self.SHSEL = [0] * 16
        self.busy = False               # not architectural: some unit may be running (units_and_slot)
        self._sel = False               # not architectural: some pin is on a shift output (pads)
        self.OUT = [0] * 16
        self.SHD = [0] * 16
        self.OE = [0] * 8
        self.SHOE = [0] * 8
        self.OD = [0] * 8
        self.BYP = [0] * 8              # only LANES are used
        self.SHBYP = [0] * 8
        self.OWN = [0] * 16
        self.BUSY = 0
        self.host_miso = 0
        self._dirty = True
        self._pads = (0, 0, 0)
        self.log = None                 # optional list of (cycle, tid, pc, word) at execute

    # ---- 3.4 arming -------------------------------------------------------
    def schedule(self, t, step, a):
        if t.STALE:
            t.D = (a + 1) & 0xFFFF
            t.LATE = 1
        else:
            t.D = (t.D + step) & 0xFFFF
        t.D_WRITTEN = 1
        delta = (t.D - a) % 65536
        t.DUE = 1 if delta == 0 or delta >= 0x8000 else 0

    def arm(self, t, step, a):
        self.schedule(t, step, a)
        t.P = 1
        t.KIND = TICK

    def now(self, t, a):
        t.D = (a + 1) & 0xFFFF
        t.D_WRITTEN = 1
        t.P = 1
        t.KIND = TICK
        t.DUE = 0

    def wait(self, t, mask, pat, bank, ne, rt, inbox, a):
        t.MASK = mask & 255
        t.PAT = pat & 255
        t.BANK = bank
        t.NE = ne
        t.INBOX = inbox
        t.TIMED = rt is not None
        t.DUE = 0
        if t.TIMED:
            self.schedule(t, rt, a)
        t.P = 1
        t.KIND = WAIT
        t.WAITING = 1
        t.PC = self.next(t)

    # ---- 3.4 firing -------------------------------------------------------
    def events(self, t, TB):
        if t.KIND == TICK:
            if t.DUE or t.D == TB:
                if t.DUE:
                    t.LATE = 1
                self.commit(t)
                t.S = self.SYNC2
                t.P = 0
        else:
            byte = (self.SYNC2 >> (8 * t.BANK)) & 255
            match = ((byte ^ t.PAT) & t.MASK) == 0
            if t.NE:
                match = not match
            if match:
                self.commit(t)
                t.S = self.SYNC2
                t.D = TB
                t.D_WRITTEN = 1
                t.T = 1
                t.P = 0
            elif t.INBOX and self.FULL[t.TID]:
                t.S = self.SYNC2
                t.D = TB
                t.D_WRITTEN = 1
                t.T = 0
                t.P = 0
            elif t.TIMED and (t.DUE or t.D == TB):
                t.S = self.SYNC2
                t.T = 0
                t.P = 0

    # ---- 3.5 --------------------------------------------------------------
    @staticmethod
    def update_stale(t):
        if t.D_WRITTEN:
            t.STALE = 0
        elif t.HIT:
            t.STALE = 1
        t.D_WRITTEN = 0

    # ---- 3.6 --------------------------------------------------------------
    def next(self, t):
        return (t.PC + 1) % self.DEPTH

    def slot(self, t, P_start, a):
        if not t.RUN:
            pass
        elif t.MEM and t.MEM_DONE:
            self.finish_mem(t)
        elif t.STOPREQ:
            self.stop(t)
        elif not t.IV:
            pass
        elif t.WAITING:
            if not P_start:
                t.WAITING = 0
                self.retire(t)
                if t.RUN:
                    self.execute(t, t.IR, 0, a)
        else:
            self.execute(t, t.IR, P_start, a)

    def execute(self, t, word, P_start, a):
        d = decode(word)
        if d is None:
            t.ILL = 1
            self.stop(t)
            return
        row, f = d
        w = row.w == 'y' or (row.w == 'pin' and (f['i'] != 0 or f['o'] != 3))
        if w and P_start:
            return
        if self.log is not None:
            self.log.append((self.c, t.TID, t.PC, word))
        out = row.fn(self, t, f, a)
        if out == HOLD:
            return
        if out != PCSET:
            t.PC = self.next(t)
        self.retire(t)

    @staticmethod
    def stop(t):
        t.RUN = 0
        if not t.P:
            t.WAITING = 0
        t.P = 0
        t.STOPREQ = 0
        t.STEPPING = 0

    @staticmethod
    def retire(t):
        t.retired += 1
        if t.STEPPING:
            t.RUN = 0
            t.STEPPING = 0
            if t.STOPREQ:
                t.P = 0
            t.STOPREQ = 0

    # ---- 4.2 --------------------------------------------------------------
    def memory(self, t):
        if t.MEM:
            if not t.MEM_DONE:
                if t.MEM_KIND == LOAD:
                    t.MEM_DATA = self.mem[t.MEM_ADDR]
                else:
                    self.mem[t.MEM_ADDR] = t.MEM_DATA
                t.MEM_DONE = 1
        elif t.RUN:
            t.IR = self.mem[t.PC]
            t.IV = 1
        else:
            t.IV = 0

    def load(self, t, d, A):
        t.MEM = 1
        t.MEM_KIND = LOAD
        t.MEM_REG = d
        t.MEM_ADDR = A % self.DEPTH

    def store(self, t, V, A):
        t.MEM = 1
        t.MEM_KIND = STORE
        t.MEM_DATA = V
        t.MEM_ADDR = A % self.DEPTH

    def finish_mem(self, t):
        if t.MEM_KIND == LOAD:
            t.R[t.MEM_REG] = t.MEM_DATA
        t.MEM = 0
        t.MEM_DONE = 0
        t.PC = self.next(t)
        self.retire(t)

    # ---- 5.3 / 5.4 --------------------------------------------------------
    def shadow(self, t, p, v):
        if p != 3 and self.OWN[p] == t.TID:
            self.SHD[p] = v

    def shadow_oe(self, t, p, v):
        if self.OWN[p] == t.TID and not self.OD[p - 8]:
            self.SHOE[p - 8] = v

    def put8(self, t, b, v):
        for i in range(8):
            self.shadow(t, 8 * b + i, (v >> i) & 1)

    def oe8(self, t, v):
        for i in range(8):
            self.shadow_oe(t, 8 + i, (v >> i) & 1)

    def commit(self, t):
        tid = t.TID
        OWN, OUT, SHD, SEL, SHSEL = self.OWN, self.OUT, self.SHD, self.SEL, self.SHSEL
        for p in range(16):
            if OWN[p] == tid:
                OUT[p] = SHD[p]
                SEL[p] = SHSEL[p]
                if p >= 8:
                    self.OE[p - 8] = self.SHOE[p - 8]
                elif p in LANES:
                    self.BYP[p] = self.SHBYP[p]
        t.u.CMT = 1
        self._dirty = True
        self._sel = any(SEL)

    def claim(self, t, p):
        self.OWN[p] = t.TID
        self.SEL[p] = 0
        self.SHSEL[p] = 0
        self._dirty = True
        self._sel = any(self.SEL)

    def pcfg_in(self, t, p):
        self.claim(t, p)
        self.OD[p - 8] = 0
        self.shadow_oe(t, p, 0)

    def pcfg_pp(self, t, p):
        self.claim(t, p)
        if p >= 8:
            self.OD[p - 8] = 0
            self.shadow_oe(t, p, 1)
        elif p in LANES:
            self.SHBYP[p] = 0

    def pcfg_od(self, t, p):
        self.claim(t, p)
        self.OD[p - 8] = 1
        self.OE[p - 8] = 0
        self.SHOE[p - 8] = 0
        self.OUT[p] = 1
        self.SHD[p] = 1

    def pcfg_byp(self, t, p):
        self.claim(t, p)
        self.SHBYP[p] = 1

    # ---- 5.2 --------------------------------------------------------------
    def V(self, p):
        s = self.SEL[p]
        if s == 1:
            return self.th[self.OWN[p]].u.LA
        if s == 2:
            return self.th[self.OWN[p]].u.LB
        return self.OUT[p]

    def pads(self, ui_in):
        """uo_out, uio_out, uio_oe for the cycle whose ui_in is given.

        The bypass path reads ui_in of this same cycle, so there is no lag.
        """
        if not self._dirty and not any(self.BYP) and not self._sel:     # a shift output moves by itself
            return self._pads
        OD, V = self.OD, self.V
        uo = uio = oe = 0
        for i in range(8):
            if i == 3:
                b = self.host_miso
            elif self.BYP[i]:
                b = (ui_in >> SRC[i]) & 1
            else:
                b = V(i)
            uo |= b << i
            if OD[i]:
                oe |= (1 - V(8 + i)) << i
            else:
                uio |= V(8 + i) << i
                oe |= self.OE[i] << i
        self._pads = (uo, uio, oe)
        self._dirty = False
        return self._pads

    # ---- 5.9.4: the helpers of the eight rows. Reads of the unit go to t.pre, writes to t.u (3.2) ------------
    def tx_push(self, t, v, n, last, kind):
        u = t.u
        u.TXBUF = v
        u.TXBN = 16 if n == 0 else n
        u.TXBL = last
        u.TXBK = kind
        u.TXFULL = 1
        self.busy = True

    @staticmethod
    def rx_pop(t):
        u, s = t.u, t.pre
        u.RXPN = s.RXBN
        u.RXPE = s.RXBE
        u.RXFULL = 0

    @staticmethod
    def srd(t, d, s):
        p = t.pre
        if s == 0:
            t.R[d] = p.CRC & 0xFFFF
        elif s == 1:
            t.R[d] = p.CRC >> 16
        else:
            t.R[d] = p.RXPE

    @staticmethod
    def sstat(t, d):
        u, p = t.u, t.pre
        t.R[d] = p.status()
        t.T = p.TXUR | p.RXOVR
        if p.TXUR:
            u.TXUR = 0
        if p.RXOVR:
            u.RXOVR = 0
        if p.RXSEEN:
            u.RXSEEN = 0

    @staticmethod
    def set_cfg(t, k, v):
        u = t.u
        v &= KEEP[k]
        u.CFG[k] = v
        for n, lo, m in SLOT_FIELDS[k]:
            setattr(u, n, (v >> lo) & m)

    def sgo(self, t, m):
        if m & 1:
            t.u.TXARM = 1
        if m & 2:
            t.u.RXARM = 1
        self.busy = True

    def sstop(self, t, m):
        if m & 1:
            self.tx_stop(t.u)
        if m & 2:
            self.rx_stop(t.u)

    def shadow_sel(self, t, p, l):
        if self.OWN[p] == t.TID:
            self.SHSEL[p] = l

    # ---- 5.9.5 send. Each function takes the unit u ---------------------------------------------------------
    def tx_step(self, u):
        if u.TXRUN:
            self.tx_run(u)
        elif u.TXTL:
            self.tx_tail(u)
        else:
            if u.TXN == 0 and u.TXFULL:
                self.tx_load(u)
            if u.TXARM and u.CMT:
                u.TXARM = 0
                if u.TXN == 0:
                    u.TXUR = 1
                    self.tx_end(u)
                else:
                    u.TXRUN = 1
                    u.TXSYM = 0
                    u.TXACC = 0
                    self.tx_begin(u)
                    self.tx_symbol(u)

    @staticmethod
    def tx_load(u):
        u.TXSR = u.TXBUF
        u.TXN = u.TXBN
        u.TXLW = u.TXBL
        u.TXK = u.TXBK
        u.TXFULL = 0

    @staticmethod
    def tx_symbol(u):
        if u.TXSTF:
            b = 1 - u.TXRV
        elif u.TXAN != 0:
            b = ((u.CRC >> 31) ^ u.CXOR) if u.CTX else u.CXOR
        else:
            b = ((u.TXSR >> (u.TXN - 1)) & 1) if u.TMSB else (u.TXSR & 1)
            if u.CMODE == 2 and u.CTX and u.TXK >= 2:
                b ^= u.CRC >> 31
        u.TXB = b
        code = OUTPUT_CODE[b]
        va = code[(u.MAPA >> (2 * u.TXSYM)) & 3]
        vb = code[(u.MAPB >> (2 * u.TXSYM)) & 3]
        u.LA = (u.LA ^ va) if u.TGLA else va
        u.LB = (u.LB ^ vb) if u.TGLB else vb
        if u.TPER == 0:
            u.TXCD = 1
        else:
            u.TXACC += u.TFRAC
            u.TXCD = u.TPER + (u.TXACC >> 8)
            u.TXACC &= 255

    def tx_run(self, u):
        if u.TEXT:
            now = (self.SYNC2 >> u.TCLK) & 1
            due = now != ((self.SYNC3 >> u.TCLK) & 1) and now != u.TFALL
        else:
            due = u.TXCD <= 1
        if not due:
            if not u.TEXT:
                u.TXCD -= 1
        elif u.TXSYM < u.SYMS:
            u.TXSYM += 1
            self.tx_symbol(u)
        else:
            u.TXSYM = 0
            self.tx_done(u)
            if u.TXSTF or u.TXN != 0 or u.TXAN != 0:
                self.tx_symbol(u)
            elif u.TXLW:
                self.tx_end(u)
            elif u.TXFULL:
                self.tx_load(u)
                self.tx_symbol(u)
            else:
                u.TXUR = 1
                self.tx_end(u)

    @staticmethod
    def tx_end(u):
        u.TXRUN = 0
        if u.TAIL == 0:
            u.LA = u.IDLEA
            u.LB = u.IDLEB
        else:
            u.TXTL = 1
            u.TXCD = u.TAIL
            u.LA = u.ENDA
            u.LB = u.ENDB

    @staticmethod
    def tx_tail(u):
        if u.TXCD <= 1:
            u.TXTL = 0
            u.LA = u.IDLEA
            u.LB = u.IDLEB
        else:
            u.TXCD -= 1

    @staticmethod
    def tx_stop(u):
        u.TXARM = 0
        u.TXRUN = 0
        u.TXTL = 0
        u.TXN = 0
        u.TXFULL = 0
        u.TXSYM = 0
        u.TXSTF = 0
        u.TXAN = 0
        u.LA = u.IDLEA
        u.LB = u.IDLEB

    # ---- 5.9.6 receive ----------------------------------------------------------------------------------------
    def rx_step(self, u):
        if u.RXEP:
            if not u.RXFULL:
                self.rx_deliver(u, u.RXSR, u.RXN, 1)
                u.RXEP = 0
                if u.HUNT:
                    u.RXHUNT = 1
        elif u.RXRUN:
            self.rx_run(u)
        elif u.RXHUNT:
            if ((self.SYNC2 >> u.RPIN) & 1) == u.HLVL and ((self.SYNC3 >> u.RPIN) & 1) != u.HLVL:
                self.rx_begin(u)
        elif u.RXARM and u.CMT:
            u.RXARM = 0
            if u.HUNT:
                u.RXHUNT = 1
            else:
                self.rx_begin(u)

    def rx_begin(self, u):
        u.RXHUNT = 0
        u.RXRUN = 1
        u.RXSEEN = 1
        u.RXSR = 0
        u.RXN = 0
        u.RXTOT = 0
        u.RXSYN = 1 if u.SYNCLEN == 0 else 0
        u.RXIC = 0
        u.RXWIN = 0
        u.RXLAST = (self.SYNC3 >> u.RPIN) & 1
        u.RXKIND = u.RKIND
        u.RXOPEN = 0
        u.RXCAND = 0
        u.RXG = 1
        self.rx_cbegin(u)
        if u.RXKIND == 1:
            u.RXPH = 256
            if not u.RDIFF:
                self.rx_bit(u, (self.SYNC2 >> u.RPIN) & 1)
        else:
            u.RXPH = (u.FIRST - 1 if u.FIRST else 0) << 8

    def rx_run(self, u):
        e = ((self.SYNC2 ^ self.SYNC3) >> u.RPIN) & 1
        if u.RXIC < 65535:
            u.RXIC += 1
        p = 256 if u.RPER == 0 else (u.RPER << 8) | u.RFRAC
        if u.RXKIND == 1:
            if u.RPER > 255:
                p = 0xFF00 | u.RFRAC
            self.rx_manch(u, e, p)
        elif u.RXKIND == 2:
            now = (self.SYNC2 >> u.RCLK) & 1
            if now != ((self.SYNC3 >> u.RCLK) & 1) and now != u.RFALL:
                u.RXIC = 0
                self.rx_sample(u)
        else:
            self.rx_timed(u, e, p)
        if u.RXRUN and u.REND != 0 and u.RXIC >= u.REND:
            self.rx_end(u)

    def rx_timed(self, u, e, p):
        if u.RXPH < 256:
            u.RXPH = u.RXPH - 256 + p
            if u.REOP and ((self.SYNC2 >> u.RPIN) & 1) == 0 and ((self.SYNC2 >> u.RCLK) & 1) == 0:
                self.rx_end(u)
            else:
                self.rx_sample(u)
        else:
            u.RXPH -= 256
        if e:
            u.RXIC = 0
            if u.RESYNC:
                self.rx_pull(u, (u.FIRST - 1 if u.FIRST else 0) << 8)
                self.rx_gear(u)

    def rx_manch(self, u, e, p):
        w = (p >> 1) if u.RACQ and u.RGAIN != 0 and u.RXG < 16 else (p >> 2)
        if u.RXPH >= p - w:
            if u.RXOPEN:
                self.rx_end(u)
            u.RXOPEN = 1
            u.RXCAND = 0
            u.RXPH -= p
        if u.RXRUN and u.RXOPEN:
            if e and u.RXPH < 0:
                if u.RXCAND:
                    u.RXWIN = 1
                u.RXCAND = 1
                self.rx_pull(u, 0)
            elif e or (u.RXCAND and u.RXPH >= 0):
                u.RXOPEN = 0
                u.RXCAND = 0
                u.RXIC = 0
                self.rx_bit(u, u.RXWIN if u.RDIFF else (self.SYNC2 >> u.RPIN) & 1)
                u.RXWIN = 0
                if e:
                    self.rx_pull(u, 0)
                self.rx_gear(u)
        elif u.RXRUN and e:
            u.RXWIN = 1
            self.rx_pull(u, p >> 1)
        u.RXPH += 256

    @staticmethod
    def rx_pull(u, r):
        d = u.RXPH - r
        if u.RGAIN == 0:
            u.RXPH = r
        elif u.RXG <= 3:
            u.RXPH -= d >> 1
        elif u.RGAIN == 1:
            u.RXPH -= d >> 2
        elif u.RGAIN == 2:
            u.RXPH -= d >> 4
        else:
            u.RXPH -= d >> 5

    @staticmethod
    def rx_gear(u):
        if u.RXG < 16:
            u.RXG += 1

    def rx_sample(self, u):
        v = (self.SYNC2 >> u.RPIN) & 1
        d = 1 if v == u.RXLAST else 0
        u.RXLAST = v
        self.rx_bit(u, d if u.RDIFF else v)

    def rx_bit(self, u, b):
        b ^= u.RINV
        stuffing = u.SMODE == 1 or u.SMODE == 2
        if u.RXVIO:
            if b == u.RXRV:
                u.RXEF |= 4
            self.rx_end(u)
        elif u.RXSYN and stuffing and u.RXRL != 0 and u.RXRL >= u.SN and (u.SMODE == 1 or u.RXRL >= 2):
            if b == u.RXRV:
                u.RXVIO = 1
                u.RXEF |= 2
            else:
                self.rx_run_len(u, b)
        elif not u.RXSYN:
            if stuffing:
                self.rx_run_len(u, b)
            if u.RMSB:
                u.RXSR = ((u.RXSR << 1) | b) & 0xFFFF
                ok = ((u.RXSR ^ u.SYNC) & ((1 << u.SYNCLEN) - 1)) == 0
            else:
                u.RXSR = (u.RXSR >> 1) | (b << 15)
                ok = ((u.RXSR ^ u.SYNC) & (0xFFFF ^ (0xFFFF >> u.SYNCLEN))) == 0
            if ok:
                u.RXSYN = 1
                u.RXSR = 0
        else:
            if stuffing:
                self.rx_run_len(u, b)
            crc_on = u.CMODE == 1 or u.CMODE == 2
            if u.RXTOT == u.CSKIP and crc_on:
                u.RXCOV = 1
            if u.RXCOV and not u.CTX and crc_on:
                f = u.CRC >> 31
                self.crc_bit(u, b)
                if u.CMODE == 2:
                    b ^= f
            if u.RMSB:
                u.RXSR = ((u.RXSR << 1) | b) & 0xFFFF
            else:
                u.RXSR |= b << u.RXN
            u.RXN += 1
            u.RXTOT = (u.RXTOT + 1) & 2047
            if u.RCNT != 0 and u.RXTOT == u.RCNT:
                self.rx_end(u)
            elif u.RXN == 16:
                self.rx_deliver(u, u.RXSR, 16, 0)
                u.RXSR = 0
                u.RXN = 0

    @staticmethod
    def rx_deliver(u, v, n, last):
        if u.RXFULL:
            u.RXOVR = 1
        else:
            u.RXBUF = v
            u.RXBN = n
            u.RXBL = last
            u.RXBE = u.RXEF if last else 0
            u.RXFULL = 1

    @staticmethod
    def rx_end(u):
        u.RXRUN = 0
        if u.CMODE == 1 and not u.CTX and u.CRC == ((u.CHKH << 16) | u.CHKL):
            u.RXEF |= 1
        if u.RXSYN:
            u.RXEP = 1
        elif u.HUNT:
            u.RXHUNT = 1

    @staticmethod
    def rx_stop(u):
        u.RXARM = 0
        u.RXHUNT = 0
        u.RXRUN = 0
        u.RXEP = 0
        u.RXFULL = 0
        u.RXVIO = 0

    # ---- 5.9.7 CRC and stuffing -------------------------------------------------------------------------------
    @staticmethod
    def crc_init(u):
        if u.CINIT:
            u.CRC = 0xFFFFFFFF ^ (0xFFFFFFFF >> (u.CW + 1))
        else:
            u.CRC = 0

    @staticmethod
    def crc_bit(u, b):
        f = ((u.CRC >> 31) ^ b) if u.CMODE == 1 else (u.CRC >> 31)
        u.CRC = (u.CRC << 1) & 0xFFFFFFFF
        if f:
            u.CRC ^= (u.POLYH << 16) | u.POLYL

    def tx_begin(self, u):
        u.TXSTF = 0
        u.TXAN = 0
        u.TXRL = 0
        if u.CTX and (u.CMODE == 1 or u.CMODE == 2):
            self.crc_init(u)

    def tx_done(self, u):
        if u.TXSTF:
            u.TXSTF = 0
            self.tx_run_len(u, u.TXB)
        elif u.TXAN != 0:
            if u.CTX:
                u.CRC = (u.CRC << 1) & 0xFFFFFFFF
            u.TXAN -= 1
            if u.TXAN == 0 and u.CTX and (u.CMODE == 1 or u.CMODE == 2):
                self.crc_init(u)
            self.tx_stuff(u, u.TXB)
        else:
            if u.TXK >= 2 and u.CTX and (u.CMODE == 1 or u.CMODE == 2):
                self.crc_bit(u, u.TXB)
            if u.TXK >= 1:
                self.tx_stuff(u, u.TXB)
            else:
                u.TXRL = 0
            u.TXN -= 1
            if not u.TMSB:
                u.TXSR >>= 1
            if u.TXN == 0 and u.TXK == 3 and u.CTX and u.CMODE == 1:
                u.TXAN = u.CW + 1

    def tx_stuff(self, u, v):
        if u.SMODE == 1 or u.SMODE == 2:
            self.tx_run_len(u, v)
            if u.TXRL != 0 and u.TXRL >= u.SN and (u.SMODE == 1 or u.TXRL >= 2):
                u.TXSTF = 1

    @staticmethod
    def tx_run_len(u, v):
        if u.SMODE == 1 and v == 0:
            u.TXRL = 0
        elif u.TXRL == 0 or v != u.TXRV:
            u.TXRL = 1
        elif u.TXRL < 7:
            u.TXRL += 1
        u.TXRV = v

    def rx_cbegin(self, u):
        u.RXEF = 0
        u.RXVIO = 0
        u.RXCOV = 0
        u.RXRL = 0
        if not u.CTX and (u.CMODE == 1 or u.CMODE == 2):
            self.crc_init(u)

    @staticmethod
    def rx_run_len(u, v):
        if u.SMODE == 1 and v == 0:
            u.RXRL = 0
        elif u.RXRL == 0 or v != u.RXRV:
            u.RXRL = 1
        elif u.RXRL < 7:
            u.RXRL += 1
        u.RXRV = v

    # ---- 3.2 --------------------------------------------------------------
    def cycle(self, ui_in, uio_in):
        """Steps 2 to 5 of one cycle. Step 1 is pads() plus the board."""
        c = self.c
        TB = c & 0xFFFF
        th = self.th
        P_start = [t.P for t in th]
        for t in th:
            t.HIT = TB == (t.D ^ 0x8000)
        for t in th:
            if t.P:
                self.events(t, TB)
        i = c & 3
        self.units_and_slot(th[i], P_start[i], TB)
        self.memory(th[(i - 1) & 3])
        for t in th:
            self.update_stale(t)
            t.u.CMT = 0
            if t.LATE and t.late_at is None:
                t.late_at = c
        self.SYNC3 = self.SYNC2
        self.SYNC2 = self.SYNC1
        self.SYNC1 = (uio_in << 8) | ui_in
        self.c = c + 1

    def units_and_slot(self, t, P_start, a):
        """Step 2b, every unit's tx_step then rx_step, and step 3, the slot of thread t.

        Step 3 reads its unit as it was at the start of the cycle (3.2). A row that reads the unit is in group 1101,
        so for such a word t.pre is a copy taken before step 2b. Its writes go to the unit after step 2b's, so the
        instruction wins where both write.

        A half whose state the code would leave alone is skipped. An idle unit, both halves idle, leaves step 2b
        idle, and only tx_push and sgo start one, so they set busy, and step 2b clears it once every unit is idle
        again. A caller that writes a unit's state by hand sets busy too.
        """
        if t.IR >> 12 == 13:
            t.u0.__dict__.update(t.u.__dict__)
            t.pre = t.u0
        if self.busy:
            busy = False
            for x in self.th:
                u = x.u
                if u.TXRUN or u.TXTL or u.TXFULL or u.TXARM:
                    self.tx_step(u)
                    busy = True
                if u.RXEP or u.RXRUN or u.RXHUNT or u.RXARM:
                    self.rx_step(u)
                    busy = True
            self.busy = busy
        self.slot(t, P_start, a)
        t.pre = t.u

    # ---- 9: byte-level host commands. Call between cycles: h = c - 1 --------
    # thread8: only its low 2 bits count (9.2)
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
        t = self.th[tid & 3]
        if t.RUN:
            self.BUSY = 1
            return
        t.PC = pc % self.DEPTH

    def host_run(self, stop, start):
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
                t.D_WRITTEN = 1
                t.P = 0
                t.WAITING = 0
                t.STOPREQ = 0
                t.LATE = 0
                t.ILL = 0
                t.STALE = 0

    def host_step(self, tid):
        t = self.th[tid & 3]
        if t.RUN:
            self.BUSY = 1
            return
        t.RUN = 1
        t.STEPPING = 1
        t.WAITING = 0

    def host_rdreg(self, tid, reg):
        t = self.th[tid & 3]
        if t.RUN:
            self.BUSY = 1
            return 0
        return self.rdreg(t, reg)

    @staticmethod
    def rdreg(t, reg):
        """What RDREG reg8 reads of thread t now, between two cycles (9.2). Nothing is cleared.

        This model serves RDREG in cycle h, as section 10, item 6 allows, so 12 to 14, which a running unit moves,
        show the flops after cycle h rather than in a cycle from h + 2 to h + 5.
        """
        if reg < 8:
            return t.R[reg]
        u = t.u
        if reg == 12:
            return u.status()
        if reg == 13:
            return u.CRC & 0xFFFF
        if reg == 14:
            return u.CRC >> 16
        if reg == 15:
            return u.RXPE
        if 16 <= reg < 32:
            return u.CFG[reg - 16]
        return {8: t.D, 9: t.S, 10: t.PC,
                11: t.T | (t.LATE << 1) | (t.STALE << 2) | (t.WAITING << 3)}.get(reg, 0)

    def host_post(self, n, data):
        """Result bit 0 is 1 if refused."""
        if self.FULL[n]:
            return 1
        self.M[n] = data & 0xFFFF
        self.FULL[n] = 1
        return 0

    def host_take(self, n):
        if self.FULL[n]:
            self.FULL[n] = 0
            return 1, self.M[n]
        return 0, 0

    def host_status(self):
        def nib(name):
            return sum(getattr(t, name) << t.TID for t in self.th)
        full = sum(f << i for i, f in enumerate(self.FULL))
        out = [(nib('RUN') << 4) | nib('ILL'), (nib('LATE') << 4) | nib('STALE'),
               full, self.BUSY]
        self.BUSY = 0
        return out

    def host_abort(self):
        for t in self.th:                   # abort_thread()
            t.P = 0
            t.WAITING = 0
            if t.RUN:
                t.STOPREQ = 1
            self.tx_stop(t.u)
            self.rx_stop(t.u)
        for i in range(8):                  # abort_pins()
            self.OE[i] = 0
            self.SHOE[i] = 0
            self.SEL[8 + i] = 0
            self.SHSEL[8 + i] = 0
            if self.OD[i]:
                self.OUT[8 + i] = 1
                self.SHD[8 + i] = 1
        self._dirty = True
        self._sel = any(self.SEL)


# ---- 6.3: one handler per row. fn(chip, thread, fields, a) ------------------
H = {}


def op(*names):
    def deco(fn):
        for n in names:
            H[n] = fn
        return fn
    return deco


op('idle')(lambda c, t, f, a: None)


@op('halt')
def _halt(c, t, f, a):
    t.PC = c.next(t)
    c.stop(t)
    return PCSET


op('now')(lambda c, t, f, a: c.now(t, a))


@op('clrt')
def _clrt(c, t, f, a):
    t.T = 0


@op('sett')
def _sett(c, t, f, a):
    t.T = 1


@op('nott')
def _nott(c, t, f, a):
    t.T = 1 - t.T


@op('tlate')
def _tlate(c, t, f, a):
    t.T = t.LATE
    t.LATE = 0


@op('jr Rn')
def _jr(c, t, f, a):
    t.PC = t.R[f['n']] % c.DEPTH
    return PCSET


@op('swap Rn')
def _swap(c, t, f, a):
    v = t.R[f['n']]
    t.R[f['n']] = ((v >> 8) | (v << 8)) & 0xFFFF


@op('not Rn')
def _not(c, t, f, a):
    t.R[f['n']] = ~t.R[f['n']] & 0xFFFF


@op('rds Rn')
def _rds(c, t, f, a):
    t.R[f['n']] = t.S


@op('rdd Rn')
def _rdd(c, t, f, a):
    t.R[f['n']] = t.D


@op('wrd Rn')
def _wrd(c, t, f, a):
    t.D = t.R[f['n']]
    t.D_WRITTEN = 1


@op('rdtb Rn')
def _rdtb(c, t, f, a):
    t.R[f['n']] = a


op('put8 Rn, b')(lambda c, t, f, a: c.put8(t, f['b'], t.R[f['n']]))
op('oe8 Rn')(lambda c, t, f, a: c.oe8(t, t.R[f['n']]))


@op('tid Rn')
def _tid(c, t, f, a):
    t.R[f['n']] = t.TID


def _setpc(c, t, v):
    t.PC = v % c.DEPTH
    return PCSET


op('goto a')(lambda c, t, f, a: _setpc(c, t, f['a']))
op('jt a')(lambda c, t, f, a: _setpc(c, t, f['a']) if t.T else None)
op('jf a')(lambda c, t, f, a: _setpc(c, t, f['a']) if not t.T else None)


@op('call a')
def _call(c, t, f, a):
    t.R[7] = (t.PC + 1) % c.DEPTH
    return _setpc(c, t, f['a'])


@op('djnz Rn, a')
def _djnz(c, t, f, a):
    n = f['n']
    t.R[n] = (t.R[n] - 1) & 0xFFFF
    if t.R[n] != 0:
        return _setpc(c, t, f['a'])


def _alu(fn):
    def h(c, t, f, a):
        t.R[f['n']] = fn(t.R[f['n']], f['i']) & 0xFFFF
    return h


op('ldi Rn, i')(_alu(lambda x, i: i))
op('lui Rn, i')(_alu(lambda x, i: (x & 0xFF) | (i << 8)))
op('addi Rn, i')(_alu(lambda x, i: x + i))
op('subi Rn, i')(_alu(lambda x, i: x - i))
op('andi Rn, i')(_alu(lambda x, i: x & i))
op('ori Rn, i')(_alu(lambda x, i: x | i))


@op('cmpi Rn, i')
def _cmpi(c, t, f, a):
    t.T = 1 if t.R[f['n']] == f['i'] else 0


@op('tsti Rn, i')
def _tsti(c, t, f, a):
    t.T = 1 if t.R[f['n']] & f['i'] else 0


def _alr(fn):
    def h(c, t, f, a):
        t.R[f['d']] = fn(t.R[f['d']], t.R[f['s']]) & 0xFFFF
    return h


op('cp Rd, Rs')(_alr(lambda d, s: s))
op('add Rd, Rs')(_alr(lambda d, s: d + s))
op('sub Rd, Rs')(_alr(lambda d, s: d - s))
op('and Rd, Rs')(_alr(lambda d, s: d & s))
op('or Rd, Rs')(_alr(lambda d, s: d | s))
op('xor Rd, Rs')(_alr(lambda d, s: d ^ s))


@op('xort Rd, Rs')
def _xort(c, t, f, a):
    if t.T:
        t.R[f['d']] ^= t.R[f['s']]


@op('crcr Rd, Rs')
def _crcr(c, t, f, a):
    d = f['d']
    fb = t.T ^ (t.R[d] & 1)
    t.R[d] >>= 1                        # statements run in order: Rd = Rs sees the shift
    if fb:
        t.R[d] ^= t.R[f['s']]


@op('crcl Rd, Rs')
def _crcl(c, t, f, a):
    d = f['d']
    fb = t.T ^ ((t.R[d] >> 15) & 1)
    t.R[d] = (t.R[d] << 1) & 0xFFFF
    if fb:
        t.R[d] ^= t.R[f['s']]


@op('cmp Rd, Rs')
def _cmp(c, t, f, a):
    t.T = 1 if t.R[f['d']] == t.R[f['s']] else 0


@op('lt Rd, Rs')
def _lt(c, t, f, a):
    t.T = 1 if t.R[f['d']] < t.R[f['s']] else 0


@op('bt Rn, b')
def _bt(c, t, f, a):
    t.T = (t.R[f['n']] >> f['b']) & 1


@op('bn Rn, b')
def _bn(c, t, f, a):
    t.T = 1 - ((t.R[f['n']] >> f['b']) & 1)


@op('bx Rn, b')
def _bx(c, t, f, a):
    t.T ^= (t.R[f['n']] >> f['b']) & 1


@op('ba Rn, b')
def _ba(c, t, f, a):
    t.T &= (t.R[f['n']] >> f['b']) & 1


@op('bo Rn, b')
def _bo(c, t, f, a):
    t.T |= (t.R[f['n']] >> f['b']) & 1


@op('bs Rn, b')
def _bs(c, t, f, a):
    n, b = f['n'], f['b']
    t.R[n] = (t.R[n] & ~(1 << b) & 0xFFFF) | (t.T << b)


op('put p, 0')(lambda c, t, f, a: c.shadow(t, f['p'], 0))
op('put p, 1')(lambda c, t, f, a: c.shadow(t, f['p'], 1))
op('put p, T')(lambda c, t, f, a: c.shadow(t, f['p'], t.T))
op('put p, !T')(lambda c, t, f, a: c.shadow(t, f['p'], 1 - t.T))
op('put p, ^T')(lambda c, t, f, a: c.shadow(t, f['p'], c.SHD[f['p']] ^ t.T))
op('put p, ^!T')(lambda c, t, f, a: c.shadow(t, f['p'], c.SHD[f['p']] ^ t.T ^ 1))
op('oe p, v')(lambda c, t, f, a: c.shadow_oe(t, f['p'], f['v']))
op('pcfg p, IN')(lambda c, t, f, a: c.pcfg_in(t, f['p']))
op('pcfg p, PP')(lambda c, t, f, a: c.pcfg_pp(t, f['p']))
op('pcfg p, OD')(lambda c, t, f, a: c.pcfg_od(t, f['p']))
op('pcfg p, BYP')(lambda c, t, f, a: c.pcfg_byp(t, f['p']))


@op('get p')
def _get(c, t, f, a):
    t.T = (t.S >> f['p']) & 1


@op('getn p')
def _getn(c, t, f, a):
    t.T = 1 - ((t.S >> f['p']) & 1)


@op('xget p')
def _xget(c, t, f, a):
    t.T ^= (t.S >> f['p']) & 1


@op('aget p')
def _aget(c, t, f, a):
    t.T &= (t.S >> f['p']) & 1


@op('recv Rn, m')
def _recv(c, t, f, a):
    m = f['m']
    if not c.FULL[m]:
        return HOLD
    t.R[f['n']] = c.M[m]
    c.FULL[m] = 0


@op('send Rn, m')
def _send(c, t, f, a):
    m = f['m']
    if c.FULL[m]:
        return HOLD
    c.M[m] = t.R[f['n']]
    c.FULL[m] = 1


@op('tryrecv Rn, m')
def _tryrecv(c, t, f, a):
    m = f['m']
    t.T = c.FULL[m]
    if t.T:
        t.R[f['n']] = c.M[m]
        c.FULL[m] = 0


@op('trysend Rn, m')
def _trysend(c, t, f, a):
    m = f['m']
    t.T = 1 - c.FULL[m]
    if t.T:
        c.M[m] = t.R[f['n']]
        c.FULL[m] = 1


@op('ld Rd, Ra')
def _ld(c, t, f, a):
    c.load(t, f['d'], t.R[f['a']])
    return HOLD


@op('ldp Rd, Ra')
def _ldp(c, t, f, a):
    c.load(t, f['d'], t.R[f['a']])
    t.R[f['a']] = (t.R[f['a']] + 1) & 0xFFFF
    return HOLD


@op('st Rd, Ra')
def _st(c, t, f, a):
    c.store(t, t.R[f['d']], t.R[f['a']])
    return HOLD


@op('stp Rd, Ra')
def _stp(c, t, f, a):
    c.store(t, t.R[f['d']], t.R[f['a']])
    t.R[f['a']] = (t.R[f['a']] + 1) & 0xFFFF
    return HOLD


@op('until p, l [, inbox]')
def _until(c, t, f, a):
    p = f['p']
    c.wait(t, 1 << (p & 7), f['l'] << (p & 7), p >> 3, 0, None, f['m'], a)
    return HOLD


@op('until p, l, Rt [, inbox]')
def _until_t(c, t, f, a):
    p = f['p']
    c.wait(t, 1 << (p & 7), f['l'] << (p & 7), p >> 3, 0, t.R[f['t']], f['m'], a)
    return HOLD


def _untilm(c, t, f, a, rt):
    r, b, m = t.R[f['r']], f['b'], f['c']
    pat = (t.S >> (8 * b)) & 255 if m == 2 else r & 255
    c.wait(t, r >> 8, pat, b, m != 0, rt, f['m'], a)
    return HOLD


op('untilm Rr, b, c [, inbox]')(lambda c, t, f, a: _untilm(c, t, f, a, None))
op('untilm Rr, b, c, Rt [, inbox]')(lambda c, t, f, a: _untilm(c, t, f, a, t.R[f['t']]))
op('tick i')(lambda c, t, f, a: c.arm(t, f['i'], a))


def _eact(t, v):
    return (0, 1, t.T, 1 - t.T)[v]


@op('edge p, v, i')
def _edge_i(c, t, f, a):
    c.shadow(t, f['p'], _eact(t, f['v']))
    c.arm(t, f['i'], a)


@op('edge p, v, Rr')
def _edge_r(c, t, f, a):
    c.shadow(t, f['p'], _eact(t, f['v']))
    c.arm(t, t.R[f['r']], a)


@op('shl8 Rr, i, o')
def _shl8(c, t, f, a):
    if f['i'] != 0:
        t.T = (t.S >> f['i']) & 1
    r = f['r']
    v = t.R[r]
    out = (v >> 7) & 1
    t.R[r] = (v & 0xFF00) | ((v << 1) & 0xFE) | t.T
    t.T = out
    c.shadow(t, f['o'], t.T)


@op('shl16 Rr, i, o')
def _shl16(c, t, f, a):
    if f['i'] != 0:
        t.T = (t.S >> f['i']) & 1
    r = f['r']
    v = t.R[r]
    out = (v >> 15) & 1
    t.R[r] = ((v << 1) | t.T) & 0xFFFF
    t.T = out
    c.shadow(t, f['o'], t.T)


@op('shr8 Rr, i, o')
def _shr8(c, t, f, a):
    if f['i'] != 0:
        t.T = (t.S >> f['i']) & 1
    r = f['r']
    v = t.R[r]
    out = v & 1
    t.R[r] = (v & 0xFF00) | ((v & 0xFF) >> 1) | (t.T << 7)
    t.T = out
    c.shadow(t, f['o'], t.T)


@op('shr16 Rr, i, o')
def _shr16(c, t, f, a):
    if f['i'] != 0:
        t.T = (t.S >> f['i']) & 1
    r = f['r']
    v = t.R[r]
    out = v & 1
    t.R[r] = (v >> 1) | (t.T << 15)
    t.T = out
    c.shadow(t, f['o'], t.T)


# The shift unit's rows (5.9.4). A need() reads the unit as before step 2b, through t.pre (3.2).
@op('txw Rs, n [, k] [, last]')
def _txw(c, t, f, a):
    if t.pre.TXFULL:
        return HOLD
    c.tx_push(t, t.R[f['s']], f['n'], f['l'], f['k'])


op('scfg Rs, k')(lambda c, t, f, a: c.set_cfg(t, f['k'], t.R[f['s']]))
op('smap p, l')(lambda c, t, f, a: c.shadow_sel(t, f['p'], f['l']))


@op('rxw Rd')
def _rxw(c, t, f, a):
    s = t.pre
    if not s.RXFULL:
        return HOLD
    t.R[f['d']] = s.RXBUF
    t.T = s.RXBL
    c.rx_pop(t)


op('sstat Rd')(lambda c, t, f, a: c.sstat(t, f['d']))
op('srd Rd, s')(lambda c, t, f, a: c.srd(t, f['d'], f['s']))
op('sgo m')(lambda c, t, f, a: c.sgo(t, f['m']))
op('sstop m')(lambda c, t, f, a: c.sstop(t, f['m']))


for _r in ROWS:
    _r.fn = H[_r.syntax]
if len(H) != len(ROWS):
    raise ValueError('handlers without a row: %s' % (set(H) - {r.syntax for r in ROWS}))
