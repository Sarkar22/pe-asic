# SPDX-License-Identifier: Apache-2.0
"""A transport for pehost.py over sim/sim.py, at the byte level, for tests in CPython.

Each transaction is split into its command and arguments as section 9.2 lists them, and run with the simulator's
host_* methods at the command's decision point h. Before h, and after it, the chip runs for the SCK time of the
bytes, as if SCK ran at clk/cycles_per_bit; pause_us runs it for that many microseconds at 50 MHz. The simulators
serve a whole command in cycle h, as section 10 item 6 allows, so this tests the bytes, not the timing.

The uio pads read their own drive (section 5.2); a pad nobody drives reads 0, as with the demo board's
pull-downs.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.join(HERE, '..', 'sim'))
import sim      # noqa: E402

CLK_MHZ = 50


class SimLink:
    def __init__(self, seed=1, cycles_per_bit=10, gap=50, ui=0):
        self.chip = sim.Chip(depth=512, seed=seed)
        self.cpb = cycles_per_bit
        self.gap = gap                  # cycles of CS high after each transaction
        self.ui = ui                    # levels of ui3..ui7 (the DIP switches)
        self.log = []                   # (out, reply) of every transaction, and ('pause', us)
        self.pads = (0, 0, 0)

    # one cycle of the chip with the link idle (CS high, SCK low) and the uio pads looped back
    def run(self, cycles):
        chip = self.chip
        ui = self.ui & 0xF8 | 1
        for _ in range(cycles):
            uo, ioo, oe = chip.pads(ui)
            self.pads = (uo, ioo, oe)
            chip.cycle(ui, ioo & oe)

    def probe(self):
        uo, ioo, oe = self.pads
        return uo & ~0x08, ioo & oe     # uo3 shows MISO, which reads 0 between transactions

    def pause_us(self, us):
        self.log.append(('pause', us))
        self.run(int(us * CLK_MHZ))

    def xfer(self, out):
        out = bytes(out)
        reply = bytearray(len(out))
        chip = self.chip
        cmd = out[0]
        n = len(out)
        done = [0]                      # bytes clocked so far

        def until(k):                   # clock the bytes before byte k ends
            if k > done[0]:
                self.run(8 * self.cpb * (k - done[0]))
                done[0] = k

        def arg(i):
            return out[i] if i < n else None

        if cmd == 0x00:                                                 # CAPS
            until(n)
            for i, v in ((1, 0x01), (2, 4 << 4 | 9)):
                if i < n:
                    reply[i] = v
        elif cmd == 0x01 and n >= 3:                                    # WRMEM
            a = out[1] << 8 | out[2]
            for k in range(3, n - 1, 2):
                until(k + 2)
                if not chip.host_wrmem(a, out[k] << 8 | out[k + 1]):
                    break                                               # dead until CS rises (9.4)
                a += 1
        elif cmd == 0x02 and n >= 3:                                    # RDMEM
            a = out[1] << 8 | out[2]
            until(3)
            for k in range(5, n - 1, 2):
                w = chip.host_rdmem(a)
                if chip.BUSY and all(t.RUN for t in chip.th):
                    break
                reply[k], reply[k + 1] = w >> 8, w & 255
                a += 1
        elif cmd == 0x10 and n >= 3:                                    # RUN
            until(3)
            chip.host_run(out[1], out[2])
        elif cmd == 0x11 and n >= 4:                                    # SETPC
            until(4)
            chip.host_setpc(out[1], out[2] << 8 | out[3])
        elif cmd == 0x12 and n >= 5:                                    # STATUS
            until(5)
            reply[1:5] = bytes(chip.host_status())
        elif cmd == 0x13 and n >= 2:                                    # STEP
            until(2)
            chip.host_step(out[1])
        elif cmd == 0x14 and n >= 3:                                    # RDREG
            until(3)
            v = chip.host_rdreg(out[1], out[2])
            if n >= 6:
                reply[4], reply[5] = v >> 8, v & 255
        elif 0x20 <= cmd < 0x28 and n >= 3:                             # POST
            until(3)
            r = chip.host_post(cmd - 0x20, out[1] << 8 | out[2])
            if n >= 4:
                reply[3] = r
        elif 0x28 <= cmd < 0x30:                                        # TAKE
            until(1)
            full, data = chip.host_take(cmd - 0x28)
            for i, v in ((1, full), (2, data >> 8), (3, data & 255)):
                if i < n:
                    reply[i] = v
        elif cmd == 0x3F:                                               # ABORT
            until(1)
            chip.host_abort()
        until(n)
        self.run(self.gap)
        reply = bytes(reply)
        self.log.append((out, reply))
        return reply
