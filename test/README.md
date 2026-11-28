# Tests at the pins

`test.py` drives the design through its pads with [cocotb](https://docs.cocotb.org/en/stable/) under Icarus Verilog. The host side speaks the SPI link of `docs/ISA.md` section 9 inside the rules of its section 9.1: at clk/6 with SCK low and high for 3 cycles each in most tests, and at every limit of 9.1 in `test_matches_simulator` and `test_host_bringup_at_the_limits`. `tb.v` holds the top module and the wires cocotb drives.

No input moves at a clock edge. Both edges sample the input pads (section 3.1: the falling-edge sample feeds RKIND 3, section 5.9.6), and THALF moves a shift output at the falling edge (5.9.5), so `tb.v` gives the tests a time reference, `tick`, the clock delayed by three quarters of a period: it falls 5 ns after each rising edge, inside the first half of the cycle, and rises 5 ns after each falling edge, inside the second. Every test drives its inputs and reads the outputs at tick's falling edge; an input written there holds for the whole cycle, so both samples of that cycle see it, one level per cycle (3.1), and an output read there is the cycle's first half. The `test_eth_*` tests and `test_unit_rdiff` also drive at tick's rising edge, the second half, and the `test_eth_*` tests read there too; the unit tests' board loops the `uio` pads back in both halves.

What the tests cover: reset state, CAPS, MISO timing, memory read and write with wrap at DEPTH, a UART frame sent by a program and decoded at the pad, RUN, STOP, STEP, SETPC and RDREG, the mailbox, BUSY, ABORT, commands cut at any byte, a STATUS cut short, a memory command that fails while all threads run, memory commands with one thread stopped, every unlisted command code, bytes sent past a reply, a reset in mid-run, what STEP and SETPC leave alone, an event firing on a stopped thread, WAITING through a second stop, deadlines 32,767 and 32,768 ahead, 16-bit compares, and the bypass lanes.

`test_bypass_override` loads `../programs/advanced/uart_bypass.s` and drives a UART line on `ui4`: with no rule, `uo4` shows `ui4` on every cycle; with a rule, the chosen frame's data bits leave as the host's byte on that frame's own bit boundaries, to the cycle, and the lane switches exactly where the program's header says, 3 cycles into the start bit and 9 into the stop bit. It runs at 16 cycles per bit with a frame rule and at 50 with a byte rule, on two threads, at RTL and at gate level.

`test_eth_arp` loads `../programs/advanced/eth_arp.s` on threads 0 and 1, posts its MAC and IP, and plays the other end of a 10BASE-T link at the pins, to the half cycle: the program's units receive with RKIND 3, which times every edge to the half cycle (5.9.6), and send with THALF, which puts every half bit at exactly 2.5 cycles (5.9.5). An ARP request, an ICMP echo request and an ARP request with an FCS error, built from IEEE 802.3 and RFCs 826 and 792 by `../programs/bench/eth_check_test.py`, with every transition moved by a random amount up to 802.3's +/-13.5 ns of zero-crossing jitter (14.3.1.3.1), reach `ui4` through the RS-485 transceiver model of `../programs/bench/frontend.py`, at both clock edges. The chip's `uo4` (DI) and `uo5` (DE), recorded in both halves of every cycle and turned back into the transmit pair by the same model, must pass every check of `../programs/bench/eth_check.py`: a link pulse at start, the ARP and echo replies field by field, Manchester code, bit rate, preamble, FCS, TP_IDL and the gap before each frame, and no reply to the bad frame. Every transmitted edge must lie within the transceiver's own skew of the grid of exact 50 ns half bits (half its tSKEW, plus 0.05 ns), so the chip adds no edge jitter; it measures 0.50 ns, the typical corner's tSKEW/2. The program's counters, read with RDMEM, must show two frames kept and one dropped. About 74,000 cycles, 34,000 of them the link.

Three more tests run the same exchange under other conditions, each under 100,000 cycles. `test_eth_fixed_jitter` sends the three frames with section 7.3's fixed pattern at 802.3's limit: every bit-boundary transition 13.5 ns late and every mid-bit one 13.5 ns early (typical corner, sampling phase 14.2 ns). `test_eth_short_pulses` sends them with its mirror, bit-boundary transitions early and mid-bit ones late, at the transceiver model's slowest corner, whose 4 ns skew turns the pair's 23 ns pulses into 19 ns pulses of RO, at a sampling phase of 4.9 ns, where the test first checks on RO itself that every such pulse falls between two rising-edge samples: only the falling-edge sample sees them. `test_eth_dribble` sends frames that end 1 to 7 bits after their FCS, which an 802.3 MAC truncates to whole octets before it checks the FCS (4.2.9; 4.2.4.2.1 b), and which the program receives with the Manchester byte end (REOP with RKIND 3, 5.9.7): an ARP request with 1 excess bit, an echo request with 7 and an ARP request from another address with 5 must be answered and the program must count them kept, and an ARP request with an FCS error and 4 excess bits dropped, with random jitter up to +/-13.5 ns on every transition, at the fastest corner, phase 9.6 ns.

Five more run it at the receiver's limits (5.9.6), each under 100,000 cycles. `test_eth_cable_isi` sends the three
frames with a cable's data-dependent jitter at 802.3's limit, every transition after a whole bit with none 13.5 ns
late and every other one 13.5 ns early (typical corner, phase 3.1 ns); there `eth_check.py` reads the requests from
the line as built, before its jitter, since its own decoder looks for each mid-bit transition within a quarter bit
of its prediction, which this jitter moves past in one step. `test_eth_short_link` takes 802.3
B.4.1's jitter budget for a short link: a transmitter equalized for the longest cable gives a short one the reverse
sign of a cable's data-dependent jitter, every transition after a whole bit with none 6 ns early and every other one
6 ns late (B.4.1's 6.0 ns for the medium with equalization), plus +/-6.5 ns at random on each, the rest of the 12.5 ns
that 14.3.1.3.1 leaves beside the MAU receiver's 1.5, at the transceiver model's slowest corner with the receiver's
skew reversed. `test_eth_mirror_uaa` sends the three frames from a universally administered source address (802.3
3.2.3 c), as a network card would, with the mirror of the fixed pattern at the slowest corner, phase 5.1 ns.
`test_eth_excess_fixed` and `test_eth_excess_mirror` send ARP requests that end in excess bits, under the fixed pattern
and under its mirror: good frames with 2, 3, 4, 6 and 7 excess bits (the 7 ending in a 1), which must be answered and
counted kept, and frames with FCS errors and 1, 3 and 6, which must be dropped. With `test_eth_dribble`, every count
from 1 to 7 is answered on a good frame. Seven excess bits that end in a 0 under the fixed pattern are one of section
7.3's stated limits, and none of these tests sends them.

`test_eth_collision` runs `../programs/advanced/eth_hd.s`, the half-duplex node with CSMA/CD (IEEE 802.3 clause 4), on
threads 0 and 1. The partner sends an ARP request, then watches the chip's `uo5` (DE): each time DE stays high for 8
cycles, longer than a link test pulse, it starts a collided attempt of its own (56 preamble bits, the SFD and a 32-bit
jam) a chosen number of bit times into the chip's transmission, 540 into the first (a late collision, with the FCS on
its way), 100 into the second and 30 into the third (the preamble), and lets the fourth through. The transmit pair, as
in `test_eth_arp`, goes to `../programs/bench/eth_hd_check.py` with the receive pair: each attempt jammed (the whole
header, then 32 bits or more after the collision, ending within 62 bit times of it, or within 34 of the SFD after a
collision in the header, with no valid FCS), each backoff r slot times with r from 0 to 2^n - 1, the gap before each
attempt, the same frame each time, and the fourth attempt the reply, whole and right. The counters, read with RDMEM,
must show one request kept, the three fragments dropped as runts, three collisions, one of them late, and no frame
dropped. About 75,000 cycles.

`test_unit_rdiff` receives on `ui5` with RKIND 3 and RDIFF (5.9.6) from a line driven at both clock edges: three frames of biphase mark at 6.144 Mbit/s, S/PDIF's bit rate at 96 kHz, whose clock edges are the bit starts, and three of differential Manchester at 4 Mbit/s, IEEE 802.5's Token Ring rate, whose clock edges are mid-bit, received with RINV; 40 to 159 random bits a frame, every change moved by a random amount up to +/-10 ns (biphase mark) or +/-20 ns, and the line's bit time 0.1% off the unit's. The words the unit receives must be the bits that `ref_decode` finds on the line as the pad saw it, and those must be the bits sent. `ref_decode` is written from the codes' definitions alone, biphase mark as AES3's channel coding gives it (EBU Tech 3250, 2.3: each bit a symbol of two states, the first state different from the previous symbol's second, the second equal to the first for a 0 and different for a 1) and differential Manchester as section 5.9.5's table gives IEEE 802.5's, and reads only the levels the pad got in each half cycle: about one bit between two changes, or two of about half a bit, and nothing else. About 19,000 cycles.

`test_unit_rper2_clean` receives three clean Manchester lines of 81 bits on `ui5` with RKIND 3 at RPER 2 with a
fraction, p = 586, 594 and 595, and RGAIN 0, a full pull: there the predicted clock edge can sit up to 128 256ths of a
cycle after the line's, more than *p*/8, so the keep test's bound is 128 at RPER 2 (5.9.6). The unit must give exactly
the line's bits, one reception a line. A bound of *p*/8 loses all three lines. About 10,000 cycles.

`test_can_third_node` and `test_can_stuff_error_flag` load `../programs/advanced/can_unit.s` on thread 0 and play a
CAN bus at 500 kbit/s at the pins with the bus model of `../programs/bench/can_unit.py`: TXD on `io0`, open drain,
through a transmit path of 5 cycles to a wired-AND bus with the bench's reference nodes, and the bus through a receive
path of 5 cycles to RXD on `ui6`. In the first, two other nodes arbitrate with the loser 50 cycles behind the winner,
so every dominant bit of the arbitration field reaches RXD 50 cycles longer. A CAN node resynchronizes on
recessive-to-dominant edges only (Bosch CAN 2.0 part A, the synchronization rules), and so does the unit with RESYNC
and RACQ (5.9.6): the chip must report the winner's frame and acknowledge it, and the bench's reference decoder must
read one clean frame on the bus. With every edge pulling the sample point, the chip loses this frame. In the second,
a sender makes a stuff error in each of three frames and then releases the bus. With SMODE 2 the reception ends at
the violating bit (5.9.7), so the chip's error flag must start within the next bit, as Error Signalling asks (here
0.29 bit times into it), and the chip must report a stuff error. About 35,000 and 58,000 cycles.
`test_can_third_node_mid` is the first with the loser 60 cycles behind and every edge of RXD in the second half of
its cycle, after the falling-edge sample, where the cycle's two samples differ (section 3.1): timed samples take an
edge and its level from the rising-edge samples (5.9.6), so the chip must report the same frame. About 35,000
cycles.

`test_usb_enumerate` loads `../programs/advanced/usb_enum.s` on threads 0, 1 and 2 and plays a USB host at the pins: the bus model of `../programs/bench/frontend.py` (the device's 1.5 kohm pull-up on D-, the host's pull-downs) between `io0` (D+) and `io1` (D-), and the host model of `../programs/bench/usb_enum.py`, whose packets are built from USB 2.0 by `../programs/bench/usb_check_test.py`. The host reads the 18-byte device descriptor at address 0 in packets of 8, sets address 12, reads the descriptor there with one of its ACKs lost, gets a NAK for an IN with nothing due, reads the 9-byte configuration descriptor and GET_CONFIGURATION (0), sets configuration 1, reads GET_CONFIGURATION (1) and GET_STATUS of interface 0, which exists once configured (USB 2.0 9.4.5), and has SET_CONFIGURATION(2) STALLed, drives a 50 us SE0 that the device takes as a reset, then reads 8 bytes at address 0 and finds address 12 silent. Every packet the host sends carries dribble bits before its EOP, 1 to 6 bit times in turn, fewer where the packet's own final 1s leave less room for USB 2.0 7.1.9.1's six, which the units' byte end drops in hardware (5.9.7). The record of D+ and D- must pass every check of `../programs/bench/usb_check.py`, told to take that SE0 as a reset (a 10 ms one would be 500,000 cycles): bit rate, jitter, SYNC, stuffing, PIDs, CRCs, EOP, inter-packet delay and response time, the control transfers with their toggles, the retry and the address change, the descriptor, and each request against USB 2.0 9.4 in its device state. The program's counters, read with RDMEM, must show address 0, 10 SETUPs and 9 completed transfers, and its configuration word 0 after the reset. About 225,000 cycles.

The `test_unit_*` tests cover the shift units of `docs/ISA.md` section 5.9, with the `uio` pads looped back as on the board, so one thread's unit can receive another's: UART bursts on a `uo` pin, timed from the commit that starts them; a UART on `io0` received by another thread's unit; a 60-byte Manchester frame with its CRC-32 appended by one unit and checked by another, against `zlib`; USB low-speed style packets with NRZI, bit stuffing and CRC-16, decoded at the pads and by the receiving unit; ABORT and `pcfg` taking pins off their shift outputs, and `smap` on a pin its thread doesn't own; `sstat`'s sticky bits and T; and RDREG 12 to 31. Their programs are assembly source in `test.py`, assembled by `../sim/asm.py`.

`test_matches_simulator` loads random programs, runs them on all four threads with random host traffic, and steps the simulator in `../sim/` beside the RTL, one cycle per clock. The programs draw from all 89 rows, the shift units' included, and seldom draw a row that can stop or block a thread for long. The host traffic meets every limit of section 9.1 and goes no further: SCK low or high for 2 cycles at a period of 6, CS falling 1 cycle before the first rising SCK and rising 1 cycle after the last falling one, and CS high for 3 cycles between transactions. Each thread starts by claiming its pins, the bypass lanes in `BYP` so that they follow their inputs. After each transaction the host acts on what it finds: a thread stopped by an illegal word resumes after a WRMEM replaces that word, a halted one resumes with RUN, a thread the host stopped stays stopped while 3 others run, and one that runs but has retired nothing since the last look starts again at a fresh PC, with the word it sat on rewritten. Half of the SETPC, STEP and RDREG commands name a stopped thread when there is one. Every output pin except MISO must match in both halves of every cycle, and every MISO byte must be what its command returns. Random images seldom move a pin at a falling edge (seeds 1 to 16: no cycle whose halves differ), so the half-cycle paths are compared every cycle by `../verif/lockstep.py --gen half` and `--gen rk3` and at the pins by the `test_eth_*` tests and `test_unit_rdiff`. It fails too if the stimulus is weak: fewer than one instruction retired per 8 cycles, or fewer than one output change per 100 cycles. Most output changes are lanes following their inputs; the log line also counts the changes that are not only on a lane in bypass, about 1 in 200 cycles. `make -B COCOTB_TEST_FILTER=test_matches_simulator LOCK_SEED=7` runs it alone on another seed (the default is 2); seeds 1 to 16 all passed, with at least 0.48 instructions retired and 0.29 output changes per cycle. It runs at RTL; `LOCK_GATES=yes` runs it at gate level too.

The `test_host_*` tests run `../host/pehost.py` and `../host/bringup.py`, the demo board's host library, unchanged: cocotb's `bridge` and `resume` let its blocking calls move the pins. They use the RP2350's timing as `../host/board.py` sets it up (SCK 5 MHz from the PLL that also makes the chip's clock, and the SPI block's gaps between bytes), section 9.1's limits (clk/6 with phases of 2 and 4 cycles, CS high for exactly 3 cycles and falling 1 cycle before the first rising SCK), and MicroPython's default 4 MHz SCK, unrelated to the chip's clock. They run the bring-up, a program loaded from an image file, and CAPS with MISO read at points around each rising SCK; at gate level they also compare every pin with the RTL.

The tests need cocotb 2.0.1; the Makefile stops with a message on any other version.

## How to run

RTL simulation:

```sh
make -B
```

Gate-level simulation: harden the project, copy the netlist `../tt_submission/tt_um_sarkar22_pe.v` (the GDS action provides the same file) to `gate_level_netlist.v`, then:

```sh
make -B GATES=yes
```

At gate level `tb.v` also runs the RTL core beside the netlist and compares every output pin in both halves of every cycle, 2 ns after each edge of tick, and `test_netlist_matches_rtl` runs firmware on all four threads, including bypass lanes and open-drain pins, and fails if any output pin differs in any half cycle. The PDK's SRAM model samples on its clock net, so in a zero-delay run a clock tree that puts the SRAM clock many buffers behind the flops can make this test fail although the timing is fine.

## Waveforms

An RTL run writes `tb.fst`. View it with `gtkwave tb.fst tb.gtkw` or `surfer tb.fst`. A gate-level run is too long to dump.
