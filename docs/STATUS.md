# Status

Updated 2026-11-20.

## Where the design is

- **Chip:** 8x4 tiles on IHP SG13CMOS5L, with the March shuttle's flow, since 2026-11-06 ([DECISIONS.md](DECISIONS.md)).
- **Core RTL:** complete for the published instruction set, in `src/`. Four threads, the pin layer with bypass and open drain, the SPI host link, and one 512x16 SRAM for code and data.
- **Shift units:** one per thread, in `src/pe_shift.sv` (section 5.9): sending and receiving with their own bit timing, CRC up to 32 bits, bit stuffing, USB's end of packet and a Manchester decoder, for low-speed USB and 10 Mbit Ethernet at 50 MHz. The simulator in `sim/` models them.
- **Specification:** [ISA.md](ISA.md), sections 1 to 11. Changes until the hardware freezes are listed in [DECISIONS.md](DECISIONS.md). Three simulators of it are in [`sim/`](../sim/README.md), one compiled from its code blocks and table.
- **Programs:** 59 programs for 20 protocols in [`programs/`](../programs/README.md), each checked on the simulator in `sim/`. Eleven of them run their protocol on the shift units: an Ethernet node that answers ARP and ping, a USB device that a host enumerates, an SPI target, a CAN node, an SD card host, a 1-Wire master, a UART with parity, WS2812, XY2-100, PRBS generators and a CRC engine.
- **No person has read the RTL yet.** [AI_USE.md](AI_USE.md) says what was checked by machine.

## Chip build

The first build with the shift units, from a local build with the TT flow and `src/config.json` (8x4 tiles, IHP SG13CMOS5L, LibreLane 3.1.0.dev4, IHP-Open-PDK `89d42ee6`, 20 ns clock). The online build of `0d6769e` matches it on all 214 values of `metrics.json`:

| Measure | Value |
|---|---|
| Synthesized (`AREA 0`) | 28,130 cells, 429,616 µm² |
| Standard cells after placement and routing, with repair and hold buffers | 38,738 cells, 557,910 µm² |
| SRAM macro | `RM_IHPSG13_1P_512x16_c2_bm_bist`, 45,309 µm² |
| Utilization | 49.9% of the 1,208,170 µm² core |
| Setup slack, worst | +2.03 ns, slow corner (1.08 V, 125 °C) |
| Hold slack, worst | +0.08 ns, fast corner (1.32 V, −40 °C) |
| Max slew, max capacitance, in every corner | 0, 0; no pin comes within 30% of its slew limit |
| Route DRC, KLayout DRC, LVS, antenna, power grid | 0, 0, 0, 0, 0 |
| SRAM power columns, `tools/check_macro_power.py` on this build's GDS | all 56 on the right supply net, 0 bad |
| Power | 13.5 mW, typical corner |
| Shortest clock period that meets setup, slow corner | 18.0 ns (55.6 MHz). At 16.667 ns (60 MHz), 41 endpoints miss by up to 1.3 ns; the worst path is in a unit's receive logic |
| Local run time, 8 threads | 42 min, then 9.5 min for its 32 gate-level tests |

The flow stops the build on a setup, hold, max-slew or max-capacitance violation in any of the three corners. It also reports 247 max-fanout warnings against the library's limit of 8; all of them are clock-tree leaf buffers that drive 10 to 19 flops ([DECISIONS.md](DECISIONS.md), 2026-11-07). The settings that got here, and why each is needed, are in `src/config.json` and in [DECISIONS.md](DECISIONS.md), 2026-11-11.

The last build without the shift units (online build of `913813a`) had 10,645 cells, 181,003 µm², 18.7% utilization, +7.53 ns of setup slack and 4.8 mW.

## Tests

| Check | Result |
|---|---|
| `test/`: cocotb at the pins, over the real SPI link | 35 pass, 1 skipped (a gate-level-only test) on Icarus 12.0 and 13.0, with nine tests of the shift units, one of a bypass lane taken over and handed back at exact cycles, and two end to end: a 10BASE-T ARP and ping exchange, and a USB enumeration |
| Gate level, on the hardened netlist with the shift units and the RTL beside it, pin by pin | 35 pass, 1 skipped (the lockstep test, which `LOCK_GATES=yes` runs at gate level too), on Icarus 13.0 |
| `programs/check.py`: every program on the simulator | 830 checks pass |
| 10BASE-T and USB low speed end to end, on the shift units at 50 MHz, judged by checkers written from IEEE 802.3, the RFCs and USB 2.0 before the firmware (`programs/bench/eth_check.py`, `usb_check.py`, and a board model in `frontend.py`) | 10BASE-T (`advanced/eth_arp.s`): every ARP and echo reply right field by field; link pulses 14.5 to 15.7 ms apart; TP_IDL, FCS and the gap before each frame inside clause 14 through three transceiver corners; every thread pair, 4 alignments and 8 sampling phases. USB (`advanced/usb_enum.s`): enumeration with resets, keep-alives, lost ACKs and faults, every answer 4.1 to 5.6 bit times after the host's EOP, against 6.5 allowed. Both on the RTL every cycle against `sim/sim.py` (9.7 M cycles, 0 differ) and both cocotb tests on the submitted netlist. 26 bugs planted in the programs are caught by their benches. A review planted 20 more, in the programs and the checkers, and 19 were caught; the one missed changes nothing observable |
| Every feature of the shift units that a real protocol uses, run by one: an SPI target on outside clocks (`advanced/spi_unit.s`), CAN 2.0A with the units' stuffing and CRC-15 (`can_unit.s`), an SD card host with CRC7 and CRC16 (`sd_unit.s`), a 1-Wire master on an open-drain shift output (`ow_unit.s`), a UART with parity (`uart_par_unit.s`), WS2812 (`ws2812_unit.s`), XY2-100 on three units started together (`xy2_unit.s`), PRBS7, 9 and 15 (`lfsr_unit.s`) and an idle send half as a CRC engine (`crc_unit.s`) | Each judged by a reference written apart from its program: sigrok's decoders, `zlib`, the standards' check values, a card model from the SD specification, a controller model from the RP2350 data sheet. The RTL against `sim/sim.py` every cycle on each: 229 scenarios, 15.5 M cycles, 0 differ. The stages planted 84 bugs and their benches caught 82; the other two change nothing a host sees, because the firmware's length check drops those packets anyway. A review planted 27 more, all caught |
| `sim/compare.py`: the three simulators of `sim/`, one compiled from ISA.md's code blocks and table, side by side on every cycle | Every program three ways and 300 random images: 477 runs, 3,216,000 cycles, 0 differ. Four changes planted in a copy of the document each make the simulator that follows them stand apart |
| `sim/claims.py`: statements ISA.md makes about its own tables and examples | 226 pass |
| `programs/sigrok/`: each program's bus as a logic analyzer on the board would see it, decoded by sigrok's protocol decoders, which other people wrote from the protocol documents | 46 of 59 programs decoded. 220 checks pass on the simulator and 365 on the RTL, whose pads and board levels equal the simulator's on every cycle. 12 planted bugs caught. It found two firmware bugs, now fixed: in the I2C controller and the CEC initiator. See [programs/sigrok/README.md](../programs/sigrok/README.md) |
| `verif/lockstep.py`: the RTL against `sim/sim.py` every cycle, the whole chip, with host commands through the real command block and, in one mode, through the real SPI block at the pads | 8,151 images (52 M cycles) and 2,708 scripted corner cases, 0 differ. Planted bugs: 31 of 31 in the shift units' wiring, 10 of 10 in host replies and memory, 6 of 8 in the link, 79 of 80 from the core's earlier list; the 3 missed are equivalent within section 9.1. See [COVERAGE.md](COVERAGE.md) |
| `verif/coverage.sh`: Verilator line, branch and toggle coverage of `src/` under the lockstep and every program bench on the RTL, and functional coverage against [ISA.md](ISA.md) | All 487 program-bench checks pass on the RTL. Line points 309 of 310, branch 550 of 578, toggle 98.4%; no point that wasn't hit can be. Functional points 844 of 852; the 8 left are state changes section 5.9 can't make. See [COVERAGE.md](COVERAGE.md) |
| `formal/`: SymbiYosys proofs on the RTL, read with slang as the flow reads it | Proved for every state reachable from reset, by k-induction: an open-drain pin can't drive high; all 24 pads follow section 5.2's equations; a bypass lane has no flop; thread pin isolation, 114 rules; each shift unit's timing, 10 checks for every TPER, TFRAC and TAIL; every flop's reset value; no pad drives until a program asks. A cover reaches each property's trigger, and 14 planted bugs are caught. See [formal/README.md](../formal/README.md) |
| `formal/chip.eqy`: the gate netlist against the RTL, both the synthesis output and the final netlist after layout, the one submitted | Proved: 108 of 108 pieces on each, by k-induction and, for the 6 that hold the host's command state machine, PDR. Netlist bugs planted in the pieces each rule closes are caught. An earlier result, "90 of 109 pieces", came from EQY's `sat` strategy, whose base case checks nothing on this design, so it was not a proof. See [formal/README.md](../formal/README.md) |
| TT precheck: 10 checks, among them KLayout DRC, the boundary and the Verilog syntax | pass |
| `scripts/setup.sh` on GitHub's ubuntu-24.04 runner, which already has make, git, curl and Python with venv: the pinned tools, then the program checks and the RTL tests | pass: 5 min 40 s, 1.45 GB of disk (venv 463 MB, PDK 982 MB, `tt/` 9.4 MB) |
| Local build of the core without the shift units (`913813a`) with the TT flow, after `scripts/setup.sh --harden`, on a 22-thread laptop | `metrics.json` identical to the online build's, all 196 values; 16 min, 3.4 GB of memory; gate level on its netlist 23 pass, 1 skipped |

The pin-level tests catch 50 of 81 bugs planted in the core RTL before the shift units were added. `verif/lockstep.py` catches 79 of the 80 that still apply; the last one differs only when an SCK phase is shorter than section 9.1 allows.

## Open items

- No SRAM of this kind is known to us to have been built on this process yet. The instruction set keeps a 128-word option for a latch array, which isn't built.
- The CAN bench can't see a wrong sample point: its transceiver model's longest loop delay is 22 cycles.
- The specification has not been read by an outside person yet.
- 10BASE-T at 50 MHz: the receiver loses no frame with up to ±12 ns of random jitter on every edge, at any sampling phase. At 802.3's worst case of ±13.5 ns it loses frames at one sampling phase in eight: 1 in 5 at that phase, 2.5% over all phases, and 11% over all phases with the worst fixed pattern of early and late edges. The transmitter's edges sit up to 5 ns off the ideal grid: inside the 8.5 ns allowed into a resistive load, over the 3.5 ns allowed through 802.3's cable model. Both come from timing edges in 20 ns steps.
- 10BASE-T: no collision detection (half-duplex CSMA/CD), echo replies up to 57 data octets, no IP options or reassembly. The program uses 510 of the 512 words.
- USB: `usb_enum.s` takes one dribble bit before the host's EOP (USB 2.0 7.1.9.1, which lets a hub add up to six bit times with no transition) by reading its CRC register, and refuses two or more. It STALLs GET_DESCRIPTOR(CONFIGURATION) and SET_CONFIGURATION, which a real host asks next: they need about 20 more words, and 3 are free.
- CAN on the units (`can_unit.s`): RESYNC pulls the bit grid on both edges, where a CAN receiver resynchronizes on recessive-to-dominant edges only, so a dominant bit stretched by about 25 cycles at 500 kbit/s (13 at 1 Mbit/s), by a transceiver or by two transmitters out of step in arbitration, breaks the reception. On the simulator, RGAIN 1 with the sample at two thirds of the bit takes 52 cycles; the program doesn't use it yet. No extended frames, error counters or overload frames.
- SD card host (`sd_unit.s`): a card output delay between H − 1 and H cycles, at H cycles a half clock, loses the response's start bit, because a hunt takes no sample in the cycle it starts. 1-bit bus only (a 4-bit bus needs five receive halves), reads only, clock up to 8.33 MHz.
- SPI target on the units (`spi_unit.s`): words framed by a CS pulse each run up to 3.57 MHz; a stream with CS held low runs to 8.33 MHz (16.7 MHz with MISO moved on the capture edge).
- I2S and TDM, two of the flexibility pool's protocols, have not run on the units yet. The LFSR starts only from all ones.

## Dates

| Date | What |
|---|---|
| 2026-10-30 | Template design builds and passes every online check |
| 2026-11-05 | Core RTL with the SRAM on `main`, every online check green |
| 2026-11-06 | Move to 8x4 tiles and the March shuttle's flow |
| 2026-11-07 | Slew and capacitance checks enforced in every corner, clock-buffer fanout warnings explained |
| 2026-11-10 | Shift units in the RTL; every test and program passes on it |
| 2026-11-11 | First chip build with the shift units passes every enforced check |
| 2026-11-17 | 10BASE-T ARP and ping, and a USB enumeration, end to end on the RTL and on the submitted netlist |
| 2026-11-17 | Every feature of the shift units that a real protocol uses run by one, on the simulator and the RTL |
| 2026-12-06 | Hardware freeze, tag `hw-freeze` ([flexibility trial](../flex/README.md)) |
| 2026-12-07 | Three protocols drawn for the flexibility trial |
| 2027-01-13 | Submission |
