# Status

Updated 2026-11-27.

## Where the design is

- **Chip:** 8x4 tiles on IHP SG13CMOS5L, with the March shuttle's flow, since 2026-11-06 ([DECISIONS.md](DECISIONS.md)).
- **Core RTL:** complete for the published instruction set, in `src/`. Four threads, the pin layer with bypass and open drain, the SPI host link, and one 512x16 SRAM for code and data.
- **Shift units:** one per thread, in `src/pe_shift.sv` (section 5.9): sending and receiving with their own bit timing, to the half cycle where 10BASE-T needs it (THALF on send, RKIND 3 on receive, with the cell library's latches: sections 3.1 and 5.2), CRC up to 32 bits, bit stuffing, USB's end of packet, a Manchester decoder, and byte ends that keep whole bytes as USB and IEEE 802.3 ask, for low-speed USB and 10 Mbit Ethernet at 50 MHz. The three simulators in `sim/` model them.
- **Specification:** [ISA.md](ISA.md), sections 1 to 11. Changes until the hardware freezes are listed in [DECISIONS.md](DECISIONS.md). Three simulators of it are in [`sim/`](../sim/README.md), one compiled from its code blocks and table.
- **Programs:** 61 programs for 24 protocols in [`programs/`](../programs/README.md), each checked on the simulator in `sim/`; the table in [README.md](../README.md#protocols) gives each protocol's role, speed and checks. Thirteen of them run their protocol on the shift units: an Ethernet node that answers ARP and ping, a USB device that a host enumerates, an SPI target, a CAN node, an SD card host, a 1-Wire master, a UART with parity, WS2812, XY2-100, I2S and TDM as controller and as target, PRBS generators and a CRC engine.

## Chip build

The build of the current RTL, from a local build with the TT flow and `src/config.json` (8x4 tiles, IHP SG13CMOS5L, LibreLane 3.1.0.dev4, IHP-Open-PDK `89d42ee6`, 20 ns clock). The RTL and the flow settings are the same at `04f143e` and `ca6a4b9`. The online build of `04f143e` (gds run 45941778685) matches it on all 243 values of `metrics.json`, and the online build of `ca6a4b9` (gds run 45971600207) gives the same netlist and layout:

| Measure | Value |
|---|---|
| Synthesized (`AREA 0`) | 35,523 cells, 519,245 µm² |
| Standard cells after placement and routing, with repair and hold buffers | 48,061 cells, 669,954 µm² |
| SRAM macro | `RM_IHPSG13_1P_512x16_c2_bm_bist`, 45,309 µm² |
| Utilization | 59.2% of the 1,208,170 µm² core, over the 58% placement target ([DECISIONS.md](DECISIONS.md), 2026-11-26) |
| Setup | No violation in any corner. At the slow corner (1.08 V, 125 °C) the worst slack is +1.23 ns between flops and +0.61 ns from the half-cycle output latches to the pads. The flow's overall worst, 0.00 ns, is on the paths from the input pads into the latches of the falling-edge sample, which borrow time |
| Hold slack, worst | +0.035 ns, fast corner (1.32 V, −40 °C) |
| Max slew, max capacitance, in every corner | 0, 0 |
| Route DRC, KLayout DRC, LVS, antenna, power grid | 0, 0, 0, 0, 0 |
| SRAM power columns, `tools/check_macro_power.py` on this build's GDS | all 56 on the right supply net, 0 bad |
| Power | 15.4 mW, typical corner |
| Online run time | 2 h 38 min for the build, then 52 min for the gate-level job (gds run 45971600207); 3 h 44 min and 52 min in gds run 45941778685 |

The flow stops the build on a setup, hold, max-slew or max-capacitance violation in any of the three corners. It also reports 276 max-fanout warnings against the library's limit of 8; all of them are clock-tree leaf buffers that drive 10 to 19 clock pins ([DECISIONS.md](DECISIONS.md), 2026-11-07). The settings that got here, and why each is needed, are in `src/config.json` and in [DECISIONS.md](DECISIONS.md), 2026-11-11 and 2026-11-23.

The first build with the shift units (`0d6769e`, 2026-11-11) had 38,738 cells, 557,910 µm², 49.9% utilization and 13.5 mW. The last build without them (online build of `913813a`) had 10,645 cells, 181,003 µm², 18.7% utilization, +7.53 ns of setup slack and 4.8 mW.

## Tests

| Check | Result |
|---|---|
| `test/`: cocotb at the pins, over the real SPI link | 48 pass, 1 skipped (a gate-level-only test) on Icarus 12.0 and 13.0: eleven tests of the shift units, one of a bypass lane taken over and handed back at exact cycles, and end to end on the units: nine 10BASE-T tests of ARP and ping (at 802.3's jitter, section 7.3's fixed pattern and its mirror, a cable's and a short link's jitter, from a universally administered address, and with frames that end in excess bits), three CAN tests (a third node behind two arbitrating ones, a stuff error's flag) and a USB enumeration |
| Gate level, on the submitted netlist with the RTL beside it, pin by pin | 48 pass, 1 skipped (the lockstep test, which `LOCK_GATES=yes` runs at gate level too), on Icarus 13.0, in the online builds of `ca6a4b9` (gds run 45971600207) and `04f143e` (45941778685) |
| `programs/check.py`: every program on the simulator | 957 checks pass, on 61 programs (956 on GitHub's runner, which has no sigrok-cli for one USB checker test) |
| 10BASE-T and USB low speed end to end, on the shift units at 50 MHz, judged by checkers written from IEEE 802.3, the RFCs and USB 2.0 before the firmware (`programs/bench/eth_check.py`, `usb_check.py`, and a board model in `frontend.py`) | 10BASE-T (`advanced/eth_arp.s`): every ARP and echo reply right field by field; link pulses 14.85 to 15.73 ms apart; TP_IDL, FCS and the gap before each frame inside clause 14 through three transceiver corners; every thread pair, 4 alignments and 8 sampling phases; no frame of the bench lost at 802.3's ±13.5 ns of jitter, random or in the fixed patterns of section 7.3 (some frames whose 7 excess bits end in a 0 are lost under one of them: see the open items), and every transmit edge within the transceiver's own skew of the 50 ns grid. USB (`advanced/usb_enum.s`): enumeration with resets, keep-alives, lost ACKs and faults, every answer 4.1 to 5.5 bit times after the host's EOP, against 6.5 allowed, with 1 to 6 dribble bit times before an EOP taken as without them. Both benches also run with the RTL in place of the simulator (`verif/coverage.sh`), and the cocotb tests pass at RTL and on the submitted netlist. 26 bugs planted in the programs are caught by their benches; of 20 more planted later, in the programs and the checkers, 19 were caught, and the one missed changes nothing observable |
| Every feature of the shift units that a real protocol uses, run by one: an SPI target on outside clocks (`advanced/spi_unit.s`), CAN 2.0A with the units' stuffing and CRC-15 (`can_unit.s`), an SD card host with CRC7 and CRC16 (`sd_unit.s`), a 1-Wire master on an open-drain shift output (`ow_unit.s`), a UART with parity (`uart_par_unit.s`), WS2812 (`ws2812_unit.s`), XY2-100 on three units started together (`xy2_unit.s`), PRBS7, 9 and 15 (`lfsr_unit.s`), an idle send half as a CRC engine (`crc_unit.s`), and an I2S and TDM controller and target (`i2s_ctl.s`, `i2s_tgt.s`) | Each judged by a reference written apart from its program: sigrok's decoders, `zlib`, the standards' check values, a card model from the SD specification, a controller model from the RP2350 data sheet, and models from the I2S bus specification and the TLV320ADC3140's TDM mode. Every bench also runs with the RTL in place of the simulator (`verif/coverage.sh`), and `programs/sigrok/check.py --rtl` finds the RTL's pads equal to the simulator's on every cycle of their sigrok scenarios. Of 84 bugs planted in these programs and in `usb_enum.s`, the benches caught 82; the other two, in `usb_enum.s`, change nothing a host sees, because its length check drops those packets anyway. 27 more planted later were all caught, and the I2S and TDM bench catches the 6 broken copies it carries. The firmware fixes of 2026-11-27 came with bugs planted in `sd_unit.s` and `can_unit.s`: 29 of 30 caught, the other changing nothing observable; of 14 planted later, 2 were caught only after a new bench row for each, which the benches now carry |
| `sim/compare.py`: the three simulators of `sim/`, one compiled from ISA.md's code blocks and table, side by side on every cycle | Every program three ways, 300 random images and 400 aimed at the half cycles, RKIND 3 (100 of them on 802.3 frames with real FCSs) and CAN: 883 runs, 5,664,000 cycles, 0 differ. Four changes planted in a copy of the document each make the simulator that follows them stand apart |
| `sim/claims.py`: statements ISA.md makes about its own tables and examples | 240 pass |
| `programs/sigrok/`: each program's bus as a logic analyzer on the board would see it, decoded by sigrok's protocol decoders, which other people wrote from the protocol documents | 48 of 61 programs decoded. 243 checks pass on the simulator and 404 on the RTL, whose pads and board levels equal the simulator's on every cycle: 800 runs, 85.8 million cycles. 12 planted bugs caught. It found two firmware bugs, now fixed: in the I2C controller and the CEC initiator. See [programs/sigrok/README.md](../programs/sigrok/README.md) |
| `verif/lockstep.py`: the RTL against `sim/sim.py` every cycle, the whole chip, with host commands through the real command block and, in one mode, through the real SPI block at the pads | 9,951 images (63 M cycles) and 2,708 scripted corner cases, 0 differ, among them images aimed at the half cycles, at RKIND 3, at 802.3 frames with real FCSs and at CAN lines. Planted bugs: 31 of 31 in the shift units' wiring, 10 of 10 in host replies and memory, 6 of 8 in the link, 79 of 80 from the core's earlier list; the 3 missed are equivalent within the specification. See [COVERAGE.md](COVERAGE.md) 2.5, which gives the RTL each set last ran on |
| `verif/coverage.sh`: Verilator line, branch and toggle coverage of `src/` under the lockstep and every program bench on the RTL, and functional coverage against [ISA.md](ISA.md) | The program benches of `04f143e` pass on the RTL, 897 checks of 24 benches. Line points 312 of 313, branch 660 of 688, toggle 98.2%; no point that wasn't hit can be. Functional points 909 of 917; the 8 left are state changes section 5.9 can't make. See [COVERAGE.md](COVERAGE.md) |
| `formal/`: SymbiYosys proofs on the RTL, read with slang as the flow reads it | Proved for every state reachable from reset, by k-induction: an open-drain pin can't drive high; all 24 pads follow section 5.2's equations in both halves of the cycle; a bypass lane has no flop; thread pin isolation, 114 rules; each shift unit's timing, 11 checks for every TPER, TFRAC and TAIL, THALF's moves at the falling edge among them; RKIND 3's decoder equals the code of section 5.9.6 in every state a reception can reach; every flop's and latch's reset value; no pad drives until a program asks. A cover reaches each property's trigger, and 33 planted bugs are caught. See [formal/README.md](../formal/README.md) |
| `formal/chip.eqy`: the gate netlist against the RTL, both the synthesis output and the final netlist after layout, the one submitted | Proved: 125 of 125 pieces on each, by k-induction and, for the 6 that hold the host's command state machine, PDR, with the 40 half-cycle latches boxed on both sides. 12 netlist bugs planted in the pieces each rule closes are caught. An earlier result, "90 of 109 pieces", came from EQY's `sat` strategy, whose base case checks nothing on this design, so it was not a proof. See [formal/README.md](../formal/README.md) |
| TT precheck: 10 checks, among them KLayout DRC, the boundary and the Verilog syntax | pass (gds runs 45971600207 and 45941778685) |
| `scripts/setup.sh` on GitHub's ubuntu-24.04 runner, which already has make, git, curl and Python with venv: the pinned tools, then the program checks and the RTL tests | pass on 2026-11-06 (setup run 42290760572): 5 min 40 s, about a minute of it the installs, with the 458 program checks and 24 tests of then; 1.45 GB of disk (venv 463 MB, PDK 982 MB, `tt/` 9.4 MB). The job runs only when the setup files change |
| Local build of the core without the shift units (`913813a`) with the TT flow, after `scripts/setup.sh --harden`, on a 22-thread laptop | `metrics.json` identical to the online build's, all 196 values; 16 min, 3.4 GB of memory; gate level on its netlist 23 pass, 1 skipped |

The pin-level tests catch 50 of 81 bugs planted in the core RTL before the shift units were added. `verif/lockstep.py` catches 79 of the 80 that still apply; the last one differs only when an SCK phase is shorter than section 9.1 allows.

## Open items

- I know of no SRAM of this kind built on this process yet. The instruction set keeps a 128-word option for a latch array, which isn't built.
- Utilization is 59.2%, over the 58% placement target. Every enforced check passes and the online build passed Tiny Tapeout's precheck, so it stands ([DECISIONS.md](DECISIONS.md), 2026-11-26). The build's margins are thin: hold has +0.035 ns at the fast corner, the paths from the half-cycle output latches to the pads have +0.61 ns of setup at the slow corner, and detailed routing came clean at its 27th iteration of 30.
- 10BASE-T at 50 MHz, receive: RKIND 3 loses no frame at 802.3's ±13.5 ns with random jitter, the fixed patterns of section 7.3, a cable's data-dependent jitter, steps of up to 20 ns and a partner 0.01% off, at any sampling phase, transceiver corner and skew sign. It takes data-dependent jitter of the opposite sign, which a transmitter equalized for a long cable gives on a short one, up to 10 ns alone, up to 4 ns with random jitter making ±13.5 ns, and up to 6 ns with random jitter making ±12.5 ns, IEEE 802.3 B.4.1's split for a short link. Beyond those it loses frames, and so it does to steps of 21 ns or more, to spikes, and, under the pattern of late bit-boundary edges, to some frames whose 7 excess bits end in a 0. The opposite sign at 10 ns alone, 20 ns steps and B.4.1's split hold only with a clock close to 50% duty: with the falling edge 1 to 3.3 ns off they lose up to 66 and 19 of 960 frames and 3 of 3,840 (section 7.4). These limits stand ([DECISIONS.md](DECISIONS.md), 2026-11-26).
- 10BASE-T at 50 MHz, send: with THALF every edge sits on the 50 ns grid within the transceiver's own skew at a 50% clock, and a falling edge *d* ns off moves it by about *d*/2 (section 7.4). IEEE 802.3's transmit template needs an equalizing network on the board (section 7.3). `eth_arp.s` drives the simpler board of its header, which serves the bench, not a conforming link; its transmit constants change with the board.
- 10BASE-T: no collision detection (half-duplex CSMA/CD), echo replies up to 57 data octets, no IP options or reassembly. The program uses 511 of the 512 words.
- USB: `usb_enum.s` answers GET_DESCRIPTOR for its device and configuration descriptors, and SET_ADDRESS, and takes 1 to 6 dribble bit times before a host's EOP through the units' byte end (USB 2.0 7.1.9.1). It STALLs SET_CONFIGURATION, which a host asks next: that takes 6 more words, and 2 are free.
- CAN in firmware (`can.s`): its bench can't see a wrong sample point, because its transceiver model's longest loop delay is 22 cycles. The bench of the node on the units does: dominant bits stretched by up to 72 cycles at RXD are clean, and by 76 are not.
- CAN on the units (`can_unit.s`): extended frames are taken for errors; no error counters, error-passive state or overload frames, and no form check of the end of frame. A stuff error's flag starts in the next bit for bit times of 40 to 140 cycles; a slower rate needs a timed commit there. 5 code words are free below the frame buffer.
- SD card host (`sd_unit.s`): 1-bit bus only (a 4-bit bus needs five receive halves), reads only, clock up to 8.33 MHz.
- SPI target on the units (`spi_unit.s`): words framed by a CS pulse each run up to 3.57 MHz; a stream with CS held low runs to 8.33 MHz (16.7 MHz with MISO moved on the capture edge). MISO is a `uo` pin, so it isn't released while CS is high.
- The LFSR starts only from all ones (CINIT 1, section 5.9.7).

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
| 2026-11-20 | Every feature of the shift units that a real protocol uses run by one, on the simulator and the RTL |
| 2026-11-21 | Half-cycle timing in the shift units, from the cell library's latches: THALF on send, RKIND 3 and the byte ends on receive |
| 2026-11-26 | The 10BASE-T receive decoder finished; a reception keeps its settings; CAN's receive rules; the chip build of that RTL passes every enforced check |
| 2026-11-27 | The online build of that RTL matches the local one, with precheck and the gate-level tests passing; SD's start bit and CAN's error flag fixed in firmware, with no RTL change, and every online check green on that commit |
| 2026-12-06 | Hardware freeze, tag `hw-freeze` ([flexibility trial](../flex/README.md)) |
| 2026-12-07 | Three protocols drawn for the flexibility trial |
| 2027-01-13 | Submission |
