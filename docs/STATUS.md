# Status

Updated 2026-11-11.

## Where the design is

- **Chip:** 8x4 tiles on IHP SG13CMOS5L, with the March shuttle's flow, since 2026-11-06 ([DECISIONS.md](DECISIONS.md)).
- **Core RTL:** complete for the published instruction set, in `src/`. Four threads, the pin layer with bypass and open drain, the SPI host link, and one 512x16 SRAM for code and data.
- **Shift units:** one per thread, in `src/pe_shift.sv` (section 5.9): sending and receiving with their own bit timing, CRC up to 32 bits, bit stuffing, USB's end of packet and a Manchester decoder, for low-speed USB and 10 Mbit Ethernet at 50 MHz. The simulator in `sim/` models them.
- **Specification:** [ISA.md](ISA.md), sections 1 to 11. Changes until the hardware freezes are listed in [DECISIONS.md](DECISIONS.md).
- **Programs:** 47 programs for 17 protocols in [`programs/`](../programs/README.md), each checked on the simulator in `sim/`.
- **No person has read the RTL yet.** [AI_USE.md](AI_USE.md) says what was checked by machine.

## Chip build

The first build with the shift units, from a local build with the TT flow and this branch's `src/config.json` (8x4 tiles, IHP SG13CMOS5L, LibreLane 3.1.0.dev4, IHP-Open-PDK `89d42ee6`, 20 ns clock):

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
| Local run time, 8 threads | 42 min, then 9.5 min for the gate-level tests |

The flow stops the build on a setup, hold, max-slew or max-capacitance violation in any of the three corners. It also reports 247 max-fanout warnings against the library's limit of 8; all of them are clock-tree leaf buffers that drive 10 to 19 flops ([DECISIONS.md](DECISIONS.md), 2026-11-07). The settings that got here, and why each is needed, are in `src/config.json` and in [DECISIONS.md](DECISIONS.md), 2026-11-11.

The last build without the shift units (online build of `913813a`) had 10,645 cells, 181,003 µm², 18.7% utilization, +7.53 ns of setup slack and 4.8 mW.

## Tests

| Check | Result |
|---|---|
| `test/`: cocotb at the pins, over the real SPI link | 32 pass, 1 skipped (an RTL-only test) on Icarus 12.0 and 13.0, with nine tests of the shift units |
| Gate level, on the hardened netlist with the shift units and the RTL beside it, pin by pin | 32 pass, 1 skipped (the RTL-only test), on Icarus 13.0 |
| `programs/check.py`: every program on the simulator | 487 checks pass |
| `programs/sigrok/`: each program's bus as a logic analyzer on the board would see it, decoded by sigrok's protocol decoders, which other people wrote from the protocol documents | 38 of 47 programs decoded. 175 checks pass on the simulator and 291 on the RTL, whose pads and board levels equal the simulator's on every cycle. 11 planted bugs caught. It found two firmware bugs, now fixed: in the I2C controller and the CEC initiator. See [programs/sigrok/README.md](../programs/sigrok/README.md) |
| `verif/lockstep.py`: the RTL against `sim/sim.py` every cycle, the whole chip, with host commands through the real command block and, in one mode, through the real SPI block at the pads | 8,151 images (52 M cycles) and 2,708 scripted corner cases, 0 differ. Planted bugs: 31 of 31 in the shift units' wiring, 10 of 10 in host replies and memory, 6 of 8 in the link, 79 of 80 from the core's earlier list; the 3 missed are equivalent within section 9.1. See [COVERAGE.md](COVERAGE.md) |
| `verif/coverage.sh`: Verilator line, branch and toggle coverage of `src/` under the lockstep and every program bench on the RTL, and functional coverage against [ISA.md](ISA.md) | All 487 program-bench checks pass on the RTL. Line points 309 of 310, branch 550 of 578, toggle 98.4%; no point that wasn't hit can be. Functional points 844 of 852; the 8 left are state changes section 5.9 can't make. See [COVERAGE.md](COVERAGE.md) |
| `formal/`: SymbiYosys proofs on the RTL, read with slang as the flow reads it | Proved for every state reachable from reset, by k-induction: an open-drain pin can't drive high; all 24 pads follow section 5.2's equations; a bypass lane has no flop; thread pin isolation, 114 rules; each shift unit's timing, 10 checks for every TPER, TFRAC and TAIL; every flop's reset value; no pad drives until a program asks. A cover reaches each property's trigger, and 14 planted bugs are caught. See [formal/README.md](../formal/README.md) |
| `formal/chip.eqy`: the synthesized netlist against the RTL | Partly proved: 90 of 109 pieces, among them every output pin but uo3. The pieces holding the units, the threads and the pin flops don't close yet |
| TT precheck: 10 checks, among them KLayout DRC, the boundary and the Verilog syntax | pass |
| `scripts/setup.sh` on GitHub's ubuntu-24.04 runner, which already has make, git, curl and Python with venv: the pinned tools, then the program checks and the RTL tests | pass: 5 min 40 s, 1.45 GB of disk (venv 463 MB, PDK 982 MB, `tt/` 9.4 MB) |
| Local build of the core without the shift units (`913813a`) with the TT flow, after `scripts/setup.sh --harden`, on a 22-thread laptop | `metrics.json` identical to the online build's, all 196 values; 16 min, 3.4 GB of memory; gate level on its netlist 23 pass, 1 skipped |

The pin-level tests catch 50 of 81 bugs planted in the core RTL before the shift units were added. `verif/lockstep.py` catches 79 of the 80 that still apply; the last one differs only when an SCK phase is shorter than section 9.1 allows.

## Open items

- No SRAM of this kind is known to us to have been built on this process yet. The instruction set keeps a 128-word option for a latch array, which isn't built.
- The CAN bench can't see a wrong sample point: its transceiver model's longest loop delay is 22 cycles.
- The equivalence check proves only part of the synthesized netlist equal to the RTL ([formal/README.md](../formal/README.md)). Gate-level simulation checks the rest.
- The specification has not been read by an outside person yet.

## Dates

| Date | What |
|---|---|
| 2026-10-30 | Template design builds and passes every online check |
| 2026-11-05 | Core RTL with the SRAM on `main`, every online check green |
| 2026-11-06 | Move to 8x4 tiles and the March shuttle's flow |
| 2026-11-07 | Slew and capacitance checks enforced in every corner, clock-buffer fanout warnings explained |
| 2026-11-10 | Shift units in the RTL; every test and program passes on it |
| 2026-11-11 | First chip build with the shift units passes every enforced check |
| 2026-12-06 | Hardware freeze, tag `hw-freeze` ([flexibility trial](../flex/README.md)) |
| 2026-12-07 | Three protocols drawn for the flexibility trial |
| 2027-01-13 | Submission |
