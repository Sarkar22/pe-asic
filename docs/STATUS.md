# Status

Updated 2026-11-06.

## Where the design is

- **Chip:** 8x4 tiles on IHP SG13CMOS5L, with the March shuttle's flow, since 2026-11-06 ([DECISIONS.md](DECISIONS.md)).
- **Core RTL:** complete for the published instruction set, in `src/`. Four threads, the pin layer with bypass and open drain, the SPI host link, and one 512x16 SRAM for code and data.
- **Specification:** [ISA.md](ISA.md), sections 1 to 11. Changes until the hardware freezes are listed in [DECISIONS.md](DECISIONS.md).
- **Programs:** 47 programs for 17 protocols in [`programs/`](../programs/README.md), each checked on the simulator in `sim/`.
- **No person has read the RTL yet.** [AI_USE.md](AI_USE.md) says what was checked by machine.

## Chip build

From the online build of `913813a` (8x4 tiles, IHP SG13CMOS5L, LibreLane 3.1.0.dev4, IHP-Open-PDK `89d42ee6`, 20 ns clock):

| Measure | Value |
|---|---|
| Standard cells | 10,645 cells, 181,003 µm² |
| SRAM macro | `RM_IHPSG13_1P_512x16_c2_bm_bist`, 45,309 µm² |
| Utilization | 18.7% of the 1,208,173 µm² core |
| Setup slack, worst | +7.53 ns, slow corner (1.08 V, 125 °C) |
| Hold slack, worst | +0.10 ns, fast corner (1.32 V, −40 °C) |
| Route DRC, KLayout DRC, LVS, antenna, power grid | 0, 0, 0, 0, 0 |
| SRAM power columns, `tools/check_macro_power.py` on this build's GDS | all 56 on the right supply net, 0 bad |
| Power | 4.8 mW |
| Online run time | gds 33 min, precheck 6 min |

The flow now stops the build on a setup or hold violation in any of the three corners. The last 6x4 build, with the old flow, had +4.62 ns of setup slack. The cell and SRAM timing files are the same in both PDK versions, so the gain comes from the layout and the flow.

## Tests

| Check | Result |
|---|---|
| `test/`: cocotb at the pins, over the real SPI link | 23 pass, 1 skipped (an RTL-only test) on Icarus 12.0 and 13.0 |
| Gate level, on the hardened 8x4 netlist with the RTL beside it, pin by pin | 23 pass, 1 skipped |
| `programs/check.py`: every program on the simulator | 458 checks pass |
| TT precheck: 10 checks, among them KLayout DRC, the boundary and the Verilog syntax | pass |
| `scripts/setup.sh` on GitHub's ubuntu-24.04 runner, which already has make, git, curl and Python with venv: the pinned tools, then the program checks and the RTL tests | pass: 5 min 40 s, 1.45 GB of disk (venv 463 MB, PDK 982 MB, `tt/` 9.4 MB) |

The pin-level tests catch 50 of 81 bugs planted in the RTL to test them. The other 31 are caught by slower development benches that compare hidden state with the simulator every cycle. Those benches are not in this repo yet.

## Open items

- The flow reports 101 max-fanout warnings against a limit of 8: 98 on clock buffers that drive 9 to 16 flops, and 3 on buffers it inserted itself, with 9 to 12 loads. It reports no max-capacitance or max-slew warnings in any corner. The last 6x4 build had 3 max-capacitance warnings on the SRAM's `A_DOUT[0..2]` and 15 max-slew warnings in the slow corner; the SRAM's LEF and timing files did not change, so those went with the new layout. The template's config doesn't enforce slew, capacitance or fanout. Before the next milestone the slew and capacitance checks get enforced, and the fanout warnings get fixed or explained.
- No SRAM of this kind is known to us to have been built on this process yet. The instruction set keeps a 128-word option for a latch array, which isn't built.
- The CAN bench can't see a wrong sample point: its transceiver model's longest loop delay is 22 cycles.
- The specification has not been read by an outside person yet.

## Dates

| Date | What |
|---|---|
| 2026-10-30 | Template design builds and passes every online check |
| 2026-11-05 | Core RTL with the SRAM on `main`, every online check green |
| 2026-11-06 | Move to 8x4 tiles and the March shuttle's flow |
| 2026-11-22 | Slew and capacitance checks enforced, clock-buffer fanout warnings fixed or explained |
| 2026-12-06 | Hardware freeze, tag `hw-freeze` ([flexibility trial](../flex/README.md)) |
| 2026-12-07 | Three protocols drawn for the flexibility trial |
| 2027-01-13 | Submission |
