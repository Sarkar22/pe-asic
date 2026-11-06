# Status

Updated 2026-11-05.

## Where the design is

- **Core RTL:** complete for the published instruction set, in `src/`. Four threads, the pin layer with bypass and open drain, the SPI host link, and one 512x16 SRAM for code and data.
- **Specification:** [ISA.md](ISA.md), sections 1 to 11. Changes until the hardware freezes are listed in [DECISIONS.md](DECISIONS.md).
- **Programs:** 47 programs for 17 protocols in [`programs/`](../programs/README.md), each checked on the simulator in `sim/`.
- **No person has read the RTL yet.** [AI_USE.md](AI_USE.md) says what was checked by machine.

## Chip build

From the online build of `d00ddaa` (6x4 tiles, IHP SG13CMOS5L, 20 ns clock):

| Measure | Value |
|---|---|
| Standard cells | 11,313 cells, 176,766 µm² |
| SRAM macro | `RM_IHPSG13_1P_512x16_c2_bm_bist`, 45,309 µm² |
| Utilization | 24.6% |
| Setup slack, worst | +4.62 ns, slow corner (1.08 V, 125 °C) |
| Hold slack, worst | +0.10 ns, fast corner (1.32 V, −40 °C) |
| Route DRC, KLayout DRC, LVS, antenna | 0, 0, 0, 0 |
| Power | 4.8 mW |
| Online run time | gds 31 min, precheck 35 min |

## Tests

| Check | Result |
|---|---|
| `test/`: cocotb at the pins, over the real SPI link | 23 pass, 1 skipped (an RTL-only test) on Icarus 12.0 and 13.0 |
| Gate level, on the hardened netlist with the RTL beside it, pin by pin | 23 pass, 1 skipped |
| `programs/check.py`: every program on the simulator | 458 checks pass |
| TT precheck | pass |
| `scripts/setup.sh` on GitHub's ubuntu-24.04 runner, which already has make, git, curl and Python with venv: the pinned tools, then the program checks and the RTL tests | pass |

The pin-level tests catch 50 of 81 bugs planted in the RTL to test them. The other 31 are caught by slower development benches that compare hidden state with the simulator every cycle. Those benches are not in this repo yet.

## Open items

- The flow reports 3 max-capacitance warnings on the SRAM's `A_DOUT[0..2]`, 15 max-slew warnings in the slow corner and 95 max-fanout warnings on clock leaf buffers. The template's config doesn't enforce these. They get fixed, and slow-corner checks enforced, before the next milestone.
- No SRAM of this kind is known to us to have been built on this process yet. The instruction set keeps a 128-word option for a latch array, which isn't built.
- The CAN bench can't see a wrong sample point: its transceiver model's longest loop delay is 22 cycles.
- The specification has not been read by an outside person yet.

## Dates

| Date | What |
|---|---|
| 2026-10-30 | Template design builds and passes every online check |
| 2026-11-05 | Core RTL with the SRAM on `main`, every online check green |
| 2026-11-22 | Chip warnings above fixed, slow-corner timing enforced |
| 2026-12-06 | Hardware freeze, tag `hw-freeze` ([flexibility trial](../flex/README.md)) |
| 2026-12-07 | Three protocols drawn for the flexibility trial |
| 2027-01-13 | Submission |
