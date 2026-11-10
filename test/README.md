# Tests at the pins

`test.py` drives the design through its pads with [cocotb](https://docs.cocotb.org/en/stable/) under Icarus Verilog. The host side speaks the SPI link of `docs/ISA.md` section 9 at the fastest legal rate, clk/6. `tb.v` holds the top module and the wires cocotb drives.

What the tests cover: reset state, CAPS, MISO timing, memory read and write with wrap at DEPTH, a UART frame sent by a program and decoded at the pad, RUN, STOP, STEP, SETPC and RDREG, the mailbox, BUSY, ABORT, commands cut at any byte, a STATUS cut short, a memory command that fails while all threads run, memory commands with one thread stopped, every unlisted command code, bytes sent past a reply, a reset in mid-run, what STEP and SETPC leave alone, an event firing on a stopped thread, WAITING through a second stop, deadlines 32,767 and 32,768 ahead, 16-bit compares, and the bypass lanes.

The `test_unit_*` tests cover the shift units of `docs/ISA.md` section 5.9, with the `uio` pads looped back as on the board, so one thread's unit can receive another's: UART bursts on a `uo` pin, timed from the commit that starts them; a UART on `io0` received by another thread's unit; a 60-byte Manchester frame with its CRC-32 appended by one unit and checked by another, against `zlib`; USB low-speed style packets with NRZI, bit stuffing and CRC-16, decoded at the pads and by the receiving unit; ABORT and `pcfg` taking pins off their shift outputs, and `smap` on a pin its thread doesn't own; `sstat`'s sticky bits and T; and RDREG 12 to 31. Their programs are assembly source in `test.py`, assembled by `../sim/asm.py`.

`test_matches_simulator` runs at RTL only. It loads random programs, runs them on all four threads with random host traffic, and steps the simulator in `../sim/` beside the RTL, one cycle per clock. The simulator doesn't have the shift units yet, so the programs leave out their eight rows, which it lists in `PENDING`. Every output pin except MISO must match on every cycle, and every MISO byte must be what its command returns. It fails too if the stimulus is weak: fewer than one instruction retired per 8 cycles, or fewer than one output change per 100 cycles.

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

At gate level `tb.v` also runs the RTL core beside the netlist, and `test_netlist_matches_rtl` runs firmware on all four threads, including bypass lanes and open-drain pins, and fails if any output pin differs on any cycle. The PDK's SRAM model samples on its clock net, so in a zero-delay run a clock tree that puts the SRAM clock many buffers behind the flops can make this test fail although the timing is fine.

## Waveforms

An RTL run writes `tb.fst`. View it with `gtkwave tb.fst tb.gtkw` or `surfer tb.fst`. A gate-level run is too long to dump.
