# Tests at the pins

`test.py` drives the design through its pads with [cocotb](https://docs.cocotb.org/en/stable/) under Icarus Verilog. The host side speaks the SPI link of `docs/ISA.md` section 9 inside the rules of its section 9.1: at clk/6 with SCK low and high for 3 cycles each in most tests, and at every limit of 9.1 in `test_matches_simulator`. `tb.v` holds the top module and the wires cocotb drives.

What the tests cover: reset state, CAPS, MISO timing, memory read and write with wrap at DEPTH, a UART frame sent by a program and decoded at the pad, RUN, STOP, STEP, SETPC and RDREG, the mailbox, BUSY, ABORT, commands cut at any byte, a STATUS cut short, a memory command that fails while all threads run, memory commands with one thread stopped, every unlisted command code, bytes sent past a reply, a reset in mid-run, what STEP and SETPC leave alone, an event firing on a stopped thread, WAITING through a second stop, deadlines 32,767 and 32,768 ahead, 16-bit compares, and the bypass lanes.

`test_bypass_override` loads `../programs/advanced/uart_bypass.s` and drives a UART line on `ui4`: with no rule, `uo4` shows `ui4` on every cycle; with a rule, the chosen frame's data bits leave as the host's byte on that frame's own bit boundaries, to the cycle, and the lane switches exactly where the program's header says, 3 cycles into the start bit and 9 into the stop bit. It runs at 16 cycles per bit with a frame rule and at 50 with a byte rule, on two threads, at RTL and at gate level.

The `test_unit_*` tests cover the shift units of `docs/ISA.md` section 5.9, with the `uio` pads looped back as on the board, so one thread's unit can receive another's: UART bursts on a `uo` pin, timed from the commit that starts them; a UART on `io0` received by another thread's unit; a 60-byte Manchester frame with its CRC-32 appended by one unit and checked by another, against `zlib`; USB low-speed style packets with NRZI, bit stuffing and CRC-16, decoded at the pads and by the receiving unit; ABORT and `pcfg` taking pins off their shift outputs, and `smap` on a pin its thread doesn't own; `sstat`'s sticky bits and T; and RDREG 12 to 31. Their programs are assembly source in `test.py`, assembled by `../sim/asm.py`.

`test_matches_simulator` loads random programs, runs them on all four threads with random host traffic, and steps the simulator in `../sim/` beside the RTL, one cycle per clock. The programs draw from all 89 rows, the shift units' included, and seldom draw a row that can stop or block a thread for long. The host traffic meets every limit of section 9.1 and goes no further: SCK low or high for 2 cycles at a period of 6, CS falling 1 cycle before the first rising SCK and rising 1 cycle after the last falling one, and CS high for 3 cycles between transactions. Each thread starts by claiming its pins, the bypass lanes in `BYP` so that they follow their inputs. After each transaction the host acts on what it finds: a thread stopped by an illegal word resumes after a WRMEM replaces that word, a halted one resumes with RUN, a thread the host stopped stays stopped while 3 others run, and one that runs but has retired nothing since the last look starts again at a fresh PC, with the word it sat on rewritten. Half of the SETPC, STEP and RDREG commands name a stopped thread when there is one. Every output pin except MISO must match on every cycle, and every MISO byte must be what its command returns. It fails too if the stimulus is weak: fewer than one instruction retired per 8 cycles, or fewer than one output change per 100 cycles. Most output changes are lanes following their inputs; the log line also counts the changes that are not only on a lane in bypass, about 1 in 200 cycles. `make -B COCOTB_TEST_FILTER=test_matches_simulator LOCK_SEED=7` runs it alone on another seed (the default is 2); seeds 1 to 16 all passed, with at least 0.48 instructions retired and 0.29 output changes per cycle. It runs at RTL; `LOCK_GATES=yes` runs it at gate level too.

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
