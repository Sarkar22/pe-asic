# Formal proofs

Proofs of the chip's safety and timing properties with SymbiYosys, and an equivalence check of the
pre-layout netlist against the RTL with EQY. Every proof reads the RTL in `src/` with slang
(`read_slang`), as the flow does; the property wrappers are plain SystemVerilog that `read_verilog -formal
-sv` reads. Comments cite sections of [`docs/ISA.md`](../docs/ISA.md).

## Results

"Proved" means proved for every reachable state, with no bound: k-induction (`smtbmc`, solver bitwuzla)
closed at the depth shown, and its base case started from reset. Times are wall-clock, one job at a time, in
the LibreLane image.

| Property | Spec | Method | Depth | Result | Time |
|---|---|---|---|---|---|
| **1. Open drain can't drive high** (`pads.sv`, `od_*`): with OD[i], `uio_out[i]` is 0 and `uio_oe[i]` is !V(8+i), in every state, on the whole chip | 5.3: "In `OD` mode the output data to the pad is forced to 0 and only the enable moves, so the pin can't drive high whatever firmware does." 5.2: "A shift output on a pin in `OD` releases the pin on 1 and pulls it low on 0, and the pin still can't drive high." | k-induction | 1 | proved | 3 s |
| **1b. The pad equations** (`pads.sv`, `eq_*`): `uo_out`, `uio_out` and `uio_oe` equal the equations of 5.2 on all 24 pins, with V(p) formed from OUT, SEL, OWN, LA and LB | 5.2, the five equations | k-induction | 1 | proved | (same job) |
| **2. Bypass is transparent and immediate** (`pads.sv`, `byp_*`): with BYP[i], `uo_out[i]` equals `ui_in[SRC(i)]` in the same cycle, for a free `ui_in` | 5.8: "Mode `BYP` gives a pad-to-pad path with no flop in it." 3.2: "A lane in bypass has no flop in its path, so it depends on `ui_in` too." | k-induction | 1 | proved | (same job) |
| **3. Thread pin isolation** (`isolation.sv`, 114 rules): each pin flop (OUT, SHD, OE, SHOE, OD, BYP, SHBYP, OWN, SEL, SHSEL) changes only in a cycle with one of its causes: its owner's instruction naming it, its owner's commit, a `pcfg` of it (and then OWN becomes the claimer), or ABORT. The rule is stated in full in `isolation.sv` | 5.3: "A thread's writes to pins it doesn't own are dropped, and its commit moves only pins it owns. `pcfg` takes a pin with no check, even from a running thread". 5.4: `commit()`, `shadow()`. 9.3: `abort_pins()` | k-induction | 3 | proved | 5 s |
| **3b. Units are their own thread's** (`isolation.sv`): LA, LB of unit t change only while its send half runs or is in its tail, at thread t's commit, at thread t's `sstop tx`, or at ABORT; every step-3 strobe of unit t comes from thread t's instruction of that row; unit t's CMT is thread t's commit | 5.9.1: "A unit starts only at a commit of its own thread". "Units share nothing with each other, so one thread's unit can't change another thread's timing." | k-induction | 3 | proved | (same job) |
| **3c. ABORT releases every uio pad** (`isolation.sv`, `abort_*`): from the next cycle `uio_oe` is 0, OE, SHOE and the uio SEL, SHSEL are 0, OUT and SHD are 1 on open-drain pins | 9.3: "So every `uio` pad is undriven" | k-induction | 3 | proved | (same job) |
| **4. The unit's timing** (`shift_timing.sv`, one unit, every input free in every cycle): `start_cmt` TXRUN rises only after TXARM and CMT; `start_sym` in the commit's cycle `tx_symbol()` runs and from the next cycle LA, LB show symbol 0 of the first bit (MAPA, MAPB, TGLA, TGLB, TMSB, LFSR) and TXCD is TPER; `start_empty` a start with no word is an underrun; `sym_tper` with TFRAC 0 each symbol lasts exactly TPER cycles (TPER 0 as 1); `sym_frac` with any TFRAC it lasts TPER plus the carry of TXACC + TFRAC; `hold` LA, LB move only at those cycles; `tail_len` ENDA, ENDB for exactly TAIL cycles, then IDLEA, IDLEB; `stop_tx` and `stop_rx` the cycle after `sstop` or ABORT the half is stopped; `rx_start` a reception starts only at a commit or the hunt's edge | 5.9.5: "A symbol written in cycle *w* is on the pad from *w* + 1 (section 5.7), for exactly TPER cycles plus any fraction carry. TPER 0 gives one cycle and leaves TXACC alone, whatever TFRAC is. The commit at E writes the first symbol". "the outputs show ENDA and ENDB for TAIL cycles, then IDLEA and IDLEB". 5.9.4: "A stopped send half shows its idle levels on its outputs from the next cycle." 5.9.6: "Without HUNT, a reception begins at the commit." | k-induction, with 7 helper invariants that are proved too | 4 | proved for every TPER, TFRAC and TAIL (16, 8 and 8 bits, no bound) | 1 s |
| **5. Reset: every flop** (`reset_flops.sv`, generated, 473 flop cells, 3569 bits): every flop is an asynchronous-load flop on `rst_n` (the synchroniser) or on the synchronised reset, its value under reset is section 8's (0, or 192 and 128 for each unit's window points), and it holds that value in every reset cycle and in cycle 0 | 8: "Reset is asynchronous, active low, on every flop, and its release passes a synchroniser". The reset table | structural check (`gen_reset.py`) and k-induction | 2 | proved | 4 s |
| **5b. No pad drives until a program asks** (`reset_pads.sv`): `uio_oe` is 0 until a running thread has `oe`, `oe8` or `pcfg PP/OD` on a uio pin in stage 4; every uo pin but uo3 is 0 until one has a word that writes or configures a uo pin. In cycle 0, `uio_oe` and `uo_out` are 0 | 8: "So after reset every `uio` pin is undriven, and every `uo` pin is low." "Nothing runs until the host starts it." | k-induction | 3 | proved | 5 s |
| **6. Netlist against RTL** (`chip.eqy`) | | EQY, SAT induction per piece | to 5 | **partly**: 90 of 109 pieces proved equivalent, 18 unknown, 1 failed. Details below | 1 min |

The real host link is in every proof above (pe_host and pe_spi read from free `ui_in` pins), so they hold for
every host command sequence; the memory's read data is a free input in every cycle (`sram_free.v`), so they
hold for every program.

### Covers

Every assertion's trigger is reached from reset in a cover trace, so none passes vacuously. The cover models
are freer than the chip: the host's commands are free in every cover task, and `covers_deep` frees more
(below). So a trace may reach its trigger sooner than the chip could, but each trigger is one a program and
the host can also produce on the chip, only more slowly.

| Cover task | Model | Covers | Deepest | Time |
|---|---|---|---|---|
| `pads:cover` | whole chip, host commands free | open-drain pins pulled low by OUT (8), lanes in bypass following a moving input while their owner runs (5) | step 14 | 722 s |
| `isolation:cover` | whole chip, host commands free | per pin (15; uo3 has none: no legal word names it and its flops are held at 0): a claim of another thread's pin, another thread's commit with SHD and OUT apart that leaves the pin, the owner's commit moving it, a non-owner's `put` that would change SHD; per unit (4): `scfg`, `txw`, `sgo tx`, `sstop rx` strobes | step 10 | 306 s |
| `shift_timing:cover` | one unit | a start with a word and without, `sstop tx` and ABORT while running, `sstop rx` while receiving, a reception started at a commit and at a hunt, a TPER-3 symbol, a long and a short symbol with TFRAC, a 3-cycle tail | step 6 | 6 s |
| `reset:flops_cover` | whole chip | cycle 0, with the pads as section 8 says | step 3 | 5 s |
| `covers_deep:cover` | reduced (`cut_cover.ys`) | thread 0's open-drain pin on its shift output pulled low (step 17) and released (21), unit 0's LA moving at a start (13) and while running (18), a driven pad released by ABORT (14), a pad driving (13) and a uo pin high (9) once a program asked | step 21 | 709 s |

`covers_deep.sv` holds the covers whose traces are too deep for the whole chip in reasonable time (past
step 17, each step took over 10 minutes there). Its model also frees the register file's and the mailbox's
read data and cuts units 1 to 3. Each trigger there is one the chip can reach with a longer program: a free
register read stands for a value `ldi` and `lui` can load, a free memory read for a program word, and a free
host command for one sent over SPI. The traces themselves have not been replayed on the chip.

## What is cut, and why that is sound

A cut makes a signal a free input in every cycle. That adds behaviours and removes none, so an assertion
proved with the cut holds without it.

- **Memory read data** (`sram_free.v`), in every chip-level job: every program and every loaded value.
- **Host commands** (`cut_host.ys`), in the cover and bmc tasks only: the host's command outputs to the core
  are free, so a trace can start a thread or abort in any cycle. The prove tasks keep the real host.
- **Register file, mailbox, units 1 to 3** (`cut_cover.ys`), in `covers_deep` only.
- **The unit's inputs** (`shift_timing.sv`): one unit with every input free, including combinations the core
  never makes (two strobes in a cycle). `isolation.sv` proves the core side: each unit's CMT is its thread's
  commit and its strobes come only from that thread's instructions.
- **"W names p"** (`isolation.sv`): a word counts as naming a pin when its bits have that row's shape, whether
  or not it runs, so the rule allows a superset of the changes the rows make.

The proofs hold for every state reachable from reset. A k-induction step at depth 1 checks every state the
transition relation can produce, which includes every reachable one.

## Teeth

`teeth.sh` plants one one-line bug at a time in a scratch copy of `src/` and expects the matching check to
fail with a counterexample trace from reset. All 14 are caught; the step is the trace's last. The prove
tasks fail on all 13 sby bugs too (5 in the base case, 8 in the induction step); the bmc tasks give a trace
from reset for each.

| Bug | Change | Check | Fails |
|---|---|---|---|
| `od_drives_high` | `pe_pins.sv`: `uio_out = v[15:8];` | `pads:bmc` | `od_never_high`, `eq_out`, step 8 |
| `bypass_flop` | `pe_core.sv`: pe_pins gets `sync1_q[7:0]` for `ui_in` | `pads:bmc` | `byp_uo`, `eq_uo` (uo4 to uo7), step 13 |
| `put_not_owner` | `pe_pins.sv`: `mine[p] = 1'b1;` | `isolation:bmc` | the SHSEL rule (another thread's `smap` lands), step 9 |
| `commit_any` | `pe_pins.sv`: `moves[p] = \|commit;` | `isolation:bmc` | the SEL rule (another thread's commit moves the switch), step 10 |
| `abort_keeps_sel` | `pe_pins.sv`: ABORT leaves SEL | `isolation:bmc` | `abort_state`, step 13 |
| `unit_stop_shared` | `pe_core.sv`: `.sstop_tx(u_sstop_tx)` | `isolation:bmc` | unit t's `sstop tx` comes from thread t, step 7 |
| `symbol_too_long` | `pe_shift.sv`: `tx_due` on TXCD == 0 | `shift_timing:bmc` | `sym_tper`, `sym_frac`, `inv_cnt`, step 4 |
| `start_without_commit` | `pe_shift.sv`: `if (tx_arm_q)` | `shift_timing:bmc` | `start_cmt`, step 3 |
| `fraction_lost` | `pe_shift.sv`: TXCD without the carry | `shift_timing:bmc` | `sym_frac`, `inv_cnt`, step 5 |
| `sstop_no_idle` | `pe_shift.sv`: `sstop tx` leaves LA | `shift_timing:bmc` | `stop_tx_idle`, step 3 |
| `own_reset` | `pe_pins.sv`: OWN resets to 1 | `reset:flops_prove` | OWN's reset value, step 0 |
| `no_reset_s` | `pe_thread.sv`: S's reset removed | `reset:flops_prove` | S's reset value, step 0 |
| `pcfg_in_drives` | `pe_pins.sv`: `pcfg IN` sets SHOE | `reset:pads_bmc` | `inv_uio`, 5b's invariant, step 8: SHOE is set though no thread asked, before the commit that would drive the pad |
| `netlist_differs` | `pe_pins.sv`: `uio_out = v[15:8] & od_q;` | `chip.eqy` | the uio pads' piece, proved on the real RTL, fails |

## The equivalence check

`chip.eqy` compares `tt_um_sarkar22_pe.nl.v`, the output of the flow's synthesis step (IHP SG13CMOS5L cells,
AREA 0), against the RTL. The cells' functions come from the PDK's liberty. The SRAM macro and its 44 pin
buffers, which the RTL instantiates by name, are black boxes on both sides. EQY matches nets of the same name,
cuts both designs there, and proves each piece between by SAT-based induction. The RTL side is only read,
flattened, split into one net per bit and rid of slang's port buffers (`unbuf.v`).

Proved: 90 of the 109 pieces. Together they hold 772 of the 4110 matched points, 659 of them flop outputs
(of 3536): the decoder, the stage-3 and stage-4 pipeline registers (`a4_q`, `b4_q`, `op4_q`, `ls_addr_q`,
`ls_data_q`, the valid bits), the register file, the SRAM's inputs (enable, write, read, address, data), the
synchroniser's first stage, the mailbox's full flags, and every output pin but `uo3` (`uo_out[7:4]`,
`uo_out[2:0]`, `uio_out`, `uio_oe`).

Not proved: the other 19 pieces, which hold most of the state: the four shift units, the four threads, the
pin flops, the mailbox, the host's registers and `uo_out[3]`. 18 end unknown (no difference in the base case,
but the induction step did not close up to length 5) and `uo_out[3]` fails its base case. What is known to stand
in the way:

- **Flops with no name.** 16 of the netlist's 3562 flops have no name: each unit's three reset-to-1
  window-point bits (`pwq_q[7:6]`, `pwh_q[7]`), built as inverted flops, and 4 in the host. Their RTL flops
  have nothing to match. Naming the 12 in the units (the inverse of each cell's output) did not make the
  units' pieces close, so they are not the only cause there; that cause is not found yet, nor the threads'
  and the pins'.
- **A re-encoded state register.** The synthesis's FSM pass re-encoded the host's command state `cmd_q` from
  binary to one-hot and dropped the reset state's bit, keeping the names: the netlist's `cmd_q[1]` is not the
  RTL's `cmd_q[1]`. Those names are not matched, so the host's pieces hold two encodings of one state, which
  induction can't relate. (EQY's `[recode]` is meant for this; it found no entity to recode here. Re-encoding
  the RTL side with yosys's own FSM passes gave the same 90 proved pieces, and is left out because it would be
  trusted, not proved.)
- **Cut points one side doesn't read.** The synthesis kept some combinational names on nets it then
  recomputes. Matched, such a net becomes a free input that only one side reads, and a piece fails for values
  the chip can't have; `core.fetch`, `core.exec.tb_next`, `core.decode.ra`, `core.decode.rb` and
  `core.host.rd_here` are unmatched for that reason. Matching only names of the form `*_q` instead makes
  one piece of the whole chip (88k cells), whose induction failed: the netlist keeps one name per net, and
  1470 flops carry another one (`chk[0]` for `chklo_q[0]`), so they stayed unmatched.
- **`uo_out[3]`.** The host link's MISO flop is named only by the output pin, so its piece holds the flop on
  both sides with separate free starting values, and the base case fails on those.

What would close it: names on the 16 flops (a synthesis setting that keeps them), `fsm` left out of the
synthesis or its encoding fixed, and a look at the threads' and pins' induction traces. Until then,
gate-level simulation is the check on the logic this one leaves.

## Running

```sh
formal/run.sh                     # every job, about 30 minutes
formal/run.sh isolation:prove     # one job
formal/teeth.sh                   # the planted bugs
```

Each job is `<file>:<task>` for a `.sby` file, or `chip.eqy`. With no tools on PATH, run in the flow's image:

```sh
FORMAL_IMAGE=ghcr.io/librelane/librelane:3.1.0.dev4 formal/run.sh
```

| Variable | Use |
|---|---|
| `FORMAL_IMAGE` | run each job in this container image (it has sby, eqy, yosys with slang and bitwuzla) |
| `FORMAL_DOCKER` | the docker command (default `docker`), for example a wrapper that adds memory and CPU limits |
| `FORMAL_WORK` | where the solvers' work folders go (default `formal/work`; every job together is about 300 MB) |
| `PDK_ROOT`, `FORMAL_NETLIST` | the equivalence check: the PDK and the netlist to compare |
| `FORMAL_JOBS` | parallel pieces in the equivalence check (default 8) |

The image has neither make nor awk, which eqy's strategies use; `bin/make` and `bin/awk` stand in for the
parts eqy needs, and `run.sh` puts them on PATH only when the real ones are missing.

`run.sh reset:gen` regenerates `reset_flops.sv` and `probes_reset.ys` from the RTL and fails if they differ from
the ones here: after a change to the flops, copy the new ones in.

The proofs alone (the prove tasks) take under a minute; the covers take about 25 minutes and the equivalence
check about one. A CI workflow would need the LibreLane image (or an OSS CAD Suite with yosys-slang and
bitwuzla), about 30 minutes for every job or about a minute for the prove tasks, and for the equivalence check
the PDK and a netlist from the synthesis step.

## Files

| File | What |
|---|---|
| `run.sh`, `teeth.sh` | run the jobs, plant the bugs |
| `pads.sv`, `pads.sby` | properties 1, 1b, 2 |
| `isolation.sv`, `isolation.sby` | properties 3, 3b, 3c |
| `shift_timing.sv`, `shift_timing.sby` | property 4 |
| `reset_flops.sv`, `probes_reset.ys`, `gen_reset.py`, `reset_flops.ys` | property 5; the first two are generated |
| `reset_pads.sv`, `reset.sby` | property 5b, and the tasks of property 5 |
| `covers_deep.sv`, `covers_deep.sby`, `cut_cover.ys` | the deep covers |
| `chip.eqy`, `buf_bbox.v`, `unbuf.v` | the equivalence check |
| `read_chip.ys`, `read_shift.ys`, `sram_free.v` | read the design |
| `probes_*.ys` | connect each wrapper's probes to the flops it names, after flattening |
| `cut_host.ys` | the host-command cut |
| `bin/make`, `bin/awk` | stand-ins for eqy, used only where the real tools are missing |
