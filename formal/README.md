# Formal proofs

Proofs of the chip's safety and timing properties with SymbiYosys, and an equivalence check of the gate
netlists, the synthesis output and the final netlist after layout, against the RTL with EQY. Every proof reads
the RTL in `src/` with slang
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
| **6. Netlist against RTL** (`chip.eqy`): the gate netlist's 4134 matched nets (the 24 output pins, flop outputs, the SRAM's inputs) equal the RTL's, in every state reachable from reset, for the synthesis output and for the final netlist after layout | | EQY: both designs cut at the matched nets into 108 pieces; k-induction on each, PDR on the 6 that hold the host's command FSM | 3 | **proved: 108 of 108 pieces**, on both netlists. Details below | 68 s, 75 s |

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
from reset for each. The equivalence check's own teeth, bugs in the netlist, are under "The equivalence check".

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

`chip.eqy` compares a gate netlist with the RTL, either the flow's synthesis output (the synthesis step's
`tt_um_sarkar22_pe.nl.v`, IHP SG13CMOS5L cells, AREA 0) or the final netlist after placement and routing
(`final/nl/tt_um_sarkar22_pe.nl.v`, the one submitted); one file serves both. The cells' functions come from the
PDK's liberty (typical corner), and the layout's repair buffers, hold cells and clock buffers are cells like the
others, each X = A. The SRAM macro and its 44 pin buffers, which the RTL instantiates by name, are black boxes on
both sides: their inputs are compared, and their outputs are inputs both sides share. The RTL side is only read,
flattened, split into one net per bit and rid of slang's port buffers (`unbuf.v`).

EQY matches nets of the same name, cuts both designs at them and proves each piece between. A matched net is an
input of the pieces that read it and a compared output of the piece that drives it, so a wrong match makes a
piece fail; it can't make one pass. Each piece is proved first by k-induction (SBY, solver bitwuzla, depth 3:
a base case from the initial state and an induction step), and where that leaves it open, by PDR (ABC's `pdr`),
which finds its own invariants. In EQY's model the RTL's flops start unknown and the netlist's anywhere, and a
piece is proved when its outputs equal the RTL's wherever the RTL's are known; since every flop resets
(property 5), that covers every run from reset.

| Netlist | Pieces proved | By k-induction | By PDR | Time |
|---|---|---|---|---|
| synthesis output | **108 of 108** | 102 | 6: the host, the mailbox, the four threads | 68 s |
| final netlist, after layout | **108 of 108** | 102 | the same 6 | 75 s |

The two runs match the same 4249 names into the same 108 pieces and compare the same 4134 points: the layout
changed no name the check uses. The six PDR pieces are the ones that hold the host's command FSM (below); PDR takes
17 s on the host's piece and at most 5 s on each of the others.

What it takes, besides EQY's default matching by name. Each rule is in `chip.eqy` with how it was derived from
the netlist:

| Rule | Why | Pieces |
|---|---|---|
| **Inverted flops**: for each unit's three reset-to-1 window-point bits (`pwq_q[7:6]`, `pwh_q[7]`; section 8: 192 and 128), `inv_flops.v` puts two inverters after the flop cell and gives the net between them the RTL's name | the library has only reset-to-0 flops, so the synthesis keeps these bits inverted, in flops with no name. Each cell sits in its register's numbering (`_49293_` is `pwh_q[6]`, `_49295_` `pwh_q[8]`), and the mux of `pw = wide ? pwh_q : pwq_q` takes the two bit-7 cells on its two inputs. Naming only the inverse of the cell's output would cut the RTL side alone: the netlist's loads read the cell itself | the four units |
| **Constant flops**: 12 pin flop bits (`sel_q[7:6]`, `shsel_q[7:6]`, `byp_q[3:1]`, `shbyp_q[3:1]`, `shd_q[3]`, `out_q[3]`) are matched to netlist nets tied to 0 | the synthesis found them constant and removed them (its log: "Setting constant 0-bit at position ..."); no legal word sets them. The RTL's flop is compared with 0 in its piece | the pin flops |
| **A merged flop**: the SPI's `sck_q` is matched to unit 0's `sync3[1]` | `sck_q <= sck` with `sck = sync2_q[1]` (pe_spi.sv), and `sync3_q <= sync2_q` (pe_core.sv): the same input, the same reset; the netlist's only flop with D = `sync2[1]` serves both. Both RTL flops are compared with it | the host's argument bytes, the host |
| **`core.host.miso` not matched** | in the RTL it is an alias of `spi.tx_q[7]` ahead of `uo_out[3]`; in the netlist the flop drives `uo_out[3]` and the name sits on a buffer after it, so matched, it would cut the two sides on either side of the flop | `uo_out[3]`, now in the host's piece |
| **The command FSM left unmatched, and PDR** | the synthesis's FSM pass re-encoded `cmd_q` (binary, 11 states) one-hot, ten bits with the reset state all zero, beside four unnamed flops that hold the binary code (each reads exactly the one-hot bits of the states with its bit set). The one-hot bits keep the RTL's names, so neither encoding matches. Each piece that reads the state holds both encodings, and k-induction can't relate them; PDR does | the host, the mailbox, the four threads |
| **Merges kept apart**: the host's `reg_q`, which the netlist names after unit 0's port `rd_reg`, is merged with the host, and the SCK flop with unit 0, by the merge patterns themselves | either would join unit 0's piece with the host's into one too large for PDR (this EQY's `nomerge` has no effect) | unit 0, the host |
| **Final netlist**: the 44 SRAM pin buffers boxed by instance name, every other buffer its liberty function; the names flattening and the layout add (`hold5290.X`, `net5288`, `clknet_*`) made private | the RTL has only `core.*` names and ports, and with the buffers X = A each net keeps its synthesis name | all, on the final netlist |
| Unmatched, as before: `core.fetch`, `core.exec.tb_next`, `core.decode.ra`, `core.decode.rb`, `core.host.rd_here`, the clock's port wires | the synthesis kept these names on nets it computes again from their inputs; matched, they would give one side a free value the chip can't have | |

Bit 15 of each unit's TXMODE was removed as constant too (KEEP0), but it is not matched to a constant: the
host's RDREG reads it in the host's piece, and EQY compares a constant-matched bit only in the piece that drives
it, while in a piece that reads it the RTL gets a free value and the netlist its 0. PDR found exactly that
counterexample (RDREG of TXMODE with bit 15 at 1 on the RTL side), so the four RTL flops stay in the host's
piece, where PDR shows they stay 0.

EQY's own `sat` strategy is not used. With this yosys (0.69), its base case (`sat -tempinduct -set-init-undef
-set-def-formal ...`) has no model at all on a piece that has flops on both sides: `-set-init-undef` with
`-set-def-formal` is unsatisfiable there, so the base case passes without checking anything, and the strategy
proved two pieces with planted bugs, among them an inverted `uo_out[3]` buffer. This check's earlier result, "90 of
109 pieces proved", came from that strategy, so those were not proofs. SBY's k-induction found both bugs in its
base case.

### What the check assumes

- **One clock.** EQY puts every flop on one global clock and never looks at the clock nets. In both netlists
  every flop's clock pin, and the SRAM's, reaches the `clk` pin through buffers only (6 for every flop in the
  final netlist, 9 for the SRAM), checked from the netlist text, not by EQY. So the SRAM's `A_CLK`, tied off on
  both sides, stands for a real connection to `clk`.
- **Black boxes.** The SRAM macro and its 44 pin buffers: their inputs are proved equal, their behaviour is not
  part of this check (the RTL instantiates the same cells).
- **Cell functions.** Each cell behaves as its liberty function says; the layout against the netlist is LVS's
  check, not this one.
- **Starting state.** EQY's, as above; with every flop reset (property 5) it covers every run from reset.

### Teeth

Bugs planted in scratch copies of a netlist (`plant_netlist.py`; the runs here put several in one copy, in pieces
that share no logic, and `teeth.sh` plants them one at a time). Each must make the piece it is in fail, with a
counterexample.

| Bug | Netlist | Technique it tests | Piece that fails, on | Found by |
|---|---|---|---|---|
| `swap_reset:_49784_`: unit 1's `pwq_q[7]` resets to the wrong value | synthesis | inverted flops | unit1, `pwq_q[7]` | k-induction, base case |
| `swap_reset:_49294_`: unit 0's `pwh_q[7]` the same | synthesis | inverted flops, the merges kept apart | unit0, `pwh_q[7]` | k-induction, base case |
| `invert_d:_49109_`: `byp_q[0]` loads the inverse | synthesis | constant flops | pins, `byp_q[0]` | k-induction, base case |
| `drop_flop:_49108_`: the real flop `byp_q[4]` removed and its net tied to 0, as the synthesis does to the constant ones | synthesis | constant flops (a wrong one is caught) | pins, `byp_q[4]`; and `uo_out.4`, which reads it | k-induction, base case |
| `hold_d:_51293_`: `hm_wdata[9]` never loads | synthesis | the merged SCK flop | `core.hm_wdata.9`, `hm_wdata[9]` | k-induction, base case |
| `buf_to_inv:_52053_`: `uo_out[3]` through an inverter | synthesis | `core.host.miso` unmatched | host, `uo_out[3]` | k-induction, base case |
| `rewire_d:_51385_:_51391_`: the FSM's one-hot RUN bit takes SETPC's next state | synthesis | the FSM unmatched, PDR | thread0 (by PDR), thread1-3, on `run_d`, `d_d` | PDR; k-induction, base case |
| `invert_d:_51983_`: bit 0 of the FSM's unnamed binary copy loads the inverse | synthesis | the FSM unmatched, PDR | host, `spi.tx_q[0]` | PDR |
| `buf_to_inv:fanout3296`: a layout fanout buffer on unit 2's `tx_stf_q` inverts | final | the final netlist's buffers | unit2, `la_d`, `lb_d` | PDR |
| `buf_to_inv:hold5195`: a hold cell on the host's `addr_q[7]` inverts | final | the final netlist's buffers | host, `addr_q[7]`; `core.mem.addr.0`, `sram_addr[7]` | k-induction, base case |

In each run every other piece still passes. The RTL-side tooth `netlist_differs` (above) fails the uio pads' piece.

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
| `PDK_ROOT`, `FORMAL_NETLIST` | the equivalence check: the PDK and the netlist to compare, the synthesis step's `tt_um_sarkar22_pe.nl.v` or the final `final/nl/tt_um_sarkar22_pe.nl.v` |
| `FORMAL_JOBS` | parallel pieces in the equivalence check (default 4) |

The image has neither make nor awk, which eqy's strategies use; `bin/make` and `bin/awk` stand in for the
parts eqy needs, and `run.sh` puts them on PATH only when the real ones are missing.

`run.sh reset:gen` regenerates `reset_flops.sv` and `probes_reset.ys` from the RTL and fails if they differ from
the ones here: after a change to the flops, copy the new ones in.

The proofs alone (the prove tasks) take under a minute; the covers take about 25 minutes and the equivalence
check a minute or two with 4 jobs. A CI workflow would need the LibreLane image (or an OSS CAD Suite with
yosys-slang and bitwuzla), about 30 minutes for every job or about a minute for the prove tasks, and for the
equivalence check the PDK and a netlist from the flow. An EQY run writes a folder per piece, about 3000 files and
60 MB in all, and `run.sh` replaces an earlier `chip_eqy` folder in `FORMAL_WORK`. `teeth.sh` with `FORMAL_NETLIST`
set also runs the netlist bugs, one EQY run each.

## Files

| File | What |
|---|---|
| `run.sh`, `teeth.sh` | run the jobs, plant the bugs (in the RTL, or in the netlist) |
| `pads.sv`, `pads.sby` | properties 1, 1b, 2 |
| `isolation.sv`, `isolation.sby` | properties 3, 3b, 3c |
| `shift_timing.sv`, `shift_timing.sby` | property 4 |
| `reset_flops.sv`, `probes_reset.ys`, `gen_reset.py`, `reset_flops.ys` | property 5; the first two are generated |
| `reset_pads.sv`, `reset.sby` | property 5b, and the tasks of property 5 |
| `covers_deep.sv`, `covers_deep.sby`, `cut_cover.ys` | the deep covers |
| `chip.eqy`, `buf_bbox.v`, `pinbuf_box.v`, `inv_flops.v`, `unbuf.v` | the equivalence check |
| `plant_netlist.py` | plant a bug in a scratch copy of a netlist, for `teeth.sh` |
| `read_chip.ys`, `read_shift.ys`, `sram_free.v` | read the design |
| `probes_*.ys` | connect each wrapper's probes to the flops it names, after flattening |
| `cut_host.ys` | the host-command cut |
| `bin/make`, `bin/awk` | stand-ins for eqy, used only where the real tools are missing |
