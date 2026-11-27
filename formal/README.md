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
| **1. Open drain can't drive high** (`pads.sv`, `od_*`): with OD[i], `uio_out[i]` is 0 and `uio_oe[i]` is !V(8+i), in both halves of every cycle, in every state, on the whole chip | 5.3: "In `OD` mode the output data to the pad is forced to 0 and only the enable moves, so the pin can't drive high whatever firmware does." 5.2: "A shift output on a pin in `OD` releases the pin on 1 and pulls it low on 0, and the pin still can't drive high." | k-induction | 1 | proved | 4 s |
| **1b. The pad equations** (`pads.sv`, `eq_*`): `uo_out`, `uio_out` and `uio_oe` equal the equations of 5.2 on all 24 pins in both halves of every cycle: in the second half with V(p) formed from OUT, SEL, OWN, LA and LB, in the first with V0(p), where each unit's outputs are OLA and OLB if its TXH is set (a move THALF put at the falling edge). The pads come from each unit's pad flops P and N and the latch behind N (5.2, implementation note), so this proves that path against the names of 5.9.2 | 5.2, the five equations, V0 and V | k-induction | 1 | proved | (same job) |
| **2. Bypass is transparent and immediate** (`pads.sv`, `byp_*`): with BYP[i], `uo_out[i]` equals `ui_in[SRC(i)]` in the same cycle, for a free `ui_in` | 5.8: "Mode `BYP` gives a pad-to-pad path with no flop in it." 3.2: "A lane in bypass has no flop in its path, so it depends on `ui_in` too." | k-induction | 1 | proved | (same job) |
| **3. Thread pin isolation** (`isolation.sv`, 114 rules): each pin flop (OUT, SHD, OE, SHOE, OD, BYP, SHBYP, OWN, SEL, SHSEL) changes only in a cycle with one of its causes: its owner's instruction naming it, its owner's commit, a `pcfg` of it (and then OWN becomes the claimer), or ABORT. The rule is stated in full in `isolation.sv` | 5.3: "A thread's writes to pins it doesn't own are dropped, and its commit moves only pins it owns. `pcfg` takes a pin with no check, even from a running thread". 5.4: `commit()`, `shadow()`. 9.3: `abort_pins()` | k-induction | 3 | proved | 5 s |
| **3b. Units are their own thread's** (`isolation.sv`): LA, LB of unit t change only while its send half runs or is in its tail, at thread t's commit, at thread t's `sstop tx`, or at ABORT; every step-3 strobe of unit t comes from thread t's instruction of that row; unit t's CMT is thread t's commit | 5.9.1: "A unit starts only at a commit of its own thread". "Units share nothing with each other, so one thread's unit can't change another thread's timing." | k-induction | 3 | proved | (same job) |
| **3c. ABORT releases every uio pad** (`isolation.sv`, `abort_*`): from the next cycle `uio_oe` is 0, OE, SHOE and the uio SEL, SHSEL are 0, OUT and SHD are 1 on open-drain pins | 9.3: "So every `uio` pad is undriven" | k-induction | 3 | proved | (same job) |
| **4. The unit's timing** (`shift_timing.sv`, one unit, every input free in every cycle): `start_cmt` TXRUN rises only after TXARM and CMT; `start_sym` in the commit's cycle `tx_symbol()` runs and from the next cycle LA, LB show symbol 0 of the first bit (MAPA, MAPB, TGLA, TGLB, TMSB, LFSR) and TXCD is TPER; `start_empty` a start with no word is an underrun; `sym_tper` with TFRAC 0 each symbol lasts exactly TPER cycles (TPER 0 as 1); `sym_frac` with any TFRAC it lasts TPER plus the carry of TXACC + TFRAC; `hold` LA, LB move only at those cycles; `tail_len` ENDA, ENDB for exactly TAIL cycles, then IDLEA, IDLEB; `stop_tx` and `stop_rx` the cycle after `sstop` or ABORT the half is stopped; `rx_start` a reception starts only at a commit or the hunt's edge; `half_pads` the unit's pads in both halves of every cycle: LA and LB in the second half, and in the first half too, except after a move (a symbol, the end of a send, the end of a tail) with THALF, TPER 1 or more, TEXT 0 and bit 7 of TXACC as `tx_half()` reads it (0 at a start), and no `sstop tx` or ABORT, where the first half shows the levels from before the move; formed from LA, LB, TXACC and the slots, not from TXH; and 0 in reset and in cycle 0. With `sym_frac` and the TXACC invariant it gives the half-cycle grid: symbol *k* reaches the pad at E + 1 + *k* x (TPER + TFRAC / 256), rounded down to a half cycle | 5.9.5: "A symbol written in cycle *w* is on the pad from *w* + 1 (section 5.7), for exactly TPER cycles plus any fraction carry. TPER 0 gives one cycle and leaves TXACC alone, whatever TFRAC is. The commit at E writes the first symbol". "the outputs show ENDA and ENDB for TAIL cycles, then IDLEA and IDLEB". 5.9.4: "A stopped send half shows its idle levels on its outputs from the next cycle." 5.9.6: "Without HUNT, a reception begins at the commit." 5.9.5, Half cycles: "a symbol, a tail or an idle level that starts while it is 128 or more sets TXH in `tx_half`, so the pads show the new levels from the falling edge of the cycle after the write". "`sstop tx` and ABORT move the outputs at a rising edge." | k-induction, with 7 helper invariants that are proved too | 4 | proved for every TPER, TFRAC and TAIL (16, 8 and 8 bits, no bound), THALF on or off | 1 s |
| **5. Reset: every flop and latch** (`reset_flops.sv`, generated, 543 flop cells, 4053 bits, and 24 latch cells): every flop is an asynchronous-load flop on `rst_n` (the synchroniser) or on the synchronised reset, its value under reset is section 8's (0, or 192 and 128 for each unit's window points: 5.9.6, implementation note; RKIND 3's own *p* and window points are loaded when a reception starts, so they reset to 0, as RXRG and RXRA do), and it holds that value in every reset cycle and in cycle 0. Every latch is gated by `clk`; each `sg13cmos5l_dllrq_1` (16 behind the input pads' falling-edge samples, 8 behind the units' N flops) is cleared by the synchronised reset and shows 0 in both halves of every reset cycle, and the 8 behind N in cycle 0 too; each `sg13cmos5l_dlhq_1` has no reset and drives only the D input of one of those, so a pad's level from before reset can't reach SYNC1F | 8: "Reset is asynchronous, active low, on every flop, and its release passes a synchroniser". The reset table. 5.2, implementation note: "at reset P, N and SYNC1F are 0, and the output latch is cleared with them" | structural check (`gen_reset.py`) and k-induction | 2 | proved | 12 s |
| **5b. No pad drives until a program asks** (`reset_pads.sv`): `uio_oe` is 0 until a running thread has `oe`, `oe8` or `pcfg PP/OD` on a uio pin in stage 4; every uo pin but uo3 is 0 until one has a word that writes or configures a uo pin. In cycle 0, `uio_oe` and `uo_out` are 0. Each output latch shows either half, freely | 8: "So after reset every `uio` pin is undriven, and every `uo` pin is low, in both halves of the cycle." "Nothing runs until the host starts it." | k-induction | 3 | proved | 5 s |
| **6. Netlist against RTL** (`chip.eqy`): the gate netlist's 4733 compared points (the 24 output pins, flop outputs, the inputs of the SRAM and of the half-cycle latches) equal the RTL's, in every state reachable from reset, for the synthesis output and for the final netlist after layout | | EQY: both designs cut at the matched nets into 125 pieces; k-induction on each, PDR on the 6 that hold the host's command FSM | 3 | **proved: 125 of 125 pieces**, on both netlists of the build of `04f143e`, whose `src/` is `ca6a4b9`'s, with the 40 latch cells boxed on both sides as the SRAM pin buffers are. Details below | 209 s, 145 s |
| **7. RKIND 3's datapath is its code** (`rk3_equiv.sv`): the block `rk3n_f` that `src/pe_shift.sv` instantiates for RKIND 3, read from that file, gives every output of the code of 5.9.6's `rx_manch3` (`rk3_code.sv`, that code transcribed in wide arithmetic): whether a bit is given and its value, whether the window closes empty, whether the reception still runs, RXPH's next value, RXOPEN, RXCAND, RXWIN, RXG and RXCE, in every state that satisfies J' (7b), with `rx_bit`'s end free, SMODE 2's end at a stuff violation included | 5.9.6, `rx_manch3`, and its implementation note: "the arithmetic is 15 bits and RXCE 13" | k-induction (the check is combinational) | 1 | proved | 12 s |
| **7b. J' holds on the code** (`rk3_inv.sv`): from the start of a reception, every state the code of 5.9.6 reaches satisfies J': RXG 1 to 16; RXCAND only with RXOPEN; RXPH at least 256 - *p* with the window open and at least 128 - *p* with it closed, below *p* + 128 with RGAIN 1 to 3 and below 7*p*/4 - 128 with RGAIN 0; with RXCAND, RXCE from 256 - *p* to -1. For every *p* from 512 to 4095, every RGAIN, RACQ and RDIFF, any pin levels at both edges and any end of `rx_bit`; a reception that ends starts again | 5.9.6, the Period bullet | k-induction | 8 | proved | 3 s |
| **7c. J' holds on the unit** (`rk3_unit.sv`): `src/pe_shift.sv` with every input free in every cycle, so any `scfg` in any cycle, RXPER and RXOPT written in mid-reception included: in every cycle of a reception of RKIND 3, J' with the reception's own *p* and RGAIN (RXP3, RXRG), and RKIND 3's window points equal to RXP3 - RXP3/4 and RXP3 - RXP3/2. No assumption but reset in the first step: a reception keeps the *p*, RGAIN and RACQ it started with, so no write of RXPER or RXOPT can take it out of J'. And a reception takes RXP3, RXRG and RXRA from the slots as the cycle it starts in reads them, before that cycle's own `scfg` (3.2) | 5.9.6, Start: "with RKIND 3 the *p*, RGAIN and RACQ of that cycle too (RXP3, RXRG and RXRA)"; section 10, defined 8 | k-induction | 4 | proved | 2 s |

Properties 7 to 7c together say that in every state the unit can reach, RKIND 3's datapath computes what the code
of 5.9.6 says: 7b that the ranges of the Period bullet are the code's own, so its 15-bit arithmetic is wide enough,
7c that the unit stays inside them whatever firmware writes, and 7 that inside them the block equals the code.

The real host link is in every whole-chip proof above (pe_host and pe_spi read from free `ui_in` pins), so they hold for
every host command sequence; the memory's read data is a free input in every cycle (`sram_free.v`), so they
hold for every program.

### Covers

Every assertion's trigger is reached from reset in a cover trace, so none passes vacuously. The cover models
are freer than the chip: the host's commands are free in every cover task, and `covers_deep` frees more
(below). So a trace may reach its trigger sooner than the chip could, but each trigger is one a program and
the host can also produce on the chip, only more slowly.

| Cover task | Model | Covers | Deepest | Time |
|---|---|---|---|---|
| `pads:cover` | whole chip, host commands free | open-drain pins pulled low by OUT (8), lanes in bypass following a moving input while their owner runs (5) | step 14 | 914 s (one CPU) |
| `isolation:cover` | whole chip, host commands free | per pin (15; uo3 has none: no legal word names it and its flops are held at 0): a claim of another thread's pin, another thread's commit with SHD and OUT apart that leaves the pin, the owner's commit moving it, a non-owner's `put` that would change SHD; per unit (4): `scfg`, `txw`, `sgo tx`, `sstop rx` strobes | step 10 | 306 s |
| `shift_timing:cover` | one unit | a start with a word and without, `sstop tx` and ABORT while running, `sstop rx` while receiving, a reception started at a commit and at a hunt, a TPER-3 symbol, a long and a short symbol with TFRAC, a 3-cycle tail; a move THALF puts at the falling edge, by a symbol (with its first half showing the old level), by the end of a send and by the end of a tail, and the second half of such a move showing the new level | step 6 | 12 s |
| `reset:flops_cover` | whole chip | cycle 0, with the pads as section 8 says | step 3 | 6 s |
| `rk3_inv:cover` | the code of 5.9.6, as in 7b | RXPH below 0 with the window closed (step 2), and below -*p*/2 with it open (step 5) | step 5 | 1 s |
| `rk3_unit:cover` | one unit, every input free, as in 7c | from reset, RXPH below -*p*/2 with the window open (step 9), and at *p* + 128 or more with it closed, which RGAIN 0 allows (step 12); a reception that starts in the cycle of a write of RXPER or RXOPT | step 12 | 39 s |
| `covers_deep:cover` | reduced (`cut_cover.ys`) | thread 0's open-drain pin on its shift output pulled low (step 17) and released (21), unit 0's LA moving at a start (13) and while running (18), a driven pad released by ABORT (14), a pad driving (13) and a uo pin high (9) once a program asked | step 21 | 709 s |

On the RTL with the half-cycle latches every cover task was run again and reaches every cover: `pads:cover`
914 s, `isolation:cover` 365 s and `covers_deep:cover` 1181 s, each on one CPU, `shift_timing:cover` 12 s and
`reset:flops_cover` 6 s. With RKIND 3's decoder of 2026-11-21 and the Manchester byte end, every prove task
passes again (1 to 6 s each) and every cover task reaches every cover: `pads:cover` 1012 s, `isolation:cover`
339 s, `covers_deep:cover` 1344 s, `shift_timing:cover` 12 s and `reset:flops_cover` 7 s. With RKIND 3's keep test,
structural rule and held *p*, RGAIN and RACQ, timed samples' RESYNC with RACQ and SMODE 2's end at the violating bit
(5.9.6, 5.9.7), every prove task passes again (2 to 12 s each) and every cover task reaches every cover, each on one
CPU from a fresh `FORMAL_WORK`: `pads:cover` 1031 s, `isolation:cover` 393 s, `covers_deep:cover` 1391 s,
`shift_timing:cover` 15 s, `reset:flops_cover` 12 s, `rk3_inv:cover` 1 s and `rk3_unit:cover` 39 s.

`covers_deep.sv` holds the covers whose traces are too deep for the whole chip in reasonable time (past
step 17, each step took over 10 minutes there). Its model also frees the register file's and the mailbox's
read data and cuts units 1 to 3. Each trigger there is one the chip can reach with a longer program: a free
register read stands for a value `ldi` and `lui` can load, a free memory read for a program word, and a free
host command for one sent over SPI. The traces themselves have not been replayed on the chip.

## The latches: one step per cycle, two halves

Each proof takes one step per clock cycle, with every flop on one global clock, so the clock net carries no level
(the flow's `formalff -clk2ff` marks it replaced by that clock, and it reads as a constant within a step). The
latches of section 5.2's implementation note still give the pads a level in each half of the cycle, so
`cells_formal.v` models them for this step:

- `sg13cmos5l_dllrq_1` (open while the clock is low, cleared by RESET_B) shows, in each step, one of its two halves:
  in the first half the value it held from the rising edge, D's value in the step before; in the second half D. A
  free bit `first` in each latch picks the half. `pads.sv` and `shift_timing.sv` tie every output latch's `first` to
  one free input, `half1`, so each of their assertions holds in both halves; the other proofs leave each latch's
  choice free, which only adds behaviours. While RESET_B is low it shows and holds 0.
- `sg13cmos5l_dlhq_1` (open while the clock is high) holds the pad's level at the falling edge. That level is not
  tied to the pad's levels at the rising edges (section 10, item 2), so its output is free in every step, and so is
  SYNC1F. That is sound for every property here; the RKIND 3 decoder that reads SYNC2F is checked against sim.py
  every cycle by `verif/lockstep.py`.

Two pitfalls of this setup, both caught while writing it: a latch model that took its half from the clock net
proved the old one-level pad equations, since that net is a constant in the step (a seven-line test of the model,
Q == D on a toggling D, passed too); and an input named `first` in `shift_timing.sv` silently merged with the
wrapper's existing wire of that name, so the first half was checked only at a start until a cover stayed
unreached. Each harness's half input is now `half1`, and every new assertion has a cover and a planted bug.

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
fail with a counterexample trace from reset. On the RTL with the half-cycle latches, all 21 bugs of the proofs
are caught (8 of them new: 5 for the half cycles and the output latches' reset, and 3 that only the structural
check of `gen_reset.py` can see, since the clock net carries no level in a proof); the step is the trace's last. The prove tasks
failed on the first 13 too (5 in the base case, 8 in the induction step); the bmc tasks give a trace from reset
for each. `netlist_differs` is the equivalence check's: on the final netlist of the build of 2026-11-21, with RKIND 3's
decoder of that day, exactly the uio pads' piece fails and the other 124 are proved. On the RTL with
RKIND 3's decoder of 2026-11-21 and the Manchester byte end, all 24 bugs of the proofs are caught, the 3 new
ones being the reset values of the state that decoder adds: RKIND 3's *p*, RXCE and the half of RXC8 that the
Manchester byte end added. On the RTL with RKIND 3's keep test, structural rule and held *p*, RGAIN and RACQ, the
32 bugs of the proofs other than `netlist_differs` (which needs a netlist) are all caught. The 9 new ones: two reset values
(RXP3 and RXRG; RXP3 now resets to 0, so the old bug that reset it to 0 is gone); four in RKIND 3's block, which
`rk3_equiv` catches with a state of J' (7b, 7c) in which the block and the code differ, not a trace from reset; a
reception that doesn't load RXP3, and one that starts in the cycle of a write of RXOPT and takes the written RFRAC,
which `rk3_unit` catches in its base case, a trace from reset; and a sign error in the code's pull, which `rk3_inv`
catches in its base case. On that RTL's final netlist `netlist_differs` is caught
too: exactly the uio pads' piece fails and the other 124 are proved.
The equivalence check's own teeth, bugs in the netlist, are under "The equivalence check".

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
| `half_on_bit6` | `pe_shift.sv`: `tx_half()` reads bit 6 of TXACC | `shift_timing:bmc` | `half_pads_first`, step 5 |
| `half_not_stopped` | `pe_shift.sv`: `sstop tx` leaves TXH, so the outputs move at a falling edge | `shift_timing:bmc` | `half_pads_first`, step 5 |
| `n_never_moves` | `pe_shift.sv`: the N flop never moves | `shift_timing:bmc` | `half_pads_second`, step 5; `pads:prove` fails too, in its induction step (a THALF move at the chip's pads takes a longer program than `pads:bmc` reaches) |
| `p_without_move` | `pe_shift.sv`: P left without the half-cycle move | `shift_timing:bmc` | `half_pads_second`, step 5; `pads:prove` fails too |
| `latch_no_reset` | `pe_shift.sv`: output A's latch is not cleared by reset | `shift_timing:bmc` | `half_pads_reset`, step 0 |
| `latch_gate_inverted` | `pe_shift.sv`: output A's latch open while the clock is high, so its halves swap | `reset:gen` | the structural check: GATE_N is not `clk` |
| `pair_no_reset` | `pe_core.sv`: the falling-edge sample's second latch not cleared by reset | `reset:gen` | the structural check: RESET_B is not the synchronised reset |
| `sync1f_first_latch` | `pe_core.sv`: SYNC1F reads the first latch, which opens at the very rising edge SYNC1F samples on | `reset:gen` | the structural check: the open-high latch reaches a flop |
| `p3_reset_512` | `pe_shift.sv`: RKIND 3's *p* (RXP3) resets to 512, though a reception loads it at its start | `reset:gen` | the structural check: `p3_q` resets to 512, section 8 says 0 |
| `rg3_reset_1` | `pe_shift.sv`: RXRG resets to 1 | `reset:gen` | the structural check: `rg3_q` resets to 1, section 8 says 0 |
| `rk3_f1_removed` | `pe_shift.sv`, `rk3n_f`: before the window the structural rule fires from RXPH 0, not only within *p*/8 of the window's opening | `rk3_equiv:prove` | the bit, whether the reception runs, and RXPH's next value |
| `rk3_keep_bound` | `rk3n_f`: the keep test's bound *p*/8 at RPER 2 too, not 128 | `rk3_equiv:prove` | whether the reception runs, the bit's value, RXPH's next value |
| `rk3_two_edge_pull` | `rk3n_f`: a cycle with an edge in each half pulls | `rk3_equiv:prove` | RXPH's next value |
| `rk3_smode2_end` | `rk3n_f`: with SMODE 2 a first-half bit that is a stuff violation doesn't end the reception, so the second half step runs | `rk3_equiv:prove` | whether the reception runs, RXPH's next value, RXWIN |
| `rk3_p3_not_loaded` | `pe_shift.sv`: a reception doesn't load RXP3 | `rk3_unit:prove` | *p* at least 512 and the window points against RXP3, in the base case at step 3 |
| `rk3_p3_scfg_start` | `pe_shift.sv`: a reception that starts in the cycle of a write of RXOPT takes the written RFRAC | `rk3_unit:prove` | RXP3 against the slots the start cycle read, in the base case |
| `rk3_code_pull_sign` | `rk3_code.sv`: the code's pull moves RXPH away from the edge | `rk3_inv:prove` | J''s RXPH range, in the base case at step 3 |
| `rxce_reset_1` | `pe_shift.sv`: RXCE resets to -1 | `reset:gen` | the structural check: `rx_ce_q` resets to 8191, section 8 says 0 |
| `rxc8_low_reset` | `pe_shift.sv`: bit 15 of RXC8, in the half the Manchester byte end added, resets to 1 | `reset:gen` | the structural check: `rx_c8_q` resets to 32768, section 8 says 0 |
| `own_reset` | `pe_pins.sv`: OWN resets to 1 | `reset:flops_prove` | OWN's reset value, step 0 |
| `no_reset_s` | `pe_thread.sv`: S's reset removed | `reset:flops_prove` | S's reset value, step 0 |
| `pcfg_in_drives` | `pe_pins.sv`: `pcfg IN` sets SHOE | `reset:pads_bmc` | `inv_uio`, 5b's invariant, step 8: SHOE is set though no thread asked, before the commit that would drive the pad |
| `netlist_differs` | `pe_pins.sv`: `uio_out = v[15:8] & od_q;` | `chip.eqy` | the uio pads' piece, proved on the real RTL, fails |

## The equivalence check

`chip.eqy` compares a gate netlist with the RTL, either the flow's synthesis output (the synthesis step's
`tt_um_sarkar22_pe.nl.v`, IHP SG13CMOS5L cells, AREA 0) or the final netlist after placement and routing
(`final/nl/tt_um_sarkar22_pe.nl.v`, the one submitted); one file serves both. The cells' functions come from the
PDK's liberty (typical corner), and the layout's repair buffers, hold cells and clock buffers are cells like the
others, each X = A. The SRAM macro, its 44 pin buffers and the 40 half-cycle latches (16 `sg13cmos5l_dlhq_1` and 24
`sg13cmos5l_dllrq_1`, section 5.2), which the RTL instantiates by name, are black boxes on both sides: their inputs
are compared, and their outputs are inputs both sides share. The RTL side is only read,
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
| synthesis output | **125 of 125** | 119 | 6: the host, the mailbox, the four threads | 209 s (3 proofs at a time) |
| final netlist, after layout | **125 of 125** | 119 | the same 6 | 145 s (4 at a time) |

The two runs match the same 4932 names into the same 125 pieces and compare the same 4733 points: the layout
changed no name the check uses. Sixteen of the pieces are the falling-edge latches' inputs, one per pad
(`core.ui_in.0` to `core.uio_in.7`), and the units' N flops are compared as their output latches' inputs. The six
PDR pieces are the ones that hold the host's command FSM (below).

What it takes, besides EQY's default matching by name. Each rule is in `chip.eqy` with how it was derived from
the netlist:

| Rule | Why | Pieces |
|---|---|---|
| **Inverted flops**: for each unit's three reset-to-1 window-point bits (`pwq_q[7:6]`, `pwh_q[7]`; section 8: 192 and 128), `inv_flops.v` puts two inverters after the flop cell and gives the net between them the RTL's name | the library has only reset-to-0 flops, so the synthesis keeps these bits inverted, in flops with no name. Each cell sits in its register's numbering (`_62887_` is `pwh_q[6]`, `_62889_` `pwh_q[8]`), and the mux of `pw = wide ? pwh_q : pwq_q` takes the two bit-7 cells on its two inputs. RKIND 3's own *p* and window points, and RXRG and RXRA, are loaded when a reception starts and reset to 0 (section 8), so they need no rule. Naming only the inverse of the cell's output would cut the RTL side alone: the netlist's loads read the cell itself | the four units |
| **Constant flops**: 12 pin flop bits (`sel_q[7:6]`, `shsel_q[7:6]`, `byp_q[3:1]`, `shbyp_q[3:1]`, `shd_q[3]`, `out_q[3]`) are matched to netlist nets tied to 0 | the synthesis found them constant and removed them (its log: "Setting constant 0-bit at position ..."); no legal word sets them. The RTL's flop is compared with 0 in its piece | the pin flops |
| **A merged flop**: the SPI's `sck_q` is matched to unit 0's `sync3[1]` | `sck_q <= sck` with `sck = sync2_q[1]` (pe_spi.sv), and `sync3_q <= sync2_q` (pe_core.sv): the same input, the same reset; the netlist's only flop with D = `sync2[1]` serves both. Both RTL flops are compared with it | the host's argument bytes, the host |
| **`core.host.miso` not matched** | in the RTL it is an alias of `spi.tx_q[7]` ahead of `uo_out[3]`; in the netlist the flop drives `uo_out[3]` and the name sits on a buffer after it, so matched, it would cut the two sides on either side of the flop | `uo_out[3]`, now in the host's piece |
| **The command FSM left unmatched, and PDR** | the synthesis's FSM pass re-encoded `cmd_q` (binary, 11 states) one-hot, ten bits with the reset state all zero, beside four unnamed flops that hold the binary code (each reads exactly the one-hot bits of the states with its bit set). The one-hot bits keep the RTL's names, so neither encoding matches. Each piece that reads the state holds both encodings, and k-induction can't relate them; PDR does | the host, the mailbox, the four threads |
| **Merges kept apart**: the host's `reg_q`, which the netlist names after unit 0's port `rd_reg`, is merged with the host, and the SCK flop with unit 0, by the merge patterns themselves; the units' latch boxes (`lat_na`, `lat_nb`) are kept out of the units' merges too | either would join unit 0's piece with the host's into one too large for PDR (this EQY's `nomerge` has no effect); a merge that names a box cell crashes EQY's partitioning | unit 0, the host |
| **Final netlist**: the 44 SRAM pin buffers boxed by instance name, every other buffer its liberty function; the names flattening and the layout add (`hold5290.X`, `net5288`, `clknet_*`) made private | the RTL has only `core.*` names and ports, and with the buffers X = A each net keeps its synthesis name | all, on the final netlist |
| **Latch boxes**: each latch cell becomes a box of the same instance name on both sides (`latch_box.v`, by the techmap rule `latch_map.v`), with its D (and RESET_B) and Q but no gate pin | the RTL instantiates these cells by name (pe_core.sv, pe_shift.sv) and the netlist keeps the 40 instances under the same names. EQY puts every flop on one global clock and has no clock to give a box: a box input driven by `clk` makes a piece with no logic, which EQY rejects (why the SRAM's `A_CLK` is tied off) | the pads' latch inputs, the units, the pads |
| **Flops under their copies' names**, no rule needed: RXPH's flops for bits 0 to 16 carry `ph` (`ph = {rx_ph_q[16], rx_ph_q[16:0]}`, pe_shift.sv), and RKIND 3's *p* and window points carry `p3`, `pq3` and `ph3w`, every bit | the synthesis kept the copies' names on those flops instead of the registers'. The RTL has both names on the same nets, so the two sides match there by EQY's default rule, flop to flop | the four units |
| Unmatched, as before: `core.fetch`, `core.exec.tb_next`, `core.decode.ra`, `core.decode.rb`, `core.host.rd_here`, the clock's port wires | the synthesis kept these names on nets it computes again from their inputs; matched, they would give one side a free value the chip can't have | |

A constant flop that another piece reads must not be matched to a constant: EQY compares a constant-matched bit
only in the piece that drives it, while in a piece that reads it the RTL gets a free value and the netlist its 0.
The 12 pin bits are read by no other piece. (Bit 15 of each unit's TXMODE, once removed as constant this way, is
THALF now, a real flop matched like the others.)

EQY's own `sat` strategy is not used. With this yosys (0.69), its base case (`sat -tempinduct -set-init-undef
-set-def-formal ...`) has no model at all on a piece that has flops on both sides: `-set-init-undef` with
`-set-def-formal` is unsatisfiable there, so the base case passes without checking anything, and the strategy
proved two pieces with planted bugs, among them an inverted `uo_out[3]` buffer. This check's earlier result, "90 of
109 pieces proved", came from that strategy, so those were not proofs. SBY's k-induction found both bugs in its
base case.

### What the check assumes

- **One clock.** EQY puts every flop on one global clock and never looks at the clock nets. In both netlists
  every flop's clock pin, every latch's gate pin and the SRAM's clock pin reach the `clk` pin through buffers
  only, with no inverter (6 for every flop and latch in the final netlist, 9 for the SRAM), checked from the
  netlist text, not by EQY; so do each latch pair's wiring (a pad's `dlhq_1` drives only its pair's `dllrq_1`) and
  each latch's reset. So the SRAM's `A_CLK`, tied off on both sides, and the latch boxes, which have no gate pin,
  stand for real connections to `clk`.
- **Black boxes.** The SRAM macro, its 44 pin buffers and the 40 latches: their inputs are proved equal, their
  behaviour is not part of this check (the RTL instantiates the same cells; the latches' halves are properties
  1b, 4 and 5, and the gate-level test runs the library's own latch models).
- **Cell functions.** Each cell behaves as its liberty function says; the layout against the netlist is LVS's
  check, not this one.
- **Starting state.** EQY's, as above; with every flop reset (property 5) it covers every run from reset.

### Teeth

Bugs planted in scratch copies of a netlist (`plant_netlist.py`; the runs here put several in one copy, in pieces
that share no logic, and `teeth.sh` plants them one at a time). Each must make the piece it is in fail, with a
counterexample.

| Bug | Netlist | Technique it tests | Piece that fails |
|---|---|---|---|
| `swap_reset:_63475_`: unit 1's `pwq_q[7]` resets to the wrong value | synthesis | inverted flops | unit1 |
| `swap_reset:_62888_`: unit 0's `pwh_q[7]` the same | synthesis | inverted flops, the merges kept apart | unit0 |
| `invert_d:_62751_`: `byp_q[0]` loads the inverse | synthesis | constant flops | pins |
| `drop_flop:_62750_`: the real flop `byp_q[4]` removed and its net tied to 0, as the synthesis does to the constant ones | synthesis | constant flops (a wrong one is caught) | pins; and `uo_out.4`, which reads it |
| `hold_d:_65263_`: `hm_wdata[9]` never loads | synthesis | the merged SCK flop | `core.hm_wdata.9` |
| `buf_to_inv:_66177_`: `uo_out[3]` through an inverter | synthesis | `core.host.miso` unmatched | host |
| `rewire_d:_65355_:_65361_`: the FSM's one-hot RUN bit takes SETPC's next state | synthesis | the FSM unmatched, PDR | thread0 to thread3 |
| `invert_d:_66107_`: a bit of the FSM's unnamed binary copy loads the inverse | synthesis | the FSM unmatched, PDR | host |
| `invert_pin:_43201_:B`: unit 0's output A reads its latch inverted | synthesis | the latch boxes' outputs | every pad piece unit 0's output A reaches (`uo_out.0` and 8 more) |
| `buf_to_inv:fanout3948`: a layout fanout buffer on unit 2's `tx_stf_q` inverts | final | the final netlist's buffers | unit2 |
| `buf_to_inv:hold6349`: a hold cell on the host's `addr_q[7]` inverts | final | the final netlist's buffers | host; `core.mem.addr.0` |
| `buf_to_inv:wire6263`: the repeater in front of `ui_in[3]`'s falling-edge latch inverts | final | the latch boxes' inputs | `core.ui_in.3`; `uo_out.0` (the repeater feeds the rising-edge sample too) |

In each run every other piece still passes. Three more bugs put an inverter straight at a latch box's pin, where the
real netlist has only a buffer or a wire: the hold cell between a pad's two latches, the hold cell in front of an
output latch, and an inverter on an output latch's RESET_B. EQY pairs a box's pins with the RTL's by position, so each
gives a pin net that is no longer the RTL's net under another name, and EQY stops before proving anything
("conflicting matches for gold bit ..."): an error, not a pass. The same holds for a netlist whose pad logic skips
an output latch (EQY: the RTL piece's latch output "is used but has no driver"). These are not in `teeth.sh`, which
counts only a failing piece.

## Running

```sh
formal/run.sh                     # every job, about 51 minutes
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

The proofs alone (the prove tasks) take under a minute; the covers take about 48 minutes and the equivalence
check two to four minutes with 3 or 4 jobs. A CI workflow would need the LibreLane image (or an OSS CAD Suite with
yosys-slang and bitwuzla), about 51 minutes for every job or about a minute for the prove tasks, and for the
equivalence check the PDK and a netlist from the flow. An EQY run writes a folder per piece, about 3000 files and
60 MB in all, and `run.sh` replaces an earlier `chip_eqy` folder in `FORMAL_WORK`. `teeth.sh` with `FORMAL_NETLIST`
set also runs the netlist bugs, one EQY run each.

## Files

| File | What |
|---|---|
| `run.sh`, `teeth.sh` | run the jobs, plant the bugs (in the RTL, or in the netlist) |
| `pads.sv`, `pads.sby`, `probes_half.ys` | properties 1, 1b, 2; the probes of each unit's TXH, OLA, OLB and its output latches' half |
| `isolation.sv`, `isolation.sby` | properties 3, 3b, 3c |
| `shift_timing.sv`, `shift_timing.sby` | property 4 |
| `reset_flops.sv`, `probes_reset.ys`, `gen_reset.py`, `reset_flops.ys` | property 5; the first two are generated |
| `reset_pads.sv`, `reset.sby` | property 5b, and the tasks of property 5 |
| `rk3_equiv.sv`, `rk3_equiv.sby`, `rk3_code.sv` | property 7; the code of 5.9.6's `rx_manch3`, transcribed |
| `rk3_inv.sv`, `rk3_inv.sby` | property 7b |
| `rk3_unit.sv`, `rk3_unit.sby`, `probes_rk3.ys` | property 7c |
| `covers_deep.sv`, `covers_deep.sby`, `cut_cover.ys` | the deep covers |
| `chip.eqy`, `buf_bbox.v`, `pinbuf_box.v`, `latch_box.v`, `latch_map.v`, `inv_flops.v`, `unbuf.v` | the equivalence check |
| `plant_netlist.py` | plant a bug in a scratch copy of a netlist, for `teeth.sh` |
| `read_chip.ys`, `read_shift.ys`, `sram_free.v` | read the design |
| `cells_formal.v` | the library cells the RTL instantiates (the SRAM pin buffer and the two latches), for a step of one cycle |
| `probes_*.ys` | connect each wrapper's probes to the flops it names, after flattening |
| `cut_host.ys` | the host-command cut |
| `bin/make`, `bin/awk` | stand-ins for eqy, used only where the real tools are missing |
