# Coverage

Updated 2026-11-11.

Three measures of how much of the chip the tests reach, all from one set of runs:

1. **Lockstep** (`verif/lockstep.py`): the RTL runs beside the cycle simulator `sim/sim.py`, and every state both can
   show is compared after every cycle.
2. **Code coverage** (`verif/vlcov.py`): Verilator's line, branch and toggle coverage of `src/`, under that stimulus
   and every program bench of `programs/check.py`.
3. **Functional coverage** (`verif/fcov.py`): points taken from [ISA.md](ISA.md), counted on the `sim/sim.py` side
   of the lockstep runs.

## 1. Running it

| What | Command | Time, 8 threads |
|---|---|---|
| The Verilator models (Verilator 5 and a C++ compiler) | `make -C verif JOBS=8` | |
| One lockstep run, on those models | `python3 verif/lockstep.py --gen chips --images 200` | 10 s |
| Everything in this page: coverage models, every run of section 2.4, the program benches on the RTL, the reports | `verif/coverage.sh [OUT]` | 20 min |
| The proof of section 3.3, with SymbiYosys and bitwuzla as in `formal/` | `sby -f tx_rl.sby prove` and `cover`, in `verif/unreachable/` | seconds |

`verif/coverage.sh` writes `OUT/code.md` and `OUT/code_uncovered.txt` (code coverage per module, and every point
never hit), `OUT/func.md` and `OUT/func_points.txt` (functional coverage per group, and every point with its
count), and `OUT/merged.dat` for `verilator_coverage --annotate`. `VERILATOR` may name a wrapper that runs
Verilator elsewhere, such as a container, with `VERILATOR_ROOT` naming a copy of its runtime for this machine's
compiler. Every run must pass: the script stops at the first difference or failed check.

A failing image prints its first difference in full, and a command that runs that image alone:

```
FAIL image 5: cycle 1167: SEL[10]: sim.py 2, RTL 0
    rerun: verif/lockstep.py --gen chips --first 5 --images 1 --cycles 6000 --depths 512 --host full
```

(That one is a planted bug of section 2.5: a commit that doesn't copy SHSEL to SEL.)

The bench is Python 3's standard library and the models of `verif/Makefile`.

## 2. The lockstep bench

### 2.1 What it compares

Both chips get the same memory, the same pad inputs every cycle and the same host commands. After every cycle:

1. The pads: `uo_out`, `uio_out` and `uio_oe`, with the bypass lanes on this cycle's `ui_in` (5.8; `sim/sim.py`
   doesn't lag them, section 10).
2. Every thread: PC, R0..R7, T, D, S, LATE, STALE, ILL, RUN, P, WAITING, STOPREQ, STEPPING, MEM, MEM_DONE, MEM_KIND
   and MEM_REG, and while P the event: KIND, DUE, MASK, PAT, BANK, NE, TIMED and INBOX (sections 2, 3.4).
3. The pipeline, by where each thread's memory cycle puts its word (3.1, 4.2): IV and IR in stages 2 to 4, a load's
   data, and a `ld` or `st`'s address and data in its memory cycle. TB, SYNC1, SYNC2 and SYNC3.
4. Every shift unit: each name of 5.9.2 (CMT aside: it lasts one cycle, so between cycles it reads 0) and the 16
   slots of 5.9.3; and the RTL's window points PWQ and PWH against p - p/4 and p - p/2 of the slots as they stand
   (5.9.6, implementation note).
5. The pin layer (5.2 to 5.4, 5.9.2): OUT, SHD, OE, SHOE, OD, BYP, SHBYP, OWN, SEL and SHSEL.
6. M0..M7, FULL and BUSY.
7. All of memory, every 64 cycles and at the end.
8. Every host command's result, as section 9 defines it (below).

### 2.2 Images

| Generator | Images |
|---|---|
| `chips` | 128 words biased to the eight rows of 5.9.4 and to unit settings that do something (`scfg` of a slot value from a small table, `sgo`, `txw`, `rxw`, `sstat`, `srd`, `smap`, `pcfg`), the rest random legal words; threads start at random addresses. The uio pads are looped back, so a pin the chip drives reads its own drive (5.2) and units receive their own and each other's sends |
| `prog` | A program per thread, at 128 × *i* in 512 words: it writes 6 to 16 slots (RPIN on io0 to io3), claims one or two of io0 to io3, and sometimes a uo pin, maps them to output A or B, starts its unit, then loops: bursts of `txw`, `sstat` with `rxw` when RXFULL is set, `srd`, ticks, and now and then `scfg`, `sstop`, `sgo`, `smap`, `pcfg` or a random legal word |
| `rows` | Every word of memory a random legal word of any of the 89 rows of 6.3, and 7% random 16-bit words, so some are illegal (section 10). The inputs hold for 1, 3, 20 or 200 cycles. Every 1500 cycles one thread's D is put half the ring from TB on both chips, so STALE sets (3.5) without waiting 32,768 cycles |
| `wide` | As `prog`, but every slot is written, each field drawn over its whole range (5.9.3): every CRC width, CINIT, CXOR, CMODE, SMODE and SN, TPER and RPER up to 65535, TFRAC, TAIL, every decoder and option. Pins are any of io0 to io7 in IN, PP or OD, uo pins and bypass lanes. One image in four is reset in mid-run (section 8): `rst_n` goes low for 4 edges on the RTL and is released through its synchroniser, and `sim/sim.py` starts again as a new chip with the RTL's memory, which has no reset |
| `directed` | Programs in the assembly language of 6.6 for corners chance doesn't reach, one per seed, with no random host traffic: RXIC saturating at 65535 and starting again at an edge (5.9.6); TPER 65535 with a TFRAC carry (5.9.5), stopped by an ABORT in the very cycle that writes the 65,536-cycle symbol, then started again; and CTX moved in the middle of a CRC append (section 10, defined 17) |

And `--corners`, instead of images: 2,708 scripted cases, each on every thread, with host commands landing in the
cycle the script picks, compared every cycle as above:

| # | Block | Cases | What, and the section it checks |
|---|---|---|---|
| 1 | `stepld` | 4 | A stop during a stepped `ld`, with an event the STEP left pending: `retire()` cancels it (3.6) |
| 2 | `stopslot` | 240 | A stop landing 1 to 5 cycles before a wait's timeout, match or inbox exit, and in its own cycle (3.2) |
| 3 | `stale` | 12 | STALE setting after 34,000 cycles, a stale D replaced with LATE, and each way of writing D clearing it (3.5) |
| 4 | `pcfgod` | 432 | `pcfg` moving pins in and out of OD with OE set, an ABORT landing beside it at 36 offsets (5.3, 9.3) |
| 5 | `ring` | 384 | `wrd` and `tick 0` on and just before TB 0x7FFF, 0xFFFF, 0 and 0x8000, with `now`, `tlate`, `rdtb` and a second tick on the corner (3.4, 7.2) |
| 6 | `stale4` | 32 | STALE is read in stage 4: a stale wait's match at each alignment around the slot of the tick behind it (7.1) |
| 7 | `steplate` | 48 | A RUN in the cycle where a stepped tick on a stale D sets LATE, at 12 offsets (9.3) |
| 8 | `boundary` | 128 | An `edge` and a timed wait with deadlines on both sides of the 32767-cycle limit (3.5; section 10, defined 2) |
| 9 | `nowstale` | 16 | `now` on a stale D leaves LATE alone (3.5) |
| 10 | `world` | 1,204 | Every row of 6.3 behind a tick that is still pending: three words of random legal fields, and for the shifts each combination of a pin in and out; a world row must wait for the tick, any other runs at once (3.6; 6.3, the W column and its pin rule for the shifts); and a word with a value its field's type forbids in each such field, which must stop the thread with ILL (section 10) |
| 11 | `abortpin` | 208 | An ABORT swept over 52 cycles around an `oe8` and a `put8`: a host command acts after step 5 of its cycle, so the ABORT still releases every `uio` pad, and an OD pin's OUT and SHD are 1 after it (9.3, 3.2) |

### 2.3 Host commands

One command at a time, each through the RTL's real command block (`pe_host.sv`), in one of three ways:

| Mode | How a command reaches the RTL | Results compared |
|---|---|---|
| `quick` | Only the byte that acts, in cycle *h* (9.1), with the bytes before it taken as received. A command can follow in the next gap | POST's result, TAKE's flag and word, RDREG's value, RDMEM's word in the reply register |
| `full` | Every byte, one per gap between cycles, through the block's own byte counter, so the command decodes and its arguments latch as on silicon. The first byte of a transaction resets the counter, as CS high does | Every reply byte against section 9.2: each STATUS byte against `sim/sim.py`'s flops at the start of the cycle in which the byte before it arrives, TAKE, POST, RDREG, RDMEM and CAPS, and 0 everywhere else |
| `pins` | The pins model, with the real SPI block (`pe_spi.sv`) at the pads. Every bit on `ui0` to `ui2`, SCK high and low for 2 to 5 cycles with at most clk/6, CS high for 3 to 8 cycles between transactions. A command acts in *h* = *p* + 2 of its last rising SCK *p* | As `full`, read from MISO at each rising SCK as a host would; MISO must be 0 from 3 cycles after CS rises (9.1). `uo3` is MISO, so the pad compare leaves it out |

The default, `mixed`, takes `quick` on even seeds and `full` on odd. The commands: POST, TAKE, RUN stop and start
(and both masks at once, 9.3), STEP, SETPC, STATUS, ABORT, RDREG of any `reg8` on running and stopped threads,
WRMEM and RDMEM of 1 to 3 words, CAPS, unknown command codes, commands cut before their last argument (in `pins`
also part-way through a byte), and bytes after a command's last argument; `thread8`, `stop8`, `start8` and `pc16`
carry random bits the chip must ignore. Aimed at corners: ABORT while a uio pin is on a shift output or a unit runs,
RDREG of a stopped thread whose unit runs, STEP of a stopped thread whose next word is a unit row, a stop while a
unit sends, and in `quick` POST or TAKE in the cycle a thread's mailbox row runs (9.4), ABORT beside a `pcfg` or
another pin write, a stop in the slot of a timed wait's deadline, a stop during a stepped `ld` or `st`, and a start
in the slot where a stepped row sets LATE.

Where `sim/sim.py` serves in cycle *h* what the RTL serves later (section 10, undefined 6), the bench compares at the
RTL's cycle, and checks that cycle against section 9:

1. RDREG: the RTL must serve it at the start of a cycle from *h* + 2 to *h* + 5, with `sim/sim.py`'s `rdreg()` at
   the start of that cycle (9.2).
2. WRMEM and RDMEM: the RTL's access must come in the first free memory cycle, a stopped thread's with no `ld` or
   `st` under way, *h* + 1 to *h* + 4; `sim/sim.py`'s `host_wrmem` or `host_rdmem` runs after that cycle, and RDMEM's
   word must be in the reply register 2 cycles later (9.4).
3. A memory command that finds every thread running sets BUSY in *h* + 1 on both, and nothing more (9.4).

No command lands in a cycle where the last one's delayed effect does. On silicon commands are at least 48 cycles
apart, so that never happens there; a zero-time bench must keep them apart itself.

### 2.4 Results

The runs of `verif/coverage.sh`, on the coverage models:

| Run | Generator | Host | Images × cycles | Cycles compared | Differ |
|---|---|---|---|---|---|
| 1 | `chips` | `mixed` | 2,000 × 6,000 | 12,000,000 | 0 |
| 2 | `prog` | `mixed` | 1,000 × 6,000 | 6,000,000 | 0 |
| 3 | `rows` | `mixed` | 2,000 × 6,000 | 12,000,000 | 0 |
| 4 | `wide` | `mixed` | 2,000 × 6,000 | 12,000,000 | 0 |
| 5 | `chips` | `pins` | 300 × 6,000 | 1,800,000 | 0 |
| 6 | `prog` | `pins` | 200 × 6,000 | 1,200,000 | 0 |
| 7 | `rows` | `pins` | 300 × 6,000 | 1,800,000 | 0 |
| 8 | `wide` | `pins` | 300 × 6,000 | 1,800,000 | 0 |
| 9 | `chips` | `mixed` | 16 × 70,000 | 1,120,000 | 0 |
| 10 | `rows` | `mixed` | 16 × 70,000 | 1,120,000 | 0 |
| 11 | `wide` | `mixed` | 16 × 70,000 | 1,120,000 | 0 |
| 12 | `directed` | `none` | 3 × 70,000 | 210,000 | 0 |
| 13 | `--corners` | scripted | 2,708 cases | | 0 |
| | all | | 8,151 images and the corners | 52,170,000 | 0 |

The program benches on the RTL (`vlcov.py benches`, `programs/check.py` with the RTL in place of `sim/sim.py`):
487 checks of 16 benches, 487 pass, the same 487 as on `sim/sim.py`.

### 2.5 Planted bugs

Each bug is a small edit to a copy of `src/`, built into new models and run until the first image that differs,
with the generators in turn, at most 40 images each.

| Set | Bugs | Caught | First caught by |
|---|---|---|---|
| Integration: the shift units wired into the core (`pe_core`, `pe_exec`, `pe_pins`, `pe_host`, `pe_decode`) | 31 | 31 | chips 31 |
| Host replies and the memory path | 10 | 10 | chips 9, prog 1 |
| The link: `pe_spi.sv`, and CS in `pe_host.sv` (host mode `pins`) | 8 | 6 | chips 6 |
| An earlier review's list of bugs planted in the core, before the shift units: the 80 that still apply (one no longer does), and its control, which must survive | 80 | 79 | images 65 (stub modes and `pins`), corners 12, `pins` checks of MISO and early CS 2 |

The control survives every run. The three missed are equivalent within the specification. With CS high,
`pe_host.sv` resets its byte counter and its command; the command reset is redundant, because every use of the
command waits for a byte after the first, and the first byte replaces it. `pe_spi.sv` gates a rising SCK with CS:
without the gate, a byte could complete only if the link saw CS and SCK rise in the same cycle after exactly seven
bits, with CS moving while SCK is high. And a memory command's BUSY could be held back by CS high only if CS reached
the link in *h* + 1, which needs a last SCK high phase of one cycle. Section 9.1 asks for SCK phases of at least 2
cycles and no CS move while SCK is high, and section 10 (undefined 4) leaves the rest undefined.

## 3. Code coverage

### 3.1 Method

The models of `verif/Makefile`, built with Verilator's `--coverage-line --coverage-toggle` and `cov_shim.cpp`. A
**line** point is a block of statements, a **branch** point each arm of an `if`, `case` or `?:`, and a **toggle**
point each bit of each signal, 0 to 1 and 1 to 0 counted apart. Points are summed over instances (the four
threads, the four units) and over every chip of every run: a point is hit if any instance hit it in any run. Only
`src/` is counted: the stand-ins in `verif/` (the SRAM model, the buffer cell, the SPI stub) and Verilator's own
library are left out. `project.sv`, the Tiny Tapeout wrapper, is outside the Verilator model, which starts at
`pe_core`; it is wiring, which `test/` checks at the pins. Verilator's own `verilator_coverage --annotate` on
`OUT/merged.dat` marks only two places as never run: the `pcfg` case default and the `CUT_STUFF` block, both in
section 3.3.

### 3.2 Per module

"Before": the stimulus the bench started with, generators `chips` and `prog` in modes `quick` and `full`, and the
program benches. "After": every run of section 2.4.

| Module | Line points, before | after | Branch points, before | after | Toggle points, before | after |
|---|---|---|---|---|---|---|
| `pe_core` | 7 of 7 (100.0%) | 7 of 7 (100.0%) | 10 of 10 (100.0%) | 10 of 10 (100.0%) | 2190 of 2210 (99.1%) | 2209 of 2210 (100.0%) |
| `pe_thread` | 4 of 4 (100.0%) | 4 of 4 (100.0%) | 67 of 68 (98.5%) | 68 of 68 (100.0%) | 615 of 618 (99.5%) | 618 of 618 (100.0%) |
| `pe_exec` | 83 of 85 (97.6%) | 85 of 85 (100.0%) | 38 of 38 (100.0%) | 38 of 38 (100.0%) | 1190 of 1210 (98.3%) | 1194 of 1210 (98.7%) |
| `pe_pins` | 12 of 13 (92.3%) | 12 of 13 (92.3%) | 30 of 30 (100.0%) | 30 of 30 (100.0%) | 821 of 982 (83.6%) | 924 of 982 (94.1%) |
| `pe_shift` | 62 of 62 (100.0%) | 62 of 62 (100.0%) | 317 of 346 (91.6%) | 318 of 346 (91.9%) | 4681 of 4824 (97.0%) | 4720 of 4824 (97.8%) |
| `pe_host` | 38 of 39 (97.4%) | 39 of 39 (100.0%) | 62 of 62 (100.0%) | 62 of 62 (100.0%) | 679 of 684 (99.3%) | 684 of 684 (100.0%) |
| `pe_spi` | - | 2 of 2 (100.0%) | - | 6 of 6 (100.0%) | - | 88 of 88 (100.0%) |
| `pe_mailbox` | 4 of 4 (100.0%) | 4 of 4 (100.0%) | 16 of 16 (100.0%) | 16 of 16 (100.0%) | 475 of 476 (99.8%) | 476 of 476 (100.0%) |
| `pe_regfile` | 3 of 3 (100.0%) | 3 of 3 (100.0%) | 2 of 2 (100.0%) | 2 of 2 (100.0%) | 131 of 132 (99.2%) | 132 of 132 (100.0%) |
| `pe_mem` | - | - | - | - | 178 of 178 (100.0%) | 178 of 178 (100.0%) |
| `pe_decode` | 89 of 91 (97.8%) | 91 of 91 (100.0%) | - | - | 62 of 62 (100.0%) | 62 of 62 (100.0%) |
| all | 302 of 308 (98.1%) | 309 of 310 (99.7%) | 542 of 572 (94.8%) | 550 of 578 (95.2%) | 11022 of 11376 (96.9%) | 11285 of 11464 (98.4%) |

### 3.3 Every line and branch point never hit

After every run, 3 kinds of point remain, all unreachable in this chip:

| # | Where | Point | Why it can't be hit |
|---|---|---|---|
| 1 | `pe_pins.sv:130` | the `default` of the `pcfg` mode case | CFG_IN, PP, OD and BYP are all four values of the 2-bit mode field (6.3, `pcfg`) |
| 2 | `pe_shift.sv:266` | the arm of `tx_rl_new` that holds the send half's run length at 7 | A run that reaches 7 always has its stuff bit next, since 7 is at least every SN, and a stuff bit is the complement of its run (5.9.7, `tx_symbol()` with TXSTF), so TXRL 7 never meets an equal bit. Proved for every reachable state by `verif/unreachable/tx_rl.sby` (k-induction, depth 12, with a cover that reaches TXRL 7) |
| 3 to 29 | `pe_shift.sv`, 27 points | the arm taken when a `CUT_` parameter is 1 | Each `CUT_` parameter is a build option that leaves one piece out (the header of `pe_shift.sv`); all eight are 0 in this chip. The lines, by parameter: |

| Parameter | What it would leave out | Lines of `pe_shift.sv` |
|---|---|---|
| `CUT_OCK` | outside clocks: TEXT, TFALL, TCLK, RFALL and RCLK | 155, 156, 157, 182, and 184 with `CUT_EOP` |
| `CUT_TFRAC` | TFRAC | 162 |
| `CUT_RESYNC` | RESYNC | 192 |
| `CUT_RFRAC` | RFRAC | 191, 560 |
| `CUT_CRC` | the CRC register | 195, 206 to 211, 213, 520, 779, 842 |
| `CUT_LFSR` | CMODE 2 | 212 |
| `CUT_STUFF` | the stuffer and the destuffer | 214, 215, 223, 521, 776 |
| `CUT_EOP` | REOP | 216 |

Before the holes were closed (section 4.4), 7 more were missed, all reachable and only not stimulated: the decode
and execute of `nott` and `tlate` and `tlate` clearing LATE (`pe_decode.sv:148` and `:153`, `pe_exec.sv:276` and
`:280`, `pe_thread.sv:163`), rows that no program and few random words use; CS high in `pe_host.sv:254`, which only
the pins model has; and RXIC saturating, `pe_shift.sv:745`, which takes 65,535 cycles without a reference edge.

### 3.4 Bits that never toggle

Every bit left is constant by design:

| # | Signal (module) | Bits | Why it can't toggle |
|---|---|---|---|
| 1 | `ena` (`pe_core`) | 0 | Tiny Tapeout's enable: always 1 while powered, and unused |
| 2 | `imm` (`pe_exec`) | 15 to 8 | `{8'h00, ir[7:0]}`: the immediates of 6.3 are 8 bits wide |
| 3 | `out_d`, `out_q`, `shd_d`, `shd_q`, `v` (`pe_pins`) | 3 | Pin 3 is `uo3`, MISO (9.1). `put`, `pcfg` and `smap` on `uo3` are illegal words (section 10), so OUT[3] and SHD[3] are held at 0, and V(3) is OUT[3] |
| 4 | `own_d`, `own_q`, `sel_d`, `sel_q`, `shsel_d`, `shsel_q` (`pe_pins`) | 7 and 6 | The 2-bit OWN, SEL and SHSEL fields of the same pin: no legal word claims `uo3` or puts it on a shift output |
| 5 | `byp_d`, `byp_q`, `shbyp_d`, `shbyp_q` (`pe_pins`) | 3 to 1 | Only lanes 0 and 4 to 7 have a bypass (5.8), and `pcfg BYP` on another pin is illegal, so these flops are held at 0 |
| 6 | `crc_shl`, `cs_c` (`pe_shift`) | 0 | Shifted left by one |
| 7 | `pe_word` (`pe_shift`) | 15 to 3 | `{13'd0, RXPE}`: RXPE is 3 bits (5.9.2) |
| 8 | `txmode_q` (`pe_shift`) | 15 | TXMODE bit 15 is reserved and not stored (5.9.3, KEEP in 5.9.4) |
| 9 | `pp`, `pw` / `p2`, `mfull` (`pe_shift`) | 17, 16 / 17 to 15 | Periods of 16 and 15 bits, zero-extended into the Manchester decoder's 18-bit signed arithmetic |
| 10 | `rt`, `nrt`, `sp`, `ph_beg` (`pe_shift`) | `rt`, `nrt`: 7 to 0 and 25, 24; `sp`: 25, 24; `ph_beg`: 7 to 0 and 24 | The timed decoder's 26-bit phase words, in 256ths of a cycle: `rt` and `nrt` are FIRST - 1 and FIRST shifted left by 8, so their low byte is constant (0, and 1 in `nrt`, which is inverted), and the top bits of all four extend values of 24 bits or fewer |

## 4. Functional coverage

### 4.1 Points

`verif/fcov.py` wraps `sim/sim.py`'s methods and row handlers from outside, so `sim/sim.py` itself is unchanged,
and counts each point every time it happens. The lockstep shows the RTL did the same thing in the same cycle.

| Group | Points | What |
|---|---|---|
| row | 376 | Each of the 89 rows of 6.3 run on each thread; `recv`, `send`, `txw` and `rxw` stalling on each thread; an illegal word on each thread (section 10) |
| pin | 51 | `pcfg` from each mode to each on each kind of pin (uio IN, PP and OD; lane PP and BYP; other uo PP), a claim from another thread and a `pcfg` on a pin that is on a shift output (5.3, section 10, defined 9); `smap` of each output by the owner and by another thread; each change of SEL at a commit, on each kind of pin; an open-drain pin on a shift output (5.2, 5.4) |
| unit | 300 | The settings each send and reception starts with (5.9.3): every CRC width with the CRC on, CINIT and CXOR in each CRC mode, every CMODE, SMODE with every SN, TPER, RPER, TFRAC, TAIL, FIRST, REND, RCNT, CSKIP and SYNCLEN ranges, SYMS, TGLA and TGLB, outside clocks rising and falling, TMSB, every output code, every RKIND with RDIFF, RINV, RMSB, RESYNC, RACQ, REOP, RGAIN, and the hunt. Every change of each half's state, and each send and receive state together (5.9.5, 5.9.6). What the halves do: underruns at a start and after a word, an end at the last word, stuff bits sent and removed, a stuff violation, an appended CRC, words loaded while sending and into an idle shift register, words and last words delivered, overruns, sync found, ends by RCNT, REND, REOP, a violation and the Manchester window, the CRC check passing and failing, the receive gear reaching 16, a phase pulled by an edge (5.9.5 to 5.9.7). `sstop` and ABORT in each state of each half (5.9.4, 9.3); `sstat` clearing TXUR and RXOVR |
| host | 36 | Each of RUN stop, RUN start, both masks, STEP, SETPC, RDREG and ABORT on a running and on a stopped thread; RDREG of each `reg8` group of a stopped thread; POST to a full and an empty word, TAKE of a full and an empty word, STATUS with BUSY 0 and 1, WRMEM and RDMEM with a free memory cycle and with every thread running, ABORT with a uio pin on a shift output (section 9) |
| event | 47 | Each way of arming (`tick`, `edge` with a constant and with a register, `now`, `until` and `untilm`, each with and without a timeout) and each way of ending (on time, late, a match, the inbox, a timeout on time and past due, cancelled), with LATE 0 and 1 after it; a deadline armed on a stale D, so late; STALE setting; `tlate` reading LATE 0 and 1 (3.3 to 3.5) |
| mail | 42 | `recv`, `send`, `tryrecv` and `trysend` on a full and an empty word; for each of the 8 words, a thread putting, a thread taking, the host putting and the host taking; a host POST and TAKE in the cycle a thread's mailbox row used the same word (9.4) |

### 4.2 Results

| Group | Points | Hit, before | Hit, after | Holes after |
|---|---|---|---|---|
| row | 376 | 359 | 376 | 0 |
| pin | 51 | 50 | 51 | 0 |
| unit | 300 | 274 | 292 | 8 |
| host | 36 | 36 | 36 | 0 |
| event | 47 | 43 | 47 | 0 |
| mail | 42 | 42 | 42 | 0 |
| all | 852 | 804 | 844 | 8 |

### 4.3 Holes

The 8 points never hit are changes of a half's state in one cycle that the code of 5.9.5 and 5.9.6 can't make:

| # | Point | Why it can't happen |
|---|---|---|
| 1 | send half: idle to sending | A send starts only from TXARM at a commit (`tx_step()`, idle branch), and `sgo` sets TXARM in step 3, after step 2b has run (3.2). So a half is armed for at least the rest of that cycle |
| 2 | send half: idle to its tail | A tail begins only at `tx_end()`, from a running send or from a start with no word, never from idle |
| 3 | send half: tail to sending | In a cycle with TXTL, `tx_step()` runs only `tx_tail()`; a start needs the idle branch of a later cycle |
| 4 to 6 | receive half: idle to hunting, to receiving, or holding its last word | Like 1: the half leaves idle only through RXARM, which `sgo` sets in step 3 |
| 7 | receive half: hunting to armed | RXHUNT clears only in `rx_begin()`, which starts a reception, or in `rx_stop()`, which clears RXARM too |
| 8 | receive half: holding its last word to receiving | In a cycle with RXEP, `rx_step()` only delivers the word, then hunts if HUNT; a reception starts in a later cycle |

### 4.4 What closed the holes

Each point the "before" stimulus missed is credited to the first run, in this order, that hits it:

| # | Added | Code points it closed | Functional points it closed |
|---|---|---|---|
| 1 | Generator `rows` | 120: the decode and execute of `nott` and `tlate`, `tlate` clearing LATE, and 115 toggles, 98 of them in `pe_pins.sv` (io4 to io7 and the lanes on shift outputs, bypass switched in and out) | 23: 17 rows on threads that hadn't run them, 4 event points, 1 pin and 1 unit point |
| 2 | Host mode `pins` | All of `pe_spi.sv`, which the stub model leaves out (2 line, 6 branch and 88 toggle points), and 25 more: `if (rx_idle)` in `pe_host.sv:254` (CS high), and toggles | 4 unit points |
| 3 | Generator `wide`, with resets in mid-run | 11: `rst_n` falling in each module, and the units' widest settings | 13 unit points: CRC widths, SN values, TPER and RPER of 256 and up |
| 4 | Images of 70,000 cycles, past TB's wrap | 1 toggle | 0 |
| 5 | Generator `directed` | 25: RXIC saturating (`pe_shift.sv:745`) and starting again, the top bit of TXCD and of the countdown before its decrement | 0 |
| 6 | `--corners` | 0: they aim at timing corners of code the images already reach, and earn their keep in section 2.5 | 0 |

## 5. Limits

1. Code coverage is of the RTL as Verilator simulates it. The netlist is checked by the gate-level tests of `test/`
   and, in part, by `formal/chip.eqy`.
2. A point hit is not a point checked: the lockstep compares the state after every cycle and the program benches
   check the pins, but a value can be wrong in a way neither sees. The planted bugs of section 2.5 measure that, and
   `formal/` proves the properties in its table for every reachable state.
3. Functional points are a list written from the specification. A behaviour with no point can go unmeasured; the
   point list is in `fcov.py`.
4. The lockstep compares the RTL with `sim/sim.py`, so an error both share would not show. `sim/sim.py` was
   written by hand from sections 2 to 9, and the program benches and `programs/sigrok/` check behaviour against
   the protocol standards instead.
5. Builds with a `CUT_` option of `pe_shift.sv` set are not measured.
