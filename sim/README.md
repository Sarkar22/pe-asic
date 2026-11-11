# Simulators and checks

[`docs/ISA.md`](../docs/ISA.md) is the whole specification. Everything here is Python 3, standard library only, and runs from the repository root.

| File | What it is |
|---|---|
| `asm.py` | The assembler. It reads the encodings from the table in section 6.3 |
| `sim.py` | A cycle-level simulator of the chip, written by hand from the text. `programs/check.py` runs every program of `programs/` on it |
| `example.py`, `uart_tx.s` | The program of section 11.3 on `sim.py`, with its transmit pin decoded |
| `docsim.py` | A simulator compiled from the text: from its code blocks and its instruction table |
| `isatable.py` | Reads the table of section 6.3: a decoder, and each row's Semantics cell compiled to Python |
| `sim2.py` | A second simulator written by hand from the text. It takes the row semantics from `isatable.py` and compiles the code blocks of section 5.9. `docsim.py` builds on it |
| `asm2.py` | A second assembler, on `isatable.py` |
| `compare.py` | Runs `sim.py`, `sim2.py` and `docsim.py` side by side and compares them on every cycle |
| `claims.py` | Checks 226 statements the document makes, on `sim.py`, `asm.py` and `asm2.py` |

## Compiled from the text

Section 3.4 makes the document's code blocks the normative definition, in the notation of section 6.1, and says that blocks of that form are machine-read. `docsim.py` reads them. Every block function, a line `name(args):` and the lines indented under it, becomes a Python function, line by line:

- `X := e` becomes an assignment. D is cut to 16 bits and PC taken modulo DEPTH, as section 6.1 says.
- A name in capitals is state: of the thread running the code, of that thread's shift unit if section 5.9.2 lists the name per thread, or of the chip if it is global (sections 2 and 5.9.2).
- A field of the table in section 5.9.3, such as TPER, reads its bits of the configuration slot when the line runs.
- `next` is `(PC + 1) % DEPTH`, and `a` is the current cycle (section 3.4).
- A call of another block function passes the chip, the thread and the cycle on.

There are 65 such functions, in sections 3.4, 3.5, 3.6, 4.2, 5.3, 5.4, 5.9.4 to 5.9.7, 6.1 and 9.3. Nothing is written by hand for them. A capital name that the rules can't place stops the compile, so a name the text never defines can't slip through as a Python name.

Each row of the instruction table runs its Semantics cell, which `isatable.py` compiles statement by statement under the rules of section 6.1, and is decoded by its Encoding and Fields cells. What remains by hand is what the text gives as prose and tables rather than as code blocks: the step order of section 3.2, with its rule for what step 3 sees of the unit, the pad equations of section 5.2 with V(p), step 5's `CMT := 0` and `SYNC3 := SYNC2`, the reset values of section 8, the mailbox and RDREG of section 9.2 (RDREG of the unit's registers runs the compiled `sstat` and `srd`), and the host's byte framing. So a change to a code block or to a row of the table changes what `docsim.py` does, with no other edit: `slot()` and `memory()` of sections 3.6 and 4.2, for example, run as written.

## Three simulators, and what they share

| Part of the text | `sim.py` | `sim2.py` | `docsim.py` |
|---|---|---|---|
| Step order (3.2) and pads (5.2) | by hand | by hand | by hand, its own |
| Code blocks of 3.4 to 3.6, 4.2, 5.3, 5.4, 6.1 and 9.3 | by hand | by hand | compiled |
| Code blocks of 5.9.4 to 5.9.7, the shift units | by hand | compiled, by rules of its own | compiled |
| Row semantics, section 6.3 | by hand | compiled by `isatable.py` | compiled by `isatable.py` |
| Decoding, section 6.3 | through `asm.py`'s reading of the table | `isatable.py` | `isatable.py` |
| Host commands, section 9.2 | calls | bytes | bytes, through `sim2.py`'s framing |

`sim.py` shares no code with the other two: it imports only `asm.py`, while they read the document through `isatable.py`. So each code block has three readings: outside 5.9, one compiled and two by hand; in 5.9, two compiled by different rules and one by hand. A row of the table has two readings among the simulators, `isatable.py`'s and `sim.py`'s, and `claims.py` adds a third: it runs each row's Semantics text one statement at a time and compares the result with `sim.py`'s handler on random states (the claim "6.3 every row's handler matches its Semantics text").

`sim2.py` and `docsim.py` lag a bypass path by one cycle, as section 10 allows (undefined, item 3): their pads use the previous cycle's `ui`, and `compare.py` gives `sim.py` the previous cycle's `ui` for its pads too. All three serve a host command in one call between two cycles, in cycle *h*, as section 10 allows a byte-level simulator (undefined, item 6).

## The checks

`python3 sim/compare.py` runs every program of `programs/` three ways, and 300 random images, and compares the three simulators after every cycle. It compares the pads, every thread's registers and flags, every shift unit's state and slots, the pin flops and owners, the output switch, the mailbox, BUSY and the memory, and the reply to every host command. The images are drawn uniformly over the table's rows, or biased to the shift-unit rows with settings that make the units run. It prints one line for each run that differs, naming the simulator that stands apart and the first thing in which it differs, with its value and the others', then a summary, and exits 1 if any run differs. That is about 3 million cycles, a few minutes on one core: `--jobs 4` runs it in 4 processes, and `--quick` is a shorter run. Its docstring lists the options.

`python3 sim/claims.py` checks 226 statements, each named by the section it tests, and prints PASS or FAIL for each, then the count. It exits 1 if any fails.

`compare.py --doc FILE` checks the comparison itself. `sim2.py` and `docsim.py` then read FILE, through `isatable.py`, while `sim.py` and `asm.py` go on reading `docs/ISA.md`, so the programs and the images don't change. Copy `docs/ISA.md`, change a code block or a row of the table, and see which simulator follows the change. Four such changes, one at a time, with `claims.py` run on a copy of the repository whose `docs/ISA.md` has the change:

| Change in the copy | Section | `compare.py --doc` | `claims.py` |
|---|---|---|---|
| None: an identical copy | | No run differs | Passes |
| `tx_step()` starts a send with `TXACC := 128`, not 0 | 5.9.5 | `sim.py` stands apart | Passes: no claim covers it |
| The Semantics of `tsti`: `T := (Rn & i) == i`, not `!= 0` | 6.3 | `sim.py` stands apart | Fails "6.3 every row's handler matches its Semantics text" |
| The Encoding of `cp`, with its two register fields swapped | 6.3 | `sim.py` stands apart | Passes: `asm.py` and `sim.py` both read the swapped encoding |
| `now()` sets `D := a + 2`, not `a + 1` | 3.4 | `docsim.py` stands apart | Passes: it reads no code block |

A change to a code block outside section 5.9 reaches only `docsim.py`. A change in section 5.9, or to a row of the table, reaches `sim2.py` too, because `sim2.py` compiles the code blocks of 5.9 and takes the rows from `isatable.py`. `sim.py` reads neither, so it stands against both.

## Running a program

`python3 sim/example.py` runs `uart_tx.s`, the program in section 11.3, on `sim.py`, and decodes its transmit pin. A program of your own runs like this:

```python
import sys; sys.path.insert(0, 'sim')
from asm import assemble
from sim import Chip

img = assemble(open('prog.s').read(), depth=512)     # depth 128 or 512
print(img.size, img.warnings)
chip = Chip(depth=512, seed=1)                       # seed fills unwritten memory
for addr, word in img.words.items():
    chip.host_wrmem(addr, word)
chip.host_setpc(0, 0)                                # thread 0 starts at address 0
chip.host_run(0, 1)                                  # stop mask, start mask
chip.host_post(7, 434)                               # host fills mailbox word 7; returns 1 if refused

ui, uio_in = 0, 0
for cycle in range(100000):
    uo, uio_out, uio_oe = chip.pads(ui)              # pad values this cycle: section 5.2
    # your board model goes here: work out ui and uio_in from uo, uio_out and uio_oe
    chip.cycle(ui, uio_in)
```

`chip.host_take(n)` returns `(full, word)`. `chip.th[i].LATE` and `chip.th[i].ILL` are thread flags. `ui` and `uio_in` are 8-bit integers. An undriven `uio` pad has whatever level your board model gives it.

To start at a chosen alignment, call `chip.cycle` n times before `chip.host_run`: *h* is n - 1. With n = 0 the simulator gives *h* = 65535, which section 9.1 rules out on a chip, and it behaves as n = 4.

To look inside a thread, `chip.host_run(1 << i, 0)` stops it. After 8 cycles `chip.host_rdreg(i, reg)` returns what RDREG returns (section 9.2), and `chip.host_status()` returns the four STATUS bytes. The simulator's own state can be read at any time: `chip.th[i].PC`, `.R`, `.T`, `.D`, `.S`, and `chip.M` and `chip.FULL`. These aren't visible to firmware or to a real host. `img.labels` maps labels to addresses, so `chip.host_setpc(1, img.labels['rx'])` starts a second thread at a label.

The host link isn't modelled at pin level. `host_*` calls act at once, between cycles, so `ui0`, `ui1` and `ui2` do nothing to the host link here, and `uo3` reads 0. Firmware can still read those three `ui` bits.
