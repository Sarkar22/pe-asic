# Running a program

[`docs/ISA.md`](../docs/ISA.md) is the whole specification. `asm.py` is the assembler and `sim.py` a cycle-level simulator of the chip. Python 3, standard library only.

`python3 sim/example.py` runs `uart_tx.s`, the program in section 11.3, and decodes its transmit pin.

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
