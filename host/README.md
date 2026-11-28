# Host for the demo board

The chip is loaded and run over an SPI link (ISA section 9). This folder is the host side of that link for Tiny
Tapeout's demo board v3, whose RP2350B runs MicroPython with Tiny Tapeout's SDK (tt-micropython-firmware v3.1.1),
and a first power-on test.

| File | What it is | Runs on |
|---|---|---|
| `pehost.py` | The library: every command of ISA 9.2 with its reply decoded, loading and reading back programs, starting threads, mailbox polling, and the image file reader | the board and CPython |
| `board.py` | The demo board: selects the project, makes the clock, resets the chip and drives the link on SPI0 | the board |
| `bringup.s` | The bring-up program, 81 words | the chip |
| `bringup.img` | `bringup.s` as an image file, written by `mkimage.py` | the board reads it |
| `bringup.py` | The bring-up's host side | the board and CPython |
| `mkimage.py` | Writes an image file from an assembler source, with `sim/asm.py` | CPython |
| `test_host.py`, `simlink.py`, `replay.py` | The tests below | CPython; `replay.py` also MicroPython |

## On the board

1. Set the DIP switches of `ui0`, `ui1` and `ui2` off: each switch ties its line to 3.3 V through 1 kΩ, and those lines
   are the link's CS, SCK and MOSI. `board.py` checks this and stops if one is on.
2. The pin test drives all eight `uio` pins and the `uo` pins. Unplug anything from the `uio` PMOD first, or pass
   `pins=False` below.
3. Copy `pehost.py`, `board.py`, `bringup.py` and `bringup.img` to the board's file system, for example with
   `mpremote cp pehost.py board.py bringup.py bringup.img :`.
4. At the board's REPL:

   ```python
   import board
   host = board.setup(PROJECT)     # the project's name in the SDK's shuttle index, or its mux address
   board.bringup(host)             # the whole first power-on test; board.memory_test(host) runs only its memory part
   ```

`board.setup()` prints each step as it goes:

1. It selects the project through the SDK in its `ASIC_MANUAL_INPUTS` mode, so every `ui`, `uio` and `uo` pin of the
   RP2350 is an input. It turns off the SDK's per-project settings, which would set clocks and reset.
2. It holds `rst_n` (GPIO14) low.
3. It checks the three DIP switches.
4. It sets the system clock and the peripheral clock to 100 MHz with `machine.freq(100_000_000, 100_000_000)`, and makes
   the chip's 50 MHz clock on GPIO16 with the PWM: TOP 1 and CC 1, so one 10 ns system cycle high and one low (RP2350
   datasheet 12.5.2.6). It reads the registers back and stops if they differ. GPIO16 has no clock-generator output
   (datasheet 9.4), so the PWM is the way to an even duty cycle at 50 MHz. The chip needs one close to 50% (ISA 7.3).
5. It makes CS (GPIO17, `ui0`) a plain output, high, and sets SPI0 to mode 0, MSB first, at 5 MHz on SCK GPIO18,
   MOSI GPIO19 and MISO GPIO36. Each SCK phase is then 5 chip cycles, against the 2 that ISA 9.1 asks for, and SCK
   runs at clk/10, below clk/6.
6. It holds reset for 2 ms with the clock running, CS high and SCK low, as ISA 9.1's last paragraph asks. It then
   releases `rst_n` to the board's own reset circuit by making it an input, as the SDK does, and waits 20 ms for
   board revision 3.2's RC on that line.
7. It reads CAPS. `check_caps()` names a link that reads MISO one bit early or late.

After that, `board.py` drives only the clock, CS, SCK and MOSI (`rst_n` is an input again); every other pin stays an
input until you set it up yourself with `machine.Pin`. Don't change the SDK's mode afterwards: that makes the clock
pin an input again.

Each transaction is CS low, `SPI.write_readinto`, a wait until SPI0 is idle, CS high, and 1 µs with CS high. The wait
is there because `write_readinto` can return as soon as the last byte has arrived, while SCK may still be high, and
CS must not move while SCK is high (ISA section 10, item 4). CS rises even if `write_readinto` raises, so the next
transaction still starts with a falling CS.

## What the bring-up checks

Each check prints one line, PASS or FAIL, and the run ends with the totals. A link error, such as a mailbox word that
never fills, stops it and is printed as the FAIL of the step it stopped in.

- CAPS: version 1, 4 threads, 512 words.
- STATUS after reset: no thread running, nothing illegal or late, the mailbox empty, BUSY clear.
- Memory: all 512 words written and read back twice, first with a different word at every address, then with each
  word's complement.
- The bring-up program loaded and read back.
- All four threads started by one RUN.
- The mailbox: a round trip on each of the 8 words. The host posts a word, a thread takes it and sends its
  complement back on the same word, and the host takes that.
- The `uio` pads: thread 0 drives 20 patterns on `io0..io7` and on the `uo` pins. Each time, the chip reads its own
  `uio` pads back while it drives them (ISA 5.2), and the RP2350's own inputs read the `uio` and `uo` pins (all but
  `uo3`, which is MISO). The line after this check shows what `ui3..ui7` read, which is where a DIP switch is on.
- Every thread halted by itself, with no illegal word, nothing late and the mailbox empty.
- Registers: RDREG of all 32 registers of every thread against the state the program leaves. What depends on time
  is left out: thread 0's D, every thread's STALE bit, the `ui` half of thread 0's S, and all of thread 0's S if the
  pin test was skipped. The D that the program saved on each thread at start must be the same on all four, since
  one RUN gives them one D (ISA 9.3).
- Every thread started and stopped by the host: SETPC, one RUN, a stop, and every PC read back.
- ABORT.

## The library

```python
import pehost, board
host = board.setup(PROJECT)
with open('uart_tx.img') as fh:
    img = pehost.read_image(fh)
host.load_and_verify(img)        # one WRMEM per run of addresses, then RDMEM; raises if a word differs
host.start(img.starts)           # SETPC for each thread, then one RUN
host.post_wait(7, 434)           # programs/basic/uart_tx.s: cycles per bit, on mailbox word 7
host.post_wait(0, 0x41)          # a byte to send, on word 0
print(host.status())             # run 1 ill 0 late 0 stale E full 00 busy 0: stopped threads go stale (ISA 3.5)
```

`Host` has one method per command of ISA 9.2: `caps`, `check_caps`, `wrmem`, `rdmem`, `run`, `setpc`, `status` (named
fields), `step`, `rdreg` (by number or by name: `R0`..`R7`, `D`, `S`, `PC`, `FLAGS`, `SSTAT`, `CRC_LO`, `CRC_HI`,
`ENDF`, `CFG0`..`CFG15` or a slot name such as `TXPER`), `post` (False when the word was full), `take` (`(full, data)`)
and `abort`. On top of them are `load`, `verify`, `load_and_verify`, `start`, `stop`, `wait_stopped`, `take_wait` and
`post_wait`. The commands check their arguments, because the chip would take thread 5 as thread 1 and wrap addresses.
They also check the reply bytes that ISA 9.2 fixes, so a link that misreads shows as a `LinkError`.

The library talks to a transport with two methods. `xfer(out)` makes one transaction and returns the bytes read.
`pause_us(us)` waits. `board.Link` is the board's transport. Another host only needs those two methods.

## Program images

```
python3 host/mkimage.py programs/basic/uart_tx.s --start 0=start -o uart_tx.img
```

An image is a text file that the board reads without the assembler:

```
pe-image 1
# programs/basic/uart_tx.s, assembled by sim/asm.py: 15 words
@0000 90E8 9020 0100 A5E8 ...      up to 16 words from the hex address after @
label start 0000
start 0 0000                       thread 0 starts at 0
end 15 18AD                        the word count, and a CRC-16/CCITT-FALSE over every address and word
```

The end line comes last, so a file cut short in copying is refused, and so is one with a word changed.

## Tests

- `python3 host/test_host.py` runs the CPython tests. They check the bytes each command sends, written from the table in
  ISA 9.2, and the decoding of every reply. They check the image file and its damage checks, and the library and the
  bring-up against the simulator `sim/sim.py`, at the byte level (`simlink.py`). That includes a chip with a stuck
  memory bit, a `uio` pad that never reads high, a crossed mailbox word, a stuck bit in a mailbox word's data, and a RUN
  that starts its threads in different cycles, each of which the bring-up must fail. They also run `board.py` against
  stand-ins for MicroPython's `machine` module and the SDK. These check the order of the setup, the reset rule, the SPI
  settings, the busy wait before CS rises, CS rising after an error, giving up on a busy flag that never clears, and the
  refusal when a DIP switch is on or the SDK won't enable the project.
- In `test/test.py`, the same library runs unchanged at the chip's pins, at RTL and at gate level. cocotb's
  `bridge` and `resume` let its blocking calls drive the pins:
  - `test_host_bringup`: the bring-up with the board's timing: SCK at 5 MHz from the clock's own PLL, CS on the same
    10 ns grid, and the SPI block's gaps between bytes.
  - `test_host_bringup_at_the_limits`: the whole bring-up, memory included, at ISA 9.1's limits: SCK at clk/6 with
    phases of 2 and 4 cycles, and CS high for exactly 3 cycles.
  - `test_host_runs_a_program`: `programs/basic/uart_tx.s` through `mkimage.py` and the image reader, loaded, read
    back and started on thread 1, with its UART frames checked at `uo0`; and two words loaded one address apart,
    with the word between them left alone.
  - `test_host_micropython_clock`: MicroPython's default 48 MHz peripheral clock, with SCK at 4 MHz unrelated to the
    chip's clock.
  - `test_host_miso_margins`: where MISO may be read. With the board's timing, CAPS reads right with MISO read 35 ns
    after the rising SCK, or 120 ns before it. Further out, `check_caps` names the slip.
- MicroPython: `mpy-cross` (1.29.0, from PyPI) compiles every file that runs on the board. MicroPython's unix port
  (v1.29.0) runs `replay.py`, which runs the bring-up through the library against a transcript that
  `python3 host/test_host.py --transcript FILE` records from the simulator: `micropython host/replay.py FILE`. Every
  transaction must send the same bytes as under CPython.

## Things to know

- MISO moves 2 to 3 chip cycles after a rising SCK reaches the pad (ISA 9.1), which leaves the RP2350 at least 2
  cycles of hold after its own edge. The RP2350 datasheet doesn't give the SPI block's sampling instant or any pad
  timing. The 100 MHz peripheral clock keeps the block's own steps at 10 ns, where MicroPython's default 48 MHz makes
  them 20.8 ns. If CAPS ever reads `02 92`, the link reads MISO after the chip has moved it; `00 A4`, or `00 24`
  with the SPI block's gap between bytes, before. `check_caps()` says which.
- `machine.SPI(0, ...)` without `miso=` takes MicroPython's default SPI0 RX pin for this board, GPIO16, which is the
  chip's clock. `machine.UART(0)` with its default pins takes GPIO16 and GPIO17.
- A later `machine.freq()` with one argument moves the SPI clock back to 48 MHz and restarts the PLL that the chip's
  clock comes from.
- On board revision 3.2, `uo3` (MISO) also drives segment D of the 7-segment display, through a solder jumper and 510 Ω.
- The SDK leaves every project pin an input with the RP2350's pull-down on. On RP2350 stepping A2, erratum
  RP2350-E9 can hold such an input near 2.2 V (datasheet appendix D; stepping A3 fixes it). For a net that an
  outside part drives, `board.setup(..., release_pins=(gpio, ...))` turns that pin's input buffer and pulls off. The
  RP2350 then reads such a pin as 0, so the bring-up's check of what the board sees fails on it; pass `pins=False`
  there.
