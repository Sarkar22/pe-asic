<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

One programmable core emulates digital protocols after fabrication. Each protocol is a program, not a block of hardware. Four hardware threads take turns in a fixed rotation, one instruction per 4 clock cycles for each thread, and share one 512-word memory for code and data.

A thread changes the outside world only at an event. It prepares pin values, names the cycle or the input pattern at which they should land, and carries on. In that one cycle the hardware commits the prepared pins, snapshots all inputs and records the cycle number. So edges land on exact cycles whatever the code around them is doing.

A host loads and starts programs over an SPI link: `HOST_CS`, `HOST_SCK`, `HOST_MOSI` and `HOST_MISO`, mode 0, MSB first, SCK up to a sixth of the clock. Eight mailbox words carry data between the host and the threads. See [ARCHITECTURE.md](ARCHITECTURE.md) for the blocks and how a command lands.

## How to test

From the host, after reset:

1. Write a program with `WRMEM`: CS low, `0x01`, the address as 2 bytes, then 2 bytes per word, then CS high.
2. Set the thread's program counter with `SETPC`: `0x11`, thread, PC as 2 bytes.
3. Start it with `RUN`: `0x10`, a stop mask, a start mask. Bit *i* is thread *i*.
4. Read `STATUS`, `0x12` and 4 dummy bytes, to see which threads run. `POST` (`0x20 + n`, 2 data bytes) and `TAKE` (`0x28 + n`, 3 dummy bytes) move data through mailbox word *n*.

`test/test.py` does all of this at the pins, including a UART frame sent by a program.

## External hardware

None for a loopback test: a `uio` input reads the pad even while the chip drives it. The demo board's RP2350 can act as the host on SPI0. Everything else depends on the protocol being emulated. Lines that must stay recessive through reset belong on a `uio` pin in open-drain mode with an outside pull-up.
