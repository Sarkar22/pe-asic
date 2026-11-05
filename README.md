![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg)

# Protocol emulator core

A Tiny Tapeout design for IHP SG13CMOS5L that emulates digital protocols after fabrication. One programmable core runs four hardware threads in a fixed rotation, one instruction per 4 cycles each, with pin changes that land on exact cycles and one memory for code and data. Each protocol is a program loaded over an SPI link from a host, not a block of hardware.

- [Read the documentation for project](docs/info.md)
- [Instruction set: the full specification](docs/ISA.md)
- [Simulator and assembler: run a program in Python](sim/README.md)
- [Architecture of the RTL and how to run the tests](docs/ARCHITECTURE.md)
- [Flexibility trial: the protocol pool and the draw](flex/README.md)

## Tests

`cd test && make` runs the cocotb tests at the pins under Icarus. It needs cocotb 2.0.1. `make GATES=yes` runs them on a netlist in `test/gate_level_netlist.v`, with the RTL core beside it for a pin-by-pin compare. [test/README.md](test/README.md) has the details.

## What is Tiny Tapeout?

Tiny Tapeout is an educational project that aims to make it easier and cheaper than ever to get your digital and analog designs manufactured on a real chip.

To learn more and get started, visit https://tinytapeout.com.

## Resources

- [FAQ](https://tinytapeout.com/faq/)
- [Digital design lessons](https://tinytapeout.com/digital_design/)
- [Learn how semiconductors work](https://tinytapeout.com/siliwiz/)
- [Join the community](https://tinytapeout.com/discord)
- [Build your design locally](https://www.tinytapeout.com/guides/local-hardening/)
