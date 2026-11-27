![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg)

# Protocol emulator core

A Tiny Tapeout design for IHP SG13CMOS5L that emulates digital protocols after fabrication. One programmable core runs four hardware threads in a fixed rotation, one instruction per 4 cycles each, with pin changes that land on exact cycles and one memory for code and data. Each protocol is a program loaded over an SPI link from a host, not a block of hardware.

- [Read the documentation for project](docs/info.md)
- [Instruction set: the full specification](docs/ISA.md)
- [Simulator and assembler: run a program in Python](sim/README.md)
- [Protocol programs: 61 programs and the benches that check them](programs/README.md)
- [Protocols: what the chip speaks, and how](#protocols)
- [Architecture of the RTL and how to run the tests](docs/ARCHITECTURE.md)
- [Status: chip numbers, tests and open items](docs/STATUS.md)
- [Decisions and why](docs/DECISIONS.md)
- [Flexibility trial: the protocol pool and the draw](flex/README.md)

## Protocols

Each protocol below runs as one or more programs in [`programs/`](programs/README.md) ([ISA.md](docs/ISA.md) section 11.2 has a row for each program; the two identical UART transmitters share one). Speeds are at 50 MHz, from section 7.4 and the programs' rows in section 11.2. "How" says what carries the protocol: firmware alone, firmware with a thread's shift unit (section 5.9), or a bypass lane (section 5.8). The benches are in `programs/bench/` and run on the simulator with `programs/check.py`; every bench also runs with the RTL in place of the simulator under `verif/coverage.sh` ([COVERAGE.md](docs/COVERAGE.md)). `programs/sigrok/check.py` decodes each bus with sigrok's decoders, on the simulator and, with `--rtl`, on the RTL, whose pads must equal the simulator's on every cycle. The cocotb tests in `test/` run at RTL and on the gate-level netlist.

| Protocol | Role | Speed at 50 MHz | How | Programs | Checked by |
|---|---|---|---|---|---|
| UART | Send and receive | Send 4.17 Mbaud (12 cycles a bit); receive 1.16 Mbaud with frames abutting, 2.27 Mbaud with `basic/uart_rx.s`; 16.7 Mbaud both ways on a shift unit, with parity | Firmware; shift unit, parity from its CRC register; a bypass lane that passes a line through and replaces one byte in flight | `basic/uart_tx.s`, `basic/uart_rx.s`, `standard/uart_tx.s`, `standard/uart_rx.s`, `trimmed/uart_tx.s`, `trimmed/uart_rx.s`, `advanced/uart_par_unit.s`, `advanced/uart_bypass.s` | Benches `basic.py`, `standard.py`, `trimmed.py`, `unit_small.py`, `bypass.py`; sigrok `uart`; cocotb `test_uart_frame_at_the_pad`, `test_unit_uart_tx`, `test_unit_uio_to_unit`, `test_bypass_override` |
| SPI | Controller and target | Controller 2.78 MHz (half period 9), 3.125 MHz after `now`; target 2.5 MHz in firmware, and on a shift unit 8.33 MHz with CS held low or 3.57 MHz with CS pulsed between words | Firmware; shift unit on an outside clock | `basic/spi.s`, `standard/spi.s`, `extra/spi_target.s`, `advanced/spi_unit.s` | Benches `basic.py`, `standard.py`, `trimmed.py`, `extra.py`, `spi_unit.py`; sigrok `spi` |
| I2C | Controller and target | 100 and 400 kHz; a target build for 1 MHz (Fast-mode Plus) | Firmware, on open-drain pins | `basic/i2c_write.s`, `standard/i2c.s`, `standard/i2c_write.s`, `standard/i2c_target.s`, `trimmed/i2c.s`, `extra/i2c.s`, `advanced/i2c_tgt.s`, `advanced/i2c_fp.s` | Benches `basic.py`, `standard.py`, `trimmed.py`, `extra.py`, `i2c_target.py`; sigrok `i2c`, `eeprom24xx` |
| USB low speed | Device | 33 cycles a bit, 1.5152 Mbit/s (USB allows 1.5 Mbit/s ±1.5%), so 50 MHz only | Firmware; shift units for enumeration: NRZI, bit stuffing, CRC5 and CRC16, the end of packet and dribble bits | `advanced/usb_tx.s`, `advanced/usb_rx.s`, `advanced/usb_dev.s`, `advanced/usb_enum.s` | Benches `usb_tx.py`, `usb_rx.py`, `usb_dev.py` and `usb_enum.py`, the last judged by `usb_check.py`; sigrok `usb_signalling`, `usb_packet`, `usb_request`; cocotb `test_usb_enumerate`, `test_unit_usb` |
| 10BASE-T | Node: answers ARP and ICMP echo, sends link pulses | 10 Mbit/s | Shift units on half cycles (THALF to send, RKIND 3 to receive), with the CRC-32 FCS and the byte end of a Manchester frame | `advanced/eth_arp.s` | Bench `eth.py`, judged by `eth_check.py`; no sigrok decoder; cocotb `test_eth_arp` and eight more `test_eth_*` |
| JTAG | Controller | 3.125 MHz (half period 8) with `advanced/jtag.s`; 1.56 MHz (half period 16) with `standard/jtag.s` | Firmware | `standard/jtag.s`, `advanced/jtag.s` | Benches `standard.py`, `jtag_chain.py`; sigrok `jtag` |
| SWD | Host | 1.04 MHz (half period 24) | Firmware | `extra/swd.s`, `advanced/swd.s` | Benches `extra.py`, `swd_host.py`; sigrok `swd` |
| PS/2 | Device, both directions | 12.5 kHz clock | Firmware, on open-drain pins | `advanced/ps2_dev.s` | Bench `ps2.py`; sigrok `ps2` |
| CAN 2.0A | Node: sends, receives, arbitrates and acknowledges; on a shift unit it also sends error flags | 500 kbit/s in firmware; 1 Mbit/s on a shift unit, receiving to 1.25 Mbit/s | Firmware, with stuffing and CRC-15 in code; shift unit, with stuffing, CRC-15 and the bit timing | `advanced/can.s`, `advanced/can_unit.s` | Benches `can.py`, `can_unit.py`; sigrok `can`; cocotb `test_can_third_node`, `test_can_third_node_mid`, `test_can_stuff_error_flag` |
| 1-Wire | Controller | Standard speed | Firmware; shift unit on an open-drain pin, with the CRC-8 check | `basic/onewire.s`, `standard/onewire.s`, `advanced/ow_unit.s` | Benches `basic.py`, `standard.py`, `unit_small.py`; sigrok `onewire_link`, `onewire_network` |
| WS2812 | Controller | 1.24 to 1.25 µs a bit, about 800 kbit/s | Firmware; shift unit, three symbols a bit | `basic/ws2812.s`, `standard/ws2812.s`, `advanced/ws2812_unit.s` | Benches `basic.py`, `standard.py`, `unit_small.py`; sigrok `rgb_led_ws281x` |
| HDMI CEC | Initiator | 2.4 ms bits | Firmware, on an open-drain pin | `standard/cec.s` | Bench `standard.py`; sigrok `cec` |
| LIN | Responder, header only | 19.2 kbit/s, the commander's clock up to 14% off | Firmware, with auto-baud | `extra/lin.s` | Bench `extra.py`; sigrok `uart`, `lin` |
| DMX512 | Send and receive | 250 kbit/s | Firmware | `advanced/dmx_tx.s`, `advanced/dmx_rx.s` | Bench `dmx.py`; sigrok `dmx512` |
| MDIO (IEEE 802.3 clause 22) | Station and PHY | Station 1.25 MHz; PHY to 2.08 MHz | Firmware | `advanced/mdio_sta.s`, `advanced/mdio_phy.s` | Bench `mdio.py`; sigrok `mdio` |
| NEC IR | Send | 38 kHz carrier | Firmware | `extra/nec_tx.s` | Bench `extra.py`; sigrok `ir_nec` |
| SD card, SD mode | Host on the 1-bit bus, reads | Clock to 8.33 MHz | Shift units: CRC7 and CRC16, the clock from a send half, and receive at that clock's edges read back from its pad | `advanced/sd_unit.s` | Bench `sd_unit.py`, with a card model from the SD specification; sigrok `sdcard_sd` |
| I2S | Controller and target | Controller SCK to 8.33 MHz; target 5 MHz | Shift units: the controller's two started on one cycle, the target's on SCK's edges | `advanced/i2s_ctl.s`, `advanced/i2s_tgt.s` | Bench `i2s.py`; sigrok `i2s` |
| TDM | Controller and target | Controller 6.144 MHz (48 kHz, 8 × 16 bits); target 3.072 MHz | Shift units: the controller's two started on one cycle, the target's on SCK's edges | `advanced/i2s_ctl.s`, `advanced/i2s_tgt.s` | Bench `i2s.py`; sigrok `tdm_audio` |
| XY2-100 | Controller | 2 Mbit/s | Three shift units started on one cycle, parity from the CRC register | `advanced/xy2_unit.s` | Bench `unit_small.py`, with a frame reader; no sigrok decoder |
| HDLC-style framing | Send | 1.09 Mbit/s (46 cycles a bit) | Firmware: NRZI, bit stuffing and CRC-16/X-25 | `standard/hdlc_tx.s` | Bench `standard.py`; no sigrok decoder |
| Plain Manchester | Receive | 1.67 Mbit/s with clock recovery (30 cycles a bit) | Firmware | `standard/manch_rx.s`, `advanced/manch_rx.s` | Benches `standard.py`, `manchester.py`; no sigrok decoder |
| 8-bit parallel, IEEE 1284 timing | Capture | A byte every 28 cycles | Firmware | `extra/parallel.s` | Bench `extra.py`; sigrok `parallel` |
| PRBS and scrambling (PRBS7, and ITU-T O.150's 2^9 - 1 and 2^15 - 1) | Generator and scrambler; a second unit receives or descrambles | 50 Mbit/s sending, 25 Mbit/s receiving | Shift unit's LFSR | `advanced/lfsr_unit.s` | Bench `unit_small.py`, with a checker of the named recurrences; no sigrok decoder |

The other seven programs are not protocols: `standard/crc8.s`, `standard/crc16.s`, `standard/crc32.s` and `advanced/crc_unit.s` compute CRCs, checked against standard libraries and catalogue values; `standard/reflex.s` is a reaction test; `basic/memhog.s` and `standard/memstress.s` load memory beside other programs. The units also decode biphase mark and differential Manchester, which `test_unit_rdiff` checks at the pins; no program uses them yet.

## Setup

`scripts/setup.sh` installs the tool versions the online checks use, into `~/ttsetup` and `tt/`, then runs the program checks and the RTL tests. It needs Linux x86-64, git, make, Python 3.11 or 3.12 with venv, and Icarus Verilog 12.0 or later unless you pass `--install-iverilog`. The first run downloads about 0.6 GB, most of it the PDK, and uses 1.5 GB of disk. The installs take a minute or two on a fast connection; the program checks and the RTL tests after them take about 22 and 5 minutes (`--no-test` skips them). After it, `. ~/ttsetup/env.sh` prepares each new shell. `--install-iverilog` adds Icarus 13.0, which gate level needs, and `--harden` adds the container for a local build: `scripts/setup.sh --help` has the details. Run a local build from a shell with `env.sh`: without its `PDK_ROOT`, LibreLane downloads the PDK again, into `~/.ciel`.

## Tests

`cd test && make` runs the 49 cocotb tests at the pins under Icarus. It needs cocotb 2.0.1. `make GATES=yes` runs them on a netlist in `test/gate_level_netlist.v`, with the RTL core beside it for a pin-by-pin compare. [test/README.md](test/README.md) has the details.

## What is Tiny Tapeout?

Tiny Tapeout is an educational project that aims to make it easier and cheaper than ever to get your digital and analog designs manufactured on a real chip.

To learn more and get started, visit https://tinytapeout.com.

## Resources

- [FAQ](https://tinytapeout.com/faq/)
- [Digital design lessons](https://tinytapeout.com/digital_design/)
- [Learn how semiconductors work](https://tinytapeout.com/siliwiz/)
- [Join the community](https://tinytapeout.com/discord)
- [Build your design locally](https://www.tinytapeout.com/guides/local-hardening/)
