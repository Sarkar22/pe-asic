# Programs

The programs of section 11.2 of [`docs/ISA.md`](../docs/ISA.md), 47 files, with the benches that check
them on the simulator in [`sim/`](../sim). Python 3, standard library only.

## Running

From the repository root:

```
python3 programs/check.py            # every bench: about two minutes on one laptop core
python3 programs/check.py can usb    # only the benches whose names contain one of the words
```

Each check prints one `PASS` or `FAIL` line under a `---` line naming the program or programs it
counts for. A table at the end gives the number of checks per program. The exit status is non-zero if
any check fails, a bench crashes, or a program has no check. `.github/workflows/programs.yaml` runs the
same command on every push and pull request.

## How the benches check a program

- The firmware sees only its pads and the mailbox, as on the chip. The bench loads it with the host
  calls of `sim/sim.py` and talks to it through the mailbox words.
- Device models (a UART sender, SPI and I2C targets, an EEPROM, a TAP chain, an SWD target, a PS/2 host,
  USB and CAN nodes, and so on) react to pad levels only. The board model is a wired-AND bus with pull-ups,
  and some benches add a rise time, skew, jitter or clock error.
- Decoders take the recorded pin trace and the protocol's own parameters and rules, never a constant or
  label of the program. Timing limits come from the protocol documents: UM10204 for I2C, the WS2812B
  and DS18B20 datasheets, ANSI E1.11 for DMX512, IEEE 1149.1 for JTAG, ADIv5 for SWD, USB 2.0 for low
  speed, the Bosch CAN 2.0 specification, IEEE 802.3 Clause 22 for MDIO, LIN 2.1, IEEE 1284, the NEC IR
  format, HDMI CEC, and Chapweske's description of PS/2. Each bench says which numbers it uses. SPI sets no
  clock rate, so the SPI benches use the rates section 11.2 gives.
- Many checks run a program on every thread and at all four alignments of its start cycle. A thread
  whose deadline passes sets LATE, and most checks count LATE, ILL or a bus conflict as a failure.
- Several benches plant faults: glitches, NACKs, clock stretching, stuck lines, wrong CRCs and parities,
  host stalls. Some benches also run deliberately broken copies of a program and check that they fail.

The device models and decoders were written alongside the firmware, so a misreading shared by both would
not show. Nothing here has run on hardware.

## Layout

- `basic/`: UART, SPI, write-only I2C, WS2812 and 1-Wire, plus a memory load. Section 11.1's 101-, 97- and
  112-word four-thread images are built from these.
- `standard/`: the four programs of the 135-word set in section 11.1 (UART TX and RX, SPI, the full I2C
  controller), and more protocols written with them.
- `trimmed/`: UART TX, UART RX and the I2C controller cut down so that four programs fit one 128-word memory.
- `extra/`: six more protocols.
- `advanced/`: larger programs. Some use two or three threads, and `usb_dev.s` and `can.s` need the
  512-word memory.
- `bench/`: the benches. `check.py` runs the modules listed in its `BENCHES`.

Names repeat across folders, as different code, with two exceptions: `basic/uart_tx.s`,
`standard/uart_tx.s` and `sim/uart_tx.s` are the same file, and so are `standard/i2c.s` and `extra/i2c.s`.
Each program's header comment gives its mailbox words and pins.

## Programs

Every program also has a check that it assembles with `sim/asm.py` to the word count section 11.2 lists.
The six protocol programs in `basic/` also run at 40 and 45 MHz with their timing constants rescaled,
and their benches must reject broken copies of them (wrong bit count, wrong timing, a missed NACK, a
push-pull line on an open-drain bus).

| Program | Protocol | What the bench checks |
|---|---|---|
| `basic/uart_tx.s` | UART 8N1 transmit | 115200 baud and 1 Mbaud on every thread and alignment, 12 and 32767 cycles per bit, 9600 baud, decoded from the pin |
| `basic/uart_rx.s` | UART 8N1 receive | 115200 baud and 1 Mbaud at 8 start phases, sender 2% fast and slow over 64 abutting bytes, a glitch ignored, a framing error reported |
| `basic/spi.s` | SPI controller | Modes 0 to 3 on every thread and alignment at half periods 9, 25 and 250 cycles; MOSI and MISO decoded at the pads |
| `basic/i2c_write.s` | I2C controller, write only | 100 and 400 kHz with every UM10204 interval measured at the pad, with and without rise time; stretching on every clock and after each ACK; data NACK; wrong address; tBUF over three transactions; every thread and alignment |
| `basic/ws2812.s` | WS2812 | 72 bits on every thread and alignment inside the T0H and T1H windows, two frames with the latch gap |
| `basic/onewire.s` | 1-Wire controller | Reset and presence, 5 bytes written, no device, 12 device timing corners, every thread and alignment |
| `basic/memhog.s` | Memory load | Runs next to UART TX, I2C and 1-Wire in a 512-word image; their pins and host traffic match their solo runs to the cycle |
| `standard/uart_tx.s` | UART 8N1 transmit | 434, 5208, 13 and 12 cycles per bit on four threads, every edge on the bit grid |
| `standard/uart_rx.s` | UART 8N1 receive | 434 and 50 cycles per bit on four threads; bytes read correctly when data is good only 0 to 2 cycles either side of mid-bit, at 8 start phases; glitch rejected; framing error flagged |
| `standard/spi.s` | SPI controller | All four modes at half periods 25, 10 and 9, each on every thread and alignment |
| `standard/i2c.s` | I2C controller, read and write | Against `standard/i2c_target.s` on another thread at 100 and 400 kHz; in a 512-word image with three other programs, matching its solo run |
| `standard/i2c_write.s` | I2C controller, write only | 100 and 400 kHz on four threads with UM10204 intervals; stretching at four points; NACK on address and data; stretch timeout reported |
| `standard/i2c_target.s` | I2C target, 24C02 style | Against `standard/i2c.s` on another thread: write, random reads, wrong address, UM10204 intervals at 100 and 400 kHz |
| `standard/ws2812.s` | WS2812 | One byte on four threads: T0H 200 to 500 ns, T1H 550 to 850 ns, 62 cycles per bit |
| `standard/onewire.s` | 1-Wire controller | Reset of 480 us, presence and a written byte on four threads; no device |
| `standard/jtag.s` | JTAG controller | IDCODE, IR load and a BYPASS loop against a TAP model at half periods 25 and 16 on four threads |
| `standard/cec.s` | HDMI CEC initiator | A two-block frame acknowledged with every bit window checked, on all four threads at once, each on its own pin; no follower reported |
| `standard/crc16.s` | CRC-16/CCITT-FALSE | Check value of "123456789" |
| `standard/crc8.s` | CRC-8/MAXIM | Check value of "123456789" |
| `standard/crc32.s` | CRC-32 | Check value of "123456789" |
| `standard/hdlc_tx.s` | HDLC-style framing | NRZI, bit stuffing and CRC-16/X-25 decoded from the pad; bit time posted late without LATE |
| `standard/manch_rx.s` | Manchester receive, fixed format | Five bytes at sender clock error 0, 5% and 10% either way |
| `standard/reflex.s` | Reaction test | A pre-armed output answers an input edge in exactly 3 cycles, pad to pad, on every thread and phase |
| `standard/memstress.s` | Memory load | A thread hammering `ld` and `st` changes nothing in the UART transmitter's pad trace |
| `trimmed/uart_tx.s` | UART 8N1 transmit | 434 and 13 cycles per bit; with the other two and `standard/spi.s` in one 128-word image, matching its solo run |
| `trimmed/uart_rx.s` | UART 8N1 receive | 434 and 50 cycles per bit; in the 128-word image |
| `trimmed/i2c.s` | I2C controller, read and write | Against an EEPROM model: write, random reads, wrong address, data NACK, stretching, SDA stuck, SCL stuck at STOP, UM10204 intervals at 100 and 400 kHz; in the 128-word image |
| `extra/spi_target.s` | SPI target, mode 0 | Half periods 10, 25 and 250 at 12 start phases: bytes received, and MISO valid at every rising edge |
| `extra/parallel.s` | 8-bit parallel capture | IEEE 1284 minimum timing at 12 phases; data held 0 cycles after the strobe; a byte every 28 cycles |
| `extra/swd.s` | SWD, read of DPIDR | Line reset and request as ADIv5 frames them; ACK OK, ACK WAIT, bad read parity; no contention at the turnarounds |
| `extra/i2c.s` | I2C controller, read and write | The same cases as `trimmed/i2c.s` |
| `extra/lin.s` | LIN responder header | Identifiers with the commander's clock 14% fast and slow; bad parity rejected |
| `extra/nec_tx.s` | NEC IR transmit | Carrier frequency and duty, leader, bit times and the 32-bit frame, from the pad |
| `advanced/manch_rx.s` | Manchester receive with clock recovery | Four rates on every thread at several phases, a 600-byte frame with the sender 2% fast, runs of 0x00 and 0xFF, three frames, line idling high |
| `advanced/dmx_tx.s` | DMX512 transmit | A full 512-slot universe decoded at the pad with break and MAB measured; every thread and alignment; a 1.4 ms host stall |
| `advanced/dmx_rx.s` | DMX512 receive | Slot windows at 1, 250 and 509; shortest legal break and MAB; long break; sender 2% fast and slow; marks between slots; non-zero start code ignored |
| `advanced/ps2_dev.s` | PS/2 device | Sending with clock and data timing in range on every thread and alignment; host commands FF, F2, ED, EE and FE; bad parity; host inhibit and request-to-send; slow rise |
| `advanced/mdio_sta.s` | MDIO station, Clause 22 | Read, write and read back at half periods 20, 50 and 250 with PHY output delay; every thread and alignment |
| `advanced/mdio_phy.s` | MDIO PHY, Clause 22 | Follows MDC at half periods 100 down to 20; MDIO moves 3 cycles after MDC rises |
| `advanced/jtag.s` | JTAG scan engine | Two-TAP chain IDCODEs, IR scan, BYPASS and Pause-DR at half periods 250 down to 9, TAP states walked by a pad decoder; every thread and alignment |
| `advanced/swd.s` | SWD host | DPIDR, writes and posted AP reads at 0.1 and 1 MHz; LATE reported past the limit; WAIT, FAULT, no response and parity errors; contention checked every cycle |
| `advanced/i2c_tgt.s` | I2C target, 32 bytes | Write and random read at 100 and 400 kHz with rise times; ACK 3 cycles after the SCL fall; wrong addresses ignored; repeated START; pointer wrap |
| `advanced/i2c_fp.s` | I2C target, 1 MHz build | At the UM10204 Fast-mode Plus minimums with rise time up to 120 ns: bytes right and SDA valid within 0.45 us of each SCL fall |
| `advanced/usb_tx.s` | USB low-speed transmit | Handshake, data and token packets decoded with NRZI, stuffing and EOP; every thread and alignment; every edge on the 33-cycle grid |
| `advanced/usb_rx.s` | USB low-speed receive | Random packets, alignments, D+ to D- skew up to 160 ns, rate error up to 1.5%, jitter up to 60 ns, 2-bit gaps |
| `advanced/usb_dev.s` | USB low-speed device link | SETUP and DATA0 with CRC-16, ACK, NAK, IN with DATA1, address filter, bad CRC, turnaround inside 6.5 bit times |
| `advanced/can.s` | CAN 2.0A node | Receive with stuffing and CRC-15, ACK in its slot, phases, sender 2% off, transceiver delay, CRC and stuff errors; transmit; arbitration won and lost; each direction alone on 128 words |
