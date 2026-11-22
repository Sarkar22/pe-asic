# Programs

The programs of section 11.2 of [`docs/ISA.md`](../docs/ISA.md), 50 files, with the benches that check
them on the simulator in [`sim/`](../sim). Python 3, standard library only.

## Running

From the repository root:

```
python3 programs/check.py            # every bench: about five minutes on one core
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
  format, HDMI CEC, Chapweske's description of PS/2, the Philips I2S bus specification for I2S, and the
  TDM mode of TI's TLV320ADC3140 data sheet for TDM. Each bench says which numbers it uses. SPI sets no
  clock rate, so the SPI benches use the rates section 11.2 gives.
- Many checks run a program on every thread and at all four alignments of its start cycle. A thread
  whose deadline passes sets LATE, and most checks count LATE, ILL or a bus conflict as a failure.
- Several benches plant faults: glitches, NACKs, clock stretching, stuck lines, wrong CRCs and parities,
  host stalls. Some benches also run deliberately broken copies of a program and check that they fail.

The device models and decoders were written alongside the firmware, so a misreading shared by both would
not show. Nothing here has run on hardware.

## Checkers for 10BASE-T and USB low speed

Three modules in `bench/` were written from the standards before any program that uses them, so that a
misreading in the firmware can't be shared by its checker:

- `eth_check.py` checks a recorded 10BASE-T pair against IEEE 802.3 clauses 3, 4, 7 and 14 and RFCs 826,
  894, 791, 792 and 1122: Manchester code, bit rate and jitter, preamble and SFD, frame size, the FCS (with
  `zlib.crc32`), TP_IDL, link test pulses and their period, the inter-packet gap, collisions, and the ARP and
  ICMP echo replies field by field. It returns findings with their times.
- `usb_check.py` checks a recorded D+ and D- pair against USB 2.0 chapters 7 to 9: bit rate and jitter, SYNC,
  stuffing, PIDs, CRC5 and CRC16, packet format, EOP width, inter-packet delay, the device's response time,
  keep-alive, reset, and the control transfers (stages, toggles, NAK and retry, the address change after the
  status stage, the device descriptor). It can also run sigrok's `usb_request` on the same record.
- `frontend.py` models the board: an RS-485 transceiver of the ISL3176E kind (DI, DE, RO, with the data
  sheet's delays) and ideal magnetics for 10BASE-T, and the USB bus with its pull-up and pull-downs. Every
  assumption is stated in the module.

`eth_check_test.py` and `usb_check_test.py` are their self-tests, run by `check.py`: lines built from the
standards (the 802.3 FCS bit by bit, USB CRCs as section 8.3.5 describes them), one good record that passes
every check, and one bad record per check that fails exactly that check. Their checks count for
`bench/eth_check.py` and `bench/usb_check.py` in the summary.

## Layout

- `basic/`: UART, SPI, write-only I2C, WS2812 and 1-Wire, plus a memory load. Section 11.1's 101-, 97- and
  112-word four-thread images are built from these.
- `standard/`: the four programs of the 135-word set in section 11.1 (UART TX and RX, SPI, the full I2C
  controller), and more protocols written with them.
- `trimmed/`: UART TX, UART RX and the I2C controller cut down so that four programs fit one 128-word memory.
- `extra/`: six more protocols.
- `advanced/`: larger programs. Some use two or three threads, and `usb_dev.s`, `can.s`, `eth_arp.s`,
  `usb_enum.s`, `spi_unit.s`, `can_unit.s`, `sd_unit.s`, `i2s_ctl.s` and `i2s_tgt.s` need the 512-word
  memory.
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
| `standard/i2c.s` | I2C controller, read and write | Against `standard/i2c_target.s` on another thread at 100 and 400 kHz, also with a wrong address first; against an EEPROM model, a write and a read after a wrong address and after a bus fault, with no word left in MIN; in a 512-word image with three other programs, matching its solo run |
| `standard/i2c_write.s` | I2C controller, write only | 100 and 400 kHz on four threads with UM10204 intervals; stretching at four points; NACK on address and data; stretch timeout reported |
| `standard/i2c_target.s` | I2C target, 24C02 style | Against `standard/i2c.s` on another thread: write, random reads, wrong address, UM10204 intervals at 100 and 400 kHz |
| `standard/ws2812.s` | WS2812 | One byte on four threads: T0H 200 to 500 ns, T1H 550 to 850 ns, 62 cycles per bit |
| `standard/onewire.s` | 1-Wire controller | Reset of 480 us, presence and a written byte on four threads; no device |
| `standard/jtag.s` | JTAG controller | IDCODE, IR load and a BYPASS loop against a TAP model at half periods 25 and 16 on four threads |
| `standard/cec.s` | HDMI CEC initiator | A two-block frame acknowledged with every bit window checked, on all four threads at once, each on its own pin; no follower reported; a block refused mid-frame, then the frame's last block taken unsent and the next frame whole; a broadcast that every follower accepts, and one that a follower rejects mid-frame; another initiator's frame waited out; 7 bit periods of free line before every start bit |
| `standard/crc16.s` | CRC-16/CCITT-FALSE | Check value of "123456789" |
| `standard/crc8.s` | CRC-8/MAXIM | Check value of "123456789" |
| `standard/crc32.s` | CRC-32 | Check value of "123456789" |
| `standard/hdlc_tx.s` | HDLC-style framing | NRZI, bit stuffing and CRC-16/X-25 decoded from the pad; bit time posted late without LATE |
| `standard/manch_rx.s` | Manchester receive, fixed format | Five bytes at sender clock error 0, 5% and 10% either way |
| `standard/reflex.s` | Reaction test | A pre-armed output answers an input edge in exactly 3 cycles, pad to pad, on every thread and phase |
| `standard/memstress.s` | Memory load | A thread hammering `ld` and `st` changes nothing in the UART transmitter's pad trace |
| `trimmed/uart_tx.s` | UART 8N1 transmit | 434 and 13 cycles per bit; with the other two and `standard/spi.s` in one 128-word image, matching its solo run |
| `trimmed/uart_rx.s` | UART 8N1 receive | 434 and 50 cycles per bit; in the 128-word image |
| `trimmed/i2c.s` | I2C controller, read and write | Against an EEPROM model: write, random reads, wrong address, data NACK, stretching, SDA stuck, SCL stuck at STOP, a write and a read after a wrong address and after a bus fault, UM10204 intervals at 100 and 400 kHz; in the 128-word image |
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
| `advanced/can.s` | CAN 2.0A node | Receive with stuffing and CRC-15, a stuff bit after the CRC sequence, ACK in its slot, phases, sender 2% off, transceiver delay, CRC and stuff errors; transmit; arbitration won and lost; each direction alone on 128 words |
| `advanced/eth_arp.s` | 10BASE-T node at 50 MHz: ARP (RFC 826) and ICMP echo (RFC 792) replies, link test pulses | Against a partner line built from IEEE 802.3 and the RFCs, through the RS-485 transceiver model, judged by `bench/eth_check.py`: every reply field by field, the transmit pair against clause 14 (Manchester, bit rate and edge deviation, preamble and SFD, FCS, TP_IDL, link pulses every 8 to 24 ms, the gap before each frame), at three transceiver corners; frames for another station (one with an address one word from ours for each of its three words), runts (46, 62 and 63 octets), FCS errors, ARP for another IP, bad checksums, fragments, multicast, full-size frames, an FLP burst, requests 96 and 47 bit times apart, all accounted for by the program's counters; partner clock 0.01% fast and slow, 12-bit preambles, +/-8 ns jitter at 8 sampling phases, 9 thread pairs and 4 alignments; both halves of every cycle driven and recorded (ISA 3.1): every transmit edge on the 50 ns grid within the transceiver's own skew (THALF), at each corner, and the edges with the clock's falling edge 1, 2 and 3.3 ns late reported; +/-12 ns random jitter and bit-boundary edges 10 ns late with mid-bit ones 10 ns early, at 8 phases and 3 corners, no frame lost (RKIND 3); at 8 more phases and 3 corners, no frame lost at 802.3's +/-13.5 ns random, with bit-boundary edges 10 to 13.5 ns late and mid-bit ones as early or the reverse at 12 and 13.5 ns, with the reverse at 13.5 ns over 80 phases 0.25 ns apart at the slowest corner (its 19 ns pulses), with the clock's falling edge 1 and 2 ns late, with the partner 0.01% fast and slow on 1518-octet frames, clean and with the 13.5 ns pattern, or with a phase step of up to 17 ns after a random bit; the clock 3.3 ns off, steps of 20 and 23 ns and 7.5.2.2's +/-18 ns test signal reported; frames with 1 to 7 bits after their FCS kept and answered, and dropped with a bad FCS, as 802.3 4.2.9 truncates a frame (REOP's byte end); a request that arrives while the reply goes out; an ARP request padded to 1518 octets; a lone fragment, an echo request with an IP option and one too big for the buffer left unanswered; 17 broken copies caught |
| `advanced/usb_enum.s` | USB low-speed device on endpoint 0 at 50 MHz: the control transfers of enumeration (GET_DESCRIPTOR of the device descriptor and of the configuration, SET_ADDRESS), with the shift units doing NRZI, bit stuffing, the CRC5 and CRC16 checks, the CRC16 of its own data, the end of packet and the clock recovery | Against a host model at the device's pins, through the bus model of `bench/frontend.py`, judged by `bench/usb_check.py`: the enumeration a host does after attach, with two 10 ms resets, a keep-alive every frame, the new address taken after the status stage and the old one then ignored; 8 thread placements, 4 alignments and 8 sampling phases; the host 0.05% fast and slow, edge jitter of +/-22 and +/-70 ns, every inter-packet delay at 2 bit times, EOPs of 1.25 and 1.50 us and of 670 ns at 5 phases; the host's D- 100 and 200 ns behind its D+ (TLST 210 ns); the device's D- lagging its D+ by 40 ns, an 18 ns cable, a data packet ending in a stuffed 0; NAK when nothing is due, other addresses and endpoints ignored, corrupted SETUP data and corrupted SETUP, IN and OUT tokens unanswered, STALL for unsupported requests until the next SETUP; a SET_ADDRESS left in its status stage, then a request with no data stage, leaving the address at 0; lost ACKs, with identical retries and the old address kept until the status ACK; a SETUP and a reset in the middle of a transfer; bit-stuff errors from the host (seven and eight 1s) unanswered; one dribble bit before the EOP (USB 2.0 7.1.9.1) on each kind of host packet, alone and on every host packet of whole enumerations, and 2 to 6 on every host packet of an enumeration, answered as without them (the units' byte end, ISA 5.9.7), and bits that are not a dribble unanswered, among them bits that take the CRC5 and CRC16 registers to the values a dribble bit gave, a 0 after an IN token whose CRC5 is wrong, and a 29-bit token whose CRC5 passes; GET_DESCRIPTOR(CONFIGURATION) with wLength 9, 18 and 255, and SET_CONFIGURATION STALLed; SETUP, IN and OUT tokens to endpoints 1, 2, 4 and 8 ignored; every answer 2 to 6.5 bit times after the host's EOP; the host's rate and jitter beyond the standard reported; 18 broken copies caught |
| `advanced/uart_bypass.s` | UART 8N1 pass-through on the bypass lane of `uo4`, one byte replaced in flight | 4 threads x 4 alignments x 4 sender phases at 16 (3.125 Mbaud) and 17 cycles per bit, 2 alignments at 25 and 50, and a few runs at 434, 5208 and 6553: `uo4` equals `ui4` on every cycle except the chosen frame's data bits, which carry the host's byte on that frame's own bit boundaries; the lane switches 3 cycles into the start bit and 9 into the stop bit, with `ui4`, OUT and `uo4` steady 3 cycles either side; a frame rule and a byte rule, no rule, two rules in turn; both lines decoded; 5 broken copies caught |
| `advanced/spi_unit.s` | SPI target on the shift unit, modes 0 and 3, MSB first: the receive half samples MOSI at SCK's rising edges, the send half moves MISO after its falling edges, and CS low starts each frame | Against a controller modelled on the RP2350's SPI block (PL022, Motorola frame format, RP2350 datasheet 12.3.4.10 and 12.3.4.13): mode 0 with CS pulsed high for one SCK period between bytes at half periods 8, 10, 25 and 60, and mode 3 with CS held low for transfers of 40 to 80 bits at half periods 4, 10 and 25, on every thread and alignment at 2 sampling phases; frames of 1 to 100 bits with CS held low; one endless reception at half periods 3.25 to 4, and at half period 2 with MISO moved after the capture edge; a frame cut short ended by RXEND, and without RXEND the frames after it back in step at the next CS; every MOSI word and every MISO bit compared with the stimulus, MISO setup and hold measured at each capture; the shortest CS high time between framed bytes at half period 8 (11 cycles), and the first clean half period of the endless reception (3) |
| `advanced/can_unit.s` | CAN 2.0A node in one thread on the shift unit: the unit stuffs, destuffs, appends and checks the CRC-15 and times every bit; the firmware writes RXCNT once the DLC is in, monitors its own bits and arbitrates, drives the ACK slot and sends error flags | On a wired-AND bus with transceiver delays (dominant and recessive edges apart), against frames built from the Bosch specification (`bench/can_wire.py`), another node's ACK and a rival that arbitrates by the wire (`bench/can.py`): frames of 0 to 8 bytes with all-zero and all-one stuffing, stuff bits after the CRC sequence, remote frames with DLC 0, 3 and 8, back to back at the shortest interframe space at 500 kbit/s and 1 Mbit/s, every thread and alignment, the sender 1.5% fast and slow, transceiver loops of 80 to 440 ns, dominant bits stretched by 40 to 480 ns; every reported word and end flag against the frame, the ACK in its slot on the bus; a corrupted CRC (error flag after the ACK delimiter), six equal bits (error flag), nobody to ACK (error flag after the ACK slot); transmit at 500 kbit/s and 1 Mbit/s, a stuff bit after the CRC, four frames back to back with each SOF at least 11 recessive bit times after the ACK slot before it (ACK delimiter, end of frame, intermission); arbitration lost to a rival that starts on our SOF or 1 to 20 cycles before it (the winner's frame received with a good CRC, acknowledged, and ours sent again) and won; the limit of a dominant stretch at RXD (24 cycles clean, 26 not) |
| `advanced/sd_unit.s` | SD card host in SD mode, 1-bit bus, on two threads' shift units: one sends each command with its CRC7 and the clock and receives the response, the other receives the data block on DAT0 with its CRC16; the host reads responses and blocks from memory | Against a card model written from the SD Physical Layer Simplified Specification (`bench/sd_card.py`; its CRC7 and CRC16 reproduce the examples of 4.5): the initialization CMD0, CMD8 (R7), CMD55 and ACMD41 (R3), CMD2 (R2) and CMD3 (R6) at 397 kHz on 4 thread pairs and alignments, then CMD7 and CMD17 with a 512-byte block at 6.25 MHz; reads on all 12 thread pairs at 8.33 MHz and at 6.25 and 0.5 MHz; NCR 2 to 64 clocks; blocks that start while the response is still on CMD and up to 1000 clocks later; card output delays of 0 to 2 half periods; two reads back to back; every command checked by the card, every response and block compared bit for bit, the chip's CRC checks against the reference's, 74 power-up clocks, 8 clocks between commands, no line driven from both ends; a flipped data bit, a block end bit of 0, a response with a flipped CRC, argument or end bit, a card that does not answer CMD8; the fastest clock (8.33 MHz; at 12.5 MHz the stopped clock is reported) and the output delay window that loses the start bit |
| `advanced/ow_unit.s` | 1-Wire master on the shift unit, standard speed: a shift output on an open-drain `io` pin makes every reset pulse and time slot from words, the receive half samples each slot, and the CRC register checks the CRC-8 of the bytes read | Against a 1-Wire device with a ROM code, written from the DS18B20 data sheet's signalling section (`bench/unit_small.py`): Read ROM of Maxim's worked example (CRC byte 0xA2) on every thread and alignment and of random ROM codes back to back, a flipped ROM bit (the check fails), no device, 0 and 4 bytes read; device write samples at 15 and 60 us, a read 0 held 15.5 and 60 us, presence 15 and 60 us after the release and 60 and 240 us long, a 5 us response, line rise times to 6.8 us; every reset, slot, low time and recovery measured against the data sheet; the limits (a 7.04 us rise, a read 0 held 12.9 us) |
| `advanced/uart_par_unit.s` | UART 8E1 and 8O1 on two shift units: one appends the parity bit from its CRC register, the other checks the parity and the stop bit with its own | Loopback on `io0` at 115200 baud to 16.7 Mbaud (434 to 3 cycles per bit) on 8 thread pairs and alignments, the line decoded and every received word and flag compared; a sender model on `ui4` with wrong parity bits and stop bits of 0 at 434, 50 and 17.3 cycles per bit, and 4% fast and slow; every thread pair at 3 cycles per bit, and the underrun at 2 |
| `advanced/ws2812_unit.s` | WS2812B through the shift unit: three symbols a bit (SYMS 2), output codes 1, the bit, 0, MSB first, TPER 20 + TFRAC 213 | Every high and low time against the WS2812B data sheet's windows, on every thread and alignment; bit k within one cycle of k x 1.25 us; frames of 1 to 4 LEDs with the latch gap; TPER 16 to 25 with no fraction (18 to 23 pass) and 17 + TFRAC 128; a host too slow cuts the frame |
| `advanced/lfsr_unit.s` | PRBS7, PRBS9 and PRBS15 from the unit's LFSR (CMODE 2), and an additive scrambler between two units | A checker built from each Fibonacci generator polynomial (PRBS7 x^7 + x^6 + 1; ITU-T O.150's x^9 + x^5 + 1 and x^15 + x^14 + 1) on the line: 0 errors over 1024 bits at 2 to 10 cycles a bit on 4 thread pairs, and over a whole period at 50 Mbit/s; the complement with output code 1; frames scrambled behind a sync byte and descrambled by the other unit for each width, the line XOR the data checked as the PRBS; a descrambler of the wrong width |
| `advanced/crc_unit.s` | The idle send half as a CRC engine, one bit a cycle, read with `srd hi` and `srd lo` | CRC-32 of "123456789" (0xCBF43926) and of buffers of 1 to 256 bytes against `zlib.crc32` on every thread, no underrun; CRC-8/MAXIM of "123456789" (0xA1) and of a 1-Wire ROM code with its CRC byte (0) on a 128-word memory |
| `advanced/xy2_unit.s` | XY2-100 controller on three shift units started at one cycle: one makes CLK and SYNC, the others send the X and Y positions with even parity appended | A reader of the XY2-100 frame (bits read at CLK's falling edges, control bits 0 0 1, positions MSB first, even parity, SYNC low on the parity bit only): 6 frames back to back on 8 thread placements and alignments, extreme positions, a single frame; SYNC, X and Y change only in cycles where CLK rises, and CLK rises every 25 cycles (2 Mbit/s) from the shared start cycle; a start cycle the threads reach too late is seen |
| `advanced/i2s_ctl.s` | I2S and TDM controller on two shift units started at one cycle: one makes SCK and WS (FS) from a pattern table, the other sends SD and receives the target's SD at SCK's rising edges | Against a slave transmitter model and readers written from the Philips I2S bus specification and the TLV320ADC3140's TDM mode (TX_OFFSET 1, BCLK_POL 1): I2S at 48 kHz with 2 x 32 and 2 x 16 bits and at 8 kHz, TDM at 48 kHz with 8 x 16 and 4 x 32 bits and at 8 kHz with one 8-bit slot, on 8 thread pairs and alignments; every word on the bus and every received bit, the I2S Table 1 timing (and the TDM data sheet's) on every edge; the receive half's start from TPER 3 to 5 against TFRAC; TPER 2 with the bus right and the receive start missed; 3 broken copies caught |
| `advanced/i2s_tgt.s` | I2S and TDM target on one shift unit: the receive half samples SD at SCK's rising edges, the send half moves SD out after them, and a wait on SCK and WS (FS) together starts both on each frame's first edge | Against a controller model written from the same documents, with sub-cycle phases and WS (FS) and SD 0 to 2 cycles after its falling edges: I2S at 48 kHz with 2 x 32 and 2 x 16 bits and at 8 kHz, TDM at 48 kHz with 4 x 16 and 2 x 32 bits and at 8 kHz with one 8-bit slot, on 4 threads, alignments and phases; every received frame, SD out as the controller latched it, setup at least 0.2T and hold at least 0 (8 ns for TDM); the restart limit (half period 5 clean, 4.5 not), TDM at 128 fs failing it, and a single frame at half period 3; 3 broken copies caught |
