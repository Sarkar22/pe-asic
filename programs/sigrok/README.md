# Bus checks with sigrok

The benches in [`../bench/`](../bench) check every program with device models and decoders written alongside the
firmware, so a misreading shared by a program and its bench would not show. This folder checks the same runs with
the protocol decoders of [sigrok](https://sigrok.org/) (libsigrokdecode), which other people wrote from the
protocol documents. Python 3 standard library, plus `sigrok-cli`.

## Running

From the repository root:

```
python3 programs/sigrok/check.py            # every check on sim/sim.py: about 3 minutes on one core
python3 programs/sigrok/check.py can usb    # only the groups whose names contain one of the words
python3 programs/sigrok/check.py --list     # the groups
python3 programs/sigrok/check.py --rtl      # on the RTL through verif/rtl_chip.py, checked against sim/sim.py
```

- `sigrok-cli` comes from the distribution (`apt-get install sigrok-cli` on Debian and Ubuntu). The checks were
  written against sigrok-cli 0.7.2 with libsigrokdecode 0.5.3, Ubuntu 24.04's. Without `sigrok-cli` the script
  prints `SKIP` and exits 0.
- `--rtl` needs the `stub` Verilator models of [`verif/Makefile`](../../verif/Makefile) at depths 128 and 512, in
  `verif/build/`, or in the folder `--build DIR` or `$PE_BUILD` names.
- Each decoded run leaves a VCD file in `programs/sigrok/work/sim/` or `work/rtl/` (git-ignored; `--work DIR`
  puts them elsewhere). Open one in PulseView, or run `sigrok-cli -i <file> -P <decoder>` on it.
- Each check prints one `PASS` or `FAIL` line under a `---` line naming its program or programs, then a table per
  program. The exit status is non-zero if any check fails or a group crashes. `--json FILE` writes the table.

## How a run is checked

1. **The record.** A subclass of the chip class (`sim.Chip`, or `rtl_chip.Chip` under `--rtl`) takes the place of
   `report.Chip` before the benches are imported, so every bench builds its board on it unchanged. In every cycle
   it keeps what a logic analyzer on the board would see: the 8 `uo` pins, the 8 `ui` pins as the device models
   drive them, and the levels of the 8 `uio` lines, which are the wired-AND of the chip's pads, the device models'
   pulls and the pull-ups.
2. **The VCD file.** Those 24 lines, named `uo0`..`uo7`, `ui0`..`ui7` and `uio0`..`uio7`, with a 10 ns timescale
   and two ticks per 20 ns cycle (sigrok takes a timescale of 1, 10 or 100 units), so sigrok samples at 100 MHz.
3. **The scenario** is the bench's own: its case function or scenario code, its device models, its rates, threads
   and alignments. Where a bench's run stops right after the last event, the scenario runs the board on, idle,
   long enough for sigrok to close it (a WS2812 reset, the end of a 1-Wire slot, CEC's ACK timeout, two idle LIN
   characters, a PS/2 host inhibit). Where a bench always starts thread 0, the scenario starts the program on
   other threads too.
4. **sigrok-cli** decodes the file with a decoder stack whose options come from the protocol and from the rate the
   scenario puts on the wire (a baud rate, an SPI mode, the CAN bit rate with a 75% sample point, USB low speed,
   the NEC carrier), never from a program's constants.
5. **The expected payload** comes from the stimulus: the words the host posts to the mailbox, the replies a device
   model is given, a target's memory, the frames a sender model is told to send, and the protocol's own answers
   (a PS/2 keyboard's FA, an EEPROM's random read). sigrok's payload must equal it.
6. **sigrok's error annotations** fail a good run: framing and parity errors, NACKs where an ACK is due, CRC5 and
   CRC16 errors, bad SYNC or PID, invalid DMX start and stop bits, 1-Wire timing warnings, CAN field warnings, and
   so on. Where the stimulus plants a fault (a glitch, a low stop bit, a corrupted CRC, a wrong parity, an absent
   device), the fault must show, in its place.
7. **The bench's own decoder**, where the bench has one for that bus (UART, SPI, I2C, WS2812, 1-Wire, DMX512, MDIO,
   JTAG, SWD, USB, CAN), runs on the same record and must agree with sigrok.
8. **Under `--rtl`** each scenario runs on the simulator first, then on the RTL, and the RTL's record, every pad and
   board level of every cycle, must equal the simulator's. The decode is of the RTL's record.
9. **HDMI CEC's signal free time** (HDMI CEC 1.4, CEC 9.1) is measured on the record at every start bit that sigrok
   finds in a frame of the program's: the line must have been high for 7 bit periods (16.8 ms) before it falls,
   and it must fall 7 bit periods after the end of the program's previous frame, or 5 after another initiator's.
   A frame ends one 2.4 ms bit period after the fall of its last ACK bit, where sigrok puts its last ACK annotation.

A run where sigrok and the stimulus differ in a way this README documents (below) is reported on a `DIFF` line
with its VCD file, and does not fail. Any other difference fails.

## Results

One run with `--rtl`, on the RTL of `src/` at commit 0d6769e: **291 checks, 291 PASS**. 684 scenario runs (637 with
sigrok, 36 of the CRC programs, 11 teeth) and 738 decodes by sigrok, one VCD file each (a run may decode several
buses, or one window per packet). Of the 727 decodes of scenarios, 705 are as the stimulus wants and 22 are known
differences, listed below, on the simulator and on the RTL alike; the other 11 are the teeth, all caught. In all 684
runs the RTL's record equals the simulator's, every pad and board level of every cycle: 124,281 changes over 74.8
million cycles in the runs with sigrok alone. The simulator pass alone is 175 checks in 164 s; the `--rtl` run, which
runs every scenario on both, takes 648 s, on one core. The CEC runs take 388 s of that: a CEC bit is 2.4 ms,
120,000 cycles.

Per program: a run of an image with several programs counts for each program of its group, and each decode for the
program whose bus it is (and for a memory load running beside it), so shared runs count more than once. Simulator
and RTL count the decodes that pass, of those made; Time is both chips. "items" mixes the units of the groups a
program appears in (bytes, events, frames).

| Program | sigrok decoders and options | Scenarios | Decoded | Simulator | RTL | Time |
|---|---|---|---|---|---|---|
| `basic/i2c_write.s` | i2c | 34 | 388 items | 34 of 34 | 34 of 34 | 5 s |
| `basic/memhog.s` | uart (baudrate=115200); i2c; onewire_link > onewire_network | 1 | 23 items | 3 of 3 | 3 of 3 | 0 s |
| `basic/onewire.s` | onewire_link > onewire_network | 24 | 66 items | 24 of 24 | 24 of 24 | 13 s |
| `basic/spi.s` | spi (cpol=0/1 cpha=0/1) | 99 | 804 items | 99 of 99 | 99 of 99 | 7 s |
| `basic/uart_rx.s` | uart (baudrate=115200/1000000) | 22 | 394 items | 22 of 22 | 22 of 22 | 5 s |
| `basic/uart_tx.s` | uart (baudrate=115200/1000000/4166667/9600/1526) | 39 | 217 items | 39 of 39 | 39 of 39 | 7 s |
| `basic/ws2812.s` | rgb_led_ws281x | 10 | 850 items | 10 of 10 | 10 of 10 | 2 s |
| `standard/cec.s` | cec | 6 | 183 frames | 21 of 21 | 21 of 21 | 388 s |
| `standard/crc16.s` | no decoder: binascii.crc_hqx | 16 | - | 16 of 16 | 16 of 16 | 0 s |
| `standard/crc32.s` | no decoder: zlib.crc32 | 16 | - | 16 of 16 | 16 of 16 | 0 s |
| `standard/crc8.s` | no decoder: catalogue check value 0xA1 | 4 | - | 4 of 4 | 4 of 4 | 0 s |
| `standard/hdlc_tx.s` | no decoder: libsigrokdecode has no HDLC | 0 | - | - | - | 0 s |
| `standard/i2c.s` | i2c > eeprom24xx | 5 | 149 items | 5 of 5 | 5 of 5 | 1 s |
| `standard/i2c_target.s` | i2c > eeprom24xx | 4 | 128 events | 4 of 4 | 4 of 4 | 1 s |
| `standard/i2c_write.s` | i2c | 15 | 94 items | 15 of 15 | 15 of 15 | 1 s |
| `standard/jtag.s` | jtag | 8 | 96 scans | 8 of 8 | 8 of 8 | 0 s |
| `standard/manch_rx.s` | no decoder: no plain Manchester decoder | 0 | - | - | - | 0 s |
| `standard/memstress.s` | uart (baudrate=115200) | 1 | 1 bytes | 1 of 1 | 1 of 1 | 0 s |
| `standard/onewire.s` | onewire_link > onewire_network | 5 | 9 events | 5 of 5 | 5 of 5 | 3 s |
| `standard/reflex.s` | no decoder: a reaction time, not a protocol | 0 | - | - | - | 0 s |
| `standard/spi.s` | spi (cpol=0/1 cpha=0/1) | 51 | 108 items | 51 of 51 | 51 of 51 | 1 s |
| `standard/uart_rx.s` | uart (baudrate=115200/1000000/500000) | 12 | 42 items | 12 of 12 | 12 of 12 | 1 s |
| `standard/uart_tx.s` | uart (baudrate=115200/9600/4166667/3846154) | 19 | 45 items | 19 of 19 | 19 of 19 | 4 s |
| `standard/ws2812.s` | rgb_led_ws281x | 4 | 32 bits | 4 of 4 | 4 of 4 | 1 s |
| `trimmed/i2c.s` | i2c > eeprom24xx | 12 | 234 items | 12 of 12 | 12 of 12 | 5 s |
| `trimmed/uart_rx.s` | uart (baudrate=115200/1000000/500000) | 9 | 35 items | 9 of 9 | 9 of 9 | 1 s |
| `trimmed/uart_tx.s` | uart (baudrate=115200/3846154) | 9 | 34 items | 9 of 9 | 9 of 9 | 1 s |
| `extra/i2c.s` | i2c > eeprom24xx | 11 | 213 events | 11 of 11 | 11 of 11 | 5 s |
| `extra/lin.s` | uart (baudrate=22327/19201/16843) > lin | 6 | 180 events | 6 of 6 | 6 of 6 | 42 s |
| `extra/nec_tx.s` | ir_nec (polarity=active-high cd_freq=38000) | 2 | 10 fields | 2 of 2 | 2 of 2 | 45 s |
| `extra/parallel.s` | parallel (clock_edge=rising) | 12 | 84 bytes | 12 of 12 | 12 of 12 | 1 s |
| `extra/spi_target.s` | spi (cpol=0 cpha=0) | 36 | 432 bytes | 36 of 36 | 36 of 36 | 4 s |
| `extra/swd.s` | swd | 12 | 48 events | 12 of 12 | 12 of 12 | 4 s |
| `advanced/can.s` | can (nominal_bitrate=500000/497512/502513/495050/505051/490196/510204 sample_point=75.0) | 39 | 727 frames | 39 of 39 | 39 of 39 | 9 s |
| `advanced/dmx_rx.s` | dmx512 | 13 | 2172 slots | 33 of 33 | 33 of 33 | 33 s |
| `advanced/dmx_tx.s` | dmx512 | 10 | 795 slots | 28 of 28 | 28 of 28 | 14 s |
| `advanced/i2c_fp.s` | i2c | 15 | 525 events | 15 of 15 | 15 of 15 | 1 s |
| `advanced/i2c_tgt.s` | i2c | 15 | 525 events | 15 of 15 | 15 of 15 | 2 s |
| `advanced/jtag.s` | jtag | 13 | 208 scans | 13 of 13 | 13 of 13 | 2 s |
| `advanced/manch_rx.s` | no decoder: no plain Manchester decoder | 0 | - | - | - | 0 s |
| `advanced/mdio_phy.s` | mdio | 11 | 55 frames | 11 of 11 | 11 of 11 | 2 s |
| `advanced/mdio_sta.s` | mdio | 12 | 48 frames | 12 of 12 | 12 of 12 | 2 s |
| `advanced/ps2_dev.s` | ps2 | 15 | 64 frames | 32 of 32 | 32 of 32 | 25 s |
| `advanced/swd.s` | swd | 16 | 420 events | 16 of 16 | 16 of 16 | 4 s |
| `advanced/usb_dev.s` | usb_signalling (signalling=low-speed) > usb_packet (signalling=low-speed) | 9 | 73 packets | 9 of 9 | 9 of 9 | 1 s |
| `advanced/usb_rx.s` | usb_signalling (signalling=low-speed) > usb_packet (signalling=low-speed) | 9 | 51 packets | 9 of 9 | 9 of 9 | 1 s |
| `advanced/usb_tx.s` | usb_signalling (signalling=low-speed) > usb_packet (signalling=low-speed) | 15 | 15 packets | 15 of 15 | 15 of 15 | 1 s |
| `test/test.py` | uart (baudrate=1250000); usb_signalling (signalling=low-speed) > usb_packet (signalling=low-speed) | 3 | 19 packets | 3 of 3 | 3 of 3 | 0 s |

## Teeth

Broken copies of programs, made in memory from the source text as `bench/basic.py`'s own checks do, never written
over the originals. Each must be caught by sigrok: its payload differs from the stimulus, or it reports an error.

| Copy | What it breaks | What sigrok shows |
|---|---|---|
| `uart_6pct` | `basic/uart_tx.s` with 3 cycles added to its bit, 53 for 50: 6% slow at 1 Mbaud | framing errors from the first byte on |
| `dmx_stop` | `advanced/dmx_tx.s` with one stop bit, where DMX512 has two | "Invalid stop bit" in the first slot |
| `can_crc` | `advanced/can.s` computing its CRC with the polynomial 0x4598 for 0x4599 | CRC field 0x5570 for the frame's 0x64EF |
| `can_stuff` | `advanced/can.s` stuffing after six equal bits, not five | the frame misframed: "End of frame (EOF) must be 7 recessive bits" |
| `usb_crc16` | `test/test.py`'s `UNIT_USB` with the send half's CRC polynomial 0x8004 for 0x8005 | `crc16-err` on every packet |
| `usb_sync` | `advanced/usb_dev.s` answering with SYNC 0xC0 for 0x80 | `sync-err`: SYNC 00000011 |
| `i2c_stop` | `basic/i2c_write.s` without the STOP's SDA rise | no STOP and no second START: the next address reads as data, `DW:50` |
| `i2c_ack` | `advanced/i2c_tgt.s` leaving SDA high in a data byte's ACK slot | NACK after every data byte written |
| `spi_mode` | `basic/spi.s` ignoring CPHA, so that against a mode 1 target it runs as mode 0 | MOSI bytes 4B, 78 for A5, 3C |
| `swd_turnaround` | `advanced/swd.s` holding SWDIO low through the turnaround instead of letting go | ACK `ERROR` (000) for OK |
| `onewire_reset` | `basic/onewire.s` with a reset low of 400 us, under the 480 us minimum | "Erroneous signal" and no reset |

All 11 are caught on the simulator and on the RTL, where each broken copy's record also equals the simulator's.
`bench/basic.py`'s own SPI copy, which swaps the edges, is not here: it moves MOSI in the very cycle of the sampling
edge, which a zero-delay record shows as the new level at the edge, so it decodes as if right (see `spi` below).
The bench catches that one by the setup time.

## Findings

The checker found three things that the benches had missed. Each is fixed, the benches now check it, and its runs
here pass plainly.

- `standard/i2c.s`, `extra/i2c.s` and `trimmed/i2c.s`: when the device or register byte of a write was NACKed, the
  write's data word stayed in MIN and ran as the next transaction: after a write of 0x5A to an absent device,
  sigrok showed `S, AW:2D, NACK, P`, the data byte sent as a device byte. The programs now take the data word after
  the bus is released and before the status, after a bus fault too. The benches run a good write and read after a
  wrong address and after a bus fault (`bench/i2c_eeprom.py`, and the two-thread pair in `bench/standard.py`).
- `standard/cec.s`: after a block that wasn't acknowledged, the frame's later blocks went out as a frame of their
  own, and every frame's start bit came one 50 us unit after the line's last activity, with no signal free time.
  The program now takes the rest of the frame from MIN unsent and answers 1, and starts a frame only after the line
  has been high for 7 bit periods. `bench/standard.py` checks a block refused in the middle of a frame, another
  initiator's frame during the count, and the free time before every start bit; so does this checker, on the
  record. A review found one more: the program read a broadcast's ACK bit like a direct frame's, where HDMI CEC 1.4
  inverts it (a follower pulls the slot low to reject a broadcast). It now judges each block by the frame's
  destination; sigrok's cec decoder does the same, and checks a broadcast that every follower accepts and one that
  a follower rejects in its second block.
- `bench/can.py` used identifier 0x7FF, which Bosch CAN 2.0 part A, 3.1.1, forbids: its bits ID-10 to ID-4 are all
  recessive. It now uses 0x7EF, the most recessive legal identifier. That frame's CRC sequence ends in five 0s, so a
  stuff bit follows it (part A, 5). The chip sends and receives it correctly, and sigrok reads it, but the bench's
  own decoder in `bench/can_wire.py` stopped at the CRC's last bit and read the stuff bit as the CRC delimiter; it is
  fixed too.

## Known differences

The runs that differ (22 decodes: the six 1-Wire corners, 3 DMX and 13 JTAG) are listed on `DIFF` lines with their
VCD files. Each is a place where sigrok 0.5.3 reads the bus differently from the protocol document, and each is
accepted only in the exact form described below. The rest of this section is how the scenarios go round the
decoders' ways.

**sigrok 0.5.3's decoders.** Where a decoder reads a bus differently from the protocol document, the scenario
either goes round it, as listed here, or reports the run as a known difference.

- `onewire_link`: the DS18B20 data sheet has the presence pulse start 15 to 60 us after the reset pulse ends. A
  device model that starts it at exactly 60 us reads as no presence (the fall and sigrok's timeout land on the same
  sample). Six device corners of `basic/onewire.s`.
- `dmx512`: it leaves its break search only after slot 512, so a run of shorter packets is decoded one window per
  packet, each from just before its break. It wants a break of more than 88 us and at least one sample of mark
  between slots, where ANSI E1.11 has receivers take a break of 88 us and allows no mark at all. The receive
  scenarios run with a 92 us break (the transmit minimum) and 40 ns of mark (a quarter bit for the sender 2%
  fast). One run keeps the bench's own timing, an 88 us break and no mark: sigrok never finds that break, so it
  decodes nothing in the window. That run is a known difference only if the record's break lasts exactly 88 us and
  sigrok gives no break, no slot and no error at all in the window; the firmware's slots are checked as in the
  other runs.
- `jtag`: it takes a bit on the Exit1-DR to Pause-DR clock, where IEEE 1149.1 shifts nothing, so the scan through
  Pause-DR in `advanced/jtag.s` reads 21 bits for 20.
- `ps2`: it decodes device-to-host frames only, and reports a frame only at the twelfth clock fall after its start
  bit. Each device frame is decoded in a window of its own, up to the next clock fall on the bus, and each
  scenario ends with the host holding the clock low (an inhibit) so that the last frame has one. Frames the host
  sends are not decoded; the device's answers to them are.
- `can`: it does not check the CRC (`is_valid_crc` is unwritten) and has no stuff-error warning. The expected
  payload carries each frame's CRC field, computed from the frame with CRC-15/CAN and anchored on the catalogue
  check value 0x059E, so a wrong CRC still fails. It takes the first falling edge as a start of frame, and the TX
  pad is low from reset, so the decode starts after 11 recessive bits, where a node joins the bus (bus
  integration).
- `parallel`: it reports each word at the next clock edge, so the last byte of a run is not shown.
- `i2c`: it reports no warnings at all, so a missing STOP shows as a repeated START, and timing is not checked.
- `spi`: it has no setup or hold check, and in a record where a line moves in the same cycle as the sampling edge
  it reads the new level, so a program that moves MOSI on the sampling edge decodes as if right (the bench's
  decoder catches that one by its setup time).
- `uart` reports a framing error on a LIN break's null character; that is part of the break.

## Programs without a decoder

libsigrokdecode has no decoder for `standard/hdlc_tx.s` (HDLC), `standard/manch_rx.s` and `advanced/manch_rx.s`
(plain Manchester), and `standard/reflex.s` is a reaction time, not a protocol: their benches are their only
check. The CRC programs are checked against the standard library: `standard/crc16.s` against
`binascii.crc_hqx(msg, 0xFFFF)` (CRC-16/CCITT-FALSE), `standard/crc32.s` against `zlib.crc32`, and
`standard/crc8.s` (CRC-8/MAXIM, which the library lacks) against the catalogue's check value 0xA1. The memory loads
`basic/memhog.s` and `standard/memstress.s` have no bus of their own; their neighbours' buses are decoded while
they run.
