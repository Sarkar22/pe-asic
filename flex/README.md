# Flexibility trial

The chip has to learn new protocols after fabrication. We test that on protocols we don't choose, and this folder lets anyone check the choice.

## How it works

1. **The pool is fixed first.** `decoders.csv` lists all 131 decoders of [libsigrokdecode](https://github.com/sigrokproject/libsigrokdecode/tree/71f451443029322d57376214c330b518efd84f88) at commit `71f4514`, each marked `pool` or `out` with its reason. 25 are in the pool. The `rule` column gives the first rule below that a decoder fails. The file can change only before `hw-freeze`, in a public commit that says why.
2. **The hardware freezes.** We push the tag `hw-freeze` with a GitHub Release, planned for 2026-12-06.
3. **A public random value makes the draw.** [drand](https://drand.love) quicknet round 34619812 comes out at 2026-12-07 17:00:00 UTC, and nobody can know it before then. `draw.py` turns it into 3 protocols.
4. **Firmware comes last.** The 3 protocols get firmware with the hardware frozen. Authors work from the public documents only and log their hours. A bug fix to the RTL after the freeze goes in its own pull request and is listed with the results.
5. **Every result is published,** including "can't be done, because ...".

A protocol passes when firmware takes a real role in it and the pin trace decodes with no errors in that protocol's sigrok decoder, in simulation and on the frozen RTL.

If the freeze slips, we'll commit a later round before this one comes out, so the tag always comes first.

## What is in the pool

A decoder is in the pool only if all of these hold. None of them asks whether our chip can do the protocol.

1. It reads pins directly. A decoder stacked on another decoder is out.
2. It's one protocol with framing. sigrok tags tools and bare codes `Util` or `Encoding`, and those are out. So are pin patterns with no framing.
3. It isn't UART, SPI or I2C, or anything a stock UART, SPI or I2C controller can carry with only a changed word length or bit order and one extra pin. Those are the starting point.
4. It isn't named in the competition brief: USB, Ethernet, JTAG, SWD, PS/2 and CAN, including members of those families.
5. Its firmware doesn't exist yet. We wrote firmware for WS2812, 1-Wire (which TI's SDQ matches), MDIO, DMX512, HDMI CEC, LIN, NEC infrared, 8-bit parallel capture, SWIM, AVR PDI and NES gamepad while developing the design, so those wouldn't be a fair test.
6. It fits the free pins in at least one real role: 5 inputs, 7 outputs and 8 bidirectional pins. The host link takes the other 4.
7. Its signals are digital at 3.3 V at the chip's pins. A level shifter, transceiver or demodulator may sit between the chip and a real bus.
8. Real devices run it at 2 Mbit/s or less, counted in data bits on one line.

## Repeat the draw

```sh
curl -s https://api.drand.sh/52db9ba70e0cc0f6eaf7803dd07447a1f5477735fd3f661792ba94600c84e971/public/34619812
python3 flex/draw.py <randomness>
```

`draw.py` ranks each pool name by the SHA-256 of `<randomness>:<name>` and prints the first 3. It needs only Python. drand's own tools can check the round's signature against the quicknet public key.

To check your copy of the script: with 64 zeros as the randomness it prints `dali`, `ir_rc5` and `dsi`.
