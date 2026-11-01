#!/usr/bin/env python3
"""Runs uart_tx.s in the simulator and decodes what comes out of the transmit pin."""
import pathlib

from asm import assemble
from sim import Chip

BIT = 434                                       # cycles per bit: 115200 baud at 50 MHz
MESSAGE = b"Hello from firmware"


def decode(levels, bit):
    """8N1 bytes from one sample per cycle."""
    out, i = bytearray(), 1
    while i < len(levels):
        if levels[i - 1] and not levels[i]:     # start bit
            out.append(sum(levels[i + bit * n + bit // 2] << (n - 1) for n in range(1, 9)))
            i += bit * 9 + bit // 2
        i += 1
    return bytes(out)


def main():
    img = assemble((pathlib.Path(__file__).parent / "uart_tx.s").read_text(), depth=128)
    chip = Chip(depth=128)
    for addr, word in img.words.items():
        chip.host_wrmem(addr, word)
    chip.host_setpc(0, 0)
    chip.cycle(0, 0)
    chip.host_run(0, 1)
    chip.host_post(7, BIT)

    waiting = list(MESSAGE)
    levels = []
    for _ in range(BIT * 11 * len(MESSAGE)):
        if waiting and chip.host_post(0, waiting[0]) == 0:
            waiting.pop(0)
        uo, _, _ = chip.pads(0)
        levels.append(uo & 1)
        chip.cycle(0, 0)

    print(decode(levels, BIT).decode())
    print(f"{img.size} words of firmware, LATE = {chip.th[0].LATE}")


if __name__ == "__main__":
    main()
