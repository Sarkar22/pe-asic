<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

Toolchain smoke test for a protocol emulator. A rising edge on `SEND` latches `DATA` and sends it as one 8N1 UART frame at 115200 baud (50 MHz clock) on `UART_TX`. `BUSY` is high during the frame, and `SEND` is ignored while it is.

## How to test

Set `DATA`, pulse `SEND` high for at least 3 clock cycles, and read the byte from `UART_TX` with any 115200 8N1 UART receiver.

## External hardware

None. On the demo board, `UART_TX` reaches the RP2350 UART1 RX pin.
