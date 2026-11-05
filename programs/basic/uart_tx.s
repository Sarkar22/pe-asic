; UART TX 8N1. Runs on any thread; bit time is settable at run time.
; mailbox MCFG: cycles per bit (12..32767). mailbox MIN: bytes to send.
.equ TX uo0
.equ MCFG 7
.equ MIN 0
start:  pcfg  TX, PP        ; this thread owns the pin
        put   TX, 1         ; line idles high
        now                 ; commit it
        recv  R2, MCFG      ; R2 = cycles per bit, sent by the host once
byte:   recv  R0, MIN       ; block here with the line idle
        put   TX, 0         ; start bit
        now                 ; falls one cycle later; D = that cycle
        ldi   R1, 8
bit:    shr8  R0, -, TX     ; T = next data bit, LSB first; prepared on TX
        tick  R2            ; lands exactly one bit time after the previous edge
        djnz  R1, bit
        put   TX, 1         ; stop bit
        tick  R2            ; stop bit starts
        tick  R2            ; and lasts one full bit time
        goto   byte
