; WS2812: one byte, MSB first, at 50 MHz. mailbox MIN: the byte. MOUT: 1 when done.
.equ MIN 0
.equ MOUT 1
.equ LED uo6
.equ T0H 18                 ; 360 ns high for a 0
.equ T1H 35                 ; 700 ns high for a 1
.equ TBIT 62                ; 1240 ns per bit
start:  pcfg  LED, PP
        put   LED, 0
        now
byte:   recv  R0, MIN
        now                 ; D = now, so the first rise is on time
        ldi   R1, 8
        ldi   R3, 1
bit:    shl8  R0, -, -      ; T = next bit, MSB first
        edge  LED, 1, TBIT-T1H ; rise
        edge  LED, T, T0H   ; stays high only if T = 1
        edge  LED, 0, T1H-T0H ; always low by here
        djnz  R1, bit
        li    R2, 15000     ; 300 us latch gap
        tick  R2
        send  R3, MOUT
        goto   byte
