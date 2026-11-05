; WS2812 chain driver at 50 MHz. mailbox MIN: byte count (1 or more), then the
; bytes, first byte first, MSB first.
.equ DOUT uo6
.equ MIN 3
.equ T0H 18         ; cycles high for a 0 (360 ns)
.equ T1H 35         ; cycles high for a 1 (700 ns)
.equ TBIT 62        ; cycles per bit (1240 ns)
.equ TRES 15000     ; latch gap after a frame (300 us)
start:  pcfg  DOUT, PP
        put   DOUT, 0
        now
        li    R2, TRES
frame:  recv  R3, MIN
        recv  R0, MIN
        now                 ; D was idle during the recv
byte:   ldi   R1, 8
bit:    shl8  R0, -, -      ; T = next bit
        edge  DOUT, 1, TBIT-T1H     ; rises a bit time after the last rise
        edge  DOUT, T, T0H          ; falls here for a 0, stays high for a 1
        edge  DOUT, 0, T1H-T0H      ; falls here for a 1
        djnz  R1, bit
        djnz  R3, more
        tick  R2            ; hold low for the latch gap
        goto  frame
more:   recv  R0, MIN
        goto  byte
