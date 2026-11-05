; SPI controller, all four modes from one loop. CPOL and CPHA are constants.
; mailbox MCFG: half period in cycles (9 or more). MIN: byte out. MOUT: byte in.
.equ MCFG 7
.equ MIN 0
.equ MOUT 1
.equ SKEW 0                  ; test only: shifts the slot alignment by 0..3 cycles
.equ CPOL 0
.equ CPHA 0
.equ VA CPOL ^ CPHA         ; SCK level at the edge where MOSI changes
.equ VB VA ^ 1              ; SCK level at the edge where MISO is sampled
.equ SCK uo1
.equ MOSI uo2
.equ CS uo5
.equ MISO ui4
start:  pcfg  SCK, PP
        pcfg  MOSI, PP
        pcfg  CS, PP
        put   CS, 1
        put   SCK, CPOL
        now
        recv  R2, MCFG
xfer:   recv  R0, MIN
        ldi   R1, 8
        put   CS, 0
        now
.if SKEW
        tick  8+SKEW
.endif
bit:    shl8  R0, MISO, MOSI ; last sample in at the bottom, next bit out to MOSI
        edge  SCK, VA, R2   ; commits SCK and MOSI on the same cycle
        edge  SCK, VB, R2   ; S snapshots MISO on the sample edge
        djnz  R1, bit
        shl8  R0, MISO, -   ; ninth shift: last sample in, first junk bit falls off
        edge  SCK, CPOL, R2 ; back to idle (no change at all in CPHA=1 modes)
        put   CS, 1
        tick  R2
        andi  R0, 255
        send  R0, MOUT
        goto   xfer
