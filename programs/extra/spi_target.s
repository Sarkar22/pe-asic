; SPI target, mode 0. CS-framed single bytes. MISO sends the previous byte received.
.equ CS ui3
.equ SCK ui4
.equ MOSI ui5
.equ MISO uo2
.equ MOUT 1
start:  pcfg MISO, PP
        put  MISO, 0
        now
        ldi  R2, 0
top:    until CS, 0
        cp  R0, R2
        ldi  R1, 8
        shl8 R0, -, MISO
        now
bit:    until SCK, 1
        shl8 R2, MOSI, -
        shl8 R0, -, MISO
        until SCK, 0
        djnz R1, bit
        send R2, MOUT
        until CS, 1
        goto  top
