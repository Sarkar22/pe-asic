; CRC-8/MAXIM (the 1-Wire ROM CRC) in firmware. Host posts bytes to MIN, then 0x100 to finish.
.equ MIN 0
.equ MOUT 1
start:  ldi   R4, 0x8C
        ldi   R3, 0
next:   recv  R0, MIN
        bt    R0, 8
        jt    fin
        ldi   R1, 8
bit:    shr8  R0, -, -      ; T = next bit, LSB first
        crcr  R3, R4
        djnz  R1, bit
        goto   next
fin:    send  R3, MOUT
        goto   start
