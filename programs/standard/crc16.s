; CRC-16/CCITT-FALSE in firmware: 3 slots per bit. Host posts bytes to MIN, then 0x100 to finish.
.equ MIN 0
.equ MOUT 1
start:  li    R4, 0x1021
        li    R3, 0xFFFF
next:   recv  R0, MIN
        bt    R0, 8
        jt    fin
        ldi   R1, 8
bit:    shl8  R0, -, -      ; T = next bit, MSB first
        crcl  R3, R4        ; one LFSR step; T is kept
        djnz  R1, bit
        goto   next
fin:    send  R3, MOUT
        goto   start
