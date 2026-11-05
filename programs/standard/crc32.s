; CRC-32 (IEEE, reflected) in firmware, on two 16-bit registers: 8 slots per bit.
; R2:R3 = state. Polynomial 0xEDB88320, with bit 31 cleared in R5 because R2[15] already holds
; the feedback bit when the XOR happens.
.equ MIN 0
.equ MOUT 1
start:  li    R4, 0x8320
        li    R5, 0x6DB8
        li    R2, 0xFFFF
        li    R3, 0xFFFF
next:   recv  R0, MIN
        bt    R0, 8
        jt    fin
        ldi   R1, 8
bit:    shr8  R0, -, -      ; T = next bit, LSB first
        bx    R3, 0         ; T = feedback
        shr16 R2, -, -      ; feedback parks in R2[15]; T = old R2[0]
        shr16 R3, -, -      ; which moves into R3[15]
        bt    R2, 15        ; T = feedback again
        xort  R2, R5
        xort  R3, R4
        djnz  R1, bit
        goto   next
fin:    not   R2
        not   R3
        send  R3, MOUT      ; low half first
        send  R2, MOUT
        goto   start
