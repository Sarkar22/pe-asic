; LIN responder header. MOUT: PID or 0xFF, one word.
.equ BRK 11*2604
.equ RX ui3
.equ MOUT 1
start:  li    R4, BRK
top:    until RX, 1
        until RX, 0
        until RX, 1, R4
        jt    top
        until RX, 1
        until RX, 0
        rdd   R2
        ldi   R1, 4
sy:     until RX, 1
        until RX, 0
        djnz  R1, sy
        rdd   R3
        sub   R3, R2
        clrt
        shr16 R3, -, -
        clrt
        shr16 R3, -, -
        clrt
        shr16 R3, -, -
        cp   R2, R3
        clrt
        shr16 R2, -, -
        until RX, 1
        until RX, 0
        tick  R2
        get   RX
        jt    bad
        ldi   R1, 8
bit:    tick  R3
        shr8  R0, RX, -
        djnz  R1, bit
        tick  R3
        get   RX
        jf    bad
        andi  R0, 255
        bt    R0, 0
        bx    R0, 1
        bx    R0, 2
        bx    R0, 4
        bx    R0, 6
        jt    bad
        bn    R0, 1
        bx    R0, 3
        bx    R0, 4
        bx    R0, 5
        bx    R0, 7
        jf    good
bad:    ldi   R0, 255
good:   send  R0, MOUT
        goto   top
