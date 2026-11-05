; Uses instruction memory as data, flat out. R5 counts read-back errors, R6 counts loops.
.equ BUF 120
start:  ldi   R1, BUF
        ldi   R5, 0
        ldi   R6, 0
loop:   cp   R2, R6
        ldi   R1, BUF
        stp   R2, R1        ; write and post-increment
        not   R2
        st    R2, R1
        ldi   R1, BUF
        ldp   R3, R1
        ld    R4, R1
        not   R4
        cmp   R3, R4
        jt    good
        addi  R5, 1
good:   addi  R6, 1
        goto   loop
