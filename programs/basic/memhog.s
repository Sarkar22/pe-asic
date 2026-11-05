; Stress load: walks a table with ld and st as fast as a thread can, and keeps
; taking mailbox-free branches. It touches no pin. Used only to load the machine
; in the concurrency check.
.equ TABLE 0
.equ COUNT 16
start:  li    R0, TABLE
        ldi   R1, COUNT
        ldi   R2, 0
loop:   ldp   R3, R0
        add   R2, R3
        st    R2, R0
        djnz  R1, loop
        goto  start
