; I2C target tuned for Fast-mode Plus. Same job as i2c_tgt, written for the 50-cycle bit: inlined bit
; loops, ldp and stp in place of pointer arithmetic, and the address compared one bit early, so the
; ACK is prepared in time. The price: no pointer wrap, and the 7th address bit has no START/STOP check.
.equ SDA io2
.equ SCL io3
.equ MCFG 7
.equ BASE 90
start:  recv  R6, MCFG
        pcfg  SDA, OD
        pcfg  SCL, OD
        now
        li    R4, 0x0C00
        li    R2, BASE
low:    until SCL, 1
high:   untilm R4, io, chg
        get   SCL
        jf    low
        get   SDA
        jt    high
begin:  until SCL, 0
        ldi   R1, 6
ab:     until SCL, 1
        shl8  R0, SDA, -
        untilm R4, io, chg
        get   SCL
        jt    ss
        djnz  R1, ab
        until SCL, 1        ; 7th bit, compared right here
        shl8  R0, SDA, -
        cp    R3, R0
        andi  R3, 0x7F
        cmp   R3, R6
        jf    low
        until SCL, 0
        until SCL, 1        ; R/W
        shl8  R0, SDA, -
        put   SDA, 0
        until SCL, 0        ; ACK
        bt    R0, 0
        jt    read
        until SCL, 1
        put   SDA, 1
        until SCL, 0
        ldi   R1, 7         ; pointer byte
pb:     until SCL, 1
        shl8  R0, SDA, -
        untilm R4, io, chg
        get   SCL
        jt    ss
        djnz  R1, pb
        until SCL, 1
        shl8  R0, SDA, -
        put   SDA, 0
        until SCL, 0
        cp    R2, R0
        ori   R2, BASE
        until SCL, 1
        put   SDA, 1
        until SCL, 0
wnext:  ldi   R1, 7
wb:     until SCL, 1
        shl8  R0, SDA, -
        untilm R4, io, chg
        get   SCL
        jt    ss
        djnz  R1, wb
        until SCL, 1
        shl8  R0, SDA, -
        put   SDA, 0
        until SCL, 0        ; ACK, 3 to 7 cycles after the fall at 1 MHz
        until SCL, 1
        put   SDA, 1
        until SCL, 0
        stp   R0, R2
        goto  wnext
ss:     get   SDA
        jf    begin
        goto  high
read:   ldp   R0, R2
        ldi   R1, 8
tb:     until SCL, 1
tb2:    shl8  R0, -, SDA
        until SCL, 0
        djnz  R1, tb
        until SCL, 1
        put   SDA, 1
        until SCL, 0
        ldp   R0, R2
        ldi   R1, 8
        until SCL, 1
        get   SDA
        jf    tb2
        goto  high
