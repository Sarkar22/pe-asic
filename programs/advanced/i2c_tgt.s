; I2C target, register file of 32 bytes. Write: first byte sets the pointer, the rest store.
; Read: from the pointer. Repeated START and STOP are seen between any two bits of a write byte.
; ACK is pre-armed: decided before the 8th clock's fall, committed by that fall, 3 cycles pad to pad.
; MCFG: 7-bit address.
.equ SDA io2
.equ SCL io3
.equ MCFG 7
.equ BASE 96
start:  recv  R6, MCFG
        pcfg  SDA, OD
        pcfg  SCL, OD
        now
        li    R4, 0x0C00    ; change on SDA or SCL
        li    R2, BASE
low:    until SCL, 1        ; bus monitor. S now has SCL high
high:   untilm R4, io, chg  ; SCL fell, or SDA moved with SCL high
        get   SCL
        jf    low
        get   SDA
        jt    high          ; SDA rose: STOP
begin:  until SCL, 0        ; START
        call  rb7
        cp    R3, R0
        andi  R3, 0x7F
        cmp   R3, R6
        jf    low           ; not ours: watch for the next START
        until SCL, 1        ; R/W
        shl8  R0, SDA, -
        put   SDA, 0
        until SCL, 0        ; ACK
        bt    R0, 0
        jt    read
        call  ackend
        call  rb7           ; pointer byte
        call  ackdata
        cp    R2, R0
        andi  R2, 31
        ori   R2, BASE
wnext:  call  rb7
        call  ackdata
        st    R0, R2
        addi  R2, 1
        andi  R2, 127
        ori   R2, BASE
        goto  wnext
; first 7 bits of a byte, one START or STOP check per bit
rb7:    ldi   R1, 7
rb1:    until SCL, 1
        shl8  R0, SDA, -
        untilm R4, io, chg
        get   SCL
        jt    ss
        djnz  R1, rb1
        ret
ss:     get   SDA           ; SCL stayed high, SDA moved
        jf    begin         ; fell: repeated START
        goto  high          ; rose: STOP
; 8th bit and ACK of a data byte
ackdata: until SCL, 1
        shl8  R0, SDA, -
        put   SDA, 0
        until SCL, 0
ackend: until SCL, 1
        put   SDA, 1
        until SCL, 0
        ret
read:   ld    R0, R2
        addi  R2, 1
        andi  R2, 127
        ori   R2, BASE
        ldi   R1, 8
tb:     until SCL, 1
tb2:    shl8  R0, -, SDA    ; prepared, the next fall commits it
        until SCL, 0
        djnz  R1, tb
        until SCL, 1
        put   SDA, 1        ; let go for the controller's ACK
        until SCL, 0
        ld    R0, R2
        addi  R2, 1
        andi  R2, 127
        ori   R2, BASE
        ldi   R1, 8
        until SCL, 1
        get   SDA
        jf    tb2           ; ACK: next byte, first bit prepared in this high phase
        goto  high          ; NACK
