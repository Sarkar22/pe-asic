; 24C02-style I2C target, 256 bytes. Needs the 512-word memory: one byte per word at 0x100..0x1FF.
; Handles write, current-address read and random read with repeated START. No clock stretching.
; S at the rising edge carries SDA. After it, a change on either line is either SCL falling, the
; normal case, or SDA moving with SCL high, which is START or STOP.
.equ SDA io2
.equ SCL io3
.equ DEV 0xA0
start:  pcfg  SDA, OD
        pcfg  SCL, OD
        now
        li    R2, 0x0100    ; address pointer, top byte fixed
        li    R3, 0x0C00    ; change on either line
        li    R6, 0x0C08    ; SCL high, SDA low
wait:   untilm R6, io, eq   ; START, from any state
bstart: until SCL, 0
        call  rbyte
        cp    R5, R0
        andi  R5, 0xFE
        cmpi  R5, DEV
        jf    wait
        call  ack
        bt    R0, 0
        jt    read
        call  rbyte         ; first byte of a write sets the pointer
        cp    R2, R0
        lui   R2, 1
wnext:  call  ack
        call  rbyte
        st    R0, R2
        addi  R2, 1
        lui   R2, 1
        goto  wnext
read:   ld    R0, R2
        addi  R2, 1
        lui   R2, 1
        ldi   R1, 8
tbit:   shl8  R0, -, SDA    ; prepared, committed by the falling edge wait below
        now
        until SCL, 1
        until SCL, 0
        djnz  R1, tbit
        put   SDA, 1        ; release for the controller's ACK
        now
        until SCL, 1
        get   SDA           ; 0 = ACK, send the next byte
        jt    wait
        until SCL, 0
        goto  read
rbyte:  ldi   R1, 8
rb1:    until SCL, 1
        shl8  R0, SDA, -
        untilm R3, io, chg
        get   SCL
        jt    edge2
        djnz  R1, rb1
        ret
edge2:  get   SDA           ; SDA moved while SCL stayed high
        jf    bstart        ; fell: repeated START
        goto  wait          ; rose: STOP
ack:    put   SDA, 0
        now
        until SCL, 1
        until SCL, 0
        put   SDA, 1
        now
        ret
