; I2C controller, trimmed for the shared 128-word image. Same host interface as standard/i2c.s.
; MCFG: tHIGH, then tLOW-QHOLD. MIN: word 0 = device byte<<8 | register, word 1 = data<<8 (write only).
; Bit 8 of word 0 is op: 0 write, 1 read with repeated START. MOUT: status, then byte.
; Status 0 ok, 1 NACK, 2 bus fault.
.equ MCFG 7
.equ MIN 0
.equ MOUT 1
.equ SDA io0
.equ SCL io1
.equ QHOLD 40
start:  recv  R2, MCFG
        recv  R3, MCFG
        li    R4, 30000
        pcfg  SDA, OD
        pcfg  SCL, OD
        now
xact:   recv  R5, MIN
        li    R6, 0x0303    ; both lines must be high
        now
        untilm R6, io, eq, R4
        jf    fault
        edge  SDA, 0, R2
        edge  SCL, 0, R2
        ldi   R6, 1         ; status is NACK until a path says otherwise
        cp    R0, R5
        clrt
        bs    R0, 8
        call  wbyte
        jt    stop
        cp    R0, R5
        swap  R0
        call  wbyte
        jt    stop
        bt    R5, 8
        jt    rd
        recv  R0, MIN
        call  wbyte
        jt    stop
ok:     ldi   R6, 0
stop:   put   SDA, 0
        tick  QHOLD
        edge  SCL, 1, R3
        until SCL, 1, R4
        jf    fault
        edge  SDA, 1, R2
        tick  R3
reply:  send  R6, MOUT
        send  R0, MOUT
        goto  xact
fault:  put   SDA, 1
        put   SCL, 1
        now
        ldi   R6, 2
        goto  reply
rd:     put   SDA, 1
        tick  QHOLD
        edge  SCL, 1, R3
        until SCL, 1, R4
        jf    fault
        edge  SDA, 0, R2
        edge  SCL, 0, R2
        cp    R0, R5
        call  wbyte
        jt    stop
        lui   R0, 255       ; top byte all ones releases SDA for the read
        call  wbyte
        andi  R0, 255
        goto  ok
wbyte:  now
        ori   R0, 128
        ldi   R1, 9
wb1:    shl16 R0, SDA, SDA
        tick  QHOLD
        edge  SCL, 1, R3
        until SCL, 1, R4
        jf    fault
        edge  SCL, 0, R2
        djnz  R1, wb1
        get   SDA
        ret
