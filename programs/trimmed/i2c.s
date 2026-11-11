; I2C controller, trimmed for the shared 128-word image. Same host interface as standard/i2c.s.
; MCFG: tHIGH, then tLOW-QHOLD. MIN: word 0 = device byte<<8 | register, word 1 = data<<8 (write only).
; Bit 8 of word 0 is op: 0 write, 1 read with repeated START. MOUT: status, then byte.
; Status 0 ok, 1 NACK, 2 bus fault. A write takes word 1 even when it ends before its data byte, on a
; NACK or a bus fault: after the bus is released, and before the status.
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
xact:   recv  R5, MIN
        li    R6, 0x0303    ; both lines must be high
        now
        untilm R6, io, eq, R4
        jf    fault
        edge  SDA, 0, R2
        edge  SCL, 0, R2
        cp    R0, R5
        clrt
        bs    R0, 8
        call  wbyte
        jt    fail
        cp    R0, R5
        swap  R0
        call  wbyte
        jt    fail
        bn    R5, 8         ; T = 1 for a write
        jf    rd
        bs    R5, 8         ; bit 8 set: no word of this transaction is left in MIN
        recv  R0, MIN
        call  wbyte
        jf    stop          ; ACK: R1, the bit count, is 0, the status for ok
fail:   ldi   R1, 1         ; status NACK
stop:   put   SDA, 0
        call  sclhi
        edge  SDA, 1, R2
        tick  R3
chk:    bt    R5, 8
        jt    reply
        recv  R6, MIN       ; the data word of a write that ended early; R6 is set again at xact
reply:  send  R1, MOUT
        send  R0, MOUT
        goto  xact
fault:  put   SDA, 1        ; SCL is released already
        now
        ldi   R1, 2
        goto  chk
rd:     put   SDA, 1
        call  sclhi
        edge  SDA, 0, R2
        edge  SCL, 0, R2
        cp    R0, R5
        call  wbyte
        jt    fail
        lui   R0, 255       ; top byte all ones releases SDA for the read
        call  wbyte
        andi  R0, 255
        goto  stop
sclhi:  tick  QHOLD         ; SCL rises QHOLD and tLOW-QHOLD after the last edge, and may be stretched
        edge  SCL, 1, R3
        until SCL, 1, R4
        jf    fault
        ret
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
