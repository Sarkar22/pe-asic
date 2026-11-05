; Kept for comparison: i2c.s replaces it in the four-protocol image. No bus-free check, and it
; ignores a stuck SCL at STOP.
; I2C controller: START, address+W, one data byte, STOP. Checks both ACKs, honours stretching.
; mailbox MCFG takes two words: tHIGH cycles, then tLOW-QHOLD cycles.
; mailbox MIN: address+W in the high byte, data in the low byte.
; mailbox MOUT: result, 0 = ok, 1 = NACK or stretch timeout.
.equ MCFG 7
.equ MIN 0
.equ MOUT 1
.equ SDA io0
.equ SCL io1
.equ QHOLD 40               ; SDA changes this long after SCL falls (800 ns at 50 MHz)
start:  recv  R2, MCFG
        recv  R3, MCFG
        li    R4, 30000     ; stretch limit, 600 us at 50 MHz
        pcfg  SDA, OD
        pcfg  SCL, OD
        now
xact:   recv  R0, MIN
        cp   R5, R0
        swap  R5            ; R5 high byte = data, for the second byte
        now
        edge  SDA, 0, R2    ; START: SDA falls while SCL is high
        edge  SCL, 0, R2
        call  wbyte
        jt    fail
        cp   R0, R5
        call  wbyte
        jt    fail
        ldi   R6, 0
        goto   stop
fail:   ldi   R6, 1
stop:   put   SDA, 0
        tick  QHOLD
        edge  SCL, 1, R3
        until SCL, 1, R4    ; a target may stretch here too
        edge  SDA, 1, R2    ; STOP: SDA rises while SCL is high
        tick  R3            ; bus free time
        send  R6, MOUT
        goto   xact
; send R0[15:8] then read the ACK. Returns T=0 on ACK, T=1 on NACK or timeout.
wbyte:  ori   R0, 128       ; ninth bit is a 1: releases SDA for the ACK slot
        ldi   R1, 9
wb1:    shl16 R0, -, SDA    ; prepare SDA (1 = release, 0 = pull low: open drain)
        tick  QHOLD         ; SDA moves QHOLD after SCL fell
        edge  SCL, 1, R3    ; release SCL
        until SCL, 1, R4    ; wait for SCL to really rise; D = that cycle
        jf    wbad          ; T=0: held low past the limit
        edge  SCL, 0, R2    ; tHIGH counts from the observed rise. S takes SDA here
        djnz  R1, wb1
        get   SDA           ; ACK level just before SCL fell
        ret
wbad:   sett
        ret
