; I2C controller, write transactions, open drain on SDA and SCL.
; mailbox MCFG, three words: Q = SCL fall to SDA change, R = rest of the low time,
; H = high time. mailbox MIN: low byte is the byte to send, bit 8 sends a START
; first, bit 9 sends a STOP after it. mailbox MOUT, one word per byte: 0 = ACK and
; the transaction stays open, 2 = ACK and STOP sent, 1 = NACK and STOP sent.
.equ SDA io1
.equ SCL io2
.equ MCFG 6
.equ MIN 2
.equ MOUT 4
start:  pcfg  SDA, OD
        pcfg  SCL, OD
        recv  R2, MCFG
        recv  R3, MCFG
        recv  R4, MCFG
next:   recv  R0, MIN
        bt    R0, 8
        jf    data
        put   SDA, 0        ; START: SDA falls while SCL is high
        now
        edge  SCL, 0, R4
        goto  first
data:   now                 ; D was idle during the recv; close to the first tick
first:  ldi   R1, 8
wrbit:  shl8  R0, -, SDA
        call  rise
        edge  SCL, 0, R4
        djnz  R1, wrbit
        put   SDA, 1        ; release SDA for the target's ACK
        call  rise
        edge  SCL, 0, R4
        get   SDA           ; snapshot taken as SCL falls; 1 = NACK
        jt    nack
        ldi   R1, 2
        bt    R0, 9
        jt    stop
        ldi   R1, 0
        send  R1, MOUT
        goto  next
nack:   ldi   R1, 1
stop:   put   SDA, 0
        now                 ; several slots since SCL fell; restart the low-time clock
        call  rise
        edge  SDA, 1, R4    ; STOP: SDA rises while SCL is high
        tick  R3            ; bus free time
        tick  R2
        send  R1, MOUT
        goto  next
rise:   tick  R2            ; SDA moves Q after SCL fell
        edge  SCL, 1, R3    ; release SCL
        until SCL, 1        ; a target may hold it low: this is the stretch
        ret
