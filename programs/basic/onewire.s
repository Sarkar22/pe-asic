; 1-Wire master at standard speed, 50 MHz, open drain on one uio pin.
; mailbox MIN: bit 8 set runs a reset and returns the presence result in MOUT
; (1 = a device answered); otherwise the low byte is written, LSB first.
.equ OW io4
.equ MIN 3
.equ MOUT 5
.equ TRSTL 24500    ; reset low, 490 us
.equ TPRES 3500     ; release to presence sample, 70 us
.equ TRSTH 21000    ; rest of the reset high time, 420 us
.equ TLOW1 300      ; low time of a write 1, 6 us
.equ TLOW0 3250     ; low time of a write 0, 65 us: a device may sample as late as 60 us
.equ TREC 500       ; recovery between slots, 10 us
start:  pcfg  OW, OD
        li    R2, TLOW1
        li    R3, TLOW0-TLOW1
        li    R4, TREC
cmd:    recv  R0, MIN
        now                 ; D was idle during the recv
        bt    R0, 8
        jt    reset
        ldi   R1, 8
bit:    shr8  R0, -, -      ; T = next bit
        edge  OW, 0, R4     ; slot starts after the recovery time
        edge  OW, T, R2     ; a 1 releases here
        edge  OW, 1, R3     ; a 0 releases here
        djnz  R1, bit
        goto  cmd
reset:  li    R5, TRSTL
        put   OW, 0
        now
        edge  OW, 1, R5     ; release after the reset pulse
        li    R5, TPRES
        tick  R5            ; sample the bus
        getn  OW            ; T = 1 when a device holds it low
        li    R5, TRSTH
        tick  R5
        ldi   R1, 0
        bs    R1, 0
        send  R1, MOUT
        goto  cmd
