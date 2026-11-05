; JTAG controller: reset the TAP, read the 32-bit IDCODE, select BYPASS through the IR, then
; loop a test pattern through the 1-bit bypass register.
; MCFG: half TCK period in cycles. MIN: test pattern. MOUT: IDCODE low, IDCODE high, pattern back.
.equ TCK uo1
.equ TMS uo2
.equ TDI uo5
.equ TDO ui4
.equ MCFG 7
.equ MIN 0
.equ MOUT 1
start:  recv  R3, MCFG
        pcfg  TCK, PP
        pcfg  TMS, PP
        pcfg  TDI, PP
        now
        li    R2, 0x5F      ; TMS, LSB first: 1,1,1,1,1 reset, 0 idle, 1,0,0 to Shift-DR
        ldi   R1, 9
        call  tmsq
        ldi   R2, 0
        ldi   R1, 16
        call  shft
        send  R0, MOUT      ; IDCODE low half
        ldi   R1, 15
        call  last
        send  R0, MOUT      ; IDCODE high half
        ldi   R2, 0x0D      ; 1,0 update, idle; 1,1,0,0 to Shift-IR
        ldi   R1, 6
        call  tmsq
        ldi   R2, 0x0F      ; BYPASS is all ones
        ldi   R1, 3
        call  last
        ldi   R2, 0x05      ; 1,0 update, idle; 1,0,0 to Shift-DR
        ldi   R1, 5
        call  tmsq
        recv  R2, MIN
        ldi   R1, 15
        call  last
        send  R0, MOUT
        ldi   R2, 1         ; 1,0 update, idle
        ldi   R1, 2
        call  tmsq
        halt
; R1 bits with TMS low, then one more with TMS high, which leaves the shift state
last:   cp   R6, R7
        call  shft
        put   TMS, 1
        ldi   R1, 1
        call  shft
        jr    R6
; one clock per bit: TDI and TMS move on the fall, S takes TDO on the rise. `now` re-anchors at
; each entry: JTAG is fully clocked, so a longer TCK high between routines is legal.
shft:   now
sh1:    shr16 R2, -, TDI
        edge  TCK, 0, R3
        edge  TCK, 1, R3
        shr16 R0, TDO, -
        djnz  R1, sh1
        ret
tmsq:   now
tq1:    shr16 R2, -, TMS
        edge  TCK, 0, R3
        edge  TCK, 1, R3
        djnz  R1, tq1
        ret
