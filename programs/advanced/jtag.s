; JTAG scan engine driven from the host. Two words per command:
;   MIN word 1: bit 15 = 0 TMS sequence, 1 data scan; bit 14 = TMS high on the last scan bit
;               bits 4..0 = bits, 1 to 16
;   MIN word 2: bits to send, LSB first
; A scan replies on MOUT with TDO, last bit in bit 15. TCK idles low, and may stretch between
; commands and before the last bit, which 1149.1 allows.
.equ TCK uo1
.equ TMS uo2
.equ TDI uo5
.equ TDO ui4
.equ MCFG 7
.equ MIN 0
.equ MOUT 1
start:  recv  R2, MCFG
        pcfg  TCK, PP
        pcfg  TMS, PP
        pcfg  TDI, PP
        now
cmd:    recv  R5, MIN
        recv  R0, MIN
        cp    R1, R5
        andi  R1, 31
        bt    R5, 15
        jt    scan
        now
tq:     shr16 R0, -, TMS
        edge  TCK, 0, R2
        edge  TCK, 1, R2
        djnz  R1, tq
        edge  TCK, 0, R2
        goto  cmd
scan:   put   TMS, 0
        subi  R1, 1
        cmpi  R1, 0
        jt    last
        now
sb:     shr16 R0, TDO, TDI  ; last rise's TDO in at the top, next TDI out: one instruction, as in SPI
        edge  TCK, 0, R2
        edge  TCK, 1, R2
        djnz  R1, sb
last:   shr16 R0, TDO, TDI
        bt    R5, 14
        put   TMS, T
        now                 ; TMS and TDI move here, after their hold; TCK stays high a little longer
        edge  TCK, 0, R2
        edge  TCK, 1, R2
        shr16 R0, TDO, -
        edge  TCK, 0, R2
        send  R0, MOUT
        goto  cmd
