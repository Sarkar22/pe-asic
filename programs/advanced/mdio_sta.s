; MDIO station, Clause 22. MCFG: MDC half period in cycles. MIN: frame header, 16 bits, MSB first:
; 01 op(2) phy(5) reg(5) 10, then the data word for a write. MOUT: data read, 0x0000-0xFFFF.
.equ MDC uo1
.equ MDIO io0
.equ MCFG 7
.equ MIN 0
.equ MOUT 1
start:  recv  R2, MCFG
        pcfg  MDC, PP
        pcfg  MDIO, PP
        now
cmd:    recv  R0, MIN
        cp    R5, R0
        bt    R0, 13        ; op 10 is a read
        jt    go
        recv  R4, MIN
go:     now
        put   MDIO, 1
        oe    MDIO, 1
        ldi   R1, 32
pre:    edge  MDC, 0, R2
        edge  MDC, 1, R2
        djnz  R1, pre
        ldi   R1, 14
        call  bits
        bt    R5, 13
        jt    rd
        ldi   R1, 2
        call  bits
        cp    R0, R4
        ldi   R1, 16
        call  bits
        goto  fin
rd:     oe    MDIO, 0       ; first turnaround bit is ours to release
        edge  MDC, 0, R2
        edge  MDC, 1, R2
        edge  MDC, 0, R2
        edge  MDC, 1, R2
        ldi   R1, 16
rb:     edge  MDC, 0, R2
        edge  MDC, 1, R2
        shl16 R3, MDIO, -   ; S at that rise
        djnz  R1, rb
        send  R3, MOUT
fin:    oe    MDIO, 0
        edge  MDC, 0, R2
        edge  MDC, 1, R2    ; line released, one idle clock
        edge  MDC, 0, R2
        goto  cmd
bits:   shl16 R0, -, MDIO   ; data moves with the fall, so it is stable through the rise
        edge  MDC, 0, R2
        edge  MDC, 1, R2
        djnz  R1, bits
        ret
