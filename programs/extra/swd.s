; SWD: line reset, read DPIDR, check ACK and parity. Mailbox out: data low, data high, status
; (0 ok, 1 bad ACK, 2 parity error).
.equ CLK uo1
.equ DIO io0
.equ HALF 25
.equ MOUT 1
start:  pcfg CLK, PP
        pcfg DIO, PP
        put  DIO, 1
        now
        edge CLK, 1, HALF
        ldi  R1, 56
lr:     edge CLK, 0, HALF
        edge CLK, 1, HALF
        djnz R1, lr
        put  DIO, 0
        ldi  R1, 8
id1:    edge CLK, 0, HALF
        edge CLK, 1, HALF
        djnz R1, id1
        ldi  R2, 0xA5
        ldi  R1, 8
req:    shr8 R2, -, DIO
        edge CLK, 0, HALF
        edge CLK, 1, HALF
        djnz R1, req
        oe   DIO, 0
        edge CLK, 0, HALF
        edge CLK, 1, HALF
        ldi  R1, 3
ack:    edge CLK, 0, HALF
        edge CLK, 1, HALF
        shr8 R3, DIO, -
        djnz R1, ack
        andi R3, 0xE0
        cmpi R3, 0x20
        ldi  R5, 1
        jf   tail
        ldi  R1, 32
        now
        edge CLK, 0, HALF
dat:    edge CLK, 1, HALF
        shr16 R2, DIO, -
        shr16 R0, -, -
        get  DIO
        bx   R4, 0
        bs   R4, 0
        edge CLK, 0, HALF
        djnz R1, dat
        edge CLK, 1, HALF
        get  DIO
        bx   R4, 0
        bs   R4, 0
        ldi  R5, 0
        jf   tail
        ldi  R5, 2
tail:   now
        edge CLK, 0, HALF
        edge CLK, 1, HALF
        put  DIO, 0
        oe   DIO, 1
        edge CLK, 0, HALF
        ldi  R1, 8
id2:    edge CLK, 1, HALF
        edge CLK, 0, HALF
        djnz R1, id2
        send R0, MOUT
        send R2, MOUT
        send R5, MOUT
        halt
