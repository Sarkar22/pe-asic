; UART RX 8N1, trimmed for the shared 128-word image. The host posts the half bit too, instead of
; firmware computing it. MCFG: cycles per bit, then half of that. MOUT: bytes, bit 8 = framing error.
.equ RX ui3
.equ MCFG 7
.equ MOUT 1
start:  recv  R2, MCFG
        recv  R3, MCFG
quiet:  until RX, 1
frame:  until RX, 0
        tick  R3
        get   RX
        jt    quiet
        ldi   R1, 8
bit:    tick  R2
        shr8  R0, RX, -
        djnz  R1, bit
        tick  R2
        get   RX
        jf    bad
        andi  R0, 255
        trysend R0, MOUT
        goto  frame
bad:    lui   R0, 1
        send  R0, MOUT
        goto  quiet
