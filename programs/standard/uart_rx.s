; UART RX 8N1, sampling at mid-bit. mailbox MCFG: cycles per bit. mailbox MOUT: received
; bytes, with bit 8 set on a framing error.
.equ RX ui3
.equ MCFG 7
.equ MOUT 1
start:  recv  R2, MCFG      ; R2 = cycles per bit
        cp   R3, R2
        clrt
        shr16 R3, -, -      ; R3 = half a bit
quiet:   until RX, 1         ; make sure the line is high first
frame:  until RX, 0         ; start edge; D = the cycle it was seen, S = inputs then
        tick  R3            ; middle of the start bit
        get   RX            ; T = level there (stalls until the tick has fired)
        jt    quiet          ; high again: it was a glitch
        ldi   R1, 8
bit:    tick  R2            ; next mid-bit, at an exact cycle
        shr8  R0, RX, -     ; T = S[RX], shifted in from the top, LSB first
        djnz  R1, bit
        tick  R2            ; middle of the stop bit
        get   RX
        jf    bad
        andi  R0, 255
        trysend R0, MOUT
        goto   frame
bad:    lui   R0, 1         ; mark the framing error
        send  R0, MOUT
        goto   quiet
