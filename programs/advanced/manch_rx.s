; Manchester RX, IEEE 802.3 levels (rising mid-bit edge = 1), clock recovered on every bit.
; Frame: alternating preamble, 1 1 start mark, bytes LSB first, then silence.
; MCFG: 3/4 bit in cycles. MTO: 1/2 bit. MOUT: each byte, then 0x100 + bits left when the line stops.
.equ RX ui6
.equ MCFG 7
.equ MTO 6
.equ MOUT 1
start:  recv  R3, MCFG
        recv  R5, MTO
        li    R4, 0x4000    ; change on ui6
        now
hunt:   untilm R4, ui, chg  ; any edge: in the preamble every edge is a mid-bit edge
s0:     tick  R3            ; past any boundary edge; S = level before the next mid-bit edge
        untilm R4, ui, chg, R5
        jf    hunt          ; nothing where a mid-bit edge had to be: the line stopped
        get   RX
        jf    s0
s1:     tick  R3
        untilm R4, ui, chg, R5
        jf    hunt
        get   RX
        jf    s0
byte:   ldi   R1, 8
bit:    tick  R3
        untilm R4, ui, chg, R5
        jf    end
        shr8  R0, RX, -
        djnz  R1, bit
        trysend R0, MOUT
        goto  byte
end:    cp    R2, R1
        lui   R2, 1
        send  R2, MOUT      ; send, not trysend: the last byte may not have been taken yet
        goto  hunt
