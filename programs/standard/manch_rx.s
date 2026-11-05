; Manchester RX, clock recovered per bit from the mid-bit edge. Line idles low; a rising mid-bit
; edge is a 1. A leading 1 starts the frame, then 8 data bits, LSB first.
; MCFG: 3/4 of a bit, in cycles. MOUT: received byte.
.equ RX ui6
.equ MCFG 7
.equ MOUT 1
start:  recv  R3, MCFG
        li    R4, 0x4000    ; mask = ui6, pattern unused in chg mode
        now
wait:   until RX, 1         ; the start bit's mid-bit edge
        ldi   R1, 8
bit:    tick  R3            ; 3/4 bit on: past any boundary edge, S = level before mid-bit
        untilm R4, ui, chg  ; next change against S is the mid-bit edge; D = its cycle
        shr8  R0, RX, -     ; level after the edge is the bit
        djnz  R1, bit
        send  R0, MOUT
        until RX, 0         ; back to idle
        goto   wait
