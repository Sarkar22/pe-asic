; UART RX 8N1. mailbox MCFG: cycles per bit. mailbox MOUT: each byte received, and
; after it a word with bit 8 set if that byte's stop bit was low.
.equ RX ui3
.equ MCFG 7
.equ MOUT 4
start:  recv  R2, MCFG      ; cycles per bit
        cp    R3, R2
        clrt
        shr16 R3, -, -      ; R3 = half a bit
        until RX, 1         ; line must be idle first
wait:   until RX, 0         ; D = cycle the falling edge reached SYNC2
        tick  R3            ; middle of the start bit
        ldi   R0, 0
        ldi   R1, 8
        get   RX            ; waits for that tick; T = the snapshot
        jt    wait          ; high again: a glitch, not a start bit
bit:    tick  R2            ; middle of the next bit
        shr8  R0, RX, -     ; waits for the tick; the sample enters at bit 7
        djnz  R1, bit
        trysend R0, MOUT    ; before the stop bit, so the path back to wait stays short
        tick  R2            ; middle of the stop bit
        get   RX
        jt    wait
        ldi   R0, 0         ; stop bit low
        lui   R0, 1
        trysend R0, MOUT
        until RX, 1         ; wait out a break
        goto  wait
