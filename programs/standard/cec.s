; HDMI CEC initiator on one open-drain pin. Bits last milliseconds, far past one tick's range,
; so every delay is a loop of 50 us units.
; MIN: one word per block: [15:8] byte, bit 7 EOM, bit 6 = 1 (the initiator's ACK slot).
; MOUT: 0 if every block was acknowledged, 1 if not.
.equ CEC io3
.equ UNIT 2500              ; 50 us at 50 MHz
.equ MIN 0
.equ MOUT 1
start:  pcfg  CEC, OD
        li    R4, UNIT
        now
frame:  recv  R0, MIN
        now
        edge  CEC, 0, R4    ; start bit
        ldi   R5, 73
        call  wait
        put   CEC, 1        ; released at 3.7 ms
        tick  R4
        ldi   R5, 15
        call  wait          ; 4.45 ms: the next bit opens one unit later
block:  ldi   R2, 0
        bt    R0, 7
        bs    R2, 0         ; remember EOM
        ldi   R1, 10        ; 8 data bits, EOM, ACK slot
b1:     shl16 R0, -, -      ; T = next bit, MSB first
        call  cbit
        djnz  R1, b1
        jt    bad           ; T = line in the ACK slot: low means acknowledged
        bt    R2, 0
        jt    good
        recv  R0, MIN
        goto   block
good:   ldi   R3, 0
        goto   done
bad:    ldi   R3, 1
done:   send  R3, MOUT
        goto   frame
; one bit: T in, T out = line at the 1.05 ms sample point
cbit:   cp   R6, R7
        edge  CEC, 0, R4    ; falling edge opens the bit, one unit after the last wait
        ldi   R5, 11
        call  wait
        put   CEC, T        ; at 0.6 ms a 1 releases, a 0 stays low
        tick  R4
        ldi   R5, 8
        call  wait
        tick  R4            ; 1.05 ms
        get   CEC
        ldi   R5, 8
        call  wait
        put   CEC, 1        ; at 1.5 ms everything is released
        tick  R4
        ldi   R5, 17
        call  wait          ; 2.35 ms; the lead-in above makes it exactly 2.4
        jr    R6
wait:   tick  R4
        djnz  R5, wait
        ret
