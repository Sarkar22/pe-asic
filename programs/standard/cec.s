; HDMI CEC initiator on one open-drain pin. Bits last milliseconds, far past one tick's range,
; so every delay is a loop of 50 us units.
; MIN: one word per block: [15:8] byte, bit 7 EOM, bit 6 = 1 (the initiator's ACK slot).
; MOUT: 0 if every block was acknowledged, 1 if not. A frame to destination 15 is a broadcast, where
; HDMI CEC 1.4 inverts the ACK bit: a follower pulls the slot low to reject the frame, so for a
; broadcast a high slot is the acknowledgement and a low one the failure.
; Signal free time (HDMI CEC 1.4, CEC 9.1): a start bit falls only after the line has been high for
; 7 bit periods, the longest of the three, which an initiator needs before its own next frame. The
; count starts again whenever the line goes low, so another initiator's frame is waited out too, and
; it runs while the program waits for the host's first block.
; A block that fails ends the frame: the program takes the frame's later blocks, up to the one with
; EOM, from MIN unsent, then answers 1. It does not retransmit; the host may post the frame again.
; No arbitration.
.equ CEC io3
.equ UNIT 2500              ; 50 us at 50 MHz
.equ FREE 337               ; 7 bit periods of 48 units, and one more
.equ MIN 0
.equ MOUT 1
start:  pcfg  CEC, OD
        li    R4, UNIT
busy:   until CEC, 1        ; the count starts with the line high
        li    R5, FREE
free:   until CEC, 0, R4    ; one unit of high line, unless it goes low
        jt    busy
        djnz  R5, free
        addi  R5, 1         ; free for long enough: go on one unit at a time
        tryrecv R2, MIN     ; until the host's first block is there
        jf    free
        cp    R6, R2
        swap  R6
        andi  R6, 15
        addi  R6, 1         ; destination + 1: bit 4 is set for a broadcast
        edge  CEC, 0, R4    ; start bit, one unit after the count ends
        ldi   R5, 73
        call  wait
        ldi   R5, 16
        call  put1          ; released at 3.7 ms; 4.45 ms: the next bit opens one unit later
block:  cp    R0, R2        ; R2 keeps the block: bit 7 is EOM
        ldi   R1, 10        ; 8 data bits, EOM, ACK slot
b1:     shl16 R0, -, -      ; T = next bit, MSB first
        edge  CEC, 0, R4    ; the bit opens, one unit after the last wait
        ldi   R5, 11
        call  wait
        put   CEC, T        ; at 0.6 ms a 1 releases, a 0 stays low
        ldi   R5, 10
        call  wait          ; its last tick, at 1.05 ms, is the sample point
        get   CEC
        ldi   R5, 8
        call  wait
        ldi   R5, 18
        call  put1          ; at 1.5 ms everything is released; 2.35 ms: the next bit opens at 2.4
        djnz  R1, b1
        bx    R6, 4         ; T = line in the ACK slot, inverted for a broadcast: 1 is the failure
        bs    R1, 0         ; R1 = 1 from a failure on: the answer, and the rest of the frame is dropped
next:   bt    R2, 7
        jt    done
        recv  R2, MIN
        bt    R1, 0
        jf    block
        goto  next          ; up to EOM, the frame's blocks leave MIN unsent
done:   send  R1, MOUT
        goto  busy
put1:   put   CEC, 1        ; the wait's first tick commits it
wait:   tick  R4
        djnz  R5, wait
        ret
