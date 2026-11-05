; USB low-speed transmitter: NRZI and bit stuffing live in this loop, on the wire, at 33 cycles per bit.
; Memory at PKT: packed words, LSB first. First word is 0x80 | PID << 8, which is SYNC then PID.
; MBITS: total bits of the packet, 0 ends it. D+ on io0, D- on io1, idle J (D- high) on the pull-up.
.equ DP io0
.equ DM io1
.equ STEP 33
.equ PKT 96
.equ MBITS 7
.equ MDONE 1
start:  pcfg  DP, IN        ; claim first: a put on an unclaimed pin is dropped without a word
        pcfg  DM, IN
        put   DP, 0         ; J, ready for when the pins are enabled
        put   DM, 1
        now
pkt:    recv  R5, MBITS     ; bits in this packet
        ldi   R6, PKT
        ldi   R4, 6         ; ones left before a stuffed zero
        ldi   R1, 16
        ldp   R0, R6
        oe    DP, 1
        oe    DM, 1
        now                 ; driving J from here
        tick  STEP
txbit:  shr16 R0, -, -      ; T = bit, LSB first
        put   DP, ^!T       ; 0 toggles, 1 holds
        put   DM, ^!T
        tick  STEP
        jf    zero
        djnz  R4, one
        ldi   R4, 6         ; six ones: a zero goes out, not counted as data
        clrt
        put   DP, ^!T
        put   DM, ^!T
        tick  STEP
        goto  one
zero:   ldi   R4, 6
one:    djnz  R5, more
        put   DP, 0         ; EOP: SE0 for two bit times, then J
        put   DM, 0
        tick  STEP
        put   DM, 1
        tick  STEP*2
        oe    DP, 0
        oe    DM, 0
        tick  STEP
        ldi   R0, 1
        send  R0, MDONE
        goto  pkt
more:   djnz  R1, txbit
        ldi   R1, 16
        ldp   R0, R6
        goto  txbit
