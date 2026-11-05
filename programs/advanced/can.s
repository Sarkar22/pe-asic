; Classical CAN node at 500 kbit/s, three threads. Stuffing, destuffing and CRC-15 in firmware.
; samp (thread 1): samples at SP after every edge, or B after the last sample. It stays armed, so it
;   does nothing else. Each sample goes to the decoder as D with bit 0 replaced by the sample.
; dec (thread 0): one frame at a time, destuff, CRC, reports on MOUT, drives ACK at the right cycle
;   because it knows the sampler's D.
; tx (thread 2): one frame from memory. Pre-arms SOF: if another node starts first, the wait commits
;   our SOF 3 cycles later and we are in arbitration together.
.equ RX ui6
.equ TX uo5
.equ B 100           ; cycles per bit
.equ SP 65           ; sample point after an edge
.equ TSP 72          ; transmitter's own check of its bit, after it started driving
.equ MB 1            ; sample words
.equ MOUT 4          ; decoder reports
.equ MTX 2           ; bits to send, SOF to end of data
.equ MTS 5           ; transmitter status: 0 sent and ACKed, 1 no ACK, 2 lost arbitration, 3 bit error
.equ MOWN 6          ; full while our own frame is on the bus, so the decoder doesn't ACK it
.equ PKT 200
.equ WITH_RX 1
.equ WITH_TX 1
.if WITH_RX
; ---------------------------------------------------------------- sampler
samp:   li    R3, 0x4000
        ldi   R2, B
        ldi   R5, SP
        now
s0:     untilm R3, ui, chg, R2
        jf    s1
        tick  R5
s1:     rdd   R0
        get   RX
        bs    R0, 0
        trysend R0, MB
        goto  s0
; ---------------------------------------------------------------- decoder
dec:    li    R3, 0x8B32    ; 0x4599 << 1
        pcfg  TX, PP
        put   TX, 1
        now
fidle:  ldi   R1, 7         ; seven recessive samples mean a frame is over
fi1:    recv  R6, MB
        bt    R6, 0
        jf    fidle
        djnz  R1, fi1
wsof:   recv  R6, MB
        bt    R6, 0
        jt    wsof
        ldi   R2, 0         ; CRC. SOF was a 0, which leaves it at 0
        ldi   R5, 0         ; previous bit
        ldi   R4, 4         ; run of one
        ldi   R1, 18        ; ID RTR IDE r0 DLC
        call  gb
        cp    R1, R0
        andi  R1, 15
        add   R1, R1
        add   R1, R1
        add   R1, R1
        cmpi  R1, 0
        jt    crcf
        call  gb
crcf:   ldi   R1, 15
        call  gb
        cmpi  R2, 0         ; CRC clean: the remainder is zero
        jf    badcrc
        wrd   R6            ; D = the last sample of the CRC field
        tryrecv R6, MOWN    ; our own frame? give the word back and don't ACK
        jt    own
        pcfg  TX, PP
        put   TX, 0
        tick  B+B-SP        ; ACK slot starts B-SP after the delimiter's sample
        put   TX, 1
        tick  B
        ldi   R0, 0
        lui   R0, 1
        send  R0, MOUT      ; 0x100: good frame
        goto  fidle
own:    send  R6, MOWN
        goto  fidle
badcrc: ldi   R0, 1
        lui   R0, 1
        send  R0, MOUT      ; 0x101: CRC failed, no ACK
        goto  fidle
bad:    ldi   R0, 2
        lui   R0, 1
        send  R0, MOUT      ; 0x102: stuff error
        goto  fidle
; R1 bits, destuffed, CRC'd. A word goes out every 8 bits
gb:     recv  R6, MB
        bt    R6, 0
        bx    R5, 0         ; T = differs from the bit before
        cp    R5, R6
        jf    gb1
        ldi   R4, 4
        goto  gb2
gb1:    subi  R4, 1
gb2:    bt    R6, 0
        crcl  R2, R3
        shl16 R0, -, -
        cmpi  R4, 0
        jf    gb3
        recv  R6, MB        ; a stuff bit
        bt    R6, 0
        bx    R5, 0
        jf    bad           ; six equal
        cp    R5, R6
        ldi   R4, 4
gb3:    subi  R1, 1
        tsti  R1, 7
        jt    gb
        andi  R0, 255       ; the byte alone, or reports and data collide
        send  R0, MOUT
        cmpi  R1, 0
        jf    gb
        ret
.endif
.if WITH_TX
; ---------------------------------------------------------------- transmitter
tx:     li    R7, 0x8B32
        pcfg  TX, PP        ; a uo pin resets low, which is dominant: recessive before anything else
        put   TX, 1
        now
txg:    recv  R5, MTX
        send  R5, MOWN
        ldi   R6, PKT
        ldi   R2, 0
        ldi   R1, 1
        ldi   R3, 1         ; bit 0 previous, bit 1 driven, bit 2 next. Idle was recessive, SOF is 0
wait11: pcfg  TX, PP
        now
w10:    li    R0, 1000      ; 10 recessive bit times, nothing prepared
        until RX, 0, R0
        jt    w10
        put   TX, 0         ; only now: a prepared SOF during someone's frame would corrupt it
        ldi   R0, B
        until RX, 0, R0     ; someone else's SOF commits ours, 3 cycles later
        jt    tick1
        now                 ; idle: our SOF
tick1:  tick  TSP
cmp1:   bt    R3, 1
        bx    R3, 0
        jf    same
        ldi   R4, 4
        goto  c1
same:   subi  R4, 1
c1:     cmpi  R4, 0
        jf    c2
        bn    R3, 1         ; five equal: the next bit is the complement, nothing consumed
        goto  c3
c2:     djnz  R5, c4
        djnz  R6, crcg      ; a pointer, so it jumps; crcg leaves 1 here, so the second time it falls
tail:   get   RX
        put   TX, 1
        tick  B-TSP         ; CRC delimiter
        tick  B+TSP         ; into the ACK slot
        getn  RX
        ldi   R0, 0
        jt    acked
        ldi   R0, 1
acked:  tick  1000          ; ACK delimiter, EOF, intermission
        tryrecv R6, MOWN
        send  R0, MTS
        goto  txg
crcg:   cp    R0, R2        ; CRC next, left aligned, 15 bits
        ldi   R5, 15
        ldi   R6, 1
        ldi   R1, 17
c4:     djnz  R1, c5
        ldi   R1, 16
        ldp   R0, R6
c5:     shl16 R0, -, -      ; T = next bit
        crcl  R2, R7
c3:     bs    R3, 2
        get   RX
        bx    R3, 1
        jt    diff
        bt    R3, 2
        edge  TX, T, B-TSP
        shr8  R3, -, -      ; next to driven, driven to previous, in one instruction
        tick  TSP
        goto  cmp1
diff:   bt    R3, 1         ; driven 1, seen 0: arbitration lost. Driven 0, seen 1: a bit error
        ldi   R0, 2
        jt    lost
        ldi   R0, 3
lost:   put   TX, 1
        now
        tryrecv R6, MOWN    ; the decoder may ACK the winner
        send  R0, MTS
        goto  txg
.endif
