; CAN 2.0A node on the shift unit at 500 kbit/s and 50 MHz, one thread: the unit stuffs, destuffs, appends and
; checks the CRC-15 and times every bit; the firmware writes RXCNT once the DLC is in, monitors its own bits and
; arbitrates, drives the ACK slot and sends error flags.  229 words, and the 512-word memory for the frame to send
; at FRM.  References are to the Bosch CAN Specification 2.0, part A, by section name.
;
; Pins: TXD io0 in OD (released is recessive, so the transceiver's TXD pull-up keeps the bus recessive through
; reset and until the program runs, section 11.5), RXD ui6.  Dominant is 0.
;
; Host: memory FRM holds the frame to send, SOF to the end of the data field (data frame), as 19 + 8 x DLC bits
; sent first to last: the first (19 + 8 x DLC) mod 16 bits (3 or 11) right-aligned in FRM, then 16 bits a word.
; Posting the DLC to this thread's mailbox word (TID) sends it, retried after a lost arbitration.
; - MEND gets 0x4000 sent and acknowledged, 0x4001 no ACK (error flag sent), 0x4002 arbitration lost (the frame
;   goes again at the next bus idle), 0x4003 bit error (error flag sent); and for each received frame a status,
;   0x8000 | count << 8 | flags << 5 | n.
; - MOUT gets each received frame's words as they arrive, count of them, the last one right after its status.
;   They hold the frame's destuffed bits from SOF to the CRC delimiter, the first in bit 15; the last word holds n
;   bits.  flags are the end flags of section 5.9.7: 1 is a good frame (acknowledged); 0 a CRC or delimiter error,
;   2 or 6 a stuff error (an error flag is sent for both).  A reception under 16 bits isn't reported.
;
; How each step holds (section numbers are docs/ISA.md):
; - Receive (5.9.6, 5.9.7): timed samples, SPU cycles after each edge of RXD (RESYNC, RGAIN 0) and B after the
;   last sample, starting at a hunted falling edge (SOF).  Destuffing with SMODE 2, N 5; CRC-15 from SOF on
;   (SYNCLEN 0, CSKIP 0), MSB first (RMSB).  RXCNT = 35 + 8 x min(DLC, 8), or 35 for a remote frame, ends the
;   reception at the CRC delimiter: a stuff bit after the CRC sequence is dropped inside it, and the ACK slot is
;   the next bit.  CHK is the good residue 0 fed the delimiter's 1, which is POLY, so a dominant delimiter fails
;   the check too.  The DLC is whole only in the second word, at bit 32, so RXCNT is written within 3 bit times
;   of it (at DLC 0): 56 to 60 cycles after that word's last sample here.  The receive half hunts again after
;   every reception; the tail that an ACK or an error flag starts is a reception under 16 bits, dropped.
; - RESYNC pulls the sample on rising edges too (11.5), so a dominant bit stretched by more than B - SPU cycles,
;   by a transceiver or by two transmitters out of step in arbitration, moves the next sample into the next bit.
; - ACK (data frame, ACK field): the ACK bit waits in the send half as a 1-bit word of no kind, armed.  After the
;   last word, a good end flag starts it with a commit 21 cycles after the rxw, which is 1 to 4 cycles after the
;   receiver's own start of the ACK slot (the last word reaches the buffer one cycle after the delimiter's
;   sample, and the rxw takes it 1 to 4 cycles later).
; - Send (5.9.5, 5.9.7): TMSB, TPER B, words of kind crc and the last of kind fcs: the unit stuffs from SOF,
;   appends the CRC-15 and stuffs it, with the stuff bit after its last bit when due (coding), then shows the
;   idle level (recessive, TAIL 0) from the CRC delimiter on.  A send starts at a commit only (5.9.1).
; - Start of our frame: only after bus idle (10 recessive bit times), only if no reception started since (status
;   bit 12), CTX 1 just before the commit, and a wait that commits our SOF at once if another node's SOF has
;   just come.  A SOF of another node in the last few cycles is a simultaneous start, settled by arbitration.
; - Bit monitoring (error detection, bit error; arbitration): at SPF cycles into each bit on the send grid, the
;   snapshot holds both TXD (a uio pin reads its own pad, 5.2) and RXD, and xget compares them.  A mismatch sets
;   CTX 0 and stops the send 17 to 20 cycles after the check point (sstop tx: idle levels from the next cycle).
;   Sent recessive, seen dominant, before the reception's first word: arbitration lost.  Anything else: a bit
;   error, and an error flag from the next bit on the send grid.
; - Arbitration lost: the receive half has received the bus since SOF, so its reception is the winner's frame.
;   The CRC register was the send half's, which fed it our bits; up to the lost bit they equal the bus's.  CTX 0
;   comes before the receive half samples that bit (SPF + 16 < SPU - 1 when we follow the winner's SOF by 3
;   cycles), so the receive half goes on from the CRC of the bus bits before it (section 10 item 17), checks the
;   frame, and the node acknowledges it like any other.
; - Error flags (error frame; error signalling): six dominant bits, a 6-bit word of no kind, not stuffed.  After
;   a stuff error at once (the reception ends a bit after the violation, so the flag starts a bit or more late,
;   which the superposition of error flags allows); after a CRC error at the bit after the ACK delimiter; after a
;   bit error or a missing ACK at the next bit.
; - Not done: extended frames (IDE 1) are taken for errors; no error counters or error-passive state, no overload
;   frames.
.equ TXP io0
.equ RXP ui6
.equ NRX 6
.equ B 100              ; cycles per bit
.equ SPU 75             ; the receive half samples SPU cycles after an edge
.equ SPF 40             ; the firmware's check point in each bit of its own frame
.equ TID 0              ; the thread this image runs on: its mailbox word takes send requests
.equ MOUT 4             ; received words, as they arrive
.equ MEND 5             ; a status per received frame, and send reports
.equ FRM 256
.equ CODRX 0x588E       ; CRC-15 for the receive half, equal-bit stuffing with N = 5
.equ CODTX 0x5A8E       ; the same with CTX 1: the send half appends
.equ ACKW (SPU - SPF)   ; the second look at the ACK slot, after the first
.equ KCRC 3 * B - SPU - 47     ; D to the commit of a CRC error's flag: EOF's first bit starts 3 B - SPU - 2
                                ; after the delimiter's sample, which is 2 to 5 cycles before the rxw, 43 to 46
                                ; before D
start:  sstop txrx              ; a unit isn't reset when its thread starts
        pcfg  TXP, OD           ; released: recessive
        li    R0, 0x01E4        ; TMSB, idle and end levels 1, one symbol per bit
        scfg  R0, TXMODE
        ldi   R0, 0
        scfg  R0, TXMAP         ; output A takes the bit
        scfg  R0, TXTAIL        ; no tail: the idle level from the CRC delimiter on
        scfg  R0, RXEND
        scfg  R0, RXCNT         ; SYNCLEN 0
        scfg  R0, RXSYNC
        scfg  R0, POLYLO
        scfg  R0, CHKLO
        ldi   R0, B
        scfg  R0, TXPER
        scfg  R0, RXPER
        ldi   R0, SPU
        scfg  R0, RXFIRST
        li    R0, 0x0024 | (NRX << 8)   ; timed samples, RMSB, HUNT for a falling edge of RXD
        scfg  R0, RXMODE
        li    R0, 0x0100        ; RESYNC, RGAIN 0
        scfg  R0, RXOPT
        li    R0, 0x8B32        ; x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1, left-aligned
        scfg  R0, POLYHI
        scfg  R0, CHKHI         ; the residue after the CRC delimiter
        sstop tx                ; output A shows its idle level, recessive
        smap  TXP, A
        now                     ; commits the switch, while the node is still off the bus
        li    R7, CODRX
        scfg  R7, CODING        ; CTX 0: the register is the receive half's
        now
bint:   li    R0, 11 * B        ; 11 recessive bits before joining the bus (bus integration, part B)
        until RXP, 0, R0
        jt    bint
        sgo   rx
        now                     ; the commit starts the hunt for a SOF
; ---------------------------------------------------------------- wait for bus idle (10 recessive bit times)
wbus:   li    R7, CODRX         ; a call may have used R7
        now                     ; D fresh for the timed wait
w10:    li    R0, 10 * B
        until RXP, 0, R0        ; a dominant bit within 10 bit times:
        jf    widle
        sstat R5
        bt    R5, 8             ; a reception runs (a frame, or an ACK's or error flag's tail),
        bo    R5, 9             ; or a word waits: take it
        jt    rxf
        goto  w10
widle:  sstat R5                ; clears status bit 12: from here it shows a reception that started
        bt    R5, 9
        jf    bidle
        rxw   R0                ; drop the last word of a tail that ended meanwhile
        goto  widle
bidle:  until RXP, 0, inbox     ; a SOF, or a send request in this thread's mailbox word
        jf    tx
; ---------------------------------------------------------------- receive a frame
rxf:    ldi   R2, 0
        txw   R2, 1, last       ; the ACK bit waits in the send half, armed: no commit comes before the ACK's
        sgo   tx
rx0:    rxw   R0                ; the first word, or the last word of a reception under 16 bits
        jf    rx1
        sstat R5                ; that one isn't a frame: drop it
        bt    R5, 8
        jt    rx0               ; a reception runs: take its first word
        sstop tx
        goto  wbus
rx1:    send  R0, MOUT          ; each word goes to the host as it arrives
        ldi   R4, 1             ; words of this frame
        rxw   R1                ; the second word: the DLC's bits 2 to 0 are its top bits
        jt    rxe1
        cp    R2, R1
        swap  R2
        andi  R2, 0xE0
        clrt
        shr8  R2, -, -
        shr8  R2, -, -          ; 8 x DLC bits 2..0
        bt    R0, 0             ; DLC bit 3: 8 bytes
        jf    rx2
        ldi   R2, 64
rx2:    bt    R0, 3             ; RTR: a remote frame has no data field
        jf    rx3
        ldi   R2, 0
rx3:    addi  R2, 35            ; SOF to the CRC delimiter, destuffed
        scfg  R2, RXCNT
        cp    R0, R1
rx4:    send  R0, MOUT
        addi  R4, 1
        rxw   R0
        jf    rx4
rxend:  srd   R3, end           ; the last word's end flags
        cmpi  R3, 1             ; ended by RXCNT, CRC and delimiter good, no stuff violation
        jf    rxbad
        now                     ; the ACK bit, for one bit time from the start of the ACK slot
        call  report
        goto  wbus
rxe1:   cp    R0, R1            ; a frame that ended between bits 17 and 32
        srd   R3, end
rxbad:  sstop tx                ; no ACK: disarm and drop the ACK bit
        ldi   R2, 0
        txw   R2, 6, last       ; an error flag: six dominant bits, not stuffed
        bt    R3, 1             ; a stuff violation: the flag at once
        jt    rxb1
        now                     ; a CRC or delimiter error: D 41 cycles after the rxw, before the arm, since
        sgo   tx                ; any commit starts an armed half
        li    R2, KCRC
        tick  R2                ; the flag from the bit after the ACK delimiter
        goto  rxb2
rxb1:   sgo   tx
        now
rxb2:   call  report
        goto  wbus
; the status to MEND (count << 8 | flags << 5 | the last word's bits, bit 15 set), then the last word to MOUT
report: sstat R5
        andi  R5, 31
        addi  R4, 1
        cp    R2, R4
        ori   R2, 0x80
        swap  R2
        or    R2, R5
        add   R3, R3
        add   R3, R3
        add   R3, R3
        add   R3, R3
        add   R3, R3
        or    R2, R3
        send  R2, MEND
        send  R0, MOUT
        ret
; ---------------------------------------------------------------- send a frame
tx:     recv  R2, TID           ; the DLC
        li    R6, FRM
        ldp   R0, R6            ; the first 3 or 11 bits
        ldp   R1, R6            ; the next 16
        cp    R3, R2
        clrt
        shr8  R3, -, -          ; T = DLC odd
        cp    R4, R3            ; words left after these two
        jt    tx11
        txw   R0, 3, crc
        goto  tx1
tx11:   txw   R0, 11, crc
tx1:    cmpi  R4, 0
        jf    tx2
        txw   R1, 16, fcs, last
        goto  tx3
tx2:    txw   R1, 16, crc
tx3:    sgo   tx
        li    R3, CODTX
        ldi   R0, 0
        sstat R5
        bt    R5, 12            ; a reception started since the bus went idle?
        jt    txback
        scfg  R3, CODING        ; CTX 1: tx_begin starts the CRC at the commit
        until RXP, 0, R0        ; the bus now: a SOF that just came commits our start with it
        jt    tx4
        now                     ; our SOF
tx4:    tick  SPF               ; D is our SOF's commit
mon:    get   TXP               ; at the check point: T = our TXD
        xget  RXP               ; T = TXD xor RXD
        jt    bad
        tick  B
        sstat R5
        bt    R5, 6             ; the send buffer is full
        jt    mon
        bt    R5, 5             ; the send is over: that check was the CRC delimiter's
        jf    sent
        cmpi  R4, 0
        jt    mon
        ldp   R0, R6
        subi  R4, 1
        cmpi  R4, 0
        jt    mlast
        txw   R0, 16, crc
        goto  mon
mlast:  txw   R0, 16, fcs, last
        goto  mon
sent:   getn  RXP               ; the ACK slot's check point
        tick  ACKW
        getn  RXP               ; and a later look: T = 1 if dominant
        jf    noack
        call  rearm
        now                     ; the hunt starts after the ACK's edge
        ldi   R0, 0
        lui   R0, 0x40
        send  R0, MEND          ; sent and acknowledged
        goto  wbus
noack:  ldi   R0, 0
        txw   R0, 6, last       ; an error flag from the ACK delimiter on
        sgo   tx
        tick  B - SPU           ; its commit starts the flag
        call  rearm             ; the next commit starts the hunt
        ldi   R0, 1
        lui   R0, 0x40
        send  R0, MEND
        goto  wbus
rearm:  sstop rx                ; drop our own frame's reception, whose CRC was the send half's
        li    R3, CODRX
        scfg  R3, CODING        ; CTX 0 before the next reception begins
        sgo   rx
        ret
bad:    scfg  R7, CODING        ; CTX 0: the register goes to the receive half before it samples this bit
        sstop tx                ; none of our later bits go out
        get   TXP               ; T = what we sent at this check
        jf    berr              ; dominant sent, recessive seen
        sstat R5
        bt    R5, 9             ; the reception's first word is in: past the arbitration field
        jt    berr
        ldi   R0, 2
        lui   R0, 0x40
        send  R0, MEND          ; arbitration lost
        send  R2, TID           ; our frame again at the next bus idle (Bosch 6.1: retransmitted)
        goto  rxf               ; the reception in progress is the winner's frame
berr:   ldi   R0, 0
        txw   R0, 6, last       ; an error flag from our next bit on
        sgo   tx
        tick  B - SPF
        call  rearm
        ldi   R0, 3
        lui   R0, 0x40
        send  R0, MEND
        goto  wbus
txback: sstop tx                ; a frame started first: receive it, and try again after it
        send  R2, TID
        goto  rxf
