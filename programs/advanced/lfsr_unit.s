; PRBS generator and additive scrambler on the shift unit's LFSR (CMODE 2), with a second unit that receives the
; line raw or descrambles it.  Two threads, tx and rx, 128-word memory.
;
; Threads: tx (entry tx) sends on TX io0 in PP; rx (entry rx) receives on input pin NRX, io0 for a loopback (a
; uio pin reads its own drive, section 5.2).  W picks the sequence, 7, 9 or 15: PRBS7 x^7 + x^6 + 1, and ITU-T
; O.150's 2^9 - 1 (x^9 + x^5 + 1) and 2^15 - 1 (x^15 + x^14 + 1).  INV 1 sends the PRBS complemented, for a
; checker of the other sense.  These are constants at assembly.
;
; Host: tx takes the bit time from mailbox MBIT, then requests from MTX, one word each: bits 7:0 a count K of
; 16-bit words, 1 to 127; bit 15 clear sends the sync byte and then K words of zeros, so the line carries the
; PRBS; bit 15 set sends the sync byte and then K words that follow in MTX, scrambled; bit 14 set sends the sync
; byte and then the PRBS for ever.  After each frame MTS gets the status bits seen during the send (bit 7, TXUR,
; must be 0).  rx takes from mailbox MRXC its bit time, the CODING word (0 for the raw line, or the LFSR's value
; below with CTX 0 to descramble) and K, then sends every received word to MRX.
;
; How each step holds (section numbers are docs/ISA.md):
; - LFSR (5.9.7, LFSR mode): CMODE 2 runs the register as a Galois LFSR of POLY, each covered bit advancing it with
;   bit 31 as the feedback, and a covered data bit goes out XOR bit 31.  POLY is the reciprocal of the polynomial
;   the standard names, x^w + x^(w - m) + 1 for x^w + x^m + 1 (POLY for a standard's PRBS), so the line satisfies
;   the named Fibonacci recurrence: bit n is the XOR of bits n - m and n - w.  CINIT 1: from all ones, at the
;   send's start.
; - Frame (5.9.5): the sync byte SYNC is a word of no kind, so it is neither scrambled nor advances the LFSR; the
;   data words are of kind crc, so the LFSR covers them from their first bit, all ones there.  The receiver's
;   LFSR starts from all ones at its first covered bit, the first bit after the sync pattern (CSKIP 0), so the two
;   undo each other: an additive scrambler.
; - Receive (5.9.6, timed): HUNT for the sync byte's first bit, a fall from the idle 1, the first sample half a
;   bit later, then one a bit.  SYNCLEN 8 finds the sync byte, laid out in the top 8 bits for LSB first, and RCNT
;   16 K ends the frame after its last data bit.
; - Feed (5.9.4): a word waits in the buffer while one is sent, 16 bit times.  The finite loops take 5 slots a word
;   and the endless one 2, so a frame keeps up from 2 cycles a bit, and the endless PRBS runs at one bit a cycle,
;   50 Mbit/s, with TPER 0.  The receiver needs 2 cycles a bit or more: its first sample is half a bit after the
;   fall, and FIRST 0 acts as 1 (5.9.3).
.equ TX   io0
.equ NRX  8
.equ W    7
.equ INV  0
.equ SYNC 0x96          ; LSB first on the line: 0 1 1 0 1 0 0 1
.equ MBIT 7
.equ MTX  0
.equ MTS  2
.equ MRXC 6
.equ MRX  1
.if W - 7
.if W - 9
.equ POLY 0x0006        ; W 15: x^15 + x + 1, the reciprocal of x^15 + x^14 + 1
.else
.equ POLY 0x0880        ; W 9: x^9 + x^4 + 1, the reciprocal of x^9 + x^5 + 1
.endif
.else
.equ POLY 0x0600        ; W 7: x^7 + x + 1, the reciprocal of x^7 + x^6 + 1
.endif
.equ LFSR 0x0320 | (W - 1)      ; CW w - 1, CINIT 1, CXOR 0, CMODE 2, CTX 1
tx:     sstop txrx              ; a unit isn't reset when its thread starts
        pcfg  TX, PP
        put   TX, 1             ; the line idles high
        now
        recv  R2, MBIT
        scfg  R2, TXPER
        ldi   R0, 0x20
        scfg  R0, TXMODE        ; NRZ, LSB first, IDLEA 1
        ldi   R0, 0
        scfg  R0, TXTAIL
        scfg  R0, POLYLO
        li    R0, POLY
        scfg  R0, POLYHI
        li    R0, LFSR
        scfg  R0, CODING
        sstop tx                ; output A shows IDLEA
        smap  TX, A
        now                     ; TX follows output A from here
req:    recv  R3, MTX
        cp    R4, R3
        andi  R4, 0x7F          ; K
        ldi   R0, INV
        bt    R3, 15
        jf    map
        ldi   R0, 0             ; frames go out as they are
map:    scfg  R0, TXMAP         ; output A: the bit, or with INV its complement (code 1)
        li    R0, SYNC
        txw   R0, 8             ; the sync byte, no kind: it moves into the shift register
        ldi   R0, 0             ; the PRBS sends zeros
        bt    R3, 14
        jt    forever
        bt    R3, 15
        jf    first
        recv  R0, MTX           ; a frame's first data word
first:  cmpi  R4, 1
        jt    one
        txw   R0, 16, crc       ; it waits in the buffer behind the sync byte
        sgo   tx
        now                     ; E: the sync byte's first bit is on the pad from E + 1
        subi  R4, 1
        bt    R3, 15
        jt    data
zero:   cmpi  R4, 1             ; R0 is 0
        jt    zlast
        txw   R0, 16, crc       ; zeros: the line carries the LFSR's own sequence
        subi  R4, 1
        goto  zero
zlast:  txw   R0, 16, crc, last
        goto  wend
data:   recv  R0, MTX
        cmpi  R4, 1
        jt    dlast
        txw   R0, 16, crc
        subi  R4, 1
        goto  data
dlast:  txw   R0, 16, crc, last
        goto  wend
one:    txw   R0, 16, crc, last
        sgo   tx
        now
wend:   ldi   R5, 0
wend2:  sstat R1
        or    R5, R1            ; sstat clears TXUR once read: keep every bit seen
        bt    R1, 5             ; the send half is busy until the last bit ends
        jt    wend2
        send  R5, MTS
        goto  req
forever: txw  R0, 16, crc
        sgo   tx
        now
floop:  txw   R0, 16, crc
        goto  floop
rx:     sstop txrx
        recv  R2, MRXC
        scfg  R2, RXPER
        cp    R1, R2
        clrt
        shr16 R1, -, -
        scfg  R1, RXFIRST       ; half a bit after the fall: the middle of the sync byte's first bit
        li    R0, 0x0020 | (NRX << 8)
        scfg  R0, RXMODE        ; timed samples, LSB first, HUNT for a fall to 0
        li    R0, SYNC << 8
        scfg  R0, RXSYNC        ; LSB first: the pattern in the top 8 bits
        recv  R0, MRXC
        scfg  R0, CODING        ; 0: the raw line; LFSR with CTX 0: descramble
        recv  R3, MRXC          ; K
        add   R3, R3
        add   R3, R3
        add   R3, R3
        add   R3, R3            ; 16 K bits
        li    R0, 0x4000
        add   R3, R0            ; SYNCLEN 8
        scfg  R3, RXCNT
        ldi   R0, 0
        scfg  R0, RXEND
        scfg  R0, RXOPT         ; no RESYNC, CSKIP 0
        scfg  R0, POLYLO
        li    R0, POLY
        scfg  R0, POLYHI
        sgo   rx
        now                     ; this commit starts the hunt, which then goes on by itself
rloop:  rxw   R0
        send  R0, MRX
        goto  rloop
