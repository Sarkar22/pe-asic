; UART with a parity bit, 8E1 or 8O1, on two shift units: one thread's unit sends, appending the parity bit from
; its CRC register, and another thread's unit receives, checking the parity and the stop bit with its own.  The
; two threads are independent; a loopback runs both on one io pin.  512-word or 128-word memory.
;
; Threads: tx (entry tx) sends, rx (entry rx) receives.  Pins: TX io0 in PP, RX the input pin NRX (io0 for a
; loopback: a uio pin reads its own drive, section 5.2).  ODD 0 builds even parity, 1 odd.  These are constants
; at assembly.
;
; Host: each thread takes the bit time in cycles from mailbox MBIT at start, so the host posts it twice.  tx: mailbox
; MTX takes the bytes; bit 8 set marks the last byte of a burst, which ends the send, and then MTS gets the status
; bits seen during the send (bit 7, TXUR, must be 0).  rx: MRX gets one word per frame, the 10 bits received LSB
; first (data in bits 7:0, the parity bit in bit 8, the stop bit in bit 9), with bit 15 set when the unit's check
; passed: a frame is good when bits 15 and 9 are both set.
;
; How each step holds (section numbers are docs/ISA.md):
; - Parity (5.9.7, Parity): width 1 (CW 0), POLY x + 1 (POLYHI 0x8000), CINIT 0: the register is the running XOR
;   of the covered bits.  On send, the data word is of kind fcs, so its 8 bits are covered and the register's one
;   bit is appended after them, XOR CXOR: even parity with CXOR 0, odd with CXOR 1.  The start bit and the stop
;   bit are words of no kind, not covered, and the register starts again after each append, so each frame's
;   parity covers only its own data.
; - Frames back to back (5.9.5): one send holds a whole burst: the start bit, then for each byte its data word
;   (8 bits, then the parity bit), then a 2-bit word holding the stop bit and the next start bit, and a 1-bit stop
;   bit marked last.  The buffer is read at each word boundary.  The thread puts the 2-bit word while the 9 data
;   and parity bits go out, and the next data word in the 2 bit times of that word.  Bit times from 3 cycles up
;   (16.7 Mbaud) keep a burst whole on every thread pair and alignment; at 2 the send half underruns after two
;   bytes (TXUR), and the thread then blocks in txw, since no commit starts another send (section 10, item 10).
;   The host starts the receiving thread first: the bench posts the bytes 400 cycles after RUN.
; - Receive (5.9.6, timed): HUNT for a fall to 0 on RX, the first sample 1.5 bits after it, in the middle of data
;   bit 0, then one sample a bit.  RCNT 10 ends each reception at the stop bit, so each frame is one reception and
;   one word marked last, and the half hunts again for the next start bit, half a bit later.
; - Check (5.9.7): the receive half's register covers the 8 data bits, the parity bit and the stop bit.  The data
;   and parity bits of a good frame XOR to 0 (even) or 1 (odd), and the stop bit, a 1, then moves that bit out and
;   feeds a 1 in: the residue is 0x80000000 for even parity and 0 for odd (CHK), and a frame passes only if its
;   parity is right and its stop bit is 1, or both are wrong, which bit 9 then shows.
.equ TX   io0
.equ NRX  8             ; RX pin as an input number: io0
.equ ODD  0
.equ MBIT 7
.equ MTX  0
.equ MRX  1
.equ MTS  2
tx:     sstop txrx              ; a unit isn't reset when its thread starts
        pcfg  TX, PP
        put   TX, 1             ; the line idles high
        now
        recv  R2, MBIT
        scfg  R2, TXPER
        ldi   R0, 0x20
        scfg  R0, TXMODE        ; NRZ, one symbol per bit, LSB first, IDLEA 1
        ldi   R0, 0
        scfg  R0, TXMAP
        scfg  R0, TXTAIL
        scfg  R0, POLYLO
        li    R0, 0x8000
        scfg  R0, POLYHI        ; x + 1, width 1
        li    R0, 0x0280 | (ODD << 6)
        scfg  R0, CODING        ; CW 0, CINIT 0, CXOR ODD, CMODE 1, CTX 1: the send half's register
        sstop tx                ; output A shows IDLEA
        smap  TX, A
        now                     ; TX follows output A from here
burst:  ldi   R5, 0
        recv  R0, MTX           ; the first byte of a burst
        ldi   R1, 0
        txw   R1, 1             ; the start bit, no kind
        txw   R0, 8, fcs        ; the data, LSB first, then the parity bit
        sgo   tx
        now                     ; E: the start bit is on the pad from E + 1
next:   bt    R0, 8             ; T = the byte just pushed was the last
        jt    fin
        recv  R0, MTX
        ldi   R1, 1
        txw   R1, 2             ; the stop bit (1), then the next start bit (0)
        txw   R0, 8, fcs
        goto  next
fin:    ldi   R1, 1
        txw   R1, 1, last       ; the stop bit ends the send
wait:   sstat R4
        or    R5, R4            ; sstat clears TXUR once read: keep every bit seen
        bt    R4, 5             ; the send half is busy until the stop bit ends
        jt    wait
        send  R5, MTS
        goto  burst
rx:     sstop txrx
        recv  R2, MBIT
        scfg  R2, RXPER
        cp    R1, R2
        clrt
        shr16 R1, -, -          ; half a bit
        add   R1, R2
        scfg  R1, RXFIRST       ; 1.5 bits after the fall: the middle of data bit 0
        li    R0, 0x0020 | (NRX << 8)
        scfg  R0, RXMODE        ; timed samples, LSB first, HUNT for a fall to 0, RPIN RX
        ldi   R0, 10
        scfg  R0, RXCNT         ; 8 data bits, the parity bit and the stop bit; no sync pattern
        ldi   R0, 0
        scfg  R0, RXEND
        scfg  R0, RXOPT         ; no RESYNC, CSKIP 0
        scfg  R0, POLYLO
        scfg  R0, CHKLO
        li    R0, 0x8000
        scfg  R0, POLYHI
        li    R0, (1 - ODD) << 15
        scfg  R0, CHKHI         ; 0x80000000 for even parity, 0 for odd, with the stop bit
        li    R0, 0x0080
        scfg  R0, CODING        ; CW 0, CINIT 0, CMODE 1, CTX 0: the receive half's register
        sgo   rx
        now                     ; this commit starts the hunt, which then goes on by itself
rloop:  rxw   R0                ; one word per frame, marked last
        srd   R1, end
        bt    R1, 0             ; end flag 0: the check passed
        bs    R0, 15
        send  R0, MRX
        goto  rloop
