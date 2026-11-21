; SPI target on the shift unit: MOSI by the receive half on SCK's rising edges, MISO by the send half on SCK's
; edges, modes 0 and 3, MSB first, frames started by CS.  One thread, 50 words (52 with MODE 3), and the
; 512-word memory for the MISO table at 256.
;
; Pins: CS ui3, SCK ui4, MOSI ui5 (inputs), MISO uo2.  Modes 0 and 3 both capture on SCK's rising edge and
; change on the falling edge (Motorola SPI; RP2350 datasheet 12.3.4.10 and 12.3.4.13), so one configuration
; serves both, except for mode 3's first bit (below).  MODE and TEDGE are constants at assembly.
;
; Host: mailbox MCFG takes two words at start, RXCNT (bits per frame, 1 to 2047, or 0 for one endless reception
; that ignores CS after its first fall) and RXEND (cycles with no rising SCK edge that end a frame early, or 0).
; Memory 256 up holds the MISO words, 16 bits each, sent MSB first; the host writes them before RUN.  Every
; frame sends them from the start.  Mailbox MOUT gets each received word, MSB first: the first bit of a word is
; its bit n - 1.  After a frame's last word, MEND gets that word's bit count (section 5.9.6, Words and End).
;
; How each step holds (section numbers are docs/ISA.md):
; - Receive (5.9.6): RKIND 2 samples MOSI in the cycle a rising edge of SCK reaches SYNC2.  Both pins pass the
;   same synchroniser, so the sample is MOSI as it was at the edge, within section 10 item 2's one cycle.  RMSB 1
;   puts the first bit of a word in bit n - 1.  RXCNT ends a frame after its last bit; RXEND ends one that stops
;   early, which brings the target back in step after a frame the controller cut short.
; - Send (5.9.5): TEXT with TFALL moves MISO 3 cycles, pad to pad, after each falling edge of SCK (TEDGE 1), or
;   after each rising edge (TEDGE 0, which gives the controller a whole period less 3 cycles of setup, and 2 to
;   3 cycles of hold after its capture edge).  TMSB 1 sends bit n - 1 first.  A send's first symbol lands at the
;   commit that starts it, so with TEDGE 1 mode 3's first falling edge, which puts out bit 1, would move MISO on
;   too soon: a 1-bit dummy word comes first, and the first falling edge ends it (DUMMY below).
; - Start (5.9.1, 3.3): both halves are armed by sgo, and the wait for CS low is the commit that starts them, on
;   the cycle the fall reaches SYNC2.  MISO's first bit is on the pad 3 cycles after CS falls (5.7, pre-armed
;   reaction), and the receive half has begun before the first SCK edge reaches SYNC2.  The wait for CS high
;   before sgo keeps the start out of the frame that just ended.
; - Feed (5.9.4): two MISO words wait before the start; each received word lets one more in, so the send half is
;   always a word ahead.  The send buffer is read only at a word boundary, a word time after the last refill.
; - End: a frame's last word ends rxw with T = 1.  The thread stops both halves, which drops the MISO words the
;   controller didn't clock, refills two, and waits for CS to rise and fall again.  The frame's last word and its
;   bit count go to the host after the next start.  The wait for CS high is armed at most 27 cycles after the
;   frame's last capture edge (31 in mode 3), and must see CS high before CS falls again; the wait for CS low is
;   armed 5 to 8 cycles after it sees CS high.  So framed words need CS high for about 27 - 2H cycles at half
;   period H when CS rises a period after the last capture, as the PL022 does: 11 cycles at half period 8, which
;   the PL022's one-period pulse gives down to half period 7 (the bench measures it).
.equ CS   ui3
.equ SCK  ui4
.equ MOSI ui5
.equ MISO uo2
.equ NSCK 4             ; SCK and MOSI as numbers, for the configuration words
.equ NMOSI 5
.equ MODE 0             ; 0 or 3
.equ TEDGE 1            ; 1: MISO moves after falling edges, as SPI defines it. 0: after rising edges
.equ DUMMY (MODE // 3) * TEDGE
.equ MOUT 1
.equ MEND 2
.equ MCFG 7
.equ TABLE 256
start:  sstop txrx              ; a unit isn't reset when its thread starts
        pcfg  MISO, PP
        put   MISO, 0           ; MISO idles low
        now
        recv  R1, MCFG          ; RXCNT
        recv  R2, MCFG          ; RXEND
        li    R0, 0x0204 | (TEDGE << 10) | (NSCK << 11)    ; TMSB, TEXT, TFALL = TEDGE, TCLK = SCK, idle 0
        scfg  R0, TXMODE
        ldi   R0, 0
        scfg  R0, TXMAP         ; one symbol per bit, output A takes the bit
        scfg  R0, TXPER
        scfg  R0, TXTAIL        ; no tail: an underrun shows the idle level
        scfg  R0, CODING        ; no CRC, no stuffing
        li    R0, 0x0006 | (NMOSI << 8) | (NSCK << 12)     ; RKIND 2, RMSB, rising edges, RPIN MOSI, RCLK SCK
        scfg  R0, RXMODE
        scfg  R1, RXCNT         ; SYNCLEN 0: words start at once
        scfg  R2, RXEND
        smap  MISO, A           ; MISO follows output A from the next commit
        li    R5, TABLE
        ldp   R3, R5            ; the first two MISO words stay in R3 and R4
        ldp   R4, R5            ; R5 = TABLE + 2, where each frame's refill continues
        goto  first
word:   rxw   R1                ; T = 1 on a frame's last word
        jt    last
        send  R1, MOUT
        ldp   R0, R6
        txw   R0, 16            ; one more MISO word for each word received
        goto  word
last:   sstop txrx              ; drop what the controller didn't clock
.if DUMMY
        txw   R3, 1             ; mode 3: a dummy symbol until the first falling edge
        txw   R3, 16
.else
        txw   R3, 16
        txw   R4, 16
.endif
        until CS, 1             ; the frame before has ended on the bus
        sgo   txrx
        until CS, 0             ; the fall commits: both halves start on that cycle
        cp    R6, R5
        send  R1, MOUT          ; then the last word of the frame before
        sstat R2
        andi  R2, 31
        send  R2, MEND          ; and its bit count
.if DUMMY
        txw   R4, 16            ; waits for the first falling edge to take the dummy
.endif
        goto  word
first:                          ; the same start, with no frame before it
.if DUMMY
        txw   R3, 1
        txw   R3, 16
.else
        txw   R3, 16
        txw   R4, 16
.endif
        until CS, 1
        sgo   txrx
        until CS, 0
.if DUMMY
        txw   R4, 16
.endif
        cp    R6, R5
        goto  word
