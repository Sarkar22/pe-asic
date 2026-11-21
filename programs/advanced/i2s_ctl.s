; I2S or TDM controller on two shift units started at one absolute cycle: one unit makes SCK and WS (or FS), the
; other sends SD from a table in memory and receives the target's SD at SCK's rising edges.  Two threads, entries
; clk and dat, any two of the four.  512-word memory: the tables at 256 and 448.
;
; Pins: SCK io0 (output A of the clk unit, in PP; the dat unit reads it back as its outside clock), WS or FS uo5
; (output B of the clk unit), SD out uo6 (output A of the dat unit), SD in ui4.
; FMT at assembly: 0 for I2S, 1 for TDM (only WS's idle level depends on it).
; Memory, written by the host before RUN: 256 up, the data bit stream in 16-bit words, MSB first, from the first
; frame's first bit; 448 up, P words of the frame sync line, one bit per bit of the data, repeated (P covers a
; whole number of frames and of words).
; Host: thread clk takes from MCLK the start cycle T0 (the same for both threads, at least 2000 cycles ahead of
; TB), the number of words N, TPER, TFRAC, P, and the frame sync line's two bits before the first frame (bit 1
; first); thread
; dat takes from MDAT T0, N, TPER, TFRAC and RXCNT (N x 16 when that is 2047 or less, else 0).  MOUT gets the N
; words received, MSB first.
;
; The frame, as the I2S bus specification (Philips, 1996) and the TLV320ADC3140 data sheet's TDM mode with
; TX_OFFSET 1 and BCLK_POL 1 define it: SD and WS (FS) change on SCK's falling edges and are latched on its rising
; edges, MSB first; I2S: WS is 0 for the left word and 1 for the right, and changes one SCK period before each MSB;
; TDM: FS is high for the period before the first slot's MSB.  So the sync line is the data's frame pattern one bit
; ahead, which the host lays out in the table.
;
; How each step holds (section numbers are docs/ISA.md):
; - One grid (5.9.5, 7.3): both units use TPER and TFRAC with two symbols a bit; each thread pushes its first two
;   words, arms its send, writes T0 into D with wrd and runs tick 0, which commits on exactly T0.  Both sends start
;   there, their fraction accumulators together, so every later symbol of the two lands on one cycle.
; - SCK and WS (5.9.3): output A's codes are 2 then 3, low then high, whatever the bit, so SCK falls at each bit's
;   start and rises mid-bit; output B takes the bit in both symbols, so WS (FS) changes at the falling edges.  The
;   first word of both units has 2 bits: on the clk unit the sync line's two bits before the first MSB, on the dat
;   unit dummy zeros.  A receiver latches WS (FS) only at SCK's rising edges, so it sees the change before the
;   first word only if SCK has already risen once with the old level (I2S 3.2, 6.2).  TAIL 0, so after the last
;   word SCK idles low and WS at IDLEB.
; - Receive (5.9.6): RKIND 2 samples SD in when a rising edge of SCK, read back from io0, reaches SYNC2 (5.2: a uio
;   input reads its own drive); both pass the same synchroniser, so the sample is SD in at the rising edge.  The
;   half starts at a second commit, 4 x TPER after T0, between the second dummy bit's rising edge (T0 + 3 +
;   3 x TPER in SYNC2, plus the fraction's carries) and the first MSB's (T0 + 3 + 5 x TPER): from TPER 3, or 4 and 5
;   as TFRAC's carries grow (the bench sweeps it), and on time from TPER 3.
; - Feed (5.9.4): each unit takes one word per 16 bits, at a word boundary; the dat thread takes one received word
;   between its pushes.  The last word is marked last on both units, so both sends end on one cycle.
.equ SCK  io0
.equ WS   uo5
.equ SDO  uo6
.equ SDI  ui4
.equ NSCK 8             ; io0 and ui4 as input pin numbers, for the configuration words
.equ NSDI 4
.equ FMT  0
.equ WB   16
.equ MOUT 1
.equ MCLK 4
.equ MDAT 5
.equ TBL  256
.equ PAT  448
clk:    sstop txrx              ; a unit isn't reset when its thread starts
        pcfg  SCK, PP
        pcfg  WS, PP
        put   SCK, 0            ; SCK idles low
        put   WS, 1 - FMT       ; I2S: WS idles at the right channel, so the target sees it fall; TDM: FS low
        now
        li    R0, 0x0005 | ((1 - FMT) << 6)  ; SYMS 1, TMSB 1, IDLEA 0, IDLEB = the idle sync level
        scfg  R0, TXMODE
        ldi   R0, 0x0E          ; A: codes 2, 3 (SCK low, then high); B: code 0 in both symbols (the sync bit)
        scfg  R0, TXMAP
        ldi   R0, 0
        scfg  R0, CODING        ; no CRC, no stuffing
        recv  R1, MCLK          ; T0
        recv  R2, MCLK          ; N words
        recv  R0, MCLK
        scfg  R0, TXPER
        recv  R0, MCLK
        swap  R0
        scfg  R0, TXTAIL        ; TFRAC in bits 15:8, no tail
        recv  R4, MCLK          ; P pattern words
        recv  R0, MCLK          ; the sync line's two bits before the first MSB
        sstop tx                ; outputs at IDLEA and IDLEB
        smap  SCK, A
        smap  WS, B
        now                     ; SCK and WS follow the unit from here
        txw   R0, 2
        li    R5, PAT
        cp    R6, R4
        call  pword
        cmpi  R2, 1
        jt    clast1
        txw   R0, WB            ; two words wait before the start
        subi  R2, 1
        call  start
cloop:  call  pword
        cmpi  R2, 1
        jt    clast
        txw   R0, WB
        subi  R2, 1
        goto  cloop
clast1: txw   R0, WB, last
        call  start
        goto  done
clast:  txw   R0, WB, last
done:   sstat R0
        bt    R0, 5
        jt    done              ; until the send has ended
        halt
pword:  ldp   R0, R5            ; R0 = the next pattern word; R5 wraps to PAT after P words
        djnz  R6, pw1
        li    R5, PAT
        cp    R6, R4
pw1:    ret
start:  sgo   tx
        wrd   R1                ; D = T0
        tick  0                 ; commits on T0 exactly: the send starts there
        ret
dat:    sstop txrx
        pcfg  SDO, PP
        put   SDO, 0
        now
        ldi   R0, 0x05
        scfg  R0, TXMODE        ; SYMS 1, TMSB 1, output A takes the bit in both symbols, IDLEA 0
        ldi   R0, 0
        scfg  R0, TXMAP
        scfg  R0, CODING
        li    R0, 0x0006 | (NSDI << 8) | (NSCK << 12)    ; RKIND 2, RMSB 1, rising edges, RPIN SD in, RCLK SCK
        scfg  R0, RXMODE
        recv  R1, MDAT          ; T0
        recv  R2, MDAT          ; N words to send
        cp    R4, R2            ; and to receive
        recv  R3, MDAT
        scfg  R3, TXPER
        recv  R0, MDAT
        swap  R0
        scfg  R0, TXTAIL
        recv  R0, MDAT
        scfg  R0, RXCNT
        add   R3, R3
        add   R3, R3            ; 4 x TPER
        sstop tx
        smap  SDO, A
        now
        ldi   R0, 0
        txw   R0, 2             ; the dummy bits, under the sync line's two bits before the first MSB
        li    R5, TBL
        ldp   R0, R5
        cmpi  R2, 1
        jt    dlast1
        txw   R0, WB
        subi  R2, 1
        call  start
        sgo   rx
        tick  R3                ; T0 + 4 x TPER: the reception starts after the second dummy bit's rising edge
dloop:  ldp   R0, R5
        cmpi  R2, 1
        jt    dlast
        txw   R0, WB
        subi  R2, 1
        rxw   R1
        send  R1, MOUT
        subi  R4, 1
        goto  dloop
dlast1: txw   R0, WB, last
        call  start
        sgo   rx
        tick  R3
        goto  drest
dlast:  txw   R0, WB, last
drest:  rxw   R1                ; the words still to come
        send  R1, MOUT
        djnz  R4, drest
        goto  done
