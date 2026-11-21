; I2S or TDM target on the shift unit: the receive half takes SD in at SCK's rising edges, the send half puts SD out
; on the controller's grid, and both start again at every frame, on the edge that latches the frame's WS (FS)
; change.  One thread, 512-word memory: the table of words to send at 256.
;
; Pins: SCK ui4, WS or FS ui5, SD in ui6 (inputs), SD out uo2 (output A).
; FMT at assembly: 0 for I2S, where a frame starts at the left word, after WS falls; 1 for TDM, where a frame
; starts after FS rises.  WB: the bits of a word, 16, or the frame's length when that is shorter (8 for one 8-bit
; slot).
; Host: mailbox MCFG takes W, the bits of a frame (WB to 2047, a whole number of words), then K = W / WB, the words
; of a frame.  Memory 256 up holds the words to send, WB bits each, MSB first, K a frame; the host writes them
; before RUN.  Mailbox MOUT gets each received word, MSB first, K a frame.
;
; The frame, as the I2S bus specification (Philips, 1996) and the TLV320ADC3140 data sheet's TDM mode with
; TX_OFFSET 1 and BCLK_POL 1 define it: SD and WS (FS) are latched on SCK's rising edges; WS changes one SCK
; period before the MSB of each word (I2S 3.2), FS is high for the period before the first slot's MSB.  So a frame
; starts at the rising edge that first latches the new level, and its first bit is the next rising edge's.
;
; How each step holds (section numbers are docs/ISA.md):
; - Start (5.6, 5.9.1): the wait untilm on SCK and WS (FS) together matches in the cycle a rising edge of SCK
;   reaches SYNC2 with WS low (FS high), which is the frame's first edge when WS (FS) changes while SCK is low, as a
;   controller that changes it on the falling edge does (I2S 3.1, TDM's BCLK_POL 1).  The wait is armed while WS
;   is high (FS low), so it can't match inside a frame.  Its match is the commit that starts both halves, so the
;   first frame starts exactly there, and its first bit is the next rising edge's.
; - Receive (5.9.6): RKIND 2 samples SD in when a rising edge of SCK reaches SYNC2; both pass the same
;   synchroniser, so the sample is SD in at the edge.  RXCNT W ends each frame on its last bit, which is latched at
;   the next frame's first edge, and gives the frame's last word.
; - Send (5.9.5): TEXT with TFALL 0 ends each symbol when a rising edge of SCK reaches SYNC2, so SD out moves 3
;   cycles, pad to pad, after each rising edge (a transmitter synchronized with the leading edge, I2S 3.1 and
;   Table 1: delay 3 cycles, at most 0.8T; hold 3 cycles, at least 0).  The first symbol lands at the start
;   commit.  The frame's last word is marked last, so the send ends with the frame.
; - Every later frame: both halves are armed by sgo during the frame, and the wait for the next frame's first edge
;   matches as the frame's last bit is latched, while the halves still run; that edge ends the reception (RXCNT)
;   and the send (its last symbol).  The reception's last word moves to the buffer in the next cycle, and from
;   the cycle after both halves are idle, so the now after the wait, 1 to 4 cycles after the match, commits 2 to 5
;   cycles after it and starts both.  That must come before the next rising edge reaches SYNC2, and the first bit
;   it sends must be on the pad 0.2T before that edge at the controller: 8 cycles after the edge at worst, so SCK
;   periods of 10 cycles or more (the bench sweeps it).  The first frame's start has no such limit.
; - Feed (5.9.4): two words wait before each start (the frame's first word in the shift register, the next in the
;   buffer); each received word lets one more in, the frame's K-th marked last, so the send half is always a word
;   ahead, and the next frame's first word waits in the buffer through the restart.
.equ SCK  ui4
.equ WS   ui5
.equ SDI  ui6
.equ SDO  uo2
.equ NSCK 4             ; SCK and SD in as numbers, for the configuration words
.equ NSDI 6
.equ FMT  0
.equ WB   16
.equ MOUT 1
.equ MCFG 7
.equ TBL  256
.equ START 0x3010 | (FMT << 5)  ; untilm on ui: mask SCK and WS; SCK 1, and WS 0 (I2S) or FS 1 (TDM)
start:  sstop txrx              ; a unit isn't reset when its thread starts
        pcfg  SDO, PP
        put   SDO, 0            ; SD out idles low
        now
        recv  R1, MCFG          ; W
        recv  R3, MCFG          ; K
        li    R0, 0x0204 | (NSCK << 11)                  ; TMSB, TEXT, TFALL 0, TCLK SCK, idle 0
        scfg  R0, TXMODE
        ldi   R0, 0
        scfg  R0, TXMAP         ; one symbol a bit, output A takes the bit
        scfg  R0, TXPER
        scfg  R0, TXTAIL        ; no tail
        scfg  R0, CODING        ; no CRC, no stuffing
        li    R0, 0x0006 | (NSDI << 8) | (NSCK << 12)    ; RKIND 2, RMSB 1, rising edges, RPIN SD in, RCLK SCK
        scfg  R0, RXMODE
        scfg  R1, RXCNT         ; SYNCLEN 0: words start at once
        smap  SDO, A            ; SD out follows output A from the next commit
        li    R6, START
        li    R5, TBL
        cp    R4, R3            ; words left in this frame, for the last flag
        call  push              ; the first frame's first word
        call  push              ; and the next
        until WS, FMT           ; I2S: a left word (TDM: an FS pulse) has begun,
        until WS, 1 - FMT       ; then WS rises (FS falls): a whole word before the next frame starts
        sgo   txrx
        untilm R6, ui, eq       ; the first frame starts at this commit
        goto  body
next:   sgo   txrx              ; taken at the first commit after both halves are idle
        until WS, 1 - FMT       ; I2S: in the right word (TDM: FS low)
        untilm R6, ui, eq       ; this frame's last edge, the next frame's first
        now                     ; commits 2 to 5 cycles later: both halves start
        rxw   R1                ; the frame's last word
        send  R1, MOUT
        call  push
body:   cp    R2, R3            ; K - 1 words before the frame's last
wloop:  subi  R2, 1
        cmpi  R2, 0
        jt    next
        rxw   R1
        send  R1, MOUT
        call  push
        goto  wloop
push:   ldp   R0, R5            ; one word to send, the K-th of a frame marked last
        subi  R4, 1
        cmpi  R4, 0
        jt    plast
        txw   R0, WB
        ret
plast:  txw   R0, WB, last
        cp    R4, R3
        ret
