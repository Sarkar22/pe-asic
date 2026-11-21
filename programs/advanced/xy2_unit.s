; XY2-100 controller on three shift units started at one absolute cycle: one unit makes CLK and SYNC, one sends
; the X positions and one the Y positions, each frame with its even parity bit appended by the unit's CRC
; register.  Three threads, entries clk, xax and yax, any three of the four.  128-word memory.
;
; Pins: CLK uo4 and SYNC uo5 (outputs A and B of the clk thread's unit), X uo6 and Y uo7 (output A of the xax and
; yax threads' units), all in PP.
; Host: each thread takes the start cycle T0 first, from its own mailbox word (MCLK, MX, MY), with the same value
; for all three, at least 2000 cycles ahead of TB.  clk then takes the number of frames N from MCLK; xax and yax
; take N 16-bit positions each from MX and MY, one a frame.
;
; The frame, as the XY2-100 interface defines it (and as sigrok's xy2-100 decoder reads it): 20 bits a frame at
; 2 Mbit/s, each bit starting at a rising edge of CLK and read at its falling edge; on X and Y the control bits
; 0 0 1, the 16-bit position MSB first, and a parity bit that makes the 20 bits even; SYNC high for the first 19
; bits and low for the parity bit.  Frames follow each other with no gap.
;
; How each step holds (section numbers are docs/ISA.md):
; - One grid (5.9.5, 7.3): every unit uses TPER 12 with TFRAC 128, two symbols a bit, so a bit is 25 cycles, 0.5 us,
;   with its symbols 12 and 13 cycles.  Each thread pushes its frame's first two words, arms its send, writes T0
;   into D with wrd and runs tick 0, which commits on exactly T0 (section 10, defined 3).  So all three sends
;   start on one cycle, their fraction accumulators start at 0 together, and every later symbol of the three lands
;   on one cycle too, for as long as the sends run (7.3, symbol edges of a send).
; - CLK and SYNC (5.9.3): output A's codes are 3 then 2, high then low, whatever the bit; output B takes the bit in
;   both symbols.  The clk unit sends 19 ones and a 0 a frame, MSB first, so SYNC is low for the parity bit only.
; - Parity (5.9.7, Parity): width 1, POLY x + 1, CINIT 0, CXOR 0.  The control bits are a 3-bit word of kind crc and
;   the position a 16-bit word of kind fcs, so the unit appends the XOR of those 19 bits, the even parity bit, and
;   starts its register again for the next frame.
; - Feed (5.9.4): each data unit takes two words a frame and the clk unit two, one at each word boundary; a frame is
;   500 cycles, so each thread has hundreds of cycles for each refill.  The last frame's last word is marked last.
.equ MCLK 4
.equ MX   5
.equ MY   6
.equ TPER 12
.equ TFRAC 128
clk:    sstop txrx              ; a unit isn't reset when its thread starts
        pcfg  uo4, PP
        pcfg  uo5, PP
        put   uo4, 0            ; CLK idles low
        put   uo5, 1            ; SYNC idles high
        now
        ldi   R0, 0x45
        scfg  R0, TXMODE        ; SYMS 1, TMSB 1, IDLEA 0, IDLEB 1
        ldi   R0, 0x0B
        scfg  R0, TXMAP         ; A: codes 3, 2 (CLK high, then low); B: code 0 in both symbols (SYNC = the bit)
        ldi   R0, 0
        scfg  R0, CODING        ; no CRC on this unit
        call  grid
        sstop tx                ; outputs show IDLEA and IDLEB
        smap  uo4, A
        smap  uo5, B
        now                     ; CLK and SYNC follow the unit from here
        recv  R1, MCLK          ; T0
        recv  R3, MCLK          ; N frames
        li    R4, 0xFFFF
        ldi   R5, 0x0E          ; 1 1 1 0, MSB first: the last 3 SYNC-high bits and the parity bit's low
        txw   R4, 16            ; the first frame's two words wait before the start
        cmpi  R3, 1
        jt    c1
        txw   R5, 4
        call  start
cloop:  subi  R3, 1
        txw   R4, 16
        cmpi  R3, 1
        jt    clast
        txw   R5, 4
        goto  cloop
clast:  txw   R5, 4, last
        goto  done
c1:     txw   R5, 4, last
        call  start
        goto  done
xax:    ldi   R6, MX            ; R6 names this thread's mailbox word, and so its axis
        goto  axis
yax:    ldi   R6, MY
axis:   sstop txrx
        ldi   R2, 0x05
        scfg  R2, TXMODE        ; SYMS 1, TMSB 1, output A takes the bit in both symbols, IDLEA 0
        ldi   R2, 0
        scfg  R2, TXMAP
        scfg  R2, POLYLO
        li    R2, 0x8000
        scfg  R2, POLYHI        ; x + 1: width 1, the running XOR
        li    R2, 0x0280
        scfg  R2, CODING        ; CW 0, CINIT 0, CXOR 0, CMODE 1, CTX 1: even parity appended
        call  grid
        sstop tx
        cmpi  R6, MX            ; R6 is the mailbox word this thread reads: MX for X, MY for Y
        jf    ypin
        pcfg  uo6, PP
        put   uo6, 0
        smap  uo6, A
        goto  pin2
ypin:   pcfg  uo7, PP
        put   uo7, 0
        smap  uo7, A
pin2:   now
        call  rpos              ; T0
        cp    R1, R0
        call  rpos              ; N
        cp    R3, R0
        ldi   R5, 1             ; the control bits 0 0 1
        txw   R5, 3, crc
        call  rpos
        cmpi  R3, 1
        jt    a1
        txw   R0, 16, fcs       ; the first frame's two words wait before the start
        call  start
aloop:  subi  R3, 1
        call  rpos
        txw   R5, 3, crc
        cmpi  R3, 1
        jt    alast
        txw   R0, 16, fcs
        goto  aloop
alast:  txw   R0, 16, fcs, last
        goto  done
a1:     txw   R0, 16, fcs, last
        call  start
        goto  done
rpos:   cmpi  R6, MX            ; R0 = the next word from this thread's mailbox word
        jf    rpy
        recv  R0, MX
        ret
rpy:    recv  R0, MY
        ret
grid:   ldi   R2, TPER
        scfg  R2, TXPER
        li    R2, TFRAC << 8
        scfg  R2, TXTAIL        ; no tail; TFRAC
        ret
start:  sgo   tx
        wrd   R1                ; D = T0
        tick  0                 ; commits on T0 exactly: the send starts there
        ret
done:   sstat R2
        bt    R2, 5
        jt    done
        halt
