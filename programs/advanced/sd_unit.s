; SD card host in SD mode, 1-bit bus, on two shift units: CMD with its CRC7 and the clock from one unit, the response
; by that unit's receive half, the data block on DAT0 by a second thread's unit with its CRC16.  2 threads (any two),
; 192 words, and the 512-word memory for the block buffer.  Thread A runs from `cmdth`, thread B from `datth`.
;
; Pins: CLK io0 (push-pull, shift output B of thread A), CMD io1 (push-pull with a committed turnaround, shift output
; A of thread A), DAT0 io2 (input).  The bus has pull-ups on CMD and DAT0 (SD Physical Layer Simplified Specification
; 6.5).  Both receive halves take CLK, read back from its own pad, as their outside clock (ISA 5.2: a uio input reads
; its own drive; section 10, defined 11).
;
; Host (sections 9.2, 9.4):
; - Mailbox MCFG: first H, the cycles per half clock of the power-up clocks; then 5 words per command: H for this
;   command, the command word (index in bits 5:0, response in 7:6 with 0 none, 1 a 48-bit token, 2 R2's 136 bits,
;   bit 8 set when a data block follows), argument bits 31:16, argument bits 15:0, and the time limit in words of
;   16 clocks after the command's end bit.
; - Mailbox MOUT, after each command: the response status (bits 15:8 the response words stored at RBUF, bit 4 a
;   response came, bit 5 the clock stopped early, bit 0 the check: CRC7 and end bit), then the block status from
;   thread B, 0 when none came (bit 8
;   set, bits 7:3 the last word's bit count, bits 2:0 the end flags, bit 0 the CRC16 and end bit check).
; - Memory: RBUF holds the response, most significant bit first (3 words, or 9 for R2); DBUF the block's 256 data
;   words, first byte in bits 15:8, then its CRC16 word; DCNT the words thread B received.  The host reads them with
;   RDMEM: two of the four threads run, so memory cycles are free.
;
; How each step holds (section numbers are docs/ISA.md):
; - Clock and command (5.9.5): two symbols per bit at TPER H.  Output A takes the bit in both symbols, output B is
;   0 then 1 (TXMAP 0x0E00, data with its clock), so CMD and CLK's falling edge move in one cycle and the card's
;   rising-edge capture has H cycles of setup and of hold (SD 6.6.6: 5 ns each).  TMSB sends a word's top bit first,
;   as CMD does (SD 3.6.1).  Symbol k is on the pads at E + 1 + kH after the start commit at E (5.9.5, 7.3).
; - CRC7 (5.9.7): CW 6, POLYHI 0x1200 (x^7 + x^3 + 1), CINIT 0, CXOR 0, as SD 4.5 defines it.  The first two
;   command words are of kind crc and the third, the argument's low byte, of kind fcs: the unit covers the 40 bits
;   from the start bit and appends the 7 CRC bits, top bit first.  Then a word of ones of no kind carries the end
;   bit and 15 clocks.
; - Turnaround (5.2, 5.4): OE is firmware's.  The start commit sets OE with the start bit; a tick at E + 96H, the
;   end bit's last cycle, clears it, so CMD is released on the pads as the end bit ends (bit 48 is the bus's
;   turnaround bit) and the card may answer from bit 49 on, NCR 2.  The same commit starts the response hunt.
; - CTX (5.9.7, 11.5): the append ends at E + 1 + 94H; CODING is moved to CTX 0 right after the turnaround, by
;   E + 96H + 5, before the earliest response start bit can reach SYNC2 at E + 98H + 3.
; - Response (5.9.6): RKIND 2 with RCLK the CLK pad samples CMD at each rising edge, as the card's output after the
;   falling edge allows (SD 6.6.6), on the cycle the edge reaches SYNC2, with CMD through the same synchroniser.
;   HUNT on a fall of CMD starts at the start bit; RCNT 48 (136 for R2) ends the reception after the end bit, so a
;   48-bit token is three whole words.  The CRC covers every bit from the start bit (0 bits into a zero register
;   change nothing), so a good token leaves the residue fed the end bit, CHK 0x12000000 = POLY.  For R2, CSKIP 8
;   leaves out the start, transmission and 111111 bits, so the CRC covers CID bits 127:1 and the end bit.
; - Clock loop: thread A keeps the send fed with words of ones, one per 16 clocks, and takes response words and
;   thread B's status meanwhile.  It never blocks: a pass reads the status and takes a response word if one waits
;   (RXFULL), then refills the send buffer only if TXFULL is clear, so the receive buffer is tested every 5 slots
;   when nothing is due.  A response word must be taken before the next one completes, 16 clocks later (5.9.6,
;   Words), and the send buffer refilled within a word time, 32H cycles (5.9.5, Refill).  An underrun (TXUR) means
;   the clock stopped: it is reported, not waited out.  The clock stops after one more word of 16 clocks, at least
;   the 8 the card needs before the next command (SD 4.12, NRC and NCC).
; - Data (thread B, 5.9.6, 5.9.7): RKIND 2 on DAT0, HUNT on a fall, SYNCLEN 1 with SYNC 0: the start bit is the
;   sync pattern, so words start with the first data bit, two bytes each.  CRC16-CCITT (CW 15, POLYHI 0x1021, CINIT
;   0, CXOR 0) covers the data and its CRC, and the end bit makes the residue POLY, CHK 0x10210000.  A block is 4113
;   bits after the start bit, over RCNT's 2047: RCNT starts at 0 (no limit), and after word 130, when RXTOT has passed
;   17 for the second time, the thread writes RCNT 17, which RXTOT next equals at bit 4113 = 2 x 2048 + 17 (section
;   10, defined 22).  Each word is stored within 16 clocks, the buffer's limit (11.5).
; - Not covered: the 4-bit bus, writes, and busy.  Thread B hunts on DAT0 from its start, so a card's busy signal on
;   DAT0 after an R1b command (SD 4.9.2) would start a reception: a host that sends such commands waits for DAT0 high
;   and restarts thread B.  A block that never ends leaves thread B in rxw; the host restarts it after a block's
;   time limit.
.equ CLK   io0
.equ CMD   io1
.equ DAT0  io2
.equ NCLK  8            ; io0, io1, io2 as input numbers, for the configuration words
.equ NCMD  9
.equ NDAT  10
.equ MOUT  1
.equ MBLK  5            ; thread B to thread A
.equ MCFG  7
.equ RBUF  240          ; response words, 9 at most
.equ DCNT  254
.equ DBUF  255          ; 257 words: 255 to 511
cmdth:  sstop txrx              ; a unit isn't reset when its thread starts
        pcfg  CLK, PP
        pcfg  CMD, PP
        put   CLK, 0            ; CLK idles low
        put   CMD, 1
        oe    CMD, 0            ; CMD released: the bus pull-up holds it high
        now
        li    R0, 0x0025        ; TXMODE: SYMS 1, TMSB, IDLEA 1 (CMD), IDLEB 0 (CLK)
        scfg  R0, TXMODE
        li    R0, 0x0E00        ; TXMAP: A the bit twice, B 0 then 1
        scfg  R0, TXMAP
        ldi   R0, 0
        scfg  R0, TXTAIL
        scfg  R0, POLYLO
        scfg  R0, CHKLO
        scfg  R0, RXSYNC
        scfg  R0, CODING
        li    R0, 0x1200        ; CRC7: x^7 + x^3 + 1, left-aligned
        scfg  R0, POLYHI
        scfg  R0, CHKHI         ; a good token's residue fed its end bit is POLY
        li    R0, 0x0026 | (NCMD << 8) | (NCLK << 12)       ; RKIND 2, RMSB, HUNT for a fall, rising edges of CLK
        scfg  R0, RXMODE
        smap  CLK, B
        smap  CMD, A
        recv  R1, MCFG          ; H for the power-up clocks
        scfg  R1, TXPER
        now                     ; the pins show the idle levels: CLK low, CMD released
        li    R6, 0xFFFF        ; 80 clocks with CMD high before the first command (SD 6.4.1: at least 74)
        txw   R6, 16
        txw   R6, 16
        sgo   tx
        now
        txw   R6, 16
        txw   R6, 16
        txw   R6, 16, last
next:   sstat R0                ; wait for the clock to stop: the send half is idle
        bt    R0, 5
        jt    next
        recv  R1, MCFG          ; H
        recv  R0, MCFG          ; index, response, block
        recv  R2, MCFG          ; argument 31:16
        recv  R3, MCFG          ; argument 15:0
        sstop rx
        scfg  R1, TXPER
        add   R1, R1            ; R7 = 96H: the end bit's last cycle after the start commit
        add   R1, R1
        add   R1, R1
        add   R1, R1
        add   R1, R1
        cp    R7, R1
        add   R1, R1
        add   R7, R1
        cp    R5, R0            ; first word: 0, 1, index, argument 31:24
        andi  R5, 0x3F
        ori   R5, 0x40
        swap  R5
        cp    R6, R2
        swap  R6
        andi  R6, 0xFF
        or    R5, R6
        txw   R5, 16, crc
        cp    R5, R2            ; second word: argument 23:8
        andi  R5, 0xFF
        swap  R5
        cp    R6, R3
        swap  R6
        andi  R6, 0xFF
        or    R5, R6
        txw   R5, 16, crc
        recv  R4, MCFG          ; the time limit, in words of clock after the end bit
        ldi   R6, 48            ; a 48-bit response: RCNT 48, CSKIP 0
        ldi   R1, 0
        bt    R0, 7
        jf    r48
        ldi   R6, 136           ; R2: 136 bits, CSKIP 8
        li    R1, 0x8000
r48:    scfg  R6, RXCNT
        scfg  R1, RXOPT
        li    R6, 0x0286        ; CODING: CRC7, CMODE 1, CTX 1: the send half appends it
        scfg  R6, CODING
        ldi   R1, 0             ; R1: bit 0 waits for a response, bit 1 for a block
        tsti  R0, 0xC0
        bs    R1, 0
        bt    R0, 8
        bs    R1, 1
        ldi   R0, 0             ; response status
        li    R5, RBUF
        oe    CMD, 1            ; CMD driven from the start bit on
        sgo   tx
        now                     ; E: the start bit is on the pads at E + 1
        txw   R3, 8, fcs        ; argument 7:0, then the CRC7
        cp    R3, R1
        li    R6, 0xFFFF
        txw   R6, 16            ; the end bit, then 15 clocks
        oe    CMD, 0
        bt    R3, 0
        jf    nor
        sgo   rx                ; the response hunt starts at the turnaround
nor:    tick  R7                ; E + 96H: CMD released as the end bit ends
        ldi   R1, 0x86          ; CODING: CTX 0, the receive half checks the response
        scfg  R1, CODING
        ldi   R7, 0             ; block status
cl:     sstat R1
        bt    R1, 9             ; a response word waits
        jt    resp
cl1:    bt    R1, 6             ; TXFULL: the next word of clock still waits
        jt    cl
        bt    R1, 7             ; TXUR: the clock stopped, the loop fell behind
        jt    under
        txw   R6, 16            ; one more word of clock, at once
        tryrecv R1, MBLK        ; thread B's block status
        jf    cl2
        cp    R7, R1
        andi  R3, 1
cl2:    cmpi  R3, 0
        jt    done
        djnz  R4, cl            ; the time limit, in words of clock
        goto  done
resp:   rxw   R2
        stp   R2, R5
        jf    cl1               ; R1 still holds the status read before
        srd   R0, end           ; the last word: end flag 0 is the CRC7 and end bit check
        ori   R0, 0x10
        andi  R3, 2
        goto  cl1
under:  ori   R0, 0x20          ; status bit 5: the clock stopped early
        sstop tx
        goto  fin
done:   txw   R6, 16, last      ; 16 more clocks, then the clock stops
fin:    sstop rx
        li    R1, RBUF
        sub   R5, R1
        swap  R5
        or    R0, R5
        send  R0, MOUT
        send  R7, MOUT
        goto  next
datth:  sstop txrx
        pcfg  DAT0, IN
        li    R0, 0x0026 | (NDAT << 8) | (NCLK << 12)       ; RKIND 2, RMSB, HUNT for a fall, rising edges of CLK
        scfg  R0, RXMODE
        li    R0, 0x0800        ; SYNCLEN 1, RCNT 0
        scfg  R0, RXCNT
        ldi   R0, 0
        scfg  R0, RXSYNC        ; the start bit is the sync pattern
        scfg  R0, RXOPT
        scfg  R0, POLYLO
        scfg  R0, CHKLO
        li    R0, 0x1021        ; CRC16-CCITT, left-aligned
        scfg  R0, POLYHI
        scfg  R0, CHKHI
        li    R0, 0x008F        ; CODING: CW 15, CMODE 1, CTX 0
        scfg  R0, CODING
        li    R6, DCNT
        sgo   rx
        now                     ; the hunt starts
blk:    li    R5, DBUF
        ldi   R4, 0
wl:     rxw   R1
        jt    blast
        stp   R1, R5
        addi  R4, 1
        cmpi  R4, 130
        jf    wl
        li    R0, 0x0811        ; RCNT 17 with SYNCLEN 1: the reception ends at bit 4113
        scfg  R0, RXCNT
        goto  wl
blast:  srd   R2, end
        st    R4, R6            ; the words received, at DCNT
        li    R0, 0x0800        ; no limit again for the next block
        scfg  R0, RXCNT
        sstat R1
        andi  R1, 31            ; the last word's bit count: 1, the end bit
        add   R1, R1
        add   R1, R1
        add   R1, R1
        or    R2, R1
        lui   R2, 1             ; bit 8: a block came
        send  R2, MBLK
        goto  blk
