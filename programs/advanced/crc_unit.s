; The idle send half as a CRC engine: the thread pushes a buffer from memory through its unit's send half at one
; bit a cycle, with no pin switched to the unit's outputs, and reads the CRC register with srd.  One thread,
; 128-word memory (the buffers in the free words above the code, or a 512-word memory).
;
; Host: mailbox MIN takes five words at start: TXMODE (0 for LSB first, 4 for MSB first), POLYHI, POLYLO, CODING
; (with CTX 1, and CMODE 1), and FLAGS: bit 0 reflects the 32-bit register, which puts a reflected CRC of any width
; in its low bits, and bit 1 complements the result.  Then two words per buffer: its address in memory, and its
; length in bytes, 1 or more, packed two to a word, the first byte in bits 7:0.  For each buffer MOUT gets the
; result's bits 31:16, then 15:0, then the status bits seen during the send (bit 7, TXUR, must be 0).
; CRC-32 (IEEE 802.3, zlib): TXMODE 0, POLYHI 0x04C1, POLYLO 0x1DB7, CODING 0x02BF, FLAGS 3, so "123456789" gives
; 0xCBF43926.  CRC-8/MAXIM: TXMODE 0, POLYHI 0x3100, POLYLO 0, CODING 0x0287, FLAGS 1, so it gives 0xA1, and a
; 1-Wire ROM code with its CRC byte leaves 0.
;
; How each step holds (section numbers are docs/ISA.md):
; - The engine (5.9.7, an idle send half as a CRC engine): words of kind crc go through the register in wire order,
;   one bit a cycle at TPER 0, and are sent on outputs that no pin shows.  The register starts from its initial
;   value at the send's start, and no word is of kind fcs, so nothing is appended and the register keeps the CRC
;   when the send ends; the thread reads it once status bit 5 is clear.  srd gives the remainder as it is, in wire
;   order, left-aligned, neither reflected nor complemented (11.5), so FLAGS finish a catalogue CRC.
; - Wire order (5.9.7): with TMSB 0 a 16-bit word sends its low byte first, each byte LSB first, which is the order
;   of a reflected-input CRC over the bytes in memory order.
; - Feed (5.9.4): the first word waits before the start; the loop takes ldp, txw and djnz, 4 slots, 16 cycles, one
;   word time at TPER 0, so the buffer is filled once a word.  Two words wait before the start, because the first
;   lasts only 16 cycles and the loop's first refill comes 24 cycles after the start's now.  The last word, 16 bits,
;   or 8 for an odd byte, is marked last, so the send ends without an underrun.
; - Reflection: shr16 and shl16 move one bit each through T (6.5), 16 times, so each half comes out reversed in the
;   other register.
.equ MIN  3
.equ MOUT 5
start:  sstop txrx              ; a unit isn't reset when its thread starts
        ldi   R0, 0
        scfg  R0, TXMAP
        scfg  R0, TXPER         ; TPER 0: one bit a cycle
        scfg  R0, TXTAIL
        recv  R0, MIN
        scfg  R0, TXMODE        ; SYMS 0, TMSB as given; no pin is switched to the outputs
        recv  R0, MIN
        scfg  R0, POLYHI
        recv  R0, MIN
        scfg  R0, POLYLO
        recv  R0, MIN
        scfg  R0, CODING
        recv  R6, MIN           ; FLAGS
cmd:    recv  R1, MIN           ; the buffer's address
        recv  R2, MIN           ; its length in bytes
        cp    R3, R2
        clrt
        shr16 R3, -, -          ; R3 = whole words; T = an odd byte at the end
        ldi   R4, 0
        bs    R4, 0             ; R4 = 1 when the last byte is alone in its word
        add   R3, R4            ; R3 = words to push, the odd byte's included
        ldp   R0, R1
        cmpi  R3, 1
        jt    only
        txw   R0, 16, crc       ; the first word moves into the shift register
        cmpi  R3, 2
        jt    two
        ldp   R0, R1
        txw   R0, 16, crc       ; the second waits in the buffer: two words before the start
        sgo   tx
        now
        subi  R3, 3             ; words between the second and the last
        cmpi  R3, 0
        jt    final
loop:   ldp   R0, R1
        txw   R0, 16, crc
        djnz  R3, loop          ; 4 slots: one word time at TPER 0
final:  ldp   R0, R1
        cmpi  R4, 0
        jt    f16
        txw   R0, 8, crc, last
        goto  wait
f16:    txw   R0, 16, crc, last
        goto  wait
two:    ldp   R0, R1
        cmpi  R4, 0
        jt    t16
        txw   R0, 8, crc, last
        goto  ostart
t16:    txw   R0, 16, crc, last
        goto  ostart
only:   cmpi  R4, 0
        jt    o16
        txw   R0, 8, crc, last
        goto  ostart
o16:    txw   R0, 16, crc, last
ostart: sgo   tx
        now
wait:   ldi   R5, 0
wait2:  sstat R4
        or    R5, R4            ; sstat clears TXUR once read: keep every bit seen
        bt    R4, 5             ; the send half is busy until the last bit
        jt    wait2
        srd   R2, hi
        srd   R3, lo
        bt    R6, 0
        jf    nrefl
        ldi   R4, 16
rev:    shr16 R2, -, -          ; T = the next bit of bits 31:16, from bit 16 up
        shl16 R1, -, -          ; R1 gets bits 31:16 reversed: the result's bits 15:0
        shr16 R3, -, -
        shl16 R0, -, -          ; R0 gets bits 15:0 reversed: the result's bits 31:16
        djnz  R4, rev
        goto  cmpl
nrefl:  cp    R0, R2
        cp    R1, R3
cmpl:   bt    R6, 1
        jf    out
        not   R0
        not   R1
out:    send  R0, MOUT
        send  R1, MOUT
        send  R5, MOUT
        goto  cmd
