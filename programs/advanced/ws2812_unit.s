; WS2812 chain driver on the shift unit: three symbols per bit, high, the data bit, low, so the line code is the
; unit's and the thread only feeds bytes.  One thread, 128-word memory.
;
; Pin: DOUT uo6 in PP, on output A.  Host: mailbox MIN takes a byte count (1 or more), then the bytes, first byte
; first, each sent MSB first: three bytes per LED in the chain's order, green, red, blue for a WS2812B.
;
; Timing, at 50 MHz, against the WS2812B data sheet: T0H 0.4 us and T1H 0.8 us, T0L 0.85 us and T1L 0.45 us, each
; +-150 ns, TH + TL 1.25 us +-600 ns, and a reset (latch) of more than 50 us low.
; - Line code (5.9.3, output codes; the "three thirds" row): SYMS 2 gives three symbols per bit, and MAPA 0x23 gives
;   output A the codes 3, 0, 2: a 1, the bit, a 0.  So a 0 is high for one symbol and low for two, a 1 high for
;   two and low for one.
; - Symbol time (5.9.5): TPER 20 with TFRAC 213 makes each symbol 20 + 213/256 cycles, 20 or 21 on the cycle grid,
;   so a bit is 62.5 cycles, 1.25 us, with no drift.  A 0 is high 20 or 21 cycles (400 to 420 ns) and low 41 or 42
;   (820 to 840 ns); a 1 is high 41 or 42 and low 20 or 21.  Any TPER from 18 to 23 cycles (360 to 460 ns a
;   symbol) keeps every window, with or without a fraction; the bench measures them.
; - MSB first (5.9.3, TMSB 1): an 8-bit word sends its bit 7 first.
; - Feed (5.9.4): one byte waits before the start; each later one goes in while a byte is sent, 10 us each.  The
;   last byte is marked last, so the send ends with the idle level, low, and the thread holds the line low for
;   TRES after the send ends (status bit 5), which latches the chain.
.equ DOUT uo6
.equ MIN  3
.equ TPER 20
.equ TFRAC 213
.equ TRES 15000         ; latch gap: 300 us, above the 50 us of a WS2812B and the 280 us of older parts
start:  sstop txrx              ; a unit isn't reset when its thread starts
        pcfg  DOUT, PP
        put   DOUT, 0           ; the line idles low
        now
        ldi   R0, 0x06
        scfg  R0, TXMODE        ; SYMS 2, TMSB 1, output A takes the code, IDLEA 0
        ldi   R0, 0x23
        scfg  R0, TXMAP         ; symbol 0 code 3 (1), symbol 1 code 0 (the bit), symbol 2 code 2 (0)
        ldi   R0, TPER
        scfg  R0, TXPER
        li    R0, TFRAC << 8
        scfg  R0, TXTAIL        ; no tail; TFRAC in bits 15:8
        ldi   R0, 0
        scfg  R0, CODING        ; no CRC, no stuffing
        sstop tx                ; output A shows IDLEA, low
        smap  DOUT, A
        now                     ; DOUT follows output A from here
        li    R2, TRES
frame:  recv  R3, MIN           ; byte count
        recv  R0, MIN
        cmpi  R3, 1
        jt    one
        txw   R0, 8
        sgo   tx
        now                     ; E: the first bit's high symbol is on the pad from E + 1
more:   subi  R3, 1
        recv  R0, MIN
        cmpi  R3, 1
        jt    fin
        txw   R0, 8             ; blocks until the buffer frees, once a byte
        goto  more
fin:    txw   R0, 8, last
        goto  done
one:    txw   R0, 8, last
        sgo   tx
        now
done:   sstat R1
        bt    R1, 5             ; the send half is busy until the last bit's low symbol ends
        jt    done
        now
        tick  R2                ; the next frame waits for the latch gap
        goto  frame
