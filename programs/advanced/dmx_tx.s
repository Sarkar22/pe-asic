; DMX512 transmitter, 250 kbaud 8N2. The universe streams through the mailbox, so it needs no
; frame buffer. MIN: start code first, then slots; bit 8 set on the last word of a frame.
.equ TX uo5
.equ MIN 0
start:  pcfg  TX, PP
        put   TX, 1
        now
        li    R2, 200       ; bit
        li    R3, 5500      ; break, 110 us
        li    R4, 800       ; mark after break, 16 us
        li    R5, 400       ; two stop bits
frame:  recv  R0, MIN       ; line stays marking until the host starts a frame
        put   TX, 0
        now
        put   TX, 1
        tick  R3
        tick  R4
slot:   put   TX, 0
        now                 ; re-anchored per slot: a slow host just lengthens the mark
        ldi   R1, 8
bit:    shr8  R0, -, TX
        tick  R2
        djnz  R1, bit
        put   TX, 1
        tick  R2
        tick  R5
        bt    R0, 8
        jt    frame
        recv  R0, MIN
        goto  slot
