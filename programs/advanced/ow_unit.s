; 1-Wire master at standard speed on the shift unit: the send half makes every reset pulse and time slot on an
; open-drain io pin, the receive half samples each slot, and the unit's CRC register checks the CRC-8 of the
; bytes read, such as a ROM code after Read ROM.  One thread, 128-word memory.
;
; Pin: OW io4 in OD, with an outside pull-up.  Output A of the unit drives it: a 1 releases the line, a 0 pulls
; it low, and the pin can't drive high (section 5.2, open drain on a shift output; 5.3).
;
; Host: mailbox MIN takes one word per transaction: bits 7:0 a command byte, bits 15:8 the number n of bytes to
; read after it, 0 to 254.  The thread sends a reset pulse, then MOUT gets the presence result (1: a device
; pulled the line low).  If a device answered, the thread writes the command byte in 8 slots and reads n bytes in
; 8 n read slots; MOUT gets the bits the line carried in all of those slots, LSB first, as 16-bit words (the
; command byte in bits 7:0 of the first, then the bytes read; the last word holds what is left, 8 bits when n is
; even), and then 0x100 plus the end flags (section 5.9.7): bit 0 is set when the CRC-8/MAXIM residue of the n
; bytes read is 0, as it is for a ROM code followed by its CRC byte.
;
; Timing, at 50 MHz, from the DS18B20 data sheet's 1-Wire signalling section and Maxim's 1-Wire timing notes
; (standard speed): reset low 480 us or more, then high 480 us or more, presence 15 to 60 us after the release
; and 60 to 240 us long; a time slot 60 to 120 us with at least 1 us of recovery; a write 1 low 1 to 15 us, a
; write 0 low 60 to 120 us; a read slot low at least 1 us, its data valid for 15 us after the fall.
; - Reset (5.9.5): one 2-bit word at TPER TRST: low for 492 us, then released for 492 us.  The receive half,
;   started by the same commit, takes one timed sample (RCNT 1) TPDS cycles after the release.  A new
;   transaction waits until the send half is idle (status bit 5), so the last slot before it is whole.
; - Slots: TPER TSYM (6 us), 13 bits per slot word, LSB first, so a slot is 78 us on the cycle grid of 5.9.5.
;   W1 is low for 6 us and released for 72 (a write 1 and a read slot); W0 is low for 66 us with 12 us of
;   recovery (a write 0), so a line that takes up to 7 us to rise still rises before the next slot.
; - Samples (5.9.6, timed): the first sample is FIRST cycles after the start commit E, in SYNC2, so it sees the
;   pad at E + FIRST - 2, and the first slot falls at the pad at E + 1 (5.9.5, the grid).  FIRST = TSAMP + 3
;   samples each slot 13 us after its fall, inside the 15 us a device holds a read 0, and 7 us after this
;   master's own release, which leaves 7 us for the line to rise.  RPER is one slot, and the send and the
;   samples count from the same commit, so they never drift apart.
; - CRC (5.9.7): CODING 0x0087 (width 8, POLY 0x31, CINIT 0, CXOR 0, CTX 0), so the register serves the receive
;   half.  CSKIP 8 leaves out the 8 command slots, which read back the command itself.  RCNT is 8 + 8 n, and a
;   good ROM code leaves the residue 0 (CHK 0) at the end of the reception.
; - Feed (5.9.4, 5.9.5): one word waits before the start, and each later word goes in as the buffer frees, once
;   a slot.  The thread takes a received word when one waits, so none waits longer than one slot (11.5).
.equ OW   io4
.equ NOW  12            ; io4 as an input pin number, for RPIN
.equ MIN  3
.equ MOUT 5
.equ TRST 24600         ; reset low, and the high time after it: 492 us
.equ TPDS 3500          ; presence sample, 70 us after the release
.equ TSYM 300           ; one thirteenth of a slot, 6 us
.equ TSLOT 3900         ; one slot, 78 us
.equ TSAMP 650          ; sample 13 us after each slot's fall
.equ W1   0x1FFE        ; low 6 us, then released: a write 1 or a read slot
.equ W0   0x1800        ; low 66 us, then 12 us of recovery: a write 0
start:  sstop txrx              ; a unit isn't reset when its thread starts
        pcfg  OW, OD            ; released (5.3)
        ldi   R0, 0x20
        scfg  R0, TXMODE        ; one symbol per bit, LSB first, output A takes the bit, IDLEA 1: released
        ldi   R0, 0
        scfg  R0, TXMAP
        scfg  R0, TXTAIL        ; no tail, no fraction
        scfg  R0, RXEND
        scfg  R0, POLYLO
        scfg  R0, CHKHI         ; residue 0
        scfg  R0, CHKLO
        li    R0, 0x3100
        scfg  R0, POLYHI        ; x^8 + x^5 + x^4 + 1, left-aligned (5.9.7, the 1-Wire row)
        li    R0, 0x0087
        scfg  R0, CODING        ; CW 7, CINIT 0, CXOR 0, CMODE 1, CTX 0: the receive half's CRC
        li    R0, NOW << 8
        scfg  R0, RXMODE        ; timed samples, LSB first, no hunt, RPIN OW
        sstop tx                ; output A shows IDLEA, released
        smap  OW, A
        now                     ; OW follows output A from here
cmd:    recv  R6, MIN
wtx:    sstat R0
        bt    R0, 5             ; the last slot of the transaction before may still be on the line
        jt    wtx
        li    R0, TRST
        scfg  R0, TXPER
        li    R0, TRST + TPDS + 3
        scfg  R0, RXFIRST
        ldi   R0, 1
        scfg  R0, RXCNT         ; one sample, no sync pattern
        ldi   R0, 0
        scfg  R0, RXOPT
        ldi   R0, 2
        txw   R0, 2, last       ; bit 0 low for TRST, bit 1 released for TRST
        sgo   txrx
        now                     ; E: the reset pulse falls at E + 1; the sample is TPDS after its release
        rxw   R0
wrst:   sstat R1
        bt    R1, 5             ; the send half is busy until the reset's high time ends
        jt    wrst
        bn    R0, 0             ; T = 1: the line was low, so a device answered
        ldi   R1, 0
        bs    R1, 0
        send  R1, MOUT
        jf    cmd               ; no device: nothing more
        li    R0, TSYM
        scfg  R0, TXPER
        li    R0, TSAMP + 3
        scfg  R0, RXFIRST
        li    R0, TSLOT
        scfg  R0, RXPER
        cp    R5, R6
        swap  R5
        andi  R5, 0xFF          ; n
        add   R5, R5
        add   R5, R5
        add   R5, R5
        addi  R5, 8             ; RCNT = 8 + 8 n slots
        scfg  R5, RXCNT
        li    R0, 0x8000
        scfg  R0, RXOPT         ; CSKIP 8: the CRC leaves out the command's own slots
        li    R1, W1
        li    R2, W0
        cp    R3, R6            ; the command byte, LSB first
        ldi   R4, 8             ; command slots left
        call  pick
        txw   R0, 13
        sgo   txrx
        now                     ; E: the first slot falls at E + 1
fill:   subi  R5, 1             ; R5 = slot words still to push
        cmpi  R5, 1
        jt    final
        call  pick
        txw   R0, 13            ; blocks until the buffer frees, once a slot
        sstat R0
        bt    R0, 9             ; RXFULL: a received word waits
        jf    fill
        rxw   R0
        send  R0, MOUT
        goto  fill
final:  call  pick
        txw   R0, 13, last
drain:  rxw   R0                ; the rest; T = 1 on the reception's last word
        send  R0, MOUT
        jf    drain
        srd   R0, end           ; end flag 0: the CRC check passed
        sett
        bs    R0, 8             ; 0x100 marks the end word
        send  R0, MOUT
        goto  cmd
pick:   cp    R0, R1            ; R0 = the next slot word: W1, or W0 for a command bit of 0
        cmpi  R4, 0
        jt    pdone
        subi  R4, 1
        shr8  R3, -, -          ; T = the next command bit
        jt    pdone
        cp    R0, R2
pdone:  ret
