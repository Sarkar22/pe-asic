; 1-Wire at 50 MHz: reset, presence detect, write one byte. One open-drain pin.
; mailbox MIN: byte to write. MOUT: 1 if a device answered, 0 if not (byte then not sent).
.equ MIN 0
.equ MOUT 1
.equ OW io2
start:  pcfg  OW, OD
        now
xact:   recv  R0, MIN
        ldi   R4, 0
        put   OW, 0         ; reset pulse starts
        now
        li    R2, 24000     ; 480 us
        edge  OW, 1, R2     ; release
        tick  3500          ; 70 us later the device is pulling low
        getn  OW            ; T = 1 if presence seen
        bs    R4, 0
        li    R2, 20500     ; rest of the 480 us presence window
        tick  R2
        ldi   R1, 8
        bt    R4, 0
        jf    done
wbit:   shr8  R0, -, -      ; T = next bit, LSB first
        put   OW, 0
        tick  500           ; 10 us recovery, then the slot opens
        put   OW, T         ; a 1 releases here, a 0 stays low
        tick  300           ; 6 us into the slot
        put   OW, 1
        tick  2700          ; 60 us: end of slot
        djnz  R1, wbit
done:   send  R4, MOUT
        goto   xact
