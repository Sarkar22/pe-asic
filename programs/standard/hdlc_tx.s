; HDLC-style frame, none of it in hardware: NRZI (a 0 toggles), a 0 stuffed after five 1s,
; CRC-16/X-25, flags. MCFG: cycles per bit. MIN: bytes, then 0x100 to end the frame.
.equ TXD uo7
.equ MCFG 7
.equ MIN 0
.equ MOUT 1
start:  pcfg  TXD, PP
        put   TXD, 1
        now
        recv  R2, MCFG
        now                 ; the post can come any time later, so D moves up to here
        li    R4, 0x8408
frame:  li    R3, 0xFFFF    ; CRC state
        ldi   R5, 5         ; ones left before a stuff
        call  flag
next:   recv  R0, MIN
        bt    R0, 8
        jt    fin
        call  dbyte
        goto   next
fin:    not   R3            ; FCS, low byte first
        cp   R6, R3
        cp   R0, R6
        call  dbyte
        cp   R0, R6
        swap  R0
        call  dbyte
        call  flag
        send  R5, MOUT
        goto   frame
flag:   ldi   R0, 0x7E      ; sent raw: no stuffing, no CRC
        ldi   R1, 8
fl1:    shr8  R0, -, -
        put   TXD, ^!T      ; NRZI: toggle if the bit is 0
        tick  R2
        djnz  R1, fl1
        ret
dbyte:  ldi   R1, 8
db1:    shr8  R0, -, -      ; T = bit, LSB first
        crcr  R3, R4
        put   TXD, ^!T
        tick  R2
        jt    one
        ldi   R5, 5         ; a 0 restarts the run
        goto   db2
one:    djnz  R5, db2
        ldi   R5, 5         ; five 1s: insert a 0
        clrt
        put   TXD, ^!T
        tick  R2
db2:    djnz  R1, db1
        ret
