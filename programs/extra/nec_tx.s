; NEC IR transmit. MIN: address, then command.
.equ IR uo2
.equ MIN 0
.equ THI 439
.equ TLO 877
.equ UNIT 4000
start:  pcfg  IR, PP
        put   IR, 0
        now
top:    recv  R4, MIN
        now
        li    R5, 342
        call  mark
        ldi   R5, 56
        call  space
        call  word
        recv  R4, MIN
        call  word
        ldi   R5, 21
        call  mark
        goto   top
word:   cp   R6, R7
        cp   R0, R4
        not   R4
        andi  R4, 255
        swap  R4
        or    R0, R4
        ldi   R1, 16
bit:    ldi   R5, 21
        call  mark
        ldi   R5, 7
        shr16 R0, -, -
        jf    ws
        ldi   R5, 21
ws:     call  space
        djnz  R1, bit
        jr    R6
mark:   put   IR, 1
        tick  TLO
        put   IR, 0
        tick  THI
        djnz  R5, mark
        ret
space:  tick  UNIT
        djnz  R5, space
        ret
