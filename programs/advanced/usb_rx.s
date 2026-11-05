; USB low-speed receiver, two threads, all in firmware.
; Bit thread (0): watches D+ only. untilm chg with a timeout is "edge, or a bit passed": an edge is a
; 0 and re-centres D, a timeout is a 1. "No edge" is decided a few cycles past the due time, not at
; bit centre, so the next wait is armed before the next edge. Stuffed zeros are dropped.
; Byte thread (1): one wait for SE0 or a byte. At EOP it fills the byte word, so the bit thread's next
; trysend is refused, and that refusal is the only EOP test the bit thread pays for. SYNC is long
; enough for the bit thread to relock on what is left of it.
; MOUT: bytes, then 0x100 at EOP.
.equ DP io0
.equ DM io1
.equ MOUT 4
.equ MKILL 0         ; thread 0's word
.equ MBYTE 1         ; thread 1's inbox
.equ MACK 2
.equ AFT 41          ; first "no edge" verdict, 8 cycles past the due edge. 39 to 42 are clean: 43
                     ; loses packets at -1.5%, 38 at +1.5%, and 41 keeps the most jitter margin
.equ B1 33           ; later verdicts in a run of ones. Below the true 33.33, so a run of ones wins
                     ; back any lateness a byte boundary left, instead of compounding it
bit:    pcfg  DP, IN
        pcfg  DM, IN
        li    R3, 0x0100    ; change on D+
        ldi   R2, AFT
        ldi   R5, B1
        now
idle0:  until DP, 1         ; first K of SYNC. D = its edge
sy:     untilm R3, io, chg, R2
        jt    sy            ; an edge every bit, until the second K of KK
        ldi   R4, 5         ; that 1 counts toward stuffing
        ldi   R1, 8
        goto  bit1
bit0:   untilm R3, io, chg, R2
        jf    one
zero:   shr8  R0, -, -      ; the 1 meaning "edge" goes in, so bytes come out inverted
        ldi   R4, 6
        djnz  R1, bit0
        trysend R0, MBYTE
        jf    gone
        ldi   R1, 8
        goto  bit0
o1:     djnz  R1, bit1
        trysend R0, MBYTE
        jf    gone
        ldi   R1, 8
bit1:   untilm R3, io, chg, R5
        jt    zero
one:    shr8  R0, -, -
        djnz  R4, o1
        djnz  R1, st0       ; six ones: the next bit is a stuffed zero
        trysend R0, MBYTE
        jf    gone
        ldi   R1, 8
st0:    untilm R3, io, chg, R5
        jf    gone          ; a seventh 1: stuff error, or the quiet line after EOP
        ldi   R4, 6
        goto  bit0
gone:   recv  R0, MKILL     ; EOP has happened, or will: wait for the byte thread's word
        send  R0, MACK
        goto  idle0
; byte thread
byt:    li    R3, 0x0300    ; D+ and D- both low
        now
bw:     untilm R3, io, eq, inbox
        jt    se0
        recv  R0, MBYTE
        not   R0
        lui   R0, 0
        send  R0, MOUT
        goto  bw
se0:    tick  12            ; a skewed J-K transition looks like SE0 for a moment
        get   DP
        xget  DM            ; 1 on J or K
        jt    bw
        tryrecv R2, MBYTE   ; the last bit may be a timeout, so its byte can land after SE0 began
        jf    nolast
        not   R2
        lui   R2, 0
        send  R2, MOUT
nolast: ldi   R0, 0
        lui   R0, 1         ; 0x100
        send  R0, MOUT
        send  R0, MKILL
        send  R0, MBYTE     ; bit thread's next trysend is refused
        until DM, 1
        recv  R0, MACK      ; it is back at idle
        tryrecv R0, MBYTE
        goto  bw
