; Reaction test. uo4 is prepared first, then committed by the input match itself (pre-armed).
; uo5 is the same reaction done in firmware after the wait returns.
.equ TRIG ui5
.equ A uo4
.equ B uo5
start:  pcfg  A, PP
        pcfg  B, PP
        put   A, 0
        put   B, 0
        now
        until TRIG, 0
loop:   put   A, 1          ; prepared, nothing moves yet
        until TRIG, 1       ; the match commits it, 3 cycles pad to pad
        put   B, 1          ; firmware path: next instruction after the wait
        now
        put   A, 0
        until TRIG, 0
        put   B, 0
        now
        goto   loop
