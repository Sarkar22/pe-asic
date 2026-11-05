; DMX512 receiver. Finds BREAK and MAB by their widths, checks the start code, counts slots and
; passes on only the window this fixture listens to.
; MCFG: first slot (1..512). MLEN: slots wanted. MOUT: bytes. A bad stop bit that isn't the start of
; a break sends 0x100.
.equ RX ui4
.equ MCFG 7
.equ MLEN 6
.equ MOUT 1
start:  recv  R6, MCFG      ; R6 = first slot
        recv  R5, MLEN
        add   R5, R6        ; R5 = one past the last slot
        li    R2, 200       ; bit
        li    R3, 100       ; half bit
rest:   until RX, 1
fall:   until RX, 0         ; D = the fall
        li    R0, 4000      ; 80 us: below the 88 us shortest break, above a 36 us zero byte
        until RX, 1, R0     ; high too soon: an ordinary character, or noise
        jt    fall
mab:    until RX, 1         ; low for 88 us at least: BREAK. D = its end
        li    R0, 350       ; 7 us: a shorter mark isn't a MAB
        until RX, 0, R0
        jt    fall
        until RX, 0         ; D = fall of the start bit of the start code
        call  byte
        jf    rest          ; start code with a low stop bit
        cmpi  R0, 0
        jf    rest          ; not a dimmer frame: ignore it, wait for the next break
        li    R4, 0         ; R4 = slot number
slot:   addi  R4, 1
sbit:   until RX, 0         ; next start bit: the mark between slots can last up to a second
        call  byte
        jf    brk
        lt    R4, R6        ; before the window?
        jt    slot
        lt    R4, R5
        jf    slot          ; past it
        send  R0, MOUT
        goto  slot
brk:    cmpi  R0, 0         ; low stop bit, all zeros: a BREAK that began as a character
        jf    bad
        li    R0, 2100      ; 9.5 bits in, 38 us; 42 us more makes 80
        until RX, 1, R0
        jf    mab
        goto  fall
bad:    ldi   R0, 0
        lui   R0, 1
        send  R0, MOUT
        goto  rest
; one character, D on the fall of its start bit. T=1 when the stop bit is high.
byte:   tick  R3
        ldi   R0, 0
        ldi   R1, 8
bit:    tick  R2
        shr8  R0, RX, -
        djnz  R1, bit
        tick  R2
        get   RX
        ret
