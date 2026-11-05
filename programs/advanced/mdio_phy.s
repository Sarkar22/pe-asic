; MDIO PHY, Clause 22, following a station's MDC on ui5. 32 registers live in memory at REGS.
; MCFG: this PHY's address. Hunts for a 0 after ones, then takes the header bit by bit.
.equ MDC ui5
.equ MDIO io0
.equ MCFG 7
.equ REGS 96
start:  recv  R6, MCFG
        pcfg  MDIO, IN
        put   MDIO, 0
        now
hunt:   ldi   R1, 8         ; ones first, enough to tell idle from data
ones:   until MDC, 0
        until MDC, 1
        get   MDIO
        jf    hunt
        djnz  R1, ones
zero:   until MDC, 0
        until MDC, 1
        get   MDIO
        jt    zero          ; ST's 0 taken
        ldi   R0, 0
        ldi   R1, 8         ; ST1, op, phy
hb:     until MDC, 0
        until MDC, 1
        shl16 R0, MDIO, -
        djnz  R1, hb
        cp    R4, R0
        andi  R4, 31
        cmp   R4, R6
        jf    hunt          ; compared here, because after the reg bits there is no time left
        ldi   R1, 5
hr:     until MDC, 0
        until MDC, 1
        shl16 R0, MDIO, -
        djnz  R1, hr
        bt    R0, 11        ; op 10 is a read
        jf    wr
        until MDC, 0
        put   MDIO, 0
        oe    MDIO, 1
        until MDC, 1        ; first TA rise: starts driving the 0 for the second TA bit
        andi  R0, 31
        addi  R0, REGS
        ld    R3, R0
        ldi   R1, 16
rb:     until MDC, 0
        shl16 R3, -, MDIO
        until MDC, 1        ; prepared bit commits on this rise, 3 cycles pad to pad
        djnz  R1, rb
        until MDC, 0
        oe    MDIO, 0
        until MDC, 1        ; let go at the next rise
        goto  hunt
wr:     andi  R0, 31
        addi  R0, REGS
        ldi   R1, 2         ; skip TA
wta:    until MDC, 0
        until MDC, 1
        djnz  R1, wta
        ldi   R1, 16
wb:     until MDC, 0
        until MDC, 1
        shl16 R3, MDIO, -
        djnz  R1, wb
        st    R3, R0
        goto  hunt
