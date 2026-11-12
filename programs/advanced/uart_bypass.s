; UART pass-through with one byte replaced in flight, 8N1, 16 to 6553 cycles per bit, on any thread.
; ui4 reaches uo4 through the bypass lane: in BYP, uo4 shows ui4 with no flop between them (5.2, 5.8), so a frame
; leaves as it came. At the frame the host picks, the thread takes uo4 off the lane inside the start bit, sends
; the host's byte on the incoming frame's own bit grid, and puts uo4 back on the lane inside the stop bit.
;
; Following the line. The thread waits for each start bit's fall, and ticks 9 bit times after it, into that
; frame's stop bit, where it arms the next wait: the wait is on a level, so armed in a low data bit it would fire
; there. The unit's receive half samples the middle of every bit (5.9.6) and gives each frame's byte as one word,
; 3 cycles after the middle of its data bit 7, before that tick. The thread decides there, so every decision is
; made before the first bit it changes leaves the chip.
;   frame rule: replace frame N, counting from 0 at the first frame after MACK.
;   byte rule:  replace the frame after the first one whose byte is the given byte.
;
; The switches, for a chosen frame whose start bit falls at the pad in cycle s, at B cycles per bit. OUT already
; holds the level each switch needs, so the switch is the only thing its commit moves, and the equation of 5.2
; gives uo4 the same level on both sides of it.
; - Out, at s + 3. OUT is 0 from the commit that starts the hunt. pcfg PP prepares the lane's switch to open at
;   the next commit (5.3), and the wait for the start bit commits it at its match, s + 2, so uo4 shows OUT from
;   s + 3: a pre-armed reaction, 3 cycles pad to pad (5.7). ui4 is 0 from s to s + B - 1.
; - The bits. The match sets D = s + 2 (3.3), so data bit 0's tick commits it at s + B - 1, and it is on the pad
;   from s + B (5.4); bit k from s + (k + 1)B; the stop bit, 1, from s + 9B. These are the incoming frame's own
;   bit boundaries, to the cycle, so only the 8 data bits can differ from what came in.
; - Back, at s + 9B + 9. pcfg BYP prepares the lane again and a tick commits it. ui4 is 1 from s + 9B, in its
;   stop bit, until s + 10B at the earliest, and OUT is 1.
; At each switch ui4 and OUT have been equal for 3 cycles and stay equal for 3 more, so ui4's edges are 3 cycles
; or more away from it. What a pad does between clock edges as its switch changes is undefined (10, item 3): a
; cycle-level model can't show it, so the switches are kept where nothing else moves.
;
; MCFG: cycles per bit, once. Then, for each rule, with the line idle:
; MRULE: bits 7:0 the frame number, or with bit 8 set the byte to match. MREP: the byte to send instead.
; MACK: the rule, once the thread follows the line. Start the stream after taking it.
; MOUT: the byte the replaced frame brought in. Then the thread takes the next rule.
.equ RX    ui4
.equ TXP   uo4
.equ MCFG  7
.equ MRULE 0
.equ MREP  1
.equ MACK  4
.equ MOUT  5
start:  sstop txrx            ; a unit isn't reset when its thread starts
        pcfg  TXP, BYP
        now                   ; from the next cycle uo4 shows ui4
        recv  R2, MCFG        ; B, cycles per bit
        scfg  R2, RXPER       ; a sample every bit
        cp    R3, R2
        clrt
        shr16 R3, -, -
        add   R3, R2
        scfg  R3, RXFIRST     ; the first sample 1.5 bits after the fall: the middle of data bit 0
        li    R0, 0x0420
        scfg  R0, RXMODE      ; timed samples of ui4, LSB first, a hunt for each fall to 0
        ldi   R0, 8
        scfg  R0, RXCNT       ; 8 bits end a reception: one word per frame
        ldi   R0, 0
        scfg  R0, RXEND
        scfg  R0, RXOPT
        scfg  R0, CODING
        cp    R7, R2
        add   R7, R7
        add   R7, R7          ; 4B
        cp    R6, R7
        add   R6, R2          ; 5B
        cp    R3, R2
        subi  R3, 3           ; B - 3: from the start bit's match to data bit 0's commit
rule:   recv  R5, MRULE
        recv  R1, MREP
        put   TXP, 0          ; OUT 0, the start bit's level, committed while uo4 still shows the lane
        sgo   rx
        now                   ; the hunt starts at this commit
        ldi   R4, 7
        send  R5, MACK
        bt    R5, 8           ; T: a byte rule
        andi  R5, 0xFF
        jt    match
        cmpi  R5, 0
        jt    arm
count:  until RX, 0           ; a frame's start bit, armed while the line is high
        tick  R6
        tick  R7              ; 9B after the match: the frame's stop bit
        rxw   R0              ; its byte, before that tick
        djnz  R5, count       ; N frames pass, then the next one is taken
arm:    pcfg  TXP, PP         ; after the last commit before the chosen frame: the next one takes uo4 off the lane
take:   until RX, 0           ; the chosen frame's start bit: the match takes uo4 off the lane, from s + 3
        shr8  R1, -, TXP      ; data bit 0 of the host's byte
        tick  R3              ; on the pad from s + B
bits:   shr8  R1, -, TXP
        tick  R2
        djnz  R4, bits        ; data bits 1 to 7, a bit time apart
        put   TXP, 1
        tick  R2              ; the stop bit, from s + 9B
        pcfg  TXP, BYP
        tick  9               ; uo4 back on the lane from s + 9B + 9
        rxw   R0              ; the byte that came in
        send  R0, MOUT
        sstop rx
        goto  rule
match:  until RX, 0
        tick  R6
        tick  R7
        rxw   R0
        cmp   R0, R5
        jt    arm             ; this frame's byte matches: the next frame is taken
        goto  match
