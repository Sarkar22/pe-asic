; SWD host. Host words on MIN:
;   0x00nn raw: send nn bits, 1 to 16, from the next word, LSB first. Line reset, 0xE79E, idle.
;   0x8000 + [A3 A2 RnW APnDP]: one transfer. A write is followed by data low, data high.
; MOUT: status (0 OK, 1 WAIT limit, 2 FAULT, 3 no response, 4 read parity error), then for an OK
; read: data low, data high. WAIT is retried here, up to RETRY times. MCFG: half period.
.equ CK uo1
.equ IO io0
.equ MCFG 7
.equ MIN 0
.equ MOUT 1
.equ RETRY 8
start:  recv  R2, MCFG
        pcfg  CK, PP
        pcfg  IO, PP
        put   IO, 1
        now
cmd:    recv  R5, MIN
        bt    R5, 15
        jt    xfer
        cp    R1, R5        ; raw: R5 bits, 1 to 16
        recv  R0, MIN
        now
rw:     shr16 R0, -, IO
        edge  CK, 0, R2
        edge  CK, 1, R2
        djnz  R1, rw
        goto  cmd
; one transfer
xfer:   bt    R5, 1
        jt    x1
        recv  R3, MIN       ; write data, taken before the clock starts
        recv  R6, MIN
x1:     ldi   R4, RETRY
        bt    R5, 0         ; parity of APnDP, RnW, A2, A3
        bx    R5, 1
        bx    R5, 2
        bx    R5, 3
        andi  R5, 15
        add   R5, R5
        ori   R5, 0x81      ; start, park
        bs    R5, 5         ; R5 = request byte, LSB first on the wire
again:  now
        cp    R0, R5
        ldi   R1, 8
rq:     shr16 R0, -, IO
        edge  CK, 0, R2
        edge  CK, 1, R2
        djnz  R1, rq
        oe    IO, 0         ; turnaround: let go on this falling edge
        edge  CK, 0, R2
        edge  CK, 1, R2
        ldi   R0, 0
        ldi   R1, 3
ack:    edge  CK, 0, R2
        edge  CK, 1, R2
        shr8  R0, IO, -     ; S at the rise; first bit ends at bit 5
        djnz  R1, ack
        cmpi  R0, 0x20
        jt    okay
        cmpi  R0, 0x40
        jt    wait1
        cmpi  R0, 0x80
        ldi   R0, 3
        jf    back
        ldi   R0, 2
back:   call  trn
        goto  fin
wait1:  subi  R4, 1
        ldi   R0, 1
        cmpi  R4, 0
        jt    back
        call  trn
        goto  again
trn:    now
        edge  CK, 0, R2     ; one clock with the line free; then it is ours again
        edge  CK, 1, R2
        put   IO, 1
        oe    IO, 1
        ret
okay:   bt    R5, 2         ; RnW
        jt    read
        now                 ; decoding the ACK took longer than a half period
        edge  CK, 0, R2     ; write: target lets go during this clock
        edge  CK, 1, R2
        oe    IO, 1
        cp    R0, R3
        call  wr16
        cp    R0, R6
        call  wr16
        bt    R5, 8
        put   IO, T         ; data parity
        edge  CK, 0, R2
        edge  CK, 1, R2
        ldi   R0, 0
        goto  fin
read:   call  rd16
        cp    R6, R3
        call  rd16
        now
        edge  CK, 0, R2
        edge  CK, 1, R2
        get   IO
        bx    R5, 8         ; 0 when the parity matches
        ldi   R0, 0
        jf    r1
        ldi   R0, 4
r1:     call  trn
        send  R0, MOUT
        send  R6, MOUT
        send  R3, MOUT
        goto  idl
fin:    send  R0, MOUT
idl:    put   IO, 0
        now
        ldi   R1, 8
i1:     edge  CK, 0, R2
        edge  CK, 1, R2
        djnz  R1, i1
        put   IO, 1
        now
        goto  cmd
wr16:   ldi   R1, 16
        now                 ; SWCLK may stretch between the halves of a word
w1:     shr16 R0, -, IO
        bx    R5, 8
        bs    R5, 8
        edge  CK, 0, R2
        edge  CK, 1, R2
        djnz  R1, w1
        ret
rd16:   ldi   R1, 16
        now
r2:     edge  CK, 0, R2
        edge  CK, 1, R2
        shr16 R3, IO, -
        get   IO
        bx    R5, 8
        bs    R5, 8
        djnz  R1, r2
        ret
