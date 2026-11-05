; USB low-speed device link layer, endpoint 0, three threads, all in firmware.
; bit: the receiver. byt: CRC-16 check, PID and address decisions. wire: NRZI and stuffing out.
; byt starts wire from EOP, so turnaround is firmware time, and is measured on the bench.
; ADDR: this device. MDATA: host posts the bits of the packet at DPKT, once per IN it should answer.
; MOUT: bytes of a received data packet with CRC, then 0x100 good, 0x101 bad, 0x1D2 when our data is ACKed.
.equ DP io0
.equ DM io1
.equ ADDR 0x15
.equ MOUT 6
.equ MKILL 0
.equ MBYTE 1
.equ MDATA 4
.equ MFLAG 3          ; a mailbox word as a one-bit flag: a token for us is waiting for its data
.equ MTXA 2
.equ MDONE 5
.equ MACK 7
.equ MPOST 0
.equ AFT 41
.equ B1 33
.equ STEP 33
.equ DPKT 202
; ------------------------------------------------------------------- bit thread
bit:    li    R3, 0x0100
        ldi   R2, AFT
        ldi   R5, B1
        now
idle0:  until DP, 1
sy:     untilm R3, io, chg, R2
        jt    sy
        ldi   R4, 5
        ldi   R1, 8
        goto  bit1
bit0:   untilm R3, io, chg, R2
        jf    one
zero:   shr8  R0, -, -
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
        djnz  R1, st0
        trysend R0, MBYTE
        jf    gone
        ldi   R1, 8
st0:    untilm R3, io, chg, R5
        jf    gone
        ldi   R4, 6
        goto  bit0
gone:   recv  R1, MKILL     ; not R0: shr8 keeps the high byte, and bit 15 of a byte word means EOP
        send  R1, MACK
        goto  idle0
; ------------------------------------------------------------------- byte thread
byt:    li    R6, 0xA001
        li    R3, 0xB001    ; CRC-16 residue, kept in a register: every slot here is turnaround
new:    ldi   R5, 0         ; no PID yet
        ldi   R2, 0
        lui   R2, 1         ; 0x100: no address byte yet
bw:     recv  R0, MBYTE
        bt    R0, 15        ; the wire thread's EOP marker comes through the same word, so in order
        jt    eop
        call  onb
        goto  bw
eop:    cmpi  R5, 0xD2
        jt    gack
        tsti  R5, 2         ; DATA0, DATA1: tested first, this is the path with the tightest deadline
        jf    tok
        tryrecv R0, MFLAG   ; was the token ours?
        jf    drop
        cmp   R4, R3
        jf    badc
        li    R5, HACK + (16 << 8)
        send  R5, MTXA      ; answer first, report afterwards
        li    R0, 0x100
        send  R0, MOUT
        goto  r2
badc:   li    R0, 0x101
        send  R0, MOUT
        goto  drop
gack:   li    R0, 0x1D2
        send  R0, MOUT
        goto  drop
tok:    cmpi  R5, 0x69
        jt    tin
        cmpi  R5, 0xA5      ; SOF: not ours to answer
        jt    drop
        cmpi  R2, ADDR      ; SETUP or OUT
        jf    drop
        trysend R0, MFLAG
drop:   send  R0, MKILL
        trysend R0, MBYTE   ; the refused trysend is the bit thread's stop sign
        recv  R0, MACK
        tryrecv R0, MBYTE
        goto  new
tin:    cmpi  R2, ADDR
        jf    drop
        li    R5, HNAK + (16 << 8)
        tryrecv R0, MDATA   ; the host posts this packet's bits << 8 + its address
        jf    reply
        cp    R5, R0
reply:  send  R5, MTXA      ; low byte address, high byte bits: all eight mailbox words are in use
r2:     send  R0, MKILL
        trysend R0, MBYTE
        recv  R0, MDONE
        recv  R0, MACK      ; first stop has reached the bit thread
        send  R0, MKILL     ; it decoded our own packet: stop it again
        recv  R0, MACK
        tryrecv R0, MBYTE
        goto  new
; R0 = inverted byte. R5 = PID, R2 = address byte, R4 = CRC
onb:    not   R0
        lui   R0, 0
        cmpi  R5, 0
        jf    ob1
        cp    R5, R0
        li    R4, 0xFFFF
        ret
ob1:    bt    R2, 8
        jf    ob2
        cp    R2, R0
        andi  R2, 127
ob2:    tsti  R5, 2         ; DATA0 and DATA1 have bit 1 set. Tokens need no CRC-16: don't spend the time
        jf    ob3
        trysend R0, MOUT
        shr8  R0, -, -      ; unrolled: the last byte's CRC has to finish before we can answer
        crcr  R4, R6
        shr8  R0, -, -
        crcr  R4, R6
        shr8  R0, -, -
        crcr  R4, R6
        shr8  R0, -, -
        crcr  R4, R6
        shr8  R0, -, -
        crcr  R4, R6
        shr8  R0, -, -
        crcr  R4, R6
        shr8  R0, -, -
        crcr  R4, R6
        shr8  R0, -, -
        crcr  R4, R6
ob3:    ret
; ------------------------------------------------------------------- wire thread
wire:   pcfg  DP, IN
        pcfg  DM, IN
        put   DP, 0
        put   DM, 1
        now
        li    R3, 0x0300    ; SE0
wt:     untilm R3, io, eq, inbox
        jf    pkt
        tick  16            ; idle while receiving, so this thread watches for EOP: the byte thread
        get   DP            ; may be deep in a CRC for the whole 66 cycles of SE0
        xget  DM
        jt    wt
        ldi   R0, 255
        lui   R0, 255
        send  R0, MBYTE
        until DM, 1
        goto  wt
pkt:    recv  R5, MTXA
        cp    R6, R5
        andi  R6, 255
        swap  R5
        andi  R5, 255
        ldi   R4, 6
        ldi   R1, 16
        ldp   R0, R6
        oe    DP, 1
        oe    DM, 1
        now
        tick  8             ; J for a moment only: this bit of idle would be turnaround time
txbit:  shr16 R0, -, -
        put   DP, ^!T
        put   DM, ^!T
        tick  STEP
        jf    tz
        djnz  R4, tone
        ldi   R4, 6
        clrt
        put   DP, ^!T
        put   DM, ^!T
        tick  STEP
        goto  tone
tz:     ldi   R4, 6
tone:   djnz  R5, more
        put   DP, 0
        put   DM, 0
        tick  STEP
        put   DM, 1
        tick  STEP*2
        oe    DP, 0
        oe    DM, 0
        tick  STEP
        send  R0, MDONE
        goto  wt
more:   djnz  R1, txbit
        ldi   R1, 16
        ldp   R0, R6
        goto  txbit
.org 200
HACK:   .data 0xD280        ; SYNC, then PID ACK
HNAK:   .data 0x5A80        ; SYNC, then PID NAK
