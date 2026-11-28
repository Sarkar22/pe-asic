; Bring-up program for the first power-on. host/bringup.py drives it over the link; all four threads start at
; start with one RUN.
;   Every thread: R0 := its own number, R7 := D as RUN set it, which is the same on all four (ISA 9.3).
;   Mailbox: thread t takes word t and sends its complement back on word t, then does the same on word t + 4.
;   Thread 0 then takes words from the host on word 0. For each, it drives the low byte on io0..io7 and on
;   uo0..uo7 (uo3 is the link's MISO and stays), takes a snapshot at an event 8 cycles after the commit and
;   sends it to word 4: its high byte is what the uio pads read while the chip drives them (ISA 5.2). A word with
;   bit 15 set ends this: thread 0 releases every uio pad and sets every uo pin low.
;   Then every thread sets R3..R6 and four configuration slots from its number, sets T to bit 0 of its number,
;   and halts. spin is a loop for the host's own test of starting and stopping threads.
.equ DONE 15

start:  tid   R0
        rdd   R7
        cmpi  R0, 1
        jt    t1
        cmpi  R0, 2
        jt    t2
        cmpi  R0, 3
        jt    t3
        recv  R1, 0             ; thread 0
        not   R1
        send  R1, 0
        recv  R2, 4
        not   R2
        send  R2, 4
        goto  pins
t1:     recv  R1, 1
        not   R1
        send  R1, 1
        recv  R2, 5
        not   R2
        send  R2, 5
        goto  fin
t2:     recv  R1, 2
        not   R1
        send  R1, 2
        recv  R2, 6
        not   R2
        send  R2, 6
        goto  fin
t3:     recv  R1, 3
        not   R1
        send  R1, 3
        recv  R2, 7
        not   R2
        send  R2, 7
        goto  fin

pins:   pcfg  io0, PP           ; drive starts at the first commit
        pcfg  io1, PP
        pcfg  io2, PP
        pcfg  io3, PP
        pcfg  io4, PP
        pcfg  io5, PP
        pcfg  io6, PP
        pcfg  io7, PP
        pcfg  uo0, PP
        pcfg  uo1, PP
        pcfg  uo2, PP
        pcfg  uo4, PP
        pcfg  uo5, PP
        pcfg  uo6, PP
        pcfg  uo7, PP
next:   recv  R5, 0
        bt    R5, DONE
        jt    release
        put8  R5, io
        put8  R5, uo
        now                     ; commits both banks: on the pads one cycle later
        tick  8                 ; the snapshot at this event shows the pads 6 cycles after the commit
        rds   R5
        send  R5, 4
        goto  next
release: ldi  R5, 0
        put8  R5, uo
        oe8   R5
        now

fin:    li    R3, 0x5AF0
        add   R3, R0            ; R3 = 0x5AF0 + t
        cp    R4, R3
        not   R4                ; R4 = ~R3
        cp    R5, R3
        add   R5, R3            ; R5 = 2 * R3
        cp    R6, R3
        xor   R6, R5            ; R6 = R3 ^ R5
        scfg  R3, TXPER         ; slots of an idle unit: they change nothing but what RDREG reads
        scfg  R4, RXCNT
        scfg  R5, POLYHI
        scfg  R6, CHKLO
        bt    R0, 0
halted: halt
spin:   goto  spin
