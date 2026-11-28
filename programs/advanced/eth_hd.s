; Half-duplex 10BASE-T node at 50 MHz with CSMA/CD (IEEE 802.3 clause 4): answers ARP requests (RFC 826) for its own
; addresses, sends link test pulses (14.2.1.1), defers to the line, and on a collision jams, backs off and sends the
; frame again, up to 16 attempts. Its MAU's link integrity test (14.2.1.7) and jabber function (14.2.1.6) are here
; too. Two threads, 512-word memory. It is eth_arp.s without the ICMP echo and with buffers of 32 words: CSMA/CD takes
; about 120 words, which eth_arp.s doesn't have. The shift units do the line work as there (section 5.9); this file
; says what differs. R and T may be any two threads; the host starts R at label rx and T at tx.
;
; Board and mailbox: as eth_arp.s. DI = uo4, DE = uo5, RO = ui4; MCFG (M4) takes the MAC as three words, then the IP as
;   two, each word two octets in wire order. Counters the host can read with RDMEM while threads 2 and 3 stay stopped
;   (section 9.4): NKEEP (word 7), NBAD (8), NOTHER (9) as eth_arp.s, NBUSY (10) frames taken unstored because both
;   buffers were in use or the link was down, and for the sending side NCOLL (17) collisions, NLATE (18) the late ones
;   among them, counted twice as 5.2.2.1.10 asks, NXS (19) frames dropped after 16 attempts (attemptLimit, 4.4.2) or
;   cut by the jabber function. LST (13) is the link state, 4 in LINK TEST PASS: the link indicator 14.2.1.7
;   recommends.
;
; R: as eth_arp.s, with BUFW 32. A collision fragment from the partner (a preamble, the SFD and a jam) is a runt: NBAD.
;   At a frame's first word R reads the link state LST with RELEASED, and takes the frame unstored if the link is down
;   (14.2.1.7: rcv=disable): the state there is the state at the frame's start, because T stores LST only after
;   EXTEND, which waits for the end of the frame that brought the link back. The buffer choice before a frame's first
;   store takes 19 slots with that store, against 20 before the second word. R doesn't test a frame's last word for
;   whole octets: REOP's byte end leaves 0 or 8 bits in it (5.9.7), and R's configuration ends a reception no other
;   way. Every reply is 60 octets, so T doesn't keep a frame's length (R5 only says a reply waits), and the buffer's
;   first word is unused.
; T: link pulses, then an ARP reply for each request R hands over.
;   Deference (4.2.3.2.1, 4.2.8 Deference): 64 BT (320 cycles) of RO high, which any low restarts, then 32 BT that
;   nothing restarts, then the frame. 4.2.3.2.1 lets the timer restart only in the first two thirds of the gap.
;   Collision detection: T's own unit receives RO while it sends (5.9.1: the halves share no flop; the CRC register
;   stays with the send half, CTX 1). The receive half uses R's decoder settings with SYNCLEN 3, the pattern 1, 0, 1 and
;   RCNT 1, so it delivers a word, status bit 9, three decoded bits into a preamble, about 5 BT after the partner's
;   first level on the pair (14.2.1.4 allows the MAU 9 BT). A link test pulse decodes as at most 1, 0, so it never
;   counts (14.2.1.7). `sstop rx` before each attempt clears the word; status bit 11 would not do, since only the next
;   reception clears RXSYN. T polls `sstat` at least every 8 slots while it feeds the frame's words, the last one
;   included, and every 5 while it waits for the end; it queues a word only while bit 6 reads 0, so `txw` never
;   blocks.
;   The jam (4.2.3.2.4): the bits sent after T sees the collision. T puts 16 bits behind whatever is in flight at once,
;   writes CODING with CMODE 0 and CTX and CXOR as they were (0x027F, which R5 holds while a reply waits), so a word of
;   kind fcs whose append hasn't begun appends nothing, and an append under way goes on unchanged: a change of CTX or
;   CXOR would take effect from the next cycle (section 10 item 17), which may fall between the two symbols of an
;   appended bit and split it, a Manchester code violation. Then T adds octets of no kind until the send ends 32 BT or
;   more after that moment, by underrun, with TP_IDL; after a whole FCS they leave the octets after the SFD ending in
;   an FCS that doesn't check, but for a 2^-32 chance. A collision in the preamble gives the whole header and then 32
;   jam bits, as PhysicalSignalEncap does (4.2.8). T counts cycles: the start commit is cycle E (D), a bit is
;   5 cycles, a 16-bit word 80, an append 160; R6 holds where the queued bits end, less 6, and R7 the header's end.
;   No word is marked last, so a send ends by underrun even when nothing collides.
;   Backoff (4.2.3.2.5, 4.2.8 BackOff): before attempt n + 1, r x 512 BT, r drawn from 0 to 2^min(n, 10) - 1 by a
;   16-bit LFSR (x^16 + x^5 + x^3 + x^2 + 1) that each draw stirs with TB and the MAC's last two octets, so two
;   stations draw apart (the unit's LFSR starts from all ones, the same on every chip). T waits the slot times one
;   at a time in tloop, so link pulses go on; then it defers again. A late collision, after 512 BT, is jammed and
;   retried the same way (10 Mb/s may retry, 4.2.8).
;   Link integrity (14.2.1.7, Figure 14-6): between sends the receive half keeps hunting, and T polls it in tloop. A
;   link test pulse starts a reception (status bit 12) that gives no word; RD_input gives a word (bit 9), which T
;   drains so that the half hunts on. LST holds the state: 4 in LINK TEST PASS, 2 and 3 in LINK TEST FAIL with a
;   count of 0 and 1, 6 in LINK TEST FAIL EXTEND. One count of TB wraps, LW, is both timers, restarted at every
;   change, so link_loss and link_test_max are each 67 wraps, 86.5 to 87.8 ms (50 to 150 and 25 to 150 ms), longer
;   than the longest backoff, 52.4 ms; link_test_min is 4 wraps, 3.9 to 5.2 ms (2 to 7 ms), so a pulse with LW 64 or
;   more is early. lc_max is 2, and the node powers on in LINK TEST PASS. In LINK TEST PASS an early pulse is
;   ignored; in LINK TEST FAIL it sets the count to 0, and LW running out sets it to 0 too. EXTEND has no timer:
;   RD activity there starts LW again, so a frame that arrives as link_test_max runs out can't fall back into LINK
;   TEST FAIL (Figure 14-6 leaves EXTEND only for LINK TEST PASS). Once a loop, EXTEND waits
;   up to 12 cycles for RO to stay high, which no part of a frame does (a level lasts at most a bit time, with 802.3's
;   jitter under 7 cycles) and which TP_IDL ends: RD idle (14.3.1.3.3). A loop in EXTEND takes about 30 slots, so LINK
;   TEST PASS comes 27 to 34 BT after the frame's last rise, before a frame 47 BT after it (4.4.2 note 1) begins. DO
;   is idle then, because while the link is down T sends no frame (xmit=disable: a reply waits, and is sent once the
;   link is back), takes none and lets the backoff wait; link test pulses go on in every state (TD_idle is TP_IDL,
;   14.2.2.2). A partner's frame can also look like a pulse to a poll that comes before its word, which is why the
;   pulse count leaves LINK TEST FAIL through EXTEND too. A limit: the receive half finds RD_input only where it
;   decodes 1, 0, 1, which every frame's preamble holds, so a burst with no such run (no preamble) counts as a link
;   test pulse, where 14.3.1.3.2 makes it RD activity, and a transmission with none after its preamble that outlasts
;   link_loss reads as silence once its preamble is past. Collisions are seen only while T sends, so only in LINK
;   TEST PASS. Between T's samples of TB there are under 7,000 cycles (an attempt after a backoff slot, with a late
;   collision), so it misses no wrap; the timers stand still only while a send runs on to the guard.
;   Jabber (14.2.1.6, Figure 14-5): the loops that feed a send count its words in R3 from 0, so a send fed past 65,536
;   words, 52.4 ms of jam octets to 104.9 ms of 16-bit words (xmit_max, 20 to 150 ms), stops being fed and ends by
;   underrun with its TP_IDL. The frame is dropped and counted in NXS, as a MAC drops one whose every attempt collides
;   while the MAU holds CS0 (14.2.1.6 b: 16 backoffs end within 366 ms, inside the wait below), and BOFF holds T for
;   9,766 slot times, 500.0 ms (unjab, 0.25 to 0.75 s): it sends no frame and takes none, while link test pulses and
;   the link integrity test go on. The wait counts down only in LINK TEST PASS, as a backoff does, so a link that fails
;   during it makes it last longer than 0.75 s; nothing is sent while the link is down anyway. A reply is 576 bits
;   and a jam ends within 58 BT of its collision, so only a broken loop reaches the guard.
.equ DI uo4
.equ DE uo5
.equ RO ui4
.equ RON 4                  ; RO as a pin number, for RXMODE's RPIN (5.9.3)
.equ MCFG 4
.equ BUFW 32                ; full words a buffer keeps: 64 octets, FCS included
.equ RXOPTV 0x0E00          ; RACQ for the preamble, RGAIN 3, RFRAC 0, CSKIP 0
.equ LINKW 67               ; TB wraps of link_loss and link_test_max (14.2.1.7)
.equ QUIET 12               ; cycles of RO high that end EXTEND
.equ UNJAB 9766             ; slot times of the jabber's unjab wait
; data, at addresses an ldi reaches
.equ CFG 0                  ; MAC (3 words), then IP (2 words), from the host
.equ CIP 3
.equ NATT 5                 ; attempts on the frame T is sending
.equ BOFF 6                 ; slot times of backoff, or of the jabber's unjab wait, left
.equ NKEEP 7
.equ NBAD 8
.equ NOTHER 9
.equ NBUSY 10
.equ HANDED 11              ; frames R has handed to T; written by R only
.equ RELEASED 12            ; frames T has finished with; written by T only
.equ LST 13                 ; the link state, written by T only
.equ LW 14                  ; TB wraps left on the link timer
.equ TBP 15                 ; T's last sample of TB, to see it wrap
.equ SEED 16                ; the generator's state
.equ NCOLL 17
.equ NLATE 18
.equ NXS 19
.equ ARPT 20                ; EtherType, hrd, pro, hln and pln, op of an ARP request, as received words
.equ BUF0 25                ; each buffer: a word, then the frame from word 0
.equ BUF1 BUF0 + BUFW + 1
        .org  NATT
        .data 0
        .data 0
        .data 0
        .data 0
        .data 0
        .data 0
        .data 0
        .data 0
        .data 4
        .data LINKW
        .data 0
        .data 0
        .data 0
        .data 0
        .data 0
        .data 0x0608
        .data 0x0100
        .data 0x0008
        .data 0x0406
        .data 0x0100
        .org  BUF1 + BUFW + 1
; ------------------------------------------------------------------------------------------------- R: receive
rx:     sstop txrx
        call  rxset          ; Manchester decoder on half cycles, RINV, hunt for RO falling, 5 cycles a bit
        lui   R0, 0x80       ; 0x8000 (rxset leaves R0's low byte 0)
        scfg  R0, RXCNT      ; a 16-bit sync pattern, no bit limit, no timeout (RXEND 0 from reset)
        li    R0, 0xD555
        scfg  R0, RXSYNC     ; the last preamble octet, then the SFD
        call  poly
        li    R0, 0xC704
        scfg  R0, CHKHI      ; the residue a good frame leaves
        li    R0, 0xDD7B
        scfg  R0, CHKLO
        li    R0, 0x80FF
        scfg  R0, CODING     ; CRC-32 on the receive half: all ones to start, checked at the end; REOP: the byte end
        ldi   R1, CFG
        ldi   R2, 5
rcfg:   recv  R0, MCFG       ; MAC and IP from the host
        stp   R0, R1
        djnz  R2, rcfg
        sgo   rx
        now                  ; the hunt starts at this commit
        ldi   R5, 0          ; frames handed to T
rframe: rxw   R0             ; a frame's first word
        jt    rbad           ; it ended within 16 bits of its SFD
        ldi   R1, RELEASED
        ldp   R3, R1
        ld    R6, R1         ; LST: 2 or 3 while the link is down
        cp    R4, R5
        sub   R4, R3         ; frames T still holds: 0, 1 or 2
        or    R4, R6
        bt    R4, 1
        jt    rbusy          ; both buffers in use, or the link down
        ldi   R6, BUF0 + 1
        bt    R5, 0
        jf    r1
        ldi   R6, BUF1 + 1
r1:     cp    R1, R6
        ldi   R2, BUFW
r2:     stp   R0, R1
        rxw   R0
        jt    rend
        djnz  R2, r2
r3:     rxw   R0             ; the buffer is full: take the rest unstored
        jf    r3
        ldi   R2, 1
rend:   sstat R3             ; T: a word was lost
        jt    rbad
        srd   R4, end
        bt    R4, 0          ; end flag 0: the FCS is right
        jf    rbad
        cmpi  R2, 1          ; 32 words stored: 64 octets with the FCS (R2 is BUFW + 1 less the words stored)
        jf    rbad
        cp    R1, R6
        ldp   R0, R1
        ldp   R3, R1
        and   R0, R3
        ldp   R3, R1
        and   R0, R3
        not   R0
        cmpi  R0, 0
        jt    rkeep          ; DA is the broadcast address
        cp    R1, R6
        ldi   R3, CFG
        ldi   R4, 3
rda:    ldp   R0, R1
        ldp   R7, R3
        cmp   R0, R7
        jf    rother
        djnz  R4, rda
rkeep:  addi  R5, 1
        ldi   R1, HANDED
        st    R5, R1         ; T takes the frame from here on
        ldi   R1, NKEEP
        goto  rcnt
rbad:   ldi   R1, NBAD
        goto  rcnt
rother: ldi   R1, NOTHER
        goto  rcnt
rbusy:  rxw   R0             ; take the frame unstored
        jf    rbusy
        ldi   R1, NBUSY
rcnt:   call  inc
        goto  rframe
; ------------------------------------------------------------------------------------------------- T: send
tx:     sstop txrx
        li    R0, 0x81A1
        scfg  R0, TXMODE     ; 2 symbols per bit, LSB first, THALF; idle A 1, B 0; tail A 1, B 1
        li    R0, 0x0F01
        scfg  R0, TXMAP      ; A: 1 - b, then b (802.3: a 1 is low then high). B: 1 throughout
        ldi   R0, 2
        scfg  R0, TXPER
        call  poly
        call  rxset          ; the receive half watches RO, with R's decoder
        li    R0, 0x1801
        scfg  R0, RXCNT      ; SYNCLEN 3, RCNT 1: a word one bit after the pattern
        lui   R0, 0xA0
        scfg  R0, RXSYNC     ; 1, 0, 1, newest in bit 15 (only the top 3 bits count)
        pcfg  DI, PP
        pcfg  DE, PP
        put   DI, 1
        put   DE, 0
        sgo   rx             ; the receive half hunts from this commit on
        now
        sstop tx             ; outputs at their idle levels: A 1, B 0
        smap  DI, A
        smap  DE, B
        now
        ldi   R5, 0          ; octets of a reply waiting to go, 0 for none
        call  pulse          ; the first link pulse at once
tloop:  rdtb  R2
        ldi   R1, LST
        ldp   R4, R1         ; R4: LST
        ldp   R6, R1         ; R6: LW
        ld    R0, R1
        st    R2, R1
        lt    R2, R0         ; TB went down: it wrapped
        jf    t0
        djnz  R6, tw
        ldi   R4, 2          ; link_loss or link_test_max done: LINK TEST FAIL RESET, then LINK TEST FAIL, count 0
        ldi   R6, LINKW
tw:     djnz  R3, t0         ; R3: wraps left before the next pulse
        call  pulse
t0:     sstat R0
        bt    R0, 9
        jt    tfr            ; RD_input
        bt    R0, 12
        jf    t0e            ; nothing on RD
        bt    R6, 6
        jf    tpa            ; link_test_min done
        bt    R4, 2
        jt    t0e            ; LINK TEST PASS ignores an early pulse, and EXTEND every pulse
        ldi   R4, 1          ; LINK TEST FAIL RESET: the count is 0 after the COUNT below
tpa:    bt    R4, 2
        jt    t0r            ; LINK TEST PASS (or EXTEND): both timers start again
        addi  R4, 1          ; LINK TEST FAIL COUNT
        cmpi  R4, 4
        jt    tx6            ; link_count = lc_max: EXTEND
        goto  t0r
tfr:    rxw   R2             ; drained, so the half hunts on
        cmpi  R4, 4
        jt    t0r
tx6:    ldi   R4, 6          ; LINK TEST FAIL EXTEND
t0e:    cmpi  R4, 6
        jf    t0x
        ldi   R2, QUIET
        now
        until RO, 0, R2      ; RD and DO idle (T sends nothing in EXTEND): LINK TEST PASS
        jt    t0r            ; RD active: EXTEND goes on, and LW starts again, so it never runs out in EXTEND
        ldi   R4, 4
t0r:    ldi   R6, LINKW      ; both timers start again
t0x:    subi  R1, 2
        stp   R4, R1
        st    R6, R1
        cmpi  R4, 4
        jf    tloop          ; the link is down: nothing is sent (xmit=disable), no frame taken, the backoff waits
        ldi   R1, BOFF
        ld    R0, R1
        cmpi  R0, 0
        jt    t1
        subi  R0, 1
        st    R0, R1
        tick  2560           ; one slotTime of backoff (512 BT, 4.4.2), or of the unjab wait; tloop goes on meanwhile
        goto  tloop
t1:     cmpi  R5, 0
        jt    t2
        li    R2, 320
        now
        until RO, 0, R2      ; T 1: the line was active in the first 64 BT of the gap
        jt    tloop
        ldi   R3, 0          ; the jabber's word count
        li    R0, 0x02FF
        scfg  R0, CODING     ; CRC-32 on the send half, appended complemented (a collision turns it off)
        li    R0, 0x800F
        scfg  R0, TXTAIL     ; tail 15 cycles: TP_IDL; TFRAC 128
        call  tbuf
        addi  R1, 1          ; word 0 of the reply
        tick  160            ; the last 32 BT of the gap, which the line can't restart (4.2.3.2.1)
        li    R0, 0x5555
        txw   R0, 16
        txw   R0, 16
        sstop rx             ; the collision detector afresh
        sgo   txrx
        now                  ; the attempt starts at this commit, cycle E
        txw   R0, 16
        lui   R0, 0xD5
        txw   R0, 16         ; the 7th preamble octet, then the SFD
        rdd   R7
        addi  R7, 160
        addi  R7, 161        ; R7 = E + 321: the header's 64 bits end here
        cp    R6, R7
        subi  R6, 6          ; R6: where the queued bits end, less 6
        ldi   R2, 30         ; the reply's 30 words, the last one included (every reply is 60 octets)
        ldp   R0, R1
sp:     sstat R4
        bt    R4, 9
        jt    coll           ; the receive half has seen a preamble: a collision
        bt    R4, 6
        jt    sp             ; until the buffer is empty
        djnz  R2, s3
        txw   R0, 16, fcs
        addi  R6, 240        ; 16 bits and the FCS
s5:     sstat R4
        bt    R4, 9
        jt    coll
        bt    R4, 5
        jt    s5             ; until the tail ends
        goto  sok
s3:     txw   R0, 16, crc
        addi  R6, 80
        sstat R4
        bt    R4, 9
        jt    coll
        ldp   R0, R1
        djnz  R3, sp         ; the jabber's guard
        goto  jab
; a collision, seen at cycle c
coll:   rdtb  R1             ; c
        txw   R0, 16         ; 16 bits behind whatever is in flight, at once, so the send never runs dry
        scfg  R5, CODING     ; CMODE 0, CTX and CXOR kept (R5 is 0x027F): a word of kind fcs appends nothing
        cp    R4, R6
        sub   R4, R1         ; R6 - c
        bt    R4, 15
        jt    sover          ; the frame had ended: no collision (WatchForCollision only while transmitting)
        addi  R6, 80
        cmpi  R2, 0
        cp    R2, R1         ; c, for the late test
        jf    c2             ; a data word was next
        ldi   R0, 164
        lt    R4, R0
        jt    c1             ; the append had begun: it goes out whole, no bit of it split
        subi  R6, 160        ; the append won't happen
c1:     cp    R4, R6
        sub   R4, R1
        ldi   R0, 154
        lt    R4, R0
        jf    jw             ; 32 BT or more follow c
        txw   R0, 16         ; else 16 bits more
        goto  jw
c2:     cp    R4, R7
        sub   R4, R1
        bt    R4, 15
        jt    c3             ; c after the header
        cp    R1, R7         ; seen in the header: the jam follows the SFD
c3:     addi  R1, 154        ; R6 must reach this: 32 BT after c, less 6
jl:     cp    R4, R6
        sub   R4, R1
        bt    R4, 15
        jf    jw
        txw   R0, 8          ; a jam octet
        addi  R6, 40
        djnz  R3, jl         ; the jabber's guard
; the guard ran out: the send is no longer fed, so it underruns with its TP_IDL. The frame is dropped
jab:    li    R0, UNJAB
        ldi   R1, BOFF
        st    R0, R1         ; the unjab wait, in slot times
        ldi   R1, NXS
        call  inc
        now                  ; the wait counts from here
sover:  sstat R4
        bt    R4, 5
        jt    sover          ; until the tail ends
        sstop tx             ; the 16 bits put behind it, which never went out
sok:    ldi   R3, 12         ; the next pulse 12 wraps after the frame
tdrop:  ldi   R5, 0          ; no reply waiting (the subroutines count down R5)
        ldi   R1, NATT
        st    R5, R1         ; and no attempts on the next one yet
        call  tfree
        goto  tloop
jw:     sstat R4
        bt    R4, 5
        jt    jw             ; the jam and its TP_IDL are over
        ldi   R3, 12         ; the next link pulse 12 wraps after it
        cp    R4, R2
        sub   R4, R7
        li    R0, 2240
        sub   R4, R0         ; c - (E + 2561): not negative after 512 BT
        bt    R4, 15
        jt    j1
        ldi   R1, NLATE
        call  inc            ; a late collision (4.2.8 WatchForCollision, 5.2.2.1.10)
j1:     ldi   R1, NCOLL
        call  inc
        ldi   R1, NATT
        call  inc            ; R0: attempts on this frame
        cmpi  R0, 16
        jt    txs            ; attemptLimit (4.4.2): the frame is dropped
        cp    R4, R0
        ldi   R2, 0
bm:     sett
        shl16 R2, -, -       ; R2 = 2^min(n, 10) - 1 after the n-th attempt
        bt    R2, 9
        jt    bm1            ; backoffLimit 10 (4.4.2)
        djnz  R4, bm
bm1:    call  rand
        ldi   R1, BOFF
        st    R0, R1         ; r slot times before the next attempt
        now                  ; they count from here
        goto  tloop
txs:    ldi   R1, NXS
        call  inc
        goto  tdrop
t2:     ldi   R1, HANDED
        ldp   R0, R1
        ld    R1, R1
        cmp   R0, R1
        jt    tloop
; ------------------------------------------------------------------------------------------------- T: a frame
; an ARP request for our IP: the reply in place
tframe: call  tbuf           ; R1 = the buffer's first word
        cp    R2, R1
        addi  R2, 7
        ldi   R4, ARPT
        ldi   R5, 5
        call  cmpn           ; EtherType 0x0806, hrd, pro, hln and pln, op
        jf    tdrop
        addi  R2, 8
        ldi   R4, CIP
        ldi   R5, 2
        call  cmpn           ; tpa
        jf    tdrop
        ldi   R0, 0
        ldi   R5, 9
tz:     stp   R0, R2         ; pad of zeros to 60 octets
        djnz  R5, tz
        cp    R4, R1
        addi  R4, 12
        cp    R2, R1
        addi  R2, 17
        ldi   R6, CFG
        ldi   R5, 5
        call  xmv            ; tha, tpa := sha, spa; sha, spa := our MAC and IP
        cp    R4, R1
        addi  R4, 17
        cp    R2, R1
        addi  R2, 1
        ldi   R5, 3
        call  copy           ; DA := the old sha
        ldi   R4, CFG
        ldi   R5, 3
        call  copy           ; SA := our MAC
        addi  R2, 4
        li    R0, 0x0200
        st    R0, R2         ; op := reply
        li    R5, 0x027F     ; a reply waits; R5 is also the CODING a collision writes
        goto  tloop
; ------------------------------------------------------------------------------------------------- subroutines
rxset:  li    R0, 0x002B | (RON << 8)
        scfg  R0, RXMODE     ; Manchester decoder on half cycles, RINV, hunt for RO falling, LSB first
        ldi   R0, 5
        scfg  R0, RXPER      ; 5 cycles per bit
        li    R0, RXOPTV
        scfg  R0, RXOPT
        ret
poly:   li    R0, 0x04C1
        scfg  R0, POLYHI
        li    R0, 0x1DB7
        scfg  R0, POLYLO     ; CRC-32 (802.3 3.2.8), left-aligned without x^32 (5.9.7)
        ret
; a link test pulse: a start with no word underruns, and its tail is the pulse
pulse:  li    R0, 0x8005
        scfg  R0, TXTAIL     ; tail 5 cycles: DE high for 100 ns with DI high; TFRAC 128
        sgo   tx
        now
        ldi   R3, 12
        ret
; R1 = the first word of the buffer RELEASED names
tbuf:   ldi   R1, RELEASED
        ld    R0, R1
        ldi   R1, BUF0
        bt    R0, 0
        jf    tb1
        ldi   R1, BUF1
tb1:    ret
tfree:  ldi   R1, RELEASED
inc:    ld    R0, R1
        addi  R0, 1
        st    R0, R1
        ret
; R5 words at R2 against R5 words at R4: T = 1 if all equal. Uses R0, R6
cmpn:   ldp   R0, R2
        ldp   R6, R4
        cmp   R0, R6
        jf    cmpx
        djnz  R5, cmpn
cmpx:   ret
; R5 words from R4 to R2. Uses R0
copy:   ldp   R0, R4
        stp   R0, R2
        djnz  R5, copy
        ret
; R5 words: R2 := R4, then R4 := R6, all three advancing. Uses R0
xmv:    ld    R0, R4
        stp   R0, R2
        ldp   R0, R6
        stp   R0, R4
        djnz  R5, xmv
        ret
; R0 := the generator's next value AND R2 (4.2.3.2.5). Uses R1, R4, R6
rand:   ldi   R1, SEED
        ld    R0, R1
        rdtb  R4
        xor   R0, R4         ; TB: each station's runs from its own reset
        ldi   R4, CFG + 2
        ld    R4, R4
        xor   R0, R4         ; the MAC's last two octets: no two stations share them
        ldi   R4, 0x2D       ; x^16 + x^5 + x^3 + x^2 + 1, below x^16
        clrt
        ldi   R6, 16
rs:     crcl  R0, R4         ; one LFSR step a slot
        djnz  R6, rs
        st    R0, R1
        and   R0, R2
        ret
