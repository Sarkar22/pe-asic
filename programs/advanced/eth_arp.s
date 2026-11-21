; 10BASE-T node at 50 MHz: answers ARP requests (RFC 826) and ICMP echo requests (RFC 792) for its own addresses,
; sends link test pulses (IEEE 802.3 14.2.1.1), and ignores everything else. Two threads, 512-word memory: 401 words
; placed (389 of code, 12 of counters and constants), 5 words of addresses and two 52-word buffers unplaced, words 0
; to 509 in use. The shift units do the line work (section 5.9): thread R's unit decodes the Manchester code with
; clock recovery and checks the CRC-32 FCS; thread T's unit sends Manchester, appends the FCS, and makes TP_IDL and
; the link pulses. R and T may be any two threads; the host starts R at label rx and T at tx.
;
; Board (802.3 clause 14 through an RS-485 transceiver with DE, and 1:1 magnetics):
;   DI = uo4 (shift output A), DE = uo5 (shift output B), RO = ui4, low while RD+ is above RD- (RD+ on the
;   receiver's B input) and high on a silent line (the receiver's fail-safe). uo pins are low from reset (section
;   8), so DE holds the driver off until T runs.
; Mailbox: MCFG (M4): before any frame can be answered the host posts five words, the MAC as three, then the IP as
;   two, each word two octets in wire order, the first octet in the low byte (02:00:00:00:00:01 is 0x0002, 0x0000,
;   0x0100). From the host rather than constants, because each board needs its own MAC and the network picks the
;   IP: one image serves every board. R takes them before it starts receiving.
; Counters the host can read with RDMEM while threads 2 and 3 stay stopped (section 9.4): NKEEP (word 7) frames
;   kept, NBAD (8) runts, FCS errors, odd bit counts and lost words, NOTHER (9) frames for another station, NBUSY
;   (10) frames dropped while both buffers were in use. Every frame with an SFD lands in exactly one.
;
; R: every frame. 802.3 Manchester has a rising mid-bit edge for a 1 on the pair (7.3.1.1, 14.2.1.1), so a falling
;   one on RO: the decoder (RKIND 3, RPER 5, RACQ and RGAIN 3, section 5.9.6) hunts for RO falling, the first
;   preamble bit's mid-bit edge, RINV makes the level after each clock edge the bit, and the reception ends at the
;   first bit with no clock edge, inside TP_IDL (14.3.1.3.3). RKIND 3 times every edge to the half cycle, 10 ns,
;   with the falling-edge sample (section 3.1), which leaves 13 ns between the latest bit-boundary edge and the
;   earliest clock edge at 802.3's +/-13.5 ns of jitter (14.3.1.3.1), less the clock's duty error (section 7.3). Sync is the last preamble octet and the SFD, 0xD555
;   (3.2.2, 5.9.6), so 8 preamble bits are enough. A link pulse or an FLP pulse starts a reception that finds no
;   sync and gives no word. The receive half's CRC-32 covers DA to FCS and checks the residue (5.9.7). Full words
;   go into the next free buffer, at most BUFW of them; the rest of a longer frame is taken unstored. A frame is
;   kept if its check passed, no word was lost (sstat T), it is whole octets (a last word of 0 or 8 bits), it has
;   at least 64 octets with its FCS (802.3 4.4.2, 4.2.4.2.2) and its DA is broadcast or ours (4.2.4.1.1). R hands
;   a kept frame to T by writing HANDED after the frame; T writes RELEASED when it is done with one, and R stores
;   into buffer HANDED mod 2 only while HANDED - RELEASED is under 2 (section 4.2: a store is seen by any later
;   load), so the two threads share no mailbox word.
;   Timing (sections 2, 4.2, 5.9.6): a word comes every 16 bit times, 80 cycles, 20 slots. The store loop takes 5
;   slots a word; the buffer choice before a frame's first store 16 slots with that store, so the second word
;   (80 cycles later) is taken in time; the checks after a frame at most 67 slots, 268 cycles, while the next
;   frame's first word comes no sooner than 47 + 32 bit times, 395 cycles, after the end of the frame before (4.4.2
;   note 1: the gap may shrink to 47 bit times at a receiver; 8 preamble bits, the SFD and one word). The last word
;   is taken at once, so the hunt starts again 1.75 bit times after a frame (5.9.6, End).
; T: link pulses, then each frame R hands over. A pulse is a send with no word: it underruns at once and its tail
;   (5.9.5) holds DE high for TAIL = 5 cycles with DI already high (IDLEA 1), so only DE moves and the transceiver
;   never shows the old DI (FN6307: tZH is shorter than tDD). A frame is 4 preamble words of no kind, then the frame
;   words of kind crc, the last one fcs and last: the unit appends the FCS, x^31 first (5.9.7, 3.2.8), and its tail
;   of 15 cycles with DI and DE high is TP_IDL (14.3.1.2.1). TPER 2 with TFRAC 128 and THALF puts every half bit
;   on the half-cycle grid, exactly 2.5 cycles, 50 ns, so 10 Mb/s from a 50 MHz clock (5.9.5, 7.3.2) with no
;   deterministic jitter at a 50% clock; a duty error of d ns puts each edge d/2 ns off the grid (section 7.3). Two words wait before the start commit
;   and the feed loop takes 4 slots a word against 20 (5.9.4), so a send never underruns. The next pulse is due 12
;   wraps of TB after a pulse or a frame, 14.4 to 15.7 ms, inside 16 +/- 8 ms (14.2.1.1); T samples TB at least
;   every 7,000 cycles (its longest work, a reply's checks and send), far inside a wrap of 65,536, so it misses
;   none (3.5). Before a frame T waits for RO to stay high for 96 bit times,
;   480 cycles, with `until` and a timeout from `now` (3.3, 4.2.3.2.1, 4.4.2); its own pair doesn't show on RO, so
;   after its own frame it first waits for status bit 5 to clear, the tail included. There is no collision
;   detection: a frame that arrives while a reply goes out collides with it, and is received and answered after it.
;   ARP: EtherType 0x0806, hrd 1, pro 0x0800, hln 6, pln 4, op 1 and tpa ours; the reply is built in the request's
;   buffer by RFC 826's swap, to the request's sha, padded with zeros to 60 octets (RFC 894). ICMP echo: IPv4 without
;   options or fragments, to our IP and not to the link broadcast (RFC 1122 3.3.6), both checksums right (RFC 791
;   3.1, 792; RFC 1122 3.2.1.2); the reply goes to the request's SA with the IP addresses swapped, TTL 64 (never 0,
;   RFC 1122 3.2.1.7) and the header checksum computed again, type 0 with the ICMP checksum updated by RFC 1624
;   eq. 3, the identifier, sequence number and data returned. A request is answered only if the whole datagram fits
;   the buffer: up to 85 octets of IP, 57 of ICMP data (Linux's default ping is 56).
.equ DI uo4
.equ DE uo5
.equ RO ui4
.equ RON 4                  ; RO as a pin number, for RXMODE's RPIN (5.9.3)
.equ MCFG 4
.equ BUFW 51                ; full words a buffer keeps: 102 octets, FCS included
.equ RXOPTV 0x0E00          ; RACQ for the preamble, RGAIN 3, RFRAC 0, CSKIP 0
; data, at addresses an ldi reaches
.equ CFG 0                  ; MAC (3 words), then IP (2 words), from the host
.equ CIP 3
.equ HANDED 5               ; frames R has handed to T; written by R only
.equ RELEASED 6             ; frames T has finished with; written by T only
.equ NKEEP 7
.equ NBAD 8
.equ NOTHER 9
.equ NBUSY 10
.equ TBP 11                 ; T's last sample of TB, to see it wrap
.equ ARPT 12                ; EtherType, hrd, pro, hln and pln, op of an ARP request, as received words
.equ BUF0 17                ; each buffer: its word count, then the frame from word 0
.equ BUF1 BUF0 + BUFW + 1
        .org  HANDED
        .data 0
        .data 0
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
        li    R0, 0x002B | (RON << 8)
        scfg  R0, RXMODE     ; Manchester decoder on half cycles, RINV, hunt for RO falling, LSB first
        ldi   R0, 5
        scfg  R0, RXPER      ; 5 cycles per bit
        li    R0, RXOPTV
        scfg  R0, RXOPT
        li    R0, 0x8000
        scfg  R0, RXCNT      ; a 16-bit sync pattern, no bit limit, no timeout (RXEND 0 from reset)
        li    R0, 0xD555
        scfg  R0, RXSYNC     ; the last preamble octet, then the SFD
        call  poly
        li    R0, 0xC704
        scfg  R0, CHKHI      ; the residue a good frame leaves
        li    R0, 0xDD7B
        scfg  R0, CHKLO
        ldi   R0, 0xFF
        scfg  R0, CODING     ; CRC-32 on the receive half: all ones to start, checked at the end
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
        ld    R3, R1
        cp    R4, R5
        sub   R4, R3         ; frames T still holds: 0, 1 or 2
        bt    R4, 1
        jt    rbusy
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
rend:   sstat R3             ; R3[4:0]: bits in the last word. T: a word was lost
        jt    rbad
        srd   R4, end
        bt    R4, 0          ; end flag 0: the FCS is right
        jf    rbad
        andi  R3, 0x17
        cmpi  R3, 0          ; a last word of 0 or 8 bits: whole octets
        jf    rbad
        ldi   R4, BUFW - 30
        lt    R2, R4         ; at least 32 words stored: 64 octets with the FCS
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
rkeep:  ldi   R3, BUFW + 1
        sub   R3, R2         ; words stored
        cp    R1, R6
        subi  R1, 1
        st    R3, R1         ; in the buffer's first word
        addi  R5, 1
        ldi   R1, HANDED
        st    R5, R1         ; T takes the frame from here on
        ldi   R1, NKEEP
        goto  rcnt
rbad:   ldi   R1, NBAD
        goto  rcnt
rother: ldi   R1, NOTHER
        goto  rcnt
rbusy:  rxw   R0             ; both buffers in use: take the frame unstored
        jf    rbusy
        ldi   R1, NBUSY
rcnt:   ld    R0, R1
        addi  R0, 1
        st    R0, R1
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
        li    R0, 0x02FF
        scfg  R0, CODING     ; CRC-32 on the send half, appended complemented
        pcfg  DI, PP
        pcfg  DE, PP
        put   DI, 1
        put   DE, 0
        now
        sstop tx             ; outputs at their idle levels: A 1, B 0
        smap  DI, A
        smap  DE, B
        now
        ldi   R5, 0          ; octets of a reply waiting to go, 0 for none
        call  pulse          ; the first link pulse at once
tloop:  rdtb  R2
        ldi   R1, TBP
        ld    R0, R1
        st    R2, R1
        lt    R2, R0         ; TB went down: it wrapped
        jf    t1
        djnz  R3, t1         ; R3: wraps left before the next pulse
        call  pulse
t1:     cmpi  R5, 0
        jt    t2
        li    R2, 480
        now
        until RO, 0, R2      ; T 1: the line was active within 96 bit times
        jt    tloop
        call  tbuf
        addi  R1, 1          ; word 0 of the reply
        li    R0, 0x800F
        scfg  R0, TXTAIL     ; tail 15 cycles: TP_IDL; TFRAC 128
        li    R0, 0x5555
        txw   R0, 16
        txw   R0, 16
        sgo   tx
        now                  ; the frame starts at this commit
        txw   R0, 16
        lui   R0, 0xD5
        txw   R0, 16         ; the 7th preamble octet, then the SFD
        clrt
        cp    R0, R5
        shr16 R0, -, -       ; R0 = octets / 2, T = 1 for an odd count
        jt    s1
        subi  R0, 1          ; even: the last of them ends the frame
s1:     cp    R2, R0
s2:     ldp   R0, R1
        txw   R0, 16, crc
        djnz  R2, s2
        ldp   R0, R1
        jt    s3
        txw   R0, 16, fcs, last
        goto  s4
s3:     txw   R0, 8, fcs, last
s4:     sstat R0
        bt    R0, 5
        jt    s4             ; until the tail ends
        ldi   R3, 12         ; the next pulse 12 wraps after the frame
tdrop:  ldi   R5, 0          ; no reply waiting (the subroutines count down R5)
        call  tfree
        goto  tloop
t2:     ldi   R1, HANDED
        ld    R0, R1
        ldi   R1, RELEASED
        ld    R1, R1
        cmp   R0, R1
        jt    tloop
; ------------------------------------------------------------------------------------------------- T: a frame
tframe: call  tbuf           ; R1 = the buffer's first word, R6 = words stored
        cp    R2, R1
        addi  R2, 1
        ld    R5, R2         ; DA's first two octets
        addi  R2, 6
        ld    R0, R2         ; the EtherType
        cmpi  R0, 0x08
        jt    tip
; ARP request for our IP: the reply in place
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
        ldi   R5, 60
        goto  tloop
; IPv4 (EtherType 0x0800): an ICMP echo request for our IP
tip:    bt    R5, 0
        jt    tdrop          ; to the link broadcast, not an IP broadcast: discard (RFC 1122 3.3.6)
        addi  R2, 1
        ldp   R0, R2
        andi  R0, 0xFF
        cmpi  R0, 0x45
        jf    tdrop          ; version 4, header of 5 words, no options
        ldp   R0, R2
        swap  R0             ; total length
        ldi   R4, 28
        lt    R0, R4
        jt    tdrop          ; shorter than an IP and an ICMP echo header
        cp    R4, R6
        add   R4, R6
        subi  R4, 17
        lt    R4, R0
        jt    tdrop          ; 14 + total > 2 x words - 3: longer than the frame's octets before its FCS
        st    R0, R1         ; the buffer's first word keeps the total length from here
        addi  R2, 1
        ldp   R0, R2
        li    R4, 0xFF3F
        and   R0, R4
        cmpi  R0, 0
        jf    tdrop          ; more fragments, or an offset: a fragment
        ldp   R0, R2
        swap  R0
        andi  R0, 0xFF
        cmpi  R0, 1
        jf    tdrop          ; protocol 1, ICMP
        addi  R2, 3
        ldi   R4, CIP
        ldi   R5, 2
        call  cmpn
        jf    tdrop          ; IP destination: ours
        ld    R0, R2
        cmpi  R0, 0x08
        jf    tdrop          ; type 8, code 0: echo request
        cp    R2, R1
        addi  R2, 8
        ldi   R5, 10
        call  csum
        not   R0
        cmpi  R0, 0
        jf    tdrop          ; the header checksum
        ld    R0, R1
        subi  R0, 20         ; ICMP octets
        clrt
        shr16 R0, -, -       ; R0 = their words, T = 1 for an odd count
        cp    R5, R0
        jf    ti1
        cp    R4, R2
        add   R4, R5
        ld    R0, R4
        andi  R0, 0xFF
        st    R0, R4         ; the octet after an odd message is 0, as its checksum takes it (RFC 792)
        addi  R5, 1
ti1:    call  csum
        not   R0
        cmpi  R0, 0
        jf    tdrop          ; the ICMP checksum
        cp    R0, R1
        addi  R0, 31
ti2:    lt    R2, R0
        jf    ti3
        ldi   R4, 0
        stp   R4, R2         ; pad of zeros to 60 octets
        goto  ti2
ti3:    cp    R4, R1
        addi  R4, 4
        cp    R2, R1
        addi  R2, 1
        ldi   R6, CFG
        ldi   R5, 3
        call  xmv            ; DA := SA, SA := our MAC
        addi  R4, 5
        li    R0, 0x0140
        stp   R0, R4         ; TTL 64, protocol 1
        ldi   R0, 0
        stp   R0, R4         ; header checksum 0, for the sum
        cp    R2, R4
        addi  R2, 2
        ldi   R6, CIP
        ldi   R5, 2
        call  xmv            ; IP destination := source, source := ours
        cp    R2, R1
        addi  R2, 8
        ldi   R5, 10
        call  csum
        not   R0
        subi  R2, 5
        st    R0, R2         ; the header checksum, recomputed
        addi  R2, 5
        ldi   R0, 0
        stp   R0, R2         ; type 0, code 0: echo reply
        ld    R0, R2
        not   R0
        li    R4, 0xFFF7     ; ~0x0008, the old type and code as received
        add   R0, R4
        lt    R0, R4
        jf    ti4
        addi  R0, 1
ti4:    not   R0
        st    R0, R2         ; the ICMP checksum, by RFC 1624 eq. 3
        ld    R5, R1
        addi  R5, 14         ; octets to send: the frame without its FCS
        ldi   R0, 60
        lt    R5, R0
        jf    tloop
        ldi   R5, 60         ; at least 60 (802.3 4.4.2)
        goto  tloop
; ------------------------------------------------------------------------------------------------- subroutines
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
; R1 = the first word of the buffer RELEASED names, R6 = its contents
tbuf:   ldi   R1, RELEASED
        ld    R0, R1
        ldi   R1, BUF0
        bt    R0, 0
        jf    tb1
        ldi   R1, BUF1
tb1:    ld    R6, R1
        ret
tfree:  ldi   R1, RELEASED
        ld    R0, R1
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
; R0 = the one's complement sum of R5 words at R2 (RFC 1071). Uses R6
csum:   ldi   R0, 0
cs1:    ldp   R6, R2
        add   R0, R6
        lt    R0, R6         ; a carry out of bit 15
        jf    cs2
        addi  R0, 1          ; goes back in at bit 0
cs2:    djnz  R5, cs1
        ret
