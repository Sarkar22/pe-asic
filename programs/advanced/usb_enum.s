; USB low-speed device on endpoint 0 at 50 MHz: answers the control transfers of enumeration (USB 2.0 chapter 9)
; with GET_DESCRIPTOR of its 18-byte device descriptor and SET_ADDRESS, and STALLs every other request (9.2.7).
; The shift units do the line work (section 5.9): NRZI, bit stuffing, the CRC-5 check of tokens, the CRC-16 check
; of the host's data and the CRC-16 appended to the device's own, the end of packet, and the clock recovery of
; every edge. Three threads, 512-word memory: 493 words placed and a 6-word buffer, words 0 to 498 in use, 13
; free. The host starts S at label dev, A at tok and W at rst, any three threads in one RUN, and leaves the fourth
; stopped so that RDMEM can read the counters (section 9.4).
;
; Board: D+ on io0 and D- on io1 at 3.3 V, the device's 1.5 kohm pull-up on D- (low speed, USB 7.1.5.1), the
; host's 15 kohm pull-downs on both. uio pins are undriven from reset (section 8), so the line idles at J until
; the device answers. The descriptor's idVendor and idProduct are placeholders: the host writes a product's own
; over them with WRMEM before RUN.
; Memory the host can read (RDMEM): ADDRV (word 13) the address in use, NSETUP (14) SETUP data packets ACKed,
;   NREQ (15) control transfers whose status stage completed, RSTN (11) resets seen.
;
; Line (5.9.3, 5.9.5 to 5.9.7). Both receive halves sample D+ every 33 + 85/256 cycles (1.5 Mb/s), half a bit
;   after each edge, which restarts the count (RESYNC, RGAIN 0), decode NRZI (RDIFF: no change is a 1), hunt for
;   the J-to-K of an SOP (D+ rising), take the SYNC field as their sync pattern, drop each stuffed 0 after six 1s,
;   leave the PID out of the CRC (CSKIP 8), and end a reception at the first sample where D+ and D- are both low,
;   the EOP (REOP, RCLK D-). S's unit sends NRZI on both lines (TXMAP 0x0101, toggling) at 33 cycles a bit,
;   1.5152 Mb/s, +1.0% (TLDRATE +/-1.5%); without a fraction every edge stays on the cycle grid, so its jitter is
;   0 (7.1.13.1.1 allows +/-10 ns between paired transitions, which a fraction of 85/256 breaks). The SYNC and PID
;   go as one word of kind stuff, data as kind crc, the last data word as fcs: the unit stuffs from the SYNC on and
;   appends the CRC-16, MSb first (8.3.5). Its tail is the EOP's SE0, 67 cycles, 1.34 us (TLEOPT 1.25 to 1.50);
;   the idle levels after it are J.
; S, the device: owns D+ and D-, and its unit's receive half checks the CRC-16 of every packet (CODING 0xE4EF,
;   POLYHI 0x8005, residue 0x800D). For its own data packets it moves the CRC register to the send half (CTX 1,
;   0xE6EF) and back after: the polynomial is the same, so nothing else changes. It stores the words of each
;   packet (6 at most: 8 data bytes), runs `now` 4 slots after the word that ends the reception, about 16 cycles
;   into the EOP's SE0, and starts any answer NRSP cycles after it with OE on in the same commit: its SOP comes
;   4.4 to 4.5 bit times after the SE0-to-J of a 2-bit EOP, 4.1 to 4.3 after one of 1.50 us and 5.4 to 5.6 after one
;   of 670 ns, the shortest a receiver must take (TRSPIPD1 6.5, inter-packet delay at least 2: 7.1.18.1; TLEOPR).
;   The slowest decisions, an IN answered with data and an OUT's handshake, take 33 slots, 132 cycles, of the 176.
;   After its EOP it takes its own packet, which its receive half sees because a uio pin reads its own drive (5.2),
;   waits for its own SE0-to-J with `until` (3 cycles after the pad, 5.7) and drives J NREL + 3 = 34 cycles, then
;   releases (7.1.7.4.1: J for one bit, then high impedance).
; A, the token check: its unit hunts on the same D+ with CRC-5 (CODING 0xE4E4, POLYHI 0x2800, residue 0x6000).
;   For every reception it writes VERD = receptions << 3 | end flags, the flags 0 unless the last word has 8 bits.
;   S counts the same receptions, its own packets included, and takes a token as valid when VERD equals its count
;   << 3 | 1: CRC-5 right, no stuff error, 24 bits. Both units end a reception in the same cycle, and A's store of
;   VERD comes at least 13 cycles before S's load of it. A different count means one of them missed a reception;
;   S takes A's count, so the next token is judged again.
; W, the reset: waits for SE0, then for the line to leave it within TDET cycles, 5 us. An SE0 longer than that
;   is a reset (7.1.7.5: TDETRST is at least 2.5 us; a reset is at least 10 ms; keep-alives and EOPs last 1.25 to
;   1.5 us). W counts it in RSTN; S compares RSTN with RSEEN at the first word of each packet and at every ACK,
;   and goes back to the Default state: address 0, no transfer, nothing pending (9.1.1.3).
; Transfers (8.4.6, 8.5.3, 8.6, 9.4). A token counts for us if its address is ours and its endpoint 0. SETUP and
;   OUT mark the next packet as their data. The SETUP's data, if its CRC passes, is always ACKed (8.4.6.4), and
;   starts a new transfer: GET_DESCRIPTOR(DEVICE) sends min(wLength, 18) bytes in packets of 8, DATA1 first;
;   SET_ADDRESS keeps the old address until the host ACKs the zero-length status packet (9.2.6.3, 8.5.3.1), and
;   a SETUP before that ACK cancels it, as it ends any transfer (8.5.3); anything else is STALLed until the next
;   SETUP. IN: the next data packet, the status packet, STALL, or NAK when
;   nothing is due. The data toggle flips and the data stage moves on only when the host ACKs (8.6.2, 8.6.3): an
;   IN after a lost ACK gets the same packet again. A short packet or wLength reached ends the data stage; an OUT
;   with a good DATA1 or DATA0 then gets ACK and ends the transfer, as it does at any point of a control read.
;   Corrupted packets get no handshake. Keep-alives, a low-speed EOP (Table 7-2 note 4), start no reception, so
;   only W sees them.
.equ DP io0
.equ DM io1
.equ NRSP 176               ; cycles from S's time reference to its SOP
.equ NREL 31                ; J driven NREL + 3 cycles after the device's own EOP
.equ TDET 250               ; SE0 cycles that W takes as a reset
.equ CODRX 0xE4EF           ; CRC-16, the receive half: all ones to start, checked at the end; stuffing; REOP
.equ CODTX 0xE6EF           ; the same with CTX 1: the send half appends
.equ WACK 0xD280            ; SYNC with a PID, sent low byte first: ACK
.equ WNAK 0x5A80
.equ WSTALL 0x1E80
.equ WDATA0 0xC380
.equ WDATA1 0x4B80
; data, at addresses an ldi reaches
.equ STAGE 0                ; 0 none, 1 data stage of a control read, 2 its status stage, 3 the status IN of a
                            ; request with no data, 4 stalled until the next SETUP
.equ TOK 1                  ; the packet after a SETUP (1) or OUT (2) for us is its data
.equ HS 2                   ; S's last packet waits for the host's ACK: 1 data, 2 the status packet
.equ TOG 3                  ; SYNC and PID of the next data stage packet: WDATA1 or WDATA0
.equ PTR 4                  ; its first word
.equ LEFT 5                 ; bytes still to send
.equ PADDR 6                ; the address SET_ADDRESS gave, taken at its status ACK; bit 15: none
.equ INW 7                  ; the answer to the next IN: SYNC and PID word,
.equ INP 8                  ; its data words,
.equ INN 9                  ; and its bytes, 0 for a zero-length packet
.equ RSEEN 10               ; resets S has acted on
.equ RSTN 11                ; resets W has seen
.equ VERD 12                ; A's last verdict
.equ ADDRV 13
.equ NSETUP 14
.equ NREQ 15
.equ OUTW 16               ; the handshake for the next OUT's good data: ACK, or STALL
.equ BUF 17                 ; the words of the packet being received: 6
.equ DESC 23                ; the device descriptor: 9 words
        .org  STAGE
        .data 0
        .data 0
        .data 0
        .data WDATA1
        .data DESC
        .data 0
        .data 0x8000
        .data WNAK
        .data DESC
        .data 0
        .data 0
        .data 0
        .data 0
        .data 0
        .data 0
        .data 0
        .data WACK
        .org  DESC           ; 9.6.1, Table 9-8: two bytes a word, the first in the low byte
        .data 0x0112         ; bLength 18, bDescriptorType DEVICE
        .data 0x0110         ; bcdUSB 1.10
        .data 0x0000         ; bDeviceClass, bDeviceSubClass 0: each interface names its own
        .data 0x0800         ; bDeviceProtocol 0, bMaxPacketSize0 8 (5.5.3: low speed)
        .data 0x1209         ; idVendor, a placeholder
        .data 0x0001         ; idProduct, a placeholder
        .data 0x0100         ; bcdDevice 1.00
        .data 0x0000         ; iManufacturer, iProduct: no strings
        .data 0x0100         ; iSerialNumber none, bNumConfigurations 1
; ------------------------------------------------------------------------------------------------- S: the device
dev:    sstop txrx
        pcfg  DP, IN
        pcfg  DM, IN
        ldi   R0, 0x58
        scfg  R0, TXMODE     ; one symbol a bit, LSB first, both outputs toggle, idle J (A 0, B 1), tail SE0
        li    R0, 0x0101
        scfg  R0, TXMAP      ; both outputs 1 - b: a 0 changes the line
        ldi   R0, 33
        scfg  R0, TXPER
        ldi   R0, 67
        scfg  R0, TXTAIL     ; the EOP's SE0, TFRAC 0
        call  rxcfg
        li    R0, 0x8005
        scfg  R0, POLYHI     ; CRC-16: x^16 + x^15 + x^2 + 1
        li    R0, 0x800D
        scfg  R0, CHKHI      ; its residue, 1000000000001101 (8.3.5.2)
        li    R0, CODRX
        scfg  R0, CODING
        sstop tx             ; the outputs show J
        smap  DP, A
        smap  DM, B
        sgo   rx
        now                  ; the hunt starts at this commit; OE stays 0
        ldi   R6, 0x69       ; R6: the first word of an IN to us, address 0, endpoint 0
        ldi   R3, 0          ; receptions << 3, as A counts them
rnext:  ldi   R5, BUF
        ldi   R4, 5
        rxw   R1             ; a packet's first word: PID, and the address or the first data byte
        stp   R1, R5
        jt    rend           ; one word: a handshake
        call  rchk           ; time to spare until the next word: any reset since the last packet?
rloop:  rxw   R0
        stp   R0, R5
        jt    rend
        djnz  R4, rloop
rskip:  rxw   R0             ; past 6 words: no packet of a low-speed transfer; take the rest unstored
        jf    rskip
        ldi   R5, 0          ; matches nothing below
rend:   now                  ; the time reference, 4 slots after the rxw that took the last word
        addi  R3, 8
        cmp   R1, R6
        jt    tin            ; IN, our address, endpoint bit 0 clear
        cp    R2, R1
        andi  R2, 255        ; the PID
        cmpi  R2, 0xC3
        jt    tdata
        cmpi  R2, 0x4B
        jt    tdata
        cmpi  R2, 0xD2
        jt    thand
        cp    R7, R1
        sub   R7, R6
        addi  R7, 0x69       ; the PID if the address is ours and endpoint bit 0 clear
        ldi   R4, 1
        cmpi  R7, 0x2D
        jt    ttok           ; SETUP
        ldi   R4, 2
        cmpi  R7, 0xE1
        jt    ttok           ; OUT
        goto  tnone
; an IN token for us: answer it, 33 slots at most from the reference to the start
tin:    cmpi  R5, BUF + 2    ; two words
        jf    tnone
        tsti  R0, 7          ; endpoint bits 3:1 clear
        jt    tnone
        ldi   R7, VERD
        ld    R7, R7
        xor   R7, R3
        cmpi  R7, 1          ; A's verdict for this reception: CRC-5 right, 24 bits
        jf    tbad
        ldi   R7, INW
        ldp   R0, R7         ; SYNC and PID
        bt    R0, 8          ; DATA0 and DATA1 have PID bit 0 set, the handshakes clear
        jf    ihs
        ldp   R5, R7
        ld    R4, R7         ; bytes
        cmpi  R4, 0
        jt    izlp
        call  sdata
        ldi   R0, 1
        goto  ihs2
izlp:   call  szlp
        ldi   R0, 2
        goto  ihs2
ihs:    call  shs            ; NAK or STALL
        ldi   R0, 0
ihs2:   ldi   R7, HS
        st    R0, R7         ; what the host's ACK will mean
        goto  tclr
; SETUP or OUT for us (R4: 1 or 2): the next packet is its data
ttok:   call  tvalid
        jf    tbad
        ldi   R7, TOK
        st    R4, R7
        ldi   R7, HS
        ldi   R0, 0
        st    R0, R7
        goto  rnext
; a data packet
tdata:  ldi   R7, TOK
        ld    R7, R7
        cmpi  R7, 0
        jt    tnone          ; no SETUP or OUT of ours before it
        srd   R4, end
        cmpi  R4, 1          ; CRC-16 right, no stuff error
        jf    tnone          ; no handshake to a corrupted packet (8.4.6.3)
        sstat R2
        jt    tnone          ; a word was lost
        andi  R2, 7
        cmpi  R2, 0
        jf    tnone          ; not whole bytes: a last word of 0 or 8 bits
        cmpi  R7, 1
        jt    dsetup
        ldi   R2, OUTW       ; the OUT's data: ACK, except STALL while stalled or in a status IN
        ld    R0, R2
        call  shs
        ldi   R2, STAGE
        ld    R2, R2
        cmpi  R2, 1
        jt    dfin
        cmpi  R2, 2
        jf    tclr
dfin:   call  done           ; the status stage of a control read: the transfer is complete
        goto  tclr
; the SETUP's data: ACK it, then read the request
dsetup: li    R0, WACK
        call  shs
        ldi   R7, NSETUP
        ld    R0, R7
        addi  R0, 1
        st    R0, R7
        ldi   R7, HS
        ldi   R0, 0
        st    R0, R7
        not   R0             ; 0xFFFF, bit 15 set: no address pending
        ldi   R7, PADDR
        st    R0, R7         ; a SETUP ends the transfer before it (8.5.3), a SET_ADDRESS's included (9.4.6)
        cmpi  R5, BUF + 6    ; 8 data bytes: 88 bits, 6 words
        jf    sstall
        ldi   R5, BUF
        ldp   R0, R5
        swap  R0
        andi  R0, 255        ; bmRequestType
        ldp   R1, R5         ; bRequest | wValue low << 8
        ldp   R2, R5         ; wValue high | wIndex low << 8
        cmpi  R0, 0x80
        jt    sget
        cmpi  R0, 0
        jf    sstall
        cp    R0, R1         ; SET_ADDRESS: bRequest 5, address up to 127 (9.4.6)
        andi  R0, 255
        cmpi  R0, 5
        jf    sstall
        andi  R2, 255
        cmpi  R2, 0
        jf    sstall
        swap  R1
        andi  R1, 255
        bt    R1, 7
        jt    sstall
        ldi   R7, PADDR
        st    R1, R7         ; taken when the host ACKs the status packet
        ldi   R0, 3          ; the status stage: an IN, answered with a zero-length DATA1
        goto  snew
sget:   cmpi  R1, 6          ; GET_DESCRIPTOR, wValue low (the index) 0
        jf    sstall
        andi  R2, 255
        cmpi  R2, 1          ; descriptor type DEVICE (Table 9-5)
        jf    sstall
        ldp   R0, R5         ; wIndex high | wLength low << 8
        ld    R4, R5         ; wLength high | CRC low << 8
        swap  R0
        andi  R0, 255
        andi  R4, 255
        swap  R4
        or    R0, R4         ; wLength
        ldi   R4, 18
        lt    R0, R4
        jt    sg1
        ldi   R0, 18         ; min(wLength, 18) bytes (9.4.3)
sg1:    ldi   R7, LEFT
        st    R0, R7
        ldi   R7, PTR
        ldi   R1, DESC
        st    R1, R7
        ldi   R7, TOG
        li    R1, WDATA1     ; the data stage starts with DATA1 (8.5.3)
        st    R1, R7
        cmpi  R0, 0
        ldi   R0, 1
        jf    snew
        ldi   R0, 3          ; wLength 0: no data stage, the status stage is an IN
        goto  snew
sstall: ldi   R0, 4          ; a request error: STALL until the next SETUP (9.2.7, 8.5.3.4)
snew:   ldi   R7, STAGE
        st    R0, R7
        call  prep
        goto  tclr
; a handshake from the host
thand:  cmpi  R5, BUF + 1
        jf    tnone
        sstat R2
        andi  R2, 31
        cmpi  R2, 8
        jf    tnone
        call  rchk
        ldi   R7, HS
        ld    R0, R7
        cmpi  R0, 1
        jt    hadv
        cmpi  R0, 2
        jf    tnone
        ldi   R7, PADDR      ; our status packet was ACKed: a SET_ADDRESS takes effect now (9.4.6)
        ld    R0, R7
        bt    R0, 15
        jt    hfin
        ldi   R7, ADDRV
        st    R0, R7
        cp    R6, R0
        swap  R6
        ori   R6, 0x69
        li    R0, 0x8000
        ldi   R7, PADDR
        st    R0, R7
hfin:   call  done
        goto  tnone
hadv:   ldi   R7, INP        ; our data packet was ACKed: the toggle flips and the stage moves on (8.6.3)
        ld    R0, R7
        addi  R0, 4
        ldi   R7, PTR
        st    R0, R7
        ldi   R7, INN
        ld    R1, R7
        ldi   R7, LEFT
        ld    R0, R7
        sub   R0, R1
        st    R0, R7
        ldi   R7, TOG
        ld    R2, R7
        li    R4, WDATA0 ^ WDATA1
        xor   R2, R4
        st    R2, R7
        cmpi  R1, 8
        jf    hend           ; a short packet ends the data stage
        cmpi  R0, 0
        jf    hprep          ; and so does wLength reached (5.5.3)
hend:   ldi   R7, STAGE
        ldi   R0, 2
        st    R0, R7
hprep:  call  prep
        goto  tnone
; a token that failed A's check: if A counted another reception, take its count
tbad:   ldi   R7, VERD
        ld    R7, R7
        cp    R0, R7
        xor   R0, R3
        ldi   R2, 8
        lt    R0, R2
        jt    tnone          ; same count: a bad CRC or length
        ldi   R2, 7
        not   R2
        and   R7, R2
        cp    R3, R7
tnone:  ldi   R0, 0
        ldi   R7, HS
        st    R0, R7
tclr:   ldi   R0, 0
        ldi   R7, TOK
        st    R0, R7
        goto  rnext
; T: R5, R0 and A's verdict make a token for us: two words, endpoint bits 3:1 clear, CRC-5 right, 24 bits
tvalid: cmpi  R5, BUF + 2
        jf    tvret
        tsti  R0, 7
        jt    tvno
        ldi   R2, VERD
        ld    R2, R2
        xor   R2, R3
        cmpi  R2, 1
tvret:  ret
tvno:   clrt
        ret
; a reset since the last packet: back to the Default state (9.1.1.3)
rchk:   ldi   R2, RSTN
        ld    R0, R2
        ldi   R2, RSEEN
        ld    R4, R2
        cmp   R0, R4
        jt    rcret
        st    R0, R2
        ldi   R6, 0x69
        ldi   R0, 0
        ldi   R2, STAGE
        stp   R0, R2         ; STAGE
        stp   R0, R2         ; TOK
        stp   R0, R2         ; HS
        ldi   R2, ADDRV
        st    R0, R2
        li    R0, 0x8000
        ldi   R2, PADDR
        stp   R0, R2
        li    R0, WNAK
        st    R0, R2         ; INW: nothing is due
        li    R0, WACK
        ldi   R2, OUTW
        st    R0, R2
rcret:  ldi   R4, 5
        ret
; the end of a transfer's status stage
done:   ldi   R5, NREQ
        ld    R0, R5
        addi  R0, 1
        st    R0, R5
        ldi   R5, STAGE
        ldi   R0, 0
        st    R0, R5
; INW, INP and INN for the next IN, from the stage
prep:   ldi   R2, STAGE
        ld    R2, R2
        li    R0, WACK
        cmpi  R2, 3
        jt    pout
        cmpi  R2, 4
        jf    pout1
pout:   li    R0, WSTALL
pout1:  ldi   R5, OUTW
        st    R0, R5
        ldi   R5, INW
        cmpi  R2, 1
        jt    pdata
        cmpi  R2, 3
        jt    pzlp
        li    R0, WNAK
        cmpi  R2, 4
        jf    pst
        li    R0, WSTALL
pst:    st    R0, R5
        ret
pzlp:   li    R0, WDATA1     ; status packets are DATA1 (8.5.3)
        stp   R0, R5
        stp   R0, R5
        ldi   R0, 0
        st    R0, R5
        ret
pdata:  ldi   R2, TOG
        ldp   R0, R2
        stp   R0, R5         ; the toggle's PID
        ldp   R0, R2
        stp   R0, R5         ; PTR
        ld    R0, R2         ; LEFT
        ldi   R2, 8
        lt    R0, R2
        jt    pd1
        ldi   R0, 8          ; packets of bMaxPacketSize0, the last one short
pd1:    st    R0, R5
        ret
; sends: R0 the SYNC and PID word; the start lands NRSP cycles after the reference, OE on in the same commit
shs:    txw   R0, 16, stuff, last
        sgo   tx
        oe    DP, 1
        oe    DM, 1
        tick  NRSP
        goto  sdone
szlp:   txw   R0, 16, stuff
        ldi   R0, 0
        txw   R0, 16, stuff, last    ; the CRC-16 of no data (5.9.7, 11.5)
        sgo   tx
        oe    DP, 1
        oe    DM, 1
        tick  NRSP
        goto  sdone
sdata:  li    R2, CODTX      ; R5 the words, R4 the bytes, 1 to 8
        scfg  R2, CODING     ; the CRC register to the send half
        txw   R0, 16, stuff
        sgo   tx
        oe    DP, 1
        oe    DM, 1
        tick  NRSP
sd1:    ldp   R0, R5
        cmpi  R4, 2
        jt    sd16
        cmpi  R4, 1
        jt    sd8
        txw   R0, 16, crc
        subi  R4, 2
        goto  sd1
sd16:   txw   R0, 16, fcs, last
        goto  sdone
sd8:    txw   R0, 8, fcs, last
sdone:  rxw   R0             ; our own packet, received: its last word comes at our EOP
        jf    sdone
        addi  R3, 8          ; A counts it too
        until DM, 1          ; our SE0-to-J
        oe    DP, 0
        oe    DM, 0
        tick  NREL           ; J for NREL + 3 cycles, then high impedance
        li    R0, CODRX
        scfg  R0, CODING     ; the CRC register back to the receive half
        sstat R0             ; clear the self-reception's lost words
        ret
; both receive halves: timed samples on D+, NRZI, hunt for D+ rising, the end test on D- (RCLK)
rxcfg:  li    R0, 0x9870
        scfg  R0, RXMODE     ; RDIFF, HUNT, HLVL 1, RPIN io0, RCLK io1
        ldi   R0, 33
        scfg  R0, RXPER
        li    R0, 0x8155
        scfg  R0, RXOPT      ; RFRAC 85, RESYNC, RGAIN 0, CSKIP 8
        ldi   R0, 16
        scfg  R0, RXFIRST    ; half a bit after each edge
        li    R0, 0x4000
        scfg  R0, RXCNT      ; an 8-bit sync pattern, no bit limit
        li    R0, 0x8000
        scfg  R0, RXSYNC     ; the SYNC field, 00000001, laid out as a received word
        ldi   R0, 0
        scfg  R0, RXEND
        scfg  R0, POLYLO
        scfg  R0, CHKLO
        ret
; ------------------------------------------------------------------------------------------------- A: tokens
tok:    sstop txrx
        call  rxcfg
        li    R0, 0x2800
        scfg  R0, POLYHI     ; CRC-5: x^5 + x^2 + 1, left-aligned
        li    R0, 0x6000
        scfg  R0, CHKHI      ; its residue, 01100 (8.3.5.1)
        li    R0, 0xE4E4
        scfg  R0, CODING
        sgo   rx
        now
        ldi   R3, 0
        ldi   R4, VERD
tl:     rxw   R0
        jf    tl
        srd   R1, end
        sstat R2
        addi  R3, 8
        andi  R2, 31
        cmpi  R2, 8
        jt    t8
        ldi   R1, 0
t8:     or    R1, R3
        st    R1, R4
        goto  tl
; ------------------------------------------------------------------------------------------------- W: resets
rst:    li    R3, 0x0300     ; D+ and D-, both 0: SE0
        li    R2, TDET
        ldi   R4, RSTN
rw:     untilm R3, io, eq
        untilm R3, io, ne, R2
        jt    rw             ; it ended in time: an EOP or a keep-alive
        ld    R0, R4
        addi  R0, 1
        st    R0, R4         ; a reset: S acts on it before the next packet
        untilm R3, io, ne    ; its end
        goto  rw
