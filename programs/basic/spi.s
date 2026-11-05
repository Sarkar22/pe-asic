; SPI controller, MSB first, any mode. CPOL and CPHA are constants at assembly.
; mailbox MCFG: half period in cycles. mailbox MIN: low byte goes out, bit 8 set
; on the last byte of a frame. mailbox MOUT: the byte read back in the low byte.
.equ SCK uo1
.equ MOSI uo2
.equ CS uo5
.equ MISO ui4
.equ CPOL 0
.equ CPHA 0
.equ MCFG 7
.equ MIN 1
.equ MOUT 5
start:  pcfg  SCK, PP
        pcfg  MOSI, PP
        pcfg  CS, PP
        put   CS, 1
.if CPOL
        put   SCK, 1
.endif
        now
        recv  R2, MCFG
next:   recv  R0, MIN
        ldi   R1, 8
        put   CS, 0
        now                 ; D = here, whatever the wait was
bit:    shl8  R0, MISO, MOSI    ; take the last sample, prepare the next bit
.if CPOL ^ CPHA
        edge  SCK, 1, R2
        edge  SCK, 0, R2
.else
        edge  SCK, 0, R2
        edge  SCK, 1, R2
.endif
        djnz  R1, bit
        shl8  R0, MISO, -       ; the last sample
.if CPOL
        edge  SCK, 1, R2        ; back to idle
.else
        edge  SCK, 0, R2
.endif
        bt    R0, 8             ; T = last byte of the frame
        edge  CS, T, R2         ; CS rises only then; no branch, so half period 9 holds
        send  R0, MOUT
        goto  next
