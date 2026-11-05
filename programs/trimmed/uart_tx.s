; UART TX 8N1, trimmed for the shared 128-word image. Same behaviour as standard/uart_tx.s.
; The stop bit is shifted out as a ninth data bit. MCFG: cycles per bit. MIN: bytes to send.
.equ TX uo0
.equ MCFG 7
.equ MIN 0
start:  pcfg  TX, PP
        put   TX, 1
        now
        recv  R2, MCFG
byte:   recv  R0, MIN
        lui   R0, 1         ; stop bit above the data
        ldi   R1, 9
        put   TX, 0
        now
bit:    shr16 R0, -, TX
        tick  R2
        djnz  R1, bit
        tick  R2            ; the stop bit lasts one full bit time
        goto  byte
