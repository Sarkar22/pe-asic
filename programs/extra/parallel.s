; 8-bit parallel capture. Data on io, strobe on ui4. The event snapshot holds all 16 inputs.
.equ STB ui4
.equ MOUT 1
loop:   until STB, 0
        until STB, 1
        rds  R0
        swap R0
        andi R0, 255
        send R0, MOUT
        goto  loop
