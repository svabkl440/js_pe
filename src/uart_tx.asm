; UART transmitter, 8N1, LSB first. One bit = 87 instruction slots = 174 clk
; (each slot is 2 clk, see isa.txt), i.e. 115200 baud needs a 20 MHz clk.
;
; Host interface:  put the byte on ui_in, then pulse START (uio_in[3]) high for 2+ clk (pins are sampled once per 2-clk slot).
;                  Hold the byte steady until READY drops.
; Outputs:         TX = uo_out[0]   READY = uo_out[1] (low while a frame is in flight)
;
; Timing rule: every bit must take exactly 87 slots, so each bit loop adds up to 87:
;   OUT (1) + DELAY 84 (85) + JMP (1) = 87

.equ TX     0
.equ READY  1
.equ START  11          ; input pin 11 = uio_in[3]

        SET   TX, 1             ; line idles high
idle:   WAIT  START, 1          ; block until the host asks for a byte
        PULL                    ; SR <= ui_in
        SET   READY, 0
        SET   TX, 0             ; start bit
        DELAY 84                ;   1 + 85 + 1 (the LDX below) = 87 cycles
        LDX   7                 ; 8 bits -> X counts 7..0
bit:    OUT   TX                ; data bit, LSB first
        DELAY 84
        JMP   X-- bit           ; 1 + 85 + 1 = 87 cycles per bit
        SET   TX, 1             ; stop bit
        DELAY 84                ;   1 + 85 + 1 = 87 cycles
        SET   READY, 1
        JMP   idle
