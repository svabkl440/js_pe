
; SPI master transmitter
; Mode 0, MSB first, 8-bit transfer
;
; Inputs:
;   ui_in[7:0] = byte to transmit
;   input pin 11 = START
;
; Outputs:
;   output pin 0 = MOSI
;   output pin 1 = SCLK
;   output pin 2 = CS_N
;   output pin 3 = READY
;
; Each instruction slot = 2 clocks.
; Each SCLK half-period = 10 instruction slots = 20 clocks.
; This is a basic software-timed SPI implementation.

.equ MOSI   0
.equ SCLK   1
.equ CS_N   2
.equ READY  3
.equ START  11

        SET   SCLK, 0
        SET   CS_N, 1
        SET   MOSI, 0
        SET   READY, 1

idle:   WAIT  START, 1

        PULL                  ; SR <= ui_in
        SET   READY, 0
        SET   CS_N, 0         ; select slave

        LDX   7               ; eight bits, MSB first

bit:    OUT   MOSI, MSB       ; set MOSI to next data bit
        DELAY 9
        SET   SCLK, 1         ; rising edge: slave samples MOSI
        DELAY 9
        SET   SCLK, 0         ; falling edge
        JMP   X-- bit

        SET   CS_N, 1         ; deselect slave
        SET   READY, 1
        JMP   idle
