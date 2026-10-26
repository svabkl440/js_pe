; I2C master (standard mode, ~100 kHz), byte-level command interface.
; Same ISA as uart_tx.asm. One slot = 2 clk; 20 MHz clk -> 1 slot = 100 ns.
; SCL period = 100 slots = 10 us (low 50 slots, high 50 slots).
;
; EXTERNAL HARDWARE (required, the ISA has no tri-state / open-drain):
;   uo_out[0] -> SCL driver, uo_out[1] -> SDA driver. Each goes through an
;   open-drain NON-inverting buffer (e.g. 74LVC1G07): pin=0 pulls the line low,
;   pin=1 releases it. External pull-ups (e.g. 4.7k) on SCL and SDA.
;   The bus lines are read back on SDA_IN / SCL_IN (uio_in[4] / uio_in[5]).
;
; Pins:
;   Outputs: SCL=uo_out[0]  SDA=uo_out[1]  READY=uo_out[2]  NACK=uo_out[3]
;            RXD=uo_out[4]  RXC=uo_out[5]  (serial read-data, MSB first, sample RXD on RXC rising edge)
;   Inputs:  ui_in[7:0] = byte to send (full 8 bits, host formats address<<1|R/W)
;            GO=uio_in[3]  STOPF=uio_in[0]  STARTF=uio_in[1]  READF=uio_in[2]
;            SDA_IN=uio_in[4]  SCL_IN=uio_in[5]   (uio_in[7:6] are used by the program loader)
;
; Host interface: one GO pulse (2+ clk) = one byte on the bus.
;   Put byte on ui_in and flags on STARTF/STOPF/READF, pulse GO.
;   HOLD byte and flags steady until READY returns high (flags are sampled late).
;   STARTF=1            : (repeated) START, then send ui_in byte (address byte)
;   STARTF=0, READF=0   : send ui_in byte, ACK bit is reported on NACK (1 = no ACK)
;   READF=1, STARTF=0   : receive one byte, master ACKs it; if STOPF=1 master NACKs (last byte)
;   STOPF=1             : issue STOP after this byte
;   STARTF=1 + READF=1  : STOP only (valid only inside a transaction, i.e. after a byte)
; Typical write : [START, 0xA0] [no flags, reg] [STOP, data]
; Typical read  : [START, 0xA1] [READ] [READ+STOP]  -> bytes appear serially on RXD/RXC

.equ SCL     0
.equ SDA     1
.equ READY   2
.equ NACK    3
.equ RXD     4
.equ RXC     5
.equ STOPF   8
.equ STARTF  9
.equ READF   10
.equ GO      11
.equ SDA_IN  12
.equ SCL_IN  13

; ---- STOP condition. Also runs once after reset as bus init (pins reset low) ----
stop:   SET   SDA, 0            ; SCL is low here
        DELAY 46                ; 47 slots
        SET   SCL, 1            ; release SCL
        WAIT  SCL_IN, 1         ; honour clock stretching
        DELAY 46                ; tSU;STO
        SET   SDA, 1            ; SDA rises while SCL high = STOP
        DELAY 46                ; tBUF, falls through into idle

idle:   SET   READY, 1
        WAIT  GO, 1
        PULL                    ; SR <= ui_in
        SET   READY, 0
        JMP   PIN STARTF, gstart
        JMP   PIN READF, rd

; ---- write one byte, MSB first, then read ACK ----
wr:     LDX   7
wbit:   OUT   SDA, MSB          ; SR[7] -> SDA, shift left
        DELAY 46                ; low phase: 1+1+47+1 = 50 slots
        SET   SCL, 1
        WAIT  SCL_IN, 1
        DELAY 47                ; high phase: 1+48+1 = 50 slots
        SET   SCL, 0
        JMP   X-- wbit

        SET   SDA, 1            ; release SDA for slave ACK
        DELAY 46
        SET   SCL, 1
        WAIT  SCL_IN, 1
        DELAY 46                ; sample at end of high phase
        SET   NACK, 1           ; assume NACK
        JMP   PIN SDA_IN, ackd  ; SDA high -> NACK stays 1
        SET   NACK, 0           ; SDA low -> ACK
ackd:   SET   SCL, 0
fin:    JMP   PIN STOPF, stop
        JMP   idle

; ---- (repeated) START, then send the byte ----
gstart: JMP   PIN READF, stop   ; START+READ = STOP-only command
        SET   SDA, 1
        SET   SCL, 1
        WAIT  SCL_IN, 1
        DELAY 46                ; tSU;STA
        SET   SDA, 0            ; START
        DELAY 46                ; tHD;STA
        SET   SCL, 0
        JMP   wr

; ---- read one byte, MSB first, master ACK/NACK ----
rd:     LDX   7
        SET   SDA, 1            ; release SDA so slave can drive
rbit:   DELAY 46                ; low phase
        SET   SCL, 1
        WAIT  SCL_IN, 1
        DELAY 47
        IN    SDA_IN, MSB       ; sample, shift into SR from the bottom
        SET   SCL, 0
        JMP   X-- rbit

        JMP   PIN STOPF, rack   ; last byte -> leave SDA released = NACK
        SET   SDA, 0            ; ACK
rack:   DELAY 46
        SET   SCL, 1
        WAIT  SCL_IN, 1
        DELAY 47
        SET   SCL, 0

        LDX   7                 ; shift received byte out serially
emit:   OUT   RXD, MSB
        SET   RXC, 1
        SET   RXC, 0
        JMP   X-- emit
        JMP   fin
