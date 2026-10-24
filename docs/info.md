<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

A tiny pin-oriented CPU (see `src/isa.txt`) executes firmware that lives in a foundry IHP SG13G2
`RM_IHPSG13_1P_1024x8_c2_bm_bist` SRAM macro (8 bits wide, 1024 bytes, synchronous read). Only the first
128 bytes are used: 64 instructions x 16 bits, stored low byte at the even address, high byte at the odd address.

Because the macro is 8 bits wide, each instruction is fetched with two byte reads, so **every instruction
takes exactly 2 clocks** (one "slot"). The next instruction's address is computed in the same cycle the
current one executes, so taken jumps, `DELAY` and `WAIT` all keep this fixed cost.

The SRAM has no initial contents. Firmware is loaded at run time:

- `uio[7]` LOAD_MODE = 1 holds the CPU and enables the loader.
- `uio[6]` LOAD_STROBE: each rising edge writes the byte on `ui[7:0]` to the next SRAM address
  (low byte first, 128 bytes total).
- LOAD_MODE = 0 starts execution at instruction 0.

## How to test

Run `make` in `test/` (cocotb + Icarus; uses the behavioural SRAM model in `test/models`).
Load `test/uart_tx.hex`, then put a byte on `ui[7:0]` and hold START (`uio[3]`) high for at least 2 clocks.
TX is `uo[0]`, READY is `uo[1]`.

## External hardware

None required.
