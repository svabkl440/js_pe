#!/usr/bin/env python3
"""Assembler for the pin-oriented core.

Usage:  python3 asm.py programs/uart_tx.asm            # prints the words
        python3 asm.py programs/uart_tx.asm -o out.hex  # writes one hex word per line

Syntax (one instruction per line, ';' starts a comment):
    .equ NAME value           name a number (pins, delays, ...)
    label:                    name an address (may share a line with an instruction)
    NOP
    SET   pin, 0|1            drive output pin 0-7
    OUT   pin [, LSB|MSB]     shift one bit out of SR onto output pin (default LSB first)
    IN    pin [, LSB|MSB]     shift one bit from input pin 0-15 into SR (default LSB first)
    PULL                      SR <= ui_in (parallel load)
    PUSH                      all output pins <= SR (parallel write)
    DELAY n                   take exactly n+1 cycles (n = 0-255)
    LDX n / LDY n             load scratch register (0-255)
    WAIT  pin, 0|1            stall until input pin 0-15 equals the value
    JMP   label               always jump
    JMP   X-- label           if X != 0 jump; X is decremented either way (same for Y--)
    JMP   PIN p, label        jump if input pin p is high
"""
import re
import sys

OPS = {"NOP": 0x0, "SET": 0x1, "OUT": 0x2, "IN": 0x3, "PULL": 0x4, "PUSH": 0x5,
       "DELAY": 0x6, "LDX": 0x7, "LDY": 0x8, "WAIT": 0x9, "JMP": 0xA}
DEPTH = 64          # must match DEPTH_LOG2 = 6 in the Verilog


class AsmError(Exception):
    pass


def assemble(text):
    symbols, items = {}, []
    labels = {}

    # ---- pass 1: strip comments, collect labels and .equ, list instructions
    for lineno, raw in enumerate(text.splitlines(), 1):
        line = raw.split(";")[0].strip()
        while ":" in line:
            label, line = line.split(":", 1)
            label = label.strip()
            if not re.fullmatch(r"[A-Za-z_]\w*", label):
                raise AsmError(f"line {lineno}: bad label '{label}'")
            if label in labels or label in symbols:
                raise AsmError(f"line {lineno}: '{label}' defined twice")
            labels[label] = len(items)
            line = line.strip()
        if not line:
            continue
        parts = line.replace(",", " ").split()
        mnem, args = parts[0].upper(), parts[1:]
        if mnem == ".EQU":
            if len(args) != 2:
                raise AsmError(f"line {lineno}: .equ NAME value")
            symbols[args[0]] = int(args[1], 0)
            continue
        items.append((lineno, mnem, args))

    symbols.update(labels)
    if len(items) > DEPTH:
        raise AsmError(f"program has {len(items)} words but memory holds {DEPTH}")

    def val(lineno, tok, lo, hi, what):
        try:
            v = symbols[tok] if tok in symbols else int(tok, 0)
        except ValueError:
            raise AsmError(f"line {lineno}: unknown value '{tok}'")
        if not lo <= v <= hi:
            raise AsmError(f"line {lineno}: {what} {v} out of range {lo}..{hi}")
        return v

    def need(lineno, mnem, args, n):
        if len(args) != n:
            raise AsmError(f"line {lineno}: {mnem} takes {n} operand(s)")

    # ---- pass 2: encode
    words = []
    for lineno, mnem, args in items:
        if mnem not in OPS:
            raise AsmError(f"line {lineno}: unknown instruction '{mnem}'")
        op = OPS[mnem] << 12
        if mnem in ("NOP", "PULL", "PUSH"):
            need(lineno, mnem, args, 0)
            w = op
        elif mnem == "SET":
            need(lineno, mnem, args, 2)
            w = op | (val(lineno, args[1], 0, 1, "value") << 3) \
                   | val(lineno, args[0], 0, 7, "output pin")
        elif mnem in ("OUT", "IN"):
            if len(args) not in (1, 2):
                raise AsmError(f"line {lineno}: {mnem} pin [, LSB|MSB]")
            msb = 0
            if len(args) == 2:
                d = args[1].upper()
                if d not in ("LSB", "MSB"):
                    raise AsmError(f"line {lineno}: direction must be LSB or MSB")
                msb = int(d == "MSB")
            if mnem == "OUT":
                w = op | (msb << 3) | val(lineno, args[0], 0, 7, "output pin")
            else:
                w = op | (msb << 4) | val(lineno, args[0], 0, 15, "input pin")
        elif mnem == "DELAY":
            need(lineno, mnem, args, 1)
            w = op | val(lineno, args[0], 0, 255, "delay")
        elif mnem in ("LDX", "LDY"):
            need(lineno, mnem, args, 1)
            w = op | val(lineno, args[0], 0, 255, "value")
        elif mnem == "WAIT":
            need(lineno, mnem, args, 2)
            w = op | (val(lineno, args[1], 0, 1, "value") << 4) \
                   | val(lineno, args[0], 0, 15, "input pin")
        else:  # JMP
            a0 = args[0].upper() if args else ""
            if a0 in ("X--", "Y--"):
                need(lineno, mnem, args, 2)
                cond, pin, tgt = (1 if a0 == "X--" else 2), 0, args[1]
            elif a0 == "PIN":
                need(lineno, mnem, args, 3)
                cond, pin, tgt = 3, val(lineno, args[1], 0, 15, "input pin"), args[2]
            else:
                need(lineno, mnem, args, 1)
                cond, pin, tgt = 0, 0, args[0]
            w = op | (cond << 10) | (pin << 6) | val(lineno, tgt, 0, DEPTH - 1, "address")
        words.append(w)
    return words


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    with open(sys.argv[1]) as f:
        words = assemble(f.read())
    if "-o" in sys.argv:
        with open(sys.argv[sys.argv.index("-o") + 1], "w") as f:
            f.write("".join(f"{w:04x}\n" for w in words))
    else:
        for i, w in enumerate(words):
            print(f"{i:2d}: {w:04x}")


if __name__ == "__main__":
    try:
        main()
    except AsmError as e:
        sys.exit(f"error: {e}")
