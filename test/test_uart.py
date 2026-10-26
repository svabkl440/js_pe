
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

# The instruction memory is an 8-bit synchronous SRAM, so every instruction
# takes exactly CLKS_PER_INSTR clocks (low byte read + high byte read).
CLKS_PER_INSTR = 2
CLKS_PER_BIT = 87 * CLKS_PER_INSTR   # 87 instruction slots per UART bit
START = 0b0000_1000  # uio_in[3], existing UART start input

LOAD_MODE = 0b1000_0000   # uio_in[7]
LOAD_STROBE = 0b0100_0000 # uio_in[6]


def tx(dut):
    return int(dut.uo_out.value) & 1


def ready(dut):
    return (int(dut.uo_out.value) >> 1) & 1


def read_hex_words():
    """Read the existing word-per-line UART firmware image."""
    path = Path(__file__).resolve().parent / "uart_tx.hex"

    words = []
    for line in path.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if line:
            words.append(int(line, 16))

    assert len(words) <= 64, "Firmware exceeds 64 instructions"
    return words


async def load_words(dut, words):
    """
    Load a complete 64-word program.
    Each instruction is transmitted low byte first, then high byte.
    """

    assert len(words) <= 64, "Program exceeds instruction memory"

    # Pad to the full 128-byte memory image.
    image = list(words) + [0x0000] * (64 - len(words))

    # Enter loader mode and allow input synchronizers to settle.
    dut.uio_in.value = LOAD_MODE
    await ClockCycles(dut.clk, 5)

    for word in image:
        assert 0 <= word <= 0xFFFF

        for byte in (word & 0xFF, (word >> 8) & 0xFF):
            # Keep the byte stable while strobe is low.
            dut.ui_in.value = byte
            dut.uio_in.value = LOAD_MODE
            await ClockCycles(dut.clk, 3)

            # Raise strobe to request one byte write.
            dut.uio_in.value = LOAD_MODE | LOAD_STROBE
            await ClockCycles(dut.clk, 3)

            # Lower strobe before the next byte.
            dut.uio_in.value = LOAD_MODE
            await ClockCycles(dut.clk, 3)

    # Leave loader mode. Execution starts at instruction zero.
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 5)


async def reset(dut):
    try:
        clk = Clock(dut.clk, 100, unit="ns")
    except TypeError:
        clk = Clock(dut.clk, 100, units="ns")

    cocotb.start_soon(clk.start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = LOAD_MODE
    dut.rst_n.value = 0

    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    # Load the same UART program the previous tests expected
    # to be preloaded by $readmemh.
    await load_words(dut, read_hex_words())


async def wait_for(dut, cond, limit=2000):
    for _ in range(limit):
        if cond():
            return
        await ClockCycles(dut.clk, 1)

    assert False, "timed out waiting for condition"


async def send(dut, byte):
    # Pins are sampled once per instruction slot, so START must be held
    # for at least CLKS_PER_INSTR clocks to be guaranteed to be seen.
    dut.ui_in.value = byte
    dut.uio_in.value = START
    await ClockCycles(dut.clk, CLKS_PER_INSTR)
    dut.uio_in.value = 0


@cocotb.test()
async def idles_high(dut):
    """After firmware load, TX and READY are high while idle."""
    await reset(dut)
    await ClockCycles(dut.clk, 20)

    assert tx(dut) == 1
    assert ready(dut) == 1


@cocotb.test()
async def exact_bit_timing(dut):
    """Every UART bit lasts exactly 87 clocks."""
    await reset(dut)

    for byte in [0x55, 0x00, 0xFF, 0x01, 0x80, 0xA5]:
        await send(dut, byte)
        await wait_for(dut, lambda: tx(dut) == 0)

        samples = [tx(dut)]
        for _ in range(10 * CLKS_PER_BIT - 1):
            await ClockCycles(dut.clk, 1)
            samples.append(tx(dut))

        expect = [0] + [(byte >> i) & 1 for i in range(8)] + [1]

        for k, bit in enumerate(expect):
            chunk = samples[k * CLKS_PER_BIT:(k + 1) * CLKS_PER_BIT]
            assert chunk == [bit] * CLKS_PER_BIT, (
                f"0x{byte:02X}, bit {k} wrong"
            )

        await wait_for(dut, lambda: ready(dut) == 1)


@cocotb.test()
async def ready_marks_the_frame(dut):
    """READY remains low during a UART frame and returns high."""
    await reset(dut)

    await send(dut, 0x42)
    await wait_for(dut, lambda: ready(dut) == 0)

    n = 0
    while not ready(dut):
        await ClockCycles(dut.clk, 1)
        n += 1

    assert abs(n - 10 * CLKS_PER_BIT) <= CLKS_PER_INSTR, (
        f"frame took {n} clocks"
    )


@cocotb.test()
async def runtime_loader_replaces_program(dut):
    """
    Load program A, observe pin 0 toggling, then overwrite memory
    with program B and observe pin 1 toggling. No RTL rebuild.
    """
    await reset(dut)

    # Program A:
    #   SET pin0, 0
    #   SET pin0, 1
    #   JMP 0
    program_a = [
        0x1000,
        0x1008,
        0xA000,
    ]

    await load_words(dut, program_a)

    seen_pin0 = set()
    for _ in range(20):
        await ClockCycles(dut.clk, 1)
        seen_pin0.add(int(dut.uo_out.value) & 0x01)

    assert seen_pin0 == {0, 1}, (
        f"Program A did not toggle pin 0: {seen_pin0}"
    )

    # Program B:
    #   SET pin1, 0
    #   SET pin1, 1
    #   JMP 0
    program_b = [
        0x1001,
        0x1009,
        0xA000,
    ]

    await load_words(dut, program_b)

    seen_pin1 = set()
    for _ in range(20):
        await ClockCycles(dut.clk, 1)
        seen_pin1.add((int(dut.uo_out.value) >> 1) & 0x01)

    assert seen_pin1 == {0, 1}, (
        f"Program B did not toggle pin 1: {seen_pin1}"
    )

    dut._log.info(
        "PASS: loaded program A, replaced it with program B, "
        "and observed both output pins toggle"
    )


async def edges(dut, n_clks, bit=0):
    """Return the clock index of each change on uo_out[bit]."""
    prev = (int(dut.uo_out.value) >> bit) & 1
    out = []
    for i in range(n_clks):
        await ClockCycles(dut.clk, 1)
        cur = (int(dut.uo_out.value) >> bit) & 1
        if cur != prev:
            out.append(i)
        prev = cur
    return out


@cocotb.test()
async def two_clocks_per_instruction(dut):
    """
    SET 0,0 / SET 0,1 / JMP 0 is a 3-instruction loop, so pin 0 must
    toggle every 3 instruction slots = 6 clocks, with the taken JMP costing
    the same as any other instruction (no fetch bubble).
    """
    await reset(dut)
    await load_words(dut, [0x1000, 0x1008, 0xA000])

    e = await edges(dut, 60)
    assert len(e) >= 8, f"pin did not toggle: {e}"
    # Low phase = 1 slot, high phase = 2 slots; compare same-polarity edges.
    periods = {b - a for a, b in zip(e, e[2:])}
    assert periods == {3 * CLKS_PER_INSTR}, f"edge periods {periods}"


@cocotb.test()
async def delay_and_loop_counts(dut):
    """
    LDX 3 / loop: SET 0,1 / SET 0,0 / DELAY 2 / JMP X-- loop / SET 1,1 / JMP .
    Checks DELAY n == n+1 slots and that JMP X-- runs the body X+1 times.
    """
    await reset(dut)
    prog = [
        0x7003,  # 0: LDX 3
        0x1008,  # 1: SET 0,1
        0x1000,  # 2: SET 0,0
        0x6002,  # 3: DELAY 2   (3 slots)
        0xA401,  # 4: JMP X-- 1
        0x1009,  # 5: SET 1,1
        0xA006,  # 6: JMP 6
    ]
    await load_words(dut, prog)

    pulses = 0
    prev = 0
    for _ in range(400):
        await ClockCycles(dut.clk, 1)
        v = int(dut.uo_out.value)
        if (v & 1) and not prev:
            pulses += 1
        prev = v & 1
    assert pulses == 4, f"loop body ran {pulses} times, expected 4"
    assert (int(dut.uo_out.value) >> 1) & 1 == 1, "never reached SET 1,1"

    # Slot accounting: SET(1) + SET(1) + DELAY2(3) + JMP(1) = 6 slots / iter,
    # measured between two consecutive falling edges of pin 0.
    await load_words(dut, prog)
    e = await edges(dut, 80)
    assert e[2] - e[0] == 6 * CLKS_PER_INSTR, f"iteration took {e[2]-e[0]} clks"


@cocotb.test()
async def branch_targets_cross_the_whole_memory(dut):
    """Fetch all 64 instruction slots and a jump to the last one (addr 63)."""
    await reset(dut)
    prog = [0xA000 | 63] + [0x0000] * 62 + [0x1008, 0xA03F]  # 0: JMP 63
    prog = prog[:64]
    prog[63] = 0x1008        # 63: SET 0,1
    await load_words(dut, prog)
    await ClockCycles(dut.clk, 20)
    assert int(dut.uo_out.value) & 1 == 1, "did not execute word 63"
