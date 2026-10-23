
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

CLKS_PER_BIT = 87
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
    dut.ui_in.value = byte
    dut.uio_in.value = START
    await ClockCycles(dut.clk, 1)
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

    assert abs(n - 10 * CLKS_PER_BIT) <= 2, (
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
