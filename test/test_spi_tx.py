
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer


# One instruction slot = 2 hardware clocks.
CLKS_PER_INSTR = 2

# With DELAY 9 on both sides of the SCLK low phase:
# 10 DELAY slots + SET SCLK low + JMP + OUT
# + 10 DELAY slots + SET SCLK high = 24 slots.
CLKS_PER_SPI_BIT = 24 * CLKS_PER_INSTR

MOSI_BIT = 0
SCLK_BIT = 1
CS_BIT = 2
READY_BIT = 3

START = 0b0000_1000       # Logical input pin 11 = uio_in[3]
LOAD_MODE = 0b1000_0000   # uio_in[7]
LOAD_STROBE = 0b0100_0000 # uio_in[6]


def pin(dut, bit):
    return (int(dut.uo_out.value) >> bit) & 1


def mosi(dut):
    return pin(dut, MOSI_BIT)


def sclk(dut):
    return pin(dut, SCLK_BIT)


def cs_n(dut):
    return pin(dut, CS_BIT)


def ready(dut):
    return pin(dut, READY_BIT)


def read_hex_words():
    path = Path(__file__).resolve().parent / "spi_tx.hex"
    words = []

    for line in path.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if line:
            words.append(int(line, 16))

    assert words, f"No firmware words found in {path}"
    assert len(words) <= 64, "Firmware exceeds 64 instructions"
    return words


async def tick_and_settle(dut, cycles=1):
    """Wait whole clock cycles, then sample settled synchronous outputs."""
    await ClockCycles(dut.clk, cycles)
    await Timer(1, unit="ns")


async def load_words(dut, words):
    """Load up to 64 instructions using the runtime byte loader."""
    assert len(words) <= 64, "Program exceeds instruction memory"

    image = list(words) + [0x0000] * (64 - len(words))

    dut.uio_in.value = LOAD_MODE
    await tick_and_settle(dut, 5)

    for word in image:
        assert 0 <= word <= 0xFFFF

        # SRAM stores the low byte first, then the high byte.
        for byte in (word & 0xFF, (word >> 8) & 0xFF):
            dut.ui_in.value = byte
            dut.uio_in.value = LOAD_MODE
            await tick_and_settle(dut, 3)

            dut.uio_in.value = LOAD_MODE | LOAD_STROBE
            await tick_and_settle(dut, 3)

            dut.uio_in.value = LOAD_MODE
            await tick_and_settle(dut, 3)

    # Leave loader mode and allow the CPU to begin executing.
    dut.uio_in.value = 0
    await tick_and_settle(dut, 10)


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
    await tick_and_settle(dut, 5)

    await load_words(dut, read_hex_words())


async def wait_for(dut, condition, limit=5000, message="condition"):
    """Wait for a condition while sampling after each settled clock edge."""
    for _ in range(limit):
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")

        if condition():
            return

    assert condition(), (
        f"Timeout waiting for {message}; "
        f"READY={ready(dut)}, CS_N={cs_n(dut)}, "
        f"SCLK={sclk(dut)}, MOSI={mosi(dut)}"
    )


async def send(dut, byte):
    """Start a transfer and hold START until READY acknowledges it."""
    await wait_for(
        dut,
        lambda: ready(dut) == 1,
        message="READY high before transfer",
    )

    # Set the byte before requesting the transfer.
    dut.ui_in.value = byte
    dut.uio_in.value = START

    # WAIT START,1 should advance; firmware then lowers READY.
    await wait_for(
        dut,
        lambda: ready(dut) == 0,
        message="READY low after START",
    )

    # Release START only after the request has been acknowledged.
    dut.uio_in.value = 0


async def collect_spi(dut, byte, timeout=5000):
    """Capture MOSI on SCLK rising edges while CS_N is asserted."""
    bits = []
    edge_cycles = []
    cycle = 0
    previous_sclk = sclk(dut)

    for _ in range(timeout):
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        cycle += 1

        current_sclk = sclk(dut)

        if (
            cs_n(dut) == 0
            and previous_sclk == 0
            and current_sclk == 1
        ):
            bits.append(mosi(dut))
            edge_cycles.append(cycle)

        previous_sclk = current_sclk

        if len(bits) == 8 and cs_n(dut) == 1:
            break

    assert len(bits) == 8, (
        f"Expected 8 SPI rising edges, got {len(bits)}: {bits}; "
        f"READY={ready(dut)}, CS_N={cs_n(dut)}, "
        f"SCLK={sclk(dut)}, MOSI={mosi(dut)}"
    )

    expected = [(byte >> i) & 1 for i in range(7, -1, -1)]
    assert bits == expected, (
        f"SPI data mismatch for 0x{byte:02X}: "
        f"got {bits}, expected {expected}"
    )

    return bits, edge_cycles


@cocotb.test()
async def spi_idle_levels(dut):
    await reset(dut)
    await tick_and_settle(dut, 20)

    assert sclk(dut) == 0, "SCLK should idle low"
    assert cs_n(dut) == 1, "CS_N should idle high"
    assert ready(dut) == 1, "READY should idle high"


@cocotb.test()
async def spi_transmits_msb_first(dut):
    await reset(dut)

    for byte in [0xA5, 0x55, 0x00, 0xFF, 0x80, 0x01, 0x42]:
        await send(dut, byte)

        await collect_spi(dut, byte)

        await wait_for(
            dut,
            lambda: ready(dut) == 1,
            message="READY high after transfer",
        )

        assert sclk(dut) == 0, "SCLK should return low"
        assert cs_n(dut) == 1, "CS_N should return high"


@cocotb.test()
async def spi_clock_timing(dut):
    await reset(dut)
    await send(dut, 0xA5)

    _, edge_cycles = await collect_spi(dut, 0xA5)

    periods = [
        later - earlier
        for earlier, later in zip(edge_cycles, edge_cycles[1:])
    ]

    assert len(periods) == 7, (
        f"Expected 7 rising-edge periods, got {periods}"
    )

    assert all(period == CLKS_PER_SPI_BIT for period in periods), (
        f"Unexpected rising-edge periods {periods}; "
        f"expected {CLKS_PER_SPI_BIT} clocks"
    )


@cocotb.test()
async def spi_chip_select_and_ready(dut):
    await reset(dut)
    await send(dut, 0x96)

    # READY goes low before the following SET CS_N,0 instruction executes.
    await wait_for(
        dut,
        lambda: cs_n(dut) == 0,
        message="CS_N low during transfer",
    )

    assert ready(dut) == 0, "READY should be low during transfer"

    await wait_for(
        dut,
        lambda: ready(dut) == 1,
        message="READY high after transfer",
    )

    assert cs_n(dut) == 1, "CS_N should be high after transfer"
    assert sclk(dut) == 0, "SCLK should finish low"


@cocotb.test()
async def spi_runtime_loader_replaces_program(dut):
    await reset(dut)

    program_a = [
        0x1000,  # SET pin 0, 0
        0x1008,  # SET pin 0, 1
        0xA000,  # JMP 0
    ]
    await load_words(dut, program_a)

    seen0 = set()
    for _ in range(30):
        await tick_and_settle(dut)
        seen0.add(pin(dut, 0))

    assert seen0 == {0, 1}, (
        f"Program A did not toggle pin 0: {seen0}"
    )

    program_b = [
        0x1001,  # SET pin 1, 0
        0x1009,  # SET pin 1, 1
        0xA000,  # JMP 0
    ]
    await load_words(dut, program_b)

    seen1 = set()
    for _ in range(30):
        await tick_and_settle(dut)
        seen1.add(pin(dut, 1))

    assert seen1 == {0, 1}, (
        f"Program B did not toggle pin 1: {seen1}"
    )
