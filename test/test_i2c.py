import math
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

# The instruction memory is an 8-bit synchronous SRAM, so every instruction
# takes exactly CLKS_PER_INSTR clocks (low byte read + high byte read).
CLKS_PER_INSTR = 2

# The I2C firmware is written for a 20 MHz clock (1 slot = 100 ns, SCL = 100 kHz).
# The simulated clock period below is only a label; all I2C timing limits are
# expressed in clocks of that 20 MHz design clock.
F_CLK_MHZ = 20
HEX_NAME = "i2c_master.hex"


def us(x):
    """Microseconds -> design clocks (rounded up)."""
    return math.ceil(round(x * F_CLK_MHZ, 6))


# I2C standard-mode minimums (NXP UM10204), in clocks
T_LOW = us(4.7)      # SCL low
T_HIGH = us(4.0)     # SCL high
T_SU_STA = us(4.7)   # repeated START setup
T_HD_STA = us(4.0)   # START hold
T_SU_STO = us(4.0)   # STOP setup
SCL_PERIOD = us(10)  # never faster than 100 kHz

# ---- uio_in map (host / bus -> chip) ----
STOPF = 1 << 0       # uio_in[0] issue STOP after this byte / NACK last read byte
STARTF = 1 << 1      # uio_in[1] (repeated) START before this byte
READF = 1 << 2       # uio_in[2] receive a byte instead of sending one
GO = 1 << 3          # uio_in[3] pulse to run one byte
SDA_IN = 1 << 4      # uio_in[4] SDA line read-back
SCL_IN = 1 << 5      # uio_in[5] SCL line read-back
LOAD_MODE = 0b1000_0000   # uio_in[7]
LOAD_STROBE = 0b0100_0000 # uio_in[6]

# ---- uo_out map (chip -> world) ----
O_SCL, O_SDA, O_READY, O_NACK, O_RXD, O_RXC = 0, 1, 2, 3, 4, 5

SLAVE_ADDR = 0x50


def bit(dut, n):
    return (int(dut.uo_out.value) >> n) & 1


def ready(dut):
    return bit(dut, O_READY)


def nack(dut):
    return bit(dut, O_NACK)


def read_hex_words():
    """Read the word-per-line I2C firmware image."""
    path = Path(__file__).resolve().parent / HEX_NAME

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

    # Load the I2C master firmware (the image that used to be preloaded
    # by $readmemh).
    await load_words(dut, read_hex_words())


async def wait_for(dut, cond, limit=2000):
    for _ in range(limit):
        if cond():
            return
        await ClockCycles(dut.clk, 1)

    assert False, "timed out waiting for condition"


# ---------------------------------------------------------------------------
# Open-drain I2C bus + slave model
#
# The chip has push-pull outputs, so the board uses non-inverting open-drain
# buffers: uo_out[0] / uo_out[1] = 1 releases SCL / SDA, 0 pulls it low.
# Each line is the wired-AND of the master pin and the slave, and is fed back
# to uio_in[5:4].
# ---------------------------------------------------------------------------
class I2CSlave:
    def __init__(self, addr=SLAVE_ADDR, read_data=(), stretch_clks=0):
        self.addr = addr
        self.read_data = list(read_data)
        self.stretch_clks = stretch_clks   # clocks to hold SCL low after each received byte

        self.events = []        # 'S', 'P', ('addr', a, rw), ('data', b), ('tx', b)
        self.master_acks = []   # master ACK (True) / NACK (False) after each byte it read

        self.sda = 1            # slave drive (1 = released)
        self.scl = 1            # slave clock stretch (1 = released)
        self.release_at = 0

        self.state = "IDLE"     # IDLE RX ACK TX MACK
        self.nbits = 0
        self.shift = 0
        self.first = False
        self.ack = False
        self.rw = 0
        self.txbyte = 0
        self.txbit = 0
        self.master_ack = False
        self.p_scl = None
        self.p_sda = None

    def _load_tx(self):
        self.txbyte = self.read_data.pop(0) if self.read_data else 0xFF
        self.events.append(("tx", self.txbyte))
        self.sda = (self.txbyte >> 7) & 1
        self.txbit = 1
        self.state = "TX"

    def step(self, now, scl, sda):
        """Called once per clock with the resolved line values."""
        if self.p_scl is None:
            self.p_scl, self.p_sda = scl, sda
            return
        p_scl, p_sda = self.p_scl, self.p_sda
        self.p_scl, self.p_sda = scl, sda

        if self.scl == 0 and now >= self.release_at:
            self.scl = 1

        if scl and p_scl and p_sda and not sda:           # START / repeated START
            self.events.append("S")
            self.state, self.nbits, self.shift, self.first = "RX", 0, 0, True
            self.sda = 1
        elif scl and p_scl and not p_sda and sda:         # STOP
            self.events.append("P")
            self.state = "IDLE"
            self.sda = 1
        elif scl and not p_scl:                           # SCL rising: sample
            if self.state == "RX":
                self.shift = ((self.shift << 1) | sda) & 0xFF
                self.nbits += 1
            elif self.state == "MACK":
                self.master_ack = (sda == 0)
                self.master_acks.append(self.master_ack)
        elif not scl and p_scl:                           # SCL falling: drive
            if self.state == "RX" and self.nbits == 8:
                if self.first:
                    a, self.rw = self.shift >> 1, self.shift & 1
                    self.events.append(("addr", a, self.rw))
                    self.ack = (a == self.addr)
                    self.first = False
                else:
                    self.events.append(("data", self.shift))
                    self.ack = True
                self.sda = 0 if self.ack else 1
                self.state = "ACK"
                self.nbits, self.shift = 0, 0
                if self.stretch_clks:
                    self.scl = 0
                    self.release_at = now + self.stretch_clks
            elif self.state == "ACK":
                self.sda = 1
                if not self.ack:
                    self.state = "IDLE"
                elif self.rw:
                    self._load_tx()
                else:
                    self.state = "RX"
            elif self.state == "TX":
                if self.txbit == 8:
                    self.sda = 1
                    self.state = "MACK"
                else:
                    self.sda = (self.txbyte >> (7 - self.txbit)) & 1
                    self.txbit += 1
            elif self.state == "MACK":
                self.sda = 1
                if self.master_ack:
                    self._load_tx()
                else:
                    self.state = "IDLE"


class Bus:
    """Resolves the open-drain lines, feeds them back, and records everything."""

    def __init__(self, dut, slave):
        self.dut = dut
        self.slave = slave
        self.flags = 0          # host-driven uio_in bits (STOPF/STARTF/READF/GO)
        self.t = 0
        self.scl = None
        self.sda = None
        self.init = None        # (scl, sda) when the model started
        self.log = []           # (clk, 'scl'|'sda', value) for every resolved line change
        self.rx_bits = []       # bits captured from RXD on each RXC rising edge

    async def run(self):
        dut = self.dut
        prev_rxc = 0
        while True:
            uo = int(dut.uo_out.value)
            scl = ((uo >> O_SCL) & 1) & self.slave.scl
            sda = ((uo >> O_SDA) & 1) & self.slave.sda

            self.slave.step(self.t, scl, sda)

            if self.scl is None:
                self.init = (scl, sda)
            else:
                if scl != self.scl:
                    self.log.append((self.t, "scl", scl))
                if sda != self.sda:
                    self.log.append((self.t, "sda", sda))
            self.scl, self.sda = scl, sda

            rxc = (uo >> O_RXC) & 1
            if rxc and not prev_rxc:
                self.rx_bits.append((uo >> O_RXD) & 1)
            prev_rxc = rxc

            dut.uio_in.value = self.flags | (sda << 4) | (scl << 5)
            await ClockCycles(dut.clk, 1)
            self.t += 1

    def rx_bytes(self):
        b = self.rx_bits
        assert len(b) % 8 == 0, f"partial serial byte: {len(b)} bits"
        return [int("".join(map(str, b[i:i + 8])), 2) for i in range(0, len(b), 8)]


async def i2c_reset(dut, **slave_kw):
    """Reset, load the I2C firmware, attach the bus model, wait for idle."""
    await reset(dut)
    slave = I2CSlave(**slave_kw)
    bus = Bus(dut, slave)
    cocotb.start_soon(bus.run())
    # Firmware begins with a STOP condition (bus init), then raises READY.
    await wait_for(dut, lambda: ready(dut) == 1, limit=3000)
    slave.events.clear()
    return bus


async def command(dut, bus, byte=0, start=False, stop=False, read=False, limit=12000):
    """
    Run one byte on the bus. Byte and flags are held until READY returns high.
    Returns the number of clocks READY stayed low.
    """
    flags = (STARTF if start else 0) | (STOPF if stop else 0) | (READF if read else 0)
    dut.ui_in.value = byte
    bus.flags = flags
    await ClockCycles(dut.clk, 2 * CLKS_PER_INSTR)      # flags settle first

    # Pins are sampled once per instruction slot, so GO must be held
    # for at least CLKS_PER_INSTR clocks to be guaranteed to be seen.
    bus.flags = flags | GO
    await ClockCycles(dut.clk, CLKS_PER_INSTR)
    bus.flags = flags

    await wait_for(dut, lambda: ready(dut) == 0, limit=50)
    n = 0
    while not ready(dut):
        await ClockCycles(dut.clk, 1)
        n += 1
        assert n < limit, "timed out waiting for READY"
    bus.flags = 0
    return n


def assert_idle(dut, bus):
    assert ready(dut) == 1
    assert bus.scl == 1 and bus.sda == 1, f"bus not idle: SCL={bus.scl} SDA={bus.sda}"


# ---------------------------------------------------------------------------
# I2C tests
# ---------------------------------------------------------------------------
@cocotb.test()
async def bus_idles_released(dut):
    """After firmware load the bus is released (both lines high) and READY is high."""
    bus = await i2c_reset(dut)
    await ClockCycles(dut.clk, 20)

    assert_idle(dut, bus)
    assert "S" not in bus.slave.events, "spurious START during init"


@cocotb.test()
async def write_transaction(dut):
    """START+addr(W), data, data+STOP: slave sees exactly that and ACKs every byte."""
    bus = await i2c_reset(dut)

    await command(dut, bus, (SLAVE_ADDR << 1) | 0, start=True)
    assert nack(dut) == 0, "address was not ACKed"
    await command(dut, bus, 0x12)
    assert nack(dut) == 0
    await command(dut, bus, 0x34, stop=True)
    assert nack(dut) == 0

    assert bus.slave.events == [
        "S", ("addr", SLAVE_ADDR, 0), ("data", 0x12), ("data", 0x34), "P",
    ], bus.slave.events
    assert_idle(dut, bus)


@cocotb.test()
async def every_bit_pattern_is_sent_msb_first(dut):
    """Data bytes with edge-case bit patterns arrive intact, MSB first."""
    bus = await i2c_reset(dut)
    payload = [0x00, 0xFF, 0x55, 0xAA, 0x01, 0x80, 0xA5]

    await command(dut, bus, SLAVE_ADDR << 1, start=True)
    for i, b in enumerate(payload):
        await command(dut, bus, b, stop=(i == len(payload) - 1))
        assert nack(dut) == 0, f"0x{b:02X} not ACKed"

    got = [e[1] for e in bus.slave.events if isinstance(e, tuple) and e[0] == "data"]
    assert got == payload, [f"0x{x:02X}" for x in got]
    assert bus.slave.events[-1] == "P"


@cocotb.test()
async def wrong_address_is_nacked(dut):
    """No slave answers: NACK is reported, and a STOP-only command frees the bus."""
    bus = await i2c_reset(dut)

    await command(dut, bus, (0x51 << 1) | 0, start=True)
    assert nack(dut) == 1, "NACK flag not set for an unanswered address"

    # STARTF + READF = STOP only
    await command(dut, bus, 0, start=True, read=True)

    assert bus.slave.events == ["S", ("addr", 0x51, 0), "P"], bus.slave.events
    assert_idle(dut, bus)


@cocotb.test()
async def read_transaction(dut):
    """START+addr(R), read, read+STOP: bytes come out serially, last byte is NACKed."""
    bus = await i2c_reset(dut, read_data=[0xA5, 0x3C])

    await command(dut, bus, (SLAVE_ADDR << 1) | 1, start=True)
    assert nack(dut) == 0

    await command(dut, bus, read=True)
    assert bus.rx_bytes() == [0xA5], bus.rx_bytes()

    await command(dut, bus, read=True, stop=True)
    assert bus.rx_bytes() == [0xA5, 0x3C], bus.rx_bytes()

    assert bus.slave.master_acks == [True, False], "master must ACK all but the last byte"
    assert bus.slave.events == [
        "S", ("addr", SLAVE_ADDR, 1), ("tx", 0xA5), ("tx", 0x3C), "P",
    ], bus.slave.events
    assert_idle(dut, bus)


@cocotb.test()
async def repeated_start_register_read(dut):
    """Write a register pointer, repeated START, read one byte."""
    bus = await i2c_reset(dut, read_data=[0x5A])

    await command(dut, bus, SLAVE_ADDR << 1, start=True)
    await command(dut, bus, 0x07)
    await command(dut, bus, (SLAVE_ADDR << 1) | 1, start=True)    # repeated START
    assert nack(dut) == 0
    await command(dut, bus, read=True, stop=True)

    assert bus.rx_bytes() == [0x5A]
    assert bus.slave.events == [
        "S", ("addr", SLAVE_ADDR, 0), ("data", 0x07),
        "S", ("addr", SLAVE_ADDR, 1), ("tx", 0x5A), "P",
    ], bus.slave.events
    assert_idle(dut, bus)


@cocotb.test()
async def scl_and_condition_timing(dut):
    """
    SCL never exceeds 100 kHz, low/high times meet the standard-mode minimums,
    and START hold/setup and STOP setup times are respected.
    """
    bus = await i2c_reset(dut)
    bus.log.clear()

    await command(dut, bus, SLAVE_ADDR << 1, start=True)
    await command(dut, bus, 0x55)
    await command(dut, bus, 0xAA, stop=True)

    cur_scl = bus.init[0]
    last_rise = None
    start_t = None
    highs, lows, periods = [], [], []
    prev_edge = None        # (t, value) of the previous SCL edge
    prev_rise = None

    for t, name, v in bus.log:
        if name == "scl":
            if prev_edge is not None:
                dt = t - prev_edge[0]
                if prev_edge[1] == 1:
                    highs.append(dt)
                elif dt < 4 * SCL_PERIOD:        # skip idle gaps between host commands
                    lows.append(dt)
            if v:
                if prev_rise is not None and t - prev_rise < 4 * SCL_PERIOD:
                    periods.append(t - prev_rise)
                prev_rise = t
                last_rise = t
            elif start_t is not None:
                assert t - start_t >= T_HD_STA, f"START hold {t - start_t} < {T_HD_STA} clk"
                start_t = None
            prev_edge = (t, v)
            cur_scl = v
        elif cur_scl:                              # SDA moved while SCL high
            if v == 0:
                assert last_rise is None or t - last_rise >= T_SU_STA, "START setup too short"
                start_t = t
            else:
                assert t - last_rise >= T_SU_STO, f"STOP setup {t - last_rise} < {T_SU_STO} clk"

    assert len(highs) >= 27 and len(lows) >= 27, (len(highs), len(lows))   # 3 bytes x 9 clocks
    assert min(highs) >= T_HIGH, f"SCL high {min(highs)} clk < {T_HIGH}"
    assert min(lows) >= T_LOW, f"SCL low {min(lows)} clk < {T_LOW}"
    assert min(periods) >= SCL_PERIOD, f"SCL period {min(periods)} clk < {SCL_PERIOD}"
    assert max(periods) <= SCL_PERIOD + 8 * CLKS_PER_INSTR, f"SCL period {max(periods)} clk"


@cocotb.test()
async def ready_marks_the_byte(dut):
    """READY stays low for the whole (START + byte) frame and then returns high."""
    bus = await i2c_reset(dut)

    n = await command(dut, bus, SLAVE_ADDR << 1, start=True)

    # START (~102 slots) + 8 bits (800) + ACK (~101) + tail = ~1007 slots
    expect = 1007 * CLKS_PER_INSTR
    assert abs(n - expect) <= 10 * CLKS_PER_INSTR, f"frame took {n} clocks, expected ~{expect}"


@cocotb.test()
async def clock_stretching(dut):
    """A slave that stretches SCL after every byte does not corrupt the transfer."""
    bus = await i2c_reset(dut, stretch_clks=800)

    await command(dut, bus, SLAVE_ADDR << 1, start=True)
    assert nack(dut) == 0
    await command(dut, bus, 0x3C, stop=True)
    assert nack(dut) == 0

    assert bus.slave.events == [
        "S", ("addr", SLAVE_ADDR, 0), ("data", 0x3C), "P",
    ], bus.slave.events
    assert_idle(dut, bus)


# ---------------------------------------------------------------------------
# ISA / loader tests (firmware independent, unchanged from the UART version)
# ---------------------------------------------------------------------------
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