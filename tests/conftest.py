"""Fake I2C/SPI buses standing in for real hardware in host-side tests."""

from typing import TYPE_CHECKING, cast

import pytest

from adxl36x import (
    _DEVID_AD,
    _DEVID_MST,
    _PART_ID,
    _REG_I2C_FIFO_DATA,
    _REVID_ADXL366,
    _SPI_READ_FIFO,
    _SPI_READ_REG,
    _SPI_WRITE_REG,
    ADXL366,
    ADXL367,
)

if TYPE_CHECKING:
    from busio import I2C, SPI
    from digitalio import DigitalInOut

_REGISTER_SPACE = 0x60


def _seeded_registers(revid: int) -> bytearray:
    registers = bytearray(_REGISTER_SPACE)
    registers[0x00] = _DEVID_AD
    registers[0x01] = _DEVID_MST
    registers[0x02] = _PART_ID
    registers[0x03] = revid
    return registers


class FakeI2C:
    """Duck-typed stand-in for `busio.I2C`, backed by an in-memory register file."""

    def __init__(self, *, revid: int = _REVID_ADXL366) -> None:
        self.registers = _seeded_registers(revid)
        self.fifo_queue = bytearray()
        self.writes: list[tuple[int, int]] = []

    def try_lock(self) -> bool:
        return True

    def unlock(self) -> None:
        pass

    def writeto(
        self,
        _address: int,
        buffer: bytes,
        *,
        start: int = 0,
        end: int | None = None,
    ) -> None:
        data = bytes(buffer[start : len(buffer) if end is None else end])
        if not data:
            return  # I2CDevice probe write
        register, value = data
        self.registers[register] = value
        self.writes.append((register, value))

    def readfrom_into(self, *_args: object, **_kwargs: object) -> None:
        msg = "bare readfrom_into is not used by this driver"
        raise AssertionError(msg)

    def writeto_then_readfrom(
        self,
        _address: int,
        out_buffer: bytes,
        in_buffer: bytearray,
        *,
        out_start: int = 0,
        out_end: int | None = None,
        in_start: int = 0,
        in_end: int | None = None,
    ) -> None:
        out_data = bytes(out_buffer[out_start : len(out_buffer) if out_end is None else out_end])
        register = out_data[0]
        count = (len(in_buffer) if in_end is None else in_end) - in_start
        if register == _REG_I2C_FIFO_DATA:
            chunk = bytes(self.fifo_queue[:count])
            del self.fifo_queue[:count]
        else:
            chunk = bytes(self.registers[register : register + count])
        in_buffer[in_start : in_start + count] = chunk


class FakeSPI:
    """Duck-typed stand-in for `busio.SPI`, backed by an in-memory register file."""

    def __init__(self, *, revid: int = _REVID_ADXL366) -> None:
        self.registers = _seeded_registers(revid)
        self.fifo_queue = bytearray()
        self._pending_read: tuple[str, int] | None = None

    def try_lock(self) -> bool:
        return True

    def unlock(self) -> None:
        pass

    def configure(self, *, baudrate: int, polarity: int, phase: int) -> None:
        pass

    def write(self, buffer: bytes, *, start: int = 0, end: int | None = None) -> None:
        data = bytes(buffer[start : len(buffer) if end is None else end])
        if data[0] == _SPI_WRITE_REG:
            self.registers[data[1]] = data[2]
        elif data[0] == _SPI_READ_REG:
            self._pending_read = ("register", data[1])
        elif data[0] == _SPI_READ_FIFO:
            self._pending_read = ("fifo", 0)
        else:
            msg = f"unexpected SPI command byte: {data[0]!r}"
            raise AssertionError(msg)

    def readinto(self, buffer: bytearray, *, start: int = 0, end: int | None = None) -> None:
        count = (len(buffer) if end is None else end) - start
        assert self._pending_read is not None
        kind, register = self._pending_read
        if kind == "fifo":
            chunk = bytes(self.fifo_queue[:count])
            del self.fifo_queue[:count]
        else:
            chunk = bytes(self.registers[register : register + count])
        buffer[start : start + count] = chunk
        self._pending_read = None


class FakeDigitalInOut:
    """Duck-typed stand-in for `digitalio.DigitalInOut`."""

    def __init__(self) -> None:
        self.value = False

    def switch_to_output(self, *, value: bool = False) -> None:
        self.value = value


def new_adxl366(fake: FakeI2C) -> ADXL366:
    """Construct an ADXL366 against a fake I2C bus, satisfying the type checker."""
    return ADXL366(cast("I2C", fake))


def new_adxl367(fake: FakeI2C) -> ADXL367:
    """Construct an ADXL367 against a fake I2C bus, satisfying the type checker."""
    return ADXL367(cast("I2C", fake))


def new_adxl366_spi(spi: FakeSPI, cs: FakeDigitalInOut) -> ADXL366:
    """Construct an ADXL366 over a fake SPI bus, satisfying the type checker."""
    return ADXL366.from_spi(cast("SPI", spi), cast("DigitalInOut", cs))


@pytest.fixture
def fake_i2c() -> FakeI2C:
    return FakeI2C()


@pytest.fixture
def adxl366(fake_i2c: FakeI2C) -> ADXL366:
    return new_adxl366(fake_i2c)
