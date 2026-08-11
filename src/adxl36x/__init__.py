"""CircuitPython driver for the Analog Devices ADXL366/ADXL367 accelerometers."""

import time

try:
    from typing import TYPE_CHECKING
except ImportError:
    TYPE_CHECKING = False  # ty: ignore[invalid-assignment]

from micropython import const

if TYPE_CHECKING:
    from busio import I2C, SPI
    from digitalio import DigitalInOut
    from typing_extensions import Self

_STANDARD_GRAVITY = 9.80665

# -- I2C addresses (ASEL pin low/high) --
_DEFAULT_I2C_ADDRESS = const(0x53)
_ALT_I2C_ADDRESS = const(0x1D)

# -- SPI protocol commands --
_SPI_WRITE_REG = const(0x0A)
_SPI_READ_REG = const(0x0B)
_SPI_READ_FIFO = const(0x0D)

# -- Register addresses --
_REG_DEVID_AD = const(0x00)
_REG_DEVID_MST = const(0x01)
_REG_PARTID = const(0x02)
_REG_REVID = const(0x03)
_REG_SERIAL_NUMBER_3 = const(0x04)
_REG_XDATA = const(0x08)
_REG_STATUS = const(0x0B)
_REG_FIFO_ENTRIES_L = const(0x0C)
_REG_FIFO_ENTRIES_H = const(0x0D)
_REG_XDATA_H = const(0x0E)
_REG_TEMP_H = const(0x14)
_REG_EX_ADC_H = const(0x16)
_REG_I2C_FIFO_DATA = const(0x18)
_REG_SOFT_RESET = const(0x1F)
_REG_THRESH_ACT_H = const(0x20)
_REG_THRESH_ACT_L = const(0x21)
_REG_TIME_ACT = const(0x22)
_REG_THRESH_INACT_H = const(0x23)
_REG_THRESH_INACT_L = const(0x24)
_REG_TIME_INACT_H = const(0x25)
_REG_TIME_INACT_L = const(0x26)
_REG_ACT_INACT_CTL = const(0x27)
_REG_FIFO_CONTROL = const(0x28)
_REG_FIFO_SAMPLES = const(0x29)
_REG_INTMAP1_LWR = const(0x2A)
_REG_INTMAP2_LWR = const(0x2B)
_REG_FILTER_CTL = const(0x2C)
_REG_POWER_CTL = const(0x2D)
_REG_SELF_TEST = const(0x2E)
_REG_TAP_THRESH = const(0x2F)
_REG_TAP_DUR = const(0x30)
_REG_TAP_LATENT = const(0x31)
_REG_TAP_WINDOW = const(0x32)
_REG_X_OFFSET = const(0x33)
_REG_Y_OFFSET = const(0x34)
_REG_Z_OFFSET = const(0x35)
_REG_TIMER_CTL = const(0x39)
_REG_INTMAP1_UPPER = const(0x3A)
_REG_INTMAP2_UPPER = const(0x3B)
_REG_ADC_CTL = const(0x3C)
_REG_TEMP_CTL = const(0x3D)
_REG_AXIS_MASK = const(0x43)
_REG_PEDOMETER_STEP_CNT_H = const(0x47)
_REG_PEDOMETER_CTL = const(0x49)

# -- Device/part identification --
_DEVID_AD = const(0xAD)
_DEVID_MST = const(0x1D)
_PART_ID = const(0xF7)
_REVID_ADXL367 = const(0x3)
_REVID_ADXL366 = const(0x5)
_RESET_KEY = const(0x52)

# -- POWER_CTL (0x2D) --
_POWER_CTL_MODE_MASK = const(0x03)


class OpMode:
    """Values for `ADXL367.power_mode`."""

    STANDBY = const(0x00)
    MEASURE = const(0x02)


# -- FILTER_CTL (0x2C) --
_FILTER_CTL_RANGE_MASK = const(0xC0)
_FILTER_CTL_RANGE_SHIFT = const(6)
_FILTER_CTL_ODR_MASK = const(0x07)


class Range:
    """Values for `ADXL367.g_range`.

    Unlike the ADXL34x family, ±16g isn't supported.
    """

    RANGE_2_G = const(0x00)
    RANGE_4_G = const(0x01)
    RANGE_8_G = const(0x02)


_RANGE_SCALE_MULTIPLIER = (1, 2, 4)  # indexed by Range value


class DataRate:
    """Values for `ADXL367.data_rate`."""

    RATE_12_5_HZ = const(0x00)
    RATE_25_HZ = const(0x01)
    RATE_50_HZ = const(0x02)
    RATE_100_HZ = const(0x03)
    RATE_200_HZ = const(0x04)
    RATE_400_HZ = const(0x05)


_DATA_RATE_HZ = (12.5, 25.0, 50.0, 100.0, 200.0, 400.0)  # indexed by DataRate value

# -- Acceleration/temperature scaling --
_ACCEL_SCALE_MUL = const(245166)
_ACCEL_SCALE_DIV = const(1_000_000_000)
_TEMP_OFFSET = const(1185)
_TEMP_SCALE_MUL = const(18518518)
_TEMP_SCALE_DIV = const(1_000_000_000)

# -- ACT_INACT_CTL (0x27) --
_ACT_INACT_CTL_ACT_MASK = const(0x03)
_ACT_INACT_CTL_INACT_MASK = const(0x0C)
_ACT_INACT_CTL_INACT_SHIFT = const(2)
_ACT_INACT_CTL_REF_READBACK_MASK = const(0xC0)
_ACTIVITY_ENABLE = const(0x01)
_REFERENCED_ACTIVITY_ENABLE = const(0x03)
_THRESHOLD_MAX = const(0x1FFF)
_TIME_ACT_MAX_SAMPLES = const(0xFF)
_TIME_INACT_MAX_SAMPLES = const(0xFFFF)

# -- SELF_TEST (0x2E) --
_SELF_TEST_ST = const(0x01)
_SELF_TEST_ST_FORCE = const(0x02)
# Output Change spec (Table 1): 133mg min, 222mg max, measured on XOUT. Converted to raw
# LSB codes at the +/-2g range's 0.25mg/LSB sensitivity, since self-test is only accurate
# in that range (datasheet's "Using Self Test" section) - confirmed against real hardware
# that the previously-coded 90mg/270mg bounds here didn't match the datasheet at all.
_SELF_TEST_MIN = const(532)  # 133 / 0.25
_SELF_TEST_MAX = const(888)  # 222 / 0.25
_SELF_TEST_SETTLE_S = 0.1  # settle time after entering measurement mode, before ST sequence
_SELF_TEST_SAMPLE_COUNT = const(8)  # datasheet recommends averaging 4-16 samples per side

# -- TEMP_CTL (0x3D) / ADC_CTL (0x3C) --
_TEMP_CTL_EN = const(0x01)
_TEMP_CTL_NL_COMP_EN = const(0x80)
_ADC_CTL_EN = const(0x01)

# -- Per-axis offset trim (0x33-0x35) --
_OFFSET_MAX = const(0x1F)

# -- FIFO --
_FIFO_CONTROL_MODE_MASK = const(0x03)
_FIFO_CONTROL_FORMAT_MASK = const(0x78)
_FIFO_CONTROL_FORMAT_SHIFT = const(3)
_FIFO_CONTROL_SAMPLES_MSB = const(0x04)
_FIFO_ENTRIES_H_MASK = const(0x03)
_FIFO_SAMPLE_SETS_MAX = const(0x1FF)


class FIFOMode:
    """Values for `ADXL367.configure_fifo(mode=...)`."""

    DISABLED = const(0x00)
    OLDEST_SAVED = const(0x01)
    STREAM = const(0x02)
    TRIGGERED = const(0x03)


class FIFOFormat:
    """Values for `ADXL367.configure_fifo(fifo_format=...)`.

    T/A suffixes add a temperature-or-ADC sample (whichever is enabled via
    `temperature`/`adc_value`) to each sample set.
    """

    XYZ = const(0x00)
    X = const(0x01)
    Y = const(0x02)
    Z = const(0x03)
    XYZT = const(0x04)
    XT = const(0x05)
    YT = const(0x06)
    ZT = const(0x07)
    XYZA = const(0x08)
    XA = const(0x09)
    YA = const(0x0A)
    ZA = const(0x0B)


_FIFO_CHANNEL_NAMES = ("x", "y", "z", "temp_or_adc")  # indexed by FIFO channel ID

# -- Interrupt mapping (INTMAP1/2 LWR + UPPER) --
_INTMAPX_UPPER_MASK = const(0xDF)
# name -> (is_upper_byte, bit_index)
_INTERRUPT_EVENTS = {
    "data_ready": (False, 0),
    "fifo_ready": (False, 1),
    "fifo_watermark": (False, 2),
    "fifo_overrun": (False, 3),
    "activity": (False, 4),
    "inactivity": (False, 5),
    "awake": (False, 6),
    "active_low": (False, 7),
    "single_tap": (True, 0),
    "double_tap": (True, 1),
    "temp_adc_low": (True, 2),
    "temp_adc_high": (True, 3),
    "keep_alive_timer": (True, 4),
    "user_register_error": (True, 6),
    "fuse_error": (True, 7),
}

# -- STATUS (0x0B) bits --
_STATUS_DATA_RDY = const(0x01)
_STATUS_FIFO_RDY = const(0x02)
_STATUS_FIFO_WATERMARK = const(0x04)
_STATUS_FIFO_OVERRUN = const(0x08)
_STATUS_ACT = const(0x10)
_STATUS_INACT = const(0x20)
_STATUS_AWAKE = const(0x40)
_STATUS_ERR_USER_REGS = const(0x80)


def _decode_s14(msb: int, lsb: int) -> int:
    """Decode a signed 14-bit value from an H/L register pair."""
    value = ((msb << 6) | (lsb >> 2)) & 0x3FFF
    if value & 0x2000:
        value -= 0x4000
    return value


# -- bus transport --


class I2CBus:
    """Register-level access to a device over I2C.

    Talks to `busio.I2C` directly rather than going through
    `adafruit_bus_device.I2CDevice`. Confirmed against real ADXL366 hardware:
    subclassing `I2CDevice` (even trivially, only to fix its mistyped
    `__exit__`) broke bus locking outright on that board's CircuitPython
    build - `self.i2c` came back as an unrelated int inside inherited
    `__enter__`, even though the identical operation worked fine through an
    unmodified `I2CDevice` instance. `busio.I2C`'s own locking/transfer
    methods carry no such risk and need no wrapper class at all.
    """

    def __init__(self, i2c_bus: "I2C", address: int) -> None:
        self._i2c = i2c_bus
        self._address = address

    def _lock(self) -> None:
        while not self._i2c.try_lock():
            pass

    def read_into(self, register: int, buffer: bytearray) -> None:
        self._lock()
        try:
            self._i2c.writeto_then_readfrom(self._address, bytes((register,)), buffer)
        finally:
            self._i2c.unlock()

    def write(self, register: int, value: int) -> None:
        self._lock()
        try:
            self._i2c.writeto(self._address, bytes((register, value & 0xFF)))
        finally:
            self._i2c.unlock()

    def read_fifo_into(self, buffer: bytearray) -> None:
        self._lock()
        try:
            self._i2c.writeto_then_readfrom(self._address, bytes((_REG_I2C_FIFO_DATA,)), buffer)
        finally:
            self._i2c.unlock()


class _SPIBus:
    """Register-level access to a device over SPI. See `I2CBus`."""

    def __init__(self, spi_bus: "SPI", cs: "DigitalInOut", *, baudrate: int) -> None:
        self._spi = spi_bus
        self._cs = cs
        self._baudrate = baudrate
        self._cs.switch_to_output(value=True)  # idle high; CS is active low

    def _lock(self) -> None:
        while not self._spi.try_lock():
            pass
        self._spi.configure(baudrate=self._baudrate, polarity=0, phase=0)
        self._cs.value = False

    def _unlock(self) -> None:
        self._cs.value = True
        self._spi.unlock()

    def read_into(self, register: int, buffer: bytearray) -> None:
        self._lock()
        try:
            self._spi.write(bytes((_SPI_READ_REG, register)))
            self._spi.readinto(buffer)
        finally:
            self._unlock()

    def write(self, register: int, value: int) -> None:
        self._lock()
        try:
            self._spi.write(bytes((_SPI_WRITE_REG, register, value & 0xFF)))
        finally:
            self._unlock()

    def read_fifo_into(self, buffer: bytearray) -> None:
        self._lock()
        try:
            self._spi.write(bytes((_SPI_READ_FIFO,)))
            self._spi.readinto(buffer)
        finally:
            self._unlock()


class ADXL367:
    """Driver for the Analog Devices ADXL367 accelerometer.

    Also the base class for `ADXL366`, which shares this entire register map
    and adds a few extra features. Connect over I2C with the constructor
    (mirroring `adafruit_adxl34x.ADXL345`) or over SPI with `from_spi()`.
    """

    _revid = _REVID_ADXL367

    def __init__(
        self,
        i2c_bus: "I2C | None" = None,
        *,
        address: int = _DEFAULT_I2C_ADDRESS,
        spi_bus: "SPI | None" = None,
        cs: "DigitalInOut | None" = None,
        baudrate: int = 1_000_000,
    ) -> None:
        # `from_spi()` is the intended entry point for SPI - this constructor takes
        # spi_bus/cs directly (rather than `from_spi()` building the instance itself via
        # `cls.__new__()`) because CircuitPython's built-in types don't expose `__new__`
        # as a callable attribute, unlike CPython (confirmed on real hardware).
        if spi_bus is not None:
            if cs is None:
                msg = "spi_bus requires cs (the chip-select pin) to also be provided"
                raise ValueError(msg)
            self._bus: I2CBus | _SPIBus = _SPIBus(spi_bus, cs, baudrate=baudrate)
        elif i2c_bus is not None:
            self._bus = I2CBus(i2c_bus, address)
        else:
            msg = "ADXL367() requires either i2c_bus or spi_bus (see ADXL367.from_spi())"
            raise ValueError(msg)
        self._range = Range.RANGE_2_G
        self._data_rate = DataRate.RATE_100_HZ
        self._initialize()

    @classmethod
    def from_spi(
        cls,
        spi_bus: "SPI",
        cs: "DigitalInOut",
        *,
        baudrate: int = 1_000_000,
    ) -> "Self":
        """Construct a driver instance over SPI instead of I2C."""
        return cls(spi_bus=spi_bus, cs=cs, baudrate=baudrate)

    # -- low-level bus access --

    def _read_into(self, register: int, buffer: bytearray) -> None:
        self._bus.read_into(register, buffer)

    def _read_u8(self, register: int) -> int:
        buffer = bytearray(1)
        self._read_into(register, buffer)
        return buffer[0]

    def _write_u8(self, register: int, value: int) -> None:
        self._bus.write(register, value)

    def _write_masked(self, register: int, value: int, mask: int) -> None:
        current = self._read_u8(register)
        self._write_u8(register, (current & ~mask) | (value & mask))

    def _read_fifo_into(self, buffer: bytearray) -> None:
        self._bus.read_fifo_into(buffer)

    def _set_threshold(
        self,
        high_register: int,
        low_register: int,
        threshold: int,
    ) -> None:
        if not 0 <= threshold <= _THRESHOLD_MAX:
            msg = f"threshold must be in range 0-{_THRESHOLD_MAX}, got {threshold!r}"
            raise ValueError(msg)
        self._write_u8(high_register, (threshold >> 6) & 0x7F)
        self._write_u8(low_register, (threshold & 0x3F) << 2)

    # -- initialization --

    def reset(self) -> None:
        """Perform a software reset, returning all registers to power-on defaults.

        The reset-key write commonly raises `OSError` on real hardware: the device
        starts resetting its own I2C peripheral logic mid-transaction, which cuts the
        bus transaction off from the controller's point of view even though the reset
        itself lands correctly. Confirmed against real ADXL366 hardware - the device is
        alive and reports correct ID bytes immediately after this "failed" write.
        """
        try:  # noqa: SIM105 - contextlib isn't a guaranteed-present CircuitPython core module
            self._write_u8(_REG_SOFT_RESET, _RESET_KEY)
        except OSError:
            pass
        time.sleep(0.02)

    def _initialize(self) -> None:
        self.reset()
        seen = (
            self._read_u8(_REG_DEVID_AD),
            self._read_u8(_REG_DEVID_MST),
            self._read_u8(_REG_PARTID),
            self._read_u8(_REG_REVID),
        )
        expected = (_DEVID_AD, _DEVID_MST, _PART_ID, self._revid)
        if seen != expected:
            msg = f"{type(self).__name__}: unexpected device ID bytes {seen!r} (expected {expected!r})"
            raise RuntimeError(msg)
        self.power_mode = OpMode.MEASURE

    # -- identification --

    @property
    def serial_number(self) -> int:
        """This chip's factory-programmed 32-bit unique serial number."""
        buffer = bytearray(4)
        self._read_into(_REG_SERIAL_NUMBER_3, buffer)
        return int.from_bytes(bytes(buffer), "big")

    # -- power/range/rate --

    @property
    def power_mode(self) -> int:
        """The current `OpMode` (standby or measure)."""
        return self._read_u8(_REG_POWER_CTL) & _POWER_CTL_MODE_MASK

    @power_mode.setter
    def power_mode(self, mode: int) -> None:
        self._write_masked(_REG_POWER_CTL, mode, _POWER_CTL_MODE_MASK)

    @property
    def g_range(self) -> int:
        """The measurement range, a `Range` value (tops out at ±8g, unlike ADXL34x)."""
        return self._range

    @g_range.setter
    def g_range(self, value: int) -> None:
        if value not in (Range.RANGE_2_G, Range.RANGE_4_G, Range.RANGE_8_G):
            msg = f"invalid Range value: {value!r}"
            raise ValueError(msg)
        self._write_masked(
            _REG_FILTER_CTL,
            value << _FILTER_CTL_RANGE_SHIFT,
            _FILTER_CTL_RANGE_MASK,
        )
        self._range = value

    @property
    def data_rate(self) -> int:
        """The output data rate, a `DataRate` value."""
        return self._data_rate

    @data_rate.setter
    def data_rate(self, value: int) -> None:
        if not 0 <= value <= DataRate.RATE_400_HZ:
            msg = f"invalid DataRate value: {value!r}"
            raise ValueError(msg)
        self._write_masked(_REG_FILTER_CTL, value, _FILTER_CTL_ODR_MASK)
        self._data_rate = value

    # -- acceleration --

    @property
    def raw_x(self) -> int:
        """Raw, signed 14-bit X-axis acceleration code."""
        return self._read_raw_axis(_REG_XDATA_H)

    @property
    def raw_y(self) -> int:
        """Raw, signed 14-bit Y-axis acceleration code."""
        return self._read_raw_axis(_REG_XDATA_H + 2)

    @property
    def raw_z(self) -> int:
        """Raw, signed 14-bit Z-axis acceleration code."""
        return self._read_raw_axis(_REG_XDATA_H + 4)

    def _read_raw_axis(self, high_register: int) -> int:
        buffer = bytearray(2)
        self._read_into(high_register, buffer)
        return _decode_s14(buffer[0], buffer[1])

    @property
    def raw_acceleration(self) -> tuple[int, int, int]:
        """Raw, signed 14-bit (x, y, z) codes, read in a single bus transaction."""
        buffer = bytearray(6)
        self._read_into(_REG_XDATA_H, buffer)
        return (
            _decode_s14(buffer[0], buffer[1]),
            _decode_s14(buffer[2], buffer[3]),
            _decode_s14(buffer[4], buffer[5]),
        )

    @property
    def acceleration(self) -> tuple[float, float, float]:
        """(x, y, z) acceleration in m/s^2."""
        scale = _RANGE_SCALE_MULTIPLIER[self._range] * _ACCEL_SCALE_MUL / _ACCEL_SCALE_DIV * _STANDARD_GRAVITY
        x, y, z = self.raw_acceleration
        return (x * scale, y * scale, z * scale)

    # -- offsets --

    @property
    def offset(self) -> tuple[int, int, int]:
        """Per-axis trim offsets: raw 5-bit codes (0-31); units unconfirmed."""
        return (
            self._read_u8(_REG_X_OFFSET) & _OFFSET_MAX,
            self._read_u8(_REG_Y_OFFSET) & _OFFSET_MAX,
            self._read_u8(_REG_Z_OFFSET) & _OFFSET_MAX,
        )

    @offset.setter
    def offset(self, value: tuple[int, int, int]) -> None:
        x, y, z = value
        for axis_value in (x, y, z):
            if not 0 <= axis_value <= _OFFSET_MAX:
                msg = f"offset values must be in range 0-{_OFFSET_MAX}, got {value!r}"
                raise ValueError(msg)
        self._write_u8(_REG_X_OFFSET, x)
        self._write_u8(_REG_Y_OFFSET, y)
        self._write_u8(_REG_Z_OFFSET, z)

    # -- activity/inactivity detection --

    def _seconds_to_samples(self, time_seconds: float, max_samples: int) -> int:
        """Convert a duration in seconds to the raw consecutive-sample count these
        timer registers actually store (samples = time x ODR, confirmed against the
        datasheet's free-fall TIME_INACT equation and its "1 sample to 20sec of
        motion" figure for TIME_ACT, which lines up with 0xFF samples at the slowest
        12.5Hz ODR).
        """
        samples = round(time_seconds * _DATA_RATE_HZ[self._data_rate])
        return min(max(samples, 0), max_samples)

    def enable_motion_detection(
        self,
        *,
        threshold: int = 100,
        time_: float = 1,
        referenced: bool = False,
    ) -> None:
        """Enable activity (motion) detection.

        `threshold` is a raw accelerometer-code magnitude (same units as
        `raw_acceleration`, i.e. scaled by `range`) rather than the ADXL34x's
        fixed 62.5 mg/LSB - the exact mg/LSB figure for this threshold field
        is not confirmed against the datasheet. `time_` is in seconds, converted to
        TIME_ACT's raw sample count using the current `data_rate`.
        """
        self._set_threshold(_REG_THRESH_ACT_H, _REG_THRESH_ACT_L, threshold)
        samples = self._seconds_to_samples(time_, _TIME_ACT_MAX_SAMPLES)
        self._write_u8(_REG_TIME_ACT, samples)
        mode = _REFERENCED_ACTIVITY_ENABLE if referenced else _ACTIVITY_ENABLE
        self._write_masked(_REG_ACT_INACT_CTL, mode, _ACT_INACT_CTL_ACT_MASK)

    def disable_motion_detection(self) -> None:
        """Disable activity (motion) detection."""
        self._write_masked(_REG_ACT_INACT_CTL, 0, _ACT_INACT_CTL_ACT_MASK)

    def enable_inactivity_detection(
        self,
        *,
        threshold: int = 50,
        time_: float = 3,
        referenced: bool = False,
    ) -> None:
        """Enable inactivity detection.

        See `enable_motion_detection` for the `time_` conversion. When `referenced=True`,
        this first bootstraps the reference with a throwaway absolute-mode configuration
        at the maximum threshold - guaranteed to trigger a real inactivity event
        regardless of orientation - before applying the requested configuration.
        Confirmed against real hardware: simply enabling referenced inactivity, even
        immediately after a fresh measurement-mode transition, never actually latches a
        reference on its own. Only an actual inactivity event does (the datasheet's
        other documented trigger for recalculating the reference), so one has to be
        manufactured first.
        """
        if referenced:
            self._set_threshold(_REG_THRESH_INACT_H, _REG_THRESH_INACT_L, _THRESHOLD_MAX)
            self._write_u8(_REG_TIME_INACT_H, 0)
            self._write_u8(_REG_TIME_INACT_L, 1)
            bootstrap = _ACTIVITY_ENABLE << _ACT_INACT_CTL_INACT_SHIFT
            self._write_masked(_REG_ACT_INACT_CTL, bootstrap, _ACT_INACT_CTL_INACT_MASK)
            time.sleep(4.0 / _DATA_RATE_HZ[self._data_rate])
        self._set_threshold(_REG_THRESH_INACT_H, _REG_THRESH_INACT_L, threshold)
        samples = self._seconds_to_samples(time_, _TIME_INACT_MAX_SAMPLES)
        self._write_u8(_REG_TIME_INACT_H, (samples >> 8) & 0xFF)
        self._write_u8(_REG_TIME_INACT_L, samples & 0xFF)
        mode = _REFERENCED_ACTIVITY_ENABLE if referenced else _ACTIVITY_ENABLE
        shifted = mode << _ACT_INACT_CTL_INACT_SHIFT
        self._write_masked(_REG_ACT_INACT_CTL, shifted, _ACT_INACT_CTL_INACT_MASK)

    def disable_inactivity_detection(self) -> None:
        """Disable inactivity detection."""
        self._write_masked(_REG_ACT_INACT_CTL, 0, _ACT_INACT_CTL_INACT_MASK)

    @property
    def events(self) -> dict[str, bool]:
        """Currently-set event flags from STATUS (0x0B).

        Does not yet include tap, pedometer-overflow, or ADC/temperature
        threshold events - their status-register bit positions
        (STATUS_COPY/STATUS_2/STATUS_3) aren't confirmed against the
        datasheet yet.
        """
        status = self._read_u8(_REG_STATUS)
        return {
            "data_ready": bool(status & _STATUS_DATA_RDY),
            "fifo_ready": bool(status & _STATUS_FIFO_RDY),
            "fifo_watermark": bool(status & _STATUS_FIFO_WATERMARK),
            "fifo_overrun": bool(status & _STATUS_FIFO_OVERRUN),
            "motion": bool(status & _STATUS_ACT),
            "inactivity": bool(status & _STATUS_INACT),
            "awake": bool(status & _STATUS_AWAKE),
            "error": bool(status & _STATUS_ERR_USER_REGS),
        }

    # -- interrupt routing --

    def map_interrupt(self, pin: int, events: "set[str] | frozenset[str]") -> None:
        """Route the named events (keys of `_INTERRUPT_EVENTS`) to INT1 or INT2."""
        if pin not in (1, 2):
            msg = f"pin must be 1 or 2, got {pin!r}"
            raise ValueError(msg)
        lower = upper = 0
        for name in events:
            if name not in _INTERRUPT_EVENTS:
                msg = f"unknown interrupt event: {name!r}"
                raise ValueError(msg)
            is_upper, bit = _INTERRUPT_EVENTS[name]
            if is_upper:
                upper |= 1 << bit
            else:
                lower |= 1 << bit
        lwr_register = _REG_INTMAP1_LWR if pin == 1 else _REG_INTMAP2_LWR
        upper_register = _REG_INTMAP1_UPPER if pin == 1 else _REG_INTMAP2_UPPER
        self._write_u8(lwr_register, lower)
        self._write_u8(upper_register, upper & _INTMAPX_UPPER_MASK)

    # -- temperature / external ADC --

    @property
    def temperature(self) -> float:
        """On-chip temperature in Celsius. Mutually exclusive with `adc_value`."""
        self._write_masked(_REG_TEMP_CTL, _TEMP_CTL_EN, _TEMP_CTL_EN)
        self._write_masked(_REG_ADC_CTL, 0, _ADC_CTL_EN)
        buffer = bytearray(2)
        self._read_into(_REG_TEMP_H, buffer)
        raw = _decode_s14(buffer[0], buffer[1])
        return (raw + _TEMP_OFFSET) * _TEMP_SCALE_MUL / _TEMP_SCALE_DIV

    @property
    def adc_value(self) -> int:
        """Raw signed 14-bit ADC code. Mutually exclusive with `temperature`."""
        self._write_masked(_REG_ADC_CTL, _ADC_CTL_EN, _ADC_CTL_EN)
        self._write_masked(_REG_TEMP_CTL, 0, _TEMP_CTL_EN)
        buffer = bytearray(2)
        self._read_into(_REG_EX_ADC_H, buffer)
        return _decode_s14(buffer[0], buffer[1])

    # -- FIFO --

    def configure_fifo(
        self,
        *,
        mode: int = FIFOMode.STREAM,
        fifo_format: int = FIFOFormat.XYZ,
        sample_sets: int = 128,
    ) -> None:
        """Configure the FIFO.

        `sample_sets` (0-511) is the total number of sample sets to store.
        """
        if not 0 <= sample_sets <= _FIFO_SAMPLE_SETS_MAX:
            msg = f"sample_sets must be in range 0-{_FIFO_SAMPLE_SETS_MAX}, got {sample_sets!r}"
            raise ValueError(msg)
        control = mode & _FIFO_CONTROL_MODE_MASK
        control |= (fifo_format << _FIFO_CONTROL_FORMAT_SHIFT) & _FIFO_CONTROL_FORMAT_MASK
        if sample_sets & 0x100:
            control |= _FIFO_CONTROL_SAMPLES_MSB
        self._write_u8(_REG_FIFO_CONTROL, control)
        self._write_u8(_REG_FIFO_SAMPLES, sample_sets & 0xFF)

    @property
    def fifo_entries(self) -> int:
        """Number of 16-bit words currently stored in the FIFO."""
        low = self._read_u8(_REG_FIFO_ENTRIES_L)
        high = self._read_u8(_REG_FIFO_ENTRIES_H) & _FIFO_ENTRIES_H_MASK
        return (high << 8) | low

    def read_fifo(self) -> list[tuple[str, int]]:
        """Read and drain all available FIFO entries.

        Returns a list of (channel_name, raw_signed_code) pairs, where
        channel_name is one of "x", "y", "z", "temp_or_adc". Accel-channel
        values use the same raw-code scale as `raw_acceleration`.
        """
        entries = self.fifo_entries
        buffer = bytearray(entries * 2)
        self._read_fifo_into(buffer)
        result = []
        for i in range(entries):
            high, low = buffer[2 * i], buffer[2 * i + 1]
            channel = high >> 6
            raw = ((high & 0x3F) << 8) | low
            if raw & 0x2000:
                raw -= 0x4000
            result.append((_FIFO_CHANNEL_NAMES[channel], raw))
        return result

    # -- self-test --

    def _average_raw_x(self, count: int, sample_period: float) -> float:
        total = 0
        for _ in range(count):
            total += self.raw_x
            time.sleep(sample_period)
        return total / count

    def self_test(self) -> bool:
        """Run the built-in electrostatic self-test. Returns True if it passes.

        Follows the datasheet's "Using Self Test" procedure: settle after entering
        measurement mode, then average several x-axis samples before and after
        asserting the self-test force, since a single sample is noise-sensitive enough
        to produce false failures.
        """
        previous_mode = self.power_mode
        self.power_mode = OpMode.MEASURE
        time.sleep(_SELF_TEST_SETTLE_S)
        sample_period = 1.0 / _DATA_RATE_HZ[self._data_rate]
        settle_delay = 4.0 * sample_period
        both_bits = _SELF_TEST_ST | _SELF_TEST_ST_FORCE
        try:
            self._write_masked(_REG_SELF_TEST, _SELF_TEST_ST, both_bits)
            time.sleep(settle_delay)
            before = self._average_raw_x(_SELF_TEST_SAMPLE_COUNT, sample_period)
            self._write_masked(_REG_SELF_TEST, both_bits, both_bits)
            time.sleep(settle_delay)
            after = self._average_raw_x(_SELF_TEST_SAMPLE_COUNT, sample_period)
        finally:
            self._write_u8(_REG_SELF_TEST, 0)
            self.power_mode = previous_mode
        diff = after - before
        scale = _RANGE_SCALE_MULTIPLIER[self._range]
        return _SELF_TEST_MIN * scale <= diff <= _SELF_TEST_MAX * scale


class ADXL366(ADXL367):
    """Driver for the Analog Devices ADXL366 accelerometer.

    Adds Z-axis nonlinearity compensation, activity/inactivity reference
    readback, and a step-counting pedometer on top of everything `ADXL367`
    provides - the ADXL367 silicon doesn't implement these three features.
    """

    _revid = _REVID_ADXL366

    @property
    def z_nonlinearity_compensation(self) -> bool:
        """Z-axis temperature-informed nonlinearity compensation state."""
        return bool(self._read_u8(_REG_TEMP_CTL) & _TEMP_CTL_NL_COMP_EN)

    @z_nonlinearity_compensation.setter
    def z_nonlinearity_compensation(self, enable: bool) -> None:
        value = _TEMP_CTL_NL_COMP_EN if enable else 0
        self._write_masked(_REG_TEMP_CTL, value, _TEMP_CTL_NL_COMP_EN)

    def reference_readback(self, *, inactivity: bool = False) -> tuple[int, int, int]:
        """Read back the activity/inactivity reference point.

        Only meaningful when the corresponding detector is in referenced mode.
        """
        select = 0x02 if inactivity else 0x01
        self._write_masked(
            _REG_ACT_INACT_CTL,
            select << 6,
            _ACT_INACT_CTL_REF_READBACK_MASK,
        )
        try:
            return self.raw_acceleration
        finally:
            self._write_masked(_REG_ACT_INACT_CTL, 0, _ACT_INACT_CTL_REF_READBACK_MASK)

    @property
    def pedometer_enabled(self) -> bool:
        """Whether the step-counting pedometer is enabled."""
        return bool(self._read_u8(_REG_PEDOMETER_CTL) & 0x01)

    @pedometer_enabled.setter
    def pedometer_enabled(self, enable: bool) -> None:
        self._write_masked(_REG_PEDOMETER_CTL, 0x01 if enable else 0x00, 0x01)

    @property
    def steps(self) -> int:
        """Cumulative pedometer step count."""
        buffer = bytearray(2)
        self._read_into(_REG_PEDOMETER_STEP_CNT_H, buffer)
        return (buffer[0] << 8) | buffer[1]

    def reset_steps(self) -> None:
        """Reset the pedometer step count to zero."""
        self._write_masked(_REG_PEDOMETER_CTL, 0x04, 0x04)
