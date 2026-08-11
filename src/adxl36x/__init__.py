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
_REG_X_SENS = const(0x36)
_REG_Y_SENS = const(0x37)
_REG_Z_SENS = const(0x38)
_REG_TIMER_CTL = const(0x39)
_REG_INTMAP1_UPPER = const(0x3A)
_REG_INTMAP2_UPPER = const(0x3B)
_REG_ADC_CTL = const(0x3C)
_REG_TEMP_CTL = const(0x3D)
_REG_AXIS_MASK = const(0x43)
_REG_STATUS_2 = const(0x45)
_REG_STATUS_3 = const(0x46)
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
_POWER_CTL_AUTOSLEEP = const(0x04)
_POWER_CTL_WAKEUP = const(0x08)


class OpMode:
    """Values for `ADXL367.power_mode`."""

    STANDBY = const(0x00)
    MEASURE = const(0x02)


# -- TIMER_CTL (0x39) --
_TIMER_CTL_WAKEUP_RATE_MASK = const(0xC0)
_TIMER_CTL_WAKEUP_RATE_SHIFT = const(6)
_TIMER_CTL_KEEP_ALIVE_MASK = const(0x1F)
# Datasheet documents raw codes 0 (off) through 20 (23.2 hours), each code doubling the
# previous starting from a 160ms base at code 1; codes above 20 are unspecified.
_KEEP_ALIVE_BASE_SECONDS = 0.16
_KEEP_ALIVE_MAX_CODE = const(20)


class WakeupRate:
    """Values for `ADXL367.wakeup_rate`, the sampling rate while `wakeup_mode` is enabled."""

    RATE_12_SPS = const(0x00)
    RATE_6_SPS = const(0x01)
    RATE_3_SPS = const(0x02)
    RATE_1_5_SPS = const(0x03)


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
_ACT_INACT_CTL_LINKLOOP_MASK = const(0x30)
_ACT_INACT_CTL_LINKLOOP_SHIFT = const(4)
_ACT_INACT_CTL_LINKLOOP_DEFAULT_ALIAS = const(0x2)  # undocumented 0b10 alias for DEFAULT (0b00)
_ACT_INACT_CTL_REF_READBACK_MASK = const(0xC0)
_ACTIVITY_ENABLE = const(0x01)
_REFERENCED_ACTIVITY_ENABLE = const(0x03)
_THRESHOLD_MAX = const(0x1FFF)
_TIME_ACT_MAX_SAMPLES = const(0xFF)
_TIME_INACT_MAX_SAMPLES = const(0xFFFF)


class LinkLoopMode:
    """Values for `ADXL367.link_loop_mode` (ACT_INACT_CTL's LINKLOOP bits[5:4]).

    Per the datasheet's "Linking Activity and Inactivity Detection" section, LINKED
    and LOOPED only take effect once both activity and inactivity detection are
    enabled (`enable_motion_detection()` and `enable_inactivity_detection()`) -
    otherwise the device silently falls back to DEFAULT regardless of this setting.
    """

    DEFAULT = const(0x0)
    LINKED = const(0x1)
    LOOPED = const(0x3)


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

# -- Per-axis offset trim (0x33-0x35): twos-complement, bit 4 is the sign bit, 15mg/LSB --
_OFFSET_MASK = const(0x1F)
_OFFSET_BITS = const(5)
_OFFSET_MIN = const(-16)
_OFFSET_MAX = const(15)

# -- Per-axis sensitivity trim (0x36-0x38): twos-complement, bit 5 is the sign bit, 1.56%/LSB --
_SENS_MASK = const(0x3F)
_SENS_BITS = const(6)
_SENS_MIN = const(-32)
_SENS_MAX = const(31)

# -- AXIS_MASK (0x43) --
_AXIS_MASK_TAP_AXIS_MASK = const(0x30)
_AXIS_MASK_TAP_AXIS_SHIFT = const(4)
_AXIS_MASK_ACT_INACT_MASK = const(0x07)
_AXIS_MASK_ACT_INACT_X = const(0x01)
_AXIS_MASK_ACT_INACT_Y = const(0x02)
_AXIS_MASK_ACT_INACT_Z = const(0x04)


class TapAxis:
    """Values for `ADXL367.tap_axis`: which single axis tap detection evaluates
    (AXIS_MASK's TAP_AXIS bits[5:4]).

    Named X_AXIS/Y_AXIS/Z_AXIS rather than bare X/Y/Z: CircuitPython's `const()`
    folding isn't class-scoped - it's a whole-module substitution keyed by bare
    identifier, so reusing a name already used as a const elsewhere in the file
    (`FIFOFormat.X/Y/Z`, in this case) breaks on-device compilation with a
    confusing `SyntaxError: can't assign to expression`, even though the exact
    same code is valid, unambiguous Python to CPython/ty.
    """

    X_AXIS = const(0x0)
    Y_AXIS = const(0x1)
    Z_AXIS = const(0x2)


# -- FIFO --
_FIFO_CONTROL_MODE_MASK = const(0x03)
_FIFO_CONTROL_FORMAT_MASK = const(0x78)
_FIFO_CONTROL_FORMAT_SHIFT = const(3)
_FIFO_CONTROL_SAMPLES_MSB = const(0x04)
_FIFO_ENTRIES_H_MASK = const(0x03)
_FIFO_SAMPLE_SETS_MAX = const(0x1FF)


class FIFOMode:
    """Values for `ADXL367.configure_fifo(mode=...)`. See that method's docstring for
    what each mode does.
    """

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

# -- STATUS_2 (0x45) bits --
_STATUS_2_TAP_ONE = const(0x01)
_STATUS_2_TAP_TWO = const(0x02)
_STATUS_2_TEMP_ADC_LOW = const(0x04)
_STATUS_2_TEMP_ADC_HIGH = const(0x08)
_STATUS_2_KEEP_ALIVE_TIMER = const(0x10)
_STATUS_2_FUSE_ERROR = const(0x80)

# -- STATUS_3 (0x46) bits --
_STATUS_3_PEDOMETER_OVERFLOW = const(0x01)

# -- TAP_THRESH/TAP_DUR/TAP_LATENT/TAP_WINDOW scale factors (datasheet) --
_TAP_DUR_SECONDS_PER_LSB = 625e-6
_TAP_LATENT_WINDOW_SECONDS_PER_LSB = 1.25e-3
_TAP_THRESH_MAX = const(0xFF)
_TAP_TIME_REG_MAX = const(0xFF)


def _decode_s14(msb: int, lsb: int) -> int:
    """Decode a signed 14-bit value from an H/L register pair."""
    value = ((msb << 6) | (lsb >> 2)) & 0x3FFF
    if value & 0x2000:
        value -= 0x4000
    return value


def _sign_extend(value: int, bits: int) -> int:
    """Sign-extend an unsigned `bits`-wide twos-complement field."""
    sign_bit = 1 << (bits - 1)
    value &= (1 << bits) - 1
    return value - (1 << bits) if value & sign_bit else value


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
        """Construct a driver instance over I2C.

        `spi_bus`/`cs`/`baudrate` exist so `from_spi()` can build an instance
        through this same constructor - use `from_spi()` for SPI rather than
        passing those three directly.

        :param i2c_bus: The I2C bus the device is connected to.
        :param address: The device's I2C address, set by the ASEL pin.
            Defaults to :const:`0x53` (ASEL tied high); pass :const:`0x1D`
            instead if ASEL is tied low.
        :param spi_bus: Not for direct use - see `from_spi()`.
        :param cs: Not for direct use - see `from_spi()`.
        :param baudrate: Not for direct use - see `from_spi()`.
        """
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
        """Construct a driver instance over SPI instead of I2C.

        :param spi_bus: The SPI bus the device is connected to.
        :param cs: The chip-select pin - driven low for the duration of each
            transaction, idle high otherwise.
        :param baudrate: The SPI clock frequency in Hz. Defaults to 1MHz.
        :returns: A new driver instance connected over SPI.
        """
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
    def wakeup_mode(self) -> bool:
        """Whether wake-up mode (periodic low-power sampling) is enabled.

        See `wakeup_rate` for the sampling interval used while enabled. Per the
        datasheet, wake-up mode isn't supported in low-noise or ultra-low-noise mode.
        """
        return bool(self._read_u8(_REG_POWER_CTL) & _POWER_CTL_WAKEUP)

    @wakeup_mode.setter
    def wakeup_mode(self, enable: bool) -> None:
        value = _POWER_CTL_WAKEUP if enable else 0
        self._write_masked(_REG_POWER_CTL, value, _POWER_CTL_WAKEUP)

    @property
    def autosleep(self) -> bool:
        """Whether autosleep is enabled (POWER_CTL bit 2).

        Per the datasheet's "Autosleep" section, this only takes effect while
        `link_loop_mode` is LINKED or LOOPED - in DEFAULT mode the bit is ignored.
        When active, the device autonomously drops into wake-up mode on inactivity
        and returns to measurement mode on activity, without host intervention.
        Like `wakeup_mode`, not supported in low-noise or ultra-low-noise mode.
        """
        return bool(self._read_u8(_REG_POWER_CTL) & _POWER_CTL_AUTOSLEEP)

    @autosleep.setter
    def autosleep(self, enable: bool) -> None:
        value = _POWER_CTL_AUTOSLEEP if enable else 0
        self._write_masked(_REG_POWER_CTL, value, _POWER_CTL_AUTOSLEEP)

    @property
    def wakeup_rate(self) -> int:
        """The sampling rate used while `wakeup_mode` is enabled, a `WakeupRate` value."""
        return (self._read_u8(_REG_TIMER_CTL) & _TIMER_CTL_WAKEUP_RATE_MASK) >> _TIMER_CTL_WAKEUP_RATE_SHIFT

    @wakeup_rate.setter
    def wakeup_rate(self, value: int) -> None:
        if not 0 <= value <= WakeupRate.RATE_1_5_SPS:
            msg = f"invalid WakeupRate value: {value!r}"
            raise ValueError(msg)
        self._write_masked(
            _REG_TIMER_CTL,
            value << _TIMER_CTL_WAKEUP_RATE_SHIFT,
            _TIMER_CTL_WAKEUP_RATE_MASK,
        )

    @property
    def keep_alive_timer(self) -> "float | None":
        """Keep-alive timer period in seconds, or :const:`None` if it's off (TIMER_CTL bits[4:0]).

        Only 20 discrete periods are available (160ms, doubling up to ~23.2 hours) -
        setting this to any other value snaps to the *nearest* of those 20, silently;
        read the property back afterward to see the period that actually got set.

        When it expires, ``.events["keep_alive_timer"]`` (STATUS_2 bit 4) is set and
        stays set until STATUS_2 is read - can also be routed to INT1/INT2 via
        `map_interrupt`.
        """
        raw = self._read_u8(_REG_TIMER_CTL) & _TIMER_CTL_KEEP_ALIVE_MASK
        if raw == 0:
            return None
        return _KEEP_ALIVE_BASE_SECONDS * (2 ** (raw - 1))

    @keep_alive_timer.setter
    def keep_alive_timer(self, period_seconds: "float | None") -> None:
        if period_seconds is None:
            self._write_masked(_REG_TIMER_CTL, 0, _TIMER_CTL_KEEP_ALIVE_MASK)
            return
        if period_seconds <= 0:
            msg = f"period_seconds must be positive, or None to disable, got {period_seconds!r}"
            raise ValueError(msg)
        # Valid periods are a fixed doubling sequence (20 values) - not worth pulling in
        # math.log2 for this, and CircuitPython builds aren't guaranteed to have it (this
        # ESP32 build doesn't).
        code = 1
        best_diff = abs(_KEEP_ALIVE_BASE_SECONDS - period_seconds)
        for candidate in range(2, _KEEP_ALIVE_MAX_CODE + 1):
            diff = abs(_KEEP_ALIVE_BASE_SECONDS * (2 ** (candidate - 1)) - period_seconds)
            if diff < best_diff:
                best_diff = diff
                code = candidate
        self._write_masked(_REG_TIMER_CTL, code, _TIMER_CTL_KEEP_ALIVE_MASK)

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

    # -- offsets / sensitivity trim --

    @property
    def offset(self) -> tuple[int, int, int]:
        """Per-axis offset trim: signed 5-bit codes (-16 to 15), 15mg/LSB per the datasheet."""
        return (
            _sign_extend(self._read_u8(_REG_X_OFFSET), _OFFSET_BITS),
            _sign_extend(self._read_u8(_REG_Y_OFFSET), _OFFSET_BITS),
            _sign_extend(self._read_u8(_REG_Z_OFFSET), _OFFSET_BITS),
        )

    @offset.setter
    def offset(self, value: tuple[int, int, int]) -> None:
        x, y, z = value
        for axis_value in (x, y, z):
            if not _OFFSET_MIN <= axis_value <= _OFFSET_MAX:
                msg = f"offset values must be in range {_OFFSET_MIN}-{_OFFSET_MAX}, got {value!r}"
                raise ValueError(msg)
        self._write_u8(_REG_X_OFFSET, x & _OFFSET_MASK)
        self._write_u8(_REG_Y_OFFSET, y & _OFFSET_MASK)
        self._write_u8(_REG_Z_OFFSET, z & _OFFSET_MASK)

    @property
    def sens(self) -> tuple[int, int, int]:
        """Per-axis gain trim: signed 6-bit codes (-32 to 31), 1.56%/LSB per the datasheet.

        Shares headroom with the factory trim, so a part with high factory sensitivity may
        have less room available here (per the datasheet's X_SENS/Y_SENS/Z_SENS notes).
        """
        return (
            _sign_extend(self._read_u8(_REG_X_SENS), _SENS_BITS),
            _sign_extend(self._read_u8(_REG_Y_SENS), _SENS_BITS),
            _sign_extend(self._read_u8(_REG_Z_SENS), _SENS_BITS),
        )

    @sens.setter
    def sens(self, value: tuple[int, int, int]) -> None:
        x, y, z = value
        for axis_value in (x, y, z):
            if not _SENS_MIN <= axis_value <= _SENS_MAX:
                msg = f"sens values must be in range {_SENS_MIN}-{_SENS_MAX}, got {value!r}"
                raise ValueError(msg)
        self._write_u8(_REG_X_SENS, x & _SENS_MASK)
        self._write_u8(_REG_Y_SENS, y & _SENS_MASK)
        self._write_u8(_REG_Z_SENS, z & _SENS_MASK)

    # -- activity/inactivity detection --

    @property
    def link_loop_mode(self) -> int:
        """The current `LinkLoopMode` (default, linked, or looped)."""
        raw = (self._read_u8(_REG_ACT_INACT_CTL) & _ACT_INACT_CTL_LINKLOOP_MASK) >> _ACT_INACT_CTL_LINKLOOP_SHIFT
        return LinkLoopMode.DEFAULT if raw == _ACT_INACT_CTL_LINKLOOP_DEFAULT_ALIAS else raw

    @link_loop_mode.setter
    def link_loop_mode(self, value: int) -> None:
        if value not in (LinkLoopMode.DEFAULT, LinkLoopMode.LINKED, LinkLoopMode.LOOPED):
            msg = f"invalid LinkLoopMode value: {value!r}"
            raise ValueError(msg)
        self._write_masked(
            _REG_ACT_INACT_CTL,
            value << _ACT_INACT_CTL_LINKLOOP_SHIFT,
            _ACT_INACT_CTL_LINKLOOP_MASK,
        )

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

        :param threshold: A raw accelerometer-code magnitude (same units as
            `raw_acceleration`, i.e. scaled by `g_range`) rather than the
            ADXL34x's fixed 62.5 mg/LSB - the exact mg/LSB figure for this
            threshold field is not confirmed against the datasheet. Defaults
            to :const:`100`.
        :param time_: How long, in seconds, activity must stay above
            `threshold` before an event fires. Converted to TIME_ACT's raw
            sample count using the current `data_rate`. Defaults to :const:`1`.
        :param referenced: Selects how `threshold` is compared against
            acceleration. :const:`False` (absolute, the default) compares raw
            acceleration directly against `threshold` - since gravity
            contributes a constant ~1g on whichever axis is "down", a
            `threshold` below that will trigger immediately from gravity
            alone, regardless of real motion. :const:`True` (referenced)
            compares acceleration against a reference point captured when
            detection is engaged (roughly, the orientation at that moment), so
            `threshold` measures deviation from *that*, not zero - this lets
            subtle motion be detected even with `threshold` set well below
            1g, independent of orientation.
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

        When `referenced` is :const:`True`, this first bootstraps the reference
        with a throwaway absolute-mode configuration at the maximum threshold -
        guaranteed to trigger a real inactivity event regardless of orientation -
        before applying the requested configuration. Confirmed against real
        hardware: simply enabling referenced inactivity, even immediately after a
        fresh measurement-mode transition, never actually latches a reference on
        its own. Only an actual inactivity event does (the datasheet's other
        documented trigger for recalculating the reference), so one has to be
        manufactured first.

        :param threshold: Same units as `enable_motion_detection`'s
            `threshold`. Defaults to :const:`50`.
        :param time_: How long, in seconds, acceleration must stay
            *below* `threshold` before an event fires (the inverse of activity
            detection's "above"). Same seconds-to-samples conversion as
            `enable_motion_detection`'s `time_`. Defaults to :const:`3`.
        :param referenced: Same concept as `enable_motion_detection`'s
            `referenced` - :const:`False` (absolute, the default) would never
            trigger for a device resting motionless if `threshold` is below
            1g, since gravity keeps one axis near 1g; :const:`True`
            (referenced) measures deviation from a reference point instead of
            from zero, so it can.
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
        """Currently-set event flags from STATUS (0x0B), STATUS_2 (0x45), and
        STATUS_3 (0x46):

        - ``"data_ready"``: a new sample is available. Does NOT clear on this
          read - per the datasheet, only reading the data registers themselves
          does (e.g. via `acceleration`/`raw_x`/etc.).
        - ``"fifo_ready"`` / ``"fifo_watermark"`` / ``"fifo_overrun"``: FIFO
          status. Also don't clear on this read - they clear once enough FIFO
          data has been drained (e.g. via `read_fifo()`).
        - ``"motion"`` / ``"inactivity"``: activity/inactivity has been
          detected (see `enable_motion_detection`/`enable_inactivity_detection`)
          - clears on this read.
        - ``"awake"``: whether the device is currently active. Reflects live
          state rather than a latched event, so it never "clears" - see
          `autosleep`.
        - ``"error"``: a user register error was detected (an SEU event
          disturbed a register, or the device isn't configured).
        - ``"single_tap"`` / ``"double_tap"``: a tap was detected (see
          `enable_tap_detection`) - clears on this read.
        - ``"temp_adc_low"`` / ``"temp_adc_high"``: `temperature`/`adc_value`
          crossed a configured threshold.
        - ``"keep_alive_timer"``: `keep_alive_timer` has expired - clears on
          this read.
        - ``"fuse_error"``: an internal calibration fuse error was detected.
        - ``"pedometer_overflow"``: the pedometer step counter has overflowed
          (`ADXL366` only - always :const:`False` on `ADXL367`).
        """
        status = self._read_u8(_REG_STATUS)
        status_2 = self._read_u8(_REG_STATUS_2)
        status_3 = self._read_u8(_REG_STATUS_3)
        return {
            "data_ready": bool(status & _STATUS_DATA_RDY),
            "fifo_ready": bool(status & _STATUS_FIFO_RDY),
            "fifo_watermark": bool(status & _STATUS_FIFO_WATERMARK),
            "fifo_overrun": bool(status & _STATUS_FIFO_OVERRUN),
            "motion": bool(status & _STATUS_ACT),
            "inactivity": bool(status & _STATUS_INACT),
            "awake": bool(status & _STATUS_AWAKE),
            "error": bool(status & _STATUS_ERR_USER_REGS),
            "single_tap": bool(status_2 & _STATUS_2_TAP_ONE),
            "double_tap": bool(status_2 & _STATUS_2_TAP_TWO),
            "temp_adc_low": bool(status_2 & _STATUS_2_TEMP_ADC_LOW),
            "temp_adc_high": bool(status_2 & _STATUS_2_TEMP_ADC_HIGH),
            "keep_alive_timer": bool(status_2 & _STATUS_2_KEEP_ALIVE_TIMER),
            "fuse_error": bool(status_2 & _STATUS_2_FUSE_ERROR),
            "pedometer_overflow": bool(status_3 & _STATUS_3_PEDOMETER_OVERFLOW),
        }

    # -- interrupt routing --

    def map_interrupt(self, pin: int, events: "set[str] | frozenset[str]") -> None:
        """Route the named events to an interrupt pin.

        This replaces the pin's entire mapping each call, rather than adding to
        it - pass an empty set to unmap everything from a pin.

        :param pin: Which physical pin to map to: :const:`1` for INT1 or
            :const:`2` for INT2.
        :param events: The event names to route to `pin`, replacing any
            previous mapping. Valid names: ``"data_ready"``, ``"fifo_ready"``,
            ``"fifo_watermark"``, ``"fifo_overrun"``, ``"activity"``,
            ``"inactivity"``, ``"awake"``, ``"single_tap"``, ``"double_tap"``,
            ``"temp_adc_low"``, ``"temp_adc_high"``, ``"keep_alive_timer"``,
            ``"user_register_error"``, ``"fuse_error"`` - each corresponds to
            the like-named key in `.events`, except ``"activity"``
            (``.events["motion"]``) and ``"user_register_error"``
            (``.events["error"]``), which are spelled differently here than in
            `.events`. There's also ``"active_low"``, which isn't a status
            flag at all - it reconfigures the pin itself to be active-low
            instead of the default active-high.
        """
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

        :param mode: A `FIFOMode` value. `FIFOMode.DISABLED` turns the FIFO
            off. `FIFOMode.OLDEST_SAVED` fills once and stops; new data is only
            accepted again once old entries are read out, making room (aka
            "first N"). `FIFOMode.STREAM` (the default) always holds the most
            recent data, discarding the oldest entry to make room for each new
            one (aka "last N") - useful for letting the FIFO buffer samples
            while the host is busy with other work. `FIFOMode.TRIGGERED`
            behaves like STREAM until an activity event occurs, then stops -
            like an oscilloscope's one-shot trigger, preserving the samples
            leading up to the event.
        :param fifo_format: A `FIFOFormat` value selecting which
            channel(s) each sample set contains - see `FIFOFormat`'s
            docstring for the T/A suffixes. Defaults to `FIFOFormat.XYZ`.
        :param sample_sets: The total number of sample sets to store, in
            the range 0-511. Defaults to :const:`128`.
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

        :returns: A list of ``(channel_name, raw_signed_code)`` pairs, where
            ``channel_name`` is one of ``"x"``, ``"y"``, ``"z"``,
            ``"temp_or_adc"``. Accel-channel values use the same raw-code
            scale as `raw_acceleration`.
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

    # -- tap detection --

    def enable_tap_detection(
        self,
        *,
        threshold: int = 30,
        duration: float = 0.01,
        double_tap: bool = False,
        latency: float = 0.02,
        window: float = 0.03,
    ) -> None:
        """Enable single-tap (or single- and double-tap) detection.

        :param threshold: A raw unsigned 8-bit magnitude (31.25mg/LSB per
            the datasheet, though only accurate at the +/-2g range - kept raw
            here rather than auto-converted, matching `enable_motion_detection`'s
            `threshold`). Defaults to :const:`30`.
        :param duration: The max time, in seconds, an over-threshold
            event may last to still count as a tap. Defaults to :const:`0.01`.
        :param double_tap: When :const:`True`, also configures `latency`
            and `window` to detect a second tap following the first. When
            :const:`False` (the default), only single taps are detected -
            achieved by writing :const:`0` to the datasheet's latency field,
            which it documents as disabling double-tap detection outright.
        :param latency: Only used when `double_tap` is :const:`True`:
            how long, in seconds, to wait after the first tap before a second
            tap may start. Defaults to :const:`0.02`.
        :param window: Only used when `double_tap` is :const:`True`: how
            long, in seconds, after `latency` elapses a second tap must land
            within to count. Defaults to :const:`0.03`.
        """
        if not 0 <= threshold <= _TAP_THRESH_MAX:
            msg = f"threshold must be in range 0-{_TAP_THRESH_MAX}, got {threshold!r}"
            raise ValueError(msg)
        self._write_u8(_REG_TAP_THRESH, threshold)
        duration_lsb = round(duration / _TAP_DUR_SECONDS_PER_LSB)
        self._write_u8(_REG_TAP_DUR, min(max(duration_lsb, 0), _TAP_TIME_REG_MAX))
        if double_tap:
            latency_lsb = round(latency / _TAP_LATENT_WINDOW_SECONDS_PER_LSB)
            window_lsb = round(window / _TAP_LATENT_WINDOW_SECONDS_PER_LSB)
            self._write_u8(_REG_TAP_LATENT, min(max(latency_lsb, 0), _TAP_TIME_REG_MAX))
            self._write_u8(_REG_TAP_WINDOW, min(max(window_lsb, 0), _TAP_TIME_REG_MAX))
        else:
            self._write_u8(_REG_TAP_LATENT, 0)

    def disable_tap_detection(self) -> None:
        """Disable tap and double-tap detection.

        Per the datasheet, a TAP_THRESH value of 0 disables both.
        """
        self._write_u8(_REG_TAP_THRESH, 0)

    # -- axis masking (AXIS_MASK) --

    @property
    def tap_axis(self) -> int:
        """Which axis tap detection evaluates, a `TapAxis` value."""
        return (self._read_u8(_REG_AXIS_MASK) & _AXIS_MASK_TAP_AXIS_MASK) >> _AXIS_MASK_TAP_AXIS_SHIFT

    @tap_axis.setter
    def tap_axis(self, value: int) -> None:
        if value not in (TapAxis.X_AXIS, TapAxis.Y_AXIS, TapAxis.Z_AXIS):
            msg = f"invalid TapAxis value: {value!r}"
            raise ValueError(msg)
        self._write_masked(_REG_AXIS_MASK, value << _AXIS_MASK_TAP_AXIS_SHIFT, _AXIS_MASK_TAP_AXIS_MASK)

    @property
    def blocked_axes(self) -> "frozenset[str]":
        """Axes currently excluded from activity/inactivity detection (none, by default).

        A subset of ``{"x", "y", "z"}``. Per the datasheet, a blocked axis is left out
        of both the activity and inactivity threshold comparisons - it doesn't
        affect `tap_axis` or plain acceleration reads.
        """
        raw = self._read_u8(_REG_AXIS_MASK)
        blocked = set()
        if raw & _AXIS_MASK_ACT_INACT_X:
            blocked.add("x")
        if raw & _AXIS_MASK_ACT_INACT_Y:
            blocked.add("y")
        if raw & _AXIS_MASK_ACT_INACT_Z:
            blocked.add("z")
        return frozenset(blocked)

    @blocked_axes.setter
    def blocked_axes(self, axes: "set[str] | frozenset[str]") -> None:
        unknown = set(axes) - {"x", "y", "z"}
        if unknown:
            msg = f"unknown axis name(s): {sorted(unknown)!r}"
            raise ValueError(msg)
        value = 0
        if "x" in axes:
            value |= _AXIS_MASK_ACT_INACT_X
        if "y" in axes:
            value |= _AXIS_MASK_ACT_INACT_Y
        if "z" in axes:
            value |= _AXIS_MASK_ACT_INACT_Z
        self._write_masked(_REG_AXIS_MASK, value, _AXIS_MASK_ACT_INACT_MASK)


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

        Only meaningful when the corresponding detector is in referenced mode
        (see `enable_motion_detection`/`enable_inactivity_detection`'s
        `referenced` parameter).

        :param inactivity: When :const:`True`, read the inactivity
            reference; when :const:`False` (the default), read the activity
            reference.
        :returns: The reference point as a raw ``(x, y, z)`` tuple, the same
            scale as `raw_acceleration`.
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
