"""Host-side tests exercising register-level driver logic against fake buses."""

import time

import pytest
from conftest import FakeDigitalInOut, FakeI2C, FakeSPI, new_adxl366, new_adxl366_spi, new_adxl367

from adxl36x import (
    _ACT_INACT_CTL_INACT_SHIFT,
    _ACTIVITY_ENABLE,
    _REFERENCED_ACTIVITY_ENABLE,
    _REG_ACT_INACT_CTL,
    _REG_FIFO_ENTRIES_H,
    _REG_FIFO_ENTRIES_L,
    _REG_FILTER_CTL,
    _REG_INTMAP1_LWR,
    _REG_INTMAP1_UPPER,
    _REG_STATUS,
    _REG_THRESH_INACT_H,
    _REG_TIME_ACT,
    _REG_TIME_INACT_H,
    _REG_TIME_INACT_L,
    _REG_XDATA_H,
    _REVID_ADXL366,
    _REVID_ADXL367,
    _SELF_TEST_SAMPLE_COUNT,
    _THRESHOLD_MAX,
    ADXL366,
    DataRate,
    FIFOFormat,
    FIFOMode,
    OpMode,
    Range,
    _decode_s14,
)


def _poke_axis(registers: bytearray, offset: int, signed_value: int) -> None:
    raw14 = signed_value & 0x3FFF
    registers[_REG_XDATA_H + offset] = (raw14 >> 6) & 0xFF
    registers[_REG_XDATA_H + offset + 1] = (raw14 << 2) & 0xFF


# -- construction / identification --


def test_init_enables_measurement_mode(adxl366: ADXL366) -> None:
    assert adxl366.power_mode == OpMode.MEASURE


def test_init_rejects_mismatched_revid() -> None:
    fake = FakeI2C(revid=_REVID_ADXL367)
    with pytest.raises(RuntimeError):
        new_adxl366(fake)


def test_adxl367_accepts_its_own_revid() -> None:
    fake = FakeI2C(revid=_REVID_ADXL367)
    device = new_adxl367(fake)
    assert device.power_mode == OpMode.MEASURE


def test_adxl367_rejects_adxl366_revid() -> None:
    fake = FakeI2C(revid=_REVID_ADXL366)
    with pytest.raises(RuntimeError):
        new_adxl367(fake)


def test_init_survives_expected_oserror_on_reset_write(monkeypatch: pytest.MonkeyPatch) -> None:
    """Confirmed against real ADXL366 hardware: the reset-key write commonly raises
    OSError because the device starts resetting its own I2C logic mid-transaction, even
    though the reset itself lands correctly. Construction must not fail because of it.
    """
    monkeypatch.setattr(time, "sleep", lambda _seconds: None)
    fake = FakeI2C(raise_oserror_on_reset=True)
    device = new_adxl366(fake)
    assert device.power_mode == OpMode.MEASURE


def test_from_spi_construction() -> None:
    fake_spi = FakeSPI()
    cs = FakeDigitalInOut()
    device = new_adxl366_spi(fake_spi, cs)
    assert device.power_mode == OpMode.MEASURE


# -- range / data rate --


def test_range_roundtrip_and_register_bits(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.g_range = Range.RANGE_8_G
    assert adxl366.g_range == Range.RANGE_8_G
    assert (fake_i2c.registers[_REG_FILTER_CTL] >> 6) & 0x03 == Range.RANGE_8_G


def test_range_setter_rejects_invalid_value(adxl366: ADXL366) -> None:
    with pytest.raises(ValueError, match="Range"):
        adxl366.g_range = 0x07


def test_data_rate_roundtrip_and_register_bits(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.data_rate = DataRate.RATE_400_HZ
    assert adxl366.data_rate == DataRate.RATE_400_HZ
    assert fake_i2c.registers[_REG_FILTER_CTL] & 0x07 == DataRate.RATE_400_HZ


# -- acceleration decode/scale --


@pytest.mark.parametrize(
    ("high", "low", "expected"),
    [
        (0x00, 0x00, 0),
        (0x7F, 0xFC, 8191),  # max positive 14-bit code
        (0x80, 0x00, -8192),  # min negative 14-bit code
        (0xFF, 0xFC, -1),
    ],
)
def test_decode_s14(high: int, low: int, expected: int) -> None:
    assert _decode_s14(high, low) == expected


def test_raw_acceleration_sign_extension(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    _poke_axis(fake_i2c.registers, 0, 100)
    _poke_axis(fake_i2c.registers, 2, -100)
    _poke_axis(fake_i2c.registers, 4, -8192)
    assert adxl366.raw_acceleration == (100, -100, -8192)
    assert adxl366.raw_x == 100
    assert adxl366.raw_y == -100
    assert adxl366.raw_z == -8192


def test_acceleration_scaling_at_default_range(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    _poke_axis(fake_i2c.registers, 0, 4096)
    x, _y, _z = adxl366.acceleration
    expected = 4096 * 245166 / 1_000_000_000 * 9.80665
    assert x == pytest.approx(expected)


def test_acceleration_scaling_follows_range(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.g_range = Range.RANGE_8_G
    _poke_axis(fake_i2c.registers, 0, 1000)
    x, _y, _z = adxl366.acceleration
    expected = 1000 * 245166 * 4 / 1_000_000_000 * 9.80665
    assert x == pytest.approx(expected)


# -- offsets --


def test_offset_roundtrip(adxl366: ADXL366) -> None:
    adxl366.offset = (1, 2, 3)
    assert adxl366.offset == (1, 2, 3)


def test_offset_rejects_out_of_range(adxl366: ADXL366) -> None:
    with pytest.raises(ValueError, match="offset"):
        adxl366.offset = (0, 0, 32)


# -- activity/inactivity --


def test_enable_motion_detection_rejects_large_threshold(adxl366: ADXL366) -> None:
    with pytest.raises(ValueError, match="threshold"):
        adxl366.enable_motion_detection(threshold=0x2000)


def test_enable_motion_detection_converts_seconds_to_samples(
    adxl366: ADXL366,
    fake_i2c: FakeI2C,
) -> None:
    adxl366.data_rate = DataRate.RATE_100_HZ
    adxl366.enable_motion_detection(time_=2)
    assert fake_i2c.registers[_REG_TIME_ACT] == 200


def test_enable_inactivity_detection_converts_seconds_to_samples(
    adxl366: ADXL366,
    fake_i2c: FakeI2C,
) -> None:
    adxl366.data_rate = DataRate.RATE_100_HZ
    adxl366.enable_inactivity_detection(time_=3)
    samples = (fake_i2c.registers[_REG_TIME_INACT_H] << 8) | fake_i2c.registers[_REG_TIME_INACT_L]
    assert samples == 300


def test_enable_inactivity_detection_referenced_bootstraps_first(
    adxl366: ADXL366,
    fake_i2c: FakeI2C,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Confirmed against real ADXL366 hardware: referenced inactivity never latches a
    reference just from being enabled, even right after a fresh measurement-mode
    transition - only an actual inactivity event does. So this must manufacture one
    first, via a throwaway absolute/max-threshold configuration, before applying the
    requested one.
    """
    monkeypatch.setattr(time, "sleep", lambda _seconds: None)
    adxl366.data_rate = DataRate.RATE_100_HZ
    adxl366.enable_inactivity_detection(threshold=50, time_=3, referenced=True)

    act_inact_writes = [value for register, value in fake_i2c.writes if register == _REG_ACT_INACT_CTL]
    assert len(act_inact_writes) == 2
    bootstrap_value, real_value = act_inact_writes
    assert bootstrap_value == _ACTIVITY_ENABLE << _ACT_INACT_CTL_INACT_SHIFT
    assert real_value == _REFERENCED_ACTIVITY_ENABLE << _ACT_INACT_CTL_INACT_SHIFT

    # Final register state reflects the requested threshold/time, not the bootstrap's.
    assert fake_i2c.registers[_REG_THRESH_INACT_H] != (_THRESHOLD_MAX >> 6) & 0x7F
    samples = (fake_i2c.registers[_REG_TIME_INACT_H] << 8) | fake_i2c.registers[_REG_TIME_INACT_L]
    assert samples == 300


def test_enable_inactivity_detection_absolute_skips_bootstrap(
    adxl366: ADXL366,
    fake_i2c: FakeI2C,
) -> None:
    adxl366.enable_inactivity_detection(threshold=50, time_=2, referenced=False)
    act_inact_writes = [value for register, value in fake_i2c.writes if register == _REG_ACT_INACT_CTL]
    assert len(act_inact_writes) == 1


# -- events --


def test_events_reflects_status_bits(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    fake_i2c.registers[_REG_STATUS] = 0b0001_0001  # DATA_RDY + ACT
    events = adxl366.events
    assert events["data_ready"] is True
    assert events["motion"] is True
    assert events["inactivity"] is False
    assert events["fifo_overrun"] is False


# -- interrupt mapping --


def test_map_interrupt_writes_expected_bytes(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.map_interrupt(1, {"data_ready", "activity", "double_tap"})
    assert fake_i2c.registers[_REG_INTMAP1_LWR] == 0b0001_0001
    assert fake_i2c.registers[_REG_INTMAP1_UPPER] == 0b0000_0010


def test_map_interrupt_rejects_bad_pin(adxl366: ADXL366) -> None:
    with pytest.raises(ValueError, match="pin"):
        adxl366.map_interrupt(3, {"data_ready"})


def test_map_interrupt_rejects_unknown_event(adxl366: ADXL366) -> None:
    with pytest.raises(ValueError, match="unknown"):
        adxl366.map_interrupt(1, {"not_a_real_event"})


# -- FIFO --


def test_configure_fifo_writes_expected_bits(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.configure_fifo(mode=FIFOMode.STREAM, fifo_format=FIFOFormat.XYZ, sample_sets=300)
    control = fake_i2c.registers[0x28]
    assert control & 0x03 == FIFOMode.STREAM
    assert (control >> 3) & 0x0F == FIFOFormat.XYZ
    assert control & 0x04  # bit 8 of 300 (0x12C) is set
    assert fake_i2c.registers[0x29] == 0x2C  # low byte of 300


def test_configure_fifo_rejects_out_of_range_sample_sets(adxl366: ADXL366) -> None:
    with pytest.raises(ValueError, match="sample_sets"):
        adxl366.configure_fifo(sample_sets=1000)


def test_read_fifo_decodes_tagged_channels(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    entries = 2
    fake_i2c.registers[_REG_FIFO_ENTRIES_L] = entries
    fake_i2c.registers[_REG_FIFO_ENTRIES_H] = 0
    x_word = (0b00 << 14) | (500 & 0x3FFF)
    temp_word = (0b11 << 14) | ((-50) & 0x3FFF)
    fake_i2c.fifo_queue.extend(x_word.to_bytes(2, "big"))
    fake_i2c.fifo_queue.extend(temp_word.to_bytes(2, "big"))
    result = adxl366.read_fifo()
    assert result == [("x", 500), ("temp_or_adc", -50)]


# -- temperature / ADC --


def test_temperature_and_adc_are_mutually_exclusive(
    adxl366: ADXL366,
    fake_i2c: FakeI2C,
) -> None:
    _ = adxl366.temperature
    assert fake_i2c.registers[0x3D] & 0x01  # TEMP_EN
    assert not fake_i2c.registers[0x3C] & 0x01  # ADC_EN cleared

    _ = adxl366.adc_value
    assert fake_i2c.registers[0x3C] & 0x01  # ADC_EN
    assert not fake_i2c.registers[0x3D] & 0x01  # TEMP_EN cleared


# -- self-test --


def test_self_test_passes_within_expected_delta(
    adxl366: ADXL366,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(time, "sleep", lambda _seconds: None)
    readings = iter([0] * _SELF_TEST_SAMPLE_COUNT + [700] * _SELF_TEST_SAMPLE_COUNT)
    monkeypatch.setattr(type(adxl366), "raw_x", property(lambda _self: next(readings)))
    assert adxl366.self_test() is True


def test_self_test_fails_outside_expected_delta(
    adxl366: ADXL366,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(time, "sleep", lambda _seconds: None)
    readings = iter([0] * _SELF_TEST_SAMPLE_COUNT + [100] * _SELF_TEST_SAMPLE_COUNT)
    monkeypatch.setattr(type(adxl366), "raw_x", property(lambda _self: next(readings)))
    assert adxl366.self_test() is False


# -- ADXL366-only features --


def test_pedometer_roundtrip_on_adxl366(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.pedometer_enabled = True
    assert adxl366.pedometer_enabled is True
    fake_i2c.registers[0x47] = 0x00
    fake_i2c.registers[0x48] = 0x05
    assert adxl366.steps == 5
    adxl366.reset_steps()


def test_adxl367_has_no_adxl366_only_features() -> None:
    fake = FakeI2C(revid=_REVID_ADXL367)
    device = new_adxl367(fake)
    assert not hasattr(device, "pedometer_enabled")
    assert not hasattr(device, "steps")
    assert not hasattr(device, "reset_steps")
    assert not hasattr(device, "z_nonlinearity_compensation")
    assert not hasattr(device, "reference_readback")


def test_z_nonlinearity_compensation_roundtrip_on_adxl366(adxl366: ADXL366) -> None:
    adxl366.z_nonlinearity_compensation = True
    assert adxl366.z_nonlinearity_compensation is True
    adxl366.z_nonlinearity_compensation = False
    assert adxl366.z_nonlinearity_compensation is False
