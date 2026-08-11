"""Host-side tests exercising register-level driver logic against fake buses."""

import time

import pytest
from conftest import FakeDigitalInOut, FakeI2C, FakeSPI, new_adxl366, new_adxl366_spi, new_adxl367

from adxl36x import (
    _ACT_INACT_CTL_ACT_MASK,
    _ACT_INACT_CTL_INACT_SHIFT,
    _ACTIVITY_ENABLE,
    _REFERENCED_ACTIVITY_ENABLE,
    _REG_ACT_INACT_CTL,
    _REG_AXIS_MASK,
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
    _REG_X_OFFSET,
    _REG_X_SENS,
    _REG_XDATA_H,
    _REVID_ADXL366,
    _REVID_ADXL367,
    _SELF_TEST_SAMPLE_COUNT,
    _THRESHOLD_MAX,
    ADXL366,
    DataRate,
    FIFOFormat,
    FIFOMode,
    LinkLoopMode,
    OpMode,
    Range,
    TapAxis,
    WakeupRate,
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


def test_wakeup_mode_roundtrip(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.wakeup_mode = True
    assert adxl366.wakeup_mode is True
    assert fake_i2c.registers[0x2D] & 0x08
    adxl366.wakeup_mode = False
    assert adxl366.wakeup_mode is False


def test_wakeup_rate_roundtrip_and_register_bits(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.wakeup_rate = WakeupRate.RATE_3_SPS
    assert adxl366.wakeup_rate == WakeupRate.RATE_3_SPS
    assert (fake_i2c.registers[0x39] >> 6) & 0x03 == WakeupRate.RATE_3_SPS


def test_wakeup_rate_setter_rejects_invalid_value(adxl366: ADXL366) -> None:
    with pytest.raises(ValueError, match="WakeupRate"):
        adxl366.wakeup_rate = 7


def test_autosleep_roundtrip(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.autosleep = True
    assert adxl366.autosleep is True
    assert fake_i2c.registers[0x2D] & 0x04
    adxl366.autosleep = False
    assert adxl366.autosleep is False


def test_keep_alive_timer_defaults_to_off(adxl366: ADXL366) -> None:
    assert adxl366.keep_alive_timer is None


def test_keep_alive_timer_roundtrip_and_register_bits(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.keep_alive_timer = 0.16
    assert adxl366.keep_alive_timer == pytest.approx(0.16)
    assert fake_i2c.registers[0x39] & 0x1F == 1

    adxl366.keep_alive_timer = 1.28
    assert adxl366.keep_alive_timer == pytest.approx(1.28)
    assert fake_i2c.registers[0x39] & 0x1F == 4


def test_keep_alive_timer_none_disables(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.keep_alive_timer = 1.28
    adxl366.keep_alive_timer = None
    assert adxl366.keep_alive_timer is None
    assert fake_i2c.registers[0x39] & 0x1F == 0


def test_keep_alive_timer_clamps_to_max_code(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.keep_alive_timer = 1_000_000
    assert fake_i2c.registers[0x39] & 0x1F == 20


def test_keep_alive_timer_rejects_non_positive(adxl366: ADXL366) -> None:
    with pytest.raises(ValueError, match="period_seconds"):
        adxl366.keep_alive_timer = 0

    with pytest.raises(ValueError, match="period_seconds"):
        adxl366.keep_alive_timer = -1


def test_keep_alive_timer_preserves_wakeup_rate_bits(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.wakeup_rate = WakeupRate.RATE_3_SPS
    adxl366.keep_alive_timer = 1.28
    assert adxl366.wakeup_rate == WakeupRate.RATE_3_SPS


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


# -- offsets / sensitivity trim --


def test_offset_roundtrip(adxl366: ADXL366) -> None:
    adxl366.offset = (1, 2, 3)
    assert adxl366.offset == (1, 2, 3)


def test_offset_roundtrip_negative(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.offset = (-16, -1, 15)
    assert adxl366.offset == (-16, -1, 15)
    assert fake_i2c.registers[_REG_X_OFFSET] == 0x10
    assert fake_i2c.registers[_REG_X_OFFSET + 1] == 0x1F


def test_offset_rejects_out_of_range(adxl366: ADXL366) -> None:
    with pytest.raises(ValueError, match="offset"):
        adxl366.offset = (0, 0, 32)


def test_offset_rejects_below_signed_range(adxl366: ADXL366) -> None:
    with pytest.raises(ValueError, match="offset"):
        adxl366.offset = (0, 0, -17)


def test_sens_roundtrip(adxl366: ADXL366) -> None:
    adxl366.sens = (1, 2, 3)
    assert adxl366.sens == (1, 2, 3)


def test_sens_roundtrip_negative(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.sens = (-32, -1, 31)
    assert adxl366.sens == (-32, -1, 31)
    assert fake_i2c.registers[_REG_X_SENS] == 0x20
    assert fake_i2c.registers[_REG_X_SENS + 1] == 0x3F


def test_sens_rejects_out_of_range(adxl366: ADXL366) -> None:
    with pytest.raises(ValueError, match="sens"):
        adxl366.sens = (0, 0, 32)


def test_sens_rejects_below_signed_range(adxl366: ADXL366) -> None:
    with pytest.raises(ValueError, match="sens"):
        adxl366.sens = (0, 0, -33)


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


def test_link_loop_mode_roundtrip_and_register_bits(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.link_loop_mode = LinkLoopMode.LOOPED
    assert adxl366.link_loop_mode == LinkLoopMode.LOOPED
    assert (fake_i2c.registers[_REG_ACT_INACT_CTL] >> 4) & 0x03 == LinkLoopMode.LOOPED

    adxl366.link_loop_mode = LinkLoopMode.LINKED
    assert adxl366.link_loop_mode == LinkLoopMode.LINKED

    adxl366.link_loop_mode = LinkLoopMode.DEFAULT
    assert adxl366.link_loop_mode == LinkLoopMode.DEFAULT


def test_link_loop_mode_setter_rejects_invalid_value(adxl366: ADXL366) -> None:
    with pytest.raises(ValueError, match="LinkLoopMode"):
        adxl366.link_loop_mode = 0x2


def test_link_loop_mode_treats_undocumented_alias_as_default(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    """0b10 is an undocumented alias for DEFAULT (0b00) per the datasheet's ACT_INACT_CTL
    table - shouldn't surface as a fourth, unnamed mode value if something else set it.
    """
    fake_i2c.registers[_REG_ACT_INACT_CTL] = 0x2 << 4
    assert adxl366.link_loop_mode == LinkLoopMode.DEFAULT


def test_link_loop_mode_preserves_other_act_inact_ctl_bits(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.enable_motion_detection()
    adxl366.enable_inactivity_detection()
    adxl366.link_loop_mode = LinkLoopMode.LOOPED
    assert fake_i2c.registers[_REG_ACT_INACT_CTL] & _ACT_INACT_CTL_ACT_MASK == _ACTIVITY_ENABLE
    assert (fake_i2c.registers[_REG_ACT_INACT_CTL] & 0x0C) >> _ACT_INACT_CTL_INACT_SHIFT == _ACTIVITY_ENABLE


# -- events --


def test_events_reflects_status_bits(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    fake_i2c.registers[_REG_STATUS] = 0b0001_0001  # DATA_RDY + ACT
    events = adxl366.events
    assert events["data_ready"] is True
    assert events["motion"] is True
    assert events["inactivity"] is False
    assert events["fifo_overrun"] is False


def test_events_reflects_status_2_and_status_3_bits(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    fake_i2c.registers[0x45] = 0b0000_0011  # TAP_ONE + TAP_TWO
    fake_i2c.registers[0x46] = 0b0000_0001  # PEDOMETER_OVERFLOW
    events = adxl366.events
    assert events["single_tap"] is True
    assert events["double_tap"] is True
    assert events["pedometer_overflow"] is True
    assert events["temp_adc_low"] is False
    assert events["fuse_error"] is False


# -- interrupt mapping --


def test_map_interrupt_writes_expected_bytes(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.map_interrupt(1, {"data_ready", "motion", "double_tap"})
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


# -- tap detection --


def test_enable_tap_detection_single_writes_expected_registers(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.enable_tap_detection(threshold=40, duration=0.005)
    assert fake_i2c.registers[0x2F] == 40  # TAP_THRESH
    assert fake_i2c.registers[0x30] == 8  # TAP_DUR: 0.005s / 625us = 8
    assert fake_i2c.registers[0x31] == 0  # TAP_LATENT: 0 disables double-tap


def test_enable_tap_detection_double_writes_latency_and_window(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.enable_tap_detection(double_tap=True, latency=0.025, window=0.05)
    assert fake_i2c.registers[0x31] == 20  # TAP_LATENT: 0.025s / 1.25ms
    assert fake_i2c.registers[0x32] == 40  # TAP_WINDOW: 0.05s / 1.25ms


def test_enable_tap_detection_rejects_out_of_range_threshold(adxl366: ADXL366) -> None:
    with pytest.raises(ValueError, match="threshold"):
        adxl366.enable_tap_detection(threshold=300)


def test_disable_tap_detection_zeroes_threshold(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.enable_tap_detection(threshold=40)
    adxl366.disable_tap_detection()
    assert fake_i2c.registers[0x2F] == 0


# -- axis masking --


def test_tap_axis_roundtrip_and_register_bits(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.tap_axis = TapAxis.Z_AXIS
    assert adxl366.tap_axis == TapAxis.Z_AXIS
    assert (fake_i2c.registers[_REG_AXIS_MASK] >> 4) & 0x03 == TapAxis.Z_AXIS


def test_tap_axis_setter_rejects_invalid_value(adxl366: ADXL366) -> None:
    with pytest.raises(ValueError, match="TapAxis"):
        adxl366.tap_axis = 0x3


def test_blocked_axes_defaults_to_empty(adxl366: ADXL366) -> None:
    assert adxl366.blocked_axes == frozenset()


def test_blocked_axes_roundtrip_and_register_bits(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.blocked_axes = {"x", "z"}
    assert adxl366.blocked_axes == frozenset({"x", "z"})
    assert fake_i2c.registers[_REG_AXIS_MASK] & 0x07 == 0x05

    adxl366.blocked_axes = set()
    assert adxl366.blocked_axes == frozenset()


def test_blocked_axes_rejects_unknown_axis_name(adxl366: ADXL366) -> None:
    with pytest.raises(ValueError, match="unknown axis"):
        adxl366.blocked_axes = {"w"}


def test_blocked_axes_preserves_tap_axis_bits(adxl366: ADXL366, fake_i2c: FakeI2C) -> None:
    adxl366.tap_axis = TapAxis.Y_AXIS
    adxl366.blocked_axes = {"y"}
    assert adxl366.tap_axis == TapAxis.Y_AXIS


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
