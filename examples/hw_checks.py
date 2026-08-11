"""Hardware-in-the-loop checks for ADXL366/ADXL367, exercised against real silicon.

Host-side `tests/` already covers register-level decode/scale/validation logic against
a fake bus; these checks are for everything that only exists on real hardware. Each
`check_*` function takes an already-constructed accelerometer instance - built over I2C
or SPI, on whatever pins the caller wired up - so the same checks run unmodified from
`hw_sanity_check.py` (a generic board) or from a project's own wiring-specific entry
point.
"""

import time

try:
    from typing import TYPE_CHECKING
except ImportError:
    TYPE_CHECKING = False  # ty: ignore[invalid-assignment]

from adxl36x import ADXL366, ADXL367, DataRate, FIFOFormat, FIFOMode, LinkLoopMode, Range, WakeupRate

if TYPE_CHECKING:
    from digitalio import DigitalInOut

_STANDARD_GRAVITY = 9.80665
_GRAVITY_MAGNITUDE_TOLERANCE_MPS2 = 1.5
_MIN_PLAUSIBLE_TEMP_C = -20.0
_MAX_PLAUSIBLE_TEMP_C = 60.0
_MOTION_WAIT_TIMEOUT_S = 10.0
_INACTIVITY_WAIT_TIMEOUT_S = 15.0
_INACTIVITY_TIME_S = 2
_POLL_INTERVAL_S = 0.05
_REFERENCE_SETTLE_S = 1.0
_INACTIVITY_SETTLE_S = 3.0
_MOTION_THRESHOLD = 300
_FIFO_SAMPLE_SETS = 30
_FIFO_FILL_WAIT_S = 0.5
_FIFO_RAW_MAGNITUDE_MIN = 500  # comfortably above noise floor / stuck-at-zero
_FIFO_RAW_MAGNITUDE_MAX = 8191  # full-scale 14-bit signed magnitude ceiling
_PEDOMETER_STEP_TARGET = 8  # datasheet: steps only certify in groups of 8+ consecutive valid steps
_PEDOMETER_WAIT_TIMEOUT_S = 30.0
_PEDOMETER_POLL_INTERVAL_S = 0.5
_ORIENTATION_SETTLE_S = 4.0
_ORIENTATION_AXES = ("X", "Y", "Z")
_TAP_WAIT_TIMEOUT_S = 10.0
_WAKEUP_MIN_INTERVAL_S = 0.2  # comfortably below RATE_3_SPS's ~320ms, above normal-mode noise
_WAKEUP_SAMPLE_COUNT = 3
_WAKEUP_TEST_TIMEOUT_S = 5.0
_WAKEUP_POLL_INTERVAL_S = 0.02


def _report(name: str, passed: bool, detail: str = "") -> bool:
    status = "PASS" if passed else "FAIL"
    suffix = f" - {detail}" if detail else ""
    print(f"[{status}] {name}{suffix}")
    return passed


def check_identification(accel: ADXL367) -> bool:
    """Construction already validated DEVID/PARTID/REVID; confirm serial_number reads too."""
    serial = accel.serial_number
    return _report("identification", serial != 0, f"serial={serial:#010x}")


def check_range_and_rate_roundtrip(accel: ADXL367) -> bool:
    """Confirm g_range/data_rate writes actually land in the real FILTER_CTL register."""
    results = []
    for value in (Range.RANGE_2_G, Range.RANGE_4_G, Range.RANGE_8_G):
        accel.g_range = value
        results.append(accel.g_range == value)
    for value in (DataRate.RATE_12_5_HZ, DataRate.RATE_100_HZ, DataRate.RATE_400_HZ):
        accel.data_rate = value
        results.append(accel.data_rate == value)
    accel.g_range = Range.RANGE_2_G
    accel.data_rate = DataRate.RATE_100_HZ
    return _report("range/rate roundtrip", all(results))


def check_offset_roundtrip(accel: ADXL367) -> bool:
    original = accel.offset
    accel.offset = (3, 5, 7)
    ok = accel.offset == (3, 5, 7)
    accel.offset = original
    return _report("offset roundtrip", ok, f"restored to {original}")


def check_temperature(accel: ADXL367) -> bool:
    temp_c = accel.temperature
    ok = _MIN_PLAUSIBLE_TEMP_C <= temp_c <= _MAX_PLAUSIBLE_TEMP_C
    return _report("temperature plausibility", ok, f"{temp_c:.1f}C")


def check_self_test(accel: ADXL367) -> bool:
    return _report("self_test()", accel.self_test())


def check_at_rest_gravity(accel: ADXL367) -> bool:
    """With the board stationary, the acceleration vector's magnitude should read ~1g.

    Checking magnitude rather than individual axes keeps this orientation-independent -
    there's no way to guarantee a hand-wired setup is perfectly level, and the
    datasheet's own 0g offset spec (+/-150mg on X/Y, +/-250mg on Z) already allows more
    per-axis slop than would fit a "one axis at 1g, the other two at 0g" assumption.
    """
    x, y, z = accel.acceleration
    magnitude = (x**2 + y**2 + z**2) ** 0.5
    ok = abs(magnitude - _STANDARD_GRAVITY) < _GRAVITY_MAGNITUDE_TOLERANCE_MPS2
    detail = f"|a|={magnitude:.2f} m/s^2 (x={x:+.2f} y={y:+.2f} z={z:+.2f})"
    return _report("at-rest gravity", ok, detail)


def check_fifo(accel: ADXL367) -> bool:
    """Configure FIFO stream mode, let it fill briefly, and confirm entries decode to
    plausible x/y/z channel tags and magnitudes. No physical interaction needed.
    """
    accel.configure_fifo(mode=FIFOMode.STREAM, fifo_format=FIFOFormat.XYZ, sample_sets=_FIFO_SAMPLE_SETS)
    time.sleep(_FIFO_FILL_WAIT_S)
    entries = accel.fifo_entries
    samples = accel.read_fifo() if entries else []
    accel.configure_fifo(mode=FIFOMode.DISABLED)

    channels_ok = bool(samples) and all(channel in ("x", "y", "z") for channel, _ in samples)
    complete_sets = len(samples) % 3 == 0
    magnitude = None
    magnitude_ok = False
    if channels_ok and complete_sets and samples:
        x, y, z = (value for _, value in samples[:3])
        magnitude = (x**2 + y**2 + z**2) ** 0.5
        magnitude_ok = _FIFO_RAW_MAGNITUDE_MIN <= magnitude <= _FIFO_RAW_MAGNITUDE_MAX

    ok = entries > 0 and channels_ok and complete_sets and magnitude_ok
    detail = f"entries={entries} samples={len(samples)} first_triple_magnitude={magnitude}"
    return _report("FIFO streaming", ok, detail)


def check_wakeup_mode(accel: ADXL367) -> bool:
    """Confirm wake-up mode's slower sample cadence vs. normal measurement mode.

    No physical interaction needed. Enables wake-up mode at RATE_3_SPS (~320ms/sample)
    and times consecutive DATA_READY events - that interval is unambiguously longer
    than normal mode's sub-10ms cadence at any supported ODR, so a clean pass/fail
    doesn't need precise timing, just "clearly slower."
    """
    accel.wakeup_rate = WakeupRate.RATE_3_SPS
    accel.wakeup_mode = True
    try:
        _ = accel.acceleration  # discard any DATA_READY left over from normal mode
        timestamps = []
        deadline = time.monotonic() + _WAKEUP_TEST_TIMEOUT_S
        while len(timestamps) < _WAKEUP_SAMPLE_COUNT and time.monotonic() < deadline:
            if accel.events["data_ready"]:
                timestamps.append(time.monotonic())
                _ = accel.acceleration  # clears DATA_READY
            else:
                time.sleep(_WAKEUP_POLL_INTERVAL_S)
    finally:
        accel.wakeup_mode = False
    intervals = [timestamps[i + 1] - timestamps[i] for i in range(len(timestamps) - 1)]
    ok = len(intervals) >= _WAKEUP_SAMPLE_COUNT - 1 and all(i > _WAKEUP_MIN_INTERVAL_S for i in intervals)
    detail = f"intervals={[round(i, 3) for i in intervals]}"
    return _report("wake-up mode sample cadence", ok, detail)


def check_z_nonlinearity_compensation(accel: ADXL366) -> bool:
    """ADXL366-only: verify the Z-axis nonlinearity compensation flag roundtrips."""
    original = accel.z_nonlinearity_compensation
    accel.z_nonlinearity_compensation = not original
    toggled = accel.z_nonlinearity_compensation == (not original)
    accel.z_nonlinearity_compensation = original
    restored = accel.z_nonlinearity_compensation == original
    return _report("z_nonlinearity_compensation roundtrip", toggled and restored, f"restored to {original}")


def run_all(accel: ADXL367) -> bool:
    """Run every non-interactive check in sequence. Returns True only if all of them passed."""
    checks = (
        check_identification,
        check_range_and_rate_roundtrip,
        check_offset_roundtrip,
        check_temperature,
        check_self_test,
        check_at_rest_gravity,
        check_fifo,
        check_wakeup_mode,
    )
    results = [check(accel) for check in checks]
    if isinstance(accel, ADXL366):
        results.append(check_z_nonlinearity_compensation(accel))
    main_msg = f"{sum(results)}/{len(results)} checks passed"
    print(f"\n{main_msg}\n{'-' * len(main_msg)}\n")
    return all(results)


def _wait_for_pin(pin: "DigitalInOut", timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if pin.value:
            return True
        time.sleep(_POLL_INTERVAL_S)
    return False


def _wait_for_pin_low(pin: "DigitalInOut", timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not pin.value:
            return True
        time.sleep(_POLL_INTERVAL_S)
    return False


def check_motion_interrupt(accel: ADXL367, int1_pin: "DigitalInOut") -> bool:
    """Physically move/shake the board when prompted.

    Confirms activity detection end to end: real acceleration -> ACT_INACT_CTL ->
    STATUS register -> the event routed to the physical INT1 pin via map_interrupt().
    Uses referenced (not absolute) detection - per the datasheet, absolute detection
    with a sub-1g threshold triggers immediately from gravity alone on whichever axis
    is vertical, never actually waiting for real motion.
    """
    print("\nHold the board still for a moment (capturing reference)...")
    time.sleep(_REFERENCE_SETTLE_S)
    accel.map_interrupt(1, {"activity"})
    accel.enable_motion_detection(threshold=_MOTION_THRESHOLD, time_=0, referenced=True)
    try:
        print(f"\nMOVE/SHAKE THE BOARD NOW - waiting up to {_MOTION_WAIT_TIMEOUT_S:.0f}s...")
        pin_asserted = _wait_for_pin(int1_pin, _MOTION_WAIT_TIMEOUT_S)
        status_bit = accel.events["motion"]
    finally:
        accel.disable_motion_detection()
        accel.map_interrupt(1, set())
    ok = pin_asserted and status_bit
    return _report("motion interrupt (INT1)", ok, f"pin_asserted={pin_asserted} status_bit={status_bit}")


def check_tap_interrupt(accel: ADXL367, int1_pin: "DigitalInOut") -> bool:
    """Physically tap the board once, firmly, when prompted.

    Confirms tap detection end to end: a real tap -> TAP_THRESH/TAP_DUR comparison ->
    STATUS_2 -> the event routed to the physical INT1 pin via map_interrupt(). Reuses
    INT1 sequentially with `check_motion_interrupt` - fine since each call maps and
    then clears its own event before returning.
    """
    accel.map_interrupt(1, {"single_tap"})
    accel.enable_tap_detection()
    try:
        print(f"\nTAP THE BOARD ONCE, FIRMLY - waiting up to {_TAP_WAIT_TIMEOUT_S:.0f}s...")
        pin_asserted = _wait_for_pin(int1_pin, _TAP_WAIT_TIMEOUT_S)
        status_bit = accel.events["single_tap"]
    finally:
        accel.disable_tap_detection()
        accel.map_interrupt(1, set())
    ok = pin_asserted and status_bit
    return _report("tap interrupt (INT1)", ok, f"pin_asserted={pin_asserted} status_bit={status_bit}")


def check_autosleep_wake_cycle(accel: ADXL367, int1_pin: "DigitalInOut") -> bool:
    """Hold the board still for a bit, then move/shake it, when prompted.

    Confirms linked/looped mode + autosleep end to end: once configured, the device
    autonomously drops into wake-up mode on inactivity (AWAKE -> 0) and returns to
    measurement mode on activity (AWAKE -> 1), with no further host writes to
    POWER_CTL beyond the initial setup below. AWAKE is mapped to INT1 as a level
    signal - the datasheet's documented "motion switch" use case - and checked
    against both the physical pin and the STATUS register bit. Reuses INT1
    sequentially, same as `check_tap_interrupt`.
    """
    print("\nHold the board still for a moment (settling before autosleep test)...")
    time.sleep(_INACTIVITY_SETTLE_S)
    accel.enable_motion_detection(threshold=_MOTION_THRESHOLD, time_=0, referenced=True)
    accel.enable_inactivity_detection(threshold=50, time_=_INACTIVITY_TIME_S, referenced=True)
    accel.link_loop_mode = LinkLoopMode.LOOPED
    accel.autosleep = True
    accel.map_interrupt(1, {"awake"})
    try:
        # Not asserted on: the settling sleep above means the board is often already
        # stationary by the time LOOPED+autosleep engage, so the chip can legitimately
        # read "asleep" instantly rather than starting "awake" - the sleep/wake
        # transitions below are the real proof this works, not the starting value.
        initially_awake = accel.events["awake"]
        print(f"\nKEEP HOLDING STILL - waiting up to {_INACTIVITY_WAIT_TIMEOUT_S:.0f}s for autosleep...")
        fell_asleep = _wait_for_pin_low(int1_pin, _INACTIVITY_WAIT_TIMEOUT_S)
        asleep_status_bit = not accel.events["awake"]

        print(f"\nMOVE/SHAKE THE BOARD NOW - waiting up to {_MOTION_WAIT_TIMEOUT_S:.0f}s to wake...")
        woke_up = _wait_for_pin(int1_pin, _MOTION_WAIT_TIMEOUT_S)
        awake_status_bit = accel.events["awake"]
    finally:
        accel.autosleep = False
        accel.link_loop_mode = LinkLoopMode.DEFAULT
        accel.disable_motion_detection()
        accel.disable_inactivity_detection()
        accel.map_interrupt(1, set())

    ok = fell_asleep and asleep_status_bit and woke_up and awake_status_bit
    detail = (
        f"initially_awake={initially_awake} fell_asleep={fell_asleep}({asleep_status_bit}) "
        f"woke_up={woke_up}({awake_status_bit})"
    )
    return _report("autosleep wake/sleep cycle (INT1=AWAKE)", ok, detail)


def check_inactivity_interrupt(accel: ADXL367, int2_pin: "DigitalInOut") -> bool:
    """Hold the board completely still when prompted.

    Confirms inactivity detection end to end via the physical INT2 pin, mirroring
    `check_motion_interrupt`. Uses referenced detection for the same reason: absolute
    inactivity requires *every* axis to read below threshold, which the gravity-aligned
    axis never does, so it could never trigger regardless of actual stillness.
    """
    print("\nSet the board down and hold it still...")
    time.sleep(_INACTIVITY_SETTLE_S)
    accel.map_interrupt(2, {"inactivity"})
    accel.enable_inactivity_detection(threshold=50, time_=_INACTIVITY_TIME_S, referenced=True)
    try:
        print(f"\nKEEP HOLDING STILL - waiting up to {_INACTIVITY_WAIT_TIMEOUT_S:.0f}s...")
        pin_asserted = _wait_for_pin(int2_pin, _INACTIVITY_WAIT_TIMEOUT_S)
        status_bit = accel.events["inactivity"]
    finally:
        accel.disable_inactivity_detection()
        accel.map_interrupt(2, set())
    ok = pin_asserted and status_bit
    return _report("inactivity interrupt (INT2)", ok, f"pin_asserted={pin_asserted} status_bit={status_bit}")


def check_pedometer(accel: ADXL366) -> bool:
    """ADXL366-only: mimic walking and confirm `accel.steps` climbs.

    Per the datasheet's Pedometer section, the algorithm looks for a periodic sequence
    of peak/trough acceleration pairs (like footfalls) and only certifies steps in
    batches once 8 or more consecutive ones are detected - a handful of taps or a single
    shake never crosses that bar, since it doesn't produce a sustained periodic pattern.
    Unlike the motion/inactivity checks, the pedometer isn't wired to an interrupt pin -
    it's a free-running counter register - so this just enables it, resets the count, and
    polls `steps` directly instead of waiting on a `DigitalInOut`.
    """
    accel.pedometer_enabled = True
    accel.reset_steps()
    steps = 0
    try:
        print(
            "\nMIMIC WALKING WITH THE BOARD - hold it and swing/bounce it rhythmically "
            f"(like footsteps), at least {_PEDOMETER_STEP_TARGET} steps' worth - "
            f"waiting up to {_PEDOMETER_WAIT_TIMEOUT_S:.0f}s...",
        )
        deadline = time.monotonic() + _PEDOMETER_WAIT_TIMEOUT_S
        while time.monotonic() < deadline and steps < _PEDOMETER_STEP_TARGET:
            time.sleep(_PEDOMETER_POLL_INTERVAL_S)
            steps = accel.steps
    finally:
        accel.pedometer_enabled = False
    ok = steps >= _PEDOMETER_STEP_TARGET
    return _report("pedometer step count", ok, f"steps={steps}")


def _check_axis_orientation(accel: ADXL367, index: int, label: str) -> bool:
    print(
        f"\nOrient the board flat with its {label}-axis (per the board's silkscreen) "
        "pointing straight up, then hold still...",
    )
    time.sleep(_ORIENTATION_SETTLE_S)
    values = accel.acceleration
    target = values[index]
    others = [abs(v) for i, v in enumerate(values) if i != index]
    magnitude_ok = abs(target - _STANDARD_GRAVITY) < _GRAVITY_MAGNITUDE_TOLERANCE_MPS2
    dominant = target > max(others)
    detail = f"x={values[0]:+.2f} y={values[1]:+.2f} z={values[2]:+.2f} m/s^2"
    return _report(f"{label}-axis orientation (up)", magnitude_ok and dominant, detail)


def check_orientation(accel: ADXL367) -> bool:
    """Confirm each axis's sign matches the board's silkscreen labeling.

    For each axis in turn, orient the board with that axis's printed + direction
    pointing up and confirm it reads ~+1g while clearly dominating the other two -
    catches a mirrored/rotated placement or a mislabeled silkscreen print, which
    magnitude-only checks like `check_at_rest_gravity` can't distinguish. A wrong-way
    placement reads strongly negative on the target axis, so it fails the dominance
    check naturally rather than needing a separate sign check.
    """
    results = [_check_axis_orientation(accel, i, label) for i, label in enumerate(_ORIENTATION_AXES)]
    main_msg = f"{sum(results)}/{len(results)} orientation checks passed"
    print(f"\n{main_msg}\n{'-' * len(main_msg)}\n")
    return all(results)


def run_interactive(accel: ADXL367, int1_pin: "DigitalInOut", int2_pin: "DigitalInOut") -> bool:
    """Run the checks that require physically handling the board, in sequence.

    Returns True only if all of them passed. Requires INT1/INT2 wired to GPIOs, unlike
    everything in `run_all()`.
    """
    results = [
        check_motion_interrupt(accel, int1_pin),
        check_inactivity_interrupt(accel, int2_pin),
        check_tap_interrupt(accel, int1_pin),
        check_autosleep_wake_cycle(accel, int1_pin),
        check_orientation(accel),
    ]
    if isinstance(accel, ADXL366):
        results.append(check_pedometer(accel))
    main_msg = f"{sum(results)}/{len(results)} interactive checks passed"
    print(f"\n{main_msg}\n{'-' * len(main_msg)}\n")
    return all(results)
