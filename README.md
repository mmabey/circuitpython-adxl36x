# circuitpython-adxl36x

[![PyPI](https://img.shields.io/pypi/v/circuitpython-adxl36x)](https://pypi.org/project/circuitpython-adxl36x/)
[![Python versions](https://img.shields.io/pypi/pyversions/circuitpython-adxl36x)](https://pypi.org/project/circuitpython-adxl36x/)
[![License: MIT](https://img.shields.io/pypi/l/circuitpython-adxl36x)](LICENSE)
[![CI](https://github.com/mmabey/circuitpython-adxl36x/actions/workflows/ci.yml/badge.svg)](https://github.com/mmabey/circuitpython-adxl36x/actions/workflows/ci.yml)
[![Downloads](https://img.shields.io/pypi/dm/circuitpython-adxl36x)](https://pypi.org/project/circuitpython-adxl36x/)
[![Ruff](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json)](https://github.com/astral-sh/ruff)

CircuitPython driver for the Analog Devices [ADXL366](https://www.analog.com/en/products/adxl366.html) and
[ADXL367](https://www.analog.com/en/products/adxl367.html) ultra-low-power 3-axis MEMS accelerometers.

`ADXL367` implements the full shared register map and is the base class. `ADXL366` subclasses it, adding the pedometer,
Z-axis nonlinearity compensation, and activity/inactivity reference readback - the three features the ADXL367 silicon
doesn't implement. Both support SPI and I2C.

## Relationship to `adafruit_adxl34x`

This library's public API deliberately mirrors the shape of [Adafruit's
`adafruit_adxl34x`](https://github.com/adafruit/Adafruit_CircuitPython_ADXL34x) driver for the ADXL34x family
(`.acceleration`, `.events`, `enable_tap_detection()`, `enable_motion_detection()`, `data_rate`) so that code written
against a 34x-family chip needs minimal changes to run against a 366/367. The two chip families are **not**
register-compatible — only the driver's Python API shape is shared. Notable differences:

- `range` is named `g_range` here instead — `range` shadows the Python builtin, and compatibility with the ADXL34x API
  takes a back seat to that on this one property.
- No `enable_freefall_detection()` — the 366/367 has no freefall-detection hardware.
- `Range` tops out at `RANGE_8_G` — the 366/367 doesn't support ±16g.
- `DataRate` only has 6 discrete rates (12.5 Hz-400 Hz) vs. the 34x's 16.
- `.events` includes many more keys (FIFO, pedometer, ADC/temperature thresholds) since the 366/367 exposes far more
  interrupt sources.

## Supported features

✅ Supported &nbsp;&nbsp; 🚫 Not present on this chip &nbsp;&nbsp; 🚧 Not yet supported by this driver

| Feature                                              | ADXL367 | ADXL366 |
| ----------------------------------------------------- | :-----: | :-----: |
| Acceleration (raw + scaled `m/s^2`)                  |   ✅    |   ✅    |
| `g_range` / `data_rate`                              |   ✅    |   ✅    |
| Power mode (standby / measurement)                   |   ✅    |   ✅    |
| Wake-up mode (`wakeup_mode`, `wakeup_rate`)           |   ✅    |   ✅    |
| Linked/looped activity mode (`link_loop_mode`)       |   ✅    |   ✅    |
| Autosleep (`autosleep`)                              |   ✅    |   ✅    |
| Motion (activity) detection                          |   ✅    |   ✅    |
| Inactivity detection (see note below on free fall)   |   ✅    |   ✅    |
| Offset trim (`offset`)                               |   ✅    |   ✅    |
| Sensitivity trim (`sens`)                             |   ✅    |   ✅    |
| Axis masking (`tap_axis`, `blocked_axes`)            |   ✅    |   ✅    |
| Keep-alive timer (`keep_alive_timer`)                |   ✅    |   ✅    |
| FIFO (oldest-saved / stream / triggered)             |   ✅    |   ✅    |
| Temperature sensor                                   |   ✅    |   ✅    |
| External ADC value (`adc_value`)                     |   ✅    |   ✅    |
| Self-test (`self_test()`)                            |   ✅    |   ✅    |
| Tap detection (single/double)                        |   ✅    |   ✅    |
| Interrupt mapping (`map_interrupt`) / `.events`       |   ✅    |   ✅    |
| Z-axis nonlinearity compensation                      |   🚫    |   ✅    |
| Activity/inactivity reference readback                |   🚫    |   ✅    |
| Pedometer (`pedometer_enabled`, `steps`)              |   🚫    |   ✅    |
| Noise modes (low-noise / ultra-low-noise)             |   🚧    |   🚧    |
| External ADC as activity/inactivity threshold source  |   🚧    |   🚧    |
| External clock                                        |   🚧    |   🚧    |
| External trigger                                      |   🚧    |   🚧    |

**Free fall** isn't a separate feature/method - it's a documented usage pattern of absolute (non-referenced) inactivity
detection (`enable_inactivity_detection(referenced=False)`), already fully supported.

The 🚧 rows are real hardware features (confirmed present on both chips, same register addresses, per each chip's own
datasheet) that this driver doesn't expose yet - they need either extra wiring/signal sources not on the current
breadboard (external ADC input, external clock, external trigger) or aren't yet confirmed as needed by any downstream
project (noise modes). Contributions welcome.

## Installation

On a CircuitPython board, copy `src/adxl36x/` into `CIRCUITPY/lib/adxl36x/`. No other bundle libraries are required -
this driver talks to `busio.I2C`/`busio.SPI` directly.

For boards tight on RAM/flash, or if raw source ever fails to compile on-device, each [GitHub
release](https://github.com/mmabey/circuitpython-adxl36x/releases) also attaches a precompiled `.mpy` bundle
(`lib/adxl36x/__init__.mpy`) - unzip it and copy `lib/adxl36x/` into `CIRCUITPY/lib/` instead. It's compatible with
any CircuitPython release sharing the same `.mpy` bytecode version (stable across CircuitPython 9.0.0+ as of this
writing) - not just the exact version it was built against.

For host-side development (type checking, tests) in another project:

```bash
uv add --editable ~/src/circuitpython-adxl36x
```

## Hardware validation setup

The `examples/hw_*.py` scripts and `examples/hw_checks.py` exercise real silicon and are set up to run against a
`doit_esp32_devkit_v1` board wired to an [EVAL-ADXL366Z](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/eval-adxl366z.html)
breakout, over either I2C or SPI:

**I2C** (`hw_sanity_check.py` / `hw_interactive_check.py` - edit their `board.I2C()` call to
`busio.I2C(board.D26, board.D33)` to match these pins):

| Signal | GPIO |
| ------ | ---- |
| SDA    | 33   |
| SCL    | 26   |
| INT1   | 27   |
| INT2   | 14   |

The EVAL-ADXL366Z has no onboard I2C pull-ups - add external 4.7kOhm pull-ups from SDA and SCL
to 3.3V.

**SPI** (`hw_spi_sanity_check.py` / `hw_spi_interactive_check.py`):

| Signal | GPIO |
| ------ | ---- |
| SCK    | 18   |
| MOSI   | 23   |
| MISO   | 19   |
| CS     | 26   |
| INT1   | 27   |
| INT2   | 14   |

INT1/INT2 are only needed for the interactive checks (motion/inactivity/tap interrupts, autosleep wake cycle,
orientation, pedometer) - the non-interactive sanity checks only need the data bus.

## Development

```bash
uv sync
uv run ruff check --fix
uv run ruff format
uv run ty check
uv run pytest
```
