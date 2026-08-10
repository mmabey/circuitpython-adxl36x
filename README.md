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

## Installation

On a CircuitPython board, copy `src/adxl36x/` into `CIRCUITPY/lib/adxl36x/`. No other bundle libraries are required -
this driver talks to `busio.I2C`/`busio.SPI` directly.

For host-side development (type checking, tests) in another project:

```bash
uv add --editable ~/src/circuitpython-adxl36x
```

## Development

```bash
uv sync
uv run ruff check --fix
uv run ruff format
uv run ty check
uv run pytest
```
