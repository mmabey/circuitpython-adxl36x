# circuitpython-adxl36x

CircuitPython driver for the Analog Devices [ADXL366](https://www.analog.com/en/products/adxl366.html)
and [ADXL367](https://www.analog.com/en/products/adxl367.html) ultra-low-power 3-axis MEMS
accelerometers.

`ADXL366` is the full-featured driver class; `ADXL367` subclasses it and disables the
features the ADXL367 silicon doesn't implement (pedometer, Z-axis nonlinearity compensation,
reference readback). Both support SPI and I2C.

## Relationship to `adafruit_adxl34x`

This library's public API deliberately mirrors the shape of
[Adafruit's `adafruit_adxl34x`](https://github.com/adafruit/Adafruit_CircuitPython_ADXL34x)
driver for the ADXL34x family (`.acceleration`, `.events`, `enable_tap_detection()`,
`enable_motion_detection()`, `range`, `data_rate`) so that code written against a 34x-family
chip needs minimal changes to run against a 366/367. The two chip families are **not**
register-compatible — only the driver's Python API shape is shared. Notable differences:

- No `enable_freefall_detection()` — the 366/367 has no freefall-detection hardware.
- `Range` tops out at `RANGE_8_G` — the 366/367 doesn't support ±16g.
- `DataRate` only has 6 discrete rates (12.5 Hz-400 Hz) vs. the 34x's 16.
- `.events` includes many more keys (FIFO, pedometer, ADC/temperature thresholds) since the
  366/367 exposes far more interrupt sources.

## Installation

On a CircuitPython board, copy `src/adxl36x/` into `CIRCUITPY/lib/adxl36x/`, along with the
`adafruit_bus_device` bundle library.

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
