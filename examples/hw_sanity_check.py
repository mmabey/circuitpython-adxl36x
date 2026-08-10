"""Run the non-interactive hardware sanity checks against real ADXL366/ADXL367 silicon.

Edit the I2C construction below to match your wiring (or swap in `ADXL366.from_spi(...)`
for an SPI connection). See `hw_checks.py` for what each check actually verifies.
"""

import board
from hw_checks import run_all

from adxl36x import ADXL366

i2c = board.I2C()  # or busio.I2C(board.SCL, board.SDA) with your own pins
accel = ADXL366(i2c)

run_all(accel)
