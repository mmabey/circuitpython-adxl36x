"""Run the non-interactive hardware sanity checks against real ADXL366/ADXL367 silicon
over SPI.

Edit the SPI/CS construction below to match your wiring. See `hw_checks.py` for what each
check actually verifies.
"""

import board
import busio
import digitalio
from hw_checks import run_all

from adxl36x import ADXL366

spi = busio.SPI(clock=board.SCK, MOSI=board.MOSI, MISO=board.MISO)  # or your own pins
cs = digitalio.DigitalInOut(board.D26)  # edit to match your CS wiring
accel = ADXL366.from_spi(spi, cs)

run_all(accel)
