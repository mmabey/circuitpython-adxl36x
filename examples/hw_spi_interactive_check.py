"""Run the interactive hardware checks (motion/inactivity via INT1/INT2, plus the
tilt/orientation and pedometer checks) against real ADXL366/ADXL367 silicon over SPI.

Unlike `hw_spi_sanity_check.py`, this requires INT1 and INT2 wired to GPIOs - edit the
pin assignments below to match your wiring - and needs you to physically move, still,
and reorient the board when prompted.
"""

import board
import busio
import digitalio
from hw_checks import run_interactive

from adxl36x import ADXL366

spi = busio.SPI(clock=board.SCK, MOSI=board.MOSI, MISO=board.MISO)  # or your own pins
cs = digitalio.DigitalInOut(board.D26)  # edit to match your CS wiring
accel = ADXL366.from_spi(spi, cs)

int1 = digitalio.DigitalInOut(board.D27)  # edit to match your INT1 wiring
int1.switch_to_input()
int2 = digitalio.DigitalInOut(board.D14)  # edit to match your INT2 wiring
int2.switch_to_input()

run_interactive(accel, int1, int2)
