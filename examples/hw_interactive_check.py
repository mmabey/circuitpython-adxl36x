"""Run the interactive hardware checks (motion/inactivity via INT1/INT2) against real
ADXL366/ADXL367 silicon.

Unlike `hw_sanity_check.py`, this requires INT1 and INT2 wired to GPIOs - edit the pin
assignments below to match your wiring - and needs you to physically move and then
still the board when prompted.
"""

import board
import digitalio
from hw_checks import run_interactive

from adxl36x import ADXL366

i2c = board.I2C()
accel = ADXL366(i2c)

int1 = digitalio.DigitalInOut(board.D5)  # edit to match your INT1 wiring
int1.switch_to_input()
int2 = digitalio.DigitalInOut(board.D6)  # edit to match your INT2 wiring
int2.switch_to_input()

run_interactive(accel, int1, int2)
