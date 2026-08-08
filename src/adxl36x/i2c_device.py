from typing import TYPE_CHECKING

from adafruit_bus_device.i2c_device import I2CDevice as _I2CDevice

if TYPE_CHECKING:
    from busio import I2C


class I2CDevice(_I2CDevice):
    """`I2CDevice` with a correctly-typed `__exit__`.

    `adafruit_bus_device-stubs` declares `__exit__(self) -> None` (zero
    args), which doesn't match the real 3-argument context-manager protocol
    and makes every `with I2CDevice(...) as ...:` a type error. The real
    `__exit__` only unlocks the bus and always returns a falsy value, so we
    reimplement it directly here instead of delegating to the mistyped
    parent method.
    """

    i2c: "I2C"

    def __exit__(self, *exc_info: object) -> None:
        self.i2c.unlock()
