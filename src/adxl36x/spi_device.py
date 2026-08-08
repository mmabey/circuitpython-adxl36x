from typing import TYPE_CHECKING

from adafruit_bus_device.spi_device import SPIDevice as _SPIDevice

if TYPE_CHECKING:
    from busio import SPI
    from digitalio import DigitalInOut


class SPIDevice(_SPIDevice):
    """`SPIDevice` with a correctly-typed `__exit__`. See `_I2CDevice`."""

    spi: "SPI"
    chip_select: "DigitalInOut | None"
    cs_active_value: bool

    def __exit__(self, *exc_info: object) -> None:
        # extra_clocks handling is intentionally omitted: we never construct
        # this with a nonzero extra_clocks, so upstream's post-CS clock-out
        # step would always be a no-op here.
        if self.chip_select:
            self.chip_select.value = not self.cs_active_value
        self.spi.unlock()
