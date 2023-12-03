"""
`pmw3901_spi`
================================================================================

SPI driven CircuitPython driver for pmw3901 ported from the pimoroni pmw3901-python driver:
https://github.com/pimoroni/pmw3901-python

* Author(s): N/A

Implementation Notes

**Hardware:**

* Intended to be used with adafruit FT232H General Purpose USB to GPIO, SPI, I2C bridge device:
  https://www.adafruit.com/product/2264

**Software and Dependencies:**

* Adafruit Blinka:
  https://github.com/adafruit/Adafruit_Blinka

"""
import time

try:
    from digitalio import DigitalInOut
    from busio import SPI
except ImportError:
    pass

from adafruit_bus_device.spi_device import SPIDevice

__version__ = "0.0.0+auto.0"
__repo__ = "WIP"

WAIT = -1

BG_CS_FRONT_BCM = 7
BG_CS_BACK_BCM = 8

REG_ID = 0x00
REG_DATA_READY = 0x02
REG_MOTION_BURST = 0x16
REG_POWER_UP_RESET = 0x3a
REG_ORIENTATION = 0x5b
REG_RESOLUTION = 0x4e  # PAA5100 only

REG_RAWDATA_GRAB = 0x58
REG_RAWDATA_GRAB_STATUS = 0x59


class Pmw3901_SPI:
    """
    WIP
    """

    def __init__(
            self,
            spi_bus: SPI,
            cs: DigitalInOut,
            frequency: int = 400000,
    ):  # pylint: disable=invalid-name

        # SPI setup
        self._spi = SPIDevice(spi_bus, chip_select=cs, baudrate=frequency)
        self._out_buffer = bytearray(3)
        self._in_buffer = bytearray(3)

        # toggle chip select
        cs.value = False
        time.sleep(0.05)
        cs.value = True

        # write out POWER UP RESET
        with self._spi as spi:
            # self.spi_dev.xfer2([register | 0x80, value])
            _output = bytearray(2)
            _output.append(REG_POWER_UP_RESET)
            _output.append(0x5a)
            spi.write(_output)
        time.sleep(0.02)
        _input = bytearray(5)
        for offset in range(5):
            status = 0
            spi.readinto(status, write_value=(REG_DATA_READY + offset))
            _input.append(status)
        print(f"got status {_input}")

        self._secret_sauce()

        product_id, revision = self.get_id()
        if product_id != 0x49 or revision != 0x00:
            raise RuntimeError(
                "Invalid Product ID or Revision for PMW3901: 0x{:02x}/0x{:02x}".format(product_id, revision))
        else:
            print(f"got product id {product_id} and revision {revision}")

    def get_id(self):

        _output = bytearray(2)
        with self._spi as spi:
            spi.readinto(_output, write_value=REG_ID)

        return _output[0], _output[1]

    def _secret_sauce(self):
        """Write the secret sauce registers.

        Don't ask what these do, the datasheet refuses to explain.

        They are some proprietary calibration magic.

        """

        with self._spi as spi:

            # Config phase 1
            _magic_buff = bytearray([
                0x7f, 0x00,
                0x55, 0x01,
                0x50, 0x07,

                0x7f, 0x0e,
                0x43, 0x10
            ])
            spi.write(_magic_buff)

            status = 0
            spi.readinto(status, write_value=0x67)
            if status & 0b10000000:
                spi.write([0x48, 0x04])
            else:
                spi.write([0x48, 0x02])

            # Config phase 2
            _magic_buff = bytearray([
                0x7f, 0x00,
                0x51, 0x7b,

                0x50, 0x00,
                0x55, 0x00,
                0x7f, 0x0E
            ])
            spi.write(_magic_buff)

            spi.readinto(status, write_value=0x73)
            if status == 0x00:
                _c1 = 0
                spi.readinto(_c1, write_value=0x70)
                _c2 = 0
                spi.readinto(_c2, write_value=0x71)

                if _c1 <= 28:
                    _c1 += 14
                if _c1 > 28:
                    _c1 += 11

                _c1 = max(0, min(0x3F, _c1))
                _c2 = (_c2 * 45) // 100

                _magic_buff = bytearray([
                    0x7f, 0x00,
                    0x61, 0xad,
                    0x51, 0x70,
                    0x7f, 0x0e
                ])
                spi.write(_magic_buff)
                spi.write([0x70, _c1])
                spi.write([0x71, _c2])

            # Config phase 3
            _magic_buff = bytearray([
                0x7f, 0x00,
                0x61, 0xad,
                0x7f, 0x03,
                0x40, 0x00,
                0x7f, 0x05,

                0x41, 0xb3,
                0x43, 0xf1,
                0x45, 0x14,
                0x5b, 0x32,
                0x5f, 0x34,
                0x7b, 0x08,
                0x7f, 0x06,
                0x44, 0x1b,
                0x40, 0xbf,
                0x4e, 0x3f,
                0x7f, 0x08,
                0x65, 0x20,
                0x6a, 0x18,

                0x7f, 0x09,
                0x4f, 0xaf,
                0x5f, 0x40,
                0x48, 0x80,
                0x49, 0x80,

                0x57, 0x77,
                0x60, 0x78,
                0x61, 0x78,
                0x62, 0x08,
                0x63, 0x50,
                0x7f, 0x0a,
                0x45, 0x60,
                0x7f, 0x00,
                0x4d, 0x11,

                0x55, 0x80,
                0x74, 0x21,
                0x75, 0x1f,
                0x4a, 0x78,
                0x4b, 0x78,

                0x44, 0x08,
                0x45, 0x50,
                0x64, 0xff,
                0x65, 0x1f,
                0x7f, 0x14,
                0x65, 0x67,
                0x66, 0x08,
                0x63, 0x70,
                0x7f, 0x15,
                0x48, 0x48,
                0x7f, 0x07,
                0x41, 0x0d,
                0x43, 0x14,

                0x4b, 0x0e,
                0x45, 0x0f,
                0x44, 0x42,
                0x4c, 0x80,
                0x7f, 0x10,

                0x5b, 0x02,
                0x7f, 0x07,
                0x40, 0x41,
                0x70, 0x00,
                WAIT, 0x0A,  # Sleep for 10ms

                0x32, 0x44,
                0x7f, 0x07,
                0x40, 0x40,
                0x7f, 0x06,
                0x62, 0xf0,
                0x63, 0x00,
                0x7f, 0x0d,
                0x48, 0xc0,
                0x6f, 0xd5,
                0x7f, 0x00,

                0x5b, 0xa0,
                0x4e, 0xa8,
                0x5a, 0x50,
                0x40, 0x80,
                WAIT, 0xF0,

                0x7f, 0x14,  # Enable LED_N pulsing
                0x6f, 0x1c,
                0x7f, 0x00
            ])
            spi.write(_magic_buff)


