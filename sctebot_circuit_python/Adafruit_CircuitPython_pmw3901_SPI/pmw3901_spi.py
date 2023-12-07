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
import struct
import time
import board

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
            cs: DigitalInOut = None,
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

    def set_rotation(self, degrees=0):
        """Set orientation of PMW3901 in increments of 90 degrees.

        :param degrees: rotation in multiple of 90 degrees

        """
        if degrees == 0:
            self.set_orientation(invert_x=True, invert_y=True, swap_xy=True)
        elif degrees == 90:
            self.set_orientation(invert_x=False, invert_y=True, swap_xy=False)
        elif degrees == 180:
            self.set_orientation(invert_x=False, invert_y=False, swap_xy=True)
        elif degrees == 270:
            self.set_orientation(invert_x=True, invert_y=False, swap_xy=False)
        else:
            raise TypeError("Degrees must be one of 0, 90, 180 or 270")

    def set_orientation(self, invert_x=False, invert_y=False, swap_xy=False):
        """Set orientation of PMW3901 manually.

        Swapping is performed before flipping.

        :param invert_x: invert the X axis
        :param invert_y: invert the Y axis
        :param swap_xy: swap the X/Y axes

        """
        value = 0
        if swap_xy:
            value |= 0b10000000
        if invert_y:
            value |= 0b01000000
        if invert_x:
            value |= 0b00100000

        # SPI WRITE REG_ORIENTATION, value
        with self._spi as spi:
            spi.write([REG_ORIENTATION, value])

    def get_motion(self, timeout=5):
        """Get motion data from PMW3901 using burst read.

        Reads 12 bytes sequentially from the PMW3901 and validates
        motion data against the SQUAL and Shutter_Upper values.

        Returns Delta X and Delta Y indicating 2d flow direction
        and magnitude.

        :param timeout: Timeout in seconds

        """
        t_start = time.time()
        while time.time() - t_start < timeout:
            with self._spi as spi:
                _data = bytearray(12)
                spi.readinto(_data, write_value=REG_MOTION_BURST)

                (_, dr, obs,
                 x, y, quality,
                 raw_sum, raw_max, raw_min,
                 shutter_upper, shutter_lower) = struct.unpack("<BBBhhBBBBB", _data)

                if dr & 0b10000000 and not (quality < 0x19 and shutter_upper == 0x1f):
                    return x, y

                time.sleep(0.01)

        raise RuntimeError("Timed out waiting for motion data.")

    def get_motion_slow(self, timeout=5):
        """Get motion data from PMW3901.

        Returns Delta X and Delta Y indicating 2d flow direction
        and magnitude.

        :param timeout: Timeout in seconds

        """
        t_start = time.time()
        while time.time() - t_start < timeout:
            with self._spi as spi:
                _data = bytearray(5)
                spi.readinto(_data, write_value=REG_DATA_READY)
                dr, x, y = struct.unpack("<Bhh", _data)
                if dr & 0b10000000:
                    return x, y

            time.sleep(0.001)

        raise RuntimeError("Timed out waiting for motion data.")

    def frame_capture(self, timeout=10.0):
        """Capture a raw data frame.

        Warning: This is *very* slow and of limited usefulness.

        """
        with self._spi as spi:

            _magic_buff = bytearray([
                0x7f, 0x07,
                0x4c, 0x00,
                0x7f, 0x08,
                0x6a, 0x38,
                0x7f, 0x00,
                0x55, 0x04,
                0x40, 0x80,
                0x4d, 0x11,

                WAIT, 0x0a,

                0x7f, 0x00,
                0x58, 0xff
            ])
            spi.write(_magic_buff)

            t_start = time.time()

            while True:
                status = 0
                spi.readinto(status, write_value=REG_RAWDATA_GRAB_STATUS)
                if status & 0b11000000:
                    break

                if time.time() - t_start > timeout:
                    raise RuntimeError("Frame capture init timed out.")

            spi.write([REG_RAWDATA_GRAB, 0x00])

            RAW_DATA_LEN = 1225

            t_start = time.time()
            raw_data = bytearray(RAW_DATA_LEN)
            x = 0

            while True:
                data_byte = 0
                spi.readinto(data_byte, write_value=REG_RAWDATA_GRAB)

                if data_byte & 0b11000000 == 0b01000000:  # Upper 6 bits
                    raw_data[x] &= ~0b11111100
                    raw_data[x] |= (data_byte & 0b00111111) << 2  # Held in 5:0

                if data_byte & 0b11000000 == 0b10000000:
                    raw_data[x] &= ~0b00000011
                    raw_data[x] |= (data_byte & 0b00001100) >> 2  # Held in 3:2
                    x += 1

                if x == RAW_DATA_LEN:
                    return raw_data
                if time.time() - t_start > timeout:
                    raise RuntimeError(f"Raw data capture timeout, got {x} values.")

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


class Paa5100_SPI(Pmw3901_SPI):

    def _secret_sauce(self):
        """Write the secret sauce registers for the PAA5100.

        Don't ask what these do, we'd have to make you walk the plank.

        These are some proprietary calibration magic.

        I hate this as much as you do, dear reader.

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
                0x7f, 0x0e
            ])
            spi.write(_magic_buff)

            status = 0
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

                0x5f, 0x34,
                0x7b, 0x08,
                0x5e, 0x34,
                0x5b, 0x11,
                0x6d, 0x11,
                0x45, 0x17,
                0x70, 0xe5,
                0x71, 0xe5,

                0x7f, 0x06,
                0x44, 0x1b,
                0x40, 0xbf,
                0x4e, 0x3f,

                0x7f, 0x08,
                0x66, 0x44,
                0x65, 0x20,
                0x6a, 0x3a,
                0x61, 0x05,
                0x62, 0x05,

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
                0x6f, 0x1c,

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

                WAIT, 0x0a,  # Wait 10ms

                0x7f, 0x00,
                0x32, 0x00,

                0x7f, 0x07,
                0x40, 0x40,

                0x7f, 0x06,
                0x68, 0xf0,
                0x69, 0x00,

                0x7f, 0x0d,
                0x48, 0xc0,
                0x6f, 0xd5,

                0x7f, 0x00,
                0x5b, 0xa0,
                0x4e, 0xa8,
                0x5a, 0x90,
                0x40, 0x80,
                0x73, 0x1f,

                WAIT, 0x0a,  # Wait 10ms

                0x73, 0x00
            ])
            spi.write(_magic_buff)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--rotation', type=int,
        default=0, choices=[0, 90, 180, 270],
        help='Rotation of sensor in degrees.',
    )

    args = parser.parse_args()

    spi_dev = board.SPI()

    flo = Pmw3901_SPI(spi_bus=spi_dev)
    flo.set_rotation(args.rotation)

    tx = 0
    ty = 0

    try:
        while True:
            try:
                x, y = flo.get_motion()
            except RuntimeError:
                continue

            tx += x
            ty += y

            print("Motion {:03d} {:03d} x: {:03d} y {:03d}".format(x, y, tx, ty))
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass

