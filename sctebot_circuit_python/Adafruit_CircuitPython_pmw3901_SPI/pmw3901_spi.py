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

* Using this API doc for SPI usage:
https://docs.circuitpython.org/en/latest/shared-bindings/adafruit_bus_device/spi_device/index.html#module-adafruit_bus_device.spi_device
https://learn.adafruit.com/circuitpython-digital-inputs-and-outputs/digital-outputs

"""
import struct
import time
import board
# from board import *
import busio
import digitalio

try:
    from digitalio import DigitalInOut
    from busio import SPI
except ImportError:
    pass

from adafruit_bus_device.spi_device import SPIDevice

__version__ = "0.0.0+auto.0"
__repo__ = "WIP"

# Not using WAIT from original implementation because secret_sauce bytes are in a bytearray.
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

    def __init__(self):

        # SPI setup
        self.spi_bus_ = busio.SPI(board.SCK, board.MOSI, board.MISO)
        self.cs_ = digitalio.DigitalInOut(board.D4)
        self.cs_.direction = digitalio.Direction.OUTPUT
        self.spi_device_ = SPIDevice(
            spi=self.spi_bus_,
            # chip_select=self.cs_,
            baudrate=400000,
            cs_active_value=False,
            polarity=0,
            extra_clocks=0,
            phase=0
        )

        self.init_device()

        '''
        self.set_orientation()
        self.frame_capture()
        self.get_motion_slow()
        self.get_motion()
        '''

    def init_device(self):

        print("TEMP")

        # toggle chip select
        self.cs_.value = True
        time.sleep(0.30)
        self.cs_.value = False
        time.sleep(0.033)
        self.cs_.value = True

        # write out POWER UP RESET
        self._spi_write_out_buffer(bytearray([REG_POWER_UP_RESET, 0x5a]), single_byte_writes=True)
        time.sleep(0.02)

        # status
        status_register = bytearray(5)

        time.sleep(0.018)

        # DMR_DEBUG_20240228 - May need to try a version of read from address that only reads one byte at a time?
        status_register = self._spi_read_from_incremented_address(bytearray([REG_DATA_READY]), len(status_register))

        print(f"got status {status_register}")

        self._secret_sauce()

        product_id, revision = self.get_id()

        if product_id != 0x49 or revision != 0x00:
            raise RuntimeError(
                "Invalid Product ID or Revision for PMW3901: 0x{:02x}/0x{:02x}".format(product_id, revision))
        else:
            print(f"got product id {product_id} and revision {revision}")

    def get_id(self):

        read_data = self._spi_read_from_incremented_address(bytearray([REG_ID]), 2)

        id_wr = read_data[0]
        rev_wr = read_data[1]

        return id_wr, rev_wr

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
        self._spi_write_out_buffer(bytearray([REG_ORIENTATION, value]))

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

            motion_data = self._spi_read_from_address_number_of_bytes(bytearray([REG_MOTION_BURST]), 13)

            (_, dr, obs,
             x_, y_, quality,
             raw_sum, raw_max, raw_min,
             shutter_upper, shutter_lower) = struct.unpack("<BBBhhBBBBBB", motion_data)

            print(f"motion_data {motion_data}")
            print(f"_ {_}\tdr {dr}\tobs {obs}\tx_ {x_}\ty_ {y_}\tquality {quality}\traw_sum {raw_sum}\t"
                  f"raw_max {raw_max}\traw_min {raw_min}\tshutter_upper {shutter_upper}\tshutter_lower {shutter_lower}")

            if dr & 0b10000000 and not (quality < 0x19 and shutter_upper == 0x1f):
                return x_, y_

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

            motion_data = self._spi_read_from_incremented_address(bytearray([REG_DATA_READY]), 5)

            dr, x_, y_ = struct.unpack("<Bhh", motion_data)

            print(f"motion_data_slow {motion_data}")
            print(f"dr {dr}\t x_ {x_}, y_{y_}")

            if dr & 0b10000000:
                return x_, y_

            time.sleep(0.001)

        raise RuntimeError("Timed out waiting for motion data.")

    def frame_capture(self, timeout=10.0):
        """Capture a raw data frame.

        Warning: This is *very* slow and of limited usefulness.

        """
        magic_buff = bytearray([
            0x7f, 0x07,
            0x4c, 0x00,
            0x7f, 0x08,
            0x6a, 0x38,
            0x7f, 0x00,
            0x55, 0x04,
            0x40, 0x80,
            0x4d, 0x11
        ])
        self._spi_write_out_buffer(magic_buff)

        # WAIT, 0x0a
        self._secret_sauce_wait_time_ms(0x0a)

        magic_buff = bytearray([
            0x7f, 0x00,
            0x58, 0xff
        ])
        self._spi_write_out_buffer(magic_buff)

        t_start = time.time()

        while True:
            status = self._spi_read_from_incremented_address(bytearray([REG_RAWDATA_GRAB_STATUS]), 1)
            if status[0] & 0b11000000:
                break

            if time.time() - t_start > timeout:
                raise RuntimeError("Frame capture init timed out.")

        self._spi_write_out_buffer(bytearray([REG_RAWDATA_GRAB, 0x00]))

        RAW_DATA_LEN = 1225

        t_start = time.time()
        raw_data = bytearray(RAW_DATA_LEN)
        x_ = 0

        while True:
            data_byte = self._spi_read_from_incremented_address(bytearray([REG_RAWDATA_GRAB]), 1)[0]

            if data_byte & 0b11000000 == 0b01000000:  # Upper 6 bits
                raw_data[x_] &= ~0b11111100
                raw_data[x_] |= (data_byte & 0b00111111) << 2  # Held in 5:0

            if data_byte & 0b11000000 == 0b10000000:
                raw_data[x_] &= ~0b00000011
                raw_data[x_] |= (data_byte & 0b00001100) >> 2  # Held in 3:2
                x_ += 1

            if x_ == RAW_DATA_LEN:
                return raw_data
            if time.time() - t_start > timeout:
                raise RuntimeError(f"Raw data capture timeout, got {x_} values.")

    def _secret_sauce(self):
        """Write the secret sauce registers.

        Don't ask what these do, the datasheet refuses to explain.

        They are some proprietary calibration magic.

        """
        # Config phase 1
        magic_buff = bytearray([
            0x7f, 0x00,
            0x55, 0x01,
            0x50, 0x07,

            0x7f, 0x0e,
            0x43, 0x10
        ])
        self._spi_write_out_buffer(magic_buff)

        status = self._spi_read_from_incremented_address(bytearray([0x67]), 1)
        if status[0] & 0b10000000:
            self._spi_write_out_buffer(bytearray([0x48, 0x04]))
        else:
            self._spi_write_out_buffer(bytearray([0x48, 0x02]))

        # Config phase 2
        magic_buff = bytearray([
            0x7f, 0x00,
            0x51, 0x7b,

            0x50, 0x00,
            0x55, 0x00,
            0x7f, 0x0E
        ])
        self._spi_write_out_buffer(magic_buff)

        status = self._spi_read_from_incremented_address(bytearray([0x73]), 1)
        if status[0] == 0x00:
            _c1 = self._spi_read_from_incremented_address(bytearray([0x70]), 1)
            _c2 = self._spi_read_from_incremented_address(bytearray([0x71]), 1)

            if _c1[0] <= 28:
                _c1[0] += 14
            if _c1[0] > 28:
                _c1[0] += 11

            _c1[0] = max(0, min(0x3F, _c1[0]))
            _c2[0] = (_c2[0] * 45) // 100

            magic_buff = bytearray([
                0x7f, 0x00,
                0x61, 0xad,
                0x51, 0x70,
                0x7f, 0x0e
            ])
            self._spi_write_out_buffer(magic_buff)
            self._spi_write_out_buffer(bytearray([0x70, _c1[0]]))
            self._spi_write_out_buffer(bytearray([0x71, _c2[0]]))

        # Config phase 3
        magic_buff = bytearray([
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
            0x70, 0x00
        ])
        self._spi_write_out_buffer(magic_buff)

        # WAIT, 0x0A,  # Sleep for 10ms
        self._secret_sauce_wait_time_ms(0x0a)

        magic_buff = bytearray([
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
            0x40, 0x80
        ])
        self._spi_write_out_buffer(magic_buff)

        # WAIT, 0xF0,
        self._secret_sauce_wait_time_ms(0xf0)

        magic_buff = bytearray([
            0x7f, 0x14,  # Enable LED_N pulsing
            0x6f, 0x1c,
            0x7f, 0x00
        ])
        self._spi_write_out_buffer(magic_buff)

    def _secret_sauce_wait_time_ms(self, hex_value):

        print("Sleeping for: {:02d}ms".format(hex_value))

        time.sleep(hex_value / 1000)

    def _spi_read_from_incremented_address(
            self, read_from_address: bytearray, address_offset_count: int, single_byte_reads=True
    ):

        processed_read_data_buffer = bytearray()
        unprocessed_read_data_buffer = bytearray()

        if not single_byte_reads:
            # Seems to return 0xFF <product_id> and 0xFF <revision>, these are probably dummy bytes given how SPI works?
            # readinto does not "filter" them out?
            raw_read_data_length = address_offset_count * 2
            temp_read_data_buffer = bytearray(raw_read_data_length)

            read_from_address_buffer = bytearray()
            for i in range(address_offset_count):
                read_from_address_buffer.append(read_from_address[0] + i)
                read_from_address_buffer.append(0x00)

            with self.spi_device_ as spi:
                """
                spi.write(read_from_address, start=0, end=read_length)
                spi.readinto(unprocessed_read_data_buffer, start=0, end=len(unprocessed_read_data_buffer) - 1, write_value=REG_ID)
                id = unprocessed_read_data_buffer[0]
                rev = unprocessed_read_data_buffer[1]
                """

                self.cs_.value = False
                spi.write_readinto(
                    read_from_address_buffer,
                    temp_read_data_buffer,
                    in_start=0, in_end=raw_read_data_length,
                    out_start=0, out_end=raw_read_data_length
                )
                self.cs_.value = True

            unprocessed_read_data_buffer.extend(temp_read_data_buffer)

        elif single_byte_reads:

            temp_read_data_buffer = bytearray(2)

            for i in range(address_offset_count):

                read_from_address_buffer = [(read_from_address[0] + i), 0x00]

                with self.spi_device_ as spi:
                    self.cs_.value = False
                    spi.write_readinto(
                        read_from_address_buffer,
                        temp_read_data_buffer,
                        in_start=0, in_end=2,
                        out_start=0, out_end=2
                    )
                    self.cs_.value = True

                unprocessed_read_data_buffer.extend(temp_read_data_buffer)

        else:

            temp_read_data_buffer = bytearray(1)
            for i in range(address_offset_count):

                read_from_address_buffer = [(read_from_address[0] + i)]

                with self.spi_device_ as spi:
                    self.cs_.value = False
                    spi.write(read_from_address_buffer, start=0, end=0)
                    spi.readinto(
                        temp_read_data_buffer, start=0, end=0,
                        write_value=0x00
                    )
                    self.cs_.value = True

                unprocessed_read_data_buffer.extend(temp_read_data_buffer)

        for i in range(len(unprocessed_read_data_buffer)):
            if i % 2:
                processed_read_data_buffer.append(unprocessed_read_data_buffer[i])

        return processed_read_data_buffer

    def _spi_read_from_address_number_of_bytes(self, read_from_address: bytearray, read_length: int):
        raw_read_data_length = read_length
        raw_read_data_buffer = bytearray(raw_read_data_length)

        read_from_address_buffer = bytearray()
        for i in range(read_length):
            if i == 0:
                read_from_address_buffer.append(read_from_address[0])
            else:
                read_from_address_buffer.append(0x00)

        with self.spi_device_ as spi:
            self.cs_.value = False
            spi.write_readinto(
                read_from_address_buffer,
                raw_read_data_buffer,
                in_start=0, in_end=len(raw_read_data_buffer),
                out_start=0, out_end=len(raw_read_data_buffer)
            )
            self.cs_.value = True

        return raw_read_data_buffer

    def _spi_write_out_buffer(self, write_out_buffer: bytearray, single_byte_writes=True):
        # set bit 8 for write
        processed_write_out_buffer = bytearray()

        for i in range(len(write_out_buffer)):

            if i % 2:
                # data
                processed_write_out_buffer.append(write_out_buffer[i])

                if single_byte_writes:
                    with self.spi_device_ as spi:
                        self.cs_.value = False
                        spi.write(processed_write_out_buffer)
                        self.cs_.value = True

                    processed_write_out_buffer.clear()

            else:
                # address and r/w bit
                processed_write_out_buffer.append(write_out_buffer[i] | 0x80)

        if not single_byte_writes:
            with self.spi_device_ as spi:
                self.cs_.value = False
                spi.write(processed_write_out_buffer)
                self.cs_.value = True


class Paa5100_SPI(Pmw3901_SPI):

    def _secret_sauce(self):
        """Write the secret sauce registers for the PAA5100.

        Don't ask what these do, we'd have to make you walk the plank.

        These are some proprietary calibration magic.

        I hate this as much as you do, dear reader.

        """

        # Config phase 1
        magic_buff = bytearray([
            0x7f, 0x00,
            0x55, 0x01,
            0x50, 0x07,

            0x7f, 0x0e,
            0x43, 0x10
        ])
        self._spi_write_out_buffer(magic_buff, single_byte_writes=False)

        status = self._spi_read_from_incremented_address(bytearray([0x67]), 1)
        if status[0] & 0b10000000:
            self._spi_write_out_buffer(bytearray([0x48, 0x04]))
        else:
            self._spi_write_out_buffer(bytearray([0x48, 0x02]))

        # Config phase 2
        magic_buff = bytearray([
            0x7f, 0x00,
            0x51, 0x7b,
            0x50, 0x00,
            0x55, 0x00,
            0x7f, 0x0e
        ])
        self._spi_write_out_buffer(magic_buff)

        status = self._spi_read_from_incremented_address(bytearray([0x73]), 1)

        if status[0] == 0x00:
            _c1 = self._spi_read_from_incremented_address(bytearray([0x70]), 1)
            _c2 = self._spi_read_from_incremented_address(bytearray([0x71]), 1)

            if _c1[0] <= 28:
                _c1[0] += 14
            if _c1[0] > 28:
                _c1[0] += 11
            _c1[0] = max(0, min(0x3F, _c1[0]))
            _c2[0] = (_c2[0] * 45) // 100

            magic_buff = bytearray([
                0x7f, 0x00,
                0x61, 0xad,
                0x51, 0x70,
                0x7f, 0x0e
            ])
            self._spi_write_out_buffer(magic_buff)
            self._spi_write_out_buffer(bytearray([0x70, _c1[0]]))
            self._spi_write_out_buffer(bytearray([0x71, _c2[0]]))

        # Config phase 3
        magic_buff = bytearray([
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
        ])
        self._spi_write_out_buffer(magic_buff)

        # WAIT, 0x0a,  # Wait 10ms
        self._secret_sauce_wait_time_ms(0x0a)

        magic_buff = bytearray([
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

        ])
        self._spi_write_out_buffer(magic_buff)

        # WAIT, 0x0a,  # Wait 10ms
        self._secret_sauce_wait_time_ms(0x0a)

        magic_buff = bytearray([
            0x73, 0x00
        ])
        self._spi_write_out_buffer(magic_buff)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--rotation', type=int,
        default=0, choices=[0, 90, 180, 270],
        help='Rotation of sensor in degrees.',
    )

    args = parser.parse_args()

    flo = Pmw3901_SPI()
    # flo = Paa5100_SPI()
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
