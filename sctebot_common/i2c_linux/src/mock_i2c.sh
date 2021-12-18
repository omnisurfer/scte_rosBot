#!/bin/bash

BMP180=0x77
L3GD20=0x6b
LSM303_ACCEL=0x19
LSM303_MAG=0x1e

ADA_SERVO=0x40

modprobe i2c-dev
modprobe i2c-stub chip_addr=$BMP180,$L3GD20,$LSM303_ACCEL,$LSM303_MAG,$ADA_SERVO

sleep 5

# TODO create code to select the last i2c device since this is likely the stub device that was just created above
# chmod 666 /dev/i2c-0


