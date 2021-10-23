#!/bin/bash
modprobe i2c-dev
modprobe i2c-stub chip_addr=0x77,0x6b,0x19,0x1e

sleep 5

# TODO create code to select the last i2c device since this is likely the stub device that was just created above
# chmod 666 /dev/i2c-0


