#!/bin/bash
modprobe i2c-dev
modprobe i2c-stub chip_addr=0x77,0x6b,0x19

sleep 5

chmod 666 /dev/i2c-0


