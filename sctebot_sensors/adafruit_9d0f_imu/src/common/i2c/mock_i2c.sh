#!/bin/bash
modprobe i2c-dev
modprobe i2c-stub chip_addr=0x03

sleep 5

chmod 666 /dev/i2c-0
