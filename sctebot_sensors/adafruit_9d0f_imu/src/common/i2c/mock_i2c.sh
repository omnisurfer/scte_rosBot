#!/bin/bash
modprobe i2c-dev
modprobe i2c-stub chip_addr=0x03
