#!/bin/bash

SERVO_ADD=0x40

MODE1=0x00
MODE2=0x01
PRE_SCALE=0xFE

LED0_ON_L=0x06
LED0_ON_H=0x07
LED0_OFF_L=0x08
LED0_OFF_H=0x09

LED0_ON_PWM_L=0x05
LED0_ON_PWM_H=0x00

LED0_OFF_PWM_L=0xCB
LED0_OFF_PWM_H=0x04

echo "Servo test script"

echo "Detected I2C devices:"

i2cdetect -y 1

echo "Set MODE1 to 0x90"

i2cset -y 1 $SERVO_ADD $MODE1 0x90

i2cdeump -y 1 $SERVO_ADD

echo "Set PRE_SCALE 0x1E"

i2cset -y 1 $SERVO_ADD $PRE_SCALE 0x1E

i2cdeump -y 1 $SERVO_ADD

echo "Set MODE1 to 0x00"

i2cset -y 1 $SERVO_ADD $MODE1 0x00

i2cdeump -y 1 $SERVO_ADD

echo "SET PWM"

i2cset -y 1 $SERVO_ADD $LED0_ON_L $LED0_ON_PWM_L
i2cset -y 1 $SERVO_ADD $LED0_ON_H $LED0_ON_PWM_H

i2cset -y 1 $SERVO_ADD $LED0_OFF_L $LED0_OFF_PWM_L
i2cset -y 1 $SERVO_ADD $LED0_OFF_H $LED0_OFF_PWM_H

i2cdump -y 1 $SERVO_ADD