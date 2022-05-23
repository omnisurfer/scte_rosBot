#!/bin/bash
# https://www.digikey.com/htmldatasheets/production/1801407/0/0/1/pmtk-command-packet.html

systemctl status gpsd & wait
systemctl stop gpsd & wait
systemctl stop gpsd.socket & wait

SERIAL_DEVICE=/dev/ttyAMA0

echo "Set baud..."
echo -e "\$PMTK251,115200*1F\r\n" > $SERIAL_DEVICE
sleep 3
echo "Config $SERIAL_DEVICE..."
stty -F $SERIAL_DEVICE speed 115200
sleep 3
echo "Set 10Hz..."
echo -e "\$PMTK220,100*2F\r\n" > $SERIAL_DEVICE
sleep 3

systemctl start gpsd & wait
systemctl status gpsd

