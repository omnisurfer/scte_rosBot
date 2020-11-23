#!/bin/bash
systemctl status gpsd & wait
systemctl stop gpsd & wait
systemctl stop gpsd.socket & wait

echo "Set baud..."
echo -e "\$PMTK251,115200*1F\r\n" > /dev/ttyS0
sleep 3
echo "Set 10Hz..."
echo -e "\$PMTK220,100*2F\r\n" > /dev/ttyS0
sleep 3
echo "Config ttyS0..."
stty -F /dev/ttyS0 speed 115200
sleep 3

systemctl start gpsd & wait
systemctl status gpsd
