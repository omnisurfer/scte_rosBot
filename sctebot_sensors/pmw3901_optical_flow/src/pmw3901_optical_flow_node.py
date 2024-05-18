#!/usr/bin/env python
import time

import rospy
from pmw3901 import PMW3901, PAA5100, BG_CS_FRONT_BCM, BG_CS_BACK_BCM


def main():

    sensor_module = {'pmw3901': PMW3901, 'paa5100': PAA5100}
    sensor_rotation = {'0': 0, '90': 90, '180': 180, '270': 270}
    sensor_spi_slot = {'front': BG_CS_FRONT_BCM, 'back': BG_CS_BACK_BCM}

    sensor_class = sensor_module['pmw3901']

    flo = sensor_class(spi_port=0, spi_cs_gpio=sensor_spi_slot['back'])
    flo.set_rotation(sensor_rotation['0'])

    tx = 0
    ty = 0

    rospy.init_node('pmw3901', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo('Running node...')

        try:
            x, y = flo.get_motion()
        except RuntimeError:
            continue
        tx += x
        ty += y
        print("Relative: x {:03d} y {:03d} | Absolute: x {:03d} y {:03d}".format(x, y, tx, ty))
        time.sleep(5)


if __name__ == '__main__':
    main()
