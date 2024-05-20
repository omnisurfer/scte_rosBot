#!/usr/bin/env python
import time

import rospy
from pmw3901 import PMW3901, PAA5100, BG_CS_FRONT_BCM, BG_CS_BACK_BCM


'''
odom node ref:
- https://github.com/methylDragon/ros-sensor-fusion-tutorial/blob/master/01%20-%20ROS%20and%20Sensor%20Fusion%20Tutorial.md
- http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
- https://www.reddit.com/r/robotics/comments/ggn8lr/correct_setup_of_wheel_encoders_and_odometry_in/?rdt=37191
'''


class PMW3901Module:

    class ConfigOptions:
        sensor_module = {'pmw3901': PMW3901, 'paa5100': PAA5100}
        sensor_rotation = {'0': 0, '90': 90, '180': 180, '270': 270}
        sensor_spi_cs_slot = {'front': BG_CS_FRONT_BCM, 'back': BG_CS_BACK_BCM}

    def __init__(self,
                 module=ConfigOptions.sensor_module['pmw3901'],
                 spi_port=0,
                 spi_cs_slot=ConfigOptions.sensor_spi_cs_slot['back'],
                 rotation=ConfigOptions.sensor_rotation['0']
                 ):

        self.sensor_class = module

        self.flo_sensor = self.sensor_class(
            spi_port=spi_port,
            spi_cs_gpio=spi_cs_slot
        )
        self.flo_sensor.set_rotation(rotation)

    def get_motion(self):

        return self.flo_sensor.get_motion()


def main():
    rospy.init_node('pmw3901', anonymous=True)
    rate = rospy.Rate(1)

    # Get ros parameters
    _module = rospy.get_param("/module", 'pmw3901')
    _spi_port = rospy.get_param("/spi_port", 0)
    _spi_cs_slot = rospy.get_param("/spi_cs_slot", 'back')
    _sensor_rotation = rospy.get_param("/rotation", '0')

    # check parameters
    module = PMW3901Module.ConfigOptions.sensor_module.get(_module, 'pmw3901')
    spi_cs_slot = PMW3901Module.ConfigOptions.sensor_spi_cs_slot.get(_spi_cs_slot, 'back')
    sensor_rotation = PMW3901Module.ConfigOptions.sensor_rotation.get(_sensor_rotation, '0')

    rospy.loginfo(f"Sensor module {module}, spi port/cs-slot {_spi_port}/{spi_cs_slot}, rotation {sensor_rotation}")

    pmw3901_module = PMW3901Module(
        module=module,
        spi_port=_spi_port,
        spi_cs_slot=spi_cs_slot,
        rotation=sensor_rotation
    )

    tx = 0
    ty = 0

    while not rospy.is_shutdown():
        rospy.loginfo('F Running node...')

        try:
            dx, dy = pmw3901_module.get_motion()
        except RuntimeError:
            continue
        tx += dx
        ty += dy
        print("Relative: x {:03d} y {:03d} | Absolute: x {:03d} y {:03d}".format(dx, dy, tx, ty))
        rate.sleep()


if __name__ == '__main__':
    main()
