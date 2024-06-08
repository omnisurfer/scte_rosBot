#!/usr/bin/env python
import time

import rospy
import tf
from pmw3901 import PMW3901, PAA5100, BG_CS_FRONT_BCM, BG_CS_BACK_BCM

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, Pose, TransformStamped
from tf.broadcaster import TransformBroadcaster

from math import pi, sin, cos, tan

'''
odom node ref:
- https://github.com/methylDragon/ros-sensor-fusion-tutorial/blob/master/01%20-%20ROS%20and%20Sensor%20Fusion%20Tutorial.md
- http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
- https://www.reddit.com/r/robotics/comments/ggn8lr/correct_setup_of_wheel_encoders_and_odometry_in/?rdt=37191
- https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf
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
                 rotation=ConfigOptions.sensor_rotation['0'],
                 sensor_count_to_distance_tf=0.00012  # counts per meter
                 ):

        """
        dy_count = 250, dist(cm) = 3
        c2d = 0.03m / 250cnts = 0.00012
        """

        self.sensor_class = module
        self.sensor_count_to_distance_tf = sensor_count_to_distance_tf

        self.flo_sensor = self.sensor_class(
            spi_port=spi_port,
            spi_cs_gpio=spi_cs_slot
        )
        self.flo_sensor.set_rotation(rotation)

        self.delta_x_count = 0
        self.delta_y_count = 0

    def get_motion_delta_count(self):

        # TODO turn into thread to update async to request for data?
        self.delta_x_count, self.delta_y_count = self.flo_sensor.get_motion()

        return self.delta_x_count, self.delta_y_count

    def get_motion_delta_meters(self):

        delta_x_meters = self.delta_x_count * self.sensor_count_to_distance_tf
        delta_y_meters = self.delta_y_count * self.sensor_count_to_distance_tf

        return delta_x_meters, delta_y_meters


def main():
    rospy.init_node('pmw3901', anonymous=True)
    rate = rospy.Rate(1)

    # Get ros parameters
    _module = rospy.get_param("/module", 'pmw3901')
    _spi_port = rospy.get_param("/spi_port", 0)
    _spi_cs_slot = rospy.get_param("/spi_cs_slot", 'back')
    _sensor_rotation = rospy.get_param("/rotation", '270')

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

    cdx = 0
    cdy = 0

    dx = 0.0
    dy = 0.0
    th = 0.0

    vx = 0.1
    vy = -0.1
    vth = 0.1

    time_now = rospy.Time.now()
    last_time = rospy.Time.now()

    odom_publisher = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    rospy.loginfo('Running node...')
    while not rospy.is_shutdown():

        data_updated = False

        try:
            delta_x_count, delta_y_count = pmw3901_module.get_motion_delta_count()
            delta_x, delta_y = pmw3901_module.get_motion_delta_meters()

            if delta_y == 0.0:
                delta_th = 0.0
            else:
                delta_th = tan(delta_x/delta_y)

            time_now = rospy.Time.now()

            cdx += delta_x_count
            cdy += delta_y_count
            dx += delta_x
            dy += delta_y
            th += delta_th

            # compute odometry given velocities of the sense
            dt = (time_now - last_time).to_sec()

            vx = dx / dt
            vy = dy / dt
            vth = th / dt

            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(th * 0.5)
            quaternion.w = cos(th * 0.5)

            odom_trans = TransformStamped()
            odom_trans.header.stamp = time_now
            odom_trans.header.frame_id = 'odom'
            odom_trans.child_frame_id = 'chassis'
            odom_trans.transform.translation.x = dx
            odom_trans.transform.translation.y = dy
            odom_trans.transform.translation.z = 0.0
            odom_trans.transform.rotation = quaternion

            odom_broadcaster.sendTransformMessage(odom_trans)

            odom = Odometry()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'chassis'
            odom.header.stamp = time_now
            odom.pose.pose.position.x = dx
            odom.pose.pose.position.y = dy
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = quaternion
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = vth

            odom_publisher.publish(odom)

            data_updated = True
            last_time = rospy.Time.now()

        except RuntimeError:
            print('RUNTIME_ERROR')
            continue

        if data_updated:
            print(
                f"delta_x_count {delta_x_count}, "
                f"delta_y_count {delta_y_count} - "
                f"delta_x {delta_x:.3f} "
                f"delta_y {delta_y:.3f} - "
                f"delta_th {delta_th}"
            )

            print(f"vx {vx:.3f} vy {vy:.3f} - vth {vth:.3f} - dt {dt}")
            print(f"cdx {cdx} cdy {cdy} - dx {dx:.3f} dy {dy:.3f} - th {th:.3f}")

        rate.sleep()


if __name__ == '__main__':
    main()
