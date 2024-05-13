#!/usr/bin/env python

import rospy


def main():

    rospy.init_node('pmw3901', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo('Running node...')


if __name__ == '__main__':
    main()
