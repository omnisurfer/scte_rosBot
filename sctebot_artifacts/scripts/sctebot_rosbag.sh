#!/bin/sh

ROS_TOPICS=\
"
/tf \
/tf_static \
/sctebot/joint_states \
/sctebot/gps/filtered \
/sctebot/odom \
/sctebot/odometry/filtered \
/sctebot/odometry/filtered_map
"

REALSENSE_CAMERA_TOPICS=\
"
/sctebot/realsense_d435i/color/image_raw/compressed \
/sctebot/realsense_d435i/color/camera_info \
/sctebot/realsense_d435i/left/image_raw/compressed \
/sctebot/realsense_d435i/left/camera_info
/sctebot/realsense_d435i/right/image_raw/compressed \
/sctebot/realsense_d435i/right/camera_info \
/sctebot/realsense_d435i/depth/image_rect_raw \
/sctebot/realsense_d435i/depth/camera_info
"

REALSENSE_IMU_TOPICS=\
"
/sctebot/realsense_d435i/imu/data_raw \
/sctebot/realsense_d435i/imu/data \
/sctebot/realsense_d435i/imu_corrected/data \
/sctebot/realsense_d435i/gyro/imu_info \
/sctebot/realsense_d435i/accel/imu_info
"

ADA_10_DOF_TOPICS=\
"
/sctebot/adafruit_10dof_imu/imu/data_raw \
/sctebot/adafruit_10dof_imu/mag/data_raw \
/sctebot/adafruit_10dof_imu/atm_pressure/data_raw \
/sctebot/adafruit_10dof_imu/sea_lvl_pressure/data_raw \
/sctebot/adafruit_10dof_imu/atm_temperature/data_raw \
/sctebot/adafruit_10dof_imu/atm_altitude/data_raw
"

ADA_GPS_TOPICS=\
"
/sctebot/adafruit_ultimate_gps/gps/fix
/sctebot/adafruit_ultimate_gps/gps/extended_fix
/sctebot/adafruit_ultimate_gps/odometry/gps
"

ACKERMANN_STEER_TOPICS=\
"
/sctebot/ackermann_steering_controller/cmd_vel
/sctebot/ackermann_steering_controller/odom
"

Y3LIDAR_TOPICS=\
"
/sctebot/scan
"

rosbag record record --split --duration=5m \
$ROS_TOPICS \
$REALSENSE_CAMERA_TOPICS \
$REALSENSE_IMU_TOPICS \
$ADA_10_DOF_TOPICS \
$ADA_GPS_TOPICS \
$ACKERMANN_STEER_TOPICS \
$Y3LIDAR_TOPICS &