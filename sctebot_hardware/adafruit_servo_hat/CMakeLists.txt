cmake_minimum_required(VERSION 3.0.2)
project(adafruit_servo_hat)

ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-Wall -Wno-undef -fPIC)

find_package(catkin REQUIRED COMPONENTS
        dynamic_reconfigure
        message_generation
        roscpp rosgraph_msgs
        rospy
        std_msgs
        utils
        i2c_linux
        geometry_msgs
        nav_msgs
        sensor_msgs
        tf
        realtime_tools
        hardware_interface
        controller_manager
        ackermann_steering_controller
        )

find_package(Boost 1.69.0 COMPONENTS log REQUIRED)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        roscpp
        rosgraph_msgs
        rospy
        std_msgs
        nav_msgs
        sensor_msgs
        tf
        realtime_tools
        hardware_interface
        controller_manager
        ackermann_steering_controller
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        src/
        src/devices
)

#[[]]
add_executable(
        pca9685.exe

        src/devices/pca9685.cpp
)

target_link_libraries(
        pca9685.exe

        Boost::log
        ${catkin_LIBRARIES}
)

add_library(
        pca9685

        src/devices/pca9685.cpp
)

add_library(
        odometry

        src/odometry.cpp
)

add_library(
        adafruit_servo_hat_hw_interface

        src/adafruit_servo_hat_hw_interface.cpp
)

add_executable(
        adafruit_servo_hat_node

        src/adafruit_servo_hat_node.cpp
)

target_link_libraries(
        adafruit_servo_hat_node

        adafruit_servo_hat_hw_interface
        odometry
        pca9685
        pthread
        Boost::log
        ${catkin_LIBRARIES}
)
