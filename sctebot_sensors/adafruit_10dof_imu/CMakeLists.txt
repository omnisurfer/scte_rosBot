cmake_minimum_required(VERSION 3.0.2)
project(adafruit_10dof_imu_node)

ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-Wall -Wno-undef -fPIC)


find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure message_generation roscpp rosgraph_msgs rospy std_msgs)
find_package(Boost 1.69.0 COMPONENTS log REQUIRED)

catkin_package(
        CATKIN_DEPENDS message_runtime roscpp rosgraph_msgs rospy std_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        src/
        src/common/i2c
        src/common/devices
)

## Declare a C++ library
add_library(
        i2c_linux src/common/i2c/i2c_linux.c
)

add_library(
        bmp180 src/common/devices/bmp180.cpp
)

add_library(
        l3gd20 src/common/devices/l3gd20.cpp
)

add_library(
        lsm303dlhc src/common/devices/lsm303dlhc.cpp
)

add_library(
        adafruit_10dof_imu src/adafruit_10dof_imu.cpp
)

add_executable(
        i2c_linux_test src/common/i2c/i2c_linux.c
)


add_executable(
        adafruit_10dof_imu_node src/adafruit_10dof_imu_node.cpp
)
target_link_libraries(
        adafruit_10dof_imu_node

        adafruit_10dof_imu
        l3gd20
        bmp180
        lsm303dlhc
        i2c_linux
        pthread
        Boost::log
        ${catkin_LIBRARIES}
)
