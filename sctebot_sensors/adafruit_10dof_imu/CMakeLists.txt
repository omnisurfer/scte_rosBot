cmake_minimum_required(VERSION 3.0.2)
project(adafruit_10dof_imu)

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
        sctebot_utils
        i2c_linux
        )

find_package(Boost 1.69.0 COMPONENTS log REQUIRED)

catkin_package(
        CATKIN_DEPENDS message_runtime roscpp rosgraph_msgs rospy std_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        src/
        src/devices
)

add_library(
        bmp180

        src/devices/bmp180.cpp
)

add_library(
        l3gd20

        src/devices/l3gd20.cpp
)

add_library(
        lsm303dlhc

        src/devices/lsm303dlhc.cpp
)

add_library(
        adafruit_10dof_imu

        src/adafruit_10dof_imu.cpp
        src/devices/lsm303dlhc.h src/devices/l3gd20.h src/devices/bmp180.h)

add_executable(
        adafruit_10dof_imu_node

        src/adafruit_10dof_imu_node.cpp
)

target_link_libraries(
        adafruit_10dof_imu_node

        adafruit_10dof_imu
        l3gd20
        bmp180
        lsm303dlhc
        pthread
        Boost::log
        ${catkin_LIBRARIES}
)

add_executable(
        bmp180.exe

        src/devices/bmp180.cpp
)

target_link_libraries(
        bmp180.exe

        Boost::log
        ${catkin_LIBRARIES}
)

add_executable(
        l3gd20.exe

        src/devices/l3gd20.cpp
)

target_link_libraries(
        l3gd20.exe

        Boost::log
        ${catkin_LIBRARIES}
)

add_executable(
        lsm303dlhc.exe

        src/devices/lsm303dlhc.cpp
)

target_link_libraries(
        lsm303dlhc.exe

        Boost::log
        ${catkin_LIBRARIES}
)