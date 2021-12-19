cmake_minimum_required(VERSION 3.0.2)
project(adafruit_servo_hat_node)

ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-Wall -Wno-undef -fPIC)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        dynamic_reconfigure
        message_generation
        roscpp rosgraph_msgs
        rospy
        std_msgs
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
        pca9685 src/devices/pca9685.cpp
)

add_executable(
        pca9685.exe src/devices/pca9685.cpp
)

target_link_libraries(
        pca9685.exe

        utils
        Boost::log
        ${catkin_LIBRARIES}
)


#add_executable(
#        adafruit_servo_hat_node src/adafruit_servo_hat_node.cpp
#)

#target_link_libraries(
#        adafruit_servo_hat_node

#        adafruit_servo_hat
#        l3gd20
#        ${catkin_LIBRARIES}
#)
