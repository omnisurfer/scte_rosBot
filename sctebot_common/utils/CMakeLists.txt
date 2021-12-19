cmake_minimum_required(VERSION 3.0.2)
project(utils)

#find_package(catkin REQUIRED)

#catkin_package(
#        INCLUDE_DIRS include/utils
#)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE .)

