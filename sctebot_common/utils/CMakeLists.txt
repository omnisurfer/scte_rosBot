cmake_minimum_required(VERSION 3.0.2)
project(utils)

add_compile_options(-std=c++11)
add_compile_options(-Wall -Wno-undef -fPIC)

find_package(catkin REQUIRED)

find_package(Boost 1.69.0 COMPONENTS log REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/boost_logging.cpp
)

target_link_libraries(
        ${PROJECT_NAME}

        Boost::log
        ${catkin_LIBRARIES}
)

add_executable(
        ${PROJECT_NAME}.exe

        src/boost_logging.cpp
)

target_link_libraries(
        ${PROJECT_NAME}.exe

        Boost::log
        ${catkin_LIBRARIES}
)

install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
