//
// Created by daniel on 3/12/22.
//

/*
 * referencing
 * https://github.com/CIR-KIT/steer_drive_ros/blob/kinetic-devel/steer_bot_hardware_gazebo/src/steer_bot_hardware_gazebo.cpp
 */

#ifndef ADAFRUIT_SERVO_HAT_ADAFRUIT_SERVO_HAT_HW_INTERFACE_H
#define ADAFRUIT_SERVO_HAT_ADAFRUIT_SERVO_HAT_HW_INTERFACE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

/*
 * Using CIRKIT Unit03 Robot Base for Reference - cirkit_unit03_hw.cpp
 */

enum VIRTUAL_JOINT_IND {
    VIRTUAL_JOINT_IND_RIGHT_REAR        = 0,
    VIRTUAL_JOINT_IND_LEFT_REAR         = 1,
    VIRTUAL_JOINT_IND_RIGHT_FRONT       = 2,
    VIRTUAL_JOINT_IND_LEFT_FRONT        = 3,
    VIRTUAL_JOINT_IND_RIGHT_FRONT_STEER = 4,
    VIRTUAL_JOINT_IND_LEFT_FRONT_STEER  = 5
};

#define JOINT_INDEX_FRONT 2
#define JOINT_INDEX_REAR_RIGHT 0
#define JOINT_INDEX_REAR_LEFT 1


class AdafruitServoHatHardwareInterface : public hardware_interface::RobotHW {

public:

    AdafruitServoHatHardwareInterface(const std::string& robot_namespace, const ros::NodeHandle& node_handle);
    ros::Time getTime() const { return ros::Time::now(); }
    ros::Duration getPeriod() const {return ros::Duration(0.01); }

    void read(ros::Time time, ros::Duration period);
    void write(ros::Time time, ros::Duration period);

    ~AdafruitServoHatHardwareInterface() {

    };

private:

    ros::NodeHandle _node_handle;
    std::string _robot_namespace;

    void publishSteer(double angle_cmd);

    void registerVirtualJointState(
            std::vector<double> &virtual_wheel_positions,
            std::vector<double> &virtual_wheel_velocities,
            std::vector<double> &virtual_wheel_efforts,
            std::vector<std::string> &virtual_wheels_names
            );

    hardware_interface::PositionJointInterface front_steer_joint_position_cmd_interface;
    double front_steer_position{};
    double front_steer_velocity{};
    double front_steer_effort{};
    double front_steer_position_cmd{};

    hardware_interface::VelocityJointInterface rear_wheel_joint_velocity_cmd_interface;
    double rear_wheel_position{};
    double rear_wheel_velocity{};
    double rear_wheel_effort{};
    double rear_wheel_velocity_cmd{};

    hardware_interface::JointStateInterface joint_state_interface;
    std::vector<double> virtual_wheels_position;
    std::vector<double> virtual_wheels_velocities;
    std::vector<double> virtual_wheels_effort;

    ros::Publisher steer_cmd_publisher;
};

#endif //ADAFRUIT_SERVO_HAT_ADAFRUIT_SERVO_HAT_HW_INTERFACE_H
