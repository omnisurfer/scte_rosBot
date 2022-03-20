//
// Created by daniel on 3/12/22.
//

/*
 * referencing
 * https://github.com/CIR-KIT/steer_drive_ros/blob/kinetic-devel/steer_bot_hardware_gazebo/src/steer_bot_hardware_gazebo.cpp
 */

#ifndef ADAFRUIT_SERVO_HAT_ADAFRUIT_SERVO_HAT_HW_INTERFACE_H
#define ADAFRUIT_SERVO_HAT_ADAFRUIT_SERVO_HAT_HW_INTERFACE_H

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include <controller_manager/controller_manager.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#define JOINT_INDEX_REAR_RIGHT 0
#define JOINT_INDEX_REAR_LEFT 1
#define JOINT_INDEX_FRONT 2

enum FRONT_REAR {
    FRONT = 0,
    REAR = 1
};

class AdafruitServoHatHardwareInterface : public hardware_interface::RobotHW {

public:

    AdafruitServoHatHardwareInterface();

    bool init(const std::string& robot_namespace, const ros::NodeHandle& node_handle);

    void read(ros::Time time, ros::Duration period);
    void write(ros::Time time, ros::Duration period);

    ros::Time getTime() const { return ros::Time::now(); }
    ros::Duration getPeriod() const {return ros::Duration(0.01); }

    ~AdafruitServoHatHardwareInterface() {

    };

private:

    enum {
        INDEX_RIGHT = 0,
        INDEX_LEFT = 1
    };

    ros::NodeHandle _node_handle;
    std::string _robot_namespace;

    hardware_interface::JointStateInterface _joint_state_interface;

    // region wheel names
    // rear wheel, actual joint that moves the robot
    std::string _rear_wheel_joint_name;
    double _rear_wheel_joint_position;
    double _rear_wheel_joint_velocity;
    double _rear_wheel_joint_effort;
    double _rear_wheel_joint_velocity_command;
    hardware_interface::VelocityJointInterface _rear_wheel_joint_velocity_command_interface;

    // rear wheels, virtual
    std::vector<std::string> _virtual_rear_wheel_joint_names;
    std::vector<double> _virtual_rear_wheel_joint_position;
    std::vector<double> _virtual_rear_wheel_joint_velocity;
    std::vector<double> _virtual_rear_wheel_joint_effort;
    std::vector<double> _virtual_rear_wheel_joint_velocity_command;

    // front wheels, virtual
    std::vector<std::string> _virtual_front_wheel_joint_names;
    std::vector<double> _virtual_front_wheel_joint_position;
    std::vector<double> _virtual_front_wheel_joint_velocity;
    std::vector<double> _virtual_front_wheel_joint_effort;
    std::vector<double> _virtual_front_wheel_joint_velocity_command;
    // endregion

    // front steer, actual joint that moves the robot
    std::string _front_steer_joint_name;
    double _front_steer_joint_position;
    double _front_steer_joint_velocity;
    double _front_steer_joint_effort;
    double _front_steer_joint_position_command;
    hardware_interface::VelocityJointInterface _front_steer_joint_position_command_interface;

    // front steer, virtual joints
    std::vector<std::string> _virtual_front_steer_joint_names;
    std::vector<double> _virtual_front_steer_joint_position;
    std::vector<double> _virtual_front_steer_joint_velocity;
    std::vector<double> _virtual_front_steer_joint_effort;
    std::vector<double> _virtual_front_steer_joint_position_command;

    double _wheel_separation_w;
    double _wheel_separation_h;

    // region methods
    void CleanUp();

    void GetJointNames(ros::NodeHandle &_node_handle);
    void GetWheelJointNames(ros::NodeHandle &_node_handle);
    void GetSteerJointNames(ros::NodeHandle &_node_handle);

    void RegisterHardwareInterfaces();
    void RegisterWheelInterface();
    void RegisterSteerInterface();

    void RegisterInterfaceHandles(
            hardware_interface::JointStateInterface& joint_state_interface,
            hardware_interface::JointCommandInterface& joint_command_interface,
            const std::string joint_name,
            double& joint_position,
            double& joint_velocity,
            double& joint_effort,
            double& joint_command
            );

    void RegisterInterfaceHandles(
            hardware_interface::JointStateInterface& joint_state_interface,
            const std::string joint_name,
            double& joint_position,
            double& joint_velocity,
            double& joint_effort
            );

    void RegisterJointStateInterfaceHandle(
            hardware_interface::JointStateInterface& joint_state_interface,
            const std::string joint_name,
            double& joint_position,
            double& joint_velocity,
            double& joint_effort
            );

    void RegisterCommandJointInterfaceHandle(
            hardware_interface::JointStateInterface& joint_state_interface,
            hardware_interface::JointCommandInterface& joint_command_interface,
            const std::string joint_name,
            double& joint_command
            );

    // passed on ComputeEffCommandFromVelError

    void GetCurrentState(
            std::vector<double>& joint_position,
            std::vector<double>& joint_velocity,
            std::vector<double>& joint_effort,
            const int interface_index
            );

    // endregion
};

#endif //ADAFRUIT_SERVO_HAT_ADAFRUIT_SERVO_HAT_HW_INTERFACE_H
