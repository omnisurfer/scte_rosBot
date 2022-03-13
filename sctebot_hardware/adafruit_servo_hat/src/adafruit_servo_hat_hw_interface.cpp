//
// Created by daniel on 3/12/22.
//
#include "adafruit_servo_hat_hw_interface.h"

AdafruitServoHatHardwareInterface::AdafruitServoHatHardwareInterface():
    // using the same params from the steerbot plugin since they get uploaded
    // during the description launch. Longer term may want to create
    // a version with this module namespace to avoid confusion...
    _robot_namespace("steer_bot_hardware_gazebo/") {

}

bool AdafruitServoHatHardwareInterface::init(const std::string &robot_namespace, const ros::NodeHandle &node_handle) {
    this->_node_handle = node_handle;
    this->_robot_namespace = robot_namespace + "/steer_bot_hardware_gazebo/";

    this->CleanUp();
    this->GetJointNames(this->_node_handle);
    this->RegisterHardwareInterfaces();

    return true;
}

void AdafruitServoHatHardwareInterface::read(ros::Time time, ros::Duration period) {

    std::cout << "read not implemented" << std::endl;
}

void AdafruitServoHatHardwareInterface::write(ros::Time time, ros::Duration period) {
    std::cout << "write not implemented" << std::endl;
}

void AdafruitServoHatHardwareInterface::CleanUp() {

    _rear_wheel_joint_name.clear();
    _rear_wheel_joint_position = 0;
    _rear_wheel_joint_velocity = 0;
    _rear_wheel_joint_effort = 0;
    _rear_wheel_joint_velocity_command = 0;

    _virtual_rear_wheel_joint_names.clear();
    _virtual_rear_wheel_joint_position.clear();
    _virtual_rear_wheel_joint_velocity.clear();
    _virtual_rear_wheel_joint_effort.clear();
    _virtual_rear_wheel_joint_velocity_command.clear();

    _virtual_front_wheel_joint_names.clear();
    _virtual_front_wheel_joint_position.clear();
    _virtual_front_wheel_joint_velocity.clear();
    _virtual_front_wheel_joint_effort.clear();
    _virtual_front_wheel_joint_velocity_command.clear();

    _front_steer_joint_name.clear();
    _front_steer_joint_position = 0;
    _front_steer_joint_velocity = 0;
    _front_steer_joint_effort = 0;
    _front_steer_joint_position_command = 0;

    _virtual_front_steer_joint_names.clear();
    _virtual_front_steer_joint_position.clear();
    _virtual_front_steer_joint_velocity.clear();
    _virtual_front_steer_joint_effort.clear();
    _virtual_front_steer_joint_position_command.clear();

}

void AdafruitServoHatHardwareInterface::GetJointNames(ros::NodeHandle &_node_handle) {
    this->GetWheelJointNames(_node_handle);
    this->GetSteerJointNames(_node_handle);
}

void AdafruitServoHatHardwareInterface::GetWheelJointNames(ros::NodeHandle &_node_handle) {

    this->_node_handle.getParam(this->_robot_namespace + "rear_wheel", _rear_wheel_joint_name);

    // virtual wheel joints
    this->_node_handle.getParam(this->_robot_namespace + "virtual_rear_wheels", _virtual_rear_wheel_joint_names);
    int degrees_of_freedom = (int)_virtual_rear_wheel_joint_names.size();
    _virtual_rear_wheel_joint_position.resize(degrees_of_freedom);
    _virtual_rear_wheel_joint_velocity.resize(degrees_of_freedom);
    _virtual_rear_wheel_joint_effort.resize(degrees_of_freedom);
    _virtual_rear_wheel_joint_velocity_command.resize(degrees_of_freedom);

    this->_node_handle.getParam(this->_robot_namespace + "virtual_front_wheels", _virtual_front_wheel_joint_names);
    degrees_of_freedom = (int)_virtual_front_wheel_joint_names.size();
    _virtual_front_wheel_joint_position.resize(degrees_of_freedom);
    _virtual_front_wheel_joint_velocity.resize(degrees_of_freedom);
    _virtual_front_wheel_joint_effort.resize(degrees_of_freedom);
    _virtual_front_wheel_joint_velocity_command.resize(degrees_of_freedom);
}

void AdafruitServoHatHardwareInterface::GetSteerJointNames(ros::NodeHandle &_node_handle) {

    this->_node_handle.getParam(this->_robot_namespace + "front_steer", _front_steer_joint_name);

    this->_node_handle.getParam(this->_robot_namespace + "virtual_front_steers", _virtual_front_steer_joint_names);
    int degrees_of_freedom = (int)_virtual_front_steer_joint_names.size();
    _virtual_front_steer_joint_position.resize(degrees_of_freedom);
    _virtual_front_steer_joint_velocity.resize(degrees_of_freedom);
    _virtual_front_steer_joint_effort.resize(degrees_of_freedom);
    _virtual_front_steer_joint_position_command.resize(degrees_of_freedom);
}

void AdafruitServoHatHardwareInterface::RegisterHardwareInterfaces() {

    this->RegisterSteerInterface();
    this->RegisterWheelInterface();

}

void AdafruitServoHatHardwareInterface::RegisterSteerInterface() {
    std::cout << "RegisterSteerInterface not implemented" << std::endl;


    // actual steer joints
    this->RegisterInterfaceHandles(
            _joint_state_interface,
            _front_steer_joint_position_command_interface,
            _front_steer_joint_name,
            _front_steer_joint_position,
            _front_steer_joint_velocity,
            _front_steer_joint_effort,
            _front_steer_joint_position_command
            );

    // virtual steer joints
    for(int i = 0; i < (int)_virtual_front_steer_joint_names.size(); i++) {
        this->RegisterInterfaceHandles(
                _joint_state_interface,
                _front_steer_joint_position_command_interface,
                _virtual_front_steer_joint_names[i],
                _virtual_front_steer_joint_position[i],
                _virtual_front_steer_joint_velocity[i],
                _virtual_front_steer_joint_effort[i],
                _virtual_front_steer_joint_position_command[i]
                );
    }
}

void AdafruitServoHatHardwareInterface::RegisterWheelInterface() {
    std::cout << "RegisterWheelInterface not implemented" << std::endl;
}

void AdafruitServoHatHardwareInterface::RegisterInterfaceHandles(
        hardware_interface::JointStateInterface &joint_state_interface,
        hardware_interface::JointCommandInterface &joint_command_interface,
        const std::string joint_name,
        double &joint_position,
        double &joint_velocity,
        double &joint_effort,
        double &joint_command
        ) {

    this->RegisterJointStateInterfaceHandle(
            joint_state_interface,
            joint_name,
            joint_position,
            joint_velocity,
            joint_effort
            );

    this->RegisterCommandJointInterfaceHandle(
            joint_state_interface,
            joint_command_interface,
            joint_name,
            joint_command
            );
}

void AdafruitServoHatHardwareInterface::RegisterJointStateInterfaceHandle(
        hardware_interface::JointStateInterface &joint_state_interface,
        const std::string joint_name,
        double &joint_position,
        double &joint_velocity,
        double &joint_effort
        ) {

    hardware_interface::JointStateHandle state_handle(
            joint_name,
            &joint_position,
            &joint_velocity,
            &joint_effort
            );

    _joint_state_interface.registerHandle(state_handle);

    ROS_INFO_STREAM("Registered joint '" << joint_name << "' in the JointStateInterface");
}

void AdafruitServoHatHardwareInterface::RegisterCommandJointInterfaceHandle(
        hardware_interface::JointStateInterface &joint_state_interface,
        hardware_interface::JointCommandInterface &joint_command_interface,
        const std::string joint_name,
        double &joint_command
        ) {

    hardware_interface::JointHandle _handle(joint_state_interface.getHandle(joint_name),
                                            &joint_command
                                            );

    joint_command_interface.registerHandle(_handle);

    ROS_INFO_STREAM("Registered joint '" << joint_name << "' in the CommandJointInterface");
}



