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

    node_handle.getParam(robot_namespace + "wheel_separation_w", _wheel_separation_w);
    node_handle.getParam(robot_namespace + "wheel_separation_h", _wheel_separation_h);
    ROS_INFO_STREAM("wheel_separation_w = " << _wheel_separation_w);
    ROS_INFO_STREAM("wheel_separation_h = " << _wheel_separation_h);

    // position joint limits interface
    /**/
    std::vector<std::string> front_steer_joint_position_command_interface_handle_names = _front_steer_joint_position_command_interface.getNames();
    for (size_t i = 0; i < front_steer_joint_position_command_interface_handle_names.size(); ++i) {

        const std::string front_steer_interface_name = front_steer_joint_position_command_interface_handle_names[i];

        ROS_INFO_STREAM("front_steer_interface_name " << front_steer_interface_name);

        // skip non position interface for steer
        if(front_steer_interface_name != _virtual_front_steer_joint_names[INDEX_RIGHT] && front_steer_interface_name != _virtual_front_steer_joint_names[INDEX_LEFT])
            continue;

        hardware_interface::JointHandle front_steer_command_handle = _front_steer_joint_position_command_interface.getHandle(front_steer_interface_name);

        ROS_INFO_STREAM("front_steer_command_handle " << front_steer_command_handle.getName());
    }
    /**/

    jointStatePublisher = this->_node_handle.advertise<sensor_msgs::JointState>("joint_states", 1);

    return true;
}

// take data from hardware and send to ROS
void AdafruitServoHatHardwareInterface::read(ros::Time time, ros::Duration period) {

    //std::cout << "read " << _virtual_rear_wheel_joint_position.size() << std::endl;

    _front_steer_joint_position = 0.5;
    _rear_wheel_joint_position = 0.5;
    _rear_wheel_joint_velocity = 0.0;

    const double h = 0.75;
    const double w = 0.28;

    _virtual_rear_wheel_joint_velocity[0] = 0.1;
    _virtual_rear_wheel_joint_position[0] = 0.0;
    _virtual_rear_wheel_joint_velocity[1] = 0.1;
    _virtual_rear_wheel_joint_position[1] = 0.0;
}

// take commands from ROS and send to hardware
void AdafruitServoHatHardwareInterface::write(ros::Time time, ros::Duration period) {
    //std::cout << "write not implemented" << std::endl;

    sensor_msgs::JointState joint_state;

    // https://choreonoid.org/en/documents/latest/ros/tank-tutorial/step2.html

    /*
header:
seq: 1950
stamp:
secs: 216
nsecs: 603000000
frame_id: ''
name:
- front_left_steer_joint
- front_left_wheel_joint
- front_right_steer_joint
- front_right_wheel_joint
- front_steer_joint
- rear_left_wheel_joint
- rear_right_wheel_joint
- rear_wheel_joint
position: [0.0, 0.0, 0.0, 0.0, 0.6, 0.0, 0.0, 0.0]
velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
     */

    joint_state.header.stamp = ros::Time::now();

    joint_state.name.resize(8);
    joint_state.position.resize(8);
    joint_state.velocity.resize(8);
    joint_state.effort.resize(8);

    joint_state.name[0] = "front_left_steer_joint";
    joint_state.name[1] = "front_left_wheel_joint";
    joint_state.name[2] = "front_right_steer_joint";
    joint_state.name[3] = "front_right_wheel_joint";
    joint_state.name[4] = "front_steer_joint";
    joint_state.name[5] = "rear_left_wheel_joint";
    joint_state.name[6] = "rear_right_wheel_joint";
    joint_state.name[7] = "rear_wheel_joint";

    joint_state.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,};
    /*
    joint_state.position[1] = 0.0;
    joint_state.position[2] = 0.0;
    joint_state.position[3] = 0.0;
    joint_state.position[4] = 0.0;
    joint_state.position[5] = 0.0;
    joint_state.position[6] = 0.0;
    joint_state.position[7] = 0.0;
    */

    joint_state.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,};
    /*
    joint_state.velocity[1] = 0.0;
    joint_state.velocity[2] = 0.0;
    joint_state.velocity[3] = 0.0;
    joint_state.velocity[4] = 0.0;
    joint_state.velocity[5] = 0.0;
    joint_state.velocity[6] = 0.0;
    joint_state.velocity[7] = 0.0;
    */

    joint_state.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,};
    /*
    joint_state.effort[1] = 0.0;
    joint_state.effort[2] = 0.0;
    joint_state.effort[3] = 0.0;
    joint_state.effort[4] = 0.0;
    joint_state.effort[5] = 0.0;
    joint_state.effort[6] = 0.0;
    joint_state.effort[7] = 0.0;
    */

    jointStatePublisher.publish(joint_state);

    _rear_wheel_joint_position = 0.5;
    _front_steer_joint_position = 0.5;
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

    registerInterface(&_joint_state_interface);
    //registerInterface(&_rear_wheel_joint_velocity_command_interface);
    registerInterface(&_front_steer_joint_position_command_interface);
}

void AdafruitServoHatHardwareInterface::RegisterSteerInterface() {

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

    this->RegisterInterfaceHandles(
            _joint_state_interface,
            _rear_wheel_joint_velocity_command_interface,
            _rear_wheel_joint_name,
            _rear_wheel_joint_position,
            _rear_wheel_joint_velocity,
            _rear_wheel_joint_effort,
            _rear_wheel_joint_velocity_command
            );

    // virtual rear wheel joints
    for(int i = 0; i < (int)_virtual_rear_wheel_joint_names.size(); i++) {
        this->RegisterInterfaceHandles(
                _joint_state_interface,
                _rear_wheel_joint_velocity_command_interface,
                _virtual_rear_wheel_joint_names[i],
                _virtual_rear_wheel_joint_position[i],
                _virtual_front_steer_joint_velocity[i],
                _virtual_rear_wheel_joint_effort[i],
                _virtual_rear_wheel_joint_velocity_command[i]
                );
    }

    // virtual front wheel joints
    for(int i = 0; i < (int)_virtual_front_wheel_joint_names.size(); i++) {
        this->RegisterInterfaceHandles(
                _joint_state_interface,
                _virtual_front_wheel_joint_names[i],
                _virtual_front_wheel_joint_position[i],
                _virtual_front_wheel_joint_velocity[i],
                _virtual_front_wheel_joint_effort[i]
                );
    }
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

void AdafruitServoHatHardwareInterface::RegisterInterfaceHandles(
        hardware_interface::JointStateInterface& joint_state_interface,
        const std::string joint_name,
        double& joint_position,
        double& joint_velocity,
        double& joint_effort
        ) {

    this->RegisterJointStateInterfaceHandle(joint_state_interface,
                                            joint_name,
                                            joint_position,
                                            joint_velocity,
                                            joint_effort);
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



