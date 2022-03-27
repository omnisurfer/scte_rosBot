//
// Created by daniel on 3/12/22.
//
#include "adafruit_servo_hat_hw_interface.h"

AdafruitServoHatHardwareInterface::AdafruitServoHatHardwareInterface(const std::string& robot_namespace, const ros::NodeHandle& node_handle):
        _node_handle(node_handle) {

    this->_robot_namespace = robot_namespace + "/steer_bot_hardware_gazebo/";

    ros::NodeHandle n("~");
    std::string front_steer_joint_names("front_steer_joint");
    std::string rear_wheel_joint_names("rear_wheel_joint");

    /*
    steer_bot_hardware_gazebo_cpp: jnt name front_left_steer_joint
    steer_bot_hardware_gazebo_cpp: jnt name front_left_wheel_joint
    steer_bot_hardware_gazebo_cpp: jnt name front_right_steer_joint
    steer_bot_hardware_gazebo_cpp: jnt name front_right_wheel_joint
    steer_bot_hardware_gazebo_cpp: jnt name front_steer_joint
    steer_bot_hardware_gazebo_cpp: jnt name rear_left_wheel_joint
    steer_bot_hardware_gazebo_cpp: jnt name rear_right_wheel_joint
    steer_bot_hardware_gazebo_cpp: jnt name rear_wheel_joint
     *
     */

    std::vector<std::string> virtual_wheels_names;
    /*
    virtual_wheels_names.emplace_back("base_to_right_rear_wheel");
    virtual_wheels_names.emplace_back("base_to_left_rear_wheel");
    virtual_wheels_names.emplace_back("base_to_right_front_wheel");
    virtual_wheels_names.emplace_back("base_to_left_front_wheel");
    virtual_wheels_names.emplace_back("base_to_right_front_steer");
    virtual_wheels_names.emplace_back("base_to_left_front_steer");
    */
    virtual_wheels_names.emplace_back("rear_right_wheel_joint");
    virtual_wheels_names.emplace_back("rear_left_wheel_joint");
    virtual_wheels_names.emplace_back("front_right_wheel_joint");
    virtual_wheels_names.emplace_back("front_left_wheel_joint");
    virtual_wheels_names.emplace_back("front_right_steer_joint");
    virtual_wheels_names.emplace_back("front_left_steer_joint");

    virtual_wheels_velocities.resize(6);
    virtual_wheels_position.resize(6);
    virtual_wheels_effort.resize(6);

    hardware_interface::JointStateHandle front_steer_state_handle(
            front_steer_joint_names,
            &front_steer_position,
            &front_steer_velocity,
            &front_steer_effort
    );
    joint_state_interface.registerHandle(front_steer_state_handle);

    hardware_interface::JointHandle front_steer_position_cmd_handle(
            joint_state_interface.getHandle(front_steer_joint_names),
            &front_steer_position_cmd
    );
    front_steer_joint_position_cmd_interface.registerHandle(front_steer_position_cmd_handle);

    hardware_interface::JointStateHandle rear_wheel_state_handle(
            rear_wheel_joint_names,
            &rear_wheel_position,
            &rear_wheel_velocity,
            &rear_wheel_effort
    );
    joint_state_interface.registerHandle(rear_wheel_state_handle);

    hardware_interface::JointHandle rear_wheel_velocity_cmd_handle(
            joint_state_interface.getHandle(rear_wheel_joint_names),
            &rear_wheel_velocity_cmd
    );
    rear_wheel_joint_velocity_cmd_interface.registerHandle(rear_wheel_velocity_cmd_handle);

    this->registerVirtualJointState(
            virtual_wheels_position,
            virtual_wheels_velocities,
            virtual_wheels_effort,
            virtual_wheels_names
    );

    registerInterface(&front_steer_joint_position_cmd_interface);
    registerInterface(&rear_wheel_joint_velocity_cmd_interface);
    registerInterface(&joint_state_interface);
}

void AdafruitServoHatHardwareInterface::registerVirtualJointState(std::vector<double> &virtual_wheel_positions,
                                                                  std::vector<double> &virtual_wheel_velocities,
                                                                  std::vector<double> &virtual_wheel_efforts,
                                                                  std::vector<std::string> &virtual_wheels_names) {
    for(int i = 0; i < 6; ++i) {
        hardware_interface::JointStateHandle state_handle(
                virtual_wheels_names[i],
                &virtual_wheel_positions[i],
                &virtual_wheel_velocities[i],
                &virtual_wheel_efforts[i]
                );
        joint_state_interface.registerHandle(state_handle);
    }
}

// take data from hardware and send to ROS
void AdafruitServoHatHardwareInterface::read(ros::Time time, ros::Duration period) {

#if 1
    /* DEBUG JOINT STATE */
    // TODO replace this with joint position read from the adafruit servo hat driver
    sensor_msgs::JointState joints_state = sensor_msgs::JointState();

    joints_state.position.resize(6);
    joints_state.velocity.resize(6);
    joints_state.effort.resize(6);

    static float i = 0.0;
    i += 0.1;

    if(i > 1.0) {
        i = 0.0;
    }

    joints_state.position[JOINT_INDEX_FRONT] = i;
    joints_state.position[JOINT_INDEX_REAR_LEFT] = i;
    joints_state.position[JOINT_INDEX_REAR_RIGHT] = i;

    joints_state.velocity[JOINT_INDEX_REAR_LEFT] = i;
    joints_state.velocity[JOINT_INDEX_REAR_RIGHT] = i;
    /* END DEBUG JOINT STATE */
#endif

    front_steer_position = joints_state.position[JOINT_INDEX_FRONT];
    rear_wheel_position =
            (joints_state.position[JOINT_INDEX_REAR_RIGHT] + joints_state.position[JOINT_INDEX_REAR_LEFT]) / 2.0;
    rear_wheel_velocity =
            (joints_state.velocity[JOINT_INDEX_REAR_RIGHT] + joints_state.velocity[JOINT_INDEX_REAR_LEFT]) / 2.0;

    const double h = 0.75;
    const double w = 0.28;

    virtual_wheels_velocities[VIRTUAL_JOINT_IND_RIGHT_REAR] = joints_state.velocity[JOINT_INDEX_REAR_RIGHT];
    virtual_wheels_position[VIRTUAL_JOINT_IND_RIGHT_REAR] = joints_state.position[JOINT_INDEX_REAR_RIGHT];
    virtual_wheels_velocities[VIRTUAL_JOINT_IND_LEFT_REAR] = joints_state.velocity[JOINT_INDEX_REAR_LEFT];
    virtual_wheels_position[VIRTUAL_JOINT_IND_LEFT_REAR] = joints_state.position[JOINT_INDEX_REAR_LEFT];

    virtual_wheels_velocities[VIRTUAL_JOINT_IND_RIGHT_FRONT] = virtual_wheels_velocities[VIRTUAL_JOINT_IND_RIGHT_REAR];
    virtual_wheels_position[VIRTUAL_JOINT_IND_RIGHT_FRONT] = virtual_wheels_position[VIRTUAL_JOINT_IND_RIGHT_REAR];
    virtual_wheels_velocities[VIRTUAL_JOINT_IND_LEFT_FRONT] = virtual_wheels_velocities[VIRTUAL_JOINT_IND_LEFT_REAR];
    virtual_wheels_position[VIRTUAL_JOINT_IND_LEFT_FRONT] = virtual_wheels_position[VIRTUAL_JOINT_IND_LEFT_REAR];

    virtual_wheels_position[VIRTUAL_JOINT_IND_RIGHT_FRONT_STEER] =
            atan2(2.0*h*tan(front_steer_position),
            2*h + w/2.0*tan(front_steer_position)
            );
    virtual_wheels_position[VIRTUAL_JOINT_IND_LEFT_FRONT_STEER]  =
            atan2(2.0*h*tan(front_steer_position),
                  2*h - w/2.0*tan(front_steer_position)
                  );
}

// take commands from ROS and send to hardware
void AdafruitServoHatHardwareInterface::write(ros::Time time, ros::Duration period) {
    // TODO write out the desired steer and velocity command to the servo hat here

    if(rear_wheel_velocity_cmd > 0.0) {
        std::cout << "rear vel cmd: " << rear_wheel_velocity_cmd << " steer cmd: " << front_steer_position_cmd << std::endl;
    }

}
