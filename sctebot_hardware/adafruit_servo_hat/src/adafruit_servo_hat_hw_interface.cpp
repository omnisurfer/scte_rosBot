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

    double linear_velocity_x;
    double angular_position_z;

    this->get_odometry_update(linear_velocity_x, angular_position_z);

    /* Odometry update */

    bool _open_loop_odom = true;
    bool _enable_odom_tf = true;

    /*
     * Look at https://github.com/CIR-KIT/steer_drive_ros/blob/kinetic-devel/steer_drive_controller/src/steer_drive_controller.cpp
     * for reference
     */
    if (_open_loop_odom) {
        _odometry.updateOpenLoop(linear_velocity_x, linear_velocity_x, time);
    }
    else {
        // TODO read in real positions
    }

#if 1
    /* DEBUG JOINT STATE */
    sensor_msgs::JointState joints_state = sensor_msgs::JointState();

    joints_state.position.resize(6);
    joints_state.velocity.resize(6);
    joints_state.effort.resize(6);

    /*
     * 0.4m/s ~ 0.9MPH ~ 1.3RPM
     * 2.0m/s ~ 4.5MPH ~ 6.4RPM
     */
    static int64_t sample_at_loop_rate = 0;
    static double wheel_position = 0.0;

    double max_velocity_ms = 2.0;
    double tire_radius = 0.05;
    double tire_circumference_m = 2 * M_PI * tire_radius; // 0.314m
    double max_rpm = (max_velocity_ms / tire_circumference_m);

    double cmd_vel_rpm = (linear_velocity_x / max_velocity_ms) * max_rpm;

    // wheel_position = sin(double(sample_at_loop_rate) * period.toSec() * cmd_vel_rpm);
    wheel_position += (linear_velocity_x / max_velocity_ms) * period.toSec();

    // std::cout << "wheel/vel/period" << wheel_position << "," << cmd_vel_rpm << "," << period.toSec() << std::endl;

    sample_at_loop_rate = (sample_at_loop_rate + 1);

    // TODO populate with real values
    joints_state.position[JOINT_INDEX_FRONT] = angular_position_z;
    joints_state.position[JOINT_INDEX_REAR_LEFT] = wheel_position;
    joints_state.position[JOINT_INDEX_REAR_RIGHT] = wheel_position;

    // velocity state does not seem to update the visual transform. Unsure what else it may do.
    //joints_state.velocity[JOINT_INDEX_REAR_LEFT] = wheel_position;
    //joints_state.velocity[JOINT_INDEX_REAR_RIGHT] = wheel_position;
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

    // region Publish odometry

    // TODO figure out minimum time dt so that I don't mis-publish
    // Compute and store orientation info

    const geometry_msgs::Quaternion orientation(
            tf::createQuaternionMsgFromYaw(_odometry.getHeading()));

#if ENABLE_REALTIME_PUBLISHERS

    if(_odom_realtime_publisher->trylock()) {

        _odom_realtime_publisher->msg_.header.stamp = time;
        _odom_realtime_publisher->msg_.pose.pose.position.x = _odometry.getX();
        _odom_realtime_publisher->msg_.pose.pose.position.y = _odometry.getY();
        _odom_realtime_publisher->msg_.pose.pose.orientation = orientation;
        _odom_realtime_publisher->msg_.twist.twist.linear.x = _odometry.getLinear();
        _odom_realtime_publisher->msg_.twist.twist.angular.z = _odometry.getAngular();

        _odom_realtime_publisher->unlockAndPublish();
    }
#else
    nav_msgs::Odometry _odom_msg;

    _odom_msg.header.stamp = time;
    _odom_msg.pose.pose.position.x = _odometry.getX();
    _odom_msg.pose.pose.position.y = _odometry.getY();
    _odom_msg.pose.pose.orientation = orientation;

    _odom_msg.twist.twist.linear.x = _odometry.getLinear();
    _odom_msg.twist.twist.angular.z = _odometry.getAngular();

    _odom_publisher.publish(_odom_msg);

#endif

#if ENABLE_REALTIME_PUBLISHERS
    // May not need the tf publisher. I think the controller hardware interface will take care of this...
    if(_enable_odom_tf && _tf_odom_realtime_publisher->trylock()) {
        geometry_msgs::TransformStamped& odom_frame = _tf_odom_realtime_publisher->msg_.transforms[0];
        odom_frame.header.stamp = time;
        odom_frame.transform.translation.x = _odometry.getX();
        odom_frame.transform.translation.y = _odometry.getY();
        odom_frame.transform.rotation = orientation;
        _tf_odom_realtime_publisher->unlockAndPublish();
    }
#else
    if(_enable_odom_tf) {

        tf::Transform _transform;
        tf::Quaternion _q_rot;

        _transform.setOrigin(tf::Vector3(_odometry.getX(), _odometry.getY(), 0.0));
        _q_rot.setRPY(0.0, 0.0, _odometry.getHeading());

        _transform.setRotation(_q_rot);
        _tf_odom_broadcaster.sendTransform(
                tf::StampedTransform(_transform, ros::Time::now(), "world", _robot_namespace)
                );
    }
#endif
    // endregion
}

// take commands from ROS and send to hardware
void AdafruitServoHatHardwareInterface::write(ros::Time time, ros::Duration period) {
    // TODO write out the desired steer and velocity command to the servo hat here

    if(rear_wheel_velocity_cmd > 0.0 && false) {
        std::cout << "rear vel cmd: " << rear_wheel_velocity_cmd << " steer cmd: " << front_steer_position_cmd << std::endl;
    }

}

void AdafruitServoHatHardwareInterface::brake() {

    // TODO need to mutex this
    rear_wheel_velocity_cmd = 0.0;
    front_steer_position_cmd = 0.0;

}
