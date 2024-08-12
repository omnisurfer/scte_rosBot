//
// Created by daniel on 3/12/22.
//
#include "adafruit_servo_hat_hw_interface.h"

AdafruitServoHatHardwareInterface::AdafruitServoHatHardwareInterface(const std::string& robot_namespace, const ros::NodeHandle& node_handle):
        node_handle_(node_handle) 
{
    
    this->_robot_namespace = ros::this_node::getName() + "/";

    ros::NodeHandle n("~");

    // front_steer_joint
    std::string front_steer_joint_names("front_steer_joint");
    
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
    
    // rear_wheel_joint
    std::string rear_wheel_joint_names("rear_wheel_joint");
    
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

    std::vector<std::string> virtual_wheels_names;

    virtual_wheels_names.emplace_back("rear_right_wheel_joint");
    virtual_wheels_names.emplace_back("rear_left_wheel_joint");
    virtual_wheels_names.emplace_back("front_right_wheel_joint");
    virtual_wheels_names.emplace_back("front_left_wheel_joint");
    virtual_wheels_names.emplace_back("front_right_steer_joint");
    virtual_wheels_names.emplace_back("front_left_steer_joint");

    virtual_wheels_velocities.resize(6);
    virtual_wheels_position.resize(6);
    virtual_wheels_effort.resize(6);
    
    this->registerVirtualJointState(
        virtual_wheels_position,
        virtual_wheels_velocities,
        virtual_wheels_effort,
        virtual_wheels_names
    );

    // register interfaces
    registerInterface(&front_steer_joint_position_cmd_interface);
    registerInterface(&rear_wheel_joint_velocity_cmd_interface);
    registerInterface(&joint_state_interface);

    // Status publisher
    servo_hat_status_pub_ = n.advertise<adafruit_servo_hat::AdafruitServoHatStatus>("status", 10);
}

void AdafruitServoHatHardwareInterface::registerVirtualJointState(std::vector<double> &virtual_wheel_positions,
                                                                  std::vector<double> &virtual_wheel_velocities,
                                                                  std::vector<double> &virtual_wheel_efforts,
                                                                  std::vector<std::string> &virtual_wheels_names) 
{
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

    double current_linear_velocity_x;
    double current_angular_velocity_z;

    this->get_odometry_update(current_linear_velocity_x, current_angular_velocity_z);
        
    /* Odometry update */

    /*
     * Look at https://github.com/CIR-KIT/steer_drive_ros/blob/kinetic-devel/steer_drive_controller/src/steer_drive_controller.cpp
     * for reference
     */
    if (open_loop_odom_) {
        odometry_.updateOpenLoop(current_linear_velocity_x, current_angular_velocity_z, time);
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
    static double wheel_position = 0.0;

    double max_velocity_ms = this->max_linear_x_speed_m_s_;
    double tire_radius_m = this->tire_radius_m_;
    double tire_circumference_m = 2 * M_PI * tire_radius_m; // 0.314m
    double max_rpm = (max_velocity_ms / tire_circumference_m);

    double cmd_vel_rpm = (current_linear_velocity_x / max_velocity_ms) * max_rpm;    
    wheel_position += (current_linear_velocity_x / max_velocity_ms) * period.toSec();
    
    //ROS_DEBUG_THROTTLE(3.0, "wheel/vel/period %f/%f/%f", wheel_position, cmd_vel_rpm, period.toSec());
    
    // TODO populate with real values
    joints_state.position[JOINT_INDEX_FRONT] = current_angular_velocity_z;
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

    const double wheel_separation_h = this->wheel_separation_h_;
    const double wheel_separation_w = this->wheel_separation_w_;

    virtual_wheels_velocities[VIRTUAL_JOINT_IND_RIGHT_REAR] = joints_state.velocity[JOINT_INDEX_REAR_RIGHT];
    virtual_wheels_position[VIRTUAL_JOINT_IND_RIGHT_REAR] = joints_state.position[JOINT_INDEX_REAR_RIGHT];
    virtual_wheels_velocities[VIRTUAL_JOINT_IND_LEFT_REAR] = joints_state.velocity[JOINT_INDEX_REAR_LEFT];
    virtual_wheels_position[VIRTUAL_JOINT_IND_LEFT_REAR] = joints_state.position[JOINT_INDEX_REAR_LEFT];

    virtual_wheels_velocities[VIRTUAL_JOINT_IND_RIGHT_FRONT] = virtual_wheels_velocities[VIRTUAL_JOINT_IND_RIGHT_REAR];
    virtual_wheels_position[VIRTUAL_JOINT_IND_RIGHT_FRONT] = virtual_wheels_position[VIRTUAL_JOINT_IND_RIGHT_REAR];
    virtual_wheels_velocities[VIRTUAL_JOINT_IND_LEFT_FRONT] = virtual_wheels_velocities[VIRTUAL_JOINT_IND_LEFT_REAR];
    virtual_wheels_position[VIRTUAL_JOINT_IND_LEFT_FRONT] = virtual_wheels_position[VIRTUAL_JOINT_IND_LEFT_REAR];

    virtual_wheels_position[VIRTUAL_JOINT_IND_RIGHT_FRONT_STEER] =
            atan2(2.0 * wheel_separation_h * tan(front_steer_position),
                  2 * wheel_separation_h + wheel_separation_w / 2.0 * tan(front_steer_position)
            );
    virtual_wheels_position[VIRTUAL_JOINT_IND_LEFT_FRONT_STEER]  =
            atan2(2.0 * wheel_separation_h * tan(front_steer_position),
                  2 * wheel_separation_h - wheel_separation_w / 2.0 * tan(front_steer_position)
                  );
    
    // TODO figure out minimum time dt so that I don't mis-publish
    // Compute and store orientation info

    const geometry_msgs::Quaternion orientation(
            tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

    nav_msgs::Odometry odom_msg;

    odom_msg.header.stamp = time;
    odom_msg.pose.pose.position.x = odometry_.getX();
    odom_msg.pose.pose.position.y = odometry_.getY();
    odom_msg.pose.pose.orientation = orientation;

    odom_msg.twist.twist.linear.x = odometry_.getLinear();
    odom_msg.twist.twist.angular.z = odometry_.getAngular();

    odom_publisher_.publish(odom_msg);

    if(enable_odom_tf_) {

        tf::Transform _transform;
        tf::Quaternion _q_rot;

        _transform.setOrigin(tf::Vector3(odometry_.getX(), odometry_.getY(), 0.0));
        _q_rot.setRPY(0.0, 0.0, odometry_.getHeading());

        _transform.setRotation(_q_rot);
        tf_odom_broadcaster_.sendTransform(
                tf::StampedTransform(_transform, ros::Time::now(), "world", _robot_namespace)
                );
    }
}

// take commands from ROS and send to hardware
void AdafruitServoHatHardwareInterface::write(ros::Time time, ros::Duration period) {
    // TODO write out the desired steer and velocity command to the servo hat here

    double linear_x_velocity, angular_z_velocity;
    double linear_x_velocity_pwm, angular_z_velocity_pwm;

    this->current_command_mutex_.lock();
    {
        linear_x_velocity = this->current_commanded_linear_x_velocity_;
        angular_z_velocity = this->current_commanded_angular_z_velocity_;
    }
    this->current_command_mutex_.unlock();

    linear_x_velocity_pwm = command_linear_x_velocity(linear_x_velocity);
    angular_z_velocity_pwm = command_angular_z_velocity(angular_z_velocity);

    if(rear_wheel_velocity_cmd > 0.0) {
        ROS_DEBUG_THROTTLE(3.0, "write: rear vel cmd %f steer cmd %f", rear_wheel_velocity_cmd, front_steer_position_cmd);
    }

    adafruit_servo_hat::AdafruitServoHatStatus msg;
    msg.commanded_angular_z_pwm = angular_z_velocity_pwm;
    msg.commanded_angular_z_velocity = angular_z_velocity;
    msg.commanded_linear_x_pwm = linear_x_velocity_pwm;
    msg.commanded_linear_x_velocity = linear_x_velocity;

    servo_hat_status_pub_.publish(msg);
}

void AdafruitServoHatHardwareInterface::brake() {

    // TODO need to mutex this
    rear_wheel_velocity_cmd = 0.0;
    front_steer_position_cmd = 0.0;

    this->command_linear_x_velocity(0.0);
    this->command_angular_z_velocity(0.0);
}
