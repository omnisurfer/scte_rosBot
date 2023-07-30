//
// Created by daniel on 3/12/22.
//

/*
 * referencing
 * https://github.com/CIR-KIT/steer_drive_ros/blob/kinetic-devel/steer_bot_hardware_gazebo/src/steer_bot_hardware_gazebo.cpp
 */

#ifndef ADAFRUIT_SERVO_HAT_ADAFRUIT_SERVO_HAT_HW_INTERFACE_H
#define ADAFRUIT_SERVO_HAT_ADAFRUIT_SERVO_HAT_HW_INTERFACE_H

/*
 * Enabling realtime causes seg faults on the raspberry pi.
 * Not sure why.
 */
#define ENABLE_REALTIME_PUBLISHERS 0

#define ENABLE_PCA9685_LED_DEVICE 1
#define PCA9685_RPI_ADDRESS 0x40

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <iostream>
#include <thread>
#include <utility>
#include <condition_variable>

#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <nav_msgs/Odometry.h>

#include "odometry.h"
#include "pca9685.h"

/*
 * Using as reference
 * CIRKIT Unit03 Robot Base for Reference - cirkit_unit03_hw.cpp
 * http://wiki.ros.org/ros_control/Tutorials/Create%20your%20own%20hardware%20interface
 * http://wiki.ros.org/ros_control
 * http://wiki.ros.org/tf/Tutorials
 *
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

private:

    ros::NodeHandle _node_handle;
    std::string _robot_namespace;

    int _i2c_bus_number;
    int _i2c_device_address = 0;

    double _max_linear_x_speed_m_s;
    double _max_linear_speed_of_vehicle_as_geared_m_s;
    double _max_angular_z_rad_s;
    double _tire_radius_m;

    double _wheel_separation_h;
    double _wheel_separation_w;

    // Using these for debug. If needed, should probably put a mutex around them
    std::mutex _current_command_mutex;
    double _current_commanded_linear_x_velocity;
    double _current_commanded_angular_z_position;

    std::unique_ptr<Pca9685LEDController> pca9685DeviceHandle;

    void publishSteer(double angle_cmd);

    void brake();

    // region Joint Variables

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

    // endregion

    // region Odometry
    ackermann_steering_controller::Odometry _odometry;

#if ENABLE_REALTIME_PUBLISHERS
    // TODO ROS noetic on raspberry pi does not support realtimePublisher!
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> _odom_realtime_publisher;
    std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > _tf_odom_realtime_publisher;
#else
    ros::Publisher _odom_publisher;
    tf::TransformBroadcaster _tf_odom_broadcaster;
#endif

    /// Wheel separation, wrt the midpoint of the wheel width:
    double wheel_separation_h_ = 1.0;

    /// Wheel radius (assuming it's the same for the left and right wheels):
    double wheel_radius_ = 0.5;

    /// Wheel separation and radius calibration multipliers:
    double wheel_separation_h_multiplier_ = 0.1;
    double wheel_radius_multiplier_ = 0.1;
    double steer_pos_multiplier_ = 0.1;

    // endregion

public:

    AdafruitServoHatHardwareInterface(const std::string& robot_namespace, const ros::NodeHandle& node_handle);
    ros::Time getTime() const { return ros::Time::now(); }
    ros::Duration getPeriod() const {return ros::Duration(0.01); }

    void read(ros::Time time, ros::Duration period);
    void write(ros::Time time, ros::Duration period);

    ~AdafruitServoHatHardwareInterface() {

    };

    int init_device(
            int i2c_bus_number,
            double max_linear_speed_m_s,
            double max_linear_speed_of_vehicle_as_geared_m_s,
            double max_angular_rad_s,
            double tire_radius_m,
            double wheel_separation_h,
            double wheel_separation_w,
            void (*handle_pca9685_status)(int x, int y)
    ) {

        bool init_ok = true;

        this->pca9685DeviceHandle.reset(new Pca9685LEDController());

        this->_max_linear_x_speed_m_s = max_linear_speed_m_s;
        this->_max_linear_speed_of_vehicle_as_geared_m_s = max_linear_speed_of_vehicle_as_geared_m_s;
        this->_max_angular_z_rad_s = max_angular_rad_s;
        this->_tire_radius_m = tire_radius_m;

        this->_wheel_separation_h = wheel_separation_h;
        this->_wheel_separation_w = wheel_separation_w;

        this->_i2c_bus_number = i2c_bus_number;

#if ENABLE_PCA9685_LED_DEVICE
        // region ENABLE_PCA9685_LED_DEVICE

        this->_i2c_device_address = PCA9685_RPI_ADDRESS;

        this->pca9685DeviceHandle->config_device(
                this->_i2c_bus_number,
                this->_i2c_device_address,
                10,
                "pca9685_led_pwm",
                handle_pca9685_status
        );

        std::cout << "connecting to " << this->_i2c_bus_number << " at " << this->_i2c_device_address << std::endl;

        if(!this->pca9685DeviceHandle->connect_to_device()) {
            init_ok = false;
        }

        // endregion
#endif

        // region Odometry
        int velocity_rolling_window_size = 10;

        _odometry.setVelocityRollingWindowSize(velocity_rolling_window_size);

        const double wheel_separation_height = wheel_separation_h_multiplier_ * wheel_separation_h_;
        const double wheel_radius = wheel_radius_multiplier_ * wheel_radius_;;

        _odometry.setWheelParams(wheel_separation_height,wheel_radius);

        std::string frame_id = "world";
        std::string child_frame_id = "base_frame";
        double pose_position_z = 0.0;

        boost::array<double, 36> pose_covariance = {
                0.01, 0., 0., 0., 0., 0.,
                0., 0.01, 0., 0., 0., 0.,
                0., 0., 0.01, 0., 0., 0.,
                0., 0., 0., 0.01, 0., 0.,
                0., 0., 0., 0., 0.01, 0.,
                0., 0., 0., 0., 0., 0.01 };

        double twist_linear_y = 0.0;
        double twist_linear_z = 0.0;
        double twist_angular_x = 0.0;
        double twist_angular_y = 0.0;

        boost::array<double, 36> twist_covariance = {
                0.01, 0., 0., 0., 0., 0.,
                0., 0.01, 0., 0., 0., 0.,
                0., 0., 0.01, 0., 0., 0.,
                0., 0., 0., 0.01, 0., 0.,
                0., 0., 0., 0., 0.01, 0.,
                0., 0., 0., 0., 0., 0.01 };

        // TODO need real covariance and other configs
#if ENABLE_REALTIME_PUBLISHERS
        _odom_realtime_publisher.reset(
                new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(this->_node_handle, "odom", 100)
        );

        _odom_realtime_publisher->msg_.header.frame_id = frame_id;
        _odom_realtime_publisher->msg_.child_frame_id = child_frame_id;
        _odom_realtime_publisher->msg_.pose.pose.position.z = pose_position_z;
        _odom_realtime_publisher->msg_.pose.covariance = pose_covariance;

        _odom_realtime_publisher->msg_.twist.twist.linear.y = twist_linear_y;
        _odom_realtime_publisher->msg_.twist.twist.linear.z = twist_linear_z;
        _odom_realtime_publisher->msg_.twist.twist.angular.x = twist_angular_x;
        _odom_realtime_publisher->msg_.twist.twist.angular.y = twist_angular_y;
        _odom_realtime_publisher->msg_.twist.covariance = twist_covariance;
#else
        /*
         * This odom publisher may compliment the ackermann steering controller odom which may be based
         * purely on dead reckoning.
         */
        _odom_publisher = _node_handle.advertise<nav_msgs::Odometry>("odom", 100);

        nav_msgs::Odometry _odom_msg;
        _odom_msg.header.frame_id = frame_id;
        _odom_msg.child_frame_id = child_frame_id;
        _odom_msg.pose.pose.position.z = pose_position_z;
        _odom_msg.pose.covariance = pose_covariance;

        _odom_msg.twist.twist.linear.y = twist_linear_y;
        _odom_msg.twist.twist.linear.z = twist_linear_z;
        _odom_msg.twist.twist.angular.x = twist_angular_x;
        _odom_msg.twist.twist.angular.y = twist_angular_y;
        _odom_msg.twist.covariance = twist_covariance;
#endif

        // endregion

        // set commands to zero
        this->brake();

        return init_ok;
    }

    void run() {

#if ENABLE_PCA9685_LED_DEVICE

        int op_pwm_max_count_cycle = 4095;
        float op_pwm_on_percent = 0.0;
        float op_pwm_min_limit_duty_cycle = 0.03;
        float op_pwm_max_limit_duty_cycle = 0.125;
        float op_pwm_min_operating_duty_cycle = 0.03;
        float op_pwm_max_operating_duty_cycle = 0.125;
        float op_pwm_on_delay = 0.0;

        this->pca9685DeviceHandle->init_device(
                op_pwm_max_count_cycle,
                op_pwm_on_delay,
                op_pwm_min_limit_duty_cycle,
                op_pwm_max_limit_duty_cycle,
                op_pwm_min_operating_duty_cycle,
                op_pwm_max_operating_duty_cycle
        );
#endif

        void brake();

        ros::Time time = ros::Time();

        _odometry.init(time);

    }

    void command_liner_x_velocity(double cmd_linear_x_velocity) {

        this->_current_command_mutex.lock();
        {
            this->_current_commanded_linear_x_velocity = cmd_linear_x_velocity;
        }
        this->_current_command_mutex.unlock();

        double cmd_linear_pwm;

        //clamp the velocity to be within the driver max/min
        double lower_velocity_limit = this->_max_linear_x_speed_m_s * -0.25;
        double upper_velocity_limit = this->_max_linear_x_speed_m_s;
        cmd_linear_x_velocity = std::max(lower_velocity_limit, std::min(cmd_linear_x_velocity, upper_velocity_limit));

        cmd_linear_pwm = (cmd_linear_x_velocity / this->_max_linear_speed_of_vehicle_as_geared_m_s) * 0.5 + 0.5;

        std::cout << "cmd_lin_x: " << cmd_linear_x_velocity << " cmd_lin_pwm: " << cmd_linear_pwm << std::endl;

        // TODO these calls will go into the write command
        this->command_pwm(Pca9685LEDController::LED1, float(cmd_linear_pwm));
    }

    void command_angular_z_velocity(double cmd_angular_z_velocity) {

        this->_current_command_mutex.lock();
        {
            this->_current_commanded_angular_z_position = cmd_angular_z_velocity;
        }
        this->_current_command_mutex.unlock();

        double cmd_angular_pwm;

        // clamp the angular velocity
        double lower_velocity_limit = this->_max_angular_z_rad_s * -0.25;
        double upper_velocity_limit = this->_max_angular_z_rad_s;
        cmd_angular_z_velocity = std::max(lower_velocity_limit, std::min(cmd_angular_z_velocity, upper_velocity_limit));

        cmd_angular_pwm = (cmd_angular_z_velocity / this->_max_angular_z_rad_s) * 0.5 + 0.5;

        std::cout << "cmd_ang_x: " << cmd_angular_z_velocity << " cmd_ang_pwm: " << cmd_angular_pwm << std::endl;

        // TODO these calls will go into the write command
        this->command_pwm(Pca9685LEDController::LED0, float(cmd_angular_pwm));
    }

    void command_pwm(Pca9685LEDController::LEDn led_n, float pwm_on_percent) {

        this->pca9685DeviceHandle->set_pwm(led_n, pwm_on_percent);

    }

    void get_odometry_update(double& linear_x_velocity, double& angular_z_position) {

        this->_current_command_mutex.lock();
        {
            linear_x_velocity = this->_current_commanded_linear_x_velocity;
            angular_z_position = this->_current_commanded_angular_z_position;
        }
        this->_current_command_mutex.unlock();

        //std::cout << "get_odometry_update x " << linear_x_velocity << " z " << angular_z_position << std::endl;
    }
};

#endif //ADAFRUIT_SERVO_HAT_ADAFRUIT_SERVO_HAT_HW_INTERFACE_H
