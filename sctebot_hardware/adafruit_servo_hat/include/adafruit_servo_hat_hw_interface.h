//
// Created by daniel on 3/12/22.
//

/*
 * referencing
 * https://github.com/CIR-KIT/steer_drive_ros/blob/kinetic-devel/steer_bot_hardware_gazebo/src/steer_bot_hardware_gazebo.cpp
 * Modified by removing realtime publisher because of incompatibility with raspbery pi ROS noetic. Causes seg faults. Probably need to recopile ubuntu for realtime but
 * too much effort for EoL based ROS.
 */

#ifndef ADAFRUIT_SERVO_HAT_ADAFRUIT_SERVO_HAT_HW_INTERFACE_H
#define ADAFRUIT_SERVO_HAT_ADAFRUIT_SERVO_HAT_HW_INTERFACE_H

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

//#include "adafruit_servo_hat/AdafruitServoHatStatus.h"
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

        ros::NodeHandle node_handle_;
        std::string _robot_namespace;

        // PWM driver
        int i2c_bus_number_;
        int i2c_device_address_ = 0;

        std::unique_ptr<Pca9685LEDController> pca9685DeviceHandle;

        double max_linear_x_speed_m_s_;
        double max_linear_speed_of_vehicle_as_geared_m_s_;
        double max_angular_z_rad_s_;
        double tire_radius_m_;

        double wheel_separation_h_;
        double wheel_separation_w_;

        // Wheel separation and radius calibration multipliers
        double wheel_separation_h_multiplier_ = 1.0;
        double wheel_radius_multiplier_ = 1.0;
        double steer_pos_multiplier_ = 1.0;

        // Using these for debug. If needed, should probably put a mutex around them
        std::mutex current_command_mutex_;
        double current_commanded_linear_x_velocity_;
        double current_commanded_angular_z_velocity_;
        
        void publishSteer(double angle_cmd);

        void brake();

        // Joints
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

        ros::Publisher steer_cmd_publisher_;

        // Odometry
        bool open_loop_odom_ = true;
        bool enable_odom_tf_ = false;

        int velocity_rolling_window_size_ = 10;

        ackermann_steering_controller::Odometry odometry_;
        ros::Publisher odom_publisher_;        
        tf::TransformBroadcaster tf_odom_broadcaster_;

        ros::Publisher servo_hat_status_pub_;

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

            this->max_linear_x_speed_m_s_ = max_linear_speed_m_s;
            this->max_linear_speed_of_vehicle_as_geared_m_s_ = max_linear_speed_of_vehicle_as_geared_m_s;
            this->max_angular_z_rad_s_ = max_angular_rad_s;
            this->tire_radius_m_ = tire_radius_m;

            this->wheel_separation_h_ = wheel_separation_h;
            this->wheel_separation_w_ = wheel_separation_w;

            this->i2c_bus_number_ = i2c_bus_number;

            #if ENABLE_PCA9685_LED_DEVICE
            this->i2c_device_address_ = PCA9685_RPI_ADDRESS;

            this->pca9685DeviceHandle->config_device(
                this->i2c_bus_number_,
                this->i2c_device_address_,
                10,
                "pca9685_led_pwm",
                handle_pca9685_status
            );
            
            ROS_DEBUG("connecting to %i at %i", this->i2c_bus_number_, this->i2c_device_address_);

            if(!this->pca9685DeviceHandle->connect_to_device()) {
                init_ok = false;
            }        
            #endif

            // Odometry
            odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size_);

            const double wheel_separation_height = wheel_separation_h_multiplier_ * wheel_separation_h_;
            const double wheel_radius = wheel_radius_multiplier_ * tire_radius_m;

            odometry_.setWheelParams(wheel_separation_height, wheel_radius);

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

            // TODO need real covariance and other configs
            boost::array<double, 36> twist_covariance = {
                    0.01, 0., 0., 0., 0., 0.,
                    0., 0.01, 0., 0., 0., 0.,
                    0., 0., 0.01, 0., 0., 0.,
                    0., 0., 0., 0.01, 0., 0.,
                    0., 0., 0., 0., 0.01, 0.,
                    0., 0., 0., 0., 0., 0.01 };

            /*
            * This odom publisher may compliment the ackermann steering controller odom which may be based
            * purely on dead reckoning.
            */
            odom_publisher_ = node_handle_.advertise<nav_msgs::Odometry>(_robot_namespace + "odom", 100);

            nav_msgs::Odometry odom_msg_;
            odom_msg_.header.frame_id = frame_id;
            odom_msg_.child_frame_id = child_frame_id;
            odom_msg_.pose.pose.position.z = pose_position_z;
            odom_msg_.pose.covariance = pose_covariance;

            odom_msg_.twist.twist.linear.y = twist_linear_y;
            odom_msg_.twist.twist.linear.z = twist_linear_z;
            odom_msg_.twist.twist.angular.x = twist_angular_x;
            odom_msg_.twist.twist.angular.y = twist_angular_y;
            odom_msg_.twist.covariance = twist_covariance;
            
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

        odometry_.init(time);

    }

    void request_linear_x_velocity(double request_linear_x_velocity)
    {
        this->current_command_mutex_.lock();
        {
            this->current_commanded_linear_x_velocity_ = request_linear_x_velocity;
        }
        this->current_command_mutex_.unlock();
    }

    void request_angular_z_velocity(double request_angular_z_velocity)
    {
        this->current_command_mutex_.lock();
        {
            this->current_commanded_angular_z_velocity_ = request_angular_z_velocity;
        }
        this->current_command_mutex_.unlock();
    }

    double command_liner_x_velocity(double cmd_linear_x_velocity) {
        
        double cmd_linear_pwm;

        //clamp the velocity to be within the driver max/min
        double lower_velocity_limit = this->max_linear_x_speed_m_s_ * -0.25;
        double upper_velocity_limit = this->max_linear_x_speed_m_s_;
        cmd_linear_x_velocity = std::max(lower_velocity_limit, std::min(cmd_linear_x_velocity, upper_velocity_limit));

        cmd_linear_pwm = (cmd_linear_x_velocity / this->max_linear_speed_of_vehicle_as_geared_m_s_) * 0.5 + 0.5;
        
        ROS_DEBUG_THROTTLE(1.0, "command_liner_x_velocity: cmd_x_velocity %f cmd_linear_pwm: %f", cmd_linear_x_velocity, cmd_linear_pwm);

        // TODO these calls will go into the write command
        this->command_pwm(Pca9685LEDController::LED1, float(cmd_linear_pwm));

        return float(cmd_linear_pwm);
    }

    double command_angular_z_velocity(double cmd_angular_z_velocity) {
        
        double cmd_angular_pwm;

        // clamp the angular velocity
        double lower_velocity_limit = this->max_angular_z_rad_s_ * -0.25;
        double upper_velocity_limit = this->max_angular_z_rad_s_;
        cmd_angular_z_velocity = std::max(lower_velocity_limit, std::min(cmd_angular_z_velocity, upper_velocity_limit));

        // DMR_DEBUG_20231104 - Inverting the result. For some reason direction is inverted. Need to look into this.
        //cmd_angular_z_velocity *= -1.0;
        cmd_angular_pwm = (cmd_angular_z_velocity / this->max_angular_z_rad_s_) * 0.5 + 0.5;
        
        ROS_DEBUG_THROTTLE(3.0, "command_angular_z_velocity: cmd_z_velocity %f cmd_angular_pwm %f", cmd_angular_z_velocity, cmd_angular_pwm);

        // TODO these calls will go into the write command
        this->command_pwm(Pca9685LEDController::LED0, float(cmd_angular_pwm));

        return float(cmd_angular_pwm);
    }

    void command_pwm(Pca9685LEDController::LEDn led_n, float pwm_on_percent) {

        this->pca9685DeviceHandle->set_pwm(led_n, pwm_on_percent);

    }

    void get_odometry_update(double& linear_x_velocity, double& angular_z_velocity) {

        // A real odometry udpate would require some hardware. For now just feeding in the commanded velocities    
        this->current_command_mutex_.lock();
        {
            linear_x_velocity = this->current_commanded_linear_x_velocity_;

            if(linear_x_velocity > 0 || linear_x_velocity < 0)
            {
                angular_z_velocity = this->current_commanded_angular_z_velocity_;
            }
            else
            {
                angular_z_velocity = 0.0;
            }
        }
        this->current_command_mutex_.unlock();
        
        //ROS_DEBUG_THROTTLE(3.0, "get_odometry_update: linear_x_velocity %f angular_z_velocity %f", linear_x_velocity, angular_z_velocity);
    }
};

#endif //ADAFRUIT_SERVO_HAT_ADAFRUIT_SERVO_HAT_HW_INTERFACE_H
