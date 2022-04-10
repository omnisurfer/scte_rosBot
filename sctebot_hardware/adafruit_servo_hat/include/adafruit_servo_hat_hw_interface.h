//
// Created by daniel on 3/12/22.
//

/*
 * referencing
 * https://github.com/CIR-KIT/steer_drive_ros/blob/kinetic-devel/steer_bot_hardware_gazebo/src/steer_bot_hardware_gazebo.cpp
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

#include <nav_msgs/Odometry.h>

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
            double max_angular_rad_s,
            void (*handle_pca9685_status)(int x, int y)
    ) {

        bool init_ok = true;

        pca9685DeviceHandle.reset(new Pca9685LEDController());

        _max_linear_x_speed_m_s = max_linear_speed_m_s;
        _max_angular_z_rad_s = max_angular_rad_s;

        _i2c_bus_number = i2c_bus_number;


#if ENABLE_PCA9685_LED_DEVICE
        _i2c_device_address = PCA9685_RPI_ADDRESS;

        pca9685DeviceHandle->config_device(
                _i2c_bus_number,
                _i2c_device_address,
                10,
                "pca9685_led_pwm",
                handle_pca9685_status
        );

        std::cout << "connecting to " << _i2c_bus_number << " at " << _i2c_device_address << std::endl;

        if(!pca9685DeviceHandle->connect_to_device()) {
            init_ok = false;
        }
#endif

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

        pca9685DeviceHandle->init_device(
                op_pwm_max_count_cycle,
                op_pwm_on_delay,
                op_pwm_min_limit_duty_cycle,
                op_pwm_max_limit_duty_cycle,
                op_pwm_min_operating_duty_cycle,
                op_pwm_max_operating_duty_cycle
        );
#endif
    }

    void command_liner_x_velocity(double cmd_linear_x_velocity) {

        this->_debug_current_commanded_linear_x_velocity = cmd_linear_x_velocity;

        double cmd_linear_pwm;
        cmd_linear_pwm = (cmd_linear_x_velocity / this->_max_linear_x_speed_m_s) * 0.5 + 0.5;

        // TODO these calls will go into the write command
        this->command_pwm(Pca9685LEDController::LED0, float(cmd_linear_pwm));
    }

    void command_angular_z_velocity(double cmd_angular_z_velocity) {

        this->_debug_current_commanded_angular_z_velocity = cmd_angular_z_velocity;

        double cmd_angular_pwm;
        cmd_angular_pwm = (cmd_angular_z_velocity / this->_max_angular_z_rad_s) * 0.5 + 0.5;

        // TODO these calls will go into the write command
        this->command_pwm(Pca9685LEDController::LED1, float(cmd_angular_pwm));
    }

    void command_pwm(Pca9685LEDController::LEDn led_n, float pwm_on_percent) {

        pca9685DeviceHandle->set_pwm(led_n, pwm_on_percent);

    }

    void get_odometry_update(double& linear_x_velocity, double& angular_z_velocity) {
        linear_x_velocity = this->_debug_current_commanded_linear_x_velocity;
        angular_z_velocity = this->_debug_current_commanded_angular_z_velocity;
    }

private:

    ros::NodeHandle _node_handle;
    std::string _robot_namespace;

    int _i2c_bus_number;
    int _i2c_device_address = 0;

    double _max_linear_x_speed_m_s;
    double _max_angular_z_rad_s;

    // Using these for debug. If needed, should probably put a mutex around them
    double _debug_current_commanded_linear_x_velocity;
    double _debug_current_commanded_angular_z_velocity;

    std::unique_ptr<Pca9685LEDController> pca9685DeviceHandle;

    void publishSteer(double angle_cmd);

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

};

#endif //ADAFRUIT_SERVO_HAT_ADAFRUIT_SERVO_HAT_HW_INTERFACE_H
