//
// Created by user on 12/23/21.
//
#include <signal.h>

#include <ros/ros.h>

#include "adafruit_servo_hat.h"

void signal_handler(int sig) {

    std::cout << "ROS signal handler " << sig << std::endl;

    ros::shutdown();
}

void handle_servo_callback(int x, int y) {

    //std::cout << "servo callback called!" << std::endl;

    // Boost logging causing thread race conditions (all of them???)
    //BOOST_LOG_TRIVIAL(debug) << "handle_servo_callback " << x << " " << y;
    //BOOST_LOG_TRIVIAL(info) << "callback info message";

}

int main(int argc, char* argv[]) {

    std::string node_name = "adafruit_servo_hat_node";
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);

    ros::NodeHandle ros_node_handle;
    signal(SIGINT | SIGTERM | SIGABRT | SIGKILL, signal_handler);

    ros::Rate loop_rate(1);

    // region ROS Params
    std::string robot_namespace;

    if (ros_node_handle.getParam("/robot_namespace", robot_namespace)) {
        ROS_INFO("%s: robot_namespace %s", node_name.c_str(), robot_namespace.c_str());
    } else {
        robot_namespace = "sctebot_debug";
        ROS_WARN("%s: robot_namespace not found, using default %s", node_name.c_str(), robot_namespace.c_str());
    }
    // endregion

    bool run_ros_publisher = true;
    bool run_i2c_code = true;

#if 0
    std::unique_ptr<Pca9685LEDController> pca9685LedController(new Pca9685LEDController());

    if(run_i2c_code) {

        int i2c_bus_number = 0;
        int i2c_device_address = 0x40;
        int update_period_ms = 1000;

        // lame way to do this but good enough for debug
        if(argv[1]) {
            if (!memcmp("-d", argv[1], 2)) {

                char* p_end;
                i2c_bus_number = (int)std::strtol(argv[2], &p_end, 10);

                if (*p_end) {
                    //not sure what to do in this case
                }
            }
        }

        if(i2c_bus_number > 1) {
            ROS_WARN("%s: WARNING! I2C Bus number is %i", node_name.c_str(), i2c_bus_number);
        }

        bool init_ok = true;

        int op_pwm_max_count_cycle = 4095;
        float op_pwm_on_percent = 0.0;
        float op_pwm_min_limit_duty_cycle = 0.03;
        float op_pwm_max_limit_duty_cycle = 0.125;
        float op_pwm_min_operating_duty_cycle = 0.03;
        float op_pwm_max_operating_duty_cycle = 0.125;
        float op_pwm_on_delay = 0.0;

        pca9685LedController->config_device(
                i2c_bus_number,
                i2c_device_address,
                update_period_ms,
                "pca9685_servo",
                handle_servo_callback
        );

        if(!pca9685LedController->connect_to_device()) {
            std::cout << "pca9685 failed to connect" << std::endl;
            return 0;
        }
        else {

            pca9685LedController->init_device(
                    op_pwm_max_count_cycle,
                    op_pwm_on_delay,
                    op_pwm_min_limit_duty_cycle,
                    op_pwm_max_limit_duty_cycle,
                    op_pwm_min_operating_duty_cycle,
                    op_pwm_max_operating_duty_cycle
            );

        }

        if(init_ok) {
            //pca9685LedController->run();
            ROS_INFO("%s: Servo Hat initialization success", node_name.c_str());
        }
        else {
            ROS_WARN("%s: Servo Hat initialization failed", node_name.c_str());
            return 0;
        }
    }

    if(run_ros_publisher) {

    }
#else
    std::unique_ptr<AdaFruitServoHat> adaFruitServoHat(new AdaFruitServoHat());

    if(run_i2c_code) {

        int i2c_bus_number = 0;

        // lame way to do this but good enough for debug
        if(argv[1]) {
            if (!memcmp("-d", argv[1], 2)) {

                char* p_end;
                i2c_bus_number = (int)std::strtol(argv[2], &p_end, 10);

                if (*p_end) {
                    //not sure what to do in this case
                }
            }
        }

        if(i2c_bus_number > 1) {
            ROS_WARN("%s: WARNING! I2C Bus number is %i", node_name.c_str(), i2c_bus_number);
        }

        // TODO change this to an exception?
        bool init_ok = true;

        init_ok = adaFruitServoHat->init_device(
                i2c_bus_number,
                handle_servo_callback
                );

        if(init_ok) {
            adaFruitServoHat->run();
            ROS_INFO("%s: SERVO HAT initialization success", node_name.c_str());
        }
        else {
            ROS_WARN("%s: SERVO HAT initialization failed", node_name.c_str());
            return 0;
        }
    }

    if(run_ros_publisher) {

    }
#endif



    std::cout << "Adafruit Servo Hat node running..." << std::endl;

    while(ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();

        bool shutdown = ros::isShuttingDown();

        if(shutdown) {
            std::cout << node_name + ": Shutting down ROS node" << std::endl;
            break;
        }
    }

    if(run_ros_publisher) {

    }

    return 0;
}