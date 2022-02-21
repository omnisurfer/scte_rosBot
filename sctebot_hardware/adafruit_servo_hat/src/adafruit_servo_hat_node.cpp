//
// Created by user on 12/23/21.
//
#include <signal.h>

#include <ros/ros.h>

#include "adafruit_servo_hat.h"

// TODO put this into a class instead of a global variable. Only OK for proof of concept.
std::unique_ptr<AdaFruitServoHat> adaFruitServoHat;

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

void handle_twist_command_callback(const geometry_msgs::Twist::ConstPtr& msg) {

    ROS_INFO("I got twist msg l_x [%f] m/s, a_z [%f] rad/s", msg->linear.x, msg->angular.z);

    double linear_x = msg->linear.x + 2.0;
    double angular_z = msg->angular.z + 1.5;

    if(linear_x > 1.0) {
        linear_x = 1.0;
    }
    else if (linear_x < 0.0) {
        linear_x = 0.0;
    }

    if(angular_z > 1.0) {
        angular_z = 1.0;
    }
    else if (angular_z < 0.0) {
        angular_z = 0.0;
    }

    adaFruitServoHat->command_pwm(Pca9685LEDController::LED0, float(linear_x));
    adaFruitServoHat->command_pwm(Pca9685LEDController::LED1, float(angular_z));

}

int main(int argc, char* argv[]) {

    std::string node_name = "adafruit_servo_hat_node";
    ROS_INFO("%s: initializing node", node_name.c_str());

    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);

    ros::NodeHandle ros_node_handle;
    signal(SIGINT | SIGTERM | SIGABRT | SIGKILL, signal_handler);

    ros::Rate loop_rate(1);

    // region ROS Params
    std::string robot_namespace;

    if (ros_node_handle.getParam("/robot_namespace", robot_namespace)) {
        ROS_INFO("%s: robot_namespace %s", node_name.c_str(), robot_namespace.c_str());
    } else {
        robot_namespace = "sctebot";
        ROS_WARN("%s: robot_namespace not found, using default %s", node_name.c_str(), robot_namespace.c_str());
    }
    // endregion

    bool run_ros_subscriber = true;
    bool run_i2c_code = true;

    //std::unique_ptr<AdaFruitServoHat> adaFruitServoHat(new AdaFruitServoHat());
    adaFruitServoHat.reset(new AdaFruitServoHat());

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

        ROS_INFO("%s: Connecting to I2C Bus number %i", node_name.c_str(), i2c_bus_number);

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

    ros::Subscriber command_twist_ros_subscriber;

    //if(run_ros_subscriber) {

    std::string cmd_vel_topic = "/" + robot_namespace + "/ackermann_steering_controller/" + "cmd_vel";
    command_twist_ros_subscriber = ros_node_handle.subscribe(cmd_vel_topic, 1, handle_twist_command_callback);

    std::cout << "sub count " << command_twist_ros_subscriber.getTopic() << std::endl;
    //}

    std::cout << "Adafruit Servo Hat node running..." << std::endl;

    // TODO DEBUG sign of life
    /*
    float op_pwm_on_percent = 0.0;
    float pwm_delta = 0.1;
    float pwm_gain = 1.0;
    */

    while(ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();

        /*
        if(op_pwm_on_percent > 1.0) {
            op_pwm_on_percent = 1.0;
            pwm_gain = -1.0;
        }
        else if (op_pwm_on_percent < 0.0) {
            op_pwm_on_percent = 0.0;
            pwm_gain = 1.0;
        }

        op_pwm_on_percent += pwm_delta * pwm_gain;

        std::cout << "pwm set to " << op_pwm_on_percent << std::endl;

        adaFruitServoHat->command_pwm(Pca9685LEDController::LED0, op_pwm_on_percent);
        adaFruitServoHat->command_pwm(Pca9685LEDController::LED1, op_pwm_on_percent);
        */

        bool shutdown = ros::isShuttingDown();

        if(shutdown) {
            std::cout << node_name + ": Shutting down ROS node" << std::endl;
            break;
        }
    }

    if(run_ros_subscriber) {

    }

    return 0;
}
