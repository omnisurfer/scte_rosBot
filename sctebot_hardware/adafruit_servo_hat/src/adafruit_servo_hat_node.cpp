//
// Created by user on 12/23/21.
//
#include <signal.h>

#include <ros/ros.h>

#include "adafruit_servo_hat_hw_interface.h"

// TODO create a thread and interface through guarded variables?
std::shared_ptr<AdafruitServoHatHardwareInterface> adafruit_servo_hat;

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

    double linear_x_velocity = msg->linear.x;
    double angular_z_velocity = msg->angular.z;

    adafruit_servo_hat->command_liner_x_velocity(linear_x_velocity);
    adafruit_servo_hat->command_angular_z_velocity(angular_z_velocity);

    //ROS_INFO("Twist msg l_x [%f] m/s, a_z [%f] rad/s", msg->linear.x, msg->angular.z);
}

int main(int argc, char* argv[]) {

    signal(SIGINT | SIGTERM | SIGABRT | SIGKILL, signal_handler);

    std::string node_name = "adafruit_servo_hat_node";

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

    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);

    ros::NodeHandle ros_node_handle;

    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ROS_INFO("%s: initializing node", node_name.c_str());

    // region ROS Params
    std::string robot_namespace;
    std::string cmd_vel_topic;
    double update_rate = 100.0;

    double max_linear_speed_m_s = 2.0;
    double max_angular_rad_s = 1.5;

    node_name = ros::this_node::getName();

    if (ros_node_handle.getParam("robot_namespace", robot_namespace)) {
        ROS_INFO("%s: robot_namespace %s", node_name.c_str(), robot_namespace.c_str());
    } else {
        robot_namespace = "sctebot";
        ROS_WARN("%s: robot_namespace not found, using default %s", node_name.c_str(), robot_namespace.c_str());
    }

    if(ros_node_handle.getParam(node_name + "/update_rate", update_rate)) {
        ROS_INFO("%s: robot_namespace %f", node_name.c_str(), update_rate);
    } else {
        ROS_WARN("%s: update_rate not found, using default %f", node_name.c_str(), update_rate);
    }

    if(ros_node_handle.getParam(node_name + "/i2c_bus_number", i2c_bus_number)) {
        ROS_INFO("%s: i2c_bus_number %i", node_name.c_str(), i2c_bus_number);
    } else {
        ROS_WARN("%s: i2c_bus_number not found, using default %i", node_name.c_str(), i2c_bus_number);
    }

    if(ros_node_handle.getParam(node_name + "/cmd_vel_topic", cmd_vel_topic)) {
        ROS_INFO("%s: cmd_vel_topic %s", node_name.c_str(), (robot_namespace + '/' + cmd_vel_topic).c_str());
    } else {
        cmd_vel_topic = "ackermann_steering_controller/cmd_vel";
        ROS_WARN("%s: cmd_vel_topic not found, using default %s", node_name.c_str(), (robot_namespace + '/' + cmd_vel_topic).c_str());
    }

    if(ros_node_handle.getParam(node_name + "/max_linear_speed_m_s", max_linear_speed_m_s)) {
        ROS_INFO("%s: max_linear_speed_m_s %f", node_name.c_str(), max_linear_speed_m_s);
    } else {
        ROS_WARN("%s: max_linear_speed_m_s, using default %f", node_name.c_str(), max_linear_speed_m_s);
    }

    if(ros_node_handle.getParam(node_name + "/max_angular_speed_rad_s", max_angular_rad_s)) {
        ROS_INFO("%s: max_angular_rad_s %f", node_name.c_str(), max_angular_rad_s);
    } else {
        ROS_WARN("%s: max_angular_rad_s, using default %f", node_name.c_str(), max_angular_rad_s);
    }
    // endregion

    /*
     * ackermann steering controller publishes odom. Not sure if this odoms is needed...
     */
    // ros::Publisher odometry_publisher;
    ros::Publisher steering_publisher;
    tf::TransformBroadcaster odometry_tf_broadcaster;

    ros::Subscriber command_twist_ros_subscriber;

    // odometry_publisher = ros_node_handle.advertise<nav_msgs::Odometry>("/odom", 1);
    steering_publisher = ros_node_handle.advertise<geometry_msgs::Twist>("/steer_ctrl", 1);

    ros::Time current_ros_time, last_ros_time;

    bool run_ros_subscriber = true;
    bool run_i2c_code = true;

    adafruit_servo_hat.reset(new AdafruitServoHatHardwareInterface(robot_namespace, ros_node_handle));

    if(run_i2c_code) {

        ROS_INFO("%s: Connecting to I2C Bus number %i", node_name.c_str(), i2c_bus_number);

        // TODO change this to an exception?
        bool init_ok = true;

        init_ok = adafruit_servo_hat->init_device(
                i2c_bus_number,
                max_linear_speed_m_s,
                max_angular_rad_s,
                handle_servo_callback
                );

        if(init_ok) {
            adafruit_servo_hat->run();
            ROS_INFO("%s: SERVO HAT initialization success", node_name.c_str());
        }
        else {
            ROS_WARN("%s: SERVO HAT initialization failed", node_name.c_str());
            return 0;
        }
    }

    if (run_ros_subscriber) {

        command_twist_ros_subscriber = ros_node_handle.subscribe(cmd_vel_topic, 1, handle_twist_command_callback);
    }

    double controller_period = 1.0;

    //AdafruitServoHatHardwareInterface adafruit_servo_hat_hw_interface(robot_namespace, ros_node_handle);
    controller_manager::ControllerManager cm(adafruit_servo_hat.get(), ros_node_handle);

    controller_period = adafruit_servo_hat->getPeriod().toSec();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate loop_rate(1.0 / controller_period);

    std::cout << "Adafruit Servo Hat node running..." << std::endl;

    int debug_count = 0;
    while(ros::ok()) {

        ros::Time now = adafruit_servo_hat->getTime();
        ros::Duration dt = adafruit_servo_hat->getPeriod();

        adafruit_servo_hat->read(now, dt);
        cm.update(now, dt);
        adafruit_servo_hat->write(now, dt);

        bool shutdown = ros::isShuttingDown();

        if(shutdown) {
            std::cout << node_name + ": Shutting down ROS node" << std::endl;
            break;
        }

        loop_rate.sleep();
    }
    spinner.stop();

    if(run_ros_subscriber) {
        // TODO close stuff
    }

    return 0;
}

