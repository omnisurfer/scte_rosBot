//
// Created by user on 12/23/21.
//
#include <signal.h>

#include <ros/ros.h>

#include "adafruit_servo_hat_hw_interface.h"


double twist_linear_x_velocity, twist_angular_z_velocity;

// TODO create a thread and interface through guarded variables?
std::shared_ptr<AdafruitServoHatHardwareInterface> adafruit_servo_hat;

void signal_handler(int sig) {

    ROS_DEBUG_THROTTLE(3.0, "signal_handler: %i", sig);

    ros::shutdown();
}

void handle_servo_callback(int x, int y) {

    // TODO (20240728) - Currently no actual feedback from the servo driver so not printing anything
    //ROS_DEBUG_THROTTLE(3.0, "servo handle_servo_callback: x %i, y %i", x, y);
}

void handle_twist_command_callback(const geometry_msgs::Twist::ConstPtr& msg) {

    twist_linear_x_velocity = msg->linear.x;
    twist_angular_z_velocity = msg->angular.z;

    adafruit_servo_hat->request_linear_x_velocity(twist_linear_x_velocity);
    adafruit_servo_hat->request_angular_z_velocity(twist_angular_z_velocity);

    ROS_DEBUG_THROTTLE(3.0, "handle_twist_command_callback: linear_x_m/s [%f], angular_z_rad/s [%f]", msg->linear.x, msg->angular.z);
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
    
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    ROS_INFO("%s: initializing node", node_name.c_str());

    // region ROS Params
    std::string robot_namespace;
    std::string cmd_vel_topic;
    double update_rate = 100.0;

    double max_linear_speed_m_s = 2.4;
    double max_linear_speed_of_vehicle_as_geared_m_s = 12.5;
    double max_angular_rad_s = 1.5;

    double tire_radius_m = 0.05;

    double wheel_separation_h = 0.75;
    double wheel_separation_w = 0.28;

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

    if(ros_node_handle.getParam(node_name + "/max_linear_speed_of_vehicle_as_geared_m_s", max_linear_speed_of_vehicle_as_geared_m_s)) {
        ROS_INFO("%s: max_linear_speed_of_vehicle_as_geared_m_s %f", node_name.c_str(), max_linear_speed_of_vehicle_as_geared_m_s);
    } else {
        ROS_WARN("%s: max_linear_speed_of_vehicle_as_geared_m_s, using default %f", node_name.c_str(), max_linear_speed_of_vehicle_as_geared_m_s);
    }

    if(ros_node_handle.getParam(node_name + "/max_angular_speed_rad_s", max_angular_rad_s)) {
        ROS_INFO("%s: max_angular_rad_s %f", node_name.c_str(), max_angular_rad_s);
    } else {
        ROS_WARN("%s: max_angular_rad_s, using default %f", node_name.c_str(), max_angular_rad_s);
    }

    if(ros_node_handle.getParam(node_name + "/tire_radius_m", tire_radius_m)) {
        ROS_INFO("%s: tire_radius_m %f", node_name.c_str(), tire_radius_m);
    } else {
        ROS_WARN("%s: tire_radius_m, using default %f", node_name.c_str(), tire_radius_m);
    }

    if(ros_node_handle.getParam(node_name + "/wheel_separation_h", wheel_separation_h)) {
        ROS_INFO("%s: wheel_separation_h %f", node_name.c_str(), wheel_separation_h);
    } else {
        ROS_WARN("%s: wheel_separation_h, using default %f", node_name.c_str(), wheel_separation_h);
    }

    if(ros_node_handle.getParam(node_name + "/wheel_separation_w", wheel_separation_w)) {
        ROS_INFO("%s: wheel_separation_w %f", node_name.c_str(), wheel_separation_w);
    } else {
        ROS_WARN("%s: wheel_separation_w, using default %f", node_name.c_str(), wheel_separation_w);
    }
    // endregion

    /*
     * ackermann steering controller publishes odom. Not sure if this odoms is needed...
     */
    // ros::Publisher odometry_publisher;
    ros::Publisher steering_publisher;
    tf::TransformBroadcaster odometry_tf_broadcaster;

    ros::Subscriber command_twist_ros_subscriber;
    steering_publisher = ros_node_handle.advertise<geometry_msgs::Twist>("/steer_ctrl", 1);

    ros::Time current_ros_time, last_ros_time;

    bool run_ros_subscriber = true;
    bool run_i2c_code = true;

    adafruit_servo_hat.reset(new AdafruitServoHatHardwareInterface(robot_namespace, ros_node_handle));

    if(run_i2c_code) {

        ROS_INFO("%s: connecting to I2C Bus number %i", node_name.c_str(), i2c_bus_number);

        // TODO change this to an exception?
        bool init_ok = true;

        init_ok = adafruit_servo_hat->init_device(
                i2c_bus_number,
                max_linear_speed_m_s,
                max_linear_speed_of_vehicle_as_geared_m_s,
                max_angular_rad_s,
                tire_radius_m,
                wheel_separation_h,
                wheel_separation_w,
                handle_servo_callback
                );

        if(init_ok) {
            adafruit_servo_hat->run();
            ROS_DEBUG("%s: initialization success", node_name.c_str());
        }
        else {
            ROS_WARN("%s: initialization failed", node_name.c_str());
            return 0;
        }
    }

    if (run_ros_subscriber) {

        command_twist_ros_subscriber = ros_node_handle.subscribe(cmd_vel_topic, 1, handle_twist_command_callback);
    }
        
    controller_manager::ControllerManager cm(adafruit_servo_hat.get(), ros_node_handle);

    double controller_period = adafruit_servo_hat->getPeriod().toSec();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate loop_rate(1.0 / controller_period);
    
    ROS_INFO("Adafruit Servo Hat node running...");
    
    while(ros::ok()) {

        ros::Time now = adafruit_servo_hat->getTime();
        ros::Duration dt = adafruit_servo_hat->getPeriod();

        adafruit_servo_hat->read(now, dt);
        cm.update(now, dt);
        adafruit_servo_hat->write(now, dt);

        bool shutdown = ros::isShuttingDown();

        if(shutdown) {            
            ROS_INFO("%s: shutting down ROS node", node_name.c_str());
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

