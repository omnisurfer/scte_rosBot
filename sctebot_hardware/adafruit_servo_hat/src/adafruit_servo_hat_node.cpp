//
// Created by user on 12/23/21.
//
#include <signal.h>

#include <ros/ros.h>

#include "adafruit_servo_hat.h"

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <sensor_msgs/JointState.h>

#define JOINT_INDEX_REAR_RIGHT 0
#define JOINT_INDEX_REAR_LEFT 1
#define JOINT_INDEX_FRONT 2

enum VIRTUAL_JOINT_IND {
    VIRTUAL_JOINT_IND_RIGHT_REAR        = 0,
    VIRTUAL_JOINT_IND_LEFT_REAR         = 1,
    VIRTUAL_JOINT_IND_RIGHT_FRONT       = 2,
    VIRTUAL_JOINT_IND_LEFT_FRONT        = 3,
    VIRTUAL_JOINT_IND_RIGHT_FRONT_STEER = 4,
    VIRTUAL_JOINT_IND_LEFT_FRONT_STEER  = 5
};

class AdafruitServoHatHardwareInterface : public hardware_interface::RobotHW {

public:

    AdafruitServoHatHardwareInterface(const ros::NodeHandle& node_handle);
    ros::Time getTime() const { return ros::Time::now(); }
    ros::Duration getPeriod() const {return ros::Duration(0.01); }
    void read();
    void write();

    ~AdafruitServoHatHardwareInterface() {

    };

protected:
    void publishSteer(double angle_cmd);
    void registerVirtualJointState(
            std::vector<double> &virtual_wheels_pos,
            std::vector<double> &virtual_wheels_vel,
            std::vector<double> &virtual_wheels_eff,
            std::vector<std::string> &virtual_wheels_names
            );

    ros::NodeHandle node_handle;
    hardware_interface::PositionJointInterface front_steer_jnt_pos_cmd_interface;
    double front_steer_pos;
    double front_steer_vel;
    double front_steer_eff;
    double front_steer_pos_cmd;

    hardware_interface::VelocityJointInterface rear_wheel_jnt_vel_cmd_interface;
    double rear_wheel_pos;
    double rear_wheel_vel;
    double rear_wheel_eff;
    double rear_wheel_vel_cmd;

    hardware_interface::JointStateInterface joint_state_interface;
    std::vector<double> virtual_wheels_pos;
    std::vector<double> virtual_wheels_vel;
    std::vector<double> virtual_wheels_eff;

    ros::Publisher steer_cmd_publisher;
};

AdafruitServoHatHardwareInterface::AdafruitServoHatHardwareInterface(const ros::NodeHandle &node_handle):
    node_handle(node_handle),
    steer_cmd_publisher(this->node_handle.advertise<geometry_msgs::Twist>("/steer_ctrl", 1))
{
    ros::NodeHandle _node_handle("~");
    std::string front_steer_joint_name("front_steer_joint");
    std::string rear_wheel_joint_name("rear_wheel_joint");
    std::vector<std::string> virtual_wheels_names;
    virtual_wheels_names.emplace_back("base_to_right_rear_wheel");
    virtual_wheels_names.emplace_back("base_to_left_rear_wheel");
    virtual_wheels_names.emplace_back("base_to_right_front_wheel");
    virtual_wheels_names.emplace_back("base_to_left_front_wheel");
    virtual_wheels_names.emplace_back("base_to_right_front_steer");
    virtual_wheels_names.emplace_back("base_to_left_front_steer");

    virtual_wheels_vel.resize(6);
    virtual_wheels_pos.resize(6);
    virtual_wheels_eff.resize(6);

    // front steer
    hardware_interface::JointStateHandle front_steer_state_handle(
            front_steer_joint_name,
            &front_steer_pos,
            &front_steer_vel,
            &front_steer_eff
            );

    joint_state_interface.registerHandle(front_steer_state_handle);

    hardware_interface::JointHandle front_steer_pos_cmd_handle(
            joint_state_interface.getHandle(front_steer_joint_name),
            &front_steer_pos_cmd
            );

    front_steer_jnt_pos_cmd_interface.registerHandle(front_steer_pos_cmd_handle);

    // rear wheels
    hardware_interface::JointStateHandle rear_wheel_state_handle(
            rear_wheel_joint_name,
            &rear_wheel_pos,
            &rear_wheel_vel,
            &rear_wheel_eff
            );

    joint_state_interface.registerHandle(rear_wheel_state_handle);

    hardware_interface::JointHandle rear_wheel_vel_cmd_handle(
            joint_state_interface.getHandle(rear_wheel_joint_name),
            &rear_wheel_vel_cmd
            );

    rear_wheel_jnt_vel_cmd_interface.registerHandle(rear_wheel_vel_cmd_handle);

    this->registerVirtualJointState(
            virtual_wheels_pos,
            virtual_wheels_vel,
            virtual_wheels_eff,
            virtual_wheels_names
            );

    registerInterface(&front_steer_jnt_pos_cmd_interface);
    registerInterface(&rear_wheel_jnt_vel_cmd_interface);
    registerInterface(&joint_state_interface);
}

void AdafruitServoHatHardwareInterface::registerVirtualJointState(std::vector<double> &virtual_wheels_pos,
                                                                  std::vector<double> &virtual_wheels_vel,
                                                                  std::vector<double> &virtual_wheels_eff,
                                                                  std::vector<std::string> &virtual_wheels_names) {
    for(int i = 0; i < 6; ++i) {
        hardware_interface::JointStateHandle state_handle(
                virtual_wheels_names[i],
                &virtual_wheels_pos[i],
                &virtual_wheels_vel[i],
                &virtual_wheels_eff[i]
                );
        joint_state_interface.registerHandle(state_handle);
    }
}

void AdafruitServoHatHardwareInterface::publishSteer(double angle_cmd) {
    double limited_angle = angle_cmd * 180.0 / M_PI;
    limited_angle = std::max(limited_angle, -60.0);
    limited_angle = std::min(limited_angle, 60.0);
    geometry_msgs::Twist steer;

    double angle_diff = limited_angle - (front_steer_pos * 180.0 / M_PI);

    if(angle_diff > 0) {
        steer.angular.z = 1.0;
        steer.angular.x = fabs(angle_diff);
    } else if (angle_diff < 0) {
        steer.angular.z = -1.0;
        steer.angular.x = fabs(angle_diff);
    } else {
        steer.angular.z = 0;
        steer.angular.x = 0;
    }

    steer_cmd_publisher.publish(steer);
}

void AdafruitServoHatHardwareInterface::read() {

    sensor_msgs::JointState joints_state;

    front_steer_pos = joints_state.position[JOINT_INDEX_FRONT];

    rear_wheel_pos = (joints_state.position[JOINT_INDEX_REAR_RIGHT]
            + joints_state.position[JOINT_INDEX_REAR_RIGHT]) / 2.0;

    rear_wheel_vel = (joints_state.velocity[JOINT_INDEX_REAR_RIGHT]
            + joints_state.velocity[JOINT_INDEX_REAR_LEFT]) /  2.0;

    const double h = 0.75;
    const double w = 0.28;

    virtual_wheels_vel[VIRTUAL_JOINT_IND_RIGHT_REAR] = joints_state.velocity[JOINT_INDEX_REAR_RIGHT];
    virtual_wheels_pos[VIRTUAL_JOINT_IND_RIGHT_REAR] = joints_state.position[JOINT_INDEX_REAR_RIGHT];
    virtual_wheels_vel[VIRTUAL_JOINT_IND_LEFT_REAR] = joints_state.velocity[JOINT_INDEX_REAR_LEFT];
    virtual_wheels_pos[VIRTUAL_JOINT_IND_LEFT_REAR] = joints_state.velocity[JOINT_INDEX_REAR_LEFT];

    virtual_wheels_vel[VIRTUAL_JOINT_IND_RIGHT_FRONT] = virtual_wheels_vel[VIRTUAL_JOINT_IND_RIGHT_REAR];
    virtual_wheels_pos[VIRTUAL_JOINT_IND_RIGHT_FRONT] = virtual_wheels_pos[VIRTUAL_JOINT_IND_RIGHT_REAR];
    virtual_wheels_vel[VIRTUAL_JOINT_IND_LEFT_FRONT] = virtual_wheels_vel[VIRTUAL_JOINT_IND_LEFT_REAR];
    virtual_wheels_pos[VIRTUAL_JOINT_IND_LEFT_FRONT] = virtual_wheels_pos[VIRTUAL_JOINT_IND_LEFT_REAR];

    virtual_wheels_pos[VIRTUAL_JOINT_IND_RIGHT_FRONT_STEER] = atan2(2.0 * h * tan(front_steer_pos),
                                                                   2.0 * h + w / 2.0 * tan(front_steer_pos)
                                                                   );
    virtual_wheels_pos[VIRTUAL_JOINT_IND_LEFT_FRONT_STEER] = atan2(2.0 * h * tan(front_steer_pos),
                                                                   2.0 * h - w / 2.0 * tan(front_steer_pos)
                                                                   );
}

void AdafruitServoHatHardwareInterface::write() {
    this->publishSteer(front_steer_pos_cmd);
}

// TODO create a thread and interface through guarded variables?
std::unique_ptr<AdaFruitServoHat> adaFruitServoHat;
double max_linear_speed_m_s = 2.0;
double max_angular_rad_s = 1.5;

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

    double linear_x = msg->linear.x;
    double angular_z = msg->angular.z;
    double cmd_linear_pwm;
    double cmd_angular_pwm;

    cmd_linear_pwm = (linear_x / max_linear_speed_m_s) * 0.5 + 0.5;

    cmd_angular_pwm = (angular_z / max_angular_rad_s) * 0.5 + 0.5;

    adaFruitServoHat->command_pwm(Pca9685LEDController::LED0, float(cmd_linear_pwm));
    adaFruitServoHat->command_pwm(Pca9685LEDController::LED1, float(cmd_angular_pwm));

    ROS_DEBUG("Twist msg l_x [%f] m/s, a_z [%f] rad/s - Cmd l_x [%f], a_z [%f]", msg->linear.x, msg->angular.z, cmd_linear_pwm, cmd_angular_pwm);
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
    double update_rate = 20.0;

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
        ROS_INFO("%s: cmd_vel_topic %s", node_name.c_str(), (robot_namespace + cmd_vel_topic).c_str());
    } else {
        cmd_vel_topic = "/ackermann_steering_controller/cmd_vel";
        ROS_WARN("%s: cmd_vel_topic not found, using default %s", node_name.c_str(), (robot_namespace + cmd_vel_topic).c_str());
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

    cmd_vel_topic = robot_namespace + cmd_vel_topic;
    // endregion

    ros::Publisher odometry_publisher;
    ros::Publisher steering_publisher;
    tf::TransformBroadcaster odometry_tf_broadcaster;

    ros::Subscriber command_twist_ros_subscriber;

    odometry_publisher = ros_node_handle.advertise<nav_msgs::Odometry>("/odom", 1);
    steering_publisher = ros_node_handle.advertise<geometry_msgs::Twist>("/steer_ctrl", 1);

    ros::Time current_ros_time, last_ros_time;

    ros::Rate loop_rate(update_rate);

    bool run_ros_subscriber = true;
    bool run_i2c_code = true;

    adaFruitServoHat.reset(new AdaFruitServoHat());

    if(run_i2c_code) {

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

    if (run_ros_subscriber) {

        command_twist_ros_subscriber = ros_node_handle.subscribe(cmd_vel_topic, 1, handle_twist_command_callback);
    }

#if 1
    AdafruitServoHatHardwareInterface adafruit_servo_hat_hw_interface(ros_node_handle);
    controller_manager::ControllerManager cm(&adafruit_servo_hat_hw_interface);

#endif

    std::cout << "Adafruit Servo Hat node running..." << std::endl;

    while(ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();

#if 0
        // http://wiki.ros.org/ros_control/Tutorials/Create%20your%20own%20hardware%20interface
        // Debug Odom/TF code
        current_ros_time = ros::Time::now();

        geometry_msgs::Quaternion odom_quaternion {
            tf::createQuaternionMsgFromYaw(0.0)
        };
        geometry_msgs::TransformStamped odom_transform;
        odom_transform.header.stamp = current_ros_time;
        odom_transform.header.frame_id = "odom";
        odom_transform.child_frame_id = "base_link";

        odom_transform.transform.translation.x = 0.0;
        odom_transform.transform.translation.y = 0.0;
        odom_transform.transform.translation.z = 0.0;
        odom_transform.transform.rotation = odom_quaternion;

        odometry_tf_broadcaster.sendTransform(odom_transform);

        nav_msgs::Odometry odom_message;
        odom_message.header.stamp = current_ros_time;
        odom_message.header.frame_id = "odom";

        odom_message.pose.pose.position.x = 0.0;
        odom_message.pose.pose.position.y = 0.0;
        odom_message.pose.pose.position.z = 0.0;
        odom_message.pose.pose.orientation = odom_quaternion;

        odom_message.child_frame_id = "base_link";
        odom_message.twist.twist.linear.x = 0.0;
        odom_message.twist.twist.linear.y = 0.0;
        odom_message.twist.twist.linear.z = 0.0;

        odometry_publisher.publish(odom_message);
#endif
        ros::Time now = adafruit_servo_hat_hw_interface.getTime();
        ros::Duration dt = adafruit_servo_hat_hw_interface.getPeriod();

        adafruit_servo_hat_hw_interface.read();
        cm.update(now, dt);
        adafruit_servo_hat_hw_interface.write();

        bool shutdown = ros::isShuttingDown();

        if(shutdown) {
            std::cout << node_name + ": Shutting down ROS node" << std::endl;
            break;
        }
    }

    if(run_ros_subscriber) {
        // TODO close stuff
    }

    return 0;
}

