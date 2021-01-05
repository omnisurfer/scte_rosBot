#ifndef SCTEBOTINTERFACEPLUGIN_H
#define SCTEBOTINTERFACEPLUGIN_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/TwistStamped.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace gazebo {

// Kinematics parameters
#define SCTEBOT_STEERING_RATIO      1.0     // ratio between steering wheel angle and tire angle
#define SCTEBOT_LOCK_TO_LOCK_REVS   1.0     // number of steering wheel turns to go from lock to lock
#define SCTEBOT_MAX_STEER_ANGLE     (M_PI * SCTEBOT_LOCK_TO_LOCK_REVS / SCTEBOT_STEERING_RATIO)
#define SCTEBOT_WHEELBASE           1.0     // distance between front and rear axles
#define SCTEBOT_TRACK_WIDTH         1.0     // distance between front wheels

// Drag parameters
#define ROLLING_RESISTANCE_COEFF    0.01
#define AERO_DRAG_COEFF             0.35
#define GRAVITY_ACCEL               9.81
#define VEHICLE_MASS                10.0
#define WHEEL_RADIUS                0.05
#define MAX_BRAKE_TORQUE            10.0

// Gear states
enum {
    DRIVE = 0,
    REVERSE = 1
};

class ScteBotInterfacePlugin : public ModelPlugin {
public:
    ScteBotInterfacePlugin();
    virtual ~ScteBotInterfacePlugin();

protected:
    virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
    virtual void Reset();

private:
    void feedbackTimerCallback(const ros::TimerEvent& event);
    void tfTimerCallBack(const ros::TimerEvent& event);
    void OnUpdate(const common::UpdateInfo& info);

    void recvSteeringCmd(const std_msgs::Float64ConstPtr& msg);
    void recvThrottleCmd(const std_msgs::Float64ConstPtr& msg);
    void recvBreakCmd(const std_msgs::Float64ConstPtr& msg);
    void recvGearCmd(const std_msgs::UInt8ConstPtr& msg);

    void twistStateUpdate();
    void driveUpdate();
    void steeringUpdate(const common::UpdateInfo& info);
    void dragUpdate();
    void stopWheels();
    void setAllWheelTorque(double torque);
    void setRearWheelTorque(double torque);

    void initLinksAndJoints(physics::ModelPtr model, sdf::ElementPtr sdf);
    void initSteering(void);
    void initPhysics(void);

    ros::NodeHandle* rosNodeHandle;
    ros::Publisher rosTwistPublisher;
    ros::Publisher rosGearStatePublisher;

    ros::Subscriber rosSteeringCmdSubscriber;
    ros::Subscriber rosThrottleCmdSubscriber;
    ros::Subscriber rosBreakCmdSubscriber;
    ros::Subscriber rosGearCmdSubscriber;
    ros::Subscriber rosModelStatesSubscriber;

    ros::Timer rosFeedbackTimer;
    ros::Timer rosTFTimer;

    tf::TransformBroadcaster rosTransformBroadcaster;
    geometry_msgs::Twist rosTwistMessage;

    bool vehicleRollover;

#if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Pose3d gazeboWorldPose;
#else
    gazebo::math::Pose gazeboWorldPose;
#endif

    event::ConnectionPtr updateConnection;
    physics::JointPtr steer_fl_joint;
    physics::JointPtr steer_fr_joint;

    physics::JointPtr wheel_fl_joint;
    physics::JointPtr wheel_fr_joint;
    physics::JointPtr wheel_rl_joint;
    physics::JointPtr wheel_rr_joint;

    physics::LinkPtr footprint_link;

    common::Time lastUpdateTime;

    // SDF parameters
    std::string robotName;
    bool publishTf;
    double tfPublishFreq;

    // Steering values
    double rightAngle;
    double leftAngle;
    double targetAngle;
    double currentSteeringAngle;

    // Brakes
    double brakeCmd;
    ros::Time rosBrakeTimeStamp;

    // Throttle
    double throttleCmd;
    ros::Time rosThrottleTimeStamp;

    // Gear
    uint8_t gearCmd;
};

GZ_REGISTER_MODEL_PLUGIN(ScteBotInterfacePlugin)

}

#endif SCTEBOTINTERFACEPLUGIN_H