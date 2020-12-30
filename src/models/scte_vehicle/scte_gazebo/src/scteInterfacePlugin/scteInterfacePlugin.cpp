//
// Created by user on 12/29/20.
//
/* Code modified from source located here: https://github.com/robustify/audibot/ */

#include <scteInterfacePlugin.h>

namespace gazebo {

    ScteBotInterfacePlugin::ScteBotInterfacePlugin() {
        targetAngle = 0.0;
        brakeCmd = 0.0;
        throttleCmd = 0.0;
        gearCmd = DRIVE;
        currentSteeringAngle = 0.0;
        vehicleRollover = false;
    }

    void ScteBotInterfacePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
        // Gazebo init
        steer_fl_joint = model->GetJoint("steer_fl");
        steer_fr_joint = model->GetJoint("steer_fr");

        wheel_fl_joint = model->GetJoint("wheel_fl");
        wheel_fr_joint = model->GetJoint("wheel_fr");
        wheel_rl_joint = model->GetJoint("wheel_rl");
        wheel_rr_joint = model->GetJoint("wheel_rr");

        footprint_link = model->GetJoint("base_footprint");

        // Load SDF parameters
        if (sdf->HasElement("pubTf")) {
            sdf->GetElement("pubTf")->GetValue()->Get(publishTF);
        }
        else {
            publishTF = false;
        }

    }

    void ScteBotInterfacePlugin::OnUpdate(const common::UpdateInfo& info) {

    }

    void ScteBotInterfacePlugin::twistStateUpdate() {

    }

    void ScteBotInterfacePlugin::driveUdpate() {

    }

    void ScteBotInterfacePlugin::steeringUpdate(const common::UpdateInfo &info) {

    }

    void ScteBotInterfacePlugin::dragUpdate() {

    }

    void ScteBotInterfacePlugin::setAllWheelTorque(double torque) {

    }

    void ScteBotInterfacePlugin::setRearWheelTorque(double torque) {

    }

    void ScteBotInterfacePlugin::stopWheels() {

    }

    void ScteBotInterfacePlugin::recvSteeringCmd(const std_msgs::Float64ConstPtr &msg) {

    }

    void ScteBotInterfacePlugin::recvThrottleCmd(const std_msgs::Float64ConstPtr &msg) {

    }

    void ScteBotInterfacePlugin::recvBreakCmd(const std_msgs::Float64ConstPtr &msg) {

    }

    void ScteBotInterfacePlugin::recvGearCmd(const std_msgs::UInt8ConstPtr &msg) {

    }

    void ScteBotInterfacePlugin::feedBackTimerCallback(const ros::TimerEvent &event) {

    }

    void ScteBotInterfacePlugin::tfTimerCallBack(const ros::TimerEvent &event) {

    }

    void ScteBotInterfacePlugin::Reset() {
        //does nothing...
    }

    ScteBotInterfacePlugin::~ScteBotInterfacePlugin() noexcept {
        rosNodeHandle->shutdown();
        delete rosNodeHandle;
    }
}