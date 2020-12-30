//
// Created by user on 12/29/20.
//
/* Code modified from source located here: https://github.com/robustify/audibot/ */

/*TODO(drowan_20201229): Need to modify xacro to have the links this plugin is looking for. This is why I am
 * getting the assertion fail when trying to execute.
 *
 */

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
        int i = 0;

        std::cout << "DEBUG: " << i++ << "\r\n";

        // Gazebo init
        steer_fl_joint = model->GetJoint("steer_fl");
        steer_fr_joint = model->GetJoint("steer_fr");

        wheel_fl_joint = model->GetJoint("wheel_fl");
        wheel_fr_joint = model->GetJoint("wheel_fr");
        wheel_rl_joint = model->GetJoint("wheel_rl");
        wheel_rr_joint = model->GetJoint("wheel_rr");

        footprint_link = model->GetJoint("base_footprint");

        std::cout << "DEBUG: " << i++ << "\r\n";

        // Load SDF parameters
        if (sdf->HasElement("pubTf")) {
            sdf->GetElement("pubTf")->GetValue()->Get(publishTf);
        }
        else {
            publishTf = false;
        }

        if (sdf->HasElement("robotName")) {
            sdf::ParamPtr sdf_robot_name = sdf->GetElement("robotName")->GetValue();
            if(sdf_robot_name) {
                sdf_robot_name->Get(robotName);
            } else {
                robotName = std::string("");
            }
        } else {
            robotName = std::string("");
        }

        if(sdf->HasElement("pubTf")) {
            sdf->GetElement("pubTf")->GetValue()->Get(publishTf);
        } else {
            publishTf = false;
        }

        if(sdf->HasElement("tfFreq")) {
            sdf->GetElement("tffreq")->GetValue()->Get(tfPublishFreq);
        } else {
            tfPublishFreq = 100.0;
        }

        std::cout << "DEBUG: " << i++ << "\r\n";

        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ScteBotInterfacePlugin::OnUpdate, this, std::placeholders::_1));

        steer_fl_joint->SetParam("fmax", 0, 99999.0);
        steer_fr_joint->SetParam("fmax", 0, 99999.0);

        std::cout << "DEBUG: " << i++ << "\r\n";

        // ROS init
        rosNodeHandle = new ros::NodeHandle(robotName);

        rosSteeringCmdSubscriber = rosNodeHandle->subscribe("steering_cmd", 1, &ScteBotInterfacePlugin::recvSteeringCmd, this);
        rosThrottleCmdSubscriber = rosNodeHandle->subscribe("throttle_cmd", 1, &ScteBotInterfacePlugin::recvThrottleCmd, this);
        rosBreakCmdSubscriber = rosNodeHandle->subscribe("brake_cmd", 1, &ScteBotInterfacePlugin::recvBreakCmd, this);
        rosGearCmdSubscriber = rosNodeHandle->subscribe("gear_cmd", 1, &ScteBotInterfacePlugin::recvGearCmd, this);

        rosTwistPublisher = rosNodeHandle->advertise<geometry_msgs::TwistStamped>("twist", 1);
        rosGearStatePublisher = rosNodeHandle->advertise<std_msgs::UInt8>("gear_state", 1);
        rosFeedbackTimer = rosNodeHandle->createTimer(ros::Duration(0.02), &ScteBotInterfacePlugin::feedBackTimerCallback, this);

        if(publishTf) {
            rosTFTimer = rosNodeHandle->createTimer(ros::Duration(1.0 / tfPublishFreq), &ScteBotInterfacePlugin::tfTimerCallBack, this);
        }

        std::cout << "DEBUG END: " << i++ << "\r\n";

    }

    void ScteBotInterfacePlugin::OnUpdate(const common::UpdateInfo& info) {
        if(lastUpdateTime == common::Time(0)) {
            lastUpdateTime = info.simTime;
            return;
        }

        twistStateUpdate();
        driveUdpate();
        steeringUpdate(info);
        dragUpdate();
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
        std::cout << "Got steering cmd! \r\n";
    }

    void ScteBotInterfacePlugin::recvThrottleCmd(const std_msgs::Float64ConstPtr &msg) {
        std::cout << "Got throttle cmd! \r\n";
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