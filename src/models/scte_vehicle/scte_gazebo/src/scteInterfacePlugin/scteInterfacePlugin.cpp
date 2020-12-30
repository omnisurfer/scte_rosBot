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
        footprint_link = model->GetLink("base_footprint");

        steer_fl_joint = model->GetJoint("front_left_wheel_steer_TO_front_axle");
        steer_fr_joint = model->GetJoint("front_right_wheel_steer_TO_front_axle");

        wheel_fl_joint = model->GetJoint("front_left_wheel_TO_front_axle");
        wheel_fr_joint = model->GetJoint("front_right_wheel_TO_front_axle");
        wheel_rl_joint = model->GetJoint("rear_left_wheel_TO_rear_axle");
        wheel_rr_joint = model->GetJoint("rear_right_wheel_TO_rear_axle");

        std::cout << "base_footprint_joint: " << footprint_link << "\r\n";

        std::cout << "steer_fl_joint: " << steer_fl_joint << "\r\n";
        std::cout << "steer_fr_joint: " << steer_fr_joint << "\r\n";

        std::cout << "wheel_fl_joint: " << wheel_fl_joint << "\r\n";
        std::cout << "wheel_fr_joint: " << wheel_fr_joint << "\r\n";
        std::cout << "wheel_rl_joint: " << wheel_rl_joint << "\r\n";
        std::cout << "wheel_rr_joint: " << wheel_rr_joint << "\r\n";

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
#if GAZEBO_MAJOR_VERSION >= 9
        gazeboWorldPose = footprint_link->WorldPose();
        rosTwistMessage.linear.x = footprint_link->RelativeLinearVel().X();
        rosTwistMessage.angular.z = footprint_link->RelativeAngularAccel().Z();
        vehicleRollover = (fabs(gazeboWorldPose.Rot().X()) > 0.2 || fabs(gazeboWorldPose.Rot().Y() > 0.2));
#else
        gazeboWorldPose = footprint_link->GetWorldPose();
        rosTwistMessage.linear.x = footprint_link->GetRelativeLinearVel().x;
        rosTwistMessage.angular.z = footprint_link->GetRelativeAngularVel().z;
        vehicleRollover =(fabs(gazeboWorldPose.rot.x) > 0.2 || fabs(gazeboWorldPose.rot.y) > 0.2);
#endif
    }

    void ScteBotInterfacePlugin::driveUdpate() {
        if(vehicleRollover) {
            stopWheels();
            return;
        }

        // brakes have precedence over throttle
        ros::Time currentTimeStamp = ros::Time::now();

        if((brakeCmd > 0) && ((currentTimeStamp - rosBrakeTimeStamp).toSec() < 0.25)) {
            double brakeTorqueFactor = 1.0;
            if(rosTwistMessage.linear.x < -0.1) {
                brakeTorqueFactor = -1.0;
            } else if (rosTwistMessage.linear.x < 0.1){
                brakeTorqueFactor = 1.0 + (rosTwistMessage.linear.x - 0.1) / 0.1;
            }

            setAllWheelTorque(-brakeTorqueFactor * brakeCmd);
        } else {
            if((currentTimeStamp - rosThrottleTimeStamp).toSec() < 0.25) {
                double throttleTorque;
                if(gearCmd == DRIVE) {
                    throttleTorque = throttleCmd * 4000.0 - 40.1 * rosTwistMessage.linear.x;
                    if(throttleTorque < 0.0) {
                        throttleTorque = 0.0;
                    }
                } else { // REVERSE
                    throttleTorque = -throttleCmd * 4000.0 - 250.0 * rosTwistMessage.linear.x;
                    if(throttleTorque > 0.0) {
                        throttleTorque = 0.0;
                    }
                }

                setRearWheelTorque(throttleTorque);
            }
        }
    }

    void ScteBotInterfacePlugin::steeringUpdate(const common::UpdateInfo &info) {
        double timeStep = (info.simTime - lastUpdateTime).Double();
        lastUpdateTime = info.simTime;

        // arbitrarily set maximum steering rate 800 deg/s
        const double maxRate = 800.0 * M_PI / 180.0 / SCTEBOT_STEERING_RATIO;
        double maxIncrement = timeStep * maxRate;

        if((targetAngle - currentSteeringAngle) > maxIncrement) {
            currentSteeringAngle += maxIncrement;
        } else if((targetAngle - currentSteeringAngle) < -maxIncrement) {
            currentSteeringAngle -= maxIncrement;
        }

        // compute ackermann steering angles for each wheel
        double tAlpha = tan(currentSteeringAngle);
        double leftSteer = atan(SCTEBOT_WHEELBASE * tAlpha / (SCTEBOT_WHEELBASE - 0.5 * SCTEBOT_TRACK_WIDTH * tAlpha));
        double rightSteer = atan(SCTEBOT_WHEELBASE * tAlpha / (SCTEBOT_WHEELBASE + 0.5 * SCTEBOT_TRACK_WIDTH * tAlpha));

#if GAZEBO_MAJOR_VERSION >= 9
        steer_fl_joint->SetParam("vel", 0, 100 * (leftSteer - steer_fl_joint->Position(0)));
        steer_fr_joint->SetParam("vel", 0, 100 * (rightSteer - steer_fr_joint->Position(0)));
#else
        steer_fl_joint->SetParam("vel", 0, 100 * (leftSteer - steer_fl_joint->GetAngle(0).Radian()));
        steer_fr_joint->SetParam("vel", 0, 100 * (rightSteer - steer_fr_joint->GetAngle(0).Radian()));
#endif
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