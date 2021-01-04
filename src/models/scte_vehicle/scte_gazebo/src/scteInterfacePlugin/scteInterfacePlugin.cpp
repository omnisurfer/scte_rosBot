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

        // Gazebo init
        footprint_link = model->GetLink("base_footprint");

        steer_fl_joint = model->GetJoint("front_left_wheel_steer_TO_front_axle");
        steer_fr_joint = model->GetJoint("front_right_wheel_steer_TO_front_axle");

        wheel_fl_joint = model->GetJoint("front_left_wheel_TO_front_axle");
        wheel_fr_joint = model->GetJoint("front_right_wheel_TO_front_axle");
        wheel_rl_joint = model->GetJoint("rear_left_wheel_TO_rear_axle");
        wheel_rr_joint = model->GetJoint("rear_right_wheel_TO_rear_axle");

        // Load SDF parameters
        if (sdf->HasElement("robotName")) {
            sdf::ParamPtr sdf_robot_name = sdf->GetElement("robotName")->GetValue();
            if(sdf_robot_name) {
                sdf_robot_name->Get(robotName);

                std::cout << "robotNameA: " << robotName << "\r\n";
            } else {
                robotName = std::string("");

                std::cout << "robotNameB: " << robotName << "\r\n";
            }
        } else {
            robotName = std::string("");

            std::cout << "robotNameC: " << robotName << "\r\n";
        }

        if(sdf->HasElement("pubTf")) {
            sdf->GetElement("pubTf")->GetValue()->Get(publishTf);
        } else {
            publishTf = false;
        }

        if(sdf->HasElement("tfFreq")) {
            sdf->GetElement("tfFreq")->GetValue()->Get(tfPublishFreq);
        } else {
            tfPublishFreq = 100.0;
        }

        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ScteBotInterfacePlugin::OnUpdate, this, std::placeholders::_1));

        steer_fl_joint->SetParam("fmax", 0, 99999.0);
        steer_fr_joint->SetParam("fmax", 0, 99999.0);

        // ROS init
        rosNodeHandle = new ros::NodeHandle(robotName);

        rosSteeringCmdSubscriber = rosNodeHandle->subscribe("steering_cmd", 1, &ScteBotInterfacePlugin::recvSteeringCmd, this);
        rosThrottleCmdSubscriber = rosNodeHandle->subscribe("throttle_cmd", 1, &ScteBotInterfacePlugin::recvThrottleCmd, this);
        rosBreakCmdSubscriber = rosNodeHandle->subscribe("brake_cmd", 1, &ScteBotInterfacePlugin::recvBreakCmd, this);
        rosGearCmdSubscriber = rosNodeHandle->subscribe("gear_cmd", 1, &ScteBotInterfacePlugin::recvGearCmd, this);

        rosTwistPublisher = rosNodeHandle->advertise<geometry_msgs::TwistStamped>("twist", 1);
        rosGearStatePublisher = rosNodeHandle->advertise<std_msgs::UInt8>("gear_state", 1);
        rosFeedbackTimer = rosNodeHandle->createTimer(ros::Duration(0.02),
                                                      &ScteBotInterfacePlugin::feedbackTimerCallback, this);

        if(publishTf) {
            rosTFTimer = rosNodeHandle->createTimer(ros::Duration(1.0 / tfPublishFreq), &ScteBotInterfacePlugin::tfTimerCallBack, this);
        }

    }

    void ScteBotInterfacePlugin::OnUpdate(const common::UpdateInfo& info) {
        if(lastUpdateTime == common::Time(0)) {
            lastUpdateTime = info.simTime;
            return;
        }

        twistStateUpdate();
        driveUpdate();
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

    void ScteBotInterfacePlugin::driveUpdate() {
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
            if((currentTimeStamp - rosThrottleTimeStamp).toSec() < 0.25) { //original was 0.25
                double throttleTorque;
                double magicNumber = 4000.0; //was 4000 in example
                if(gearCmd == DRIVE) {
                    throttleTorque = throttleCmd * magicNumber - 40.1 * rosTwistMessage.linear.x;
                    if(throttleTorque < 0.0) {
                        throttleTorque = 0.0;
                    }
                } else { // REVERSE
                    throttleTorque = -throttleCmd * magicNumber - 250.0 * rosTwistMessage.linear.x;
                    if(throttleTorque > 0.0) {
                        throttleTorque = 0.0;
                    }
                }

                //setRearWheelTorque(throttleTorque);
                std::cout << "applying total torque of: " << throttleTorque << "\r\n";
                setAllWheelTorque(throttleTorque);
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
        // apply rolling resistance and aerodynamic drag forces
        double rollingResistanceTorque = ROLLING_RESISTANCE_COEFF * VEHICLE_MASS * GRAVITY_ACCEL;
        double dragForce = AERO_DRAG_COEFF * rosTwistMessage.linear.x * rosTwistMessage.linear.x;
        double dragTorque = dragForce * WHEEL_RADIUS; // implemented as a torque disturbance

        if(rosTwistMessage.linear.x > 0.0 ) {
            setAllWheelTorque(-rollingResistanceTorque);
            setAllWheelTorque(-dragTorque);
        } else {
            setAllWheelTorque(rollingResistanceTorque);
            setAllWheelTorque(dragTorque);
        }
    }

    void ScteBotInterfacePlugin::setAllWheelTorque(double torque) {
        wheel_rl_joint->SetForce(0, 0.25 * torque);
        wheel_rr_joint->SetForce(0, 0.25 * torque);

        wheel_fl_joint->SetForce(0, 0.25 * torque);
        wheel_fr_joint->SetForce(0, 0.25 * torque);
    }

    void ScteBotInterfacePlugin::setRearWheelTorque(double torque) {
        wheel_rl_joint->SetForce(0, 0.25 * torque);
        wheel_rr_joint->SetForce(0, 0.25 * torque);
    }

    void ScteBotInterfacePlugin::stopWheels() {
        wheel_rl_joint->SetForce(0, -1000.0 * wheel_rl_joint->GetVelocity(0));
        wheel_rr_joint->SetForce(0, -1000.0 * wheel_rr_joint->GetVelocity(0));

        wheel_fl_joint->SetForce(0, -1000.0 * wheel_fl_joint->GetVelocity(0));
        wheel_fr_joint->SetForce(0, -1000.0 * wheel_fr_joint->GetVelocity(0));
    }

    void ScteBotInterfacePlugin::recvSteeringCmd(const std_msgs::Float64ConstPtr &msg) {
        if(!std::isfinite(msg->data)) {
            targetAngle = 0.0;
            return;
        }

        targetAngle = msg->data / SCTEBOT_STEERING_RATIO;
        if(targetAngle > SCTEBOT_MAX_STEER_ANGLE) {
            targetAngle = SCTEBOT_MAX_STEER_ANGLE;
        } else if (targetAngle < -SCTEBOT_MAX_STEER_ANGLE) {
            targetAngle = -SCTEBOT_MAX_STEER_ANGLE;
        }
    }

    void ScteBotInterfacePlugin::recvThrottleCmd(const std_msgs::Float64ConstPtr &msg) {
        throttleCmd = msg->data;

        if(throttleCmd < 0.0) {
            throttleCmd = 0.0;
        } else if (throttleCmd > 1.0) {
            throttleCmd = 1.0;
        }

        rosThrottleTimeStamp = ros::Time::now();
    }

    void ScteBotInterfacePlugin::recvBreakCmd(const std_msgs::Float64ConstPtr &msg) {
        brakeCmd = msg->data;

        if(brakeCmd < 0) {
            brakeCmd = 0;
        } else if (brakeCmd > MAX_BRAKE_TORQUE) {
            brakeCmd = MAX_BRAKE_TORQUE;
        }
        rosBrakeTimeStamp = ros::Time::now();
    }

    void ScteBotInterfacePlugin::recvGearCmd(const std_msgs::UInt8ConstPtr &msg) {

        if(msg->data > REVERSE) {
            ROS_WARN("Invalid gear command received [%u]", gearCmd);
        } else {
            gearCmd = msg->data;
        }
    }

    void ScteBotInterfacePlugin::feedbackTimerCallback(const ros::TimerEvent &event) {
        geometry_msgs::TwistStamped twistStampedMessage;
        twistStampedMessage.header.frame_id = tf::resolve(robotName, footprint_link->GetName());
        twistStampedMessage.header.stamp = event.current_real;
        twistStampedMessage.twist = rosTwistMessage;

        rosTwistPublisher.publish(twistStampedMessage);

        std_msgs::UInt8 gearStateMessage;
        gearStateMessage.data = gearCmd;

        rosGearStatePublisher.publish(gearStateMessage);
    }

    void ScteBotInterfacePlugin::tfTimerCallBack(const ros::TimerEvent &event) {

        // Don't publish redundant TF within the delta
        if((event.current_real - event.last_real).toSec() < 1e-6) {
            return;
        }

        tf::StampedTransform stampedTransform;

        stampedTransform.frame_id_ = "world";
        //drowan_NOTE_20210102: the prefix robotName allows for multiple vehicles to be simulated BUT
        //this requires a separate tf_prefixer script to be used since the functionality to allow
        //multiple tf frames was silently removed in recent ROS updates:
        //https://github.com/robustify/audibot/blob/noetic-devel/audibot_gazebo/scripts/tf_prefixer.py
        stampedTransform.child_frame_id_ = tf::resolve(robotName, footprint_link->GetName());
        stampedTransform.stamp_ = event.current_real;

#if GAZEBO_MAJOR_VERSION >= 9
        stampedTransform.setOrigin(tf::Vector3(
                gazeboWorldPose.Pos().X(),
                gazeboWorldPose.Pos().Y(),
                gazeboWorldPose.Pos().Z())
                );
        stampedTransform.setRotation(tf::Quaternion(
                gazeboWorldPose.Rot().X(),
                gazeboWorldPose.Rot().Y(),
                gazeboWorldPose.Rot().Z(),
                gazeboWorldPose.Rot().W())
                );
#else
        stampedTransform.setOrigin(tf::Vector3(gazeboWorldPose.pos.x, gazeboWorldPose.pos.y, gazeboWorldPose.z));
        stampedTransform.setRotation(tf::Quaternion(gazeboWorldPose.rot.x, gazeboWorldPose.rot.y, gazeboWorldPose.rot.z, gazeboWorldPose.rot.w));
#endif
        rosTransformBroadcaster.sendTransform(stampedTransform);
    }

    void ScteBotInterfacePlugin::Reset() {
        //does nothing...
        ROS_INFO("Reset currently not implemented...");
    }

    ScteBotInterfacePlugin::~ScteBotInterfacePlugin() noexcept {
        rosNodeHandle->shutdown();
        delete rosNodeHandle;
    }
}