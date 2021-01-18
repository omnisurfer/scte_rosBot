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

        initLinksAndJoints(model, sdf);

        initSteeringKinematics(model, sdf);

        initPhysics(model, sdf);

        // Load SDF parameters
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

        std::stringstream outputStream;
        outputStream << "Robot namespace is " << robotName;
        ROS_INFO("%s", outputStream.str().c_str());

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
                double driveMagicNumber = 40.1;
                double reverseMagicNumber = 255.0;
                if(gearCmd == DRIVE) {
                    throttleTorque = throttleCmd * magicNumber - driveMagicNumber * rosTwistMessage.linear.x;
                    if(throttleTorque < 0.0) {
                        throttleTorque = 0.0;
                    }
                } else { // REVERSE
                    throttleTorque = -throttleCmd * magicNumber - reverseMagicNumber * rosTwistMessage.linear.x;
                    if(throttleTorque > 0.0) {
                        throttleTorque = 0.0;
                    }
                }

                //setRearWheelTorque(throttleTorque);
                // std::cout << "applying total torque of: " << throttleTorque << "\r\n";
                setAllWheelTorque(throttleTorque);
            }
        }
    }

    void ScteBotInterfacePlugin::steeringUpdate(const common::UpdateInfo &info) {
        double timeStep = (info.simTime - lastUpdateTime).Double();
        lastUpdateTime = info.simTime;

        // arbitrarily set maximum steering rate 800 deg/s
        const double maxRate = 800.0 * M_PI / 180.0 / steeringRatio;
        double maxIncrement = timeStep * maxRate;

        if((targetAngle - currentSteeringAngle) > maxIncrement) {
            currentSteeringAngle += maxIncrement;
        } else if((targetAngle - currentSteeringAngle) < -maxIncrement) {
            currentSteeringAngle -= maxIncrement;
        }

        std::stringstream outputStream;
        outputStream << "targetAngle: " << targetAngle << " currentSteeringAngle: " << currentSteeringAngle << " maxRate: " << maxRate << " maxIncrement: " << maxIncrement;
        //ROS_INFO("%s", outputStream.str().c_str());

        // compute ackermann steering angles for each wheel
        double tAlpha = tan(currentSteeringAngle);
        double leftSteer = atan(wheelbase * tAlpha / (wheelbase - 0.5 * trackWidth * tAlpha));
        double rightSteer = atan(wheelbase * tAlpha / (wheelbase + 0.5 * trackWidth * tAlpha));

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
        double rollingResistanceTorque = rollingResistanceCoefficient * vehicleMass * gravityAcceleration;
        double dragForce = aeroDragCoefficient * rosTwistMessage.linear.x * rosTwistMessage.linear.x;
        double dragTorque = dragForce * wheelRadius; // implemented as a torque disturbance

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

        std::stringstream outputStream;

        targetAngle = msg->data / steeringRatio;

        /*
        TODO drowan_20210117_NOTES: limiting max steering angle to 90% of the calculated max until I can figure
         out why the model crashes if I allow the full travel. Has something to do with the inertia probably as
         adjusting the mass of the wheel seems to change the behavior (heavier = better behaved).
         gzclient: /build/ogre-1.9-kiU5_5/ogre-1.9-1.9.0+dfsg1/OgreMain/include/OgreAxisAlignedBox.h:251: void Ogre::AxisAlignedBox::setExtents(const Ogre::Vector3&, const Ogre::Vector3&): Assertion `(min.x <= max.x && min.y <= max.y && min.z <= max.z) && "The minimum corner of the box must be less than or equal to maximum corner"' failed.
        */
        if(targetAngle > maxSteeringAngle * 0.9) {
            targetAngle = maxSteeringAngle * 0.9;

            outputStream << "pos greater than targetAngle: " << targetAngle << " maxSteeringAngle: " << maxSteeringAngle;
            ROS_INFO("%s", outputStream.str().c_str());

        } else if (targetAngle < -maxSteeringAngle * 0.9) {
            targetAngle = -(maxSteeringAngle * 0.9);

            outputStream << "neg greater than targetAngle: " << targetAngle << " maxSteeringAngle: " << maxSteeringAngle;
            ROS_INFO("%s", outputStream.str().c_str());
        } else {
            outputStream << "targetAngle: " << targetAngle << " maxSteeringAngle: " << maxSteeringAngle;
            ROS_INFO("%s", outputStream.str().c_str());
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
        } else if (brakeCmd > maxBrakeTorque) {
            brakeCmd = maxBrakeTorque;
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

    void ScteBotInterfacePlugin::initLinksAndJoints(physics::ModelPtr model, sdf::ElementPtr sdf) {
        ROS_INFO("Loading model links and joints...");

        if(sdf->HasElement("footprint_link")) {
            std::string footprintLink;

            sdf->GetElement("footprint_link")->GetValue()->Get(footprintLink);

            footprint_link = model->GetLink(footprintLink);

        } else {
            ROS_ERROR("Missing or incorrectly named link for footprint_link.");
            return;
        }

        if(sdf->HasElement("steer_fl_joint")) {
            std::string steerJoint;

            sdf->GetElement("steer_fl_joint")->GetValue()->Get(steerJoint);

            steer_fl_joint = model->GetJoint(steerJoint);

        } else {
            ROS_ERROR("Missing or incorrectly named link for steer_fl_joint.");
            return;
        }

        if(sdf->HasElement("steer_fr_joint")) {
            std::string steerJoint;

            sdf->GetElement("steer_fr_joint")->GetValue()->Get(steerJoint);

            steer_fr_joint = model->GetJoint(steerJoint);

        } else {
            ROS_ERROR("Missing or incorrectly named link for steer_fr_joint.");
            return;
        }

        if(sdf->HasElement("wheel_fl_joint")) {
            std::string wheelJoint;

            sdf->GetElement("wheel_fl_joint")->GetValue()->Get(wheelJoint);

            wheel_fl_joint = model->GetJoint(wheelJoint);

        } else {
            ROS_ERROR("Missing or incorrectly named link for wheel_fr_joint.");
            return;
        }

        if(sdf->HasElement("wheel_fr_joint")) {
            std::string wheelJoint;

            sdf->GetElement("wheel_fr_joint")->GetValue()->Get(wheelJoint);

            wheel_fr_joint = model->GetJoint(wheelJoint);

        } else {
            ROS_ERROR("Missing or incorrectly named link for wheel_fr_joint.");
            return;
        }

        if(sdf->HasElement("wheel_rl_joint")) {
            std::string wheelJoint;

            sdf->GetElement("wheel_rl_joint")->GetValue()->Get(wheelJoint);

            wheel_rl_joint = model->GetJoint(wheelJoint);

        } else {
            ROS_ERROR("Missing or incorrectly named link for wheel_rl_joint.");
            return;
        }

        if(sdf->HasElement("wheel_rr_joint")) {
            std::string wheelJoint;

            sdf->GetElement("wheel_rr_joint")->GetValue()->Get(wheelJoint);

            wheel_rr_joint = model->GetJoint(wheelJoint);

        } else {
            ROS_ERROR("Missing or incorrectly named link for wheel_rr_joint.");
            return;
        }
    }

    void ScteBotInterfacePlugin::initSteeringKinematics(physics::ModelPtr model, sdf::ElementPtr sdf) {
        ROS_INFO("Loading steering geometry...");

        if(sdf->HasElement("steering_ratio")) {
            double value;

            sdf->GetElement("steering_ratio")->GetValue()->Get(value);

            steeringRatio = value;

            if(steeringRatio != 0.0) {
                std::stringstream outputStream;

                outputStream << "\tsteering_ratio is " << steeringRatio;

                ROS_INFO("%s", outputStream.str().c_str());
            } else {
                ROS_ERROR("steering_ratio must not equal 0");
            }

        }
        else {
            ROS_ERROR("Missing steering_ratio parameters");
            return;
        }

        if(sdf->HasElement("lock_to_lock_revolutions")) {
            double value;

            sdf->GetElement("lock_to_lock_revolutions")->GetValue()->Get(value);

            lockToLockRevolutions = value;

            std::stringstream outputStream;

            outputStream << "\tlock_to_lock_revolutions is " << lockToLockRevolutions;

            ROS_INFO("%s", outputStream.str().c_str());

        }
        else {
            ROS_ERROR("Missing lock_to_lock_revolutions parameters");
            return;
        }

        if(sdf->HasElement("wheelbase")) {
            double value;

            sdf->GetElement("wheelbase")->GetValue()->Get(value);

            wheelbase = value;

            std::stringstream outputStream;

            outputStream << "\twheelbase is " << wheelbase;

            ROS_INFO("%s", outputStream.str().c_str());

        }
        else {
            ROS_ERROR("Missing wheelbase parameters");
            return;
        }

        if(sdf->HasElement("track_width")) {
            double value;

            sdf->GetElement("track_width")->GetValue()->Get(value);

            trackWidth = value;

            std::stringstream outputStream;

            outputStream << "\ttrack_width is " << trackWidth;

            ROS_INFO("%s", outputStream.str().c_str());

        }
        else {
            ROS_ERROR("Missing track_width parameters");
            return;
        }

        if(sdf->HasElement("wheel_radius")) {
            double value;

            sdf->GetElement("wheel_radius")->GetValue()->Get(value);

            wheelRadius = value;

            std::stringstream outputStream;

            outputStream << "\twheel_radius is " << wheelRadius;

            ROS_INFO("%s", outputStream.str().c_str());

        }
        else {
            ROS_ERROR("Missing wheel_radius parameters");
            return;
        }

        maxSteeringAngle =  M_PI * lockToLockRevolutions / (steeringRatio);
    }

    void ScteBotInterfacePlugin::initPhysics(physics::ModelPtr model, sdf::ElementPtr sdf) {
        ROS_INFO("Loading steering geometry...");

        if(sdf->HasElement("rolling_resistance_coeff")) {
            double value;

            sdf->GetElement("rolling_resistance_coeff")->GetValue()->Get(value);

            rollingResistanceCoefficient = value;

            std::stringstream outputStream;

            outputStream << "\trolling_resistance_coeff is " << rollingResistanceCoefficient;

            ROS_INFO("%s", outputStream.str().c_str());

        }
        else {
            ROS_ERROR("Missing rolling_resistance_coeff parameters");
            return;
        }

        if(sdf->HasElement("aero_drag_coeff")) {
            double value;

            sdf->GetElement("aero_drag_coeff")->GetValue()->Get(value);

            aeroDragCoefficient = value;

            std::stringstream outputStream;

            outputStream << "\taero_drag_coeff is " << aeroDragCoefficient;

            ROS_INFO("%s", outputStream.str().c_str());

        }
        else {
            ROS_ERROR("Missing aero_drag_coeff parameters");
            return;
        }

        if(sdf->HasElement("gravity_accel")) {
            double value;

            sdf->GetElement("gravity_accel")->GetValue()->Get(value);

            gravityAcceleration = value;

            std::stringstream outputStream;

            outputStream << "\tgravity_accel is " << gravityAcceleration;

            ROS_INFO("%s", outputStream.str().c_str());

        }
        else {
            ROS_ERROR("Missing gravity_accel parameters");
            return;
        }

        if(sdf->HasElement("vehicle_mass")) {
            double value;

            sdf->GetElement("vehicle_mass")->GetValue()->Get(value);

            vehicleMass = value;

            std::stringstream outputStream;

            outputStream << "\tvehicle_mass is " << vehicleMass;

            ROS_INFO("%s", outputStream.str().c_str());

        }
        else {
            ROS_ERROR("Missing vehicle_mass parameters");
            return;
        }

        if(sdf->HasElement("max_brake_torque")) {
            double value;

            sdf->GetElement("max_brake_torque")->GetValue()->Get(value);

            maxBrakeTorque = value;

            std::stringstream outputStream;

            outputStream << "\tmax_brake_torque is " << maxBrakeTorque;

            ROS_INFO("%s", outputStream.str().c_str());

        }
        else {
            ROS_ERROR("Missing max_brake_torque parameters");
            return;
        }

    }

    ScteBotInterfacePlugin::~ScteBotInterfacePlugin() noexcept {
        rosNodeHandle->shutdown();
        delete rosNodeHandle;
    }
}