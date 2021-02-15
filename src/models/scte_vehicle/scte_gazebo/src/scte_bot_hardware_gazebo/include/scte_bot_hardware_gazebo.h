//
// Created by user on 2/14/21.
//

/* ref:
 * https://sir.upc.edu/projects/rostutorials/10-gazebo_control_tutorial/index.html
 * https://github.com/CIR-KIT/steer_drive_ros
 * http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5
 *
 * https://github.com/ros-controls/ros_control
 */

#ifndef SCTE_BOT_HARDWARE_GAZEBO_H
#define SCTE_BOT_HARDWARE_GAZEBO_H

#include <hardware_interface/robot_hw.h>
#include <gazebo_ros_control/robot_hw_sim.h>

#endif //SCTE_BOT_HARDWARE_GAZEBO_H

namespace scte_bot_hardware_gazebo {

class ScteBotHardwareGazebo : public gazebo_ros_control::RobotHWSim {

public:
    ScteBotHardwareGazebo();
    virtual ~ScteBotHardwareGazebo();

    bool initSim(const std::string& robotNamespace,
                 ros::Nodehandle rosModelNodeHandle,
                 gazebo::physics::ModelPtr gazeboParentModel,
                 const urdf::Model* const urdfModelPointer,
                 std::vector<transmission_interface::TransmissionInfo> transmissions
                 );

    void readSim(ros::Time time, ros::Duration period);

    void writeSim(ros::Time time, ros::Duration period);
};

}
