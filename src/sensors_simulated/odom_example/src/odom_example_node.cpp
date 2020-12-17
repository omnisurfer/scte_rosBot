//
// Created by user on 11/13/20.
//
//http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <ros/console.h>

#include <iostream>

int main(int argc, char** argv) {

    ros::init(argc, argv, "demo_odometry_publisher");

    ros::NodeHandle nodeHandle;
    ros::Publisher odomPublisher = nodeHandle.advertise<nav_msgs::Odometry>("demo_odom/odom", 50);
    tf::TransformBroadcaster odomBroadcaster;

    bool publish_odom_tf = false;

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx = 0.1;
    double vy = -0.1;
    double vth = 0.1;

    std::string childFrameId = "base_link";

    ros::Time currentTime, lastTime;
    currentTime = ros::Time::now();
    lastTime = ros::Time::now();

    ros::Rate rosRate(100.0);

    ROS_INFO("Starting Demo Odometry Publisher");

    while(nodeHandle.ok()) {
        ros::spinOnce();

        currentTime = ros::Time::now();

        //compute odometry
        double dt = (currentTime - lastTime).toSec();
        double deltaX = (vx * cos(th) - vy * sin(th)) * dt;
        double deltaY = (vx * sin(th) + vy * cos(th)) * dt;
        double deltaTh = vth * dt;

        x += deltaX;
        y += deltaY;
        th += deltaTh;

        geometry_msgs::Quaternion odomQuaternion = tf::createQuaternionMsgFromYaw(th);

        geometry_msgs::TransformStamped odomTransformed;
        odomTransformed.header.stamp = currentTime;
        odomTransformed.header.frame_id = "odom";
        odomTransformed.child_frame_id = childFrameId;

        odomTransformed.transform.translation.x = x;
        odomTransformed.transform.translation.y = y;
        odomTransformed.transform.translation.z = 0.0;
        odomTransformed.transform.rotation = odomQuaternion;

        //send the transform
        if(publish_odom_tf) {
            odomBroadcaster.sendTransform(odomTransformed);
        }
        else {
            //do nothing
        }

        //publish odometry message
        nav_msgs::Odometry odomMsg;
        odomMsg.header.stamp = currentTime;
        odomMsg.header.frame_id = "odom";

        //set position
        odomMsg.pose.pose.position.x = x;
        odomMsg.pose.pose.position.y = y;
        odomMsg.pose.pose.position.z = 0.0;
        odomMsg.pose.pose.orientation = odomQuaternion;

        //set velocity
        odomMsg.child_frame_id = childFrameId;
        odomMsg.twist.twist.linear.x = vx;
        odomMsg.twist.twist.linear.y = vy;
        odomMsg.twist.twist.linear.z = vth;

        //publish the message
        odomPublisher.publish(odomMsg);

        lastTime = currentTime;
        rosRate.sleep();
    }

    ROS_INFO("Stopping Demo Odometry Publisher");

    std::cout << "EXIT received!";
}