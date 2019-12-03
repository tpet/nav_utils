#ifndef NAV_UTILS_TF_TO_ODOM_H
#define NAV_UTILS_TF_TO_ODOM_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <topic_tools/shape_shifter.h>
#include <tf2_client/tf2_client.h>

namespace nav_utils
{
geometry_msgs::Pose transform_to_pose(const geometry_msgs::Transform &tf);
nav_msgs::Odometry transform_to_odometry(const geometry_msgs::TransformStamped &odom);

class TransformToOdometry {
private:
    std::string parent_frame_;
    std::string child_frame_;
    std::string no_wait_frame_;
    double timeout_;
    double timer_freq_;
    ros::Timer timer_;
    ros::Publisher odom_pub_;
    tf2_client::BufferPtr tf_;
    ros::Subscriber trigger_sub_;
    std::string resolveChildFrame(const std::string &child_frame);
public:
    TransformToOdometry(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    std_msgs::Header extract_header(const topic_tools::ShapeShifter &msg);
    geometry_msgs::Pose lookupPose(const ros::Time &stamp, const std::string &child_frame);
    void publishMessages(const ros::Time &stamp, const std::string &child_frame);
    void triggerReceived(const topic_tools::ShapeShifter &msg);
    void timerCallback(const ros::TimerEvent& event);
};
}

#endif //NAV_UTILS_TF_TO_ODOM_H
