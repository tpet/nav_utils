#ifndef ODOM_PROC_TF_TO_ODOM_H
#define ODOM_PROC_TF_TO_ODOM_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros_type_introspection/ros_introspection.hpp>
#include <topic_tools/shape_shifter.h>
#include <tf2_client/tf2_client.h>

namespace odom_proc
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
    RosIntrospection::Parser parser;
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

#endif //ODOM_PROC_TF_TO_ODOM_H
