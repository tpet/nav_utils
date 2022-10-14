#pragma once

#include <nav_utils/header_subscriber.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace nav_utils
{

class OrientFrame {
protected:
    std::string parent_frame_;
    std::string child_frame_;
    std::string oriented_frame_;
    std::string no_wait_frame_;
    std::string align_ = "xyz";
    float timeout_ = 0.0;
    bool timeout_relative_ = false;
    int trigger_queue_size_ = 1;
    int tf_queue_size_ = 1;
    bool latch_ = false;
    ros::Publisher tf_pub_;
    tf2_ros::Buffer tf_;
    std::unique_ptr<tf2_ros::TransformListener> tf_sub_;
    nav_utils::HeaderSubscriber trigger_sub_;
public:
    OrientFrame(ros::NodeHandle & nh, ros::NodeHandle & pnh);
    void publishFor(const std_msgs::Header & msg) const;
};

}
