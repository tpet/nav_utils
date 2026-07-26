#ifndef NAV_UTILS_ODOM_TO_TF_H
#define NAV_UTILS_ODOM_TO_TF_H

#ifdef NAV_UTILS_ROS2
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#else
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#endif

namespace nav_utils
{

#ifdef NAV_UTILS_ROS2
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Transform;
using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;
#else
using Pose = geometry_msgs::Pose;
using Transform = geometry_msgs::Transform;
using TransformStamped = geometry_msgs::TransformStamped;
using Odometry = nav_msgs::Odometry;
#endif

Transform pose_to_transform(const Pose &pose);
TransformStamped odometry_to_transform(const Odometry &odom);

class OdometryToTransform
#ifdef NAV_UTILS_ROS2
  : public rclcpp::Node
#endif
{
private:
    tf2_ros::TransformBroadcaster tf_pub_;
    std::string parent_frame_;
    std::string child_frame_;
    bool invert_tf_{false};
#ifdef NAV_UTILS_ROS2
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
#else
  ros::Subscriber odom_sub_;
#endif
public:
#ifdef NAV_UTILS_ROS2
    explicit OdometryToTransform(const rclcpp::NodeOptions& options);
#else
    OdometryToTransform(ros::NodeHandle &nh, ros::NodeHandle &pnh);
#endif
    void odometryReceived(const Odometry &msg);
};

}

#endif //NAV_UTILS_ODOM_TO_TF_H
