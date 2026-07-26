#ifndef NAV_UTILS_ODOM_TWIST_TO_CHILD_FRAME_H
#define NAV_UTILS_ODOM_TWIST_TO_CHILD_FRAME_H

#ifdef NAV_UTILS_ROS2
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#else
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#endif

namespace nav_utils
{

#ifdef NAV_UTILS_ROS2
using Odometry = nav_msgs::msg::Odometry;
#else
using Odometry = nav_msgs::Odometry;
#endif

class OdometryTwistToChildFrame
#ifdef NAV_UTILS_ROS2
  : public rclcpp::Node
#endif
{
private:
#ifdef NAV_UTILS_ROS2
  rclcpp::Publisher<Odometry>::SharedPtr odom_out_pub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
#else
  ros::Publisher odom_out_pub_;
  ros::Subscriber odom_sub_;
#endif
  bool transform_linear_ {true};
  bool transform_angular_ {true};

public:
#ifdef NAV_UTILS_ROS2
  explicit OdometryTwistToChildFrame(const rclcpp::NodeOptions& options);
#else
  OdometryTwistToChildFrame(ros::NodeHandle &nh, ros::NodeHandle &pnh);
#endif

  void processOdometry(const Odometry &odom);
};
}

#endif //NAV_UTILS_ODOM_TWIST_TO_CHILD_FRAME_H
