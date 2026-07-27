#ifndef NAV_UTILS_ODOM_PROC_H
#define NAV_UTILS_ODOM_PROC_H

#include <Eigen/Eigen>

#ifdef NAV_UTILS_ROS2
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2_ros/buffer.hpp>
#else
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_client/tf2_client.h>
#endif

namespace nav_utils
{
#ifdef NAV_UTILS_ROS2
using Odometry = nav_msgs::msg::Odometry;
#else
using Odometry = nav_msgs::Odometry;
#endif

class OdometryProc
#ifdef NAV_UTILS_ROS2
  : public rclcpp::Node
#endif
{
private:
    std::string renamed_parent_frame_;
    std::string split_child_frame_;
    double max_age_;
    Eigen::Isometry3d last_;
#ifdef NAV_UTILS_ROS2
    rclcpp::Publisher<Odometry>::SharedPtr odom_out_pub_;
    rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
#else
    ros::Publisher odom_out_pub_;
    ros::Subscriber odom_sub_;
    tf2_client::BufferPtr tf_;
#endif
public:
#ifdef NAV_UTILS_ROS2
    explicit OdometryProc(const rclcpp::NodeOptions& options);
#else
    OdometryProc(ros::NodeHandle &nh, ros::NodeHandle &pnh);
#endif
    Odometry processOdometry(const Odometry &odom);
    void odometryReceived(const Odometry &odom);
};
}

#endif //NAV_UTILS_ODOM_PROC_H
