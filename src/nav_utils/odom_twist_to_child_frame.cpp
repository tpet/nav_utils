#include <Eigen/Core>
#include <Eigen/Geometry>

#include <nav_utils/odom_twist_to_child_frame.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#ifdef NAV_UTILS_ROS2
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

namespace nav_utils
{

#ifdef NAV_UTILS_ROS2
using geometry_msgs::msg::Transform;
#else
using geometry_msgs::Transform;
#endif

#ifdef NAV_UTILS_ROS2
OdometryTwistToChildFrame::OdometryTwistToChildFrame(const rclcpp::NodeOptions& options)
  : rclcpp::Node("nav_utils", options)
{
  declare_parameter("transform_linear", transform_linear_);
  declare_parameter("transform_angular", transform_angular_);
  get_parameter("transform_linear", transform_linear_);
  get_parameter("transform_angular", transform_angular_);

  odom_out_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom_out", 5);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", 5, [this](const nav_msgs::msg::Odometry& msg) { this->processOdometry(msg); });
}
#else
OdometryTwistToChildFrame::OdometryTwistToChildFrame(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
  pnh.param("transform_linear", transform_linear_, transform_linear_);
  pnh.param("transform_angular", transform_angular_, transform_angular_);

  odom_out_pub_ = nh.advertise<nav_msgs::Odometry>("odom_out", 5);
  odom_sub_ = nh.subscribe("odom", 5, &OdometryTwistToChildFrame::processOdometry, this);
}
#endif

void OdometryTwistToChildFrame::processOdometry(const Odometry &odom)
{
  tf2::Transform child_to_parent_tf;
  tf2::convert(odom.pose.pose, child_to_parent_tf);
  const auto parent_to_child_tf = child_to_parent_tf.inverse();
  
  Odometry odom_out = odom;
  
  // Transform linear twist (just rotate the vector, do not apply translation!)
  if (transform_linear_)
  {
    tf2::Vector3 linear_twist;
    tf2::convert(odom.twist.twist.linear, linear_twist);
    tf2::convert(parent_to_child_tf.getBasis() * linear_twist, odom_out.twist.twist.linear);
  }

  // Transform angular velocities
  if (transform_angular_)
  {
    tf2::Vector3 angular_twist;
    tf2::convert(odom.twist.twist.angular, angular_twist);
    tf2::convert(parent_to_child_tf.getBasis() * angular_twist, odom_out.twist.twist.angular);
  }
  
  // Transform the covariance
  if (transform_linear_ && transform_angular_)
  {
    odom_out.twist.covariance = tf2::transformCovariance(odom_out.twist.covariance, parent_to_child_tf);
  }
  else if (transform_linear_ || transform_angular_)
  {
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> cov_world(odom_out.twist.covariance.data());

    Transform tf;
    tf2::convert(parent_to_child_tf, tf);
    const auto parent_to_child_tf_eig = tf2::transformToEigen(tf);
    const Eigen::Matrix3d R_world_to_body = parent_to_child_tf_eig.linear();

    // Construct the 6x6 block-diagonal rotation matrix
    Eigen::Matrix<double, 6, 6> R_twist = Eigen::Matrix<double, 6, 6>::Identity();
    if (transform_linear_)
      R_twist.block<3, 3>(0, 0) = R_world_to_body;
    if (transform_angular_)
      R_twist.block<3, 3>(3, 3) = R_world_to_body;

    // Rotate the covariance matrix: Cov_body = R * Cov_world * R^T
    Eigen::Matrix<double, 6, 6> cov_body = R_twist * cov_world * R_twist.transpose();

    cov_world = cov_body;
  }

#ifdef NAV_UTILS_ROS2
  odom_out_pub_->publish(odom_out);
#else
  odom_out_pub_.publish(odom_out);
#endif
}

}

#ifdef NAV_UTILS_ROS2
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nav_utils::OdometryTwistToChildFrame);
#endif
