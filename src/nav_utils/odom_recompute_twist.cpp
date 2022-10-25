#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_utils/odom_recompute_twist.h>

namespace nav_utils
{
OdometryRecomputeTwist::OdometryRecomputeTwist(ros::NodeHandle &nh, ros::NodeHandle &pnh):
  initialized_(false)
{
  odom_out_pub_ = nh.advertise<nav_msgs::Odometry>("odom_out", 5);
  odom_sub_ = nh.subscribe("odom", 5, &OdometryRecomputeTwist::processOdometry, this);
}

void OdometryRecomputeTwist::processOdometry(const nav_msgs::Odometry &odom)
{
  if (initialized_ && ros::Time::now() + ros::Duration(3) < last_ros_time_)
  {
    initialized_ = false;
    ROS_INFO("ROS time jumped back, resetting.");
  }
  
  last_ros_time_ = ros::Time::now();

  if (!initialized_)
  {
    tf2::convert(odom.pose.pose, last_);
    last_stamp_ = odom.header.stamp;
    initialized_ = true;
    return;
  }
  
  const auto dt = (odom.header.stamp - last_stamp_).toSec();
  if (dt <= 0)
  {
    ROS_ERROR_THROTTLE(1.0, "Negative timestamp sequence encountered.");
    return;
  }
  
  tf2::Transform this_pose;
  tf2::convert(odom.pose.pose, this_pose);
  const auto diff = last_.inverse() * this_pose;

  nav_msgs::Odometry odom_out = odom;

  tf2::convert(diff.getOrigin() / dt, odom_out.twist.twist.linear);

  // To get the angular difference, convert to angle-axis representation and
  // divide the angle by dt; the final x/y/z components correspond to roll/pitch/yaw
  const auto angular_diff = diff.getRotation();
  const auto angle = angular_diff.getAngle();
  const auto axis = angular_diff.getAxis();
  tf2::Quaternion scaled_angular_diff(axis, angle / dt);

  tf2::Matrix3x3(scaled_angular_diff).getRPY(
    odom_out.twist.twist.angular.x,
    odom_out.twist.twist.angular.y,
    odom_out.twist.twist.angular.z);
  
  // twist covariance remains the same, there is probably no good way to improve it
  
  odom_out_pub_.publish(odom_out);

  last_ = this_pose;
  last_stamp_ = odom.header.stamp;
}

}
