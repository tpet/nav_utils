#include <nav_utils/odom_downsample.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace nav_utils
{
OdometryDownsample::OdometryDownsample(ros::NodeHandle &nh, ros::NodeHandle &pnh):
  every_nth_(2u), num_thrown_(0u), initialized_(false)
{
  int intParam;
  pnh.param("every_nth", intParam, static_cast<int>(every_nth_));
  every_nth_ = static_cast<size_t>(intParam);
    
  odom_out_pub_ = nh.advertise<nav_msgs::Odometry>("odom_out", 5);
  odom_sub_ = nh.subscribe("odom", 5, &OdometryDownsample::processOdometry, this);
}

void OdometryDownsample::processOdometry(const nav_msgs::Odometry &odom)
{
  if (!initialized_)
  {
    tf2::convert(odom.pose.pose, last_);
    last_stamp_ = odom.header.stamp;
    initialized_ = true;
    return;
  }
  
  if (num_thrown_ < every_nth_)
  {
    num_thrown_ += 1;
    return;
  }

  const auto dt = (odom.header.stamp - last_stamp_).toSec();
  if (dt <= 0)
  {
    num_thrown_ += 1;
    ROS_ERROR_THROTTLE(1.0, "Negative timestamp sequence encountered.");
    return;
  }
  
  num_thrown_ = 0;
  tf2::Transform this_pose;
  tf2::convert(odom.pose.pose, this_pose);
  
  nav_msgs::Odometry odom_out = odom;
  const auto diff = last_.inverse() * this_pose;
  const auto& linear_twist = diff.getOrigin() / dt;
  odom_out.twist.twist.linear.x = linear_twist.x();
  odom_out.twist.twist.linear.y = linear_twist.y();
  odom_out.twist.twist.linear.z = linear_twist.z();
  const auto& angular_twist = diff.getBasis();
  tf2::Quaternion tf2_quat_twist;
  angular_twist.getRotation(tf2_quat_twist);
  Eigen::Quaterniond eigen_quat_twist;
  tf2::convert(tf2_quat_twist, eigen_quat_twist);
  Eigen::AngleAxisd aa_twist(eigen_quat_twist);
  aa_twist.angle() /= dt;
  eigen_quat_twist = aa_twist;
  tf2::convert(eigen_quat_twist, tf2_quat_twist);
  tf2::Matrix3x3 tf2_mat_twist(tf2_quat_twist);
  double roll, pitch, yaw;
  tf2_mat_twist.getRPY(roll, pitch, yaw);
  odom_out.twist.twist.angular.x = roll;
  odom_out.twist.twist.angular.y = pitch;
  odom_out.twist.twist.angular.z = yaw;
  
  // TODO also add correct twist covariance
  
  odom_out_pub_.publish(odom_out);

  last_ = this_pose;
  last_stamp_ = odom.header.stamp;
}

}
