#include <Eigen/Geometry>

#include <nav_msgs/Odometry.h>
#include <nav_utils/odom_twist_to_child_frame.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace nav_utils
{
OdometryTwistToChildFrame::OdometryTwistToChildFrame(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
  odom_out_pub_ = nh.advertise<nav_msgs::Odometry>("odom_out", 5);
  odom_sub_ = nh.subscribe("odom", 5, &OdometryTwistToChildFrame::processOdometry, this);
}

void OdometryTwistToChildFrame::processOdometry(const nav_msgs::Odometry &odom)
{
  tf2::Quaternion parent_to_child_rot;
  tf2::convert(odom.pose.pose.orientation, parent_to_child_rot);
  
  nav_msgs::Odometry odom_out = odom;
  
  // Transform linear twist (just rotate the vector, do not apply translation!)
  tf2::Vector3 linear_twist;
  tf2::convert(odom.twist.twist.linear, linear_twist);
  tf2::convert(tf2::quatRotate(parent_to_child_rot, linear_twist), odom_out.twist.twist.linear);

  // Transform angular velocities
  tf2::Vector3 angular_twist;
  tf2::convert(odom.twist.twist.angular, angular_twist);
  tf2::convert(tf2::quatRotate(parent_to_child_rot, angular_twist), odom_out.twist.twist.angular);
  
  // Transform the covariance
  Eigen::Quaterniond eigen_rot_quat;
  tf2::fromMsg(odom.pose.pose.orientation, eigen_rot_quat);
  Eigen::Matrix3d eigen_rot = eigen_rot_quat.toRotationMatrix();

  Eigen::Matrix<double, 6, 6> rot6d;
  rot6d.topLeftCorner<3, 3>() = eigen_rot;
  rot6d.bottomRightCorner<3, 3>() = eigen_rot;
  rot6d.topRightCorner<3, 3>() = Eigen::Matrix3d::Zero();
  rot6d.bottomLeftCorner<3, 3>() = Eigen::Matrix3d::Zero();

  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> cov(odom_out.twist.covariance.data());
  cov = rot6d * cov * rot6d.transpose();
  
  odom_out_pub_.publish(odom_out);
}

}
