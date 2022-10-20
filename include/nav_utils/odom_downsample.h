#ifndef NAV_UTILS_ODOM_DOWNSAMPLE_H
#define NAV_UTILS_ODOM_DOWNSAMPLE_H

#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace nav_utils
{
class OdometryDownsample {
private:
  size_t every_nth_;
  size_t num_thrown_;
  bool initialized_;
  tf2::Transform last_;
  ros::Time last_stamp_;
  ros::Publisher odom_out_pub_;
  ros::Subscriber odom_sub_;
public:
  OdometryDownsample(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  void processOdometry(const nav_msgs::Odometry &odom);
};
}

#endif //NAV_UTILS_ODOM_DOWNSAMPLE_H
