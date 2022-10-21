#ifndef NAV_UTILS_ODOM_RECOMPUTE_TWIST_H
#define NAV_UTILS_ODOM_RECOMPUTE_TWIST_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>

namespace nav_utils
{
class OdometryRecomputeTwist {
private:
  bool initialized_;
  tf2::Transform last_;
  ros::Time last_stamp_;
  ros::Time last_ros_time_;
  ros::Publisher odom_out_pub_;
  ros::Subscriber odom_sub_;
public:
  OdometryRecomputeTwist(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  void processOdometry(const nav_msgs::Odometry &odom);
};
}

#endif //NAV_UTILS_ODOM_RECOMPUTE_TWIST_H
