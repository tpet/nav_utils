#ifndef NAV_UTILS_ODOM_TWIST_TO_CHILD_FRAME_H
#define NAV_UTILS_ODOM_TWIST_TO_CHILD_FRAME_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace nav_utils
{
class OdometryTwistToChildFrame {
private:
  ros::Publisher odom_out_pub_;
  ros::Subscriber odom_sub_;
public:
  OdometryTwistToChildFrame(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  void processOdometry(const nav_msgs::Odometry &odom);
};
}

#endif //NAV_UTILS_ODOM_TWIST_TO_CHILD_FRAME_H
