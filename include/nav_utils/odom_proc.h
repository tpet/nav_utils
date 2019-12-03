#ifndef NAV_UTILS_ODOM_PROC_H
#define NAV_UTILS_ODOM_PROC_H

#include <Eigen/Eigen>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_client/tf2_client.h>

namespace nav_utils
{
class OdometryProc {
private:
    std::string renamed_parent_frame_;
    std::string split_child_frame_;
    double max_age_;
    Eigen::Isometry3d  last_;
    ros::Publisher odom_out_pub_;
    ros::Subscriber odom_sub_;
    tf2_client::BufferPtr tf_;
public:
    OdometryProc(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    nav_msgs::Odometry processOdometry(const nav_msgs::Odometry &odom);
    void odometryReceived(const nav_msgs::Odometry &odom);
};
}

#endif //NAV_UTILS_ODOM_PROC_H
