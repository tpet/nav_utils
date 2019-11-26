#ifndef ODOM_PROC_ODOM_PROC_H
#define ODOM_PROC_ODOM_PROC_H

#include <Eigen/Eigen>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_client/tf2_client.h>

namespace odom_proc
{
class OdometryProc {
private:
    std::string child_frame_;
    Eigen::AffineCompact3d last_;
    ros::Publisher odom_split_pub_;
    ros::Subscriber odom_sub_;
//    tf2_ros::BufferInterface &tf_;
    tf2_client::BufferPtr tf_;
public:
    OdometryProc(ros::NodeHandle &nh);
    void odometryReceived(const nav_msgs::Odometry &odom);
};
}

#endif //ODOM_PROC_ODOM_PROC_H
