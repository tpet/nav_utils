#ifndef ODOM_PROC_ODOM_TO_TF_H
#define ODOM_PROC_ODOM_TO_TF_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

namespace odom_proc
{
geometry_msgs::Transform pose_to_transform(const geometry_msgs::Pose &pose);
geometry_msgs::TransformStamped odometry_to_transform(const nav_msgs::Odometry &odom);

class OdometryToTransform {
private:
    tf2_ros::TransformBroadcaster tf_pub_;
    ros::Subscriber odom_sub_;
public:
    OdometryToTransform(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    void odometryReceived(const nav_msgs::Odometry &msg);
};
}

#endif //ODOM_PROC_ODOM_TO_TF_H
