#ifndef NAV_UTILS_ODOM_TO_TF_H
#define NAV_UTILS_ODOM_TO_TF_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

namespace nav_utils
{
geometry_msgs::Transform pose_to_transform(const geometry_msgs::Pose &pose);
geometry_msgs::TransformStamped odometry_to_transform(const nav_msgs::Odometry &odom);

class OdometryToTransform {
private:
    tf2_ros::TransformBroadcaster tf_pub_;
    ros::Subscriber odom_sub_;
    std::string parent_frame_;
    std::string child_frame_;
    bool invert_tf_{false};
public:
    OdometryToTransform(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    void odometryReceived(const nav_msgs::Odometry &msg);
};
}

#endif //NAV_UTILS_ODOM_TO_TF_H
