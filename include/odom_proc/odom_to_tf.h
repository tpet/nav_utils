#ifndef ODOM_PROC_ODOM_TO_TF_H
#define ODOM_PROC_ODOM_TO_TF_H

#include <odom_proc/odom_proc.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

namespace odom_proc
{

geometry_msgs::TransformStamped odometry_to_transform(const nav_msgs::Odometry &odom);

class OdometryToTransform {
private:
    ros::Subscriber odom_sub_;
    tf2_ros::TransformBroadcaster tf_pub_;
public:
    OdometryToTransform(ros::NodeHandle &nh);
    void odometryReceived(const nav_msgs::Odometry &msg);
};

}

#endif //ODOM_PROC_ODOM_TO_TF_H
