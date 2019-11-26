#include <odom_proc/odom_proc.h>
#include <tf2_client/tf2_client.h>
#include <tf2_ros/buffer.h>


namespace odom_proc
{

OdometryProc::OdometryProc(ros::NodeHandle &nh)
{
    ros::NodeHandle pnh(nh, "~");
    int queue_size = 2;
    pnh.param("child_frame", child_frame_, child_frame_);
    odom_split_pub_ = nh.advertise<nav_msgs::Odometry>("odom_split", 2);
    tf_ = tf2_client::get_buffer(nh);
    odom_sub_ = nh.subscribe("odom", static_cast<uint32_t>(queue_size),
                             &OdometryProc::odometryReceived, this);
}

void OdometryProc::odometryReceived(const nav_msgs::Odometry &odom)
{

}

}
