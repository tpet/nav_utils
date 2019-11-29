#include <odom_proc/odom_proc.h>
#include <tf2_client/tf2_client.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <message_filters/chain.h>

namespace odom_proc
{

using nav_msgs::Odometry;
using geometry_msgs::TransformStamped;

OdometryProc::OdometryProc(ros::NodeHandle &nh, ros::NodeHandle &pnh):
        renamed_parent_frame_(""),
        split_child_frame_(""),
        max_age_(std::numeric_limits<double>::infinity())
{
    pnh.param("renamed_parent_frame", renamed_parent_frame_, renamed_parent_frame_);
    pnh.param("split_child_frame", split_child_frame_, split_child_frame_);
    pnh.param("max_age", max_age_, max_age_);
    int queue_size = 2;
    odom_out_pub_ = nh.advertise<nav_msgs::Odometry>("odom_out", 2);
    tf_ = tf2_client::get_buffer(nh, pnh);
    odom_sub_ = nh.subscribe("odom", static_cast<uint32_t>(queue_size),
                             &OdometryProc::odometryReceived, this);
}

Odometry OdometryProc::processOdometry(const Odometry &odom)
{
    Odometry odom_out = odom;
    if (!renamed_parent_frame_.empty())
        odom_out.header.frame_id = renamed_parent_frame_;
    if (split_child_frame_.empty())
        return odom_out;
    TransformStamped tf_cr = tf_->lookupTransform(
            odom.child_frame_id,
            split_child_frame_,
            odom_out.header.stamp,
            ros::Duration(1.0));
    Eigen::Isometry3d T_cr = tf2::transformToEigen(tf_cr.transform);
    Eigen::Isometry3d T_oc;
    tf2::convert(odom.pose.pose, T_oc);
    Eigen::Isometry3d T_or = T_oc * T_cr;
    tf2::convert(T_or, odom_out.pose.pose);
    odom_out.child_frame_id = split_child_frame_;
    return odom_out;
}

void OdometryProc::odometryReceived(const Odometry &odom)
{
    double age = (ros::Time::now() - odom.header.stamp).toSec();
    if (age > max_age_)
    {
        ROS_WARN("Skipping odometry too old (%.3g s > %.3g s).",
                age, max_age_);
        return;
    }
    Odometry odom_out = processOdometry(odom);
    odom_out_pub_.publish(odom_out);
}

}
