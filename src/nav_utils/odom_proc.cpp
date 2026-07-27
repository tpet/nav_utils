#include <nav_utils/odom_proc.h>

#ifdef NAV_UTILS_ROS2
#include <rclcpp/time.hpp>
#include <tf2/convert.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2/convert.h>
#include <tf2_client/tf2_client.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#endif

namespace nav_utils
{

#ifdef NAV_UTILS_ROS2
using geometry_msgs::msg::TransformStamped;
#else
using geometry_msgs::TransformStamped;
#endif

#ifdef NAV_UTILS_ROS2
OdometryProc::OdometryProc(const rclcpp::NodeOptions& options)
  : rclcpp::Node("nav_utils", options),
    renamed_parent_frame_(""),
    split_child_frame_(""),
    max_age_(std::numeric_limits<double>::infinity())
{
    declare_parameter("renamed_parent_frame", renamed_parent_frame_);
    declare_parameter("split_child_frame", split_child_frame_);
    declare_parameter("max_age", max_age_);
    get_parameter("renamed_parent_frame", renamed_parent_frame_);
    get_parameter("split_child_frame", split_child_frame_);
    get_parameter("max_age", max_age_);

    odom_out_pub_ = create_publisher<Odometry>("odom_out", 5);
    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    odom_sub_ = create_subscription<Odometry>(
        "odom", 5, [this](const Odometry& msg) { this->odometryReceived(msg); });
}
#else
OdometryProc::OdometryProc(ros::NodeHandle &nh, ros::NodeHandle &pnh):
        renamed_parent_frame_(""),
        split_child_frame_(""),
        max_age_(std::numeric_limits<double>::infinity())
{
    pnh.param("renamed_parent_frame", renamed_parent_frame_, renamed_parent_frame_);
    pnh.param("split_child_frame", split_child_frame_, split_child_frame_);
    pnh.param("max_age", max_age_, max_age_);

    odom_out_pub_ = nh.advertise<nav_msgs::Odometry>("odom_out", 5);
    tf_ = tf2_client::get_buffer(nh, pnh);
    odom_sub_ = nh.subscribe("odom", 5, &OdometryProc::odometryReceived, this);
}
#endif

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
            {1, 0});
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
#ifdef NAV_UTILS_ROS2
    double age = (now() - odom.header.stamp).seconds();
#else
    double age = (ros::Time::now() - odom.header.stamp).toSec();
#endif
    if (age > max_age_)
    {
#ifdef NAV_UTILS_ROS2
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Skipping odometry too old (%.3g s > %.3g s).",
                age, max_age_);
#else
        ROS_WARN_THROTTLE(1.0, "Skipping odometry too old (%.3g s > %.3g s).", age, max_age_);
#endif
        return;
    }
    try {
        Odometry odom_out = processOdometry(odom);
#ifdef NAV_UTILS_ROS2
        odom_out_pub_->publish(odom_out);
#else
        odom_out_pub_.publish(odom_out);
#endif
    }
    catch (tf2::TransformException &ex)
    {
#ifdef NAV_UTILS_ROS2
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Transform lookup failed: %s.", ex.what());
#else
        ROS_WARN_THROTTLE(1.0, "Transform lookup failed: %s.", ex.what());
#endif
    }
}

}

#ifdef NAV_UTILS_ROS2
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nav_utils::OdometryProc)
#endif
