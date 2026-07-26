#include <nav_utils/odom_to_tf.h>

#ifdef NAV_UTILS_ROS2
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace nav_utils
{
Transform pose_to_transform(const Pose &pose)
{
    Transform tf;
    tf.rotation = pose.orientation;
    tf.translation.x = pose.position.x;
    tf.translation.y = pose.position.y;
    tf.translation.z = pose.position.z;
    return tf;
}

TransformStamped odometry_to_transform(const Odometry &odom)
{
    TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = odom.child_frame_id;
    tf.transform = pose_to_transform(odom.pose.pose);
    return tf;
}

#ifdef NAV_UTILS_ROS2
OdometryToTransform::OdometryToTransform(const rclcpp::NodeOptions& options)
  : rclcpp::Node("nav_utils", options), tf_pub_(this)
{
    declare_parameter("parent_frame", parent_frame_);
    declare_parameter("child_frame", child_frame_);
    declare_parameter("invert_tf", invert_tf_);

    get_parameter("parent_frame", parent_frame_);
    get_parameter("child_frame", child_frame_);
    get_parameter("invert_tf", invert_tf_);

    odom_sub_ = create_subscription<Odometry>(
        "odom", 5, [this](const Odometry& msg) { this->odometryReceived(msg); });
}
#else
OdometryToTransform::OdometryToTransform(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    pnh.param("parent_frame", parent_frame_, parent_frame_);
    pnh.param("child_frame", child_frame_, child_frame_);
    pnh.param("invert_tf", invert_tf_, invert_tf_);
    odom_sub_ = nh.subscribe("odom", 5, &OdometryToTransform::odometryReceived, this);
}
#endif

void OdometryToTransform::odometryReceived(const Odometry &msg)
{
    TransformStamped tf = odometry_to_transform(msg);
    if (!parent_frame_.empty())
        tf.header.frame_id = parent_frame_;
    if (!child_frame_.empty())
        tf.child_frame_id = child_frame_;
    if (invert_tf_)
      tf.transform = tf2::eigenToTransform(tf2::transformToEigen(tf.transform).inverse()).transform;
    tf_pub_.sendTransform(tf);
}

}

#ifdef NAV_UTILS_ROS2
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nav_utils::OdometryToTransform);
#endif