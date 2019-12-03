#include <nav_utils/odom_to_tf.h>

namespace nav_utils
{
geometry_msgs::Transform pose_to_transform(const geometry_msgs::Pose &pose)
{
    geometry_msgs::Transform tf;
    tf.rotation = pose.orientation;
    tf.translation.x = pose.position.x;
    tf.translation.y = pose.position.y;
    tf.translation.z = pose.position.z;
    return tf;
}

geometry_msgs::TransformStamped odometry_to_transform(const nav_msgs::Odometry &odom)
{
    geometry_msgs::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = odom.child_frame_id;
    tf.transform = pose_to_transform(odom.pose.pose);
    return tf;
}

OdometryToTransform::OdometryToTransform(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    odom_sub_ = nh.subscribe("odom", 5, &OdometryToTransform::odometryReceived, this);
}

void OdometryToTransform::odometryReceived(const nav_msgs::Odometry &msg)
{
    tf_pub_.sendTransform(odometry_to_transform(msg));
}

}
