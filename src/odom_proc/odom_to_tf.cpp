#include <odom_proc/odom_to_tf.h>

namespace odom_proc
{

geometry_msgs::TransformStamped odometry_to_transform(const nav_msgs::Odometry &odom)
{
    geometry_msgs::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = odom.child_frame_id;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf.transform.translation.x = odom.pose.pose.position.x;
    tf.transform.translation.y = odom.pose.pose.position.y;
    tf.transform.translation.z = odom.pose.pose.position.z;
}

OdometryToTransform::OdometryToTransform(ros::NodeHandle &nh)
{
    ros::NodeHandle pnh(nh, "~");
    int queue_size = 2;
    pnh.param("queue_size", queue_size, queue_size);
    odom_sub_ = nh.subscribe("odom", static_cast<uint32_t>(queue_size),
            &OdometryToTransform::odometryReceived, this);
}

void OdometryToTransform::odometryReceived(const nav_msgs::Odometry &msg)
{
    tf_pub_.sendTransform(odometry_to_transform(msg));
}

}
