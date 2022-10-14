#include <nav_utils/no_wait_lookup.h>
#include <nav_utils/orient_frame.h>
#include <tf2_msgs/TFMessage.h>

namespace nav_utils
{

OrientFrame::OrientFrame(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    pnh.param("parent_frame", parent_frame_, parent_frame_);
    pnh.param("child_frame", child_frame_, child_frame_);
    pnh.param("oriented_frame", oriented_frame_, oriented_frame_);
    pnh.param("no_wait_frame", no_wait_frame_, no_wait_frame_);
    pnh.param("align", align_, align_);
    pnh.param("timeout", timeout_, timeout_);
    pnh.param("timeout_relative", timeout_relative_, timeout_relative_);
    pnh.param("trigger_queue_size", trigger_queue_size_, trigger_queue_size_);
    pnh.param("tf_queue_size", tf_queue_size_, tf_queue_size_);
    pnh.param("latch", latch_, latch_);
    tf_pub_ = nh.advertise<tf2_msgs::TFMessage>("/tf", uint32_t(tf_queue_size_), latch_);
    tf_sub_ = std::make_unique<tf2_ros::TransformListener>(tf_);
    auto callback = [this](const std_msgs::Header & msg) { publishFor(msg); };
    trigger_sub_ = HeaderSubscriber(nh, "trigger", uint32_t(trigger_queue_size_), callback);
}

void OrientFrame::publishFor(const std_msgs::Header & trigger) const
{
    std::string child_frame = child_frame_.empty() ? trigger.frame_id : child_frame_;
    geometry_msgs::TransformStamped tf;
    try
    {
        tf = lookup_transform(tf_, parent_frame_, child_frame, trigger.stamp, ros::Duration(timeout_), no_wait_frame_);
    }
    catch (const tf2::TransformException & ex)
    {
        ROS_ERROR("Cannot transform from %s to %s: %s.", child_frame.c_str(), parent_frame_.c_str(), ex.what());
        return;
    }

    tf.header.stamp = trigger.stamp;
    tf.header.frame_id = parent_frame_;
    tf.child_frame_id = oriented_frame_;
    if (align_ == "xyz")
    {
        tf.transform.rotation.x = 0;
        tf.transform.rotation.y = 0;
        tf.transform.rotation.z = 0;
        tf.transform.rotation.w = 1;
    }
    else
    {
        Eigen::Isometry3d T = tf2::transformToEigen(tf.transform);
        Eigen::Vector3d a;
        Eigen::Vector3d b;
        if (align_ == "x")
        {
            a = T.linear().col(0);
            b = Eigen::Vector3d::UnitX();
        }
        else if (align_ == "y")
        {
            a = T.linear().col(1);
            b = Eigen::Vector3d::UnitY();
        }
        else if (align_ == "z")
        {
            a = T.linear().col(2);
            b = Eigen::Vector3d::UnitZ();
        }
        else
        {
            ROS_ERROR_STREAM("Invalid align: " << align_);
            return;
        }
        Eigen::Vector3d v = a.cross(b).normalized();
        double angle = std::acos(a.dot(b));
        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(angle, v) * T.linear();
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
    }
    tf2_msgs::TFMessage msg;
    msg.transforms.push_back(tf);
    tf_pub_.publish(msg);
}

}
