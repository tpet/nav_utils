#include <nav_utils/tf_to_odom.h>
#include <nav_utils/utils.h>
#include <tf2_eigen/tf2_eigen.h>

namespace nav_utils
{
geometry_msgs::Pose transform_to_pose(const geometry_msgs::Transform &tf)
{
    geometry_msgs::Pose pose;
    pose.orientation = tf.rotation;
    pose.position.x = tf.translation.x;
    pose.position.y = tf.translation.y;
    pose.position.z = tf.translation.z;
    return pose;
}

nav_msgs::Odometry transform_to_odometry(const geometry_msgs::TransformStamped &tf)
{
    nav_msgs::Odometry odom;
    odom.header = tf.header;
    odom.child_frame_id = tf.child_frame_id;
    odom.pose.pose = transform_to_pose(tf.transform);
    return odom;
}

TransformToOdometry::TransformToOdometry(ros::NodeHandle &nh, ros::NodeHandle &pnh):
        parent_frame_(""),
        child_frame_(""),
        no_wait_frame_(""),
        timeout_(1.0),
        timer_freq_(0.0),
        trigger_queue_size_(5),
        odom_queue_size_(5),
        sleep_after_trigger_(0.0)
{
    pnh.param("parent_frame", parent_frame_, parent_frame_);
    pnh.param("child_frame", child_frame_, child_frame_);
    pnh.param("no_wait_frame", no_wait_frame_, no_wait_frame_);
    pnh.param("timeout", timeout_, timeout_);
    pnh.param("timeout_relative", timeout_relative_, timeout_relative_);
    pnh.param("timer_freq", timer_freq_, timer_freq_);
    pnh.param("odom_queue_size", odom_queue_size_, odom_queue_size_);
    pnh.param("trigger_queue_size", trigger_queue_size_, trigger_queue_size_);
    pnh.param("sleep_after_trigger", sleep_after_trigger_, sleep_after_trigger_);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", static_cast<uint32_t>(odom_queue_size_));
    tf_ = tf2_client::get_buffer(nh, pnh);
    trigger_sub_ = nh.subscribe("trigger", static_cast<uint32_t>(trigger_queue_size_),
            &TransformToOdometry::triggerReceived, this);
    if (timer_freq_ > 0.0)
    {
        ros::Duration period(1. / timer_freq_);
        timer_ = nh.createTimer(period, &TransformToOdometry::timerCallback, this);
    }
}

std::string TransformToOdometry::resolveChildFrame(const std::string &child_frame)
{
    return child_frame_.empty() ? child_frame : child_frame_;
}

std_msgs::Header TransformToOdometry::extract_header(const topic_tools::ShapeShifter &msg)
{
    // See https://answers.ros.org/question/226392/extract-header-from-shapeshifter/
    static std::string md5sum = ros::message_traits::md5sum<std_msgs::Header>();
    static std::string datatype = ros::message_traits::datatype<std_msgs::Header>();
    static std::string definition = ros::message_traits::definition<std_msgs::Header>();
    auto &tmp = const_cast<topic_tools::ShapeShifter &>(msg);
    tmp.morph(md5sum, datatype, definition, "");
    std_msgs::Header::Ptr header = tmp.instantiate<std_msgs::Header>();
    return *header;
}

geometry_msgs::Pose TransformToOdometry::lookupPose(const ros::Time &stamp, const std::string &child_frame)
{
    ros::Duration timeout(timeout_relative_
            ? std::max(timeout_ - (ros::Time::now() - stamp).toSec(), 0.)
            : timeout_);
    if (no_wait_frame_.empty())
    {
        geometry_msgs::TransformStamped tf_pc = tf_->lookupTransform(parent_frame_, child_frame, stamp, timeout);
        geometry_msgs::Pose pose = transform_to_pose(tf_pc.transform);
        return pose;
    }
    ros::Time no_wait;
    geometry_msgs::TransformStamped tf_pn = tf_->lookupTransform(parent_frame_, no_wait_frame_, no_wait, timeout);
//    geometry_msgs::TransformStamped tf_pn;
    Eigen::Isometry3d T_pn = tf2::transformToEigen(tf_pn.transform);
    // Waiting lookup slows it to around 13 Hz when using timer.
    geometry_msgs::TransformStamped tf_nc = tf_->lookupTransform(no_wait_frame_, child_frame, stamp, timeout);
//    geometry_msgs::TransformStamped tf_nc = tf_->lookupTransform(no_wait_frame_, child_frame, no_wait,
//            ros::Duration(timeout_));
//    geometry_msgs::TransformStamped tf_nc;
    Eigen::Isometry3d T_nc = tf2::transformToEigen(tf_nc.transform);
    Eigen::Isometry3d T_pc = T_pn * T_nc;
    geometry_msgs::Pose pose;
    tf2::convert(T_pc, pose);
    return pose;
}

void TransformToOdometry::publishMessages(const ros::Time &stamp, const std::string &child_frame)
{
    nav_msgs::Odometry odom;
    odom.header.frame_id = parent_frame_;
    odom.header.stamp = stamp;
    odom.child_frame_id = child_frame;
    odom.pose.pose = lookupPose(stamp, child_frame);
    odom_pub_.publish(odom);
}

void TransformToOdometry::triggerReceived(const topic_tools::ShapeShifter &msg)
{
    std_msgs::Header header = extract_header(msg);
    ROS_DEBUG("trigger (header: %s): delay: %.3g s",
            header.frame_id.c_str(), (ros::Time::now() - header.stamp).toSec());
    try
    {
        publishMessages(header.stamp, resolveChildFrame(header.frame_id));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR_THROTTLE(1.0, "Could not lookup transform to %s from %s at time %.3g s ago.",
                parent_frame_.c_str(), resolveChildFrame(header.frame_id).c_str(),
                (ros::Time::now() - header.stamp).toSec());
    }
    if (sleep_after_trigger_ > 0.0)
    {
        sleepFor(sleep_after_trigger_);
    }
}

void TransformToOdometry::timerCallback(const ros::TimerEvent& event)
{
    ROS_DEBUG("timer: delay: %.3g s, expected: %.3g s, real: %.3g s",
            (ros::Time::now() - event.current_expected).toSec(),
            event.current_expected.toSec(), event.current_real.toSec());
    try
    {
        publishMessages(event.current_real, resolveChildFrame(""));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR_THROTTLE(1.0, "Could not lookup transform to %s from %s at time %.3g s ago.",
                parent_frame_.c_str(), child_frame_.c_str(),
                (ros::Time::now() - event.current_real).toSec());
    }
}
}
