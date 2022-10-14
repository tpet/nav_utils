#pragma once

#include <Eigen/Eigen>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>

namespace nav_utils
{

geometry_msgs::TransformStamped lookup_transform(const tf2_ros::BufferInterface & tf,
                                                 const std::string & target_frame,
                                                 const std::string & source_frame,
                                                 const ros::Time & time,
                                                 const ros::Duration & timeout,
                                                 const std::string & no_wait_frame = "")
{
    if (no_wait_frame.empty())
        return tf.lookupTransform(target_frame, source_frame, time, timeout);
    ros::Time no_wait;
    geometry_msgs::TransformStamped tf_n2t = tf.lookupTransform(target_frame, no_wait_frame, no_wait, ros::Duration());
    Eigen::Isometry3d n2t = tf2::transformToEigen(tf_n2t.transform);
    geometry_msgs::TransformStamped tf_s2n = tf.lookupTransform(no_wait_frame, source_frame, time, timeout);
    Eigen::Isometry3d s2n = tf2::transformToEigen(tf_s2n.transform);
    Eigen::Isometry3d s2t = n2t * s2n;
    auto tf_s2t = tf2::eigenToTransform(s2t);
    tf_s2t.header.frame_id = target_frame;
    tf_s2t.header.stamp = time;
    tf_s2t.child_frame_id = source_frame;
    return tf_s2t;
}

}
