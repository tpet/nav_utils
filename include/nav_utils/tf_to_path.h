#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <topic_tools/shape_shifter.h>
#include <nav_msgs/Path.h>

#include <tf2_client/tf2_client.h>
#include <tf2_ros/buffer.h>

#include <mutex>
#include <set>

using time_point = std::pair<double, geometry_msgs::Point>;
struct time_point_cmp
{
  bool operator()(const time_point &lhs, const time_point &rhs) const {
    return lhs.first < rhs.first;
  }
};

namespace nav_utils
{

/* std::pair<double, geometry_msgs::Point> transform_stamped_to_time_point_pair(const geometry_msgs::TransformStamped &tf); */
geometry_msgs::PoseStamped pair_to_pose(const time_point &p);
geometry_msgs::Point       transform_to_point(const geometry_msgs::Transform &tf);
double                     squared_norm(const geometry_msgs::Point &point_A, const geometry_msgs::Point &point_B);
double                     squared_norm(const time_point &point_A, const time_point &point_B);

class TransformToPath {
private:
  ros::Publisher                          pub_path_;
  ros::Subscriber                         sub_tf_msg_;
  ros::Timer                              timer_pub_path_;
  void                                    callbackTfMsg(const tf2_msgs::TFMessage::ConstPtr &msg);
  void                                    publishTimerCallback(const ros::TimerEvent &event);
  std::vector<geometry_msgs::PoseStamped> trajectoryToPath();

  tf2_client::BufferPtr buffer_;
  ros::Duration         lookup_timeout_;

  std::string              parent_frame_;
  std::vector<std::string> children_frames_;

  std::mutex                           mutex_trajectory_;
  std::set<time_point, time_point_cmp> trajectory_;  // Ordered set by time

  float publish_frequency_;
  float sample_distance_squared_;

  int sub_queue_size_;

  int prev_path_size_ = 0;

public:
  TransformToPath(ros::NodeHandle &nh, ros::NodeHandle &pnh);

  /* std_msgs::Header    extract_header(const topic_tools::ShapeShifter &msg); */
  /* geometry_msgs::Pose lookupPose(const ros::Time &stamp, const std::string &child_frame); */
  /* void                publishMessages(const ros::Time &stamp, const std::string &child_frame); */
  /* void                triggerReceived(const topic_tools::ShapeShifter &msg); */
  /* void                timerCallback(const ros::TimerEvent &event); */
};
}  // namespace nav_utils
