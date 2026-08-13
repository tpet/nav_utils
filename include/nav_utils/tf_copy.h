#ifndef NAV_UTILS_TF_COPY_H
#define NAV_UTILS_TF_COPY_H

#include <memory>
#include <optional>

#ifdef NAV_UTILS_ROS2
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#else
#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <tf2_client/tf2_client.h>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#endif

namespace nav_utils
{

#ifdef NAV_UTILS_ROS2
using geometry_msgs::msg::Transform;
using geometry_msgs::msg::TransformStamped;
using tf2_msgs::msg::TFMessage;
#else
using Transform = geometry_msgs::Transform;
using TransformStamped = geometry_msgs::TransformStamped;
using TFMessage = tf2_msgs::TFMessage;
#endif

class TfCopy
#ifdef NAV_UTILS_ROS2
  : public rclcpp::Node
#endif
{
public:
#ifdef NAV_UTILS_ROS2
  explicit TfCopy(const rclcpp::NodeOptions& options);
  ~TfCopy() override = default;
#else
  TfCopy(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  virtual ~TfCopy() = default;
#endif

  void run();

protected:
  void tfCb(const TFMessage& msg, bool is_static);
  void copyTf(const TransformStamped& msg);
  void periodicCb();

#ifdef NAV_UTILS_ROS2
  tf2_ros::Buffer::SharedPtr buffer;
  rclcpp::Subscription<TFMessage>::SharedPtr tfSub;
  rclcpp::Subscription<TFMessage>::SharedPtr tfStaticSub;
#else
  std::shared_ptr<tf2_ros::Buffer> buffer;
  ros::Subscriber tfSub;
  ros::Subscriber tfStaticSub;
#endif
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfPublisher;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tfStaticPublisher;

  Transform lastTransform;

  std::string parentFrame;
  std::string childFrame;
  std::string newParentFrame;
  std::string newChildFrame;
#ifdef NAV_UTILS_ROS2
  rclcpp::Rate::UniquePtr publishRate;
  rclcpp::TimerBase::SharedPtr publishTimer;
#else
  std::unique_ptr<ros::Rate> publishRate;
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
#endif
  bool staticTf;
  bool invertTf;
  bool updateStamp;
  bool errorIfTfMissing;
  std::optional<bool> directMode;
};

}
#endif //NAV_UTILS_TF_COPY_H