#include <nav_utils/tf_copy.h>

#ifdef NAV_UTILS_ROS2
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

namespace nav_utils
{

inline bool equal(const Transform& t1, const Transform& t2)
{
  return
    t1.translation.x == t2.translation.x &&
    t1.translation.y == t2.translation.y &&
    t1.translation.z == t2.translation.z &&
    t1.rotation.x == t2.rotation.x &&
    t1.rotation.y == t2.rotation.y &&
    t1.rotation.z == t2.rotation.z &&
    t1.rotation.w == t2.rotation.w;
}

inline Transform invertTransform(const Transform& msg) {
  return tf2::eigenToTransform(tf2::transformToEigen(msg).inverse()).transform;
}

#ifdef NAV_UTILS_ROS2

TfCopy::TfCopy(const rclcpp::NodeOptions& options)
  : rclcpp::Node("nav_utils", options), parentFrame("map"), childFrame("odom"), staticTf(false), invertTf(false),
    updateStamp(false), errorIfTfMissing(true)
{
  this->declare_parameter("parent_frame", this->parentFrame);
  this->declare_parameter("child_frame", this->childFrame);
  this->declare_parameter("new_parent_frame", this->get_parameter("parent_frame").as_string() + "_copy");
  this->declare_parameter("new_child_frame", this->get_parameter("child_frame").as_string() + "_copy");
  this->declare_parameter("publish_frequency", 10.0);
  this->declare_parameter("error_if_tf_missing", this->errorIfTfMissing);
  this->declare_parameter("invert_tf", this->invertTf);
  this->declare_parameter("static_tf", this->staticTf);
  this->declare_parameter("update_stamp", this->updateStamp);
  this->declare_parameter("input_tf_topic", "/tf");
  this->declare_parameter("input_tf_static_topic", "/tf_static");

  this->parentFrame = this->get_parameter("parent_frame").as_string();
  this->childFrame = this->get_parameter("child_frame").as_string();
  this->newParentFrame = this->get_parameter("new_parent_frame").as_string();
  this->newChildFrame = this->get_parameter("new_child_frame").as_string();

  const auto frequency = this->get_parameter("publish_frequency").as_double();
  if (frequency != 0.0) {
    this->publishRate = std::make_unique<rclcpp::Rate>(frequency);
  }

  this->errorIfTfMissing = this->get_parameter("error_if_tf_missing").as_bool();
  this->invertTf = this->get_parameter("invert_tf").as_bool();
  this->staticTf = this->get_parameter("static_tf").as_bool();
  this->updateStamp = this->get_parameter("update_stamp").as_bool();

  this->buffer = std::make_shared<tf2_ros::Buffer>(get_clock());

  this->run();
}
#else
TfCopy::TfCopy(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  : nh(nh), pnh(pnh), parentFrame("map"), childFrame("odom"), staticTf(false), invertTf(false), updateStamp(false),
    errorIfTfMissing(true)
{
  pnh.param("parent_frame", this->parentFrame, this->parentFrame);
  pnh.param("child_frame", this->childFrame, this->childFrame);
  pnh.param("new_parent_frame", this->newParentFrame, this->parentFrame + "_copy");
  pnh.param("new_child_frame", this->newChildFrame, this->childFrame + "_copy");

  double frequency = 0.0;
  pnh.param("publish_frequency", frequency, 10.0);
  if (frequency != 0.0) {
    this->publishRate = std::make_unique<ros::Rate>(frequency);
  }

  pnh.param("error_if_tf_missing", this->errorIfTfMissing, this->errorIfTfMissing);
  pnh.param("invert_tf", this->invertTf, this->invertTf);
  pnh.param("static_tf", this->staticTf, this->staticTf);
  pnh.param("update_stamp", this->updateStamp, this->updateStamp);

  this->buffer = std::make_shared<tf2_ros::Buffer>();
}
#endif

void TfCopy::tfCb(const TFMessage& msg, const bool is_static)
{
  // Detect direct mode
  // If we see one of our transforms as a child, we know that for direct mode, the other has to be its parent
  // We run this detection every time to be able to cope with reparenting in the TF tree
  for (const auto& tf : msg.transforms)
  {
    const auto prevDirectMode = this->directMode;
    if (tf.child_frame_id == this->childFrame)
    {
      this->directMode = tf.header.frame_id == this->parentFrame;
      if (!prevDirectMode.has_value() || this->directMode != *prevDirectMode)
      {
#ifdef NAV_UTILS_ROS2
        RCLCPP_INFO(get_logger(), "Detected %s TF copy mode.", *this->directMode ? "direct" : "indirect");
#else
        ROS_INFO("Detected %s TF copy mode.", *this->directMode ? "direct" : "indirect");
#endif
      }
      // don't break the loop here, there could be multiple TFs for the same frame in the message
    }
    else if (tf.child_frame_id == this->parentFrame)
    {
      this->directMode = tf.header.frame_id == this->childFrame;
      if (!prevDirectMode.has_value() || this->directMode != *prevDirectMode)
      {
#ifdef NAV_UTILS_ROS2
        RCLCPP_INFO(get_logger(), "Detected %s TF copy mode.", *this->directMode ? "direct" : "indirect");
#else
        ROS_INFO("Detected %s TF copy mode.", *this->directMode ? "direct" : "indirect");
#endif
      }
      // don't break the loop here, there could be multiple TFs for the same frame in the message
    }
  }

  if (this->directMode.has_value() && *this->directMode)
  {
    for (const auto &tf : msg.transforms)
    {
      if (tf.header.frame_id == this->parentFrame && tf.child_frame_id == this->childFrame)
      {
        this->copyTf(tf);
      }
      else if (tf.header.frame_id == this->childFrame && tf.child_frame_id == this->parentFrame)
      {
        TransformStamped tf2 = tf;
        std::swap(tf2.header.frame_id, tf2.child_frame_id);
        tf2.transform = invertTransform(tf2.transform);
        this->copyTf(tf2);
      }

      // We collect static TFs all the time to be sure we have them when switching from direct mode to indirect
      // We don't need to store dynamic TFs, they should only get collected in indirect mode (we could collect them,
      // but that would be a waste of performance in direct mode).
      if (is_static) {
        this->buffer->setTransform(tf, "tf_static", is_static);
      }
    }
  }
  else
  {
    for (const auto& tf : msg.transforms)
    {
      this->buffer->setTransform(tf, "tf_copy", is_static);
    }
  }
}

void TfCopy::copyTf(const TransformStamped& msg)
{
  auto outMsg = msg;
  outMsg.header.frame_id = this->newParentFrame;
  outMsg.child_frame_id = this->newChildFrame;
  if (this->updateStamp)
  {
#ifdef NAV_UTILS_ROS2
    outMsg.header.stamp = this->now();
#else
    outMsg.header.stamp = ros::Time::now();
#endif
  }

  if (this->invertTf)
    outMsg.transform = invertTransform(outMsg.transform);

  if (this->staticTf)
  {
    this->tfStaticPublisher->sendTransform(outMsg);
    this->lastTransform = msg.transform;
  }
  else
  {
    this->tfPublisher->sendTransform(outMsg);
  }

#ifndef NDEBUG
#ifdef NAV_UTILS_ROS2
  RCLCPP_DEBUG_STREAM(get_logger(), "Published " << this->newParentFrame << "->" << this->newChildFrame);
#else
  ROS_DEBUG_STREAM("Published " << this->newParentFrame << "->" << this->newChildFrame);
#endif
#endif
}

void TfCopy::periodicCb()
{
  if (!this->directMode.has_value()) {
    if (this->errorIfTfMissing) {
#ifdef NAV_UTILS_ROS2
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "%s: Frames %s and %s not seen yet.", get_name(),
        this->parentFrame.c_str(), this->childFrame.c_str());
#else
      ROS_ERROR_THROTTLE(1.0, "%s: Frames %s and %s not seen yet.", ros::this_node::getName().c_str(),
        this->parentFrame.c_str(), this->childFrame.c_str());
#endif
    }
    return;
  }

  // In direct mode, there is nothing to be done periodically.
  if (*this->directMode)
    return;

#ifdef NAV_UTILS_ROS2
  const auto zeroTime = tf2::TimePointZero;
#else
  const auto zeroTime = ros::Time(0, 0);
#endif

  try
  {
    const auto transformMsg = this->buffer->lookupTransform(this->parentFrame, this->childFrame, zeroTime);

    if (this->staticTf && equal(transformMsg.transform, this->lastTransform))
      return;

    this->copyTf(transformMsg);
  }
  catch (const tf2::TransformException& e)
  {
    if (this->errorIfTfMissing) {
#ifdef NAV_UTILS_ROS2
      RCLCPP_ERROR_STREAM_THROTTLE(
        get_logger(), *get_clock(), 1000, get_name() << ": Error getting transform: " << e.what() << std::endl);
#else
      ROS_ERROR_STREAM_THROTTLE(
        1.0, ros::this_node::getName() << ": Error getting transform: " << e.what() << std::endl);
#endif
    }
  }
}

void TfCopy::run()
{
#ifdef NAV_UTILS_ROS2
  if (this->staticTf)
    this->tfStaticPublisher = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
  else
    this->tfPublisher = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  if (!this->staticTf)
  {
    const auto input_tf_topic = this->get_parameter("input_tf_topic").as_string();
    this->tfSub = this->create_subscription<TFMessage>(
      input_tf_topic, tf2_ros::DynamicListenerQoS(), [this](const TFMessage::ConstSharedPtr& msg) {
        this->tfCb(*msg, false);
      }, tf2_ros::detail::get_default_transform_listener_sub_options());
  }

  const auto input_tf_static_topic = this->get_parameter("input_tf_static_topic").as_string();
  this->tfStaticSub = this->create_subscription<TFMessage>(
    input_tf_static_topic, tf2_ros::StaticListenerQoS(), [this](const TFMessage::ConstSharedPtr& msg) {
      this->tfCb(*msg, true);
    }, tf2_ros::detail::get_default_transform_listener_static_sub_options());

  const auto frequency = this->get_parameter("publish_frequency").as_double();

  RCLCPP_INFO_STREAM(get_logger(), "Copying " << (this->invertTf ? "inversion of " : "") <<
    (this->staticTf ? "static " : "") << "transform " << this->parentFrame <<
    "->" << this->childFrame << " as " << this->newParentFrame << "->" <<
    this->newChildFrame << " on " << frequency << " Hz.");

  if (this->publishRate != nullptr) {
    this->publishTimer = this->create_timer(this->publishRate->period(), [this] { this->periodicCb(); });
  }
#else
  if (this->staticTf)
    this->tfStaticPublisher = std::make_unique<tf2_ros::StaticTransformBroadcaster>();
  else
    this->tfPublisher = std::make_unique<tf2_ros::TransformBroadcaster>();

  std::string input_tf_topic = "/tf";
  pnh.param("input_tf_topic", input_tf_topic, input_tf_topic);
  std::string input_tf_static_topic = "/tf_static";
  pnh.param("input_tf_static_topic", input_tf_static_topic, input_tf_static_topic);

  if (!this->staticTf)
  {
    this->tfSub = nh.subscribe<tf2_msgs::TFMessage>(input_tf_topic, 100, [this](const TFMessage::ConstPtr& msg) {
      this->tfCb(*msg, false);
    });
  }

  this->tfStaticSub = nh.subscribe<tf2_msgs::TFMessage>(input_tf_static_topic, 100,
    [this](const TFMessage::ConstPtr& msg) {
      this->tfCb(*msg, true);
    });

  double frequency = 0.0;
  pnh.param("publish_frequency", frequency, 10.0);

  ROS_INFO_STREAM("Copying " << (this->invertTf ? "inversion of " : "") <<
    (this->staticTf ? "static " : "") << "transform " << this->parentFrame <<
    "->" << this->childFrame << " as " << this->newParentFrame << "->" <<
    this->newChildFrame << " on " << frequency << " Hz.");

  ros::Time::waitForValid();
  while (ros::ok())
  {
    if (this->publishRate != nullptr)
    {
      this->publishRate->sleep();
      this->periodicCb();
    }
    else
    {
      ros::WallDuration(1.0).sleep();
    }
  }
#endif
}

}

#ifdef NAV_UTILS_ROS2
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nav_utils::TfCopy);
#endif
