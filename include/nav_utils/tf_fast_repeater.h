#ifndef NAV_UTILS_TF_FAST_REPEATER_H
#define NAV_UTILS_TF_FAST_REPEATER_H

#include <memory>

#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

namespace nav_utils
{

class TfFastRepeater
{

public:
  TfFastRepeater(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  virtual ~TfFastRepeater() = default;

  void run();

protected:
  std::unique_ptr<tf2_ros::Buffer> buffer;
  std::unique_ptr<tf2_ros::TransformListener> listener;
  tf2_ros::TransformBroadcaster tfPublisher;

  std::string parentFrame;
  std::string childFrame;
  std::string newParentFrame;
  ros::Rate publishRate;
  ros::Duration errorReportingDeferTime;

  ros::Time timeStarted;

};

}
#endif //NAV_UTILS_TF_FAST_REPEATER_H