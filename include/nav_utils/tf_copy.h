#ifndef NAV_UTILS_TF_COPY_H
#define NAV_UTILS_TF_COPY_H

#include <memory>

#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <tf2_client/tf2_client.h>

#include <geometry_msgs/Transform.h>

namespace nav_utils
{

class TfCopy
{

public:
  TfCopy(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  virtual ~TfCopy() = default;

  void run();

protected:
  tf2_client::BufferPtr buffer;
  tf2_ros::TransformBroadcaster tfPublisher;
  tf2_ros::StaticTransformBroadcaster tfStaticPublisher;

  geometry_msgs::Transform lastTransform;

  std::string parentFrame;
  std::string childFrame;
  std::string newParentFrame;
  std::string newChildFrame;
  ros::Rate publishRate;
  bool staticTf;
  bool invertTf;
  bool errorIfTfMissing;

};

}
#endif //NAV_UTILS_TF_COPY_H