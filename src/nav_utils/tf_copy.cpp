#include <nav_utils/tf_copy.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace nav_utils
{

bool equal(const geometry_msgs::Transform& t1, const geometry_msgs::Transform& t2)
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

TfCopy::TfCopy(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  : publishRate(10.0)
{
  pnh.param("parent_frame", this->parentFrame, std::string("map"));
  pnh.param("child_frame", this->childFrame, std::string("odom"));
  pnh.param("new_parent_frame", this->newParentFrame, this->parentFrame + "_copy");
  pnh.param("new_child_frame", this->newChildFrame, this->childFrame + "_copy");

  double frequency = 0.0;
  pnh.param("publish_frequency", frequency, 10.0);
  this->publishRate = ros::Rate(frequency);

  pnh.param("error_if_tf_missing", this->errorIfTfMissing, true);
  pnh.param("invert_tf", this->invertTf, false);
  pnh.param("static_tf", this->staticTf, false);

  this->buffer = tf2_client::get_buffer(nh, pnh);

  ROS_INFO_STREAM("Copying " << (this->invertTf ? "inversion of " : "") <<
    (this->staticTf ? "static " : "") << "transform " << this->parentFrame <<
    "->" << this->childFrame << " as " << this->newParentFrame << "->" <<
    this->newChildFrame << " on " << frequency << " Hz.");
}

void TfCopy::run()
{
  ros::Time::waitForValid();

  const auto zeroTime = ros::Time(0, 0);
  const auto timeout = this->publishRate.expectedCycleTime();

  while (ros::ok())
  {
    this->publishRate.sleep();

    try
    {
      const auto transformMsg = this->buffer->lookupTransform(this->parentFrame, this->childFrame, zeroTime, timeout);

      if (this->staticTf && equal(transformMsg.transform, this->lastTransform))
        continue;

      geometry_msgs::TransformStamped outMsg = transformMsg;
      outMsg.header.frame_id = this->newParentFrame;
      outMsg.child_frame_id = this->newChildFrame;

      if (this->invertTf)
      {
        outMsg.transform = tf2::eigenToTransform(tf2::transformToEigen(outMsg.transform).inverse()).transform;
      }

      if (this->staticTf) {
        this->tfStaticPublisher.sendTransform(outMsg);
        this->lastTransform = transformMsg.transform;
      } else {
        this->tfPublisher.sendTransform(outMsg);
      }
#ifndef NDEBUG
      ROS_DEBUG_STREAM("Published " << this->newParentFrame << "->" << this->newChildFrame);
#endif
    }
    catch (tf2::TransformException& e)
    {
      if (this->errorIfTfMissing)
        ROS_ERROR_STREAM_THROTTLE(1.0, ros::this_node::getName() << ": Error getting transform: " << e.what() << std::endl);
    }
  }
}
}