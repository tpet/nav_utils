#include <nav_utils/tf_fast_repeater.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace nav_utils
{

TfFastRepeater::TfFastRepeater(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  : publishRate(10.0)
{
  pnh.param("parent_frame", this->parentFrame, std::string("map"));
  pnh.param("child_frame", this->childFrame, std::string("odom"));
  pnh.param("new_parent_frame", this->newParentFrame, this->parentFrame + "_fast");
  double frequency = 0.0;
  pnh.param("publish_frequency", frequency, 10.0);
  this->publishRate = ros::Rate(frequency);
  double bufferLength = 0.0;
  pnh.param("cache_time", bufferLength, 30.0);
  double errorDeferTime = 0.0;
  pnh.param("error_reporting_defer_time", errorDeferTime, 0.0);
  this->errorReportingDeferTime = ros::Duration(errorDeferTime);

  this->buffer = std::make_unique<tf2_ros::Buffer>(ros::Duration(bufferLength));
  this->listener = std::make_unique<tf2_ros::TransformListener>(*this->buffer);

  ROS_INFO_STREAM("Fast repeating " << this->parentFrame << "->" <<
    this->childFrame << " as " << this->childFrame << "->" << this->newParentFrame
    << " on " << frequency << " Hz.");
}

void TfFastRepeater::run()
{
  ros::Time::waitForValid();
  this->timeStarted = ros::Time::now();

  std::string error;
  const auto timeout = this->publishRate.expectedCycleTime();
  const auto zeroTime = ros::Time(0);

  while (ros::ok())
  {
    this->publishRate.sleep();
    error = "";

    // time jumped back
    if (ros::Time::now() < this->timeStarted)
      this->timeStarted = ros::Time::now();

    try
    {
      if (this->buffer->canTransform(this->parentFrame, this->childFrame, zeroTime, timeout, &error))
      {
        const auto transformMsg = this->buffer->lookupTransform(this->parentFrame, this->childFrame, zeroTime);
        const auto transformE = tf2::transformToEigen(transformMsg);

        // invert the transform since we'll publish it in the opposite direction
        auto transformInvMsg = tf2::eigenToTransform(transformE.inverse());

        // prepare the inverse message and stamp it with current time
        transformInvMsg.header.frame_id = transformMsg.child_frame_id;
        transformInvMsg.child_frame_id = this->newParentFrame;
        transformInvMsg.header.stamp = ros::Time::now();

        this->tfPublisher.sendTransform(transformInvMsg);
      }
      else
      {
        if (this->timeStarted + this->errorReportingDeferTime < ros::Time::now())
          ROS_ERROR_STREAM_THROTTLE(1.0, ros::this_node::getName() << ": Error getting transform: " << error << std::endl);
      }
    }
    catch (tf2::TransformException& e)
    {
      if (this->timeStarted + this->errorReportingDeferTime < ros::Time::now())
        ROS_ERROR_STREAM_THROTTLE(1.0, ros::this_node::getName() << ": Error getting transform: " << e.what() << std::endl);
    }
  }
}
}