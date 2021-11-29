#include <nav_utils/tf_to_path.h>
#include <nav_utils/utils.h>
#include <tf2_eigen/tf2_eigen.h>

namespace nav_utils
{

geometry_msgs::PoseStamped pair_to_pose_stamped(const time_point &p) {
  geometry_msgs::PoseStamped pose;
  ros::Time                  stamp;
  stamp.fromSec(p.first);

  pose.header.stamp       = stamp;
  pose.pose.position      = p.second;
  pose.pose.orientation.w = 1.f;

  return pose;
}

double squared_norm(const geometry_msgs::Point &point_A, const geometry_msgs::Point &point_B) {
  return std::pow(point_A.x - point_B.x, 2) + std::pow(point_A.y - point_B.y, 2) + std::pow(point_A.z - point_B.z, 2);
}

double squared_norm(const time_point &point_A, const time_point &point_B) {
  return squared_norm(point_A.second, point_B.second);
}

geometry_msgs::Point transform_to_point(const geometry_msgs::Transform &tf) {
  geometry_msgs::Point point;
  point.x = tf.translation.x;
  point.y = tf.translation.y;
  point.z = tf.translation.z;
  return point;
}

TransformToPath::TransformToPath(ros::NodeHandle &nh, ros::NodeHandle &pnh) : sub_queue_size_(5) {

  pnh.getParam("parent_frame", parent_frame_);
  pnh.getParam("child_frame", child_frame_);
  pnh.param("stamp_trigger_frame", stamp_trigger_frame_, child_frame_);

  if (parent_frame_.empty() || child_frame_.empty() || stamp_trigger_frame_.empty()) {
    ROS_ERROR("[TfToPath] Parent and/or child frame is empty, nothing to do.");
    ros::shutdown();
  }

  ROS_INFO("[TfToPath] Parent frame: %s.", parent_frame_.c_str());
  ROS_INFO("[TfToPath] Child frame: %s.", child_frame_.c_str());
  ROS_INFO("[TfToPath] Frame whose direct changes trigger adding a trajectory point: %s.", stamp_trigger_frame_.c_str());

  float sample_distance;
  pnh.param("sample_distance", sample_distance, 0.2f);
  sample_distance_squared_ = sample_distance * sample_distance;
  pnh.param("pub_freq", publish_frequency_, 1.0f);
  lookup_timeout_ = ros::Duration(1.0 / publish_frequency_);

  buffer_ = tf2_client::get_buffer(nh, pnh);

  pub_path_       = nh.advertise<nav_msgs::Path>("path", 1);
  sub_tf_msg_     = nh.subscribe("tf_msg", static_cast<uint32_t>(sub_queue_size_), &TransformToPath::callbackTfMsg, this, ros::TransportHints().tcpNoDelay());
  timer_pub_path_ = nh.createTimer(ros::Rate(publish_frequency_), &TransformToPath::publishTimerCallback, this);
}

/*//{ callbackTfMsg() */
void TransformToPath::callbackTfMsg(const tf2_msgs::TFMessage::ConstPtr &msg) {

  ROS_INFO_ONCE("[TfToPath] Getting first tf msg.");

  // Process the message further only if the child frame is among the specified frames
  bool                            process = false;
  geometry_msgs::TransformStamped tf_valid;
  for (const auto &tf : msg->transforms) {

    if (tf.child_frame_id == stamp_trigger_frame_) {
      tf_valid = tf;
      process  = true;
      break;
    }
  }

  if (!process) {
    return;
  }

  ROS_INFO_ONCE("[TfToPath] First valid stamp trigger tf message received.");

  time_point pair_time_point;

  // Parent frame equal, we already have the transform
  if (tf_valid.header.frame_id == parent_frame_ && tf_valid.child_frame_id == child_frame_) {

    /* ROS_INFO("[TfToPath] no lookup"); */
    pair_time_point = {tf_valid.header.stamp.toSec(), transform_to_point(tf_valid.transform)};
  }
  // Lookup for tf if we don't know the transform parent->child
  else {
    /* ROS_INFO("[TfToPath] lookup"); */

    try {
      const auto tf_msg = buffer_->lookupTransform(parent_frame_, child_frame_, tf_valid.header.stamp, lookup_timeout_);
      pair_time_point = {tf_msg.header.stamp.toSec(), transform_to_point(tf_msg.transform)};
    }
    catch (...) {
      // if we can't transform the point right now, save it for later when its transform is available
      geometry_msgs::Point unknown_point;
      unknown_point.x = unknown_point.y = unknown_point.z = std::numeric_limits<double>::quiet_NaN();
      pair_time_point = {tf_valid.header.stamp.toSec(), unknown_point};
    }
  }

  {
    const std::lock_guard<std::mutex> lock(mutex_trajectory_);
    trajectory_.insert(pair_time_point);
  }
  /* ROS_INFO("[TfToPath] inserted msg"); */
}
/*//}*/

/*//{ publishTimerCallback() */
void TransformToPath::publishTimerCallback(const ros::TimerEvent &event) {

  if (pub_path_.getNumSubscribers() == 0)
    return;

  {
    const std::lock_guard<std::mutex> lock(mutex_trajectory_);
    if (trajectory_.empty())
      return;
  }

  nav_msgs::Path::Ptr path_msg = boost::make_shared<nav_msgs::Path>();

  path_msg->header.frame_id = parent_frame_;
  path_msg->header.stamp    = ros::Time::now();

  // TODO: optimize conversion (no need to regenerate entire path, only from last change from the history)
  path_msg->poses = trajectoryToPath();

  // Do not publish unchanged paths
  if (path_msg->poses.empty() || path_msg->poses == prev_path_.poses) {
    /* ROS_INFO("[TfToPath] won't publish empty path (path size: %ld, prev path size: %d)", path_msg->poses.size(), prev_path_size_); */
    return;
  }

  try {
    pub_path_.publish(path_msg);
    prev_path_.poses = path_msg->poses;
  }
  catch (...) {
    ROS_ERROR("[TfToPath] Exception caught during publishing on topic: %s.", pub_path_.getTopic().c_str());
  }
}
/*//}*/

/*//{ trajectoryToPath() */
std::vector<geometry_msgs::PoseStamped> TransformToPath::trajectoryToPath() {
  std::vector<geometry_msgs::PoseStamped> poses;

  size_t k = 0;
  {
    const std::lock_guard<std::mutex> lock(mutex_trajectory_);

    /* ROS_INFO("[%s]: Trajectory:", ros::this_node::getName().c_str()); */
    /* for (const auto &t_p : trajectory_) { */
    /*   ROS_INFO("[%s]: - (%.1f, %.1f, %.1f)", ros::this_node::getName().c_str(), t_p.second.x, t_p.second.y, t_p.second.z); */
    /* } */

    auto it_from = trajectory_.begin();
    poses.resize(trajectory_.size());

    // find the first point for which we have or can find a transform
    while (it_from != trajectory_.end() && std::isnan(pair_to_pose_stamped(*it_from).pose.position.x))
    {
      try {
        const auto tf_msg = buffer_->lookupTransform(parent_frame_, child_frame_, ros::Time(it_from->first), ros::Duration(0));
        it_from->second = transform_to_point(tf_msg.transform);
        break;
      } catch (...)
      {
        it_from++;
      }
    }
    if (it_from == trajectory_.end())
      return poses;

    poses.at(k++) = pair_to_pose_stamped(*it_from);

    if (poses.size() == 1)
      return poses;

    auto it_to = trajectory_.begin();
    while (it_to != it_from)
      it_to++;
    it_to++;

    while (it_to != trajectory_.end()) {
      // try to find the missing point transform, but do not lose any time in the lookup timeout
      if (std::isnan(pair_to_pose_stamped(*it_to).pose.position.x))
      {
        try {
          const auto tf_msg = buffer_->lookupTransform(parent_frame_, child_frame_, ros::Time(it_to->first), ros::Duration(0));
          it_to->second = transform_to_point(tf_msg.transform);
        } catch (...)
        {
          it_to++;
          continue;
        }
      }
      
      const double norm_sq = squared_norm(*it_from, *it_to);

      // Add sample
      if (norm_sq > sample_distance_squared_) {
        poses.at(k++) = pair_to_pose_stamped(*it_to);
        it_from       = it_to;
      }

      // TODO: we could erase samples actually

      it_to++;
    }

    if (k != poses.size())
      poses.resize(k);
  }

  /* ROS_INFO("[%s]: Poses:", ros::this_node::getName().c_str()); */
  /* for (const auto &p : poses) { */
  /*   ROS_INFO("[%s]: - (%.1f, %.1f, %.1f)", ros::this_node::getName().c_str(), p.pose.position.x, p.pose.position.y, p.pose.position.z); */
  /* } */

  return poses;
}
/*//}*/

}  // namespace nav_utils
