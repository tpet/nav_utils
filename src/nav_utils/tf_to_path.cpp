#include <nav_utils/tf_to_path.h>
#include <nav_utils/utils.h>
#include <tf2_eigen/tf2_eigen.h>

namespace nav_utils
{

geometry_msgs::PoseStamped pair_to_pose(const time_point &p) {
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

  pnh.param("parent_frame", frame_parent_);
  pnh.param("child_frame", frame_child_);
  pnh.param("pub_freq", publish_frequency_, 2.0f);

  float sample_distance;
  pnh.param("sample_distance", sample_distance, 0.2f);
  sample_distance_squared_ = sample_distance * sample_distance;

  pub_path_       = nh.advertise<nav_msgs::Path>("path", 1);
  sub_tf_msg_     = nh.subscribe("tf_msg", static_cast<uint32_t>(sub_queue_size_), &TransformToPath::callbackTfMsg, this, ros::TransportHints().tcpNoDelay());
  timer_pub_path_ = nh.createTimer(ros::Rate(publish_frequency_), &TransformToPath::publishTimerCallback, this);


  /* pnh.param("parent_frame", parent_frame_, parent_frame_); */
  /* pnh.param("child_frame", child_frame_, child_frame_); */
  /* pnh.param("no_wait_frame", no_wait_frame_, no_wait_frame_); */
  /* pnh.param("timeout", timeout_, timeout_); */
  /* pnh.param("timeout_relative", timeout_relative_, timeout_relative_); */
  /* pnh.param("timer_freq", timer_freq_, timer_freq_); */
  /* pnh.param("odom_queue_size", odom_queue_size_, odom_queue_size_); */
  /* pnh.param("trigger_queue_size", sub_queue_size_, sub_queue_size_); */
  /* pnh.param("sleep_after_trigger", sleep_after_trigger_, sleep_after_trigger_); */
  /* odom_pub_    = nh.advertise<nav_msgs::Odometry>("odom", static_cast<uint32_t>(odom_queue_size_)); */
  /* tf_          = tf2_client::get_buffer(nh, pnh); */
  /* trigger_sub_ = nh.subscribe("trigger", static_cast<uint32_t>(sub_queue_size_), &TransformToPath::triggerReceived, this); */
  /* if (timer_freq_ > 0.0) { */
  /*   ros::Duration period(1. / timer_freq_); */
  /*   timer_ = nh.createTimer(period, &TransformToPath::timerCallback, this); */
  /* } */
}

/*//{ callbackTfMsg() */
void TransformToPath::callbackTfMsg(const tf2_msgs::TFMessage::ConstPtr &msg) {

  // Accept single-transform msgs only
  if (msg->transforms.size() != 1)
    return;

  const auto &tf_stamped = msg->transforms.at(0);

  // Accept transform with given frames only
  if (tf_stamped.header.frame_id != frame_parent_ || tf_stamped.child_frame_id != frame_child_)
    return;

  const double &secs            = tf_stamped.header.stamp.toSec();
  const auto &  pair_time_point = std::make_pair(secs, transform_to_point(tf_stamped.transform));

  {
    std::scoped_lock lock(mutex_trajectory_);

    trajectory_.insert(pair_time_point);
  }
}
/*//}*/

/*//{ publishTimerCallback() */
void TransformToPath::publishTimerCallback(const ros::TimerEvent &event) {

  if (pub_path_.getNumSubscribers() == 0)
    return;

  {
    std::scoped_lock lock(mutex_trajectory_);
    if (trajectory_.empty())
      return;
  }

  nav_msgs::Path::Ptr path_msg = boost::make_shared<nav_msgs::Path>();

  path_msg->header.frame_id = frame_parent_;
  path_msg->header.stamp    = ros::Time::now();

  // TODO: optimize conversion (no need to regenerate entire path, only from last change from the history)
  path_msg->poses = trajectoryToPath();

  // Do not publish unchanged paths
  if (path_msg->poses.empty() || path_msg->poses.size() == prev_path_size_)
    return;

  try {
    pub_path_.publish(path_msg);
    prev_path_size_ = path_msg->poses.size();
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
    std::scoped_lock lock(mutex_trajectory_);

    auto it_from = trajectory_.begin();
    poses.resize(trajectory_.size());
    poses.at(k++) = pair_to_pose(*it_from);

    if (poses.size() == 1)
      return poses;

    auto it_to = trajectory_.begin();
    it_to++;

    while (it_to != trajectory_.end()) {

      const double norm_sq = squared_norm(*it_from, *it_to);

      // Add sample
      if (norm_sq > sample_distance_squared_) {
        poses.at(k++) = pair_to_pose(*it_to);
        it_from       = it_to;
      }

      it_to++;
    }

    if (k != poses.size())
      poses.resize(k);
  }

  return poses;
}
/*//}*/

}  // namespace nav_utils
