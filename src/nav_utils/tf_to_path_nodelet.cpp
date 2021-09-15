#include <nodelet/nodelet.h>
#include <nav_utils/tf_to_path.h>
#include <pluginlib/class_list_macros.h>

namespace nav_utils
{

class TransformToPathNodelet : public nodelet::Nodelet {
private:
  std::unique_ptr<TransformToPath> tf_to_odom_;

public:
  void onInit() {
    tf_to_odom_ = std::make_unique<TransformToPath>(getNodeHandle(), getPrivateNodeHandle());
  }
};

}  // namespace nav_utils

PLUGINLIB_EXPORT_CLASS(nav_utils::TransformToPathNodelet, nodelet::Nodelet)
