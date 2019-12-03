#include <nodelet/nodelet.h>
#include <nav_utils/tf_to_odom.h>
#include <pluginlib/class_list_macros.h>

namespace nav_utils {

class TransformToOdometryNodelet: public nodelet::Nodelet {
private:
    std::unique_ptr<TransformToOdometry> tf_to_odom_;
public:
    void onInit()
    {
        tf_to_odom_ = std::make_unique<TransformToOdometry>(getNodeHandle(), getPrivateNodeHandle());
    }
};

}

PLUGINLIB_EXPORT_CLASS(nav_utils::TransformToOdometryNodelet, nodelet::Nodelet)
