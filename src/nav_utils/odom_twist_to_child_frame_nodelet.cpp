#include <nodelet/nodelet.h>
#include <nav_utils/odom_twist_to_child_frame.h>
#include <pluginlib/class_list_macros.h>

namespace nav_utils {

class OdometryTwistToChildFrameNodelet: public nodelet::Nodelet {
private:
    std::unique_ptr<OdometryTwistToChildFrame> odom_twist_to_child_frame_;
public:
    void onInit()
    {
        odom_twist_to_child_frame_ = std::make_unique<OdometryTwistToChildFrame>(
          getNodeHandle(), getPrivateNodeHandle());
    }
};

}

PLUGINLIB_EXPORT_CLASS(nav_utils::OdometryTwistToChildFrameNodelet, nodelet::Nodelet)
