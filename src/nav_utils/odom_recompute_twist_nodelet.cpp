#include <nodelet/nodelet.h>
#include <nav_utils/odom_recompute_twist.h>
#include <pluginlib/class_list_macros.h>

namespace nav_utils {

class OdometryRecomputeTwistNodelet: public nodelet::Nodelet {
private:
    std::unique_ptr<OdometryRecomputeTwist> odom_recompute_twist_;
public:
    void onInit()
    {
        odom_recompute_twist_ = std::make_unique<OdometryRecomputeTwist>(getNodeHandle(), getPrivateNodeHandle());
    }
};

}

PLUGINLIB_EXPORT_CLASS(nav_utils::OdometryRecomputeTwistNodelet, nodelet::Nodelet)
