#include <nodelet/nodelet.h>
#include <nav_utils/odom_downsample.h>
#include <pluginlib/class_list_macros.h>

namespace nav_utils {

class OdometryDownsampleNodelet: public nodelet::Nodelet {
private:
    std::unique_ptr<OdometryDownsample> odom_downsample_;
public:
    void onInit()
    {
        odom_downsample_ = std::make_unique<OdometryDownsample>(getNodeHandle(), getPrivateNodeHandle());
    }
};

}

PLUGINLIB_EXPORT_CLASS(nav_utils::OdometryDownsampleNodelet, nodelet::Nodelet)
