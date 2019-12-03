#include <nodelet/nodelet.h>
#include <nav_utils/odom_to_tf.h>
#include <pluginlib/class_list_macros.h>

namespace nav_utils {

class OdometryToTransformNodelet: public nodelet::Nodelet {
private:
    std::unique_ptr<OdometryToTransform> odom_to_tf_;
public:
    void onInit()
    {
        odom_to_tf_ = std::make_unique<OdometryToTransform>(getNodeHandle(), getPrivateNodeHandle());
    }
};

}

PLUGINLIB_EXPORT_CLASS(nav_utils::OdometryToTransformNodelet, nodelet::Nodelet)
