#include <nodelet/nodelet.h>
#include <odom_proc/odom_to_tf.h>
#include <pluginlib/class_list_macros.h>

namespace odom_proc {

class OdometryToTransformNodelet: public nodelet::Nodelet {
private:
    std::unique_ptr<OdometryToTransform> odom_to_tf_;
public:
    void onInit()
    {
        odom_to_tf_ = std::make_unique<OdometryToTransform>(getNodeHandle());
    }
};

}

PLUGINLIB_EXPORT_CLASS(odom_proc::OdometryToTransformNodelet, nodelet::Nodelet)
