#include <nodelet/nodelet.h>
#include <odom_proc/tf_to_odom.h>
#include <pluginlib/class_list_macros.h>

namespace odom_proc {

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

PLUGINLIB_EXPORT_CLASS(odom_proc::TransformToOdometryNodelet, nodelet::Nodelet)
