#include <nodelet/nodelet.h>
#include <nav_utils/odom_proc.h>
#include <pluginlib/class_list_macros.h>

namespace nav_utils {

class OdometryProcNodelet: public nodelet::Nodelet {
private:
    std::unique_ptr<OdometryProc> odom_proc_;
public:
    void onInit()
    {
        odom_proc_ = std::make_unique<OdometryProc>(getNodeHandle(), getPrivateNodeHandle());
    }
};

}

PLUGINLIB_EXPORT_CLASS(nav_utils::OdometryProcNodelet, nodelet::Nodelet)
