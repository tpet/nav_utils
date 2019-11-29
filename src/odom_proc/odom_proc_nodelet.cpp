#include <nodelet/nodelet.h>
#include <odom_proc/odom_proc.h>
#include <pluginlib/class_list_macros.h>

namespace odom_proc {

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

PLUGINLIB_EXPORT_CLASS(odom_proc::OdometryProcNodelet, nodelet::Nodelet)
