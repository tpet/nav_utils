#include <thread>

#include <nodelet/nodelet.h>
#include <nav_utils/tf_copy.h>
#include <pluginlib/class_list_macros.h>

namespace nav_utils {

class TfCopyNodelet: public nodelet::Nodelet {
private:
    std::unique_ptr<TfCopy> tf_copy_;
    std::thread thread;
public:
    void onInit() override
    {
        tf_copy_ = std::make_unique<TfCopy>(getNodeHandle(), getPrivateNodeHandle());
        thread = std::thread([this]{this->tf_copy_->run();});
    }
};

}

PLUGINLIB_EXPORT_CLASS(nav_utils::TfCopyNodelet, nodelet::Nodelet)
