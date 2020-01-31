#include <thread>

#include <nodelet/nodelet.h>
#include <nav_utils/tf_fast_repeater.h>
#include <pluginlib/class_list_macros.h>

namespace nav_utils {

class TfFastRepeaterNodelet: public nodelet::Nodelet {
private:
    std::unique_ptr<TfFastRepeater> tf_fast_repeater_;
    std::thread thread;
public:
    void onInit() override
    {
        tf_fast_repeater_ = std::make_unique<TfFastRepeater>(getNodeHandle(), getPrivateNodeHandle());
        thread = std::thread([this]{this->tf_fast_repeater_->run();});
    }
};

}

PLUGINLIB_EXPORT_CLASS(nav_utils::TfFastRepeaterNodelet, nodelet::Nodelet)
