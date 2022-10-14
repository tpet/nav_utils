#pragma once

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <topic_tools/shape_shifter.h>

namespace nav_utils
{
inline
std_msgs::Header extract_header(const topic_tools::ShapeShifter &msg)
{
    // See https://answers.ros.org/question/226392/extract-header-from-shapeshifter/
    static std::string md5sum = ros::message_traits::md5sum<std_msgs::Header>();
    static std::string datatype = ros::message_traits::datatype<std_msgs::Header>();
    static std::string definition = ros::message_traits::definition<std_msgs::Header>();
    auto &tmp = const_cast<topic_tools::ShapeShifter &>(msg);
    tmp.morph(md5sum, datatype, definition, "");
    std_msgs::Header::Ptr header = tmp.instantiate<std_msgs::Header>();
    return *header;
}

class HeaderSubscriber {
public:
    typedef std::function<void(const std_msgs::Header &)> Callback;
    HeaderSubscriber() = default;
    HeaderSubscriber(const HeaderSubscriber & other) = default;
    HeaderSubscriber(ros::NodeHandle &nh, std::string topic, uint32_t queue_size, Callback callback)
    {
        any_sub_ = nh.subscribe(topic, queue_size, &HeaderSubscriber::anyCallback, this);
        callback_ = callback;
    }
protected:
    ros::Subscriber any_sub_;
    Callback callback_;
    void anyCallback(const topic_tools::ShapeShifter &msg)
    {
        auto header = extract_header(msg);
        callback_(header);
    }
};
}
