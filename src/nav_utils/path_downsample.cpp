#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_utils/paths.h>
#include <nav_utils/PointPathInt16.h>
#include <nav_utils/PointPathInt32.h>

namespace nav_utils
{

enum PointType
{
    Int8  = 1,
    Int16 = 2,
    Int32 = 3,
    Int64 = 4
};

class PathDownsample
{
public:
    PathDownsample(ros::NodeHandle &nh, ros::NodeHandle &pnh):
            nh_(nh),
            pnh_(pnh),
            skip_(1),
            unit_(0.001),
            type_(Int32)
    {
        pnh_.param("skip", skip_, skip_);
        skip_ = skip_ >= 0 ? skip_ : 0;
        pnh_.param("unit", unit_, unit_);
        pnh_.param("type", type_, type_);
        if (type_ == Int16)
        {
            pub_ = nh_.advertise<PointPathInt16>("point_path", 5);
        }
        else if (type_ == Int32)
        {
            pub_ = nh_.advertise<PointPathInt32>("point_path", 5);
        }
        sub_ = nh_.subscribe("path", 5, &PathDownsample::messageReceived, this);
    }

    template<typename Path>
    void downsampleAndPublish(const nav_msgs::Path &msg)
    {
        Path out;
        downsample(msg, out, skip_, unit_);
        pub_.publish(out);
    }

    void messageReceived(const nav_msgs::Path& msg)
    {
        if (type_ == Int16)
        {
            downsampleAndPublish<PointPathInt16>(msg);
        }
        else if (type_ == Int32)
        {
            downsampleAndPublish<PointPathInt32>(msg);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    int skip_;
    float unit_;
    int type_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_downsample");
    ros::NodeHandle nh, pnh("~");
    nav_utils::PathDownsample node(nh, pnh);
    ros::spin();
    return 0;
}
