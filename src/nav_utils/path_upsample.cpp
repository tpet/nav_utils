#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_utils/paths.h>
#include <nav_utils/PointPathInt16.h>
#include <nav_utils/PointPathInt32.h>
#include <topic_tools/shape_shifter.h>

namespace nav_utils
{
class PathUpsample
{
public:
    PathUpsample(ros::NodeHandle &nh, ros::NodeHandle &pnh):
            nh_(nh),
            pnh_(pnh)
    {
        pub_ = nh_.advertise<nav_msgs::Path>("path", 5);
        sub_ = nh_.subscribe("point_path", 5, &PathUpsample::messageReceived, this);
    }

    template<typename Path>
    void convertAndPublish(const Path& msg)
    {
        nav_msgs::Path out;
        upsample(msg, out);
        pub_.publish(out);
    }

    void messageReceived(const topic_tools::ShapeShifter::ConstPtr& msg)
    {
        if (msg->getDataType() == "nav_utils/PointPathInt16")
        {
            convertAndPublish(*msg->instantiate<PointPathInt16>());
        }
        else if (msg->getDataType() == "nav_utils/PointPathInt32")
        {
            convertAndPublish(*msg->instantiate<PointPathInt32>());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

};

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_upsample");
    ros::NodeHandle nh, pnh("~");
    nav_utils::PathUpsample node(nh, pnh);
    ros::spin();
    return 0;
}
