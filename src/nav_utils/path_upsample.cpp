#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_utils/PointPathInt32.h>

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

    void messageReceived(const PointPathInt32& msg)
    {
        nav_msgs::Path out;
        out.header = msg.header;
        out.poses.reserve(msg.positions.size());
        for (const auto& p: msg.positions)
        {
            geometry_msgs::PoseStamped pose;
            pose.header = msg.header;
            pose.pose.position.x = double(p.x) / 1000;
            pose.pose.position.y = double(p.y) / 1000;
            pose.pose.position.z = double(p.z) / 1000;
            pose.pose.orientation.w = 1.0;
            out.poses.push_back(pose);
        }
        pub_.publish(out);
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
