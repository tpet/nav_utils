#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_utils/PointPathInt32.h>

namespace nav_utils
{
class PathDownsample
{
public:
    PathDownsample(ros::NodeHandle &nh, ros::NodeHandle &pnh):
            nh_(nh),
            pnh_(pnh)
    {
        pnh_.param("skip", skip_, skip_);
        skip_ = skip_ >= 0 ? skip_ : 0;
        pub_ = nh_.advertise<PointPathInt32>("point_path", 5);
        sub_ = nh_.subscribe("path", 5, &PathDownsample::messageReceived, this);
    }

    void messageReceived(const nav_msgs::Path &msg)
    {
        PointPathInt32 out;
        out.header = msg.header;
        out.positions.reserve(msg.poses.size() / (1 + skip_) + 1);
        for (size_t i = 0; i < msg.poses.size() - 1; i += 1 + skip_) {
            PointInt32 p;
            p.x = int(msg.poses[i].pose.position.x * 1000);
            p.y = int(msg.poses[i].pose.position.y * 1000);
            p.z = int(msg.poses[i].pose.position.z * 1000);
            out.positions.push_back(p);
        }
        // Get the last position we missed above if there is any.
        if (!msg.poses.empty())
        {
            PointInt32 p;
            p.x = int(msg.poses.back().pose.position.x * 1000);
            p.y = int(msg.poses.back().pose.position.y * 1000);
            p.z = int(msg.poses.back().pose.position.z * 1000);
            out.positions.push_back(p);
        }
        pub_.publish(out);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    int skip_;
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
