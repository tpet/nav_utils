#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_utils/PoseInt32.h>

namespace nav_utils
{

void convert(const PoseInt32& msg, nav_msgs::Odometry& out, bool normalize = false)
{
    typedef PoseInt32::_position_type::_x_type P;
    typedef PoseInt32::_orientation_type::_x_type Q;
    out.header = msg.header;
    out.child_frame_id = msg.child_frame_id;
    out.pose.pose.position.x = double(msg.position.x) / 1000;
    out.pose.pose.position.y = double(msg.position.y) / 1000;
    out.pose.pose.position.z = double(msg.position.z) / 1000;
    if (normalize)
    {
        const auto& q = msg.orientation;
        const double norm = std::sqrt(double(q.x)*q.x + double(q.y)*q.y + double(q.z)*q.z + double(q.w)*q.w);
        out.pose.pose.orientation.x = msg.orientation.x / norm;
        out.pose.pose.orientation.y = msg.orientation.y / norm;
        out.pose.pose.orientation.z = msg.orientation.z / norm;
        out.pose.pose.orientation.w = msg.orientation.w / norm;
    }
    else
    {
        const Q max = std::numeric_limits<Q>::max();
        out.pose.pose.orientation.x = double(msg.orientation.x) / max;
        out.pose.pose.orientation.y = double(msg.orientation.y) / max;
        out.pose.pose.orientation.z = double(msg.orientation.z) / max;
        out.pose.pose.orientation.w = double(msg.orientation.w) / max;
    }
}

class PoseToOdom
{
public:
    PoseToOdom(ros::NodeHandle &nh, ros::NodeHandle &pnh):
            nh_(nh),
            pnh_(pnh)
    {
        pnh_.param("normalize", normalize_, normalize_);
        pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 5);
        sub_ = nh_.subscribe("pose", 5, &PoseToOdom::messageReceived, this);
    }

    void messageReceived(const PoseInt32& msg)
    {
        nav_msgs::Odometry out;
        convert(msg, out, normalize_);
        pub_.publish(out);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    bool normalize_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

};

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_to_odom");
    ros::NodeHandle nh, pnh("~");
    nav_utils::PoseToOdom node(nh, pnh);
    ros::spin();
    return 0;
}
