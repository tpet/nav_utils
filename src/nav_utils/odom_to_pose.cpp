#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_utils/PoseInt32.h>

namespace nav_utils
{

template<typename T>
T clamp(const T& v, const T& lo, const T& hi)
{
    if (v < lo)
        return lo;
    if (hi < v)
        return hi;
    return v;
}

void convert(const nav_msgs::Odometry& msg, PoseInt32& out, bool normalize = false)
{
    typedef PoseInt32::_position_type::_x_type P;
    typedef PoseInt32::_orientation_type::_x_type Q;
    out.header = msg.header;
    out.child_frame_id = msg.child_frame_id;
    out.position.x = P(msg.pose.pose.position.x * 1000);
    out.position.y = P(msg.pose.pose.position.y * 1000);
    out.position.z = P(msg.pose.pose.position.z * 1000);
    const auto max = std::numeric_limits<Q>::max();
    if (normalize)
    {
        const auto& q = msg.pose.pose.orientation;
        const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
        out.orientation.x = Q(msg.pose.pose.orientation.x / norm * max);
        out.orientation.y = Q(msg.pose.pose.orientation.y / norm * max);
        out.orientation.z = Q(msg.pose.pose.orientation.z / norm * max);
        out.orientation.w = Q(msg.pose.pose.orientation.w / norm * max);
    }
    else
    {
        out.orientation.x = Q(clamp(msg.pose.pose.orientation.x, -1.0, 1.0) * max);
        out.orientation.y = Q(clamp(msg.pose.pose.orientation.y, -1.0, 1.0) * max);
        out.orientation.z = Q(clamp(msg.pose.pose.orientation.z, -1.0, 1.0) * max);
        out.orientation.w = Q(clamp(msg.pose.pose.orientation.w, -1.0, 1.0) * max);
    }
}

class OdomToPose
{
public:
    OdomToPose(ros::NodeHandle &nh, ros::NodeHandle &pnh):
            nh_(nh),
            pnh_(pnh),
            normalize_(false)
    {
        pnh_.param("normalize", normalize_, normalize_);
        pub_ = nh_.advertise<PoseInt32>("pose", 5);
        sub_ = nh_.subscribe("odom", 5, &OdomToPose::messageReceived, this);
    }

    void messageReceived(const nav_msgs::Odometry& msg)
    {
        PoseInt32 out;
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
    ros::init(argc, argv, "odom_to_pose");
    ros::NodeHandle nh, pnh("~");
    nav_utils::OdomToPose node(nh, pnh);
    ros::spin();
    return 0;
}
