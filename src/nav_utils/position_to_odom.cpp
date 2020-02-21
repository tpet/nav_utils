#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_utils/PositionInt32.h>

namespace nav_utils
{

void convert(const PositionInt32& msg, nav_msgs::Odometry& out)
{
    typedef PositionInt32::_position_type::_x_type P;
    out.header = msg.header;
    out.child_frame_id = msg.child_frame_id;
    out.pose.pose.position.x = double(msg.position.x) / 1000;
    out.pose.pose.position.y = double(msg.position.y) / 1000;
    out.pose.pose.position.z = double(msg.position.z) / 1000;
    out.pose.pose.orientation.w = 1.0;
}

class PositionToOdom
{
public:
    PositionToOdom(ros::NodeHandle &nh, ros::NodeHandle &pnh):
            nh_(nh),
            pnh_(pnh)
    {
        pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 5);
        sub_ = nh_.subscribe("position", 5, &PositionToOdom::messageReceived, this);
    }

    void messageReceived(const PositionInt32& msg)
    {
        nav_msgs::Odometry out;
        convert(msg, out);
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
    ros::init(argc, argv, "position_to_odom");
    ros::NodeHandle nh, pnh("~");
    nav_utils::PositionToOdom node(nh, pnh);
    ros::spin();
    return 0;
}
