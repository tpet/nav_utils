#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_utils/PositionInt32.h>

namespace nav_utils
{

void convert(const nav_msgs::Odometry& msg, PositionInt32& out)
{
    typedef PositionInt32::_position_type::_x_type P;
    out.header = msg.header;
    out.child_frame_id = msg.child_frame_id;
    out.position.x = P(msg.pose.pose.position.x * 1000);
    out.position.y = P(msg.pose.pose.position.y * 1000);
    out.position.z = P(msg.pose.pose.position.z * 1000);
}

class OdomToPosition
{
public:
    OdomToPosition(ros::NodeHandle &nh, ros::NodeHandle &pnh):
            nh_(nh),
            pnh_(pnh)
    {
        pub_ = nh_.advertise<PositionInt32>("position", 5);
        sub_ = nh_.subscribe("odom", 5, &OdomToPosition::messageReceived, this);
    }

    void messageReceived(const nav_msgs::Odometry& msg)
    {
        PositionInt32 out;
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
    ros::init(argc, argv, "odom_to_position");
    ros::NodeHandle nh, pnh("~");
    nav_utils::OdomToPosition node(nh, pnh);
    ros::spin();
    return 0;
}
