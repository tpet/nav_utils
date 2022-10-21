#include <nav_utils/odom_twist_to_child_frame.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nav_utils");
    ros::NodeHandle nh, pnh("~");
    nav_utils::OdometryTwistToChildFrame node(nh, pnh);
    ros::spin();
    return 0;
};
