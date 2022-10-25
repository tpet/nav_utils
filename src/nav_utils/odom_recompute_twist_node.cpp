#include <nav_utils/odom_recompute_twist.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nav_utils");
    ros::NodeHandle nh, pnh("~");
    nav_utils::OdometryRecomputeTwist node(nh, pnh);
    ros::spin();
    return 0;
};
