#include <nav_utils/odom_downsample.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nav_utils");
    ros::NodeHandle nh, pnh("~");
    nav_utils::OdometryDownsample node(nh, pnh);
    ros::spin();
    return 0;
};
