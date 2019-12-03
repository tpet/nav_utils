#include <nav_utils/odom_proc.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nav_utils");
    ros::NodeHandle nh, pnh("~");
    nav_utils::OdometryProc node(nh, pnh);
    ros::spin();
    return 0;
};
