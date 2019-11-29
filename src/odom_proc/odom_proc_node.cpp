#include <odom_proc/odom_proc.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_proc");
    ros::NodeHandle nh;
    odom_proc::OdometryProc node(nh);
    ros::spin();
    return 0;
};
