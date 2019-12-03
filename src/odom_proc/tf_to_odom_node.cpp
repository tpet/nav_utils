#include <odom_proc/tf_to_odom.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_to_odom");
    ros::NodeHandle nh, pnh("~");
    odom_proc::TransformToOdometry node(nh, pnh);
//    ros::spin();
    ros::MultiThreadedSpinner spinner;
    spinner.spin();
//    ros::AsyncSpinner spinner(4);  // Use 4 threads
//    spinner.start();
//    ros::waitForShutdown();
    return 0;
};
