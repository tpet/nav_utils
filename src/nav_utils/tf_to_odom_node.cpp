#include <nav_utils/tf_to_odom.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_to_odom");
    ros::NodeHandle nh, pnh("~");
    nav_utils::TransformToOdometry node(nh, pnh);
    ros::spin();
    return 0;
};
