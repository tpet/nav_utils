#include <nav_utils/odom_to_tf.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_to_tf");
    ros::NodeHandle nh, pnh("~");
    nav_utils::OdometryToTransform node(nh, pnh);
    ros::spin();
    return 0;
};
