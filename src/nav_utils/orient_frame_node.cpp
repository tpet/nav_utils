#include <nav_utils/orient_frame.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "orient_frame");
    ros::NodeHandle nh, pnh("~");
    nav_utils::OrientFrame node(nh, pnh);
    ros::spin();
    return 0;
};
