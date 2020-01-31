#include <nav_utils/tf_fast_repeater.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_fast_repeater");
    ros::NodeHandle nh, pnh("~");
    nav_utils::TfFastRepeater node(nh, pnh);
    node.run();
    return 0;
};
