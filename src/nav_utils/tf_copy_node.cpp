#include <nav_utils/tf_copy.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_copy");
    ros::NodeHandle nh, pnh("~");
    nav_utils::TfCopy node(nh, pnh);
    node.run();
    return 0;
};
