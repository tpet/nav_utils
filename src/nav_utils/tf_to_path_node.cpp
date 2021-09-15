#include <nav_utils/tf_to_path.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "tf_to_path");
  ros::NodeHandle            nh, pnh("~");
  nav_utils::TransformToPath node(nh, pnh);
  ros::spin();
  return 0;
};
