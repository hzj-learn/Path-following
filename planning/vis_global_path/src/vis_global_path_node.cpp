#include <vis_global_path/vis_global_path.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "vis_global_path_node");
  ros::NodeHandle nh;
  visGoalPath instance(nh);
  ros::spin();
  return 0;
}
