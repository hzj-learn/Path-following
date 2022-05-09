#include <global_waypoints_generator/global_waypoints_generator.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "global_waypoints_generator_node");
  ros::NodeHandle nh;
  PathGenerator instance(nh);
  ros::spin();
  return 0;
}
