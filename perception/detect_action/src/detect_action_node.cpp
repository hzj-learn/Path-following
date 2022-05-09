#include <detect_action/detect_action.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "detect_action_node");
  ros::NodeHandle nh;
  DetectAction instance(nh);
  ros::spin();
  return 0;
}
