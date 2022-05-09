#include <fake_rtk_localization/fake_rtk_localization.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "fake_rtk_localization_node");
  ros::NodeHandle nh;
  FakeRTKPose instance(nh);
  ros::spin();
  return 0;
}
