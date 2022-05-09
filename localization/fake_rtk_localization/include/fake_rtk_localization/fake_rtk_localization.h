#ifndef _FAKE_RTK_Localization_
#define _FAKE_RTK_Localization_

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <cmath>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
class FakeRTKPose
{
public:
  FakeRTKPose(ros::NodeHandle nh);
  ~FakeRTKPose() {}
  void run();

private:
  void  _modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr &msg);

  ros::NodeHandle _nh;
  ros::Publisher _fake_RTK_pose_pub, _fake_vel_pub;
  ros::Publisher _vis_linear_vel_pub, _vis_angular_vel_pub;
  ros::Subscriber _sub_robot_pose;

  geometry_msgs::PoseStamped _current_pose;
  geometry_msgs::TwistStamped  _current_velocity;
  std_msgs::Float32 _vis_linear_vel ,  _vis_angular_vel;
  tf::TransformBroadcaster tf_broadcaster_;
};

#endif
