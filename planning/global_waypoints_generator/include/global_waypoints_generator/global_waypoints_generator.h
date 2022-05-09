#ifndef _GLOBAL_WAYPOINTS_GENERATOR_
#define _GLOBAL_WAYPOINTS_GENERATOR_

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <autoware_msgs/Waypoint.h>
#include <autoware_msgs/Lane.h>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Eigen>

class PathGenerator
{
public:
  PathGenerator(ros::NodeHandle nh);
  ~PathGenerator() {}

private:
  void _timerCallback(const ros::TimerEvent & event);
  void _pub_global_waypoints(std::vector<double> waypoint_x, std::vector<double> waypoint_y);
  void _goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void _linear_interpolation(const geometry_msgs::Point p1, const geometry_msgs::Point p2);
  void _flagCallback(const std_msgs::Bool::ConstPtr &msg);

  ros::NodeHandle _nh;
  ros::Subscriber _sub_goal;
  ros::Subscriber _waypoints_over_flag;
  ros::Publisher _global_waypoints_pub;
  
  ros::Timer _timer;

  bool _nav_over_flag;

  std::vector<double>_waypoint_x,_waypoint_y;
  std::vector<double> _path_x, _path_y;
  nav_msgs::Path _global_path;
  geometry_msgs::Point p1, p2;
  geometry_msgs::PoseArray _waypoints_list;
  double record_point_x,  record_point_y;
  double _sampling_rate;
};

#endif
