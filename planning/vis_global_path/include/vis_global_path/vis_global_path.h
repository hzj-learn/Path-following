#ifndef _VIS_GLOBAL_PATH_
#define _VIS_GLOBAL_PATH_

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Eigen>

class visGoalPath
{
public:
  visGoalPath(ros::NodeHandle nh);
  ~visGoalPath() {}

private:
  void _goalCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
  void _timerCallback(const ros::TimerEvent & event);
  void _draw_marker(std::vector<double> waypoint_x, std::vector<double> waypoint_y);
  void _pub_global_path(std::vector<double> waypoint_x, std::vector<double> waypoint_y);
  void _linear_interpolation(const geometry_msgs::Point p1, const geometry_msgs::Point p2);
  void _insert_points_show( std::vector<double> waypoint_x, std::vector<double>waypoint_y );
  void _pub_turning_points(std::vector<double> waypoint_x, std::vector<double>waypoint_y);
  void _points_process(const geometry_msgs::Pose msg);

  ros::NodeHandle _nh;
  ros::Subscriber _sub_goal;
  ros::Publisher _waypoint_marker_pub;
  ros::Publisher _path_pub;
  ros::Publisher _insert_points;
  ros::Publisher _turning_waypoints_pub;
  
  ros::Timer _timer;

  bool _pub_path_flag, _received_goal_flag;

  std::vector<double>_waypoint_x,_waypoint_y;
  std::vector<double> _path_x, _path_y;
  nav_msgs::Path _global_path;
  geometry_msgs::Point p1, p2;
  geometry_msgs::PoseArray _waypoints_list;
  double record_point_x,  record_point_y;
  double _sampling_rate;
};

#endif
