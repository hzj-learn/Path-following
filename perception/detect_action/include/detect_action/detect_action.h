#ifndef _DRAW_LINE_PATH_
#define _DRAW_LINE_PATH_

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <cmath>
#include<tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
class DetectAction
{
public:
  DetectAction(ros::NodeHandle nh);
  ~DetectAction() {}

private:
  void _timerCallback(const ros::TimerEvent & event);
  void _robot_footprint_show(const geometry_msgs::Point p1, const geometry_msgs::Point p2, const geometry_msgs::Point p3,  const geometry_msgs::Point p4);
  void _obstacle_line_show(const geometry_msgs::Point p1, const geometry_msgs::Point p2);
  void _get_robot_pose();
  void _get_rect_pose();
  void caculate_coordinate_points();
  bool  _isLineIntersectRectangle( geometry_msgs::Point seg1,   geometry_msgs::Point  seg2,
                                      geometry_msgs::Point  rect_point_1,   geometry_msgs::Point  rect_point_3);
                                      
  void _overlayText(std::string str);
  ros::Timer _timer;

  ros::NodeHandle _nh;
  ros::Publisher _robot_footprint_pub;
  ros::Publisher _target_display_pub;
  ros::Publisher  _marker_pub;
  ros::Publisher  _text_pub;

  tf::TransformListener listener1, listener2;
  double _robot_length,  _robot_width;
  geometry_msgs::Point _robotPoint;
  geometry_msgs::Point _rectPoint1, _rectPoint3;
  double _robotRoll, _robotPitch, _robotYaw;
 
};

#endif
