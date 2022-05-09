#include <global_waypoints_generator/global_waypoints_generator.h>

PathGenerator::PathGenerator(ros::NodeHandle nh) : _nh(nh)
{
  ros::NodeHandle _pnh("~");
  _pnh.param<double>("sampling_rate", _sampling_rate, 0.05);// 采样间隔

  _sub_goal = nh.subscribe("/move_base_simple/goal", 10, &PathGenerator::_goalCallback, this);
  _waypoints_over_flag = nh.subscribe("/nav_over_flag", 10, &PathGenerator::_flagCallback, this);

  _global_waypoints_pub = _nh.advertise<autoware_msgs::Lane>("/base_waypoints", 10);
  _timer = _nh.createTimer(ros::Duration(0.1), &PathGenerator::_timerCallback, this);
}

void PathGenerator::_goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::PoseStamped waypoint_eval;
    waypoint_eval = *msg;

    if(_waypoint_x.size() <  2)
    {
        _waypoint_x.push_back(0);
        _waypoint_y.push_back(0);
        _waypoint_x.push_back(waypoint_eval.pose.position.x);
        _waypoint_y.push_back(waypoint_eval.pose.position.y);

        p1.x = 0 ; p1.y = 0;
        p2.x = waypoint_eval.pose.position.x; p2.y = waypoint_eval.pose.position.y;     
    }
    else
    {
        _waypoint_x.erase(_waypoint_x.begin() + 1, _waypoint_x.end());
        _waypoint_y.erase(_waypoint_y.begin() + 1, _waypoint_y.end());
        _waypoint_x.push_back(waypoint_eval.pose.position.x);
        _waypoint_y.push_back(waypoint_eval.pose.position.y);
       
        p1.x = record_point_x ; p1.y = record_point_y;
        p2.x = waypoint_eval.pose.position.x; p2.y = waypoint_eval.pose.position.y;      
    }
    record_point_x = waypoint_eval.pose.position.x;
    record_point_y = waypoint_eval.pose.position.y;
     _linear_interpolation(p1, p2);
}

void PathGenerator::_flagCallback(const std_msgs::Bool::ConstPtr &msg)
{
    _nav_over_flag = msg->data;
}

void PathGenerator::_timerCallback(const ros::TimerEvent & event)
{
     if(!_nav_over_flag)
        _pub_global_waypoints(_path_x, _path_y);
}

void PathGenerator::_linear_interpolation(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
    int delta_x = abs(p2.x - p1.x);
    int delta_y = abs(p2.y - p1.y);

    int insert_number = 10;

    double k = (p2.y - p1.y)/(p2.x - p1.x ); // 斜率
    double step;
    
    _path_x.clear();
    _path_y.clear();

    if(delta_x >= delta_y)
    {
        if(p1.x == p2.x)
            return;
        if(p1.x > p2.x)
            step = -_sampling_rate;
        else
            step = _sampling_rate;
        
        double x_temp = p1.x + step;
        

        for(; p1.x > p2.x ? x_temp  > p2.x: x_temp < p2.x;  x_temp += step)
        {
            double  y_temp = k * (x_temp - p1.x) + p1.y;
            _path_x.push_back(x_temp);
            _path_y.push_back(y_temp);
        }
    }
    else
    {
        if(p1.y == p2.y)
            return;
        if(p1.y > p2.y)
            step = -_sampling_rate;
        else
            step = _sampling_rate;

        double y_temp = p1.y + step;

        for(; p1.y > p2.y ? y_temp  > p2.y: y_temp < p2.y;  y_temp += step)
        {
            double  x_temp = (y_temp - p1.y )/k + p1.x;
            _path_x.push_back(x_temp);
            _path_y.push_back(y_temp);
        }
    }
}

void PathGenerator::_pub_global_waypoints(std::vector<double> waypoint_x, std::vector<double> waypoint_y)
{
    int length = _path_x.size();
    double yaw;
    autoware_msgs::Lane base_waypoints;

    for(size_t index=0; index < length; index++) {
        autoware_msgs::Waypoint waypoints;
         waypoints.pose.pose.position.x = waypoint_x[index];
         waypoints.pose.pose.position.y = waypoint_y[index];
         waypoints.pose.pose.position.z= 0;

       if(index  <  _path_x.size()-1)
                yaw = std::atan2(_path_y[index+1] - _path_y[index], _path_x[index+1] - _path_x[index]);
        else
                yaw = yaw;
        
        Eigen::AngleAxisd rollangle(0, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd yawangle(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd pitchangle(0, Eigen::Vector3d::UnitY());
        Eigen::Quaterniond q = rollangle * yawangle * pitchangle;

         waypoints.pose.pose.orientation.x = q.x();;
         waypoints.pose.pose.orientation.y = q.y();
         waypoints.pose.pose.orientation.z = q.z();
         waypoints.pose.pose.orientation.w = q.w();

         waypoints.twist.twist.linear .x = 0.2;
         base_waypoints.waypoints.push_back(waypoints);
    }
    _global_waypoints_pub.publish(base_waypoints);
    // std::cout << base_waypoints.waypoints.size() << std::endl;
}
