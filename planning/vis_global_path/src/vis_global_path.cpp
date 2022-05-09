#include <vis_global_path/vis_global_path.h>

visGoalPath::visGoalPath(ros::NodeHandle nh) : _nh(nh)
{
  ros::NodeHandle _pnh("~");
  _pnh.param<double>("sampling_rate", _sampling_rate, 0.05);// 采样间隔

  _sub_goal = nh.subscribe("rtk_waypoints", 10, &visGoalPath::_goalCallback, this);

  _waypoint_marker_pub = _nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
  _insert_points = _nh.advertise<visualization_msgs::MarkerArray>("/insert_points",10);
  _timer = _nh.createTimer(ros::Duration(0.1), &visGoalPath::_timerCallback, this);
   
}

void visGoalPath::_goalCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    _waypoints_list.poses.clear();
    _waypoints_list = *msg;
}

void visGoalPath::_points_process(const geometry_msgs::Pose msg)
{
    geometry_msgs::Pose waypoint_eval;
    waypoint_eval = msg;
    if(_waypoint_x.empty())
    {
            _waypoint_x.push_back(0);
            _waypoint_y.push_back(0);
            record_point_x = 0;
            record_point_y = 0;
    }
    else
    {
        _waypoint_x.push_back(waypoint_eval.position.x);
        _waypoint_y.push_back(waypoint_eval.position.y);
       
        p1.x = record_point_x ; p1.y = record_point_y;
        p2.x = waypoint_eval.position.x; p2.y = waypoint_eval.position.y;      

        record_point_x = waypoint_eval.position.x;
        record_point_y = waypoint_eval.position.y;

        _linear_interpolation(p1, p2);
    }
}

void visGoalPath::_timerCallback(const ros::TimerEvent & event)
{
    _waypoint_x.clear();
    _waypoint_y.clear();
    _path_x.clear();
    _path_y.clear();
    for(int i = 0; i < _waypoints_list.poses.size(); i++){
        _points_process(_waypoints_list.poses[i]);
    }
    _draw_marker(_waypoint_x, _waypoint_y);
    _insert_points_show(_path_x, _path_y);
}

void visGoalPath::_linear_interpolation(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
    int delta_x = abs(p2.x - p1.x);
    int delta_y = abs(p2.y - p1.y);

    int insert_number = 10;

    double k = (p2.y - p1.y)/(p2.x - p1.x ); // 斜率
    double step;

    //_path_x.push_back(p1.x);
    //_path_y.push_back(p1.y);

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

void visGoalPath::_insert_points_show(std::vector<double> waypoint_x, std::vector<double>waypoint_y )
{
    visualization_msgs::MarkerArray  insert_points;
    int length = _path_x.size();
    double yaw;
    for(size_t index=0; index<length; index++) 
    {
        visualization_msgs::Marker waypoint;
        waypoint.header.frame_id = "map";
        waypoint.header.stamp = ros::Time::now();
        waypoint.id = index + 1;
        waypoint.type = visualization_msgs::Marker::ARROW;
        waypoint.action = visualization_msgs::Marker::ADD;
        waypoint.scale.x = 0.2;
        waypoint.scale.y = 0.05;
        waypoint.scale.z = 0.1;
        waypoint.color.r = 1;
        waypoint.color.g = 1;
        waypoint.color.b = 0;
        waypoint.color.a = 1;
        waypoint.pose.position.x = _path_x[index];
        waypoint.pose.position.y = _path_y[index];
        waypoint.pose.position.z = 0;

        if(index  <  _path_x.size()-1)
                yaw = std::atan2(_path_y[index+1] - _path_y[index], _path_x[index+1] - _path_x[index]);
        else
                yaw = yaw;
        
        Eigen::AngleAxisd rollangle(0, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd yawangle(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd pitchangle(0, Eigen::Vector3d::UnitY());
        Eigen::Quaterniond q = rollangle * yawangle * pitchangle;
        waypoint.pose.orientation.x = q.x();
        waypoint.pose.orientation.y = q.y();
        waypoint.pose.orientation.z = q.z();
        waypoint.pose.orientation.w = q.w();

        insert_points.markers.push_back(waypoint);
        }
     _insert_points.publish(insert_points);
}

void visGoalPath::_draw_marker(std::vector<double> waypoint_x, std::vector<double> waypoint_y)
{
   //创建一个 visualization_msgs/Marker消息，并且初始化所有共享的数据。消息成员默认为0，仅仅设置位姿成员w。  
    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/map";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
            
    //分配三个不同的id到三个markers。points_and_lines名称空间的使用确保彼此不会相互冲突。
    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;

    //设置marker类型到 POINTS, LINE_STRIP 和 LINE_LIST
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    // scale成员对于这些marker类型是不同的,POINTS marker分别使用x和y作为宽和高，然而LINE_STRIP和LINE_LIST marker仅仅使用x，定义为线的宽度。单位是米。
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;

    // 点为绿色
    points.color.g = 1.0f;
    points.color.a = 1.0; 

    // Line strip 是蓝色
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list 为红色
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    visualization_msgs::MarkerArray lines, MapPoints;

    for (std::size_t index = 0; index < waypoint_x.size(); index++)
     {
         // line_strip obtains points
          geometry_msgs::Point p, q;
          p.x = waypoint_x[index];
          p.y = waypoint_y[index];
          p.z =0;
          
          points.points.push_back(p);//路径点存储
          line_strip.points.push_back(p);//路径存储
          line_list.points.push_back(p);
          p.z += 1.0;
         line_list.points.push_back(p);
      }
 
       //发布各个markers
        _waypoint_marker_pub.publish(points);
        _waypoint_marker_pub.publish(line_strip);
        _waypoint_marker_pub.publish(line_list);
}
