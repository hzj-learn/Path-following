#include <detect_action/detect_action.h>

DetectAction::DetectAction(ros::NodeHandle nh) : _nh(nh)
{
    ros::NodeHandle _pnh("~");
    _pnh.param<double>("robot_length", _robot_length,  1.0);
    _pnh.param<double>("robot_width", _robot_width,  0.5);

    _marker_pub = nh.advertise<visualization_msgs::Marker>("obstacle_marker", 10);
    _robot_footprint_pub = nh.advertise<geometry_msgs::PolygonStamped> ("robot_footprint", 1);
    _text_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("state_display", 1);
    _timer = _nh.createTimer(ros::Duration(0.1), &DetectAction::_timerCallback, this);
}

void DetectAction::_timerCallback(const ros::TimerEvent & event)
{
  // 假设是滴管的位置
    geometry_msgs::Point p1, p2;
    p1.x = 6.8; p1.y = 8.5;
    p2.x = 7; p2.y = -1;

    _obstacle_line_show(p1, p2);// 显示滴管的位置
    caculate_coordinate_points();// 发布矩形框
    _get_rect_pose();// 获取矩形框的位置

    // 判断线段与矩形是否相交
    if(_isLineIntersectRectangle(p1, p2, _rectPoint1, _rectPoint3)){
        _overlayText("Warning");
    }
    else{
        _overlayText("Normal");
    }
    
}

// 获取当前机器人位姿
// void DetectAction::_get_robot_pose()
// {
//         tf::StampedTransform transform;
//        try{listener.lookupTransform("map","base_link", ros::Time(0), transform); } 
//         catch (tf::TransformException ex){ /*ROS_ERROR("Transfrom Failure.");*/ return ; }
//          _robotPoint.x = transform.getOrigin().x();
//          _robotPoint.y = transform.getOrigin().y();
//          _robotPoint.z = 0;
//          //double roll, pitch, yaw;
//         tf::Matrix3x3 m(transform.getRotation());
//          m.getRPY(_robotRoll, _robotPitch, _robotYaw);
//          std::cout << "_robotYaw:" << _robotYaw*180/3.1415926 << std::endl;
//  }

// 获取矩形框左对角线坐标的位置
void DetectAction::_get_rect_pose()
{
    //tf::TransformListener listener1, listener2;
    tf::StampedTransform transform1,  transform3;
    try{listener1.lookupTransform("map","rect_link_1", ros::Time(0), transform1); } 
    catch (tf::TransformException ex){ /*ROS_ERROR("Transfrom Failure.");*/ return ; }
        
    _rectPoint1.x = transform1.getOrigin().x();
    _rectPoint1.y = transform1.getOrigin().y();
    //std::cout << "_rectPoint1:   " << _rectPoint1.x << ", " <<  _rectPoint1.y << std::endl;

    try{listener2.lookupTransform("map","rect_link_3", ros::Time(0), transform3); } 
    catch (tf::TransformException ex){ /*ROS_ERROR("Transfrom Failure.");*/ return ; }
        
    _rectPoint3.x = transform3.getOrigin().x();
    _rectPoint3.y = transform3.getOrigin().y();
    //std::cout << "_rectPoint3:   " << _rectPoint3.x << ", " <<  _rectPoint3.y << std::endl;
}

// 计算矩形轮廓的四个顶点坐标
void  DetectAction::caculate_coordinate_points()
{
  // 四个顶点在局部坐标的位置
    //double angle = atan(_robot_width / _robot_length);
    geometry_msgs::Point point1,  point2, point3, point4;
    static tf::TransformBroadcaster rect_br_1, rect_br_2, rect_br_3, rect_br_4;
    tf::Transform rect_transform_1, rect_transform_2, rect_transform_3, rect_transform_4;

    point1.x = _robot_length / 2 ;  point1.y = _robot_width /2;
    rect_transform_1.setOrigin( tf::Vector3(point1.x,  point1.y, 0.0) ); rect_transform_1.setRotation( tf::Quaternion(0, 0, 0, 1) );
    rect_br_1.sendTransform(tf::StampedTransform(rect_transform_1,ros::Time::now(), "base_link", "rect_link_1"));

    point2.x = _robot_length / 2;  point2.y = - _robot_width /2;
    rect_transform_2.setOrigin( tf::Vector3(point2.x,  point2.y, 0.0) ); rect_transform_2.setRotation( tf::Quaternion(0, 0, 0, 1) );
    rect_br_2.sendTransform(tf::StampedTransform(rect_transform_2,ros::Time::now(), "base_link", "rect_link_2"));

    point3.x = - _robot_length / 2;  point3.y = - _robot_width /2;
    rect_transform_3.setOrigin( tf::Vector3(point3.x,  point3.y, 0.0) ); rect_transform_3.setRotation( tf::Quaternion(0, 0, 0, 1) );
    rect_br_3.sendTransform(tf::StampedTransform(rect_transform_3,ros::Time::now(), "base_link", "rect_link_3"));

    point4.x = - _robot_length / 2;  point4.y = _robot_width /2;
    rect_transform_4.setOrigin( tf::Vector3(point4.x,  point4.y, 0.0) ); rect_transform_4.setRotation( tf::Quaternion(0, 0, 0, 1) );
    rect_br_4.sendTransform(tf::StampedTransform(rect_transform_4,ros::Time::now(), "base_link", "rect_link_4"));
    
    // 坐标旋转+平移变换
    /*
    double angle =  _robotYaw;                                                                                                                                                                                                                                                                                                    ;
    point1.x  =  point1.x * cos(angle) - point1.y * sin(angle) + _robotPoint.x;
    point1.y  =  point1.x * sin(angle) + point1.y * cos(angle) + _robotPoint.y;

    point2.x  = point2.x * cos(angle) - point2.y * sin(angle) + _robotPoint.x;
    point2.y  = point2.x * sin(angle) + point2.y * cos(angle) + _robotPoint.y;
  
    point3.x =  point3.x * cos(angle) - point3.y * sin(angle) + _robotPoint.x;
    point3.y =  point3.x* sin(angle) + point3.y * cos(angle) + _robotPoint.y;

    point4.x =  point4.x * cos(angle) - point4.y * sin(angle) + _robotPoint.x;
    point4.y =  point4.x* sin(angle) + point4.y * cos(angle) + _robotPoint.y;
*/
  // 可视化
    _robot_footprint_show(point1, point2, point3, point4);
}

// 矩形轮廓可视化
void DetectAction::_robot_footprint_show(const geometry_msgs::Point p1, const geometry_msgs::Point p2, const geometry_msgs::Point p3, const geometry_msgs::Point p4)
{
    geometry_msgs::PolygonStamped RobotPolygon;
     RobotPolygon.header.frame_id = "base_link";
    geometry_msgs::Point32 point;

    point.x = p1.x;  point.y = p1.y; 
    RobotPolygon.polygon.points.push_back(point);

    point.x = p2.x;  point.y = p2.y; 
    RobotPolygon.polygon.points.push_back(point);

    point.x = p3.x;  point.y = p3.y; 
    RobotPolygon.polygon.points.push_back(point);

    point.x = p4.x;  point.y = p4.y; 
    RobotPolygon.polygon.points.push_back(point);

    _robot_footprint_pub.publish(RobotPolygon);
}

// 显示可视化滴管的位置（红色）
void DetectAction::_obstacle_line_show(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "map";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "points_and_lines";
     line_list.action = visualization_msgs::Marker::ADD;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.1;
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    
    line_list.points.push_back(p1);
    line_list.points.push_back(p2);
    _marker_pub.publish(line_list);
}

// 状态显示
void DetectAction::_overlayText(std::string str)
{
    jsk_rviz_plugins::OverlayText text;
    text.action = jsk_rviz_plugins::OverlayText::ADD;
    text.width = 170;
    text.height = 50;
    text.left = 0;
    text.top = 0;

    std_msgs::ColorRGBA color1, color2;
    color1.r = 0;
    color1.g = 0;
    color1.b = 0;
    color1.a = 0.4;
    text.bg_color = color1;

    color2.r = 25.0 / 255;
    color2.g = 255.0 / 255;
    color2.b = 240.0 / 255;
    color2.a = 0.8;
    text.fg_color = color2;

    text.line_width = 1;
    text.text_size = 30;
    text.font = "Ubuntu";
    text.text = str;

    _text_pub.publish(text);   
}

// 判断线段与矩形是否相交
bool DetectAction::_isLineIntersectRectangle(geometry_msgs::Point seg1,  geometry_msgs::Point  seg2,
                                      geometry_msgs::Point  rect_point_1,   geometry_msgs::Point  rect_point_3)
    {
        float lineHeight = seg1.y - seg2.y;
        float lineWidth = seg2.x - seg1.x;  
        float c = seg1.x * seg2.y - seg2.x * seg1.y;
        if ((lineHeight * rect_point_1.x + lineWidth * rect_point_1.y + c >= 0 && lineHeight * rect_point_3.x + lineWidth * rect_point_3.y + c <= 0)
            || (lineHeight * rect_point_1.x + lineWidth * rect_point_1.y + c <= 0 && lineHeight * rect_point_3.x + lineWidth * rect_point_3.y + c >= 0)
            || (lineHeight * rect_point_1.x + lineWidth * rect_point_3.y + c >= 0 && lineHeight * rect_point_3.x + lineWidth * rect_point_1.y + c <= 0)
            || (lineHeight * rect_point_1.x + lineWidth * rect_point_3.y + c <= 0 && lineHeight * rect_point_3.x + lineWidth * rect_point_1.y + c >= 0))
        {
 
            if (rect_point_1.x > rect_point_3.x)
            {
                float temp = rect_point_1.x;
                rect_point_1.x = rect_point_3.x;
                rect_point_3.x = temp;
            }
            if (rect_point_1.y < rect_point_3.y)
            {
                float temp1 = rect_point_1.y;
                rect_point_1.y = rect_point_3.y;
                rect_point_3.y = temp1;
            }
            if ((seg1.x < rect_point_1.x && seg2.x < rect_point_1.x)
                || (seg1.x > rect_point_3.x && seg2.x > rect_point_3.x)
                || (seg1.y > rect_point_1.y && seg2.y > rect_point_1.y)
                || (seg1.y < rect_point_3.y && seg2.y < rect_point_3.y))
            {
                return false;
            }
            else
            {
                return true;
            }
        }
        else
        {
            return false;
        }
    }
