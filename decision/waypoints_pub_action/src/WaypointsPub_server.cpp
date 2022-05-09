#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "waypoints_pub_action/WaypointsPubAction.h"
#include "tf/transform_datatypes.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
typedef actionlib::SimpleActionServer<waypoints_pub_action::WaypointsPubAction> Server;

geometry_msgs::Point targetPose_, robotPose_;
double targetYaw_, robotYaw_;
double xy_tolerance_ = 0.1; 
double yaw_tolerance_ = 0.17;// 10度

ros::Subscriber   _sub_robot_pose;
ros::Publisher _pub_twist_cmd;

//  a1相对于a2的夹角
double calDiffBetweenTwoAngle(const double& a1, const double& a2)
{
    double diff = a1 - a2;
    if (diff > M_PI)
        diff = diff - 2.0 * M_PI ;
    return diff;
}

double distance2points(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
    return std::fabs(std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2)));
}

bool closeEnough(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
    double distance;
    distance = distance2points(p1, p2);

    if (distance > xy_tolerance_)
        return false;
    else
        return true;
}

void rotation(double goal_angle)
{
    double angular_speed = 0.1;
    
    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = 0;
    if(goal_angle > 0){
 	    move_cmd.angular.z= angular_speed;
    }
    else if(goal_angle < 0){
 	    move_cmd.angular.z= - angular_speed;
        }
    else{
        move_cmd.angular.z= 0 ;
   	}
    _pub_twist_cmd.publish(move_cmd); 
}

void RobotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    robotPose_ = msg->pose.position;
    
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    robotYaw_ = yaw;
}

// 收到action的goal后调用的回调函数
void execute(const waypoints_pub_action::WaypointsPubGoalConstPtr& goal, Server* as)
{
    // ros::Rate r(1);
    waypoints_pub_action::WaypointsPubFeedback feedback;
    targetPose_.x = goal->x;
    targetPose_.y = goal->y;
    targetPose_.z = 0;
    targetYaw_ = goal->yaw;

    while (ros::ok()){
    double angle = calDiffBetweenTwoAngle(targetYaw_, robotYaw_);
    if(closeEnough(targetPose_, robotPose_) ){
        if(abs(angle) < yaw_tolerance_ ){
        //ROS_INFO("Dishwasher %d finish working.", goal->dishwasher_id);
        as->setSucceeded();
        break;
        }
        else
            rotation(angle);
        
        feedback.state = "Rotation";
        as->publishFeedback(feedback);
    }
    else{
        feedback.state = "Normal";
        as->publishFeedback(feedback);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoints_pub_server");
    ros::NodeHandle nh;

     _sub_robot_pose = nh.subscribe("/current_pose", 10, &RobotPoseCallback);
    _pub_twist_cmd = nh.advertise<geometry_msgs::Twist>("/base_vel_mux_node/rotation_mode/cmd_vel", 10);
	// 定义一个服务器
    Server server(nh, "waypoints_pub", boost::bind(&execute, _1, &server), false);
	
	// 服务器开始运行
    server.start();

    ros::spin();

    return 0;
}
