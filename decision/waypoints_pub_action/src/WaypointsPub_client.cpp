#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include "waypoints_pub_action/WaypointsPubAction.h"
#include "tf/transform_datatypes.h"

typedef actionlib::SimpleActionClient<waypoints_pub_action::WaypointsPubAction> Client;

ros::Publisher _pub_goal, _pub_falg;
// 当action完成后会调用次回调函数一次
void doneCb(const actionlib::SimpleClientGoalState& state,
        const waypoints_pub_action::WaypointsPubResultConstPtr& result)
{
    ROS_INFO("Goal Reached !!!"); 
    //ros::shutdown();
}

// // 当action激活后会调用次回调函数一次
void activeCb()
{
    ROS_INFO("Sending Goal !!!");
}


// 收到feedback后调用的回调函数
void feedbackCb(const waypoints_pub_action::WaypointsPubFeedbackConstPtr& feedback)
{
    //ROS_INFO(" Mode : %s ", feedback->state.c_str());
}


void setGoal(const geometry_msgs::Pose pose_msg){
	// 定义一个客户端
    Client client("waypoints_pub", true);

    // 等待服务器
    while(!client.waitForServer(ros::Duration(5.0))){  
        ROS_WARN("Waiting for the move_base action server to come up");  
    } 
	// 创建一个action的goal
    waypoints_pub_action::WaypointsPubGoal goal;
    goal.x = pose_msg.position.x;
    goal.y = pose_msg.position.y;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(pose_msg.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    goal.yaw = yaw;

    // 发送action的goal给服务器端，并且设置回调函数
    client.sendGoal(goal,  &doneCb, &activeCb, &feedbackCb);

    // 等待结果 
    client.waitForResult();  
}

// 目标点回调
// void GoalPoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg){
//     setGoal(*msg);
// }

void GoalPoseCB(const geometry_msgs::PoseArray::ConstPtr &msg){
    for(int i =0; i< msg->poses.size(); i++){
        geometry_msgs::PoseStamped goals;
        goals.header.frame_id = "map";
        goals.header.stamp = ros::Time::now();
        goals.pose = msg->poses[i];
        _pub_goal.publish(goals);

        setGoal(msg->poses[i]);
    }
    std_msgs::Bool flag;
    flag.data = true;
    _pub_falg.publish(flag);
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoints_pub_client");
    ros::NodeHandle nh;
    ros::Subscriber sub_goal = nh.subscribe("/rtk_waypoints", 10, &GoalPoseCB);
    _pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    _pub_falg = nh.advertise<std_msgs::Bool>("/nav_over_flag", 10);
    ros::spin();
    return 0;
}
