#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

geometry_msgs::PoseArray RTKPoints;
typedef struct _POSE
{
  double X;
  double Y;
  double Z;
  double or_x;
  double or_y;
  double or_z;
  double or_w;
} POSE;
 
//设置导航的定位点
POSE pose1 = {0.000,  0.000, 0.000,  0.000, 0.000, 0.000, 1.000};

POSE pose2 = {10.00,  0.000,  0.000,  0.000, 0.000, 0.707, 0.707};

POSE pose3 = {10.00,  2.000,  0.000,  0.000, 0.000, 1.000, 0.000};

POSE pose4 = {0.000,  2.000, 0.000,  0.000, 0.000, 0.707, 0.707};

POSE pose5 = {0.000,  4.000, 0.000,  0.000, 0.000, 0.000, 1.000};

POSE pose6 = {10.00,  4.000, 0.000,  0.000, 0.000, 0.707, 0.707};

POSE pose7 = {10.00,  6.000, 0.000,  0.000, 0.000, 1.000, 0.000};

POSE pose8 = {0.000,  6.000, 0.000,  0.000, 0.000, 0.707, 0.707};

POSE pose9 = {0.000,  8.000, 0.000,  0.000, 0.000, 0.000, 1.000};

POSE pose10 ={10.00,  8.000, 0.000,  0.000, 0.000, 0.000, 1.000};


void setGoal(POSE pose)
{
	geometry_msgs::Pose goal;
    goal.position.x = pose.X; 
    goal.position.y = pose.Y;  
    goal.position.z = pose.Z;   
    goal.orientation.x = pose.or_x;
    goal.orientation.y = pose.or_y;
    goal.orientation.z = pose.or_z;
    goal.orientation.w = pose.or_w;

	RTKPoints.header.frame_id = "map";  
    RTKPoints.header.stamp = ros::Time::now();  
	RTKPoints.poses.push_back(goal);   
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pub_rtk_waypoints");
	ros::NodeHandle nh;
    
	//pub
	ros::Publisher rtk_waypoints_pub = nh.advertise<geometry_msgs::PoseArray>("rtk_waypoints", 10);
	ros::Rate loopRate(10);
		
	setGoal(pose1);
	setGoal(pose2);
	setGoal(pose3);
	setGoal(pose4);
	setGoal(pose5);
	setGoal(pose6);
	setGoal(pose7);
	setGoal(pose8);
	setGoal(pose9);
	setGoal(pose10);

	while (ros::ok()) {

		rtk_waypoints_pub.publish(RTKPoints);
		ros::spinOnce();
		loopRate.sleep();
	}

	return 0;
}
