#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include "tf/tf.h"

using namespace std;

bool twist_msg_received = false;

geometry_msgs::TwistStamped twist_cmd_;


void twist_raw_callback(const geometry_msgs::TwistStampedConstPtr &msg)
{
	twist_msg_received = true;
	twist_cmd_ = *msg;
}

int main(int argc, char ** argv)
{	
	ros::init(argc, argv, "twist_convert");
	ros::NodeHandle nh;

	//Subscriber 
    	ros::Subscriber twist_pose_sub = nh.subscribe("twist_cmd", 1, &twist_raw_callback);
    
        //pub
	ros::Publisher pose_pub = nh.advertise<geometry_msgs::Twist>("/base_vel_mux_node/navigation_mode/cmd_vel", 1000);

	ros::Rate loopRate(10);
	while (ros::ok()) {
		if(twist_msg_received == true)
		{
			geometry_msgs::Twist twist;
        		twist.linear.x = twist_cmd_.twist.linear.x;
   			twist.angular.z = twist_cmd_.twist.angular.z;
			//publish amcl_pose_observed
			pose_pub.publish(twist);

			twist_msg_received = false;
		}
		ros::spinOnce();
		loopRate.sleep();
	}

	return 0;
}
