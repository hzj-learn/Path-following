#include <ros/ros.h>  
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

ros::Subscriber laser_sub_;
ros::Publisher vel_pub_;

void laserCallback(const sensor_msgs::LaserScanConstPtr &data)
{
  sensor_msgs::LaserScan send_data = *data;
  int data_number = data->ranges.size();

  for(int i = 0;i < data_number; i++)
  {
      if(std::isnan(send_data.ranges[i]))
      {
          send_data.ranges[i] = 0;
      }

      if(std::isinf(send_data.ranges[i]))
      {
          send_data.ranges[i] = 0;
      }

      if(send_data.ranges[i]>0 && send_data.ranges[i] <= 0.5)
      {
        geometry_msgs::Twist move_cmd;
	      move_cmd.linear.x = 0.0; 
	      move_cmd.angular.z = 0.0;
        vel_pub_.publish(move_cmd);
      }
  }
}
 
int main(int argc, char **argv)  
{  
    // 初始化ROS节点
    ros::init(argc,argv,"laser_detection_node");
    ros::NodeHandle nh;

    laser_sub_ = nh.subscribe("/front/scan", 100, &laserCallback);
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("/base_vel_mux_node/stop_mode/cmd_vel", 100);

    ros::spin();
    return 0;  
} 
