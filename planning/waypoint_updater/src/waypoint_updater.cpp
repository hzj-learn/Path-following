#include  <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/Waypoint.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistStamped.h>

autoware_msgs::Lane base_waypoints_;
autoware_msgs::Lane final_waypoints_;
autoware_msgs::Waypoint robotPose_;

float distance2points_pow(const geometry_msgs::Pose p1, const geometry_msgs::Pose p2)
{
    return pow(p1.position.x - p2.position.x, 2) + pow(p1.position.y - p2.position.y, 2);
}

float pointDistance(const geometry_msgs::Pose p1, const geometry_msgs::Pose p2)
{
    return sqrt((p1.position.x-p2.position.x)*(p1.position.x-p2.position.x) + (p1.position.y-p2.position.y)*(p1.position.y-p2.position.y) + (p1.position.z-p2.position.z)*(p1.position.z-p2.position.z));
}

int getNextClosePointIndex(const autoware_msgs::Lane & trajectory,
    const autoware_msgs::Waypoint &curr_pos)
{
    if (trajectory.waypoints. size() < 2 )
        return 0;
    double dis = 0, min_dis = DBL_MAX;
    int closest_idx = -1;

    for (int i = 0; i < trajectory.waypoints.size(); i++) {
        dis = pointDistance(trajectory.waypoints[i].pose.pose, robotPose_.pose.pose);
        if (dis < min_dis) {
            closest_idx = i;
            min_dis = dis;
        }
    }
    return closest_idx;
}

autoware_msgs::Lane extractTrajectory(const autoware_msgs::Lane & trajectory,
    const autoware_msgs::Waypoint &curr_pos){
    int size = trajectory.waypoints. size();
    if (size <= 2)
        return trajectory;
    
    int min_id = -1;
    float min_dist = 1.5;// 截取前方的距离

    autoware_msgs::Lane extract_trajectory = trajectory;

    double dis = 0, min_dis = DBL_MAX;
    int min_index = -1;

    for (int i = 0; i < size; ++i)
    {
        dis = pointDistance(trajectory.waypoints[i].pose.pose, robotPose_.pose.pose);
        if (dis < min_dis) 
        {
            min_id = i;
            min_dis = dis;
        }
    }
    if (min_id >= 0 )
        extract_trajectory.waypoints.erase(extract_trajectory.waypoints.begin(), extract_trajectory.waypoints.begin() + min_id);
    return extract_trajectory;
}

void current_pose_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    robotPose_.pose = *msg;
}

void _waypointsCall(const autoware_msgs::LaneConstPtr &msg)
{
    base_waypoints_ = *msg;
}

int main(int argc, char ** argv)
{	
	ros::init(argc, argv, "waypoint_updater");
	ros::NodeHandle nh;

	//Subscriber 
    ros::Subscriber global_waypoints_sub_ = nh.subscribe("base_waypoints", 1, &_waypointsCall);
    ros::Subscriber current_pose_sub_ =  nh.subscribe("current_pose", 1, &current_pose_callback);
    
    //Publisher
    ros::Publisher closest_waypoint_pub = nh.advertise<std_msgs::Int32>("/closest_waypoint", 1);// 最近路径点的序号
    ros::Publisher final_waypoint_pub = nh.advertise<autoware_msgs::Lane>("/final_waypoints", 1);// 发布最近路径点之后的路径点信息

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        std_msgs::Int32 closest_id;
        closest_id.data = getNextClosePointIndex(base_waypoints_, robotPose_);
        closest_waypoint_pub.publish(closest_id);

        final_waypoints_ = base_waypoints_;
         final_waypoints_.waypoints.erase(final_waypoints_.waypoints.begin(), final_waypoints_.waypoints.begin() + closest_id.data);
        // std::cout << "final_waypoints_.waypoints.size:   " << final_waypoints_.waypoints.size() << std::endl;

        final_waypoint_pub.publish(final_waypoints_);

        loop_rate.sleep();
        ros::spinOnce();
    }
	return 0;
}
