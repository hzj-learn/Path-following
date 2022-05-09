#include <fake_rtk_localization/fake_rtk_localization.h>

FakeRTKPose::FakeRTKPose(ros::NodeHandle nh) : _nh(nh)
{
  ros::NodeHandle _pnh("~");

  _sub_robot_pose = nh.subscribe("/gazebo/model_states", 10, &FakeRTKPose::_modelStatesCallback, this);
  _fake_RTK_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1, true);
  _fake_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/current_velocity", 1, true);
  _vis_linear_vel_pub = nh.advertise<std_msgs::Float32>("/vis_linear_velocity", 1, true);
  _vis_angular_vel_pub = nh.advertise<std_msgs::Float32>("/vis_angular_velocity", 1, true);
}

void FakeRTKPose::_modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr &msg)
{
  int modelCount = msg->name.size();

  for(int modelInd = 0; modelInd < modelCount; ++modelInd)
  {
      if(msg->name[modelInd] == "four_ws_robot")
      {
          _current_pose.pose = msg->pose[modelInd];
          _current_velocity.twist = msg->twist[modelInd];
          _vis_linear_vel.data = abs(msg->twist[modelInd].linear.x);
          _vis_angular_vel.data = msg->twist[modelInd].angular.z;
          break;
      }
  }

  // publish tf transformation
  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = ros::Time::now();
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "base_link";
  tf_msg.transform.translation.x = _current_pose.pose.position.x;
  tf_msg.transform.translation.y = _current_pose.pose.position.y;
  tf_msg.transform.translation.z = _current_pose.pose.position.z;

  tf_msg.transform.rotation.x = _current_pose.pose.orientation.x;
  tf_msg.transform.rotation.y = _current_pose.pose.orientation.y;
  tf_msg.transform.rotation.z = _current_pose.pose.orientation.z;
  tf_msg.transform.rotation.w = _current_pose.pose.orientation.w;
  tf_broadcaster_.sendTransform(tf_msg);
  run();
}

void FakeRTKPose::run()
{
  _current_pose.header.stamp = ros::Time::now();
  _fake_RTK_pose_pub.publish(_current_pose);

  _current_velocity.header.stamp = ros::Time::now();
  _fake_vel_pub.publish(_current_velocity);

  _vis_linear_vel_pub.publish(_vis_linear_vel);
  _vis_angular_vel_pub.publish(_vis_angular_vel);
}


