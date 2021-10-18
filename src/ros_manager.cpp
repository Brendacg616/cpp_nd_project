#include "ros_manager.h"

void ROSManager::Init(int argc, char**argv)
{
  ros::init(argc,argv,"OdometrySimulation");
	ROS_INFO_STREAM("Odometry Simulation");
	ros::NodeHandle nh;
  package_path =ros::package::getPath("cpp_nd_project");
  pubTwist = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
  subOdometry = nh.subscribe("odom", 1, &ROSManager::OdometryCallback, this);
}
string ROSManager::GetPkgPath() {return package_path;}

void ROSManager::OdometryCallback(const nav_msgs::Odometry &odom_msg) {
  odometry = odom_msg;
  ROS_DEBUG_STREAM("Odometry received: " << odometry.pose.pose);
}

void ROSManager::PublishVelocity(const double linear, const double angular)
{
  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x = linear;
  vel_msg.angular.z = angular;
  pubTwist.publish(vel_msg);
}

void ROSManager::GetPosition(double &x, double &y, double &z)
{
  x = odometry.pose.pose.position.x;
  y = odometry.pose.pose.position.y;
  z = odometry.pose.pose.position.z;
}

void ROSManager::Spin()
{
  ros::spinOnce();
  return;
}
