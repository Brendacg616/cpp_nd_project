#ifndef ROS_MANAGER_H
#define ROS_MANAGER_H

#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

using namespace std;
class ROSManager
{
  public:
    //ROSManager();
    void Init(int argc, char**argv);
    string GetPkgPath();
    void PublishVelocity (const double linear, const double angular);
    void GetPosition(double &x, double &y, double &z);
    void Spin();
  private:
    void OdometryCallback(const nav_msgs::Odometry &odom_msg);
    ros::Publisher pubTwist;
    ros::Subscriber subOdometry;
    string package_path;
    nav_msgs::Odometry odometry;

};

#endif
