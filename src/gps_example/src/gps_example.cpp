#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "gps_example");
  ros::NodeHandle node;
  

  ros::spin();

  return 0;
} 