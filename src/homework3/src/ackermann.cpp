#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
// defining node variables
const float L= 2.65; // 
const float WheelTireRatio = 17.3; // 
  
ros::Publisher CmdVelpub;
ros::Publisher Steeringpub;

void DiffCallback(const geometry_msgs::TwistConstPtr& msg)
{
  // msg variables
  double vel = msg -> linear.x;
  double psydot = msg -> angular.z;

  geometry_msgs::Twist CmdVel;
  std_msgs::Float64 Steering;
  double dsteering = WheelTireRatio*(atan((L*psydot)/vel));
  Steering.data = dsteering;
  CmdVel.linear.x = vel;
  Steeringpub.publish(Steering);
  CmdVelpub.publish(CmdVel);
}
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "diff_drive");
  ros::NodeHandle node;
  
  
  CmdVelpub = node.advertise<geometry_msgs::Twist>("/audibot/cmd_vel",1);
  Steeringpub = node.advertise<std_msgs::Float64>("/audibot/steering_cmd",1);
  
  ros::Subscriber sub = node.subscribe("/twist_cmd", 1, DiffCallback);
  ros::spin();

  return 0;
} 