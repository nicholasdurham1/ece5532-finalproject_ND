#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
// defining node variables
ros::Publisher CmdVelpub;
ros::Publisher Steeringpub;

void DiffCallback(const geometry_msgs::TwistConstPtr& msg)
{
  // msg variables
  double vel = msg -> linear.x;
  double psydot = msg -> angular.z; // update drive angle
  geometry_msgs::Twist CmdVel;
  std_msgs::Float64 Steering;
  double dsteering = psydot;// update drive angle
  Steering.data = dsteering;
  CmdVel.linear.x = vel;
  Steeringpub.publish(Steering);
  CmdVelpub.publish(CmdVel);
}
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "Drive");
  ros::NodeHandle node;
  
  
  CmdVelpub = node.advertise<geometry_msgs::Twist>("/audibot/cmd_vel",1);
  Steeringpub = node.advertise<std_msgs::Float64>("/audibot/steering_cmd",1);
  
  ros::Subscriber sub = node.subscribe("/twist_cmd", 1, DiffCallback);
  ros::spin();

  return 0;
} 