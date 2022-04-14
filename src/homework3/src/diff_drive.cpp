#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

// defining node variables
const float wradius = 0.2; // wheel radius
const float width = 1.0; // vehicle width
  
ros::Publisher DiffLeftpub;
ros::Publisher DiffRightpub;

void DiffCallback(const geometry_msgs::TwistConstPtr& msg)
{
  // msg variables
  double vel = msg -> linear.x;
  double psydot = msg -> angular.z;

  std_msgs::Float64 leftspeed;
  std_msgs::Float64 rightspeed;

  
  leftspeed.data = (1/wradius)*(vel-((width*psydot)/2));
  rightspeed.data = (1/wradius)*(vel+((width*psydot)/2));
  
  DiffLeftpub.publish(leftspeed);
  DiffRightpub.publish(rightspeed);

}
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "diff_drive");
  ros::NodeHandle node;
  
  
  
  DiffLeftpub = node.advertise<std_msgs::Float64>("/roundbot/left_speed_cmd",1);
  DiffRightpub = node.advertise<std_msgs::Float64>("/roundbot/right_speed_cmd",1);
  
  ros::Subscriber sub = node.subscribe("/twist_cmd", 1, DiffCallback);
  ros::spin();

  return 0;
} 