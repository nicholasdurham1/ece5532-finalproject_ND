

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

// defining node variables
const float wradius = 0.15; // wheel radius
const float length = 0.5; // wheel radius
const float width = 0.5; // wheel radius

ros::Publisher WheelSpeedpub;

void DiffCallback(const geometry_msgs::TwistConstPtr& msg)
{
  // msg variables
  double velx = msg -> linear.x;
  double vely = msg -> linear.y;
  double psydotz = msg -> angular.z;
  const double a = 1/wradius;
  const double b = 1/wradius;
  const double c = .5/wradius*(width + length); 

  std_msgs::Float64MultiArray Wheelspeed;
  
  Wheelspeed.data = {
    (velx*a - vely*b - psydotz*c),
    (velx*a + vely*b + psydotz*c),
    (velx*a + vely*b - psydotz*c),
    (velx*a - vely*b + psydotz*c)};// equations need modifed
 // Wheelspeed.data[1,0] = velx*a + vely*b + psydotz*c;
 // Wheelspeed.data[2,0] = velx*a + vely*b - psydotz*c;
 // Wheelspeed.data[3,0] = velx*a - vely*b + psydotz*c;
  
  
  WheelSpeedpub.publish(Wheelspeed);
 

}
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "mecanum_drive");
  ros::NodeHandle node;
  
  
  
  WheelSpeedpub = node.advertise<std_msgs::Float64MultiArray>("/omnibot/wheel_speed_cmd",1);
  
ros::Subscriber sub = node.subscribe("/twist_cmd", 1, DiffCallback);
  
  ros::spin();

  return 0;
} 