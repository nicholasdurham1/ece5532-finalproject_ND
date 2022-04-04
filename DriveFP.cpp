#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
// defining node variables
ros::Publisher CmdVelpub;
ros::Publisher Steeringpub;
double dis_meters;
double followDist = 30;
double vela1;
double vela2;
double orginalcommand =23;


void printCallback(const ros::TimerEvent & event ){
 ROS_INFO("velA1 %f", vela1); 
 ROS_INFO("errordist %f",dis_meters-followDist);
 //ROS_INFO("velA2 %f", vela2);
}
void distCallback(const std_msgs::Float64ConstPtr& msg)
{dis_meters = msg -> data;
//ROS_INFO("DataFromDisCall %f", dis_meters);

}


void A1Callback(const geometry_msgs::TwistStampedConstPtr& msg)
{
  // msg variables
   vela1 = msg -> twist.linear.x;
   double anga1 = msg -> twist.angular.z;
  if (dis_meters > 0){
  
// control via distance
    vela1 = (orginalcommand *0 +vela1*1) + 1*(dis_meters-followDist);


if (vela1 > orginalcommand){
  vela1 = orginalcommand;
}else if (vela1 < 0){
  vela1 = 0;
}
  }
    // print to car
  geometry_msgs::Twist CmdVel;
  CmdVel.angular.z = anga1;
  CmdVel.linear.x = vela1;
 //
  CmdVelpub.publish(CmdVel);
}
void A2Callback(const geometry_msgs::TwistStampedConstPtr& msg)
{
  // msg variables
  vela2 = msg -> twist.linear.x;

}
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "DriveFP");
  ros::NodeHandle node;

  // feed distance
   

  
  CmdVelpub = node.advertise<geometry_msgs::Twist>("/a1/cmd_vel",1);
  ros::Timer writetimer = node.createTimer(ros::Duration(0.001), printCallback);
  ros::Subscriber sub1 = node.subscribe("/a1/twist", 1, A1Callback);
  ros::Subscriber sub2 = node.subscribe("/a2/twist", 1, A2Callback);
  ros::Subscriber sub3 = node.subscribe("/dist", 1, distCallback);





 ros::spin();
  return 0;
} 