#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <ugv_course_libs/gps_conv.h>
#include <std_msgs/Float64MultiArray.h>
UTMCoords ref_coords;
tf::Vector3 relative_Vpostion;
tf::Vector3 relative_WPpostion;
nav_msgs::Path gps_path;
ros::Publisher path_pub;
ros::Publisher pub_cmd;
geometry_msgs::Twist cmd_vel;
LatLon current_coords_latlon;
//ros::Publisher CmdVelpub;
//ros::Publisher Steeringpub;
double convangle;
int i;

UTMCoords currentWaypoint_coords;
std_msgs::Float64MultiArray waypointlat;
std_msgs::Float64MultiArray waypointlon;
//std_msgs::Float64 Vheading;
geometry_msgs::TwistStamped Velocity;
// i is current waypoint


void PathCallback(const ros::TimerEvent & event ){
 geometry_msgs::PoseStamped current_pose;
 current_pose.pose.position.x = relative_Vpostion.x();
 current_pose.pose.position.y = relative_Vpostion.y();
 gps_path.poses.push_back(current_pose);
 gps_path.header.frame_id ="world";
 gps_path.header.stamp = event.current_real;
 path_pub.publish(gps_path);
}


void recvFix(const sensor_msgs::NavSatFixConstPtr& msg){
  UTMCoords current_coords(*msg);
  LatLon current_coords_latlon(*msg);
  double CM_ref = ref_coords.getCentralMeridian();
  double vlat = current_coords_latlon.getLat();
  double vlon = current_coords_latlon.getLon();
  double a = (vlon - CM_ref);
  double b = tan(a);
  double c = sin(vlat);
  convangle = atan(b*c);
  relative_Vpostion = current_coords - ref_coords;
}
// heading angle
void recvHead(const std_msgs::Float64ConstPtr& msg){
 std_msgs::Float64 Vheading(*msg);
 //ROS_INFO("Vh %f",Vheading.data);
// waypoint data
waypointlat.data.resize(8);
waypointlat.data[0] = 42.851358;
waypointlat.data[1] = 42.851383;
waypointlat.data[2] = 42.852443;
waypointlat.data[3] = 42.852021;
waypointlat.data[4] = 42.851525;
waypointlat.data[5] = 42.851344;
waypointlat.data[6] = 42.850836;
waypointlat.data[7] = 42.849644;

waypointlon.data.resize(8);
waypointlon.data[0] = -83.069485;
waypointlon.data[1] = -83.069007;
waypointlon.data[2] = -83.068013;
waypointlon.data[3] = -83.066888;
waypointlon.data[4] = -83.067044;
waypointlon.data[5] = -83.066344;
waypointlon.data[6] = -83.066440;
waypointlon.data[7] = -83.066060;
LatLon waypoint_coords_lat_lon(waypointlat.data[i], waypointlon.data[i], 0);
UTMCoords waypoint_coords = UTMCoords(waypoint_coords_lat_lon);
relative_WPpostion = waypoint_coords - ref_coords;

// calcuate heading error and correction 
double VtoWPangle = (atan2((relative_WPpostion.y() - relative_Vpostion.y() ) , (relative_WPpostion.x() - relative_Vpostion.x())));
if (VtoWPangle < 0){
  VtoWPangle = VtoWPangle + 6.28;
}
double distance = sqrt((relative_Vpostion.x()-relative_WPpostion.x())*(relative_Vpostion.x()-relative_WPpostion.x())+(relative_Vpostion.y()-relative_WPpostion.y())*(relative_Vpostion.y()-relative_WPpostion.y()));
ROS_INFO("currentpoint %d,distance %f, angle %f",i, distance, VtoWPangle);
double headingnauticaldeg = Vheading.data;
//ROS_INFO("headingnauticaldeg %f", headingnauticaldeg);
double headingroboticdeg;
if (headingnauticaldeg < 90){
  headingroboticdeg = (90-headingnauticaldeg);
  }else if((headingnauticaldeg <180 )){
  headingroboticdeg = (360-headingnauticaldeg+90);
  }else if((headingnauticaldeg <270 )){
   headingroboticdeg = (360-headingnauticaldeg+90);
  }else if((headingnauticaldeg < 360 )){
  headingroboticdeg = (360-headingnauticaldeg+90);
  }else if((headingnauticaldeg > 360 )){
  ROS_INFO("heading greater than 360");
  }
//double headingroboticrad = headingroboticdeg/57.29 + convangle;
double headingroboticrad = headingroboticdeg/57.29;
ROS_INFO("headingroboticrad %f", headingroboticrad);
ROS_INFO("convangle %f",convangle );

double headingError = (VtoWPangle - headingroboticrad) ;
ROS_INFO("hE %f",headingError);
// if statement for all positive angles



if (abs(headingError) < 3.14/2){
cmd_vel.linear.x =20 + abs(distance/10);
cmd_vel.angular.z = headingError*10;
}else{
cmd_vel.linear.x = 5  ;
cmd_vel.angular.z = 360;}

ROS_INFO("V %f, Z %f",cmd_vel.linear.x,cmd_vel.angular.z );
if (i >= 8){
cmd_vel.linear.x = 0;
cmd_vel.angular.z = 0;
}


pub_cmd.publish(cmd_vel);

if (distance < 1){
ROS_INFO("point complete" );
i ++;
}

}


// velocity and yaw
void recvtwist(const geometry_msgs::TwistStampedConstPtr& msg){
  geometry_msgs::TwistStamped Velocity(*msg);
}

void timeDelay(double t)
{
  ros::Time tic = ros::Time::now();
  while (!ros::isShuttingDown() && (ros::Time::now() - tic).toSec() < t){
    ros::spinOnce();
    ros::Duration(0.02).sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "GpsCode");
  ros::NodeHandle node;
  
  ros::Timer pathtimer = node.createTimer(ros::Duration(0.5), PathCallback);
  ros::Subscriber gps_fix = node.subscribe("/audibot/gps/fix",1,recvFix);
  ros::Subscriber gps_head = node.subscribe("/audibot/gps/heading",1,recvHead);
  ros::Subscriber gps_twist = node.subscribe("/audibot/gps/twist",1,recvtwist);
  //CmdVelpub = node.advertise<geometry_msgs::Twist>("/audibot/cmd_vel",1);
  //Steeringpub = node.advertise<std_msgs::Float64>("/audibot/steering_cmd",1);
  path_pub = node.advertise<nav_msgs::Path>("gps_path",1);
  pub_cmd = node.advertise<geometry_msgs::Twist>("/twist_cmd", 1);
  double ref_lat;
  double ref_lon;

  node.getParam("/audibot/gps/ref_lat", ref_lat);
  node.getParam("/audibot/gps/ref_lon", ref_lon);
  

  LatLon ref_coords_lat_lon(ref_lat, ref_lon, 0);
  ref_coords = UTMCoords(ref_coords_lat_lon);

  
  ros::spin();
}
