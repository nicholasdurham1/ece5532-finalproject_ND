#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <ugv_course_libs/gps_conv.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64MultiArray.h>
tf::Vector3 relative_postion;
ros::Publisher pub_markers;
UTMCoords ref_coords;

void markerTimer(const ros::TimerEvent& event){
  
std_msgs::Float64MultiArray waypointlat;
waypointlat.data.resize(8);
waypointlat.data[0] = 42.851358;
waypointlat.data[1] = 42.851383;
waypointlat.data[2] = 42.852443;
waypointlat.data[3] = 42.852021;
waypointlat.data[4] = 42.851525;
waypointlat.data[5] = 42.851344;
waypointlat.data[6] = 42.850836;
waypointlat.data[7] = 42.849644;
std_msgs::Float64MultiArray waypointlon;
waypointlon.data.resize(8);
waypointlon.data[0] = -83.069485;
waypointlon.data[1] = -83.069007;
waypointlon.data[2] = -83.068013;
waypointlon.data[3] = -83.066888;
waypointlon.data[4] = -83.067044;
waypointlon.data[5] = -83.066344;
waypointlon.data[6] = -83.066440;
waypointlon.data[7] = -83.066060;

visualization_msgs::MarkerArray marker_array_msg;
marker_array_msg.markers.resize(8);
visualization_msgs::Marker marker;

  for(int i = 0; i < 8; i++){
  //int i=0;
  LatLon waypoint_coords_lat_lon(waypointlat.data[i], waypointlon.data[i], 0);
  UTMCoords waypoint_coords = UTMCoords(waypoint_coords_lat_lon);
  relative_postion = waypoint_coords - ref_coords;
  

  marker.id = i;
  marker.header.frame_id ="world";
  marker.header.stamp= event.current_real;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 2.0;
  marker.scale.y = 2.0;
  marker.scale.z = 0.3;
  marker.color.r = 1.0;
  marker.color.b = 0.0;
  marker.color.g = 0.0;
  marker.color.a = 1.0;
  marker.pose.position.x = relative_postion.x();
  marker.pose.position.y =relative_postion.y();
  marker.pose.position.z =0.0 ;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z =0.0 ;
  marker.pose.orientation.w =1.0 ;

  marker_array_msg.markers[i] = marker;
  //ROS_INFO("marker created");
  }

pub_markers.publish(marker_array_msg);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Markers");
  ros::NodeHandle node;
 double ref_lat;
  double ref_lon;

  node.getParam("/audibot/gps/ref_lat", ref_lat);
  node.getParam("/audibot/gps/ref_lon", ref_lon);
  
  LatLon ref_coords_lat_lon(ref_lat, ref_lon, 0);
  ref_coords = UTMCoords(ref_coords_lat_lon);
  pub_markers = node.advertise<visualization_msgs::MarkerArray>("markers",1);
  ros::Timer marker_timer = node.createTimer(ros::Duration(0.05),markerTimer);

 ros::spin();
}