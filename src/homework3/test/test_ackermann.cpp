#include <ros/ros.h>
#include <gtest/gtest.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include "term_color.h"
#include <stdio.h>

ros::NodeHandle* node;
ros::Publisher pub_cmd;
double vx_actual;
double pdot_actual;
geometry_msgs::Twist cmd_vel;
bool received_twist = false;

const double vx_tolerance = 0.25;
const double pdot_tolerance = 0.02;

void recvTwist(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  vx_actual = msg->twist.linear.x;
  pdot_actual = msg->twist.angular.z;
  received_twist = true;
}

void pubCallback(const ros::TimerEvent& event)
{
  pub_cmd.publish(cmd_vel);
}

void timeDelay(double t)
{
  ros::Time tic = ros::Time::now();
  while (!ros::isShuttingDown() && (ros::Time::now() - tic).toSec() < t){
    ros::spinOnce();
    ros::Duration(0.02).sleep();
  }
}

TEST(Homework3Test, forward_reverse_test)
{
  while(!ros::isShuttingDown() && !received_twist){
    ros::Duration(0.02).sleep();
  }

  std::cout << term_color::yellow;
  test_info("Setting speed to 5.0 m/s...");
  cmd_vel.linear.x = 5.0;
  cmd_vel.angular.z = 0.0;

  timeDelay(5.0);

  EXPECT_NEAR(cmd_vel.linear.x, vx_actual, vx_tolerance);
  EXPECT_NEAR(cmd_vel.angular.z, pdot_actual, pdot_tolerance);

  test_info("Stopping vehicle...");
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;

  timeDelay(4.0);

  EXPECT_NEAR(0.0, vx_actual, 0.05);
  EXPECT_NEAR(0.0, pdot_actual, 0.02);

  test_info("Setting speed to 15.0 m/s...");
  cmd_vel.linear.x = 15.0;
  cmd_vel.angular.z = 0.0;
  timeDelay(12.0);

  EXPECT_NEAR(cmd_vel.linear.x, vx_actual, vx_tolerance);
  EXPECT_NEAR(cmd_vel.angular.z, pdot_actual, pdot_tolerance);

  test_info("Stopping vehicle...");
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  timeDelay(6.0);
}

TEST(Homework3Test, forward_turn_test)
{
  std::cout << term_color::yellow;
  test_info("Forward speed = 5.0 m/s, steering speed = 0.3 rad/s");
  cmd_vel.linear.x = 5.0;
  cmd_vel.angular.z = 0.3;
  timeDelay(5.0);

  EXPECT_NEAR(cmd_vel.linear.x, vx_actual, vx_tolerance);
  EXPECT_NEAR(cmd_vel.angular.z, pdot_actual, pdot_tolerance);

  test_info("Forward speed = 5.0 m/s, steering speed = -0.3 rad/s");
  cmd_vel.linear.x = 5.0;
  cmd_vel.angular.z = -0.3;

  timeDelay(5.0);

  EXPECT_NEAR(cmd_vel.linear.x, vx_actual, vx_tolerance);
  EXPECT_NEAR(cmd_vel.angular.z, pdot_actual, pdot_tolerance);

  test_info("Stopping vehicle...");
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  timeDelay(4.0);

  EXPECT_NEAR(0.0, vx_actual, 0.05);
  EXPECT_NEAR(0.0, pdot_actual, 0.02);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_ackermann");

  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;

  node = new ros::NodeHandle();

  pub_cmd = node->advertise<geometry_msgs::Twist>("/twist_cmd", 1);
  ros::Subscriber sub_twist = node->subscribe("/audibot/twist", 1, recvTwist);
  ros::Timer pub_timer = node->createTimer(ros::Duration(0.05), pubCallback);

  ros::AsyncSpinner spinner(3);
  spinner.start();

  int result = RUN_ALL_TESTS();

  spinner.stop();
  node->shutdown();
  delete node;

  return 0;
}
