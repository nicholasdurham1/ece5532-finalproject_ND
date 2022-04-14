#include <ros/ros.h>
#include <gtest/gtest.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include "term_color.h"
#include <stdio.h>

ros::NodeHandle* node;
ros::Publisher pub_cmd;
double vx_actual;
double vy_actual;
double pdot_actual;
geometry_msgs::Twist cmd_vel;
bool received_twist = false;

const double v_tolerance = 0.02;
const double pdot_tolerance = 0.07;

void recvTwist(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  vx_actual = msg->twist.linear.x;
  vy_actual = msg->twist.linear.y;
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

TEST(Homework3Test, box_test)
{
  while(!ros::isShuttingDown() && !received_twist){
    ros::Duration(0.02).sleep();
  }

  std::cout << term_color::yellow;
  test_info("Setting velocity to (1.0, 0) m/s...");
  cmd_vel.linear.x = 1.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  timeDelay(2.0);

  EXPECT_NEAR(cmd_vel.linear.x, vx_actual, v_tolerance);
  EXPECT_NEAR(cmd_vel.linear.y, vy_actual, v_tolerance);
  EXPECT_NEAR(cmd_vel.angular.z, pdot_actual, pdot_tolerance);

  test_info("Setting velocity to (0.0, 1.0) m/s...");
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 1.0;
  cmd_vel.angular.z = 0.0;
  timeDelay(2.0);

  EXPECT_NEAR(cmd_vel.linear.x, vx_actual, v_tolerance);
  EXPECT_NEAR(cmd_vel.linear.y, vy_actual, v_tolerance);
  EXPECT_NEAR(cmd_vel.angular.z, pdot_actual, pdot_tolerance);

  test_info("Setting velocity to (-1.0, 0) m/s...");
  cmd_vel.linear.x = -1.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  timeDelay(2.0);

  EXPECT_NEAR(cmd_vel.linear.x, vx_actual, v_tolerance);
  EXPECT_NEAR(cmd_vel.linear.y, vy_actual, v_tolerance);
  EXPECT_NEAR(cmd_vel.angular.z, pdot_actual, pdot_tolerance);

  test_info("Setting velocity to (0, -1.0) m/s...");
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = -1.0;
  cmd_vel.angular.z = 0.0;
  timeDelay(2.0);

  EXPECT_NEAR(cmd_vel.linear.x, vx_actual, v_tolerance);
  EXPECT_NEAR(cmd_vel.linear.y, vy_actual, v_tolerance);
  EXPECT_NEAR(cmd_vel.angular.z, pdot_actual, pdot_tolerance);

  test_info("Stopping vehicle...");
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  timeDelay(2.0);

  EXPECT_NEAR(0.0, vx_actual, v_tolerance);
  EXPECT_NEAR(0.0, vy_actual, v_tolerance);
  EXPECT_NEAR(0.0, pdot_actual, pdot_tolerance);
}

TEST(Homework3Test, diagonal_test)
{
  std::cout << term_color::yellow;
  test_info("Setting velocity to (0.707, 0.707) m/s...");
  cmd_vel.linear.x = 0.707;
  cmd_vel.linear.y = 0.707;
  cmd_vel.angular.z = 0.0;
  timeDelay(2.0);

  EXPECT_NEAR(cmd_vel.linear.x, vx_actual, v_tolerance);
  EXPECT_NEAR(cmd_vel.linear.y, vy_actual, v_tolerance);
  EXPECT_NEAR(cmd_vel.angular.z, pdot_actual, pdot_tolerance);

  test_info("Setting velocity to (0.707, -0.707) m/s...");
  cmd_vel.linear.x = 0.707;
  cmd_vel.linear.y = -0.707;
  cmd_vel.angular.z = 0.0;
  timeDelay(2.0);

  EXPECT_NEAR(cmd_vel.linear.x, vx_actual, v_tolerance);
  EXPECT_NEAR(cmd_vel.linear.y, vy_actual, v_tolerance);
  EXPECT_NEAR(cmd_vel.angular.z, pdot_actual, pdot_tolerance);

  test_info("Setting velocity to (-0.707, -0.707) m/s...");
  cmd_vel.linear.x = -0.707;
  cmd_vel.linear.y = -0.707;
  cmd_vel.angular.z = 0.0;
  timeDelay(2.0);

  EXPECT_NEAR(cmd_vel.linear.x, vx_actual, v_tolerance);
  EXPECT_NEAR(cmd_vel.linear.y, vy_actual, v_tolerance);
  EXPECT_NEAR(cmd_vel.angular.z, pdot_actual, pdot_tolerance);

  test_info("Setting velocity to (-0.707, 0.707) m/s...");
  cmd_vel.linear.x = -0.707;
  cmd_vel.linear.y = 0.707;
  cmd_vel.angular.z = 0.0;
  timeDelay(2.0);

  EXPECT_NEAR(cmd_vel.linear.x, vx_actual, v_tolerance);
  EXPECT_NEAR(cmd_vel.linear.y, vy_actual, v_tolerance);
  EXPECT_NEAR(cmd_vel.angular.z, pdot_actual, pdot_tolerance);

  test_info("Stopping vehicle...");
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  timeDelay(2.0);

  EXPECT_NEAR(0.0, vx_actual, v_tolerance);
  EXPECT_NEAR(0.0, vy_actual, v_tolerance);
  EXPECT_NEAR(0.0, pdot_actual, pdot_tolerance);
}

TEST(Homework3Test, rot_test)
{
  std::cout << term_color::yellow;
  test_info("Setting rotation speed to 0.5 rad/s...");
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.5;
  timeDelay(2.0);

  EXPECT_NEAR(cmd_vel.linear.x, vx_actual, v_tolerance);
  EXPECT_NEAR(cmd_vel.linear.y, vy_actual, v_tolerance);
  EXPECT_NEAR(cmd_vel.angular.z, pdot_actual, pdot_tolerance);

  test_info("Setting rotation speed to -0.5 rad/s...");
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = -0.5;
  timeDelay(2.0);

  EXPECT_NEAR(cmd_vel.linear.x, vx_actual, v_tolerance);
  EXPECT_NEAR(cmd_vel.linear.y, vy_actual, v_tolerance);
  EXPECT_NEAR(cmd_vel.angular.z, pdot_actual, pdot_tolerance);

  test_info("Stopping vehicle...");
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  timeDelay(2.0);

  EXPECT_NEAR(0.0, vx_actual, v_tolerance);
  EXPECT_NEAR(0.0, vy_actual, v_tolerance);
  EXPECT_NEAR(0.0, pdot_actual, pdot_tolerance);
}

TEST(Homework3Test, all_test)
{
  std::cout << term_color::yellow;
  test_info("Setting velocity to (0.5, 0.5) m/s and rotation speed to 0.5 rad/s...");
  cmd_vel.linear.x = 0.5;
  cmd_vel.linear.y = 0.5;
  cmd_vel.angular.z = 0.5;
  timeDelay(5.0);

  EXPECT_NEAR(cmd_vel.linear.x, vx_actual, v_tolerance);
  EXPECT_NEAR(cmd_vel.linear.y, vy_actual, v_tolerance);
  EXPECT_NEAR(cmd_vel.angular.z, pdot_actual, pdot_tolerance);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_ackermann");

  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;

  node = new ros::NodeHandle();

  pub_cmd = node->advertise<geometry_msgs::Twist>("/twist_cmd", 1);
  ros::Subscriber sub_twist = node->subscribe("/omnibot/twist", 1, recvTwist);
  ros::Timer pub_timer = node->createTimer(ros::Duration(0.05), pubCallback);

  ros::AsyncSpinner spinner(3);
  spinner.start();

  int result = RUN_ALL_TESTS();

  spinner.stop();
  node->shutdown();
  delete node;

  return 0;
}
