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

const double vx_tolerance = 0.25;
const double pdot_tolerance = 0.03;

inline void test_info(const char* message)
{
  std::cout << term_color::yellow;
  puts(message);
  std::cout << term_color::def;
}

inline void setYellow()
{
  std::cout << term_color::yellow;
}

inline void setDefault()
{
  std::cout << term_color::def;
}


void recvTwist(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  vx_actual = msg->twist.linear.x;
  pdot_actual = msg->twist.angular.z;
}

void pubCallback(const ros::TimerEvent& event)
{
  pub_cmd.publish(cmd_vel);
}

TEST(Homework3Test, forward_reverse_test)
{
  std::cout << term_color::yellow;
  test_info("Setting speed to 5.0 m/s...");
  cmd_vel.linear.x = 5.0;
  cmd_vel.angular.z = 0.0;
  int delay_count = 0;
  while (delay_count++ < 200){
    ros::spinOnce();
    ros::Duration(0.025).sleep();
  }

  double vx_diff = cmd_vel.linear.x - vx_actual;
  double pdot_diff = cmd_vel.angular.z - pdot_actual;
  EXPECT_LE(fabs(vx_diff), vx_tolerance);
  EXPECT_LE(fabs(pdot_diff), pdot_tolerance);

  test_info("Stopping vehicle...");
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  delay_count = 0;
  while (delay_count++ < 120){
    ros::spinOnce();
    ros::Duration(0.025).sleep();
  }

  EXPECT_LE(fabs(vx_actual), 0.05);
  EXPECT_LE(fabs(pdot_actual), 0.02);

  test_info("Setting speed to -5.0 m/s...");
  cmd_vel.linear.x = -5.0;
  cmd_vel.angular.z = 0.0;
  delay_count = 0;
  while (delay_count++ < 200){
    ros::spinOnce();
    ros::Duration(0.025).sleep();
  }

  vx_diff = cmd_vel.linear.x - vx_actual;
  pdot_diff = cmd_vel.angular.z - pdot_actual;
  ASSERT_LE(fabs(vx_diff), vx_tolerance);
  ASSERT_LE(fabs(pdot_diff), pdot_tolerance);

  test_info("Stopping vehicle...");
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  delay_count = 0;
  while (delay_count++ < 150){
    ros::spinOnce();
    ros::Duration(0.025).sleep();
  }
}

TEST(Homework3Test, forward_turn_test)
{
  std::cout << term_color::yellow;
  test_info("Forward speed = 5.0 m/s, steering speed = 0.3 rad/s");
  cmd_vel.linear.x = 5.0;
  cmd_vel.angular.z = 0.3;
  int delay_count = 0;
  while (delay_count++ < 160){
    ros::spinOnce();
    ros::Duration(0.025).sleep();
  }
  double vx_diff = cmd_vel.linear.x - vx_actual;
  double pdot_diff = cmd_vel.angular.z - pdot_actual;
  EXPECT_LE(fabs(vx_diff), vx_tolerance);
  EXPECT_LE(fabs(pdot_diff), pdot_tolerance);

  test_info("Forward speed = 5.0 m/s, steering speed = -0.3 rad/s");
  cmd_vel.linear.x = 5.0;
  cmd_vel.angular.z = -0.3;
  delay_count = 0;
  while (delay_count++ < 170){
    ros::spinOnce();
    ros::Duration(0.025).sleep();
  }
  vx_diff = cmd_vel.linear.x - vx_actual;
  pdot_diff = cmd_vel.angular.z - pdot_actual;
  EXPECT_LE(fabs(vx_diff), vx_tolerance);
  EXPECT_LE(fabs(pdot_diff), pdot_tolerance);

  test_info("Stopping vehicle...");
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  delay_count = 0;
  while (delay_count++ < 150){
    ros::spinOnce();
    ros::Duration(0.025).sleep();
  }
  EXPECT_LE(fabs(vx_actual), 0.05);
  EXPECT_LE(fabs(pdot_actual), 0.02);
}

TEST(Homework3Test, reverse_turn_test)
{
  std::cout << term_color::yellow;
  test_info("Forward speed = -5.0 m/s, steering speed = -0.3 rad/s...");
  cmd_vel.linear.x = -5.0;
  cmd_vel.angular.z = -0.3;
  int delay_count = 0;
  while (delay_count++ < 160){
    ros::spinOnce();
    ros::Duration(0.025).sleep();
  }
  double vx_diff = cmd_vel.linear.x - vx_actual;
  double pdot_diff = cmd_vel.angular.z - pdot_actual;
  EXPECT_LE(fabs(vx_diff), vx_tolerance);
  EXPECT_LE(fabs(pdot_diff), pdot_tolerance);

  test_info("Forward speed = -5.0 m/s, steering speed = 0.3 rad/s...");
  cmd_vel.linear.x = -5.0;
  cmd_vel.angular.z = 0.3;
  delay_count = 0;
  while (delay_count++ < 170){
    ros::spinOnce();
    ros::Duration(0.025).sleep();
  }
  vx_diff = cmd_vel.linear.x - vx_actual;
  pdot_diff = cmd_vel.angular.z - pdot_actual;
  EXPECT_LE(fabs(vx_diff), vx_tolerance);
  EXPECT_LE(fabs(pdot_diff), pdot_tolerance);

  test_info("Stopping vehicle...");
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  delay_count = 0;
  while (delay_count++ < 120){
    ros::spinOnce();
    ros::Duration(0.025).sleep();
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_ackermann");

  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;

  node = new ros::NodeHandle();

  pub_cmd = node->advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber sub_twist = node->subscribe("/audibot/twist", 1, recvTwist);
  ros::Timer pub_timer = node->createTimer(ros::Duration(0.05), pubCallback);

  ros::Rate r(ros::Duration(10.0));
  r.sleep();

  int result = RUN_ALL_TESTS();
  delete node;

  return 0;
}
