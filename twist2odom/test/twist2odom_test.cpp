/*
 *  Copyright (c) 2019 Autoware Foundation, AutonomouStuff
 *  All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file
 * except in compliance with the License. You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include "twist2odom/twist2odom.h"

class Twist2OdomTestSuite : public ::testing::Test
{
public:
  Twist2OdomTestSuite()
  {
    pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped>("current_twist", 1);
    sub_odom_ = nh_.subscribe("current_odom", 1, &Twist2OdomTestSuite::callbackFromOdom, this);
  }

  void callbackFromOdom(const nav_msgs::OdometryConstPtr& msg)
  {
    current_odom_ptr_ = std::make_shared<nav_msgs::Odometry>(*msg);
    ROS_ERROR("Got to the callback!");
  }

  void resetCurrentOdom()
  {
    current_odom_ptr_ = nullptr;
  }

  twist2odom::Twist2Odom t2o_;
  ros::NodeHandle nh_;
  ros::Publisher pub_twist_;
  ros::Subscriber sub_odom_;
  std::shared_ptr<nav_msgs::Odometry> current_odom_ptr_;
};

TEST_F(Twist2OdomTestSuite, simpleTwistPub)
{
  while (true)
  {
    if (pub_twist_.getNumSubscribers() < 1 &&
        sub_odom_.getNumPublishers() < 1)
      ros::Duration(0.1).sleep();
    else
      break;
  }

  geometry_msgs::TwistStamped twist;
  twist.twist.linear.x = 1.0;
  twist.twist.linear.y = 2.0;
  twist.twist.linear.z = 3.0;
  twist.twist.angular.x = 1.0;
  twist.twist.angular.y = 2.0;
  twist.twist.angular.z = 3.0;

  for (int i = 0; i < 10; ++i)
  {
    twist.header.stamp = ros::Time::now();
    pub_twist_.publish(twist);
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  ASSERT_FALSE(current_odom_ptr_ == nullptr);

  ASSERT_FLOAT_EQ(current_odom_ptr_->twist.twist.linear.x, 1.0);
  ASSERT_FLOAT_EQ(current_odom_ptr_->twist.twist.linear.y, 2.0);
  ASSERT_FLOAT_EQ(current_odom_ptr_->twist.twist.linear.z, 3.0);
  ASSERT_FLOAT_EQ(current_odom_ptr_->twist.twist.angular.x, 1.0);
  ASSERT_FLOAT_EQ(current_odom_ptr_->twist.twist.angular.y, 2.0);
  ASSERT_FLOAT_EQ(current_odom_ptr_->twist.twist.angular.z, 3.0);

  resetCurrentOdom();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "Twist2OdomTestSuite");

  return RUN_ALL_TESTS();
}
