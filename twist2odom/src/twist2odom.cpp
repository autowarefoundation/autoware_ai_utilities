/*
 *  Copyright (c) 2019 Autoware Foundation, AutonomouStuff
 *  All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file
 * except in compliance with the License. You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 */

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include "twist2odom/twist2odom.h"

using namespace twist2odom;  // NOLINT

Twist2Odom::Twist2Odom() : nh_(), private_nh_("~")
{
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("current_odom", 10);
  twist_sub_ = nh_.subscribe("current_twist", 10, &Twist2Odom::callbackFromTwist, this);
}

void Twist2Odom::callbackFromTwist(const geometry_msgs::TwistStampedConstPtr& msg)
{
  // TODO(jwhitleyastuff): calculate odom.pose.pose by accumulating
  nav_msgs::Odometry odom;
  odom.header = msg->header;
  odom.twist.twist = msg->twist;
  odom_pub_.publish(odom);
}
