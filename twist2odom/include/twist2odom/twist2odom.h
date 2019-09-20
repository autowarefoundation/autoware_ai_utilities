/*
 *  Copyright (c) 2019 Autoware Foundation, AutonomouStuff
 *  All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file
 * except in compliance with the License. You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef TWIST2ODOM_TWIST2ODOM_H
#define TWIST2ODOM_TWIST2ODOM_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

namespace twist2odom
{

class Twist2Odom
{
public:
  Twist2Odom();
private:
  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber twist_sub_;
  ros::Publisher odom_pub_;

  void callbackFromTwist(const geometry_msgs::TwistStampedConstPtr& msg);
};

}  // namespace twist2odom

#endif  // TWIST2ODOM_TWIST2ODOM_H
