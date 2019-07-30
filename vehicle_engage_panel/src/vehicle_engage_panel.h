/*
 * Copyright (c) 2018, TierIV Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef VEHICLE_ENGAGE_PANEL_H
#define VEHICLE_ENGAGE_PANEL_H

#include <QLabel>
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include <autoware_msgs/VehicleStatus.h>
#endif

namespace autoware_rviz_debug
{
class VehicleEngagePanel : public rviz::Panel
{
  Q_OBJECT
public:
  VehicleEngagePanel(QWidget* parent = 0);
  QLabel* status;
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

public Q_SLOTS:
  void sendTopic(const bool &engage);
  void sendTrue();
  void sendFalse();
  void vehicleStatusCallback(const autoware_msgs::VehicleStatusConstPtr& vs);
  // Then we finish up with protected member variables.
protected:
  ros::Publisher engage_pub_;
  ros::NodeHandle nh_;
  ros::Subscriber vehicle_status_sub_;
};

}  // end namespace autoware_rviz_debug

#endif  // VEHICLE_ENGAGE_PANEL_H
