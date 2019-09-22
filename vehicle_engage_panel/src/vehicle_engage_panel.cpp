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

#include <stdio.h>

#include <QHBoxLayout>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QSignalMapper>
#include <QTimer>
#include <QVBoxLayout>
#include <QMap>

#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include "vehicle_engage_panel.h"

namespace autoware_rviz_debug
{

VehicleEngagePanel::VehicleEngagePanel(QWidget* parent) : rviz::Panel(parent)
{
  engage_pub_ = nh_.advertise<std_msgs::Bool>("/vehicle/engage", 1);
  vehicle_status_sub_ = nh_.subscribe("/vehicle_status", 1, &VehicleEngagePanel::vehicleStatusCallback, this);
  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* button_layout = new QHBoxLayout;
  QPushButton* engage_button = new QPushButton("engage");
  QPushButton* disengage_button = new QPushButton("disengage");
  // engage_button->setCheckable(true);
  connect(engage_button, SIGNAL(clicked(bool)), SLOT(sendTrue()));
  connect(disengage_button, SIGNAL(clicked(bool)), SLOT(sendFalse()));
  QVBoxLayout* status_layout = new QVBoxLayout;
  status = new QLabel("NONE");
  status->setAlignment(Qt::AlignCenter);
  status->setStyleSheet("QLabel { background-color : gray;}");
  status_layout->addWidget(status);
  button_layout->addWidget(engage_button);
  button_layout->addWidget(disengage_button);
  
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(button_layout);
  layout->addLayout(status_layout);
  setLayout(layout);
}


void VehicleEngagePanel::sendTopic(const bool &engage)
{
  if (engage)
  {
    std_msgs::Bool msg;
    msg.data = engage;
    engage_pub_.publish(msg);
    if(!engage)  sleep(3);
  }
}

void VehicleEngagePanel::sendTrue()
{
  if (engage_pub_)
  {
    std_msgs::Bool msg;
    msg.data = true;
    engage_pub_.publish(msg);
  }
}

void VehicleEngagePanel::sendFalse()
{
  if (engage_pub_)
  {
    std_msgs::Bool msg;
    msg.data = false;
    engage_pub_.publish(msg);
  }
}

void VehicleEngagePanel::vehicleStatusCallback(const autoware_msgs::VehicleStatusConstPtr& vs)
{
  if(vs->drivemode==1 || vs->steeringmode==1){
    status->setStyleSheet("QLabel { background-color : dodgerblue;}");
    status->setText("AUTO");
  }
  else{
    status->setStyleSheet("QLabel { background-color : lightgreen;}");
    status->setText("MANUAL");
  }
}

void VehicleEngagePanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

void VehicleEngagePanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}

}  // end namespace autoware_rviz_debug

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_debug::VehicleEngagePanel, rviz::Panel)
