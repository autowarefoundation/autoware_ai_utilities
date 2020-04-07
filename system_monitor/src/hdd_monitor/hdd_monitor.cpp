/*
 * Copyright 2020 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file hdd_monitor.cpp
 * @brief HDD monitor class
 */

#include <algorithm>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/process.hpp>
#include <boost/regex.hpp>
#include <system_monitor/hdd_monitor/hdd_monitor.h>

namespace bp = boost::process;
namespace fs = boost::filesystem;

HDDMonitor::HDDMonitor(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh), pnh_(pnh)
{
  gethostname(hostname_, sizeof(hostname_));

  // Check if command exists
  fs::path p = bp::search_path("smartctl");
  smartctl_exists_ = (p.empty()) ? false : true;

  getTempParams();
  pnh_.param<float>("usage_warn", usage_warn_, 0.9);
  pnh_.param<float>("usage_error", usage_error_, 1.1);

  updater_.setHardwareID(hostname_);
  updater_.add("HDD Temperature", this, &HDDMonitor::checkTemp);
  updater_.add("HDD Usage", this, &HDDMonitor::checkUsage);
}

void HDDMonitor::run(void)
{
  ros::Rate rate(1.0);

  while (ros::ok())
  {
    ros::spinOnce();
    updater_.force_update();
    rate.sleep();
  }
}

void HDDMonitor::checkTemp(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (temp_params_.empty())
  {
    stat.summary(DiagStatus::ERROR, "invalid disk parameter");
    return;
  }

  if (!smartctl_exists_)
  {
    stat.summary(DiagStatus::ERROR, "smartctl error");
    stat.add("smartctl", "Command 'smartctl' not found, but can be installed with: sudo apt install smartmontools");
    return;
  }

  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;
  int index = 0;
  std::string error_str = "";

  boost::smatch match;
  const boost::regex fmodel_number("^Model Number:\\s+(.*)");
  const boost::regex fdevice_model("^Device Model:\\s+(.*)");
  const boost::regex fserial_number("^Serial Number:\\s+(.*)");
  const boost::regex ftemperature("^Temperature:\\s+(.*)");
  const boost::regex ftemperature_c(".*Temperature_Celsius.*\\s(\\d+)$");

  for (auto itr = temp_params_.begin(); itr != temp_params_.end(); ++itr, ++index)
  {
    // Get SMART information about the disk
    bp::ipstream is_out;
    bp::ipstream is_err;
    bp::child c((boost::format("sudo smartctl -a %1%") % itr->first).str(), bp::std_out > is_out, bp::std_err > is_err);
    c.wait();
    if (c.exit_code() != 0)
    {
      std::ostringstream os;
      is_err >> os.rdbuf();
      stat.add((boost::format("HDD %1%: status") % index).str(), "smartctl error");
      stat.add((boost::format("HDD %1%: name") % index).str(), itr->first.c_str());
      stat.add((boost::format("HDD %1%: smartctl") % index).str(), os.str().c_str());
      error_str = "smartctl error";
      continue;
    }

    std::string line;
    std::string model;
    std::string serial;
    float temp;

    while (std::getline(is_out, line))
    {
      // INFORMATION SECTION: Model Number(NVMe) or Device Model(ATA)
      if (boost::regex_match(line, match, fmodel_number) || boost::regex_match(line, match, fdevice_model))
        model = match[1].str();
      // INFORMATION SECTION: Serial Number
      else if (boost::regex_match(line, match, fserial_number)) serial = match[1].str();
      // INFORMATION SECTION: Temperature(NVMe)
      else if (boost::regex_match(line, match, ftemperature)) temp = std::atof(match[1].str().c_str());
      // SMART/Health Information: Temperature_Celsius RAW_VALUE(ATA)
      else if (boost::regex_match(line, match, ftemperature_c)) temp = std::atof(match[1].str().c_str());
    }

    level = DiagStatus::OK;
    if (temp >= itr->second.temp_error_) level = DiagStatus::ERROR;
    else if (temp >= itr->second.temp_warn_) level = DiagStatus::WARN;

    stat.add((boost::format("HDD %1%: status") % index).str(), temp_dict_.at(level));
    stat.add((boost::format("HDD %1%: name") % index).str(), itr->first.c_str());
    stat.add((boost::format("HDD %1%: model") % index).str(), model.c_str());
    stat.add((boost::format("HDD %1%: serial") % index).str(), serial.c_str());
    stat.addf((boost::format("HDD %1%: temperature") % index).str(), "%.1f DegC", temp);

    whole_level = std::max(whole_level, level);
  }

  if (!error_str.empty())
    stat.summary(DiagStatus::ERROR, error_str);
  else
    stat.summary(whole_level, temp_dict_.at(whole_level));
}

void HDDMonitor::checkUsage(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  // Get summary of disk space usage of ext4
  bp::ipstream is_out;
  bp::ipstream is_err;
  bp::child c("df -Pht ext4", bp::std_out > is_out, bp::std_err > is_err);
  c.wait();
  if (c.exit_code() != 0)
  {
    std::ostringstream os;
    is_err >> os.rdbuf();
    stat.summary(DiagStatus::ERROR, "df error");
    stat.add("df", os.str().c_str());
    return;
  }

  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;

  std::string line;
  int index = 0;
  std::vector<std::string> list;
  float usage;

  while (std::getline(is_out, line) && !line.empty())
  {
    // Skip header
    if ( index <= 0 )
    {
      ++index;
      continue;
    }

    boost::split(list, line, boost::is_space(), boost::token_compress_on);

    usage = std::atof(boost::trim_copy_if(list[4], boost::is_any_of("%")).c_str())*1e-2;

    level = DiagStatus::OK;
    if (usage >= usage_error_) level = DiagStatus::ERROR;
    else if (usage >= usage_warn_) level = DiagStatus::WARN;

    stat.add((boost::format("HDD %1%: status") % (index-1)).str(), usage_dict_.at(level));
    stat.add((boost::format("HDD %1%: filesystem") % (index-1)).str(), list[0].c_str());
    stat.add((boost::format("HDD %1%: size") % (index-1)).str(), list[1].c_str());
    stat.add((boost::format("HDD %1%: used") % (index-1)).str(), list[2].c_str());
    stat.add((boost::format("HDD %1%: avail") % (index-1)).str(), list[3].c_str());
    stat.add((boost::format("HDD %1%: use") % (index-1)).str(), list[4].c_str());
    stat.add((boost::format("HDD %1%: mounted on") % (index-1)).str(), list[5].c_str());

    whole_level = std::max(whole_level, level);
    ++index;
  }

  stat.summary(whole_level, usage_dict_.at(whole_level));
}

void HDDMonitor::getTempParams(void)
{
  XmlRpc::XmlRpcValue params;

  pnh_.getParam("disks", params);
  if (params.getType() != XmlRpc::XmlRpcValue::TypeArray) return;

  for (int i = 0; i < params.size(); ++i)
  {
    std::string name;
    TempParam param;

    // Skip no name
    if (!params[i]["name"].valid()) continue;

    if (params[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString)
      name = static_cast<std::string>(params[i]["name"]);

    if (params[i]["temp_warn"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      param.temp_warn_ = static_cast<double>(params[i]["temp_warn"]);

    if (params[i]["temp_error"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      param.temp_error_ = static_cast<double>(params[i]["temp_error"]);

    temp_params_[name] = param;
  }
}
