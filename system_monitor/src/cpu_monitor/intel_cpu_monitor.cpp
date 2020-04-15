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
 * @file _cpu_monitor.cpp
 * @brief  CPU monitor class
 */

#include <algorithm>
#include <string>
#include <vector>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/process.hpp>
#include <boost/regex.hpp>
#include <system_monitor/cpu_monitor/intel_cpu_monitor.h>

namespace fs = boost::filesystem;
namespace bp = boost::process;

CPUMonitor::CPUMonitor(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : CPUMonitorBase(nh, pnh)
{
  modprobeMSR();

  // Check if command exists
  fs::path p = bp::search_path("rdmsr");
  rdmsr_exists_ = (p.empty()) ? false : true;
}

void CPUMonitor::checkThrottling(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (!rdmsr_exists_)
  {
    stat.summary(DiagStatus::ERROR, "rdmsr error");
    stat.add("rdmsr", "Command 'rdmsr' not found, but can be installed with: sudo apt install msrtools");
    return;
  }

  // Read Pkg Thermal Status
  bp::ipstream is_out;
  bp::ipstream is_err;
  bp::child c("sudo rdmsr -af 0:0 0x1b1", bp::std_out > is_out, bp::std_err > is_err);
  c.wait();

  if (c.exit_code() != 0)
  {
    std::ostringstream os;
    is_err >> os.rdbuf();
    stat.summary(DiagStatus::ERROR, "rdmsr error");
    stat.add("rdmsr", os.str().c_str());
    return;
  }

  std::string line;
  std::vector<bool> vals;
  while (std::getline(is_out, line) && !line.empty())
  {
    vals.push_back(std::stoi(line));
  }

  // modprobe error
  if (vals.empty())
  {
    stat.summary(DiagStatus::ERROR, "rdmsr error");
    stat.add("rdmsr", "rdmsr gives empty result");
    return;
  }

  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;
  int index = 0;

  for (auto itr = vals.begin(); itr != vals.end(); ++itr, ++index)
  {
    if (*itr) level = DiagStatus::ERROR;
    else level = DiagStatus::OK;

    stat.add((boost::format("CPU %1%: Pkg Thermal Status") % index).str(), thermal_dict_.at(level));

    whole_level = std::max(whole_level, level);
  }

  stat.summary(whole_level, thermal_dict_.at(whole_level));
}

void CPUMonitor::getTempNames(void)
{
  const fs::path root("/sys/devices/platform");

  for (const fs::path& path :
    boost::make_iterator_range(fs::recursive_directory_iterator(root), fs::recursive_directory_iterator()))
  {
    if (fs::is_directory(path)) continue;

    boost::smatch match;
    boost::regex filter(".*temp(\\d+)_input");
    std::string temp_input = path.generic_string();

    // /sys/devices/platform/coretemp.[0-9]/hwmon/hwmon[0-9]/temp[0-9]_input ?
    if (!boost::regex_match(temp_input, match, filter)) continue;

    cpu_temp_info temp;
    temp.path_ = temp_input;
    temp.label_ = path.filename().generic_string();

    std::string label = boost::algorithm::replace_all_copy(temp_input, "input", "label");
    const fs::path label_path(label);
    fs::ifstream ifs(label_path, std::ios::in);
    if (ifs)
    {
      std::string line;
      if (std::getline(ifs, line)) temp.label_ = line;
    }
    ifs.close();
    temps_.push_back(temp);
  }

  std::sort(temps_.begin(), temps_.end(),
    [](const cpu_temp_info& c1, const cpu_temp_info& c2)
    {
      boost::smatch match;
      boost::regex filter(".*temp(\\d+)_input");
      int n1, n2 = 0;
      if (boost::regex_match(c1.path_, match, filter)) n1 = std::stoi(match[1].str());
      if (boost::regex_match(c2.path_, match, filter)) n2 = std::stoi(match[1].str());
      return n1 < n2;
    }
  );  // NOLINT
}

void CPUMonitor::modprobeMSR()
{
  bp::ipstream is_out;
  bp::ipstream is_err;
  bp::child c("sudo modprobe msr", bp::std_out > is_out, bp::std_err > is_err);
  c.wait();

  if (c.exit_code() != 0)
  {
    std::ostringstream os;
    is_err >> os.rdbuf();
    ROS_ERROR("modprobe error: %s", os.str().c_str());
  }
}
