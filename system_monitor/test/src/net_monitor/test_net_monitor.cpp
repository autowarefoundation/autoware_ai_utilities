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

#include <ifaddrs.h>
#include <linux/ethtool.h>
#include <linux/if_link.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/process.hpp>
#include <boost/range/algorithm.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <system_monitor/net_monitor/net_monitor.h>

#define DOCKER_ENV "/.dockerenv"

namespace fs = boost::filesystem;
using DiagStatus = diagnostic_msgs::DiagnosticStatus;

char** argv_;

class TestNetMonitor : public NetMonitor
{
  friend class NetMonitorTestSuite;

public:
  TestNetMonitor(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) : NetMonitor(nh, pnh) {}

  void diagCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& diag_msg) { array_ = *diag_msg; }

  void changeUsageWarn(float usage_warn) { usage_warn_ = usage_warn; }

  const std::vector<std::string> getDeviceParams(void) { return device_params_; }
  void clearDeviceParams(void) { device_params_.clear(); }

  void update(void) { updater_.force_update(); }

  const std::string removePrefix(const std::string &name) { return boost::algorithm::erase_all_copy(name, prefix_); }

  bool findDiagStatus(const std::string &name, DiagStatus& status)  // NOLINT
  {
    for (int i = 0; i < array_.status.size(); ++i)
    {
      if (removePrefix(array_.status[i].name) == name)
      {
        status = array_.status[i];
        return true;
      }
    }
    return false;
  }

private:
  diagnostic_msgs::DiagnosticArray array_;
  const std::string prefix_ = ros::this_node::getName().substr(1) + ": ";
};

class NetMonitorTestSuite : public ::testing::Test
{
public:
  NetMonitorTestSuite() : nh_(""), pnh_("~")
  {
    // Get directory of executable
    const fs::path exe_path(argv_[0]);
    exe_dir_ = exe_path.parent_path().generic_string();
    // Get dummy executable path
    iw_ = exe_dir_ + "/iw";
  }

protected:
  ros::NodeHandle nh_, pnh_;
  std::unique_ptr<TestNetMonitor> monitor_;
  ros::Subscriber sub_;
  std::string exe_dir_;
  std::string iw_;

  void SetUp(void)
  {
    monitor_ = std::make_unique<TestNetMonitor>(nh_, pnh_);
    sub_ = nh_.subscribe("/diagnostics", 1000, &TestNetMonitor::diagCallback, monitor_.get());

    // Remove dummy executable if exists
    if (fs::exists(iw_)) fs::remove(iw_);
  }

  void TearDown(void)
  {
    // Remove dummy executable if exists
    if (fs::exists(iw_)) fs::remove(iw_);
  }

  bool findValue(const DiagStatus status, const std::string &key, std::string &value)   // NOLINT
  {
    for (auto itr = status.values.begin(); itr != status.values.end(); ++itr)
    {
      if (itr->key == key)
      {
        value = itr->value;
        return true;
      }
    }
    return false;
  }

  void modifyPath(void)
  {
    // Modify PATH temporarily
    auto env = boost::this_process::environment();
    std::string new_path = env["PATH"].to_string();
    new_path.insert(0, (boost::format("%1%:") % exe_dir_).str());
    env["PATH"] = new_path;
  }
};

TEST_F(NetMonitorTestSuite, usageWarnTest)
{
  // Verify normal behavior
  {
    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    std::string value;
    ASSERT_TRUE(monitor_->findDiagStatus("Network Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify warning
  {
    // Change warning level
    monitor_->changeUsageWarn(0.0);

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("Network Usage", status));
    // Skip test if process runs inside docker
    // Don't know what interface should be monitored.
    if (!fs::exists(DOCKER_ENV)) ASSERT_EQ(status.level, DiagStatus::WARN);
  }

  // Verify normal behavior
  {
    // Change back to normal
    monitor_->changeUsageWarn(0.95);

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("Network Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(NetMonitorTestSuite, usageInvalidDeviceParameterTest)
{
  // Clear list
  monitor_->clearDeviceParams();

  // Publish topic
  monitor_->update();

  // Give time to publish
  ros::WallDuration(0.5).sleep();
  ros::spinOnce();

  // Verify
  DiagStatus status;
  ASSERT_TRUE(monitor_->findDiagStatus("Network Usage", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "invalid device parameter");
}

TEST_F(NetMonitorTestSuite, usageIwErrorTest)
{
  // Get list
  const std::vector<std::string> params = monitor_->getDeviceParams();

  const struct ifaddrs *ifa;
  struct ifaddrs *ifas = NULL;

  // Skip test if no network interface
  if (getifaddrs(&ifas) < 0) return;

  bool found = false;
  for (ifa = ifas ; ifa ; ifa = ifa->ifa_next)
  {
    // Skip no addr
    if (!ifa->ifa_addr) continue;
    // Skip loopback
    if (ifa->ifa_flags & IFF_LOOPBACK) continue;
    // Skip non AF_PACKET
    if (ifa->ifa_addr->sa_family != AF_PACKET) continue;
    // Skip device not specified
    if (boost::find(params, ifa->ifa_name) == params.end() &&
        boost::find(params, "*") == params.end() ) continue;

    int fd;
    struct ifreq ifrc;
    struct ethtool_cmd edata;

    // Get MTU information
    fd = socket(AF_INET, SOCK_DGRAM, 0);

    strncpy(ifrc.ifr_name, ifa->ifa_name, IFNAMSIZ-1);
    ifrc.ifr_data = (caddr_t)&edata;
    edata.cmd = ETHTOOL_GSET;
    if (ioctl(fd, SIOCETHTOOL, &ifrc) < 0)
    {
      // possibly wireless connection
      found = true;
      break;
    }
  }

  // Skip test if no wireless network interface
  if (!found) return;

  // Symlink iw1 to iw
  fs::create_symlink(exe_dir_ + "/iw1", iw_);

  // Modify PATH temporarily
  modifyPath();

  // Publish topic
  monitor_->update();

  // Give time to publish
  ros::WallDuration(0.5).sleep();
  ros::spinOnce();

  // Verify
  DiagStatus status;
  ASSERT_TRUE(monitor_->findDiagStatus("Network Usage", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "ioctl error");
}

int main(int argc, char **argv)
{
  argv_ = argv;
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "NetMonitorTestNode");

  return RUN_ALL_TESTS();
}
