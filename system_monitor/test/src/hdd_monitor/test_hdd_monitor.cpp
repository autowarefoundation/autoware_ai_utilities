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

#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/process.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <system_monitor/hdd_monitor/hdd_monitor.h>

namespace fs = boost::filesystem;
using DiagStatus = diagnostic_msgs::DiagnosticStatus;

char** argv_;

class TestHDDMonitor : public HDDMonitor
{
  friend class HDDMonitorTestSuite;

public:
  TestHDDMonitor(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) : HDDMonitor(nh, pnh) {}

  void diagCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& diag_msg) { array_ = *diag_msg; }

  void addTempParams(const std::string &name, float temp_warn, float temp_error)
  {
    TempParam param;
    param.temp_warn_ = temp_warn;
    param.temp_error_ = temp_error;
    temp_params_[name] = param;
  }
  void changeTempParams(float temp_warn, float temp_error)
  {
    for (auto itr = temp_params_.begin(); itr != temp_params_.end(); ++itr)
    {
      itr->second.temp_warn_ = temp_warn;
      itr->second.temp_error_ = temp_error;
    }
  }
  void clearTempParams(void) { temp_params_.clear(); }

  bool isDeviceExists(void) const
  {
    for (auto itr = temp_params_.begin(); itr != temp_params_.end(); ++itr)
    {
      if (!fs::exists(itr->first)) return false;
    }
  }

  void changeUsageWarn(float usage_warn) { usage_warn_ = usage_warn; }
  void changeUsageError(float usage_error) { usage_error_ = usage_error; }

  void setSmartctlExists(bool mpstat_exists) { smartctl_exists_ = mpstat_exists; }

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

class HDDMonitorTestSuite : public ::testing::Test
{
public:
  HDDMonitorTestSuite() : nh_(""), pnh_("~")
  {
    // Get directory of executable
    const fs::path exe_path(argv_[0]);
    exe_dir_ = exe_path.parent_path().generic_string();
    // Get dummy executable path
    df_ = exe_dir_ + "/df";
  }

protected:
  ros::NodeHandle nh_, pnh_;
  std::unique_ptr<TestHDDMonitor> monitor_;
  ros::Subscriber sub_;
  std::string exe_dir_;
  std::string df_;

  void SetUp(void)
  {
    monitor_ = std::make_unique<TestHDDMonitor>(nh_, pnh_);
    sub_ = nh_.subscribe("/diagnostics", 1000, &TestHDDMonitor::diagCallback, monitor_.get());

    // Remove dummy executable if exists
    if (fs::exists(df_)) fs::remove(df_);
  }

  void TearDown(void)
  {
    // Remove dummy executable if exists
    if (fs::exists(df_)) fs::remove(df_);
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

TEST_F(HDDMonitorTestSuite, tempWarnTest)
{
  // Skip test if no device specified in config
  if (!monitor_->isDeviceExists()) return;

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
    ASSERT_TRUE(monitor_->findDiagStatus("HDD Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
    // Get temperature
    ASSERT_TRUE(findValue(status, "HDD 0: temperature", value));
  }

  // Verify warning
  {
    // Change warning level
    monitor_->changeTempParams(0.0 , 70.0);

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("HDD Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::WARN);
  }

  // Verify normal behavior
  {
    // Change back to normal
    monitor_->changeTempParams(55.0 , 70.0);

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("HDD Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(HDDMonitorTestSuite, tempErrorTest)
{
  // Skip test if no device specified in config
  if (!monitor_->isDeviceExists()) return;

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
    ASSERT_TRUE(monitor_->findDiagStatus("HDD Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify error
  {
    // Change error level
    monitor_->changeTempParams(55.0 , 0.0);

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("HDD Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::ERROR);
  }

  // Verify normal behavior
  {
    // Change back to normal
    monitor_->changeTempParams(55.0 , 70.0);

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("HDD Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(HDDMonitorTestSuite, tempInvalidDiskParameterTest)
{
  // Clear list
  monitor_->clearTempParams();

  // Publish topic
  monitor_->update();

  // Give time to publish
  ros::WallDuration(0.5).sleep();
  ros::spinOnce();

  // Verify
  DiagStatus status;
  ASSERT_TRUE(monitor_->findDiagStatus("HDD Temperature", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "invalid disk parameter");
}

TEST_F(HDDMonitorTestSuite, tempSmartctlNotFoundTest)
{
  // Set flag false
  monitor_->setSmartctlExists(false);

  // Publish topic
  monitor_->update();

  // Give time to publish
  ros::WallDuration(0.5).sleep();
  ros::spinOnce();

  // Verify
  DiagStatus status;
  std::string value;
  ASSERT_TRUE(monitor_->findDiagStatus("HDD Temperature", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "smartctl error");
  ASSERT_TRUE(findValue(status, "smartctl", value));
  ASSERT_STREQ(value.c_str(),
    "Command 'smartctl' not found, but can be installed with: sudo apt install smartmontools");
}

TEST_F(HDDMonitorTestSuite, tempNoSuchDeviceTest)
{
  // Skip test if no device specified in config
  if (!monitor_->isDeviceExists()) return;

  // Add test file to list
  monitor_->addTempParams("/dev/sdx", 55.0, 77.0);

  // Publish topic
  monitor_->update();

  // Give time to publish
  ros::WallDuration(0.5).sleep();
  ros::spinOnce();

  // Verify
  DiagStatus status;
  std::string value;
  ASSERT_TRUE(monitor_->findDiagStatus("HDD Temperature", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "smartctl error");

  ASSERT_TRUE(findValue(status, "HDD 1: status", value));
  ASSERT_STREQ(value.c_str(), "smartctl error");
  ASSERT_TRUE(findValue(status, "HDD 1: name", value));
  ASSERT_STREQ(value.c_str(), "/dev/sdx");
  ASSERT_TRUE(findValue(status, "HDD 1: smartctl", value));
  // 'sudo smartctl -a /dev/sdx' is no set to NOPASWD for sudoers, so we get the following error.
  // sudo: no tty present and no askpass program specified\n
  ASSERT_STREQ(value.c_str(), "sudo: no tty present and no askpass program specified\n");
  // Unable to check below becasuse smartctl does not output anything to stderr when the above command set to NOPASSWD
  // ASSERT_STREQ(value.c_str(), "Smartctl open device: /dev/sdx failed: No such device");
}

TEST_F(HDDMonitorTestSuite, usageWarnTest)
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
    ASSERT_TRUE(monitor_->findDiagStatus("HDD Usage", status));
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
    ASSERT_TRUE(monitor_->findDiagStatus("HDD Usage", status));
    ASSERT_EQ(status.level, DiagStatus::WARN);
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
    ASSERT_TRUE(monitor_->findDiagStatus("HDD Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(HDDMonitorTestSuite, usageErrorTest)
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
    ASSERT_TRUE(monitor_->findDiagStatus("HDD Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify warning
  {
    // Change warning level
    monitor_->changeUsageError(0.0);

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("HDD Usage", status));
    ASSERT_EQ(status.level, DiagStatus::ERROR);
  }

  // Verify normal behavior
  {
    // Change back to normal
    monitor_->changeUsageError(0.99);

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("HDD Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(HDDMonitorTestSuite, usageDfErrorTest)
{
  // Symlink df1 to df
  fs::create_symlink(exe_dir_ + "/df1", df_);

  // Modify PATH temporarily
  modifyPath();

  // Publish topic
  monitor_->update();

  // Give time to publish
  ros::WallDuration(0.5).sleep();
  ros::spinOnce();

  // Verify
  DiagStatus status;
  std::string value;

  ASSERT_TRUE(monitor_->findDiagStatus("HDD Usage", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "df error");
  ASSERT_TRUE(findValue(status, "df", value));
}

int main(int argc, char **argv)
{
  argv_ = argv;
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "HDDMonitorTestNode");

  return RUN_ALL_TESTS();
}
