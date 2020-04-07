#ifndef SYSTEM_MONITOR_HDD_MONITOR_HDD_MONITOR_H
#define SYSTEM_MONITOR_HDD_MONITOR_HDD_MONITOR_H
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
 * @file hdd_monitor.h
 * @brief HDD monitor class
 */

#include <map>
#include <string>
#include <diagnostic_updater/diagnostic_updater.h>

/**
 * @brief error and warning temperature levels
 */
struct TempParam
{
  float temp_warn_;   //!< @brief HDD temperature(DegC) to generate warning
  float temp_error_;  //!< @brief HDD temperature(DegC) to generate error

  TempParam() : temp_warn_(55.0), temp_error_(70.0) {}
};

class HDDMonitor
{
public:
  /**
   * @brief constructor
   * @param [in] nh node handle to access global parameters
   * @param [in] pnh node handle to access private parameters
   */
  HDDMonitor(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

  /**
   * @brief main loop
   */
  void run(void);

protected:
  using DiagStatus = diagnostic_msgs::DiagnosticStatus;

  /**
   * @brief check HDD temperature
   * @param [in] stat diagnostic status message
   */
  void checkTemp(diagnostic_updater::DiagnosticStatusWrapper &stat);    // NOLINT

  /**
   * @brief check HDD usage
   * @param [in] stat diagnostic status message
   */
  void checkUsage(diagnostic_updater::DiagnosticStatusWrapper &stat);   // NOLINT

  /**
   * @brief get temperature parameters
   */
  void getTempParams(void);

  ros::NodeHandle nh_;                            //!< @brief ros node handle
  ros::NodeHandle pnh_;                           //!< @brief private ros node handle
  diagnostic_updater::Updater updater_;           //!< @brief Updater class which advertises to /diagnostics

  char hostname_[HOST_NAME_MAX+1];                //!< @brief host name
  bool smartctl_exists_;                          //!< @brief flag if smartctl exists

  float usage_warn_;                              //!< @brief HDD usage(%) to generate warning
  float usage_error_;                             //!< @brief HDD usage(%) to generate error
  std::map<std::string, TempParam> temp_params_;  //!< @brief list of error and warning levels

  /**
   * @brief HDD temperature status messages
   */
  const std::map<int, const char*> temp_dict_ =
  {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "hot"}, {DiagStatus::ERROR, "critical hot"}
  };

  /**
   * @brief HDD usage status messages
   */
  const std::map<int, const char*> usage_dict_ =
  {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "low disk space"}, {DiagStatus::ERROR, "very low disk space"}
  };
};

#endif  // SYSTEM_MONITOR_HDD_MONITOR_HDD_MONITOR_H
