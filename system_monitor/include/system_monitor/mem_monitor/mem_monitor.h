#ifndef SYSTEM_MONITOR_MEM_MONITOR_MEM_MONITOR_H
#define SYSTEM_MONITOR_MEM_MONITOR_MEM_MONITOR_H
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
 * @file mem_monitor.h
 * @brief Memory monitor class
 */

#include <map>
#include <string>
#include <diagnostic_updater/diagnostic_updater.h>

class MemMonitor
{
public:
  /**
   * @brief constructor
   * @param [in] nh node handle to access global parameters
   * @param [in] pnh node handle to access private parameters
   */
  MemMonitor(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

  /**
   * @brief main loop
   */
  void run(void);

protected:
  using DiagStatus = diagnostic_msgs::DiagnosticStatus;

  /**
   * @brief check Memory usage
   * @param @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkUsage(diagnostic_updater::DiagnosticStatusWrapper &stat);   // NOLINT(runtime/references)

  /**
   * @brief get human-readable output for memory size
   * @param [in] str size with bytes
   * @return human-readable output
   */
  std::string toHumanReadable(const std::string &str);

  ros::NodeHandle nh_;                    //!< @brief ros node handle
  ros::NodeHandle pnh_;                   //!< @brief private ros node handle
  diagnostic_updater::Updater updater_;   //!< @brief Updater class which advertises to /diagnostics

  char hostname_[HOST_NAME_MAX+1];        //!< @brief host name

  float usage_warn_;                      //!< @brief Memory usage(%) to generate warning
  float usage_error_;                     //!< @brief Memory usage(%) to generate error

  /**
   * @brief Memory usage status messages
   */
  const std::map<int, const char*> usage_dict_ =
  {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "high load"}, {DiagStatus::ERROR, "very high load"}
  };
};

#endif  // SYSTEM_MONITOR_MEM_MONITOR_MEM_MONITOR_H
