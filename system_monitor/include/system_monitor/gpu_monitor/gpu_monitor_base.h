#ifndef SYSTEM_MONITOR_GPU_MONITOR_GPU_MONITOR_BASE_H
#define SYSTEM_MONITOR_GPU_MONITOR_GPU_MONITOR_BASE_H
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
 * @file gpu_monitor.h
 * @brief GPU monitor class
 */

#include <map>
#include <diagnostic_updater/diagnostic_updater.h>

class GPUMonitorBase
{
public:
  /**
   * @brief main loop
   */
  virtual void run(void);

protected:
  using DiagStatus = diagnostic_msgs::DiagnosticStatus;

  /**
   * @brief constructor
   * @param [in] nh node handle to access global parameters
   * @param [in] pnh node handle to access private parameters
   */
  GPUMonitorBase(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

  /**
   * @brief check GPU temperature
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  virtual void checkTemp(diagnostic_updater::DiagnosticStatusWrapper &stat);  // NOLINT(runtime/references)

  /**
   * @brief check GPU usage
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  virtual void checkUsage(diagnostic_updater::DiagnosticStatusWrapper &stat);   // NOLINT(runtime/references)

  /**
   * @brief check GPU memory usage
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  virtual void checkMemoryUsage(diagnostic_updater::DiagnosticStatusWrapper &stat);   // NOLINT(runtime/references)

  /**
   * @brief check GPU throttling
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  virtual void checkThrottling(diagnostic_updater::DiagnosticStatusWrapper &stat);  // NOLINT(runtime/references)

  /**
   * @brief check GPU frequency
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  virtual void checkFrequency(diagnostic_updater::DiagnosticStatusWrapper &stat);   // NOLINT(runtime/references)

  ros::NodeHandle nh_;                    //!< @brief ros node handle
  ros::NodeHandle pnh_;                   //!< @brief private ros node handle
  diagnostic_updater::Updater updater_;   //!< @brief Updater class which advertises to /diagnostics

  char hostname_[HOST_NAME_MAX+1];        //!< @brief host name

  float temp_warn_;                       //!< @brief GPU temperature(DegC) to generate warning
  float temp_error_;                      //!< @brief GPU temperature(DegC) to generate error
  float gpu_usage_warn_;                  //!< @brief GPU usage(%) to generate warning
  float gpu_usage_error_;                 //!< @brief GPU usage(%) to generate error
  float memory_usage_warn_;               //!< @brief GPU memory usage(%) to generate warning
  float memory_usage_error_;              //!< @brief GPU memory usage(%) to generate error

  /**
   * @brief GPU temperature status messages
   */
  const std::map<int, const char*> temp_dict_ =
  {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "warm"}, {DiagStatus::ERROR, "hot"}
  };

  /**
   * @brief GPU usage status messages
   */
  const std::map<int, const char*> load_dict_ =
  {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "high load"}, {DiagStatus::ERROR, "very high load"}
  };

  /**
   * @brief GPU throttling status messages
   */
  const std::map<int, const char*> throttling_dict_ =
  {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "unused"}, {DiagStatus::ERROR, "throttling"}
  };
};

#endif  // SYSTEM_MONITOR_GPU_MONITOR_GPU_MONITOR_BASE_H
