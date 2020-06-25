#ifndef SYSTEM_MONITOR_CPU_MONITOR_INTEL_CPU_MONITOR_H
#define SYSTEM_MONITOR_CPU_MONITOR_INTEL_CPU_MONITOR_H
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
 * @file intel_cpu_monitor.h
 * @brief  CPU monitor class
 */

#include <system_monitor/cpu_monitor/cpu_monitor_base.h>

class CPUMonitor: public CPUMonitorBase
{
public:
  /**
   * @brief constructor
   * @param [in] nh node handle to access global parameters
   * @param [in] pnh node handle to access private parameters
   */
  CPUMonitor(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

protected:
  /**
   * @brief check CPU thermal throttling
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkThrottling(diagnostic_updater::DiagnosticStatusWrapper &stat) override;   // NOLINT(runtime/references)

  /**
   * @brief get names for core temperature files
   */
  void getTempNames(void) override;

  /**
   * @brief Add a loadable kernel module msr
   */
  void modprobeMSR(void);

  int msr_reader_port_;  //!< @brief port number to connect to msr_reader
};

#endif  // SYSTEM_MONITOR_CPU_MONITOR_INTEL_CPU_MONITOR_H
