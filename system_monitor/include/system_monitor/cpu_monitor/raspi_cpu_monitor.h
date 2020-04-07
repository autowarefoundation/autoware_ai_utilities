#ifndef SYSTEM_MONITOR_CPU_MONITOR_RASPI_CPU_MONITOR_H
#define SYSTEM_MONITOR_CPU_MONITOR_RASPI_CPU_MONITOR_H
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
 * @file raspi_cpu_monitor.h
 * @brief Raspberry Pi CPU monitor class
 */

#include <system_monitor/cpu_monitor/cpu_monitor_base.h>

#define raspiUnderVoltageDetected             (1 << 0)    // 0x00001
#define raspiArmFrequencyCapped               (1 << 1)    // 0x00002
#define raspiCurrentlyThrottled               (1 << 2)    // 0x00004
#define raspiSoftTemperatureLimitActive       (1 << 3)    // 0x00008
#define raspiUnderVoltageHasOccurred          (1 << 16)   // 0x10000
#define raspiArmFrequencyCappedHasOccurred    (1 << 17)   // 0x20000
#define raspiThrottlingHasOccurred            (1 << 18)   // 0x40000
#define raspiSoftTemperatureLimitHasOccurred  (1 << 19)   // 0x80000

#define raspiThermalThrottlingMask            (raspiCurrentlyThrottled | raspiSoftTemperatureLimitActive)

#define throttledToString(X) ( \
  ((X) & raspiUnderVoltageDetected)            ? "Under-voltage detected"              : \
  ((X) & raspiArmFrequencyCapped)              ? "Arm frequency capped"                : \
  ((X) & raspiCurrentlyThrottled)              ? "Currently throttled"                 : \
  ((X) & raspiSoftTemperatureLimitActive)      ? "Soft temperature limit active"       : \
  ((X) & raspiUnderVoltageHasOccurred)         ? "Under-voltage has occurred"          : \
  ((X) & raspiArmFrequencyCappedHasOccurred)   ? "Arm frequency capped has occurred"   : \
  ((X) & raspiThrottlingHasOccurred)           ? "Throttling has occurred"             : \
  ((X) & raspiSoftTemperatureLimitHasOccurred) ? "Soft temperature limit has occurred" : \
                                                 "UNKNOWN" )

class CPUMonitor: public CPUMonitorBase
{
public:
  /**
   * @brief constructor
   * @param [in] nh node handle to access global parameters
   * @param [in] pnh node handle to access private parameters
   */
  CPUMonitor(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

  /**
   * @brief get names for core temperature files
   */
  void getTempNames(void) override;

protected:
  /**
   * @brief check CPU thermal throttling
   * @param [in] stat diagnostic status message
   */
  void checkThrottling(diagnostic_updater::DiagnosticStatusWrapper &stat) override;   // NOLINT
};

#endif  // SYSTEM_MONITOR_CPU_MONITOR_RASPI_CPU_MONITOR_H