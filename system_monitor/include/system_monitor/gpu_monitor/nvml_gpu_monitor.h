#ifndef SYSTEM_MONITOR_GPU_MONITOR_NVML_GPU_MONITOR_H
#define SYSTEM_MONITOR_GPU_MONITOR_NVML_GPU_MONITOR_H
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
 * @file nvml_gpu_monitor.h
 * @brief NVML GPU monitor class
 */

#include <map>
#include <string>
#include <vector>
#include <nvml.h>
#include <system_monitor/gpu_monitor/gpu_monitor_base.h>

#define reasonToString(X) ( \
  ((X) & nvmlClocksThrottleReasonGpuIdle)                   ? "GpuIdle"                   : \
  ((X) & nvmlClocksThrottleReasonApplicationsClocksSetting) ? "ApplicationsClocksSetting" : \
  ((X) & nvmlClocksThrottleReasonSwPowerCap)                ? "SwPowerCap"                : \
  ((X) & nvmlClocksThrottleReasonHwSlowdown)                ? "HwSlowdown"                : \
  ((X) & nvmlClocksThrottleReasonSyncBoost)                 ? "SyncBoost"                 : \
  ((X) & nvmlClocksThrottleReasonSwThermalSlowdown)         ? "SwThermalSlowdown"         : \
  ((X) & nvmlClocksThrottleReasonHwThermalSlowdown)         ? "HwThermalSlowdown"         : \
  ((X) & nvmlClocksThrottleReasonHwPowerBrakeSlowdown)      ? "HwPowerBrakeSlowdown"      : \
  ((X) & nvmlClocksThrottleReasonDisplayClockSetting)       ? "DisplayClockSetting"       : \
                                                              "UNKNOWN" )

/**
 * @brief GPU information
 */
typedef struct gpu_info
{
  nvmlDevice_t device;                      //!< @brief handle for a particular device
  char name[NVML_DEVICE_NAME_BUFFER_SIZE];  //!< @brief name of device
  nvmlPciInfo_t pci;                        //!< @brief PCI information about a GPU device
  nvmlUtilization_t utilization;            //!< @brief Utilization information for a device
}
gpu_info;

class GPUMonitor: public GPUMonitorBase
{
public:
  /**
   * @brief constructor
   * @param [in] nh node handle to access global parameters
   * @param [in] pnh node handle to access private parameters
   */
  GPUMonitor(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

  /**
   * @brief main loop
   */
  void run(void) override;

protected:
  /**
   * @brief check GPU temperature
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkTemp(diagnostic_updater::DiagnosticStatusWrapper &stat) override;   // NOLINT(runtime/references)

  /**
   * @brief check GPU usage
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkUsage(diagnostic_updater::DiagnosticStatusWrapper &stat) override;  // NOLINT(runtime/references)

  /**
   * @brief check GPU memory usage
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkMemoryUsage(diagnostic_updater::DiagnosticStatusWrapper &stat) override;  // NOLINT(runtime/references)

  /**
   * @brief check GPU throttling
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkThrottling(diagnostic_updater::DiagnosticStatusWrapper &stat) override;   // NOLINT(runtime/references)

  /**
   * @brief get human-readable output for memory size
   * @param [in] size size with bytes
   * @return human-readable output
   * @note NOLINT syntax is needed since struct nvmlMemory_t has unsigned long long values to return memory size.
   */
  std::string toHumanReadable(unsigned long long size);   // NOLINT(runtime/int)

  std::vector<gpu_info> gpus_;            //!< @brief list of gpus
};

#endif  // SYSTEM_MONITOR_GPU_MONITOR_NVML_GPU_MONITOR_H
