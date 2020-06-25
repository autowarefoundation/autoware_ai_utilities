#ifndef SYSTEM_MONITOR_PROCESS_MONITOR_PROCESS_MONITOR_H
#define SYSTEM_MONITOR_PROCESS_MONITOR_PROCESS_MONITOR_H
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
 * @file process_monitor.h
 * @brief Process monitor class
 */

#include <string>
#include <vector>
#include <boost/process.hpp>
#include <diagnostic_updater/diagnostic_updater.h>
#include <system_monitor/process_monitor/diag_task.h>

namespace bp = boost::process;

class ProcessMonitor
{
public:
  /**
   * @brief constructor
   * @param [in] nh node handle to access global parameters
   * @param [in] pnh node handle to access private parameters
   */
  ProcessMonitor(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

  /**
   * @brief main loop
   */
  void run(void);

protected:
  using DiagStatus = diagnostic_msgs::DiagnosticStatus;

  /**
   * @brief monitor processes
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void monitorProcesses(diagnostic_updater::DiagnosticStatusWrapper &stat);   // NOLINT(runtime/references)

  /**
   * @brief get task summary
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @param [in] output top command output
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void getTasksSummary
    (diagnostic_updater::DiagnosticStatusWrapper &stat, const std::string &output);   // NOLINT(runtime/references)

  /**
   * @brief remove header
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @param [in] output top command output
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void removeHeader
    (diagnostic_updater::DiagnosticStatusWrapper &stat, std::string &output);  // NOLINT(runtime/references)

  /**
   * @brief get high load processes
   * @param [in] output top command output
   */
  void getHighLoadProcesses(const std::string &output);

  /**
   * @brief get high memory processes
   * @param [in] output top command output
   */
  void getHighMemoryProcesses(const std::string &output);

  /**
   * @brief get top-rated processes
   * @param [in] tasks list of diagnostics tasks for high load procs
   * @param [in] output top command output
   */
  void getTopratedProcesses(std::vector<std::shared_ptr<DiagTask>> *tasks, bp::pipe *p);

  /**
   * @brief get top-rated processes
   * @param [in] tasks list of diagnostics tasks for high load procs
   * @param [in] message Diagnostics status message
   * @param [in] error_command Error command
   * @param [in] content Error content
   */
  void setErrorContent(std::vector<std::shared_ptr<DiagTask>> *tasks, const std::string &message,
                       const std::string &error_command, const std::string &content);

  ros::NodeHandle nh_;                                    //!< @brief ros node handle
  ros::NodeHandle pnh_;                                   //!< @brief private ros node handle
  diagnostic_updater::Updater updater_;                   //!< @brief Updater class which advertises to /diagnostics

  char hostname_[HOST_NAME_MAX+1];                        //!< @brief host name

  int num_of_procs_;                                      //!< @brief number of processes to show
  std::vector<std::shared_ptr<DiagTask>> load_tasks_;     //!< @brief list of diagnostics tasks for high load procs
  std::vector<std::shared_ptr<DiagTask>> memory_tasks_;   //!< @brief list of diagnostics tasks for high memory procs
};

#endif  // SYSTEM_MONITOR_PROCESS_MONITOR_PROCESS_MONITOR_H
