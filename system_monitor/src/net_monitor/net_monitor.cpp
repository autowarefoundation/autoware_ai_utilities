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
 * @file net_monitor.cpp
 * @brief Net monitor class
 */

#include <algorithm>
#include <string>
#include <ifaddrs.h>
#include <linux/ethtool.h>
#include <linux/if_link.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <netdb.h>
#include <sys/ioctl.h>
#include <boost/format.hpp>
#include <boost/range/algorithm.hpp>
#include <system_monitor/net_monitor/net_monitor.h>

NetMonitor::NetMonitor(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh), pnh_(pnh)
{
  gethostname(hostname_, sizeof(hostname_));

  pnh_.getParam("devices", device_params_);
  pnh_.param<float>("usage_warn", usage_warn_, 0.95);

  updater_.setHardwareID(hostname_);
  updater_.add("Network Usage", this, &NetMonitor::checkUsage);

  nl80211_.init();
}

void NetMonitor::run(void)
{
  ros::Rate rate(1.0);

  while (ros::ok())
  {
    ros::spinOnce();
    updater_.force_update();
    rate.sleep();
  }

  nl80211_.shutdown();
}

void NetMonitor::checkUsage(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (device_params_.empty())
  {
    stat.summary(DiagStatus::ERROR, "invalid device parameter");
    return;
  }

  const struct ifaddrs *ifa;
  struct ifaddrs *ifas = nullptr;

  ros::Duration duration = ros::Time::now() - last_update_time_;

  // Get network interfaces
  if (getifaddrs(&ifas) < 0)
  {
    stat.summary(DiagStatus::ERROR, "getifaddrs error");
    stat.add("getifaddrs", strerror(errno));
    return;
  }

  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;
  int index = 0;
  std::string error_str = "";
  float rx_traffic;
  float tx_traffic;
  float rx_usage;
  float tx_usage;

  for (ifa = ifas ; ifa ; ifa = ifa->ifa_next)
  {
    // Skip no addr
    if (!ifa->ifa_addr) continue;
    // Skip loopback
    if (ifa->ifa_flags & IFF_LOOPBACK) continue;
    // Skip non AF_PACKET
    if (ifa->ifa_addr->sa_family != AF_PACKET) continue;
    // Skip device not specified
    if (boost::find(device_params_, ifa->ifa_name) == device_params_.end() &&
        boost::find(device_params_, "*") == device_params_.end() ) continue;

    int fd;
    struct ifreq ifrm;
    struct ifreq ifrc;
    struct ethtool_cmd edata;

    // Get MTU information
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    strncpy(ifrm.ifr_name, ifa->ifa_name, IFNAMSIZ-1);
    if (ioctl(fd, SIOCGIFMTU, &ifrm) < 0)
    {
      stat.add((boost::format("Network %1%: status") % index).str(), usage_dict_.at(level));
      stat.add((boost::format("Network %1%: interface name") % index).str(), ifa->ifa_name);
      stat.add("ioctl(SIOCGIFMTU)", strerror(errno));
      error_str = "ioctl error";
      ++index;
      continue;
    }

    // Get network capacity
    float speed = 0.0;
    strncpy(ifrc.ifr_name, ifa->ifa_name, IFNAMSIZ-1);
    ifrc.ifr_data = (caddr_t)&edata;
    edata.cmd = ETHTOOL_GSET;
    if (ioctl(fd, SIOCETHTOOL, &ifrc) < 0)
    {
      // possibly wireless connection, get bitrate(MBit/s) and convert to B/s
      speed = nl80211_.getBitrate(ifa->ifa_name) / 8;
      if (speed <= 0)
      {
        stat.add((boost::format("Network %1%: status") % index).str(), usage_dict_.at(level));
        stat.add((boost::format("Network %1%: interface name") % index).str(), ifa->ifa_name);
        stat.add("ioctl(SIOCETHTOOL)", strerror(errno));
        error_str = "ioctl error";
        ++index;
        continue;
      }
    }
    else
    {
      speed = edata.speed;
    }

    level = (ifa->ifa_flags & IFF_RUNNING) ? DiagStatus::OK : DiagStatus::ERROR;

    struct rtnl_link_stats *stats = (struct rtnl_link_stats*)ifa->ifa_data;
    if (bytes_.find(ifa->ifa_name) != bytes_.end())
    {
      rx_traffic = toMbit(stats->rx_bytes - bytes_[ifa->ifa_name].rx_bytes) / duration.toSec();
      tx_traffic = toMbit(stats->tx_bytes - bytes_[ifa->ifa_name].tx_bytes) / duration.toSec();
      rx_usage = rx_traffic / speed * 1e+2;
      tx_usage = tx_traffic / speed * 1e+2;
      if (rx_usage >= usage_warn_ || tx_usage > usage_warn_)
        level = std::max(level, static_cast<int>(DiagStatus::WARN));
    }

    stat.add((boost::format("Network %1%: status") % index).str(), usage_dict_.at(level));
    stat.add((boost::format("Network %1%: interface name") % index).str(), ifa->ifa_name);
    stat.addf((boost::format("Network %1%: rx_usage") % index).str(), "%.2f%%", rx_usage);
    stat.addf((boost::format("Network %1%: tx_usage") % index).str(), "%.2f%%", tx_usage);
    stat.addf((boost::format("Network %1%: rx_traffic") % index).str(), "%.2f MB/s", rx_traffic);
    stat.addf((boost::format("Network %1%: tx_traffic") % index).str(), "%.2f MB/s", tx_traffic);
    stat.addf((boost::format("Network %1%: capacity") % index).str(), "%.1f MB/s", speed);
    stat.add((boost::format("Network %1%: mtu") % index).str(), ifrm.ifr_mtu);
    stat.add((boost::format("Network %1%: rx_bytes") % index).str(), stats->rx_bytes);
    stat.add((boost::format("Network %1%: rx_errors") % index).str(), stats->rx_errors);
    stat.add((boost::format("Network %1%: tx_bytes") % index).str(), stats->tx_bytes);
    stat.add((boost::format("Network %1%: tx_errors") % index).str(), stats->tx_errors);
    stat.add((boost::format("Network %1%: collisions") % index).str(), stats->collisions);

    close(fd);

    bytes_[ifa->ifa_name].rx_bytes = stats->rx_bytes;
    bytes_[ifa->ifa_name].tx_bytes = stats->tx_bytes;
    whole_level = std::max(whole_level, level);
    ++index;
  }

  freeifaddrs(ifas);

  if (!error_str.empty())
    stat.summary(DiagStatus::ERROR, error_str);
  else
    stat.summary(whole_level, usage_dict_.at(whole_level));

  last_update_time_ = ros::Time::now();
}
