/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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
 *
 * Authors: Ryohsuke Mitsudome, Kenji Miyake
 */

#ifndef LANELET_AISAN_CONVERTER_AISAN_MAP_AISAN_MAP_H_
#define LANELET_AISAN_CONVERTER_AISAN_MAP_AISAN_MAP_H_

#include <string>
#include <vector>

#include "lanelet_aisan_converter/aisan_map/supported_vector_map_msgs.h"

namespace aisan_map
{
struct AisanMap
{
  // Basic Shape Class
  std::vector<vector_map_msgs::Point> points;
  std::vector<vector_map_msgs::Line> lines;
  std::vector<vector_map_msgs::Area> areas;
  std::vector<vector_map_msgs::Vector> vectors;
  std::vector<vector_map_msgs::Pole> poles;

  // Basic Shape Class for Lane
  std::vector<vector_map_msgs::DTLane> dt_lanes;
  std::vector<vector_map_msgs::Node> nodes;

  // Feature Class
  std::vector<vector_map_msgs::CrossRoad> cross_roads;
  std::vector<vector_map_msgs::CrossWalk> cross_walks;
  std::vector<vector_map_msgs::Lane> lanes;
  std::vector<vector_map_msgs::RoadSign> road_signs;
  std::vector<vector_map_msgs::Signal> signals;
  std::vector<vector_map_msgs::StopLine> stop_lines;
  std::vector<vector_map_msgs::UtilityPole> utility_poles;
  std::vector<vector_map_msgs::WayArea> way_areas;
  std::vector<vector_map_msgs::WhiteLine> white_lines;

  /**
   * @brief Write Vector Map data to CSV file
   * @param [in] save_dir Path to save directory
   */
  void toCsv(const std::string& save_dir);
};
}  // namespace aisan_map

#endif  // LANELET_AISAN_CONVERTER_AISAN_MAP_AISAN_MAP_H_
