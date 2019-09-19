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

#ifndef LANELET_AISAN_CONVERTER_LANELET2AISAN_CONVERTER_LANELET2AISAN_CONVERTER_H_
#define LANELET_AISAN_CONVERTER_LANELET2AISAN_CONVERTER_LANELET2AISAN_CONVERTER_H_

#include <memory>
#include <unordered_map>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_projection/UTM.h>

#include "lanelet_aisan_converter/aisan_map/aisan_map.h"

namespace lanelet2aisan_converter
{
using LaneletId = int64_t;
using AisanId = int32_t;

class Lanelet2AisanConverter
{
public:
  Lanelet2AisanConverter() = delete;
  Lanelet2AisanConverter(const lanelet::LaneletMapPtr& lanelet_map, const lanelet::projection::UtmProjector& projector);
  ~Lanelet2AisanConverter() = default;

  /**
   * @brief Convert Lanelet2 OSM Map to Aisan Vector Map
   * @return std::shared_ptr<AisanMap> Converted Aisan Vector Map
   */
  std::shared_ptr<aisan_map::AisanMap> run();

private:
  // Lanelet
  lanelet::LaneletMapPtr lanelet_map_;           //!< @brief Lanelet2 OSM Map
  lanelet::projection::UtmProjector projector_;  //!< @brief Lanelet2 Projector

  // Aisan
  std::shared_ptr<aisan_map::AisanMap> aisan_map_;  //!< @brief Aisan Vector Map

  // ID Map
  std::unordered_map<LaneletId, AisanId> start_lnid_map_;  //!< @brief Storing the start lnid of each lanelet
  std::unordered_map<LaneletId, AisanId> end_lnid_map_;    //!< @brief Storing the end lnid of each lanelet

  // Basic Shape Class
  AisanId createPoint(const lanelet::BasicPoint3d& p);
  AisanId createLine(const lanelet::BasicLineString3d& ps, const AisanId back_line_id, const AisanId front_line_id);
  AisanId createArea(const lanelet::BasicPolygon3d& ps);
  AisanId createVector(const lanelet::BasicPoint3d& p, const double horizontal_angle, const double vertical_angle);
  AisanId createPole(const lanelet::BasicPoint3d& p, const double horizontal_angle, const double vertical_angle,
                     const double length, const double diameter);

  // Basic Shape Class for Lane
  AisanId createDTLane(const lanelet::BasicPoint3d& p, const double dist, const double dir, const double left_width,
                       const double right_width);
  AisanId createNode(const lanelet::BasicPoint3d& p);

  // Dummy Feature
  AisanId createDummyRoadSign(const lanelet::BasicPoint3d& p);
  AisanId createDummyUtilityPoles();

  // Feature Class
  void createCrossRoads();
  void createCrossWalks();
  void createLanes();
  void createSignals();
  void createStopLines();
  void createWayAreas();
  void createWhiteLines();

  // Overwrite
  void overwriteLaneConnectivity();
};
}  // namespace lanelet2aisan_converter

#endif  // LANELET_AISAN_CONVERTER_LANELET2AISAN_CONVERTER_LANELET2AISAN_CONVERTER_H_
