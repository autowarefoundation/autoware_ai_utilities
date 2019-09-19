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

#include "lanelet_aisan_converter/lanelet2aisan_converter/lanelet2aisan_converter.h"

#include "lanelet_aisan_converter/lanelet_helper/lanelet_helper.h"

namespace lanelet2aisan_converter
{
Lanelet2AisanConverter::Lanelet2AisanConverter(const lanelet::LaneletMapPtr& lanelet_map,
                                               const lanelet::projection::UtmProjector& projector)
  : lanelet_map_(lanelet_map), projector_(projector), aisan_map_(std::make_shared<aisan_map::AisanMap>())
{
}

std::shared_ptr<aisan_map::AisanMap> Lanelet2AisanConverter::run()
{
  // Overwrite Centerline of Lanelets
  lanelet_helper::overwriteLaneletsCenterline(lanelet_map_);

  // Create Features
  createCrossRoads();
  createCrossWalks();
  createLanes();
  createSignals();
  createStopLines();
  createWayAreas();
  createWhiteLines();

  // Create Dummy Features
  createDummyUtilityPoles();

  // Create Relations
  overwriteLaneConnectivity();

  return aisan_map_;
}
}  // namespace lanelet2aisan_converter
