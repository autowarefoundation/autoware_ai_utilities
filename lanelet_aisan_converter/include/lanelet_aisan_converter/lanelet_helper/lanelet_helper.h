
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

#ifndef LANELET_AISAN_CONVERTER_LANELET_HELPER_LANELET_HELPER_H_
#define LANELET_AISAN_CONVERTER_LANELET_HELPER_LANELET_HELPER_H_

#include <string>
#include <vector>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

namespace lanelet_helper
{
/**
 * @brief Calculate distances of each segment in LineString
 * @return std::vector<double> Distances of each segment
 */
std::vector<double> calculateSegmentDistances(const lanelet::ConstLineString3d& line_string);

/**
 * @brief Calculate accumulated lengths in each point of LineString
 * @return std::vector<double> Accumulated lengths in each point
 */
std::vector<double> calculateAccumulatedLengths(const lanelet::ConstLineString3d& line_string);

/**
 * @brief Calculate direction between two points
 * @return double Direction
 */
double calculateDirection(const lanelet::BasicPoint3d& p1, const lanelet::BasicPoint3d& p2);

/**
 * @brief Calculate direction of LineString
 * @return double Direction
 */
double calculateDirection(const lanelet::ConstLineString3d& line_string, const size_t i);

/**
 * @brief Calculate directions of each point in LineString
 * @return std::vector<double> Directions of each point
 */
std::vector<double> calculateDirections(const lanelet::ConstLineString3d& line_string);

/**
 * @brief  Apply a patch for centerline because the original implementation doesn't have enough quality
 */
void overwriteLaneletsCenterline(lanelet::LaneletMapPtr lanelet_map);

/**
 * @brief Get Lanelets filtered by Location and Participants
 * @return std::vector<lanelet::Lanelet> Filtered Lanelets
 */
std::vector<lanelet::ConstLanelet> getFilteredLanelets(const lanelet::LaneletMap& lanelet_map,
                                                       const std::string& participants);

/**
 * @brief Create RoutingGraphContainer from participants
 * @return std::vector<Lanelet> RoutingGraphContainer
 */
lanelet::routing::RoutingGraphContainer createRoutingGraphContainer(const lanelet::LaneletMap& lanelet_map,
                                                                    const std::vector<std::string>& participants_list);

/**
 * @brief Get Lanelets in same intersection
 * @return std::vector<lanelet::ConstLanelet> Lanelets in same intersection
 */
std::vector<lanelet::ConstLanelet> getIntersectionLanelets(const lanelet::routing::RoutingGraphPtr& vehicle_graph,
                                                           const lanelet::ConstLanelet& lanelet_obj);

}  // namespace lanelet_helper

#endif  // LANELET_AISAN_CONVERTER_LANELET_HELPER_LANELET_HELPER_H_
