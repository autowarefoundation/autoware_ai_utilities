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

#include <algorithm>
#include <vector>

#include "lanelet_aisan_converter/lanelet_helper/lanelet_helper.h"

namespace
{
vector_map_msgs::Lane& findLane(std::vector<vector_map_msgs::Lane>* lanes, const int32_t id)
{
  for (auto& lane : *lanes)
  {
    if (lane.lnid == id)
    {
      return lane;
    }
  }
}
}  // namespace

namespace lanelet2aisan_converter
{
void Lanelet2AisanConverter::overwriteLaneConnectivity()
{
  // Build RoutingGraph
  const auto vehicle_rules =
      lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);

  lanelet::routing::RoutingGraphPtr vehicle_graph =
      lanelet::routing::RoutingGraph::build(*lanelet_map_, *vehicle_rules);

  // Iterate all Lanelets
  for (const auto& lanelet_obj : lanelet_helper::getFilteredLanelets(*lanelet_map_, lanelet::Participants::Vehicle))
  {
    constexpr std::size_t max_size = 4;

    const auto following_connections = vehicle_graph->following(lanelet_obj);
    for (size_t i = 0; i < std::min(max_size, following_connections.size()); ++i)
    {
      const auto following = following_connections.at(i);

      auto& lane = findLane(&aisan_map_->lanes, end_lnid_map_[lanelet_obj.id()]);
      auto& following_lane = findLane(&aisan_map_->lanes, start_lnid_map_[following.id()]);

      if (i == 0)
      {
        assert(lane.flid == 0);
        lane.flid = following_lane.lnid;
      }
      if (i == 1)
      {
        lane.flid2 = following_lane.lnid;
      }
      if (i == 2)
      {
        lane.flid3 = following_lane.lnid;
      }
      if (i == 3)
      {
        lane.flid4 = following_lane.lnid;
      }
    }

    const auto previous_connections = vehicle_graph->previous(lanelet_obj);
    for (size_t i = 0; i < std::min(max_size, previous_connections.size()); ++i)
    {
      const auto previous = previous_connections.at(i);

      auto& lane = findLane(&aisan_map_->lanes, start_lnid_map_[lanelet_obj.id()]);
      auto& previous_lane = findLane(&aisan_map_->lanes, end_lnid_map_[previous.id()]);

      if (i == 0)
      {
        assert(lane.blid == 0);
        lane.blid = previous_lane.lnid;
      }
      if (i == 1)
      {
        lane.blid2 = previous_lane.lnid;
      }
      if (i == 2)
      {
        lane.blid3 = previous_lane.lnid;
      }
      if (i == 3)
      {
        lane.blid4 = previous_lane.lnid;
      }
    }
  }
}
}  // namespace lanelet2aisan_converter
