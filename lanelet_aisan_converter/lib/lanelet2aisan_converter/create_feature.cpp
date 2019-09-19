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

#include <set>
#include <string>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/multi/geometries/register/multi_point.hpp>

#include <amathutils_lib/amathutils.hpp>

#include "lanelet_aisan_converter/lanelet_helper/lanelet_helper.h"
#include "lanelet_aisan_converter/lanelet2aisan_converter/unique_id_handler.h"

BOOST_GEOMETRY_REGISTER_MULTI_POINT(decltype(std::vector<lanelet::BasicPoint2d>{}))  // NOLINT

namespace
{
char convertLineColor(const std::string& type, const std::string& subtype)
{
  return 'W';  // Color is not supported in Lanelet2
}

uint convertLineType(const std::string& type, const std::string& subtype)
{
  if (subtype == "solid")
  {
    return vector_map_msgs::WhiteLine::SOLID_LINE;
  }
  if (subtype == "dashed")
  {
    return vector_map_msgs::WhiteLine::DASHED_LINE_SOLID;
  }

  return vector_map_msgs::WhiteLine::SOLID_LINE;  // default
}
}  // namespace

namespace lanelet2aisan_converter
{
AisanId Lanelet2AisanConverter::createDummyRoadSign(const lanelet::BasicPoint3d& p)
{
  // Create Vector
  const auto vid = createVector(p, 0, 90);

  // Create Pole
  const auto plid = createPole(p, 0, 0, 1.0, 0.02);

  // Create Feature
  vector_map_msgs::RoadSign road_sign{};
  road_sign.id = UniqueIdHandler<vector_map_msgs::RoadSign>::getInstance().generateUniqueInt32Id();
  road_sign.vid = vid;
  road_sign.plid = plid;
  road_sign.type = 1;    // Stop Sign
  road_sign.linkid = 0;  // Not supported

  // Add to list
  aisan_map_->road_signs.push_back(road_sign);
}

AisanId Lanelet2AisanConverter::createDummyUtilityPoles()
{
  for (const auto& pole : aisan_map_->poles)
  {
    // Create Feature
    vector_map_msgs::UtilityPole utility_pole{};
    utility_pole.id = UniqueIdHandler<vector_map_msgs::UtilityPole>::getInstance().generateUniqueInt32Id();
    utility_pole.plid = pole.plid;
    utility_pole.linkid = 0;  // Not supported

    // Add to list
    aisan_map_->utility_poles.push_back(utility_pole);
  }
}

void Lanelet2AisanConverter::createCrossRoads()
{
  // Build RoutingGraph
  const auto vehicle_rules =
      lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);

  lanelet::routing::RoutingGraphPtr vehicle_graph =
      lanelet::routing::RoutingGraph::build(*lanelet_map_, *vehicle_rules);

  // Memorize added ID
  std::set<LaneletId> already_added_set;

  for (const auto& lanelet_obj : lanelet_helper::getFilteredLanelets(*lanelet_map_, lanelet::Participants::Vehicle))
  {
    if (already_added_set.count(lanelet_obj.id()) != 0)
    {
      continue;
    }

    // Get intersection lanelets
    const auto intersection_lanelet_objs = lanelet_helper::getIntersectionLanelets(vehicle_graph, lanelet_obj);

    if (intersection_lanelet_objs.empty())
    {
      continue;
    }

    for (const auto& intersection_lanelet_obj : intersection_lanelet_objs)
    {
      already_added_set.insert(intersection_lanelet_obj.id());
    }

    // Gather all points of Lanelets in intersection
    std::vector<lanelet::BasicPoint2d> vertices;
    for (const auto& intersection_lanelet_obj : intersection_lanelet_objs)
    {
      for (const auto& p : intersection_lanelet_obj.polygon3d())
      {
        vertices.push_back(lanelet::utils::to2D(p.basicPoint()));
      }
    }

    // Get Z value
    const double z = intersection_lanelet_objs.front().centerline3d().front().z();

    // Create convex hull
    std::vector<lanelet::BasicPoint2d> convex_hull;
    boost::geometry::convex_hull(vertices, convex_hull);

    // Create 3d points
    lanelet::BasicPolygon3d ps;
    for (const auto& p : convex_hull)
    {
      ps.emplace_back(p.x(), p.y(), z);
    }

    // Create Area
    const auto aid = createArea(ps);

    // Create Feature
    vector_map_msgs::CrossRoad cross_road{};
    cross_road.id = UniqueIdHandler<vector_map_msgs::CrossRoad>::getInstance().generateUniqueInt32Id();
    cross_road.aid = aid;
    cross_road.linkid = 0;  // Not supported

    // Add to list
    aisan_map_->cross_roads.push_back(cross_road);
  }
}

void Lanelet2AisanConverter::createCrossWalks()
{
  // Prepare traffic rules
  const auto vehicle_pedestrian_graph = lanelet_helper::createRoutingGraphContainer(
      *lanelet_map_, { lanelet::Participants::Vehicle, lanelet::Participants::Pedestrian });  // NOLINT

  for (const auto& lanelet_obj : lanelet_helper::getFilteredLanelets(*lanelet_map_, lanelet::Participants::Vehicle))
  {
    // Find confligting CrossWalks
    for (const auto& cross_walk_lanelet_obj : vehicle_pedestrian_graph.conflictingInGraph(lanelet_obj, 1))
    {
      // Outline Shape
      AisanId parent_id;
      {
        // Create Area
        const lanelet::BasicPolygon3d line_string = cross_walk_lanelet_obj.polygon3d().basicPolygon();
        const auto aid = createArea(line_string);

        // Create Feature
        vector_map_msgs::CrossWalk cross_walk{};
        cross_walk.id = UniqueIdHandler<vector_map_msgs::CrossWalk>::getInstance().generateUniqueInt32Id();
        cross_walk.aid = aid;
        cross_walk.type = 0;    // Outline
        cross_walk.bdid = 0;    // No parent
        cross_walk.linkid = 0;  // Not supported

        // Save parent ID
        parent_id = cross_walk.id;

        // Add to list
        aisan_map_->cross_walks.push_back(cross_walk);
      }

      // Dummy Stripe Shape
      {
        // Create Area
        const lanelet::BasicPolygon3d line_string = cross_walk_lanelet_obj.polygon3d().basicPolygon();
        const auto aid = createArea(line_string);

        // Create Feature
        vector_map_msgs::CrossWalk cross_walk{};
        cross_walk.id = UniqueIdHandler<vector_map_msgs::CrossWalk>::getInstance().generateUniqueInt32Id();
        cross_walk.aid = aid;
        cross_walk.type = 1;  // Stripe
        cross_walk.bdid = parent_id;
        cross_walk.linkid = 0;  // Not supported

        // Add to list
        aisan_map_->cross_walks.push_back(cross_walk);
      }
    }
  }
}

void Lanelet2AisanConverter::createLanes()
{
  // Build Routing Graph
  const auto vehicle_rules =
      lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);

  lanelet::routing::RoutingGraphUPtr vehicle_graph =
      lanelet::routing::RoutingGraph::build(*lanelet_map_, *vehicle_rules);

  for (const auto& lanelet_obj : lanelet_helper::getFilteredLanelets(*lanelet_map_, lanelet::Participants::Vehicle))
  {
    // Calculate attributes of each point
    const auto centerline = lanelet_obj.centerline();
    const auto accumulated_lengths = lanelet_helper::calculateAccumulatedLengths(centerline);
    const auto directions = lanelet_helper::calculateDirections(centerline);

    // Calculate attributes of whole Lane
    const auto lane_count = vehicle_graph->besides(lanelet_obj).size() + 1;
    const auto lane_number = vehicle_graph->lefts(lanelet_obj).size() + 1;
    const auto speed_limit = vehicle_rules->speedLimit(lanelet_obj).speedLimit.value();

    // Create each point
    std::vector<AisanId> node_ids;
    for (const auto& p : centerline)
    {
      const auto node_id = createNode(p);
      node_ids.push_back(node_id);
    }

    // Prefetch Lane IDs
    std::vector<AisanId> lane_ids;
    const auto next_id = UniqueIdHandler<vector_map_msgs::Lane>::getInstance().getNextId();
    for (size_t i = 0; i < centerline.size(); ++i)
    {
      lane_ids.push_back(next_id + i);
    }

    const auto N = centerline.size();
    for (size_t i = 0; i < N - 1; ++i)
    {
      // Get attributes
      const lanelet::BasicPoint3d p = centerline[i];
      const auto dist = accumulated_lengths.at(i);
      const auto dir = directions.at(i);
      const auto left_width = lanelet::geometry::distance2d(p, lanelet_obj.leftBound());
      const auto right_width = lanelet::geometry::distance2d(p, lanelet_obj.rightBound());

      // Create DTLane
      const auto did = createDTLane(p, dist, dir, left_width, right_width);

      // Create Feature
      vector_map_msgs::Lane lane{};
      lane.lnid = UniqueIdHandler<vector_map_msgs::Lane>::getInstance().generateUniqueInt32Id();
      lane.did = did;
      lane.bnid = node_ids.at(i);
      lane.fnid = node_ids.at(i + 1);
      lane.span = accumulated_lengths.at(i + 1) - accumulated_lengths.at(i);
      lane.lcnt = lane_count;
      lane.lno = lane_number;
      lane.limitvel = speed_limit;
      lane.refvel = speed_limit;
      lane.lanecfgfg = (lane.lcnt > 1) ? 1 : 0;
      lane.roadsecid = 0;  // Not supported
      lane.clossid = 0;    // Not supported
      lane.linkwaid = 0;   // Not supported
      lane.jct = 0;        // Not supported
      lane.lanetype = 0;   // Not supported

      // Backward conectivities will be (re)calculated later
      lane.blid = (i == 0) ? 0 : lane_ids.at(i - 1);
      lane.blid2 = 0;
      lane.blid3 = 0;
      lane.blid4 = 0;

      // Forward conectivities  will be (re)calculated later
      lane.flid = (i == N - 2) ? 0 : lane_ids.at(i + 1);
      lane.flid2 = 0;
      lane.flid3 = 0;
      lane.flid4 = 0;

      // Save start/end Lane ID
      if (i == 0)
      {
        start_lnid_map_[lanelet_obj.id()] = lane.lnid;
      }
      if (i == N - 2)
      {
        end_lnid_map_[lanelet_obj.id()] = lane.lnid;
      }

      // Add to list
      aisan_map_->lanes.push_back(lane);
    }
  }
}

void Lanelet2AisanConverter::createSignals()
{
  // Memorize added ID
  std::set<LaneletId> already_added_set;

  for (const auto& lanelet_obj : lanelet_helper::getFilteredLanelets(*lanelet_map_, lanelet::Participants::Vehicle))
  {
    for (const auto traffic_light_reg_elem : lanelet_obj.regulatoryElementsAs<lanelet::TrafficLight>())
    {
      for (const auto traffic_light : traffic_light_reg_elem->trafficLights())
      {
        if (already_added_set.count(traffic_light.id()) != 0)
        {
          continue;
        }

        already_added_set.insert(traffic_light.id());

        if (!traffic_light.isLineString())
        {
          continue;
        }

        // Calculate lane angle
        const auto N = lanelet_obj.centerline().size();
        const auto p1 = lanelet_obj.centerline()[N - 1];
        const auto p2 = lanelet_obj.centerline()[N - 2];
        const auto lane_angle = atan2(p2.y() - p1.y(), p2.x() - p1.x());

        // Create bulb's color and position info
        struct BulbInfo
        {
          int color;
          double pos;
        };

        // NOLINTNEXTLINE
        std::vector<BulbInfo> bulb_info_list = {
          { vector_map_msgs::Signal::RED, 0.25 },
          { vector_map_msgs::Signal::YELLOW, 0.5 },
          { vector_map_msgs::Signal::BLUE, 0.75 },
        };

        // Calculate direction of light's LineString
        lanelet::ConstLineString3d light_bulb_points = traffic_light.lineString().value();
        const lanelet::BasicPoint3d start_point = light_bulb_points.front();
        const lanelet::BasicPoint3d end_point = light_bulb_points.back();
        const lanelet::BasicPoint3d direction_vector = end_point - start_point;

        // Create Pole
        const auto pole_height = 5.0;
        const auto mid_point = (start_point + end_point) / 2;
        const auto plid = createPole(mid_point, 0, 0, pole_height, 0.3);

        // Create each light bulb
        for (const auto& bulb_info : bulb_info_list)
        {
          // Calculate parameters
          const lanelet::BasicPoint3d p_bottom = start_point + bulb_info.pos * direction_vector;
          const lanelet::BasicPoint3d p(p_bottom.x(), p_bottom.y(), p_bottom.z() + pole_height);

          // Create vector
          const auto vid = createVector(p, amathutils::rad2deg(-lane_angle), 90);

          // Create Feature
          vector_map_msgs::Signal signal{};
          signal.id = UniqueIdHandler<vector_map_msgs::Signal>::getInstance().generateUniqueInt32Id();
          signal.vid = vid;
          signal.plid = plid;
          signal.type = bulb_info.color;
          signal.linkid = 0;  // Not supported

          // Add to list
          aisan_map_->signals.push_back(signal);
        }
      }
    }
  }
}

void Lanelet2AisanConverter::createStopLines()
{
  for (const auto& line_string : lanelet_map_->lineStringLayer)
  {
    if (line_string.attributeOr(lanelet::AttributeName::Type, "") != "stop_line")
    {
      continue;
    }

    // Create Line
    const lanelet::BasicLineString3d ps = line_string.basicLineString();
    const auto lid = createLine(ps, 0, 0);

    // Create Feature
    vector_map_msgs::StopLine stop_line{};
    stop_line.id = UniqueIdHandler<vector_map_msgs::StopLine>::getInstance().generateUniqueInt32Id();
    stop_line.lid = lid;
    stop_line.tlid = 0;    // Not supported
    stop_line.signid = 0;  // Not supported
    stop_line.linkid = 0;  // Not supported

    // Add to list
    aisan_map_->stop_lines.push_back(stop_line);
  }
}

void Lanelet2AisanConverter::createWayAreas()
{
  for (const auto& lanelet_obj : lanelet_helper::getFilteredLanelets(*lanelet_map_, lanelet::Participants::Vehicle))
  {
    // Create Area
    const auto aid = createArea(lanelet_obj.polygon3d().basicPolygon());

    // Create Feature
    vector_map_msgs::WayArea way_area{};
    way_area.waid = UniqueIdHandler<vector_map_msgs::WayArea>::getInstance().generateUniqueInt32Id();
    way_area.aid = aid;

    // Add to list
    aisan_map_->way_areas.push_back(way_area);
  }
}

void Lanelet2AisanConverter::createWhiteLines()
{
  for (const auto& lanelet_obj : lanelet_helper::getFilteredLanelets(*lanelet_map_, lanelet::Participants::Vehicle))
  {
    for (const auto& line_string : { lanelet_obj.leftBound(), lanelet_obj.rightBound() })
    {
      // Get attributes
      const std::string type = line_string.attributeOr(lanelet::AttributeName::Type, "");
      const std::string subtype = line_string.attributeOr(lanelet::AttributeName::Subtype, "");

      const lanelet::BasicLineString3d ps = line_string.basicLineString();

      // Prefetch Line IDs
      std::vector<AisanId> line_ids;
      const auto next_id = UniqueIdHandler<vector_map_msgs::Line>::getInstance().getNextId();
      for (size_t i = 0; i < ps.size(); ++i)
      {
        line_ids.push_back(next_id + i);
      }

      // Create each segment
      for (size_t i = 0; i < ps.size() - 1; ++i)
      {
        const auto back_line_id = (i == 0) ? 0 : line_ids.at(i);
        const auto front_line_id = (i == ps.size() - 2) ? 0 : line_ids.at(i + 1);

        const lanelet::BasicLineString3d segment{ ps.at(i), ps.at(i + 1) };
        const auto lid = createLine(segment, back_line_id, front_line_id);

        // Create Feature
        vector_map_msgs::WhiteLine white_line{};
        white_line.id = UniqueIdHandler<vector_map_msgs::WhiteLine>::getInstance().generateUniqueInt32Id();
        white_line.lid = lid;
        white_line.width = (type == "line_thick") ? 0.15 : 0.05;
        white_line.color = convertLineColor(type, subtype);
        white_line.type = convertLineType(type, subtype);
        white_line.linkid = 0;  // Not supported

        // Add to list
        aisan_map_->white_lines.push_back(white_line);
      }
    }
  }
}
}  // namespace lanelet2aisan_converter
