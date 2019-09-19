/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file excepoint in compliance with the License.
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

#include "lanelet_aisan_converter/lanelet_helper/lanelet_helper.h"

#include <algorithm>
#include <numeric>
#include <queue>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <amathutils_lib/amathutils.hpp>

namespace
{
std::pair<size_t, size_t> findNearestIndexPair(const std::vector<double>& accumulated_lengths,
                                               const double target_length)
{
  // List size
  const auto N = accumulated_lengths.size();

  // Front
  if (target_length < accumulated_lengths.at(1))
  {
    return std::make_pair(0, 1);
  }

  // Back
  if (target_length > accumulated_lengths.at(N - 2))
  {
    return std::make_pair(N - 2, N - 1);
  }

  // Middle
  for (size_t i = 1; i < N; ++i)
  {
    if (accumulated_lengths.at(i - 1) <= target_length && target_length <= accumulated_lengths.at(i))
    {
      return std::make_pair(i - 1, i);
    }
  }

  // Throw an exception because this never happens
  throw std::runtime_error("No nearest point found.");
}

std::vector<lanelet::BasicPoint3d> resamplePoints(const lanelet::ConstLineString3d& line_string, const int num_segments)
{
  // Calculate length
  const auto line_length = boost::geometry::length(line_string);

  // Calculate accumulated lengths
  const auto accumulated_lengths = lanelet_helper::calculateAccumulatedLengths(line_string);

  // Create each segment
  std::vector<lanelet::BasicPoint3d> resampled_points;
  for (int i = 0; i <= num_segments; ++i)
  {
    // Find two nearest points
    const auto target_length = (static_cast<double>(i) / num_segments) * line_length;
    const auto index_pair = findNearestIndexPair(accumulated_lengths, target_length);

    // Apply linear interpolation
    const lanelet::BasicPoint3d back_point = line_string[index_pair.first];
    const lanelet::BasicPoint3d front_point = line_string[index_pair.second];
    const auto direction_vector = (front_point - back_point);

    const auto back_length = accumulated_lengths.at(index_pair.first);
    const auto front_length = accumulated_lengths.at(index_pair.second);
    const auto segment_length = front_length - back_length;
    const auto target_point = back_point + (direction_vector * (target_length - back_length) / segment_length);

    // Add to list
    resampled_points.push_back(target_point);
  }

  return resampled_points;
}

lanelet::LineString3d generateFineCenterline(const lanelet::ConstLanelet& lanelet_obj)
{
  // Parameter
  constexpr double point_interval = 1.0;  // [m]

  // Get length of longer border
  const double left_length = boost::geometry::length(lanelet_obj.leftBound());
  const double right_length = boost::geometry::length(lanelet_obj.rightBound());
  const double longer_distance = (left_length > right_length) ? left_length : right_length;
  const int num_segments = std::max(static_cast<int>(ceil(longer_distance / point_interval)), 1);

  // Resample points
  const auto left_points = resamplePoints(lanelet_obj.leftBound(), num_segments);
  const auto right_points = resamplePoints(lanelet_obj.rightBound(), num_segments);

  // Create centerline
  lanelet::LineString3d centerline(lanelet::utils::getId());
  for (int i = 0; i <= num_segments; i++)
  {
    // Add ID for the average point of left and right
    const auto p = (right_points.at(i) + left_points.at(i)) / 2;
    const lanelet::Point3d center_point(lanelet::utils::getId(), p.x(), p.y(), p.z());
    centerline.push_back(center_point);
  }

  return centerline;
}
}  // namespace

namespace lanelet_helper
{
std::vector<double> calculateSegmentDistances(const lanelet::ConstLineString3d& line_string)
{
  std::vector<double> segment_distances;
  segment_distances.reserve(line_string.size() - 1);

  for (size_t i = 1; i < line_string.size(); ++i)
  {
    const auto distance = lanelet::geometry::distance2d(line_string[i], line_string[i - 1]);
    segment_distances.push_back(distance);
  }

  return segment_distances;
}

std::vector<double> calculateAccumulatedLengths(const lanelet::ConstLineString3d& line_string)
{
  const auto segment_distances = calculateSegmentDistances(line_string);

  std::vector<double> accumulated_lengths{ 0 };
  accumulated_lengths.reserve(segment_distances.size() + 1);
  std::partial_sum(std::begin(segment_distances), std::end(segment_distances), std::back_inserter(accumulated_lengths));

  return accumulated_lengths;
}

double calculateDirection(const lanelet::BasicPoint3d& p1, const lanelet::BasicPoint3d& p2)
{
  const auto dy = p2.y() - p1.y();
  const auto dx = p2.x() - p1.x();
  const auto direction = atan2(dx, dy);  // cw from north

  return direction;
}

double calculateDirection(const lanelet::ConstLineString3d& line_string, const size_t i)
{
  const lanelet::BasicPoint3d p_now = line_string[i];

  // Front
  if (i == 0)
  {
    const lanelet::BasicPoint3d p_front = line_string[i + 1];
    return calculateDirection(p_now, p_front);
  }

  // Back
  if (i == line_string.size() - 1)
  {
    const lanelet::BasicPoint3d p_back = line_string[i - 1];
    return calculateDirection(p_back, p_now);
  }

  // Middle
  const lanelet::BasicPoint3d p_front = line_string[i + 1];
  const lanelet::BasicPoint3d p_back = line_string[i - 1];

  const auto average_direction =
      amathutils::normalizeRadian(calculateDirection(p_back, p_now) + calculateDirection(p_now, p_front)) / 2;

  return average_direction;
}

std::vector<double> calculateDirections(const lanelet::ConstLineString3d& line_string)
{
  std::vector<double> directions;
  directions.reserve(line_string.size());

  for (size_t i = 0; i < line_string.size(); ++i)
  {
    const double direction = calculateDirection(line_string, i);

    directions.push_back(direction);
  }

  return directions;
}

void overwriteLaneletsCenterline(lanelet::LaneletMapPtr lanelet_map)
{
  for (auto& lanelet_obj : lanelet_map->laneletLayer)
  {
    const auto fine_center_line = generateFineCenterline(lanelet_obj);
    lanelet_obj.setCenterline(fine_center_line);
  }
}

std::vector<lanelet::ConstLanelet> getFilteredLanelets(const lanelet::LaneletMap& lanelet_map,
                                                       const std::string& participants)
{
  // Create traffic rules and routing graph for participants
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules =
      lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, participants);
  lanelet::routing::RoutingGraphConstPtr routing_graph =
      lanelet::routing::RoutingGraph::build(lanelet_map, *traffic_rules);

  // Extract lanelets
  std::vector<lanelet::ConstLanelet> lanelet_objs;
  for (const auto& lanelet_obj : lanelet_map.laneletLayer)
  {
    // Ignore incomplete Lanelets
    if (lanelet_obj.leftBound().empty() || lanelet_obj.rightBound().empty())
    {
      continue;
    }

    // Filter by traffic rules
    if (!traffic_rules->canPass(lanelet_obj))
    {
      continue;
    }

    lanelet_objs.push_back(lanelet_obj);
  }

  return lanelet_objs;
}

lanelet::routing::RoutingGraphContainer createRoutingGraphContainer(const lanelet::LaneletMap& lanelet_map,
                                                                    const std::vector<std::string>& participants_list)
{
  std::vector<lanelet::routing::RoutingGraphPtr> routing_graphs;

  for (const auto& participants : participants_list)
  {
    const auto traffic_rules =
        lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, participants);

    lanelet::routing::RoutingGraphPtr routing_graph =
        lanelet::routing::RoutingGraph::build(lanelet_map, *traffic_rules);
    routing_graphs.push_back(routing_graph);
  }

  lanelet::routing::RoutingGraphContainer routing_graph_container(routing_graphs);

  return routing_graph_container;
}

std::vector<lanelet::ConstLanelet> getIntersectionLanelets(const lanelet::routing::RoutingGraphPtr& vehicle_graph,
                                                           const lanelet::ConstLanelet& start_lanelet_obj)
{
  // Initialize data
  std::queue<lanelet::ConstLanelet> queue_lanelet_objs;
  std::vector<lanelet::ConstLanelet> intersection_lanelet_objs;
  std::set<int64_t> already_added_set;
  queue_lanelet_objs.push(start_lanelet_obj);

  // Search until queue become empty
  while (!queue_lanelet_objs.empty())
  {
    const lanelet::ConstLanelet lanelet_obj = queue_lanelet_objs.front();
    queue_lanelet_objs.pop();

    for (const auto conflicting_lanelet_or_area : vehicle_graph->conflicting(lanelet_obj))
    {
      // Ignore Areas
      if (!conflicting_lanelet_or_area.lanelet())
      {
        continue;
      }

      lanelet::ConstLanelet conflicting_lanelet_obj = conflicting_lanelet_or_area.lanelet().value();

      // Ignore same Lanelets
      if (lanelet_obj.id() == conflicting_lanelet_obj.id())
      {
        continue;
      }

      // Register original/conflicting Lanelets
      if (already_added_set.count(lanelet_obj.id()) == 0)
      {
        intersection_lanelet_objs.push_back(lanelet_obj);
        already_added_set.insert(lanelet_obj.id());
      }
      if (already_added_set.count(conflicting_lanelet_obj.id()) == 0)
      {
        queue_lanelet_objs.push(conflicting_lanelet_obj);
        intersection_lanelet_objs.push_back(conflicting_lanelet_obj);
        already_added_set.insert(conflicting_lanelet_obj.id());
      }
    }
  }

  return intersection_lanelet_objs;
}
}  // namespace lanelet_helper
