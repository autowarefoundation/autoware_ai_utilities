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

#include <vector>

#include "lanelet_aisan_converter/lanelet2aisan_converter/unique_id_handler.h"

namespace
{
double deg2dms(const double deg)
{
  const auto abs_deg = std::abs(deg);

  const auto d = std::floor(abs_deg);
  const auto ms = abs_deg - d;
  const auto m = std::floor(ms * 60.0);
  const auto s = (ms - m / 60.0) * 3600.0;

  return std::copysign(d + m * 0.01 + s * 0.0001, deg);
}

}  // namespace

namespace lanelet2aisan_converter
{
AisanId Lanelet2AisanConverter::createPoint(const lanelet::BasicPoint3d& local_point)
{
  // Convert local coordinate to global
  const auto global_point = projector_.reverse(local_point);

  // Create Shape
  vector_map_msgs::Point point{};
  point.pid = UniqueIdHandler<vector_map_msgs::Point>::getInstance().generateUniqueInt32Id();
  point.b = deg2dms(global_point.lat);
  point.l = deg2dms(global_point.lon);
  point.h = global_point.ele;
  point.bx = local_point.y();  // Reverse X and Y for JGD2011
  point.ly = local_point.x();  // Reverse X and Y for JGD2011
  point.ref = 0;               // Not supported
  point.mcode1 = 0;            // Not supported
  point.mcode2 = 0;            // Not supported
  point.mcode3 = 0;            // Not supported

  // Add to list
  aisan_map_->points.push_back(point);

  return point.pid;
}

AisanId Lanelet2AisanConverter::createLine(const lanelet::BasicLineString3d& ps, const AisanId back_line_id,
                                           const AisanId front_line_id)
{
  // Create each point
  std::vector<AisanId> point_ids;
  for (const auto& p : ps)
  {
    const auto id = createPoint(p);
    point_ids.push_back(id);
  }

  // Create Shape
  vector_map_msgs::Line line{};
  line.lid = UniqueIdHandler<vector_map_msgs::Line>::getInstance().generateUniqueInt32Id();
  line.bpid = point_ids.front();
  line.fpid = point_ids.back();
  line.blid = back_line_id;
  line.flid = front_line_id;

  // Add to list
  aisan_map_->lines.push_back(line);

  return line.lid;
}

AisanId Lanelet2AisanConverter::createArea(const lanelet::BasicPolygon3d& ps)
{
  // Prefetch Line IDs
  std::vector<AisanId> line_ids;
  const auto next_id = UniqueIdHandler<vector_map_msgs::Line>::getInstance().getNextId();
  for (size_t i = 0; i < ps.size(); ++i)
  {
    line_ids.push_back(next_id + i);
  }

  // Create each segment
  for (size_t i = 0; i < ps.size(); ++i)
  {
    const size_t back_idx = i;
    const size_t front_idx = (i == ps.size() - 1) ? 0 : i + 1;

    const auto back_line_id = (i == 0) ? 0 : line_ids.at(back_idx);
    const auto front_line_id = (i == ps.size() - 1) ? 0 : line_ids.at(front_idx);

    const lanelet::BasicLineString3d segment{ ps.at(back_idx), ps.at(front_idx) };
    const auto id = createLine(segment, back_line_id, front_line_id);
  }

  // Create Shape
  vector_map_msgs::Area area{};
  area.aid = UniqueIdHandler<vector_map_msgs::Area>::getInstance().generateUniqueInt32Id();
  area.slid = line_ids.front();
  area.elid = line_ids.back();

  // Add to list
  aisan_map_->areas.push_back(area);

  return area.aid;
}

AisanId Lanelet2AisanConverter::createVector(const lanelet::BasicPoint3d& p, const double horizontal_angle,
                                             const double vertical_angle)
{
  // Create Point
  const auto pid = createPoint(p);

  // Create Shape
  vector_map_msgs::Vector vector{};
  vector.vid = UniqueIdHandler<vector_map_msgs::Vector>::getInstance().generateUniqueInt32Id();
  vector.pid = pid;
  vector.hang = horizontal_angle;
  vector.vang = vertical_angle;

  // Add to list
  aisan_map_->vectors.push_back(vector);

  return vector.vid;
}

AisanId Lanelet2AisanConverter::createPole(const lanelet::BasicPoint3d& p, const double horizontal_angle,
                                           const double vertical_angle, const double length, const double diameter)
{
  // Create vector
  const auto vid = createVector(p, horizontal_angle, vertical_angle);

  // Create Shape
  vector_map_msgs::Pole pole{};
  pole.plid = UniqueIdHandler<vector_map_msgs::Pole>::getInstance().generateUniqueInt32Id();
  pole.vid = vid;
  pole.length = length;
  pole.dim = diameter;

  // Add to list
  aisan_map_->poles.push_back(pole);

  return pole.plid;
}

AisanId Lanelet2AisanConverter::createDTLane(const lanelet::BasicPoint3d& p, const double dist, const double dir,
                                             const double left_width, const double right_width)
{
  // Create Point
  const auto pid = createPoint(p);

  // Create Feature
  vector_map_msgs::DTLane dt_lane{};
  dt_lane.did = UniqueIdHandler<vector_map_msgs::DTLane>::getInstance().generateUniqueInt32Id();
  dt_lane.dist = dist;
  dt_lane.pid = pid;
  dt_lane.dir = dir;
  dt_lane.apara = 0;  // Not supported
  dt_lane.r = 9e10;   // Not supported
  dt_lane.slope = 0;  // Not supported
  dt_lane.cant = 0;   // Not supported
  dt_lane.lw = left_width;
  dt_lane.rw = right_width;

  // Add to list
  aisan_map_->dt_lanes.push_back(dt_lane);

  return dt_lane.did;
}

AisanId Lanelet2AisanConverter::createNode(const lanelet::BasicPoint3d& p)
{
  // Create Point
  const auto pid = createPoint(p);

  // Create Feature
  vector_map_msgs::Node node{};
  node.nid = UniqueIdHandler<vector_map_msgs::Node>::getInstance().generateUniqueInt32Id();
  node.pid = pid;

  // Add to list
  aisan_map_->nodes.push_back(node);

  return node.nid;
}

}  // namespace lanelet2aisan_converter
