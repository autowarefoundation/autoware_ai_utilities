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

#include <iostream>
#include <string>

#include <ros/ros.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include "lanelet_aisan_converter/aisan_map/aisan_map.h"

void printUsage()
{
  std::cout << "Required parameters" << std::endl;
  std::cout << "_map_file:=<path to lanelet map>" << std::endl;
  std::cout << "_save_dir:=<path to output dir of aisan map>" << std::endl;
  std::cout << "_origin_lat:=<latitude of origin for XYZ projection>" << std::endl;
  std::cout << "_origin_lon:=<longitude of origin for XYZ projection>" << std::endl;
}

int main(int argc, char* argv[])
{
  // Initialize ROS
  ros::init(argc, argv, "lanelet2vectormap");
  ros::NodeHandle private_nh("~");

  // Check parameters
  if (!private_nh.hasParam("map_file"))
  {
    ROS_ERROR_STREAM("You must specify file path!");
    printUsage();
    exit(1);
  }

  if (!private_nh.hasParam("origin_lat") || !private_nh.hasParam("origin_lon"))
  {
    ROS_ERROR_STREAM("You must specify an origin!");
    printUsage();
    exit(1);
  }

  // Get parameters
  std::string map_file;
  std::string save_dir;
  double origin_lat;
  double origin_lon;

  private_nh.param<std::string>("map_file", map_file, "");
  private_nh.param<std::string>("save_dir", save_dir, "./aisan_vector_map");
  private_nh.param<double>("origin_lat", origin_lat, 0.0);
  private_nh.param<double>("origin_lon", origin_lon, 0.0);

  // Load Lanelet2 map
  const lanelet::Origin origin({ origin_lat, origin_lon });  // NOLINT
  const lanelet::projection::UtmProjector projector(origin);
  lanelet::ErrorMessages errors;
  lanelet::LaneletMapPtr lanelet_map = lanelet::load(map_file, projector, &errors);

  // Error handling
  if (!errors.empty())
  {
    // Display errors
    for (const auto& error : errors)
    {
      ROS_ERROR_STREAM(error);
    }

    // Exit if any error exists
    exit(1);
  }

  // Convert Lanelet2 OSM Map to Aisan Vector Map
  lanelet2aisan_converter::Lanelet2AisanConverter converter(lanelet_map, projector);
  const auto aisan_map = converter.run();

  // Write to CSV
  aisan_map->toCsv(save_dir);

  return 0;
}
