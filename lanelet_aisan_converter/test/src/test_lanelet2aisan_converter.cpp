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

#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <ros/ros.h>
#include <ros/package.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <vector_map/vector_map.h>

#include "lanelet_aisan_converter/aisan_map/csv_def.h"
#include "lanelet_aisan_converter/lanelet2aisan_converter/lanelet2aisan_converter.h"

class TestSuite : public ::testing::Test
{
public:
  TestSuite() : nh_("")
  {
  }
  ~TestSuite() = default;

protected:
  virtual void SetUp()
  {
    fs::create_directories(output_base_dir_);
  }

  virtual void TearDown()
  {
    fs::remove_all(output_base_dir_);
  }

  ros::NodeHandle nh_;
  const fs::path output_base_dir_ = "/tmp/test_lanelet_aisan_converter";

  // NOLINTNEXTLINE
  const std::unordered_map<std::string, vector_map::Category> category_map_ = {
    { "point.csv", vector_map::Category::POINT },
    { "vector.csv", vector_map::Category::VECTOR },
    { "line.csv", vector_map::Category::LINE },
    { "area.csv", vector_map::Category::AREA },
    { "pole.csv", vector_map::Category::POLE },
    { "box.csv", vector_map::Category::BOX },
    { "dtlane.csv", vector_map::Category::DTLANE },
    { "node.csv", vector_map::Category::NODE },
    { "lane.csv", vector_map::Category::LANE },
    { "wayarea.csv", vector_map::Category::WAY_AREA },
    { "roadedge.csv", vector_map::Category::ROAD_EDGE },
    { "gutter.csv", vector_map::Category::GUTTER },
    { "curb.csv", vector_map::Category::CURB },
    { "whiteline.csv", vector_map::Category::WHITE_LINE },
    { "stopline.csv", vector_map::Category::STOP_LINE },
    { "zebrazone.csv", vector_map::Category::ZEBRA_ZONE },
    { "crosswalk.csv", vector_map::Category::CROSS_WALK },
    { "road_surface_mark.csv", vector_map::Category::ROAD_MARK },
    { "poledata.csv", vector_map::Category::ROAD_POLE },
    { "roadsign.csv", vector_map::Category::ROAD_SIGN },
    { "signaldata.csv", vector_map::Category::SIGNAL },
    { "streetlight.csv", vector_map::Category::STREET_LIGHT },
    { "utilitypole.csv", vector_map::Category::UTILITY_POLE },
    { "guardrail.csv", vector_map::Category::GUARD_RAIL },
    { "sidewalk.csv", vector_map::Category::SIDE_WALK },
    { "driveon_portion.csv", vector_map::Category::DRIVE_ON_PORTION },
    { "intersection.csv", vector_map::Category::CROSS_ROAD },
    { "sidestrip.csv", vector_map::Category::SIDE_STRIP },
    { "curvemirror.csv", vector_map::Category::CURVE_MIRROR },
    { "wall.csv", vector_map::Category::WALL },
    { "fence.csv", vector_map::Category::FENCE },
    { "railroad_crossing.csv", vector_map::Category::RAIL_CROSSING },
  };
};

TEST_F(TestSuite, ConvertingExampleLaneletMap)
{
  // Paremeters for the sample map
  const auto origin_lat = 49.00331750371;
  const auto origin_lon = 8.42403027399;
  const auto lanelet2_maps_package = ros::package::getPath("lanelet2_maps");
  const auto map_file = fs::path(lanelet2_maps_package) / "res" / "mapping_example.osm";

  // Load Lanelet2 map
  const lanelet::Origin origin({ origin_lat, origin_lon });  // NOLINT
  const lanelet::projection::UtmProjector projector(origin);
  lanelet::ErrorMessages errors;
  lanelet::LaneletMapPtr lanelet_map = lanelet::load(map_file.string(), projector, &errors);

  // Can load map with no error
  ASSERT_TRUE(errors.empty());

  // Convert Lanelet2 OSM Map to Aisan Vector Map
  lanelet2aisan_converter::Lanelet2AisanConverter converter(lanelet_map, projector);
  const auto aisan_map = converter.run();

  // Write to CSV
  const auto save_dir = output_base_dir_ / "aisan_vector_map";
  aisan_map->toCsv(save_dir.string());

  // Create a file for notifying completion
  std::ofstream ofs((save_dir / ".completion_notice").string());

  // Collect CSV file names and categories
  std::ostringstream oss_files;
  vector_map::category_t category = vector_map::Category::NONE;
  for (const auto& itr : fs::directory_iterator(save_dir))
  {
    const auto p = itr.path();
    if (p.extension() == ".csv")
    {
      oss_files << " " << fs::absolute(p).string();
      category |= category_map_.at(p.filename().string());
    }
  }

  // Wait for subscribing map topics
  vector_map::VectorMap vmap;
  vmap.subscribe(nh_, category);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestNode");
  return RUN_ALL_TESTS();
}
