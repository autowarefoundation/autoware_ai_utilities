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

#include "lanelet_aisan_converter/aisan_map/aisan_map.h"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "lanelet_aisan_converter/aisan_map/csv_def.h"

namespace
{
/**
 * @brief Write objects to CSV file
 * @param [in] save_dir The directory where CSV file is saved
 * @param [in] objs Aisan Map objects
 * @return void
 */
template <class T>
void writeCsv(const std::string& save_dir_str, const std::vector<T>& objs)
{
  if (objs.empty())
  {
    return;
  }

  const fs::path save_dir(save_dir_str);

  if (!fs::exists(save_dir))
  {
    fs::create_directories(save_dir);
  }

  const auto file_path = save_dir / std::string(aisan_map::csv_def<T>::csv_file_name);
  std::ofstream ofs(file_path.string());

  if (!ofs)
  {
    throw std::runtime_error("Failed to open file: " + file_path.string());
  }

  // Write header row
  ofs << aisan_map::csv_def<T>::csv_header_row << std::endl;

  // Write data rows
  for (const auto& obj : objs)
  {
    ofs << aisan_map::csv_def<T>::createCsvDataRow(obj) << std::endl;
  }

  ofs.close();
}

}  // namespace

namespace aisan_map
{
void AisanMap::toCsv(const std::string& save_dir)
{
  writeCsv(save_dir, areas);
  writeCsv(save_dir, cross_roads);
  writeCsv(save_dir, cross_walks);
  writeCsv(save_dir, dt_lanes);
  writeCsv(save_dir, lanes);
  writeCsv(save_dir, lines);
  writeCsv(save_dir, nodes);
  writeCsv(save_dir, points);
  writeCsv(save_dir, poles);
  writeCsv(save_dir, road_signs);
  writeCsv(save_dir, signals);
  writeCsv(save_dir, stop_lines);
  writeCsv(save_dir, utility_poles);
  writeCsv(save_dir, vectors);
  writeCsv(save_dir, way_areas);
  writeCsv(save_dir, white_lines);
}
}  // namespace aisan_map
