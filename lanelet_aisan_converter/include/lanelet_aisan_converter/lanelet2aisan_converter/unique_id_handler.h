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

#ifndef LANELET_AISAN_CONVERTER_LANELET2AISAN_CONVERTER_UNIQUE_ID_HANDLER_H_
#define LANELET_AISAN_CONVERTER_LANELET2AISAN_CONVERTER_UNIQUE_ID_HANDLER_H_

#include <unordered_map>

namespace lanelet2aisan_converter
{
template <class T>
class UniqueIdHandler
{
private:
  UniqueIdHandler() : max_id_(1), id_map_({})
  {
  }
  ~UniqueIdHandler() = default;

public:
  UniqueIdHandler(const UniqueIdHandler&) = delete;
  UniqueIdHandler& operator=(const UniqueIdHandler&) = delete;
  UniqueIdHandler(UniqueIdHandler&&) = delete;
  UniqueIdHandler& operator=(UniqueIdHandler&&) = delete;

  /**
   * @brief Get singleton instance
   * @return UniqueIdHandler& singleton instance
   */
  static UniqueIdHandler& getInstance()
  {
    static UniqueIdHandler instance;
    return instance;
  }

  /**
   * @brief Get unique 32bit ID from 64bit ID to prevent overflow
   * @param [in] id 64bit ID
   * @return int32_t 32bit ID
   */
  int32_t getUniqueInt32Id(const int64_t id)
  {
    if (id_map_.count(id))
    {
      return id_map_[id];
    }

    id_map_[id] = max_id_++;
    return id_map_[id];
  }

  /**
   * @brief Get unique 32bit ID
   * @return int32_t 32bit ID
   */
  int32_t generateUniqueInt32Id()
  {
    return max_id_++;
  }

  /**
   * @brief Get next ID
   * @return int32_t Next ID
   */
  int32_t getNextId()
  {
    return max_id_;
  }

private:
  int32_t max_id_;
  std::unordered_map<int64_t, int32_t> id_map_;
};
}  // namespace lanelet2aisan_converter

#endif  // LANELET_AISAN_CONVERTER_LANELET2AISAN_CONVERTER_UNIQUE_ID_HANDLER_H_
