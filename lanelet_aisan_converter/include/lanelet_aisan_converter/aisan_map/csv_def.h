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

#ifndef LANELET_AISAN_CONVERTER_AISAN_MAP_CSV_DEF_H_
#define LANELET_AISAN_CONVERTER_AISAN_MAP_CSV_DEF_H_

#include <iomanip>
#include <iostream>
#include <string>

#include "lanelet_aisan_converter/aisan_map/supported_vector_map_msgs.h"

namespace aisan_map
{
template <class T>
struct csv_def;

template <>
struct csv_def<vector_map_msgs::Point>
{
  static constexpr const char* csv_file_name = "point.csv";
  static constexpr const char* csv_header_row = "PID,B,L,H,Bx,Ly,ReF,MCODE1,MCODE2,MCODE3";

  static inline std::string createCsvDataRow(const vector_map_msgs::Point& obj)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(10);

    oss << obj.pid << ",";
    oss << obj.b << ",";
    oss << obj.l << ",";
    oss << obj.h << ",";
    oss << obj.bx << ",";
    oss << obj.ly << ",";
    oss << obj.ref << ",";
    oss << obj.mcode1 << ",";
    oss << obj.mcode2 << ",";
    oss << obj.mcode3;

    return oss.str();
  }
};

template <>
struct csv_def<vector_map_msgs::Line>
{
  static constexpr const char* csv_file_name = "line.csv";
  static constexpr const char* csv_header_row = "LID,BPID,FPID,BLID,FLID";

  static inline std::string createCsvDataRow(const vector_map_msgs::Line& obj)
  {
    std::ostringstream oss;

    oss << obj.lid << ",";
    oss << obj.bpid << ",";
    oss << obj.fpid << ",";
    oss << obj.blid << ",";
    oss << obj.flid;

    return oss.str();
  }
};

template <>
struct csv_def<vector_map_msgs::Area>
{
  static constexpr const char* csv_file_name = "area.csv";
  static constexpr const char* csv_header_row = "AID,SLID,ELID";

  static inline std::string createCsvDataRow(const vector_map_msgs::Area& obj)
  {
    std::ostringstream oss;

    oss << obj.aid << ",";
    oss << obj.slid << ",";
    oss << obj.elid;

    return oss.str();
  }
};

template <>
struct csv_def<vector_map_msgs::Vector>
{
  static constexpr const char* csv_file_name = "vector.csv";
  static constexpr const char* csv_header_row = "VID,PID,Hang,Vang";

  static inline std::string createCsvDataRow(const vector_map_msgs::Vector& obj)
  {
    std::ostringstream oss;

    oss << obj.vid << ",";
    oss << obj.pid << ",";
    oss << obj.hang << ",";
    oss << obj.vang;

    return oss.str();
  }
};

template <>
struct csv_def<vector_map_msgs::Pole>
{
  static constexpr const char* csv_file_name = "pole.csv";
  static constexpr const char* csv_header_row = "PLID,VID,Length,Dim";

  static inline std::string createCsvDataRow(const vector_map_msgs::Pole& obj)
  {
    std::ostringstream oss;

    oss << obj.plid << ",";
    oss << obj.vid << ",";
    oss << obj.length << ",";
    oss << obj.dim;

    return oss.str();
  }
};

template <>
struct csv_def<vector_map_msgs::CrossRoad>
{
  static constexpr const char* csv_file_name = "intersection.csv";
  static constexpr const char* csv_header_row = "ID,AID,LinkID";

  static inline std::string createCsvDataRow(const vector_map_msgs::CrossRoad& obj)
  {
    std::ostringstream oss;

    oss << obj.id << ",";
    oss << obj.aid << ",";
    oss << obj.linkid;

    return oss.str();
  }
};

template <>
struct csv_def<vector_map_msgs::CrossWalk>
{
  static constexpr const char* csv_file_name = "crosswalk.csv";
  static constexpr const char* csv_header_row = "ID,AID,Type,BdID,LinkID";

  static inline std::string createCsvDataRow(const vector_map_msgs::CrossWalk& obj)
  {
    std::ostringstream oss;

    oss << obj.id << ",";
    oss << obj.aid << ",";
    oss << obj.type << ",";
    oss << obj.bdid << ",";
    oss << obj.linkid;

    return oss.str();
  }
};

template <>
struct csv_def<vector_map_msgs::DTLane>
{
  static constexpr const char* csv_file_name = "dtlane.csv";
  static constexpr const char* csv_header_row = "DID,Dist,PID,Dir,Apara,r,slope,cant,LW,RW";

  static inline std::string createCsvDataRow(const vector_map_msgs::DTLane& obj)
  {
    std::ostringstream oss;

    oss << obj.did << ",";
    oss << obj.dist << ",";
    oss << obj.pid << ",";
    oss << obj.dir << ",";
    oss << obj.apara << ",";
    oss << obj.r << ",";
    oss << obj.slope << ",";
    oss << obj.cant << ",";
    oss << obj.lw << ",";
    oss << obj.rw;

    return oss.str();
  }
};

template <>
struct csv_def<vector_map_msgs::Lane>
{
  static constexpr const char* csv_file_name = "lane.csv";
  static constexpr const char* csv_header_row = "LnID,DID,BLID,FLID,BNID,FNID,JCT,BLID2,BLID3,BLID4,"
                                                "FLID2,FLID3,FLID4,ClosssID,Span,LCnt,Lno,LaneType,"
                                                "LimitVel,RefVel,RoadSecID,LaneChgFG,LinkWAID";

  static inline std::string createCsvDataRow(const vector_map_msgs::Lane& obj)
  {
    std::ostringstream oss;

    oss << obj.lnid << ",";
    oss << obj.did << ",";
    oss << obj.blid << ",";
    oss << obj.flid << ",";
    oss << obj.bnid << ",";
    oss << obj.fnid << ",";
    oss << obj.jct << ",";
    oss << obj.blid2 << ",";
    oss << obj.blid3 << ",";
    oss << obj.blid4 << ",";
    oss << obj.flid2 << ",";
    oss << obj.flid3 << ",";
    oss << obj.flid4 << ",";
    oss << obj.clossid << ",";
    oss << obj.span << ",";
    oss << obj.lcnt << ",";
    oss << obj.lno << ",";
    oss << obj.lanetype << ",";
    oss << obj.limitvel << ",";
    oss << obj.refvel << ",";
    oss << obj.roadsecid << ",";
    oss << obj.lanecfgfg << ",";
    oss << obj.linkwaid;

    return oss.str();
  }
};

template <>
struct csv_def<vector_map_msgs::Node>
{
  static constexpr const char* csv_file_name = "node.csv";
  static constexpr const char* csv_header_row = "NID,PID";

  static inline std::string createCsvDataRow(const vector_map_msgs::Node& obj)
  {
    std::ostringstream oss;

    oss << obj.nid << ",";
    oss << obj.pid;

    return oss.str();
  }
};

template <>
struct csv_def<vector_map_msgs::RoadSign>
{
  static constexpr const char* csv_file_name = "roadsign.csv";
  static constexpr const char* csv_header_row = "ID,VID,PLID,Type,LinkID";

  static inline std::string createCsvDataRow(const vector_map_msgs::RoadSign& obj)
  {
    std::ostringstream oss;

    oss << obj.id << ",";
    oss << obj.vid << ",";
    oss << obj.plid << ",";
    oss << obj.type << ",";
    oss << obj.linkid;

    return oss.str();
  }
};

template <>
struct csv_def<vector_map_msgs::Signal>
{
  static constexpr const char* csv_file_name = "signaldata.csv";
  static constexpr const char* csv_header_row = "ID,VID,PLID,Type,LinkID";

  static inline std::string createCsvDataRow(const vector_map_msgs::Signal& obj)
  {
    std::ostringstream oss;

    oss << obj.id << ",";
    oss << obj.vid << ",";
    oss << obj.plid << ",";
    oss << obj.type << ",";
    oss << obj.linkid;

    return oss.str();
  }
};

template <>
struct csv_def<vector_map_msgs::StopLine>
{
  static constexpr const char* csv_file_name = "stopline.csv";
  static constexpr const char* csv_header_row = "ID,LID,TLID,SignID,LinkID";

  static inline std::string createCsvDataRow(const vector_map_msgs::StopLine& obj)
  {
    std::ostringstream oss;

    oss << obj.id << ",";
    oss << obj.lid << ",";
    oss << obj.tlid << ",";
    oss << obj.signid << ",";
    oss << obj.linkid;

    return oss.str();
  }
};

template <>
struct csv_def<vector_map_msgs::UtilityPole>
{
  static constexpr const char* csv_file_name = "poledata.csv";
  static constexpr const char* csv_header_row = "ID,PLID,LinkID";

  static inline std::string createCsvDataRow(const vector_map_msgs::UtilityPole& obj)
  {
    std::ostringstream oss;

    oss << obj.id << ",";
    oss << obj.plid << ",";
    oss << obj.linkid;

    return oss.str();
  }
};

template <>
struct csv_def<vector_map_msgs::WayArea>
{
  static constexpr const char* csv_file_name = "wayarea.csv";
  static constexpr const char* csv_header_row = "ID,AID";

  static inline std::string createCsvDataRow(const vector_map_msgs::WayArea& obj)
  {
    std::ostringstream oss;

    oss << obj.waid << ",";
    oss << obj.aid;

    return oss.str();
  }
};

template <>
struct csv_def<vector_map_msgs::WhiteLine>
{
  static constexpr const char* csv_file_name = "whiteline.csv";
  static constexpr const char* csv_header_row = "ID,LID,Width,Color,type,LinkID";

  static inline std::string createCsvDataRow(const vector_map_msgs::WhiteLine& obj)
  {
    std::ostringstream oss;

    oss << obj.id << ",";
    oss << obj.lid << ",";
    oss << obj.width << ",";
    oss << obj.color << ",";
    oss << obj.type << ",";
    oss << obj.linkid;

    return oss.str();
  }
};
}  // namespace aisan_map

#endif  // LANELET_AISAN_CONVERTER_AISAN_MAP_CSV_DEF_H_
