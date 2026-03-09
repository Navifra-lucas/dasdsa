/*
 * @file	: heuristic.hpp
 * @date	: Feb 7, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: util for global planner
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef GLOBAL_PLANNER_HEURISTIC_HPP_
#define GLOBAL_PLANNER_HEURISTIC_HPP_

#include <vector>

#include "utils/grid_map/map_info.hpp"
#include "utils/grid_map/grid_struct.hpp"
#include "utils/grid_map/grid_map_calculator.hpp"

#include "path_planner/search_planner/utils/utils.hpp"

namespace NVFR {

struct CmpHeuristic {
  bool operator() (const SPU::Node* node1, const SPU::Node* node2) const;
};

// bool WaveFrontHeuristic(
//   const std::vector<int8_t> map,
//   const MapInfo_t st_map_info,
//   Node* start_node,
//   Node* goal_node,
//   std::vector<Node>& nodes);

template <class NODE>
bool WaveFrontHeuristic(
  const std::vector<int8_t> map,
  const MapInfo_t st_map_info,
  NODE* start_node,
  NODE* goal_node,
  std::vector<NODE>& nodes);

} // namespace NVFR

#endif
