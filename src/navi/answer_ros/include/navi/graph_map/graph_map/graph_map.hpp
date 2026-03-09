/*
 * @file	: graph_map.hpp
 * @date	: Nov 6, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: graph map
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef GRAPH_MAP_HPP_
#define GRAPH_MAP_HPP_

#include <queue>
#include <memory>
#include <string>
#include <mutex>
#include <tuple>

#include "utils/data_manager.hpp"

#include "graph_map/graph_node.hpp"
#include "graph_map/map_loader.hpp"
#include "graph_map/utils.hpp"

namespace NVFR {

struct ClosestNode_t
{
  bool b_find = false;
  bool b_on_edge = false;
  GraphNode o_node1;
  GraphNode o_node2;
  std::string s_edge_id;
};

class GraphMap
{
public:
  GraphMap() = default;
  GraphMap(const std::string& s_file);
  ~GraphMap() = default;

  void SetParam();

  void SetFile(const std::string& s_file);

  bool LoadMap();

  void SetRobotPos(double d_x_m, double d_y_m, double d_yaw_rad);

  GraphNode GetNode(const std::string& s_node_id, const std::string& s_node_name) const;

  ClosestNode_t FindClosestNode() const;
  ClosestNode_t FindClosestNode(
    const Pose& o_robot_pose,
    const std::vector<GraphNode>& vec_nodes,
    const std::vector<SimplePath>& vec_paths) const;

  std::vector<GraphEdge> PlanPath(
    const std::string& s_node_id, const std::string& s_node_name);
  std::vector<GraphEdge> PlanPath(
    const std::string& s_start_node_id, const std::string& s_start_node_name,
    const std::string& s_goal_node_id, const std::string& s_goal_node_name);

private:
  std::string s_file_;

  // graph map
  MapLoader o_map_loader_;
  mutable std::mutex mtx_map_;

  // robot pose
  DataManager<Pose> o_robot_pose_;
  // Pose o_robot_pose_;
  // mutable std::mutex mtx_robot_pose_;

};

} // namespace NVFR

#endif
