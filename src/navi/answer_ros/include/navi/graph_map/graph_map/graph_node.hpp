/*
 * @file	: graph_node.hpp
 * @date	: Nov 6, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: graph node structure
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef GRAPH_NODE_HPP_
#define GRAPH_NODE_HPP_

#include <vector>
#include <string>

#include "utils/motion_planner_type_list.hpp"
#include "utils/pose.hpp"
#include "utils/node_info.hpp"
#include "utils/mission_edge.hpp"

namespace NVFR {

class GraphEdge : public MissionEdge
{
public:
  const std::string& GetId() const { return s_id_; }
  int GetFromNodeNum() const { return n_from_node_num_; }
  int GetToNodeNum() const { return n_to_node_num_; }

  void SetId(const std::string& s_id) { s_id_ = s_id; }
  void SetFromNodeNum(int n_from_node_num) { n_from_node_num_ = n_from_node_num; }
  void SetToNodeNum(int n_to_node_num) { n_to_node_num_ = n_to_node_num; }

  bool operator==(const std::string& s_cmp_str) const {
    return s_id_ == s_cmp_str;
  }

private:
  std::string s_id_;
  int n_from_node_num_;
  int n_to_node_num_;

};

class GraphNode : public NodeInfo
{
public:
  bool GetArriveAlign() const { return b_arrive_align_; }
  int GetNodeNum() const { return n_node_num_; }
  std::vector<int>& GetRefVecFromNodes() { return vec_from_nodes_; }
  std::vector<GraphEdge>& GetRefVecEdges() { return vec_edges_; }
  const std::vector<int>& GetConstVecFromNodes() const { return vec_from_nodes_; }
  const std::vector<GraphEdge>& GetConstVecEdges() const { return vec_edges_; }

  void SetArriveAlign(bool b_arrive_align) { b_arrive_align_ = b_arrive_align; }
  void SetNodeNum(int n_node_num) { n_node_num_ = n_node_num; }

  bool operator==(const int& n_cmp_num) const {
    return n_node_num_ == n_cmp_num;
  }
  bool operator==(const std::string& s_cmp_str) const {
    return (s_id_ == s_cmp_str) || (s_name_ == s_cmp_str);
  }

private:
  bool b_arrive_align_ = false;
  int n_node_num_ = -1;
  std::vector<int> vec_from_nodes_;
  std::vector<GraphEdge> vec_edges_;

};

class SimplePath
{
public:
  const std::string& GetId() const { return s_id_; }
  int GetFromNodeNum() const { return n_from_node_num_; }
  int GetToNodeNum() const { return n_to_node_num_; }
  Path& GetRefPath() { return path_; }
  const Path& GetConstPath() const { return path_; }

  void SetId(const std::string& s_id) { s_id_ = s_id; }
  void SetFromNodeNum(int n_from_node_num) { n_from_node_num_ = n_from_node_num; }
  void SetToNodeNum(int n_to_node_num) { n_to_node_num_ = n_to_node_num; }

  bool operator==(const std::string& s_cmp_str) const {
    return s_id_ == s_cmp_str;
  }

private:
  std::string s_id_;
  int n_from_node_num_;
  int n_to_node_num_;
  Path path_;

};

} // namespace NVFR

#endif
