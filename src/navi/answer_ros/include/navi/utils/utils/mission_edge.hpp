/*
 * @file	: mission_edge.hpp
 * @date	: Feb 6, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: class for mission edge
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef MISSION_EDGE_HPP_
#define MISSION_EDGE_HPP_

#include <string>
#include <vector>
#include <memory>
#include <stdint.h>

#include "utils/motion_planner_type_list.hpp"
#include "utils/pose.hpp"
#include "utils/mission_info.hpp"

namespace NVFR {

class MissionEdge
{
public:
  using Ptr = std::shared_ptr<MissionEdge>;
  using ConstPtr = std::shared_ptr<const MissionEdge>;

  MissionEdge():e_path_type_(MPTL::EDGE_PATH::LINE),d_target_spd_(0.5),n_end_idx_(-1) {};
  virtual ~MissionEdge() {};

  const NodeInfo& GetConstFromNode() const { return o_from_node_; };
  const NodeInfo& GetConstToNode() const { return o_to_node_; };
  NodeInfo GetFromNode() const { return o_from_node_; };
  NodeInfo GetToNode() const { return o_to_node_; };
  const NodeInfo* GetFromNodePtr() { return &o_from_node_; };
  const NodeInfo* GetToNodePtr() { return &o_to_node_; };
  const std::string& GetFromNodeId() const { return o_from_node_.GetId(); };
  const std::string& GetToNodeId() const { return o_to_node_.GetId(); };
  const std::string& GetFromNodeName() const { return o_from_node_.GetName(); };
  const std::string& GetToNodeName() const { return o_to_node_.GetName(); };
  const Pose& GetFromNodePose() const { return o_from_node_.GetConstPose(); };
  const Pose& GetToNodePose() const { return o_to_node_.GetConstPose(); };
  const DriveInfo& GetDriveInfo() const { return o_drive_info_; };
  MPTL::MOVEDIR GetMoveDir() const { return o_drive_info_.GetMoveDir(); };
  MPTL::DRIVE GetDriveType() const { return o_drive_info_.GetDriveType(); };
  MPTL::START GetStartType() const { return o_drive_info_.GetStartType(); };
  MPTL::GOAL GetGoalType() const { return o_drive_info_.GetGoalType(); };
  MPTL::EDGE_PATH GetPathType() const { return e_path_type_; };
  const Pose& GetFromNodeControl() const { return o_from_node_control_; };
  const Pose& GetToNodeControl() const { return o_to_node_control_; };
  const AvoidInfo& GetAvoidInfo() const { return o_avoid_info_; };
  double GetTimeControl() const { return d_T_control_; };
  double GetTargetSpd() const { return d_target_spd_; };
  double GetRobotYaw() const { return d_robot_yaw_; };
  double GetLength() const { return d_length_m_; };
  int GetEndIdx() const { return n_end_idx_; };

  void SetFromNode(const NodeInfo& o_node) { o_from_node_ = o_node; };
  void SetToNode(const NodeInfo& o_node) { o_to_node_ = o_node; };
  void SetFromNode(const std::string& s_id, const std::string& s_name, const Pose& o_pose);
  void SetToNode(const std::string& s_id, const std::string& s_name, const Pose& o_pose);
  void SetFromNodeId(const std::string& s_id) { o_from_node_.SetId(s_id); };
  void SetToNodeId(const std::string& s_id) { o_to_node_.SetId(s_id); };
  void SetFromNodeName(const std::string& s_name) { o_from_node_.SetName(s_name); };
  void SetToNodeName(const std::string& s_name) { o_to_node_.SetName(s_name); };
  void SetFromNodePose(const Pose& o_pose) { o_from_node_.SetPose(o_pose); };
  void SetToNodePose(const Pose& o_pose) { o_to_node_.SetPose(o_pose); };
  void SetDriveInfo(const DriveInfo& o_drive_info) { o_drive_info_ = o_drive_info; };
  void SetMoveDir(const MPTL::MOVEDIR& e_move_dir) { o_drive_info_.SetMoveDir(e_move_dir); };
  void SetDriveType(const MPTL::DRIVE& e_drive_type) { o_drive_info_.SetDriveType(e_drive_type); };
  void SetStartType(const MPTL::START& e_start_type) { o_drive_info_.SetStartType(e_start_type); };
  void SetGoalType(const MPTL::GOAL& e_goal_type) { o_drive_info_.SetGoalType(e_goal_type); };
  void SetPathType(MPTL::EDGE_PATH e_path_type) { e_path_type_ = e_path_type; };
  void SetFromNodeControl(const Pose& o_pos) { o_from_node_control_ = o_pos; };
  void SetToNodeControl(const Pose& o_pos) { o_to_node_control_ = o_pos; };
  void SetAvoidInfo(const AvoidInfo& o_avoid_info) { o_avoid_info_ = o_avoid_info; };
  void SetTimeControl(double d_T_control) { d_T_control_ = d_T_control; };
  void SetTargetSpd(double d_target_spd) { d_target_spd_ = d_target_spd; };
  void SetRobotYaw(double d_robot_yaw) { d_robot_yaw_ =  d_robot_yaw; };
  void SetLength(double d_length_m) { d_length_m_ = d_length_m; };
  void SetEndIdx(int n_end_idx) { n_end_idx_ = n_end_idx; };

  bool IsValid(TypeHandler::Ptr o_type_handler_ptr, int n_ref_path_size) const;
  bool IsValid(TypeHandler::Ptr o_type_handler_ptr) const;
  bool IsValid(int n_ref_path_size) const;
  bool IsSerial(const MissionEdge& o_next_edge) const;

  double GetFromNodeYaw() const;
  double GetToNodeYaw() const;

  MissionEdge Reverse();

  /**
   * @brief remove duplicated edges
   * @param front previous edges
   * @param rear  new edges
   * @param n_overlap_lvl default=0, [0: one overlap with last], [1: all overlap from first]
   * @return true (success) / false (fail)
   */
  static bool RemoveOverlapEdges(
    const std::vector<MissionEdge>& front,
    std::vector<MissionEdge>& rear,
    int n_overlap_lvl=0);

  bool operator==(const MissionEdge& rhs) const;
  bool operator!=(const MissionEdge& rhs) const;

  virtual std::string toStr() const;
  friend std::ostream& operator<<(std::ostream& os, const MissionEdge& o);

protected:
  // node info
  NodeInfo o_from_node_;
  NodeInfo o_to_node_;

  // drive info
  DriveInfo o_drive_info_;

  // gen_path info
  MPTL::EDGE_PATH e_path_type_;
  // LINE   : o_from_node_pos(x,y), o_to_node_pos(x,y)
  // ARC    : o_from_node_pos(x,y,yaw), o_to_node_pos(x,y,yaw)
  // BEZIER : o_from_node_pos(x,y), o_from_node_control(x,y), o_to_node_pos(x,y), o_to_node_control(x,y)
  Pose o_from_node_control_;
  Pose o_to_node_control_;
  // POLY   : o_from_node_pos(x,y,yaw,curv), o_to_node_pos(x,y,yaw,curv), d_T_control(time)
  double d_T_control_;

  // avoid info
  AvoidInfo o_avoid_info_;

  double d_target_spd_; // target speed [m/s]

  double d_robot_yaw_; // robot angle for quad

  double d_length_m_; // length of path

  int n_end_idx_;

private:

};

} // namespace NVFR

#endif
