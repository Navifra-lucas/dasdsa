/*
 * @file	: following_path_manager.hpp
 * @date	: Feb 28, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Path Manager for current path state
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef FOLLOWING_PATH_MANAGER_HPP_
#define FOLLOWING_PATH_MANAGER_HPP_

#include <vector>
#include <memory>
#include <mutex>

#include "utils/timer.hpp"
#include "utils/pose.hpp"
#include "utils/motion_planner_type.hpp"
#include "utils/drive_info.hpp"
#include "utils/mission_edge.hpp"
#include "utils/data_storage.hpp"
#include "utils/param/sub/local_planner_param.hpp"

namespace NVFR {

class FollowingPathManager
{
public:
  /**
   * @note type: uint8_t
   * @param IDLE, AVOID, CB, FCB
  */
  enum class ASTATUS : uint8_t
  {
    IDLE = 0,
    AVOID = 1,
    CB,
    FCB
  };
  FollowingPathManager();
  virtual ~FollowingPathManager() {};

  void SetParam(const LocalPlannerParam_t& st_param);

  bool IsDone() const;
  const std::vector<MissionEdge>& GetMissionEdgeArr() const { return vec_mission_edge_; };
  const MissionEdge* GetCurrMissionEdgePtr(int n_robot_idx) const;
  DriveInfo GetDriveInfo() const;
  const Path& GetRefPath() const { return ref_path_; };
  const Path& GetCurrPath() const { return curr_path_; };
  const Path* GetCurrPathPtr() const { return &curr_path_; };
  int GetRobotIdxInRefPath() const { return n_ref_robot_idx_; };
  int GetRobotIdxInCurrPath() const { return n_curr_robot_idx_; };
  int GetStartIdxInCurrPath() const { return n_curr_start_idx_; };
  int GetTargetIdxInRefPath() const { return n_ref_target_idx_; };
  int GetEndAvoidIdxInCurrPath() const { return n_curr_end_avoid_idx_; };
  int GetSaftyStopIdxInCurrPath() const { return n_curr_safty_stop_idx_; };

  bool RobotIsAvoidWaypoint() const { return (n_curr_end_avoid_idx_ > n_curr_robot_idx_); };
  bool IsSaftyStop() const { return (n_curr_safty_stop_idx_ != -1); };
  bool IsAbletoAvoid() const { return e_avoid_status_ != ASTATUS::IDLE; };
  bool IsReplanTime() const { return o_rapid_timer_.CheckOver("Replan"); };

  bool SetMission(const DriveInfo&, std::vector<MissionEdge>&, Path&, const Pose&);
  bool ModifyMission(const DriveInfo&, std::vector<MissionEdge>&, Path&);
  bool UpdateMission();

  void Reset();

  bool SetAvoidPath(Path& avoid_path);
  bool SetAvoidStatus(ASTATUS e_avoid_status);
  void TrySearch(const Pose& o_robot_state);

  void SetRefTargetIdx(int n_ref_target_idx) { n_ref_target_idx_ = n_ref_target_idx; };
  void SetSaftyStopIdx(int n_collision_idx);

  void UpdateRobotIdx(const Pose& o_robot_pos);
  void UpdateStartIdx(const Pose& o_robot_state);
  void UpdateCurrPath();
  void Update(const Pose& o_robot_pos);

private:
  Path CreateCurrPath(const Path& ref_path, const Pose& o_robot_state, int n_ref_robot_idx, const LocalPlannerParam_t& st_param) const;
  void extentionCurrentPath(const LocalPlannerParam_t& st_param);
  void UpdateCurrPathWithAvoidPath(const Path& avoid_path, const LocalPlannerParam_t& st_param);

  // drive mission info
  DriveInfo o_drive_info_;
  std::vector<MissionEdge> vec_mission_edge_;

  // ref path
  Path ref_path_;
  int n_ref_robot_idx_=-1;          // index of closest waypoint with robot in reference path
  int n_ref_end_curr_idx_=-1;       // index of final waypoint of current path in reference path
  int n_ref_target_idx_=-1;         // tartget index in reference path for path planning

  // sub mission
  DriveInfo o_sub_drive_info_;
  std::vector<MissionEdge> vec_sub_mission_edge_;
  Path sub_ref_path_;
  mutable std::mutex mtx_sub_path_;

  // avoid_path
  Path avoid_path_;
  ASTATUS e_avoid_status_ = ASTATUS::IDLE;

  // curr path
  Path curr_path_;
  int n_curr_robot_idx_=-1;         // index of closest waypoint with robot in current path
  int n_curr_prev_robot_idx_=-1;    // previous index of closest waypoint with robot in current path
  int n_curr_start_idx_=-1;         // start index in current path for path planning
  int n_curr_end_avoid_idx_=-1;     // final index of avoidance in current path
  int n_curr_safty_stop_idx_=-1;    // safty stop index in current path

  // param
  LocalPlannerParam_t st_param_;

  // timer
  RapidTimer o_rapid_timer_;
};

} // namespace NVFR

#endif
