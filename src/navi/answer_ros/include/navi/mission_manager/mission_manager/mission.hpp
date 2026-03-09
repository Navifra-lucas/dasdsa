/*
 * @file	: mission.hpp
 * @date	: Feb 28, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: class for mission
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef MISSION_HPP_
#define MISSION_HPP_

#include <stdint.h>
#include <vector>
#include <memory>
#include <mutex>

#include "utils/motion_planner_type_list.hpp"
#include "utils/motion_planner_type.hpp"
#include "utils/mission_bundle.hpp"
#include "utils/pose.hpp"

namespace NVFR {

class Mission
{
public:
  using Ptr = std::shared_ptr<Mission>;
  using ConstPtr = std::shared_ptr<const Mission>;

  Mission() = default;
  Mission(const MissionBundle_t& st_mission_bundle);
  Mission(const Mission& o_mission);
  virtual ~Mission() {};

  MPTL::MISSION GetMissionType() const;
  AlignInfo GetAlignInfo() const;
  DriveInfo GetDriveInfo() const;
  MPTL::MOVEDIR GetMoveDir() const;
  MPTL::DRIVE GetDriveType() const;
  MPTL::START GetStartType() const;
  MPTL::GOAL GetGoalType() const;
  const std::vector<MissionEdge>& GetConstAllMissionEdge() const;
  std::vector<MissionEdge>& GetRefAllMissionEdge();
  std::vector<MissionEdge> GetAllMissionEdge() const;
  const MissionEdge& GetConstFirstMissionEdge() const;
  const MissionEdge& GetConstFinalMissionEdge() const;
  MissionEdge GetFirstMissionEdge() const;
  MissionEdge GetFinalMissionEdge() const;
  bool EdgeIsEmpty() const;
  size_t GetEdgeSize() const;
  const MissionBundle_t& GetConstMissionBundle() const;
  MissionBundle_t GetMissionBundle() const;

  /**
   * @brief check this mission is valid or not
   * @return true (valid) / false (invalid)
  */
  bool IsValid(TypeHandler::Ptr o_type_handler_ptr) const;

  /**
   * @brief set mission (bundle)
  */
  void SetMission(const MissionBundle_t& st_mission_bundle);

  /**
   * @brief devide discontinuous edges
   * @return if every edges are serial, size of vector is one, (self)
  */
  std::vector< std::shared_ptr<Mission> > DevideDiscontinueEdge() const;

  /**
   * @brief push back new mission into last mission, type of two mission are PATH
   * @note modify [n_end_idx in avoid info & s_m in the ref_path] in the new mission
   * @param added_mission new mission to push back into last mission
   * @return true (valid) / false (invalid)
  */
  bool PushMissionPath(const Mission& new_mission, TypeHandler::Ptr o_type_handler_ptr);

  /**
   * @brief Check new mission is serial or not (PATH)
   * @param o_new_mission_ptr new mission (PATH)
   * @return serial (true) / distribution (false)
  */
  bool IsSerialMission(const Mission& o_new_mission) const;

  /**
   * @brief update DriveInfo from std::vector<MissionEdge>
   * @return success (true) / fail (false)
  */
  bool UpdateDriveInfo();

  virtual std::string toStr() const;
  friend std::ostream& operator<<(std::ostream& os, const Mission& o);

private:
  MissionBundle_t st_mission_bundle_;

  mutable std::mutex mtx_mission_;
};

} // namespace NVFR

#endif
