/*
 * @file	: mission_handler.hpp
 * @date	: Feb 28, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: mission handler, ex) mission_queue, mission_exception, mission_path_filter, ect.
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef MISSION_HANDLER_HPP_
#define MISSION_HANDLER_HPP_

#include <stdint.h>
#include <vector>
#include <queue>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "utils/timer.hpp"
#include "utils/pose.hpp"
#include "utils/motion_planner_type_list.hpp"
#include "utils/motion_planner_type.hpp"

#include "utils/data_storage.hpp"

#include "mission_manager/mission.hpp"

namespace NVFR {

class MissionHandler
{
public:
  using Ptr = std::shared_ptr<MissionHandler>;
  using ConstPtr = std::shared_ptr<const MissionHandler>;

  MissionHandler(TypeHandler::Ptr o_type_handler_ptr);
  virtual ~MissionHandler();

  Mission::ConstPtr GetNextMissionPtr() const;
  void PopMissionPtr(Mission::ConstPtr o_mission_ptr);

  bool IsNewMission() const;
  bool IsEmptyMissionStack() const;
  size_t MissionStackSize() const;

  /**
   * @brief push mission into qu_mission
   * @param st_mission_bundle mission bundle
  */
  void PushMission(const MissionBundle_t& st_mission_bundle);

  /**
   * @brief clear all mission stack
  */
  void ClearMission();

  /**
   * @brief qu_mission to vec_mission
  */
  bool UpdateMission();

private:
  std::queue<Mission::Ptr> qu_tmp_mission_ptr_;
  std::vector<Mission::Ptr> vec_mission_ptr_list_;

  // type
  TypeHandler::Ptr o_type_handler_ptr_;

  // mutex
  mutable std::mutex mtx_;

};

} // namespace NVFR

#endif
