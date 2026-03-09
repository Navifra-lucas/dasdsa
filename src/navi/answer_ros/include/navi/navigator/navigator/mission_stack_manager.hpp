/*
 * @file	: mission_stack_manager.hpp
 * @date	: Feb 28, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Decision Maker class for path planner and motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef MISSION_STACK_MANAGER_HPP_
#define MISSION_STACK_MANAGER_HPP_

#include <vector>
#include <memory>
#include <condition_variable>
#include <mutex>

#include "utils/observer_pattern.hpp"
#include "utils/notify_cb_func.hpp"
#include "utils/loop_thread_handler.hpp"
#include "utils/motion_planner_type.hpp"
#include "utils/mission_bundle.hpp"

#include "navigator/core_state.hpp"

#include "mission_manager/mission_handler.hpp"

namespace NVFR {

class MissionStackManager : public Observer<MPTL::MISSION,MPTL::INTERRUPTION>
{
public:
  MissionStackManager(TypeHandler::Ptr);
  virtual ~MissionStackManager();
  void Terminator();

  // observer
  void Update(MPTL::MISSION, MPTL::INTERRUPTION) override;

  // notify callback functions
  bool RegistCbFuncVoid(const std::string& s_name, std::function<bool()> func);
  bool RegistCbFuncMB(const std::string& s_name, std::function<bool(MissionBundle_t&)> func);

  // input mission
  void PushMission(const MissionBundle_t& st_mission_bundle);

  // get/pop mission
  Mission::ConstPtr GetNextMissionPtr() const;
  void PopMissionPtr(Mission::ConstPtr);

private:
  // notify callback functions
  bool Notify(const std::string& s_name);
  bool Notify(const std::string& s_name, MissionBundle_t& st_mission_bundle);

  // main thread
  void UpdateMissionThread();

  // motion by mission
  void StartMission();
  void ModifyMission(); // true : pop mission

  // state data
  MPTL::MISSION e_mission_type_;
  bool b_interruption_;
  std::mutex mtx_state_;
  bool b_recheck_mission_;

  // object
  MissionHandler o_mission_handler_;
  std::unique_ptr<LoopThread> o_loop_thread_ptr_;

  // condition_variable (lock)
  std::condition_variable cv_;
  std::mutex mtx_cv_;

  // notify callback functions
  NotifyCbFunc<void> o_cb_func_void_;
  NotifyCbFunc<MissionBundle_t&> o_cb_func_MB_;

};

} // namespace NVFR

#endif
