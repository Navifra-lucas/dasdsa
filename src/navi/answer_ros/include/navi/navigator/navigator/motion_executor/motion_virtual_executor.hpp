/*
 * @file	: motion_virtual_executor.hpp
 * @date	: Feb 28, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Decision Maker class for path planner and motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef MOTION_VIRTUAL_EXECUTOR_HPP_
#define MOTION_VIRTUAL_EXECUTOR_HPP_

#include <stdint.h>
#include <vector>
#include <map>
#include <memory>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "utils/debug_visualizer.hpp"
#include "utils/key_list.hpp"
#include "utils/data_storage.hpp"
#include "utils/publish_callback.hpp"
#include "utils/notify_cb_func.hpp"
#include "utils/loop_thread_handler.hpp"
#include "utils/timer.hpp"
#include "utils/system_time_analysis.hpp"
#include "utils/pose.hpp"
#include "utils/motion_planner_type.hpp"
#include "utils/navi_info.hpp"
#include "utils/mission_bundle.hpp"
#include "utils/grid_map/grid_map_storage.hpp"
#include "utils/param/navigator_param.hpp"

#include "navigator/core_state.hpp"
#include "navigator/signal/motion_signal.hpp"

#include "collision_detector/safety_detector.hpp"
#include "navigator/motion_executor/following_path_manager.hpp"
#include "motion_controller/motion_handler.hpp"

namespace NVFR {

class MotionExecutor
  : public GridMapStorage
  , public Observer<MPTL::MISSION,MPTL::INTERRUPTION>
{
public:
  MotionExecutor(TypeHandler::Ptr o_type_handler_ptr, MissionBundle_t& st_mission_bundle, const NavigatorParam_t& st_param);
  virtual ~MotionExecutor();
  void Terminator();

  // observer
  void Update(MPTL::MISSION, MPTL::INTERRUPTION) override;

  // mission
  bool IsWrongMission() const;
  virtual bool ModifyMission(MissionBundle_t& st_mission_bundle) = 0;

  // thread
  void Execution();

  // notify callback functions
  bool RegistCbFuncVoid(const std::string& s_name, std::function<bool()> func);
  bool RegistCbFuncBool(const std::string& s_name, std::function<bool(bool)> func);

protected:
  // init
  virtual void Initialize() = 0;

  // callback functions
  bool Notify(const std::string& s_name);
  bool Notify(const std::string& s_name, bool b_flag);

  // mission
  void WrongMission();

  // signal
  bool IsCancel() const;
  void Cancel();

  // motion
  virtual bool IsDone() const = 0;
  virtual void Motion() = 0;

  bool Monitoring() const;
  bool ExecuteSafetyDetection(TargetSpeedLimit_t& st_target_speed_limit);

  void PublishNaviInfo();
  bool CheckRotationCollision(
    const Pose& o_robot_state,
    const AlignInfo& o_align_info,
    double d_interval_rad,
    double& d_robot2collision_rad);
  bool CheckPathCollision(
    const Path& path,
    int n_closest_idx,
    int& n_collision_idx);

  // handler
  TypeHandler::Ptr o_type_handler_ptr_;
  PathManager o_following_path_manager_;
  MotionHandler o_motion_handler_;
  SafetyDetector o_safety_detector_;

  // main thread
  LoopThreadHandler o_loop_thread_handler_;

  // notify callback functions
  NotifyCbFunc<void> o_cb_func_void_;
  NotifyCbFunc<bool> o_cb_func_bool_;

  // mission bundle
  MissionBundle_t st_mission_bundle_;
  mutable std::mutex mtx_mission_bundle_;

  // param
  NavigatorParam_t st_param_;

  // navi info
  NaviInfo o_navi_info_;
  mutable std::mutex mtx_navi_info_;

  // visual config
  std::map<std::string, DrawInfo> map_draw_info_;

  // system time analysis
  RapidSystemTimeAnalysis o_rsta_;

private:
  bool b_wrong_mission_ = false;
  bool b_cancel_ = false;

};

} // namespace NVFR

#endif
