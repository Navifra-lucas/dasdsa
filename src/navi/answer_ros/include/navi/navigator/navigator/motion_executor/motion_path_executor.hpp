/*
 * @file	: motion_path_executor.hpp
 * @date	: Feb 28, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Decision Maker class for path planner and motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef MOTION_PATH_EXECUTOR_HPP_
#define MOTION_PATH_EXECUTOR_HPP_

#include "navigator/motion_executor/motion_virtual_executor.hpp"

#include "path_planner/edge_planner/edge_array_generator.hpp"

namespace NVFR {

class MotionPathExecutor : public MotionExecutor
{
public:
  MotionPathExecutor(TypeHandler::Ptr o_type_handler_ptr, MissionBundle_t& st_mission_bundle, const NavigatorParam_t& st_param);
  virtual ~MotionPathExecutor();

  virtual bool ModifyMission(MissionBundle_t& st_mission_bundle) override;

protected:
  // init
  virtual void Initialize() override;

  // done condition
  virtual bool IsDone() const override;
  virtual void Motion() override;

private:
  enum class MotionStage {
    START,
    ALIGN,
    PATH,
    SPIN,
    END,
  } e_motion_stage_;
  bool MotionAlign();
  bool MotionPath();
  bool MotionSpin();

  bool IsAligning(const MotionHandler& o_motion_handler, const Pose& o_closest_wp, const Pose& o_robot_state) const;

  void UpdateNaviInfo(Pose o_robot_state, const DriveInfo& o_drive_info,
    bool b_pause, bool b_stop, bool b_obstacle, bool b_collision, bool b_away_from_path,
    const TargetSpeedLimit_t& st_target_speed_limit,
    double d_align_target_deg);

  void Avoid();

  bool IsAbletoAvoid(const PathManager& o_local_path_handler);

  std::condition_variable cv_avoid_;
  mutable std::mutex mtx_avoid_;

};

} // namespace NVFR

#endif
