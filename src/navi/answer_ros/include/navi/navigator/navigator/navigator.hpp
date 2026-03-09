/*
 * @file	: navigator.hpp
 * @date	: Feb 28, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Decision Maker class for path planner and motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIGATOR_HPP_
#define NAVIGATOR_HPP_

#include <stdint.h>
#include <vector>
#include <memory>
#include <atomic>
#include <condition_variable>
#include <mutex>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "utils/debug_visualizer.hpp"
#include "utils/motion_planner_type.hpp"
#include "utils/timer.hpp"
#include "utils/pose.hpp"
#include "utils/robot_config.hpp"
#include "utils/mission_bundle.hpp"
#include "utils/publish_callback.hpp"
#include "utils/loop_thread_handler.hpp"
#include "utils/grid_map/grid_map_storage.hpp"
#include "utils/param/navigator_param.hpp"

#include "sensor/pcl/pcl_cluster.hpp"
#include "mission_manager/mission_handler.hpp"

#include "navigator/core_state.hpp"
#include "navigator/mission_stack_manager.hpp"
#include "navigator/motion_executor.hpp"

namespace NVFR {

class Navigator : public GridMapStorage
{
public:
  Navigator(MPTL::KINEMATICS kinematics_type, MPTL::CONTROLLER controller_type, MPTL::AVOID avoid_type);
  Navigator(MPTL::KINEMATICS kinematics_type, MPTL::CONTROLLER controller_type, std::vector<MPTL::AVOID> avoid_type_list);
  virtual ~Navigator();
  void Terminator();

  // regist callback functions
  template <typename ClassType, typename... Args>
  bool RegistCbFunc(int n_key, void (ClassType::*func)(const Args& ...), ClassType* obj)
  {
    return PublishCb<Args ...>::GetInstance()->RegistCbFunc(n_key, [obj, func](const Args& ...args) { (obj->*func)(args...); });
  }

  // check data set (subscriptions)
  int CheckDataSet();

  // parameters
  void WriteParam(const NavigatorParam_t& st_param);
  void ReadParam();

  // input signal
  void Pause() { MotionSignal::GetInstance()->Pause(); };
  void Resume() { MotionSignal::GetInstance()->Resume(); };
  bool SetDriveSpeedPercent(int n_percent) { return MotionSignal::GetInstance()->SetDriveSpeedPercent(n_percent); };
  bool SetAlignSpeedPercent(int n_percent) { return MotionSignal::GetInstance()->SetAlignSpeedPercent(n_percent); };
  void ResetDriveSpeedPercent() { MotionSignal::GetInstance()->ResetDriveSpeedPercent(); };
  void ResetAlignSpeedPercent() { MotionSignal::GetInstance()->ResetAlignSpeedPercent(); };

  // input mission
  void Abort();
  void Cancel();
  void OneCycleStop();
  void PushMission(const MissionBundle_t& st_mission_bundle);

  // set robot state
  void SetRobotPos(double d_x_m, double d_y_m, double d_yaw_rad);
  void SetRobotVel(double d_vx_m_s, double d_vy_m_s, double d_vw_rad_s);

  // set global map & local map
  bool UpdateGlobalMap(const std::vector<int8_t>& map, const MapInfo_t& st_map_info);
  bool SetLocalSensorPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

private:
  // init
  void Initialize(TypeHandler::Ptr);

  // [notify callback functions]
  bool UpdateMission();
  bool MissionDone();
  bool IsRunning();
  bool MotionDone(bool b_abort);

  // start motion executor by status
  bool StartMotion(MissionBundle_t&);

  // modify mission of motion executor
  bool ModifyMission(MissionBundle_t&);

  // state
  CoreState o_core_state_;

  // object
  std::shared_ptr<TypeHandler> o_type_handler_ptr_;
  std::shared_ptr<MissionStackManager> o_mission_stack_manager_ptr_;
  std::shared_ptr<MotionExecutor> o_motion_executor_ptr_;
  PclCluster o_pcl_cluster_;

  // param
  NavigatorParam_t st_param_;
  std::unique_ptr<NavigatorParam_t> st_new_param_ptr_ = nullptr;
  std::mutex mtx_param_;

  // robot state data
  Pose o_robot_state_;

};

} // namespace NVFR

#endif
