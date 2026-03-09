/*
 * @file	: core_state.hpp
 * @date	: Mar 8, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Signals from human or ACS
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef CORE_STATE_HPP_
#define CORE_STATE_HPP_

#include <stdint.h>
#include <atomic>
#include <mutex>

#include "utils/singleton_generator.hpp"
#include "utils/observer_pattern.hpp"
#include "utils/motion_planner_type_list.hpp"

namespace NVFR {

class CoreState : public Subject<MPTL::MISSION,MPTL::INTERRUPTION>
{
public:
  enum class CHECK_LIST : int {
    MISSION = 0x0001,
    MOTION = 0x0010,
  };
  CoreState();
  virtual ~CoreState() = default;

  // all status
  void Reset();
  bool Done(CHECK_LIST);
  bool IsDone();

  // mission
  MPTL::MISSION GetMissionType() const;
  bool IsRunning() const;
  bool SetMissionType(MPTL::MISSION);
  void UpdateMission();

  // abort
  bool IsAbort() const;
  bool IsAbort(CHECK_LIST) const;
  bool Abort();

  // cancel
  bool IsCancel() const;
  bool IsCancel(CHECK_LIST) const;
  bool Cancel();

  // onecyclestop
  bool IsOneCycleStop() const;
  bool IsOneCycleStop(CHECK_LIST) const;
  bool OneCycleStop();

  // interruption
  bool IsInterruption() const;
  bool IsInterruption(CHECK_LIST) const;

private:
  MPTL::MISSION e_mission_type_;
  MPTL::INTERRUPTION e_interruption_;

  /*
   if cancel or onecyclestop is true, check n_done_bit_ is equal to n_check_list_ or not
   0: not yet
   1: done
   example)
   - n_check_list_ : 0x0111 means list are 0x0001, 0x0010, 0x0100
   - n_done_bit_ : 0x0101 means 0x0001 and 0x0100 are done
   - if n_done_bit_ == n_check_list_, all done
  */
  int n_done_bit_;
  const int n_check_list_;

  mutable std::mutex mtx_;

  // subject
  virtual void Notify() override;

  // bit calculator
  bool IsBitUp(int n_done_bit, int n_this_bit) const;
  bool IsBitUp(int n_done_bit, CHECK_LIST e) const;
  bool IsBitDown(int n_done_bit, int n_this_bit) const;
  bool IsBitDown(int n_done_bit, CHECK_LIST e) const;
  int BitUp(int n_done_bit, int n_this_bit) const;
  int BitUp(int n_done_bit, CHECK_LIST e) const;
  int BitDown(int n_done_bit, int n_this_bit) const;
  int BitDown(int n_done_bit, CHECK_LIST e) const;

};

} // namespace NVFR

#endif
