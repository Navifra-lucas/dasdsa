/*
 * @file	: motion_signal.hpp
 * @date	: Mar 8, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Signals from human or ACS
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef MOTION_SIGNAL_HPP_
#define MOTION_SIGNAL_HPP_

#include <memory>
#include <mutex>

#include "utils/singleton_generator.hpp"

namespace NVFR {

class MotionSignal : public NaviFra::SingletonGenerator<MotionSignal>
{
public:
  MotionSignal();
  virtual ~MotionSignal() = default;

  bool IsPause() const;

  bool IsDriveSpeedDown() const;
  bool IsAlignSpeedDown() const;

  int GetDriveSpeedPercent() const;
  int GetAlignSpeedPercent() const;

  void Reset();

  void Pause();
  void Resume();

  bool SetDriveSpeedPercent(int n_percent);
  bool SetAlignSpeedPercent(int n_percent);

  void ResetDriveSpeedPercent();
  void ResetAlignSpeedPercent();

private:
  bool b_pause_;

  int n_drive_percent_;
  int n_align_percent_;

  mutable std::mutex mtx_;
};

} // namespace NVFR

#endif
