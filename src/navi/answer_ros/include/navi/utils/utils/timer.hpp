
/*
 * @file	: timer.hpp
 * @date	: Mar. 4, 2024
 * @author	: "HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: timer based on system clock
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_TIMER_HPP_
#define NAVIFRA_TIMER_HPP_

#include <chrono>
#include <string>
#include <map>
#include <memory>

#include "utils/time_util.hpp"

namespace NVFR {

class Timer
{
public:
  using Ptr = std::shared_ptr<Timer>;

  Timer(std::string s_name);
  Timer(int n_duration_msec);
  Timer(double d_duration_sec);
  Timer(std::string s_name, int n_duration_msec);
  Timer(std::string s_name, double d_duration_sec);
  virtual ~Timer() = default;

  void SetName(const std::string& s_name);
  void SetTimer(int n_duration_msec);
  void SetTimer(double d_duration_sec);
  void Start();
  bool CheckOver(bool b_display=false) const;
  bool Over(bool b_display=false) const;
  double CheckTime() const;
  void ShowTime() const;

private:
  std::string s_name_;
  int64_t n_duration_msec_;
  mutable std::chrono::steady_clock::time_point tp_start_;
};

class RapidTimer
{
public:
  using Ptr = std::shared_ptr<RapidTimer>;

  RapidTimer() {};
  virtual ~RapidTimer();

  bool Regist(const std::string& s_key, double d_duration_sec);

  void SetTimer(const std::string& s_key, double d_duration_sec);
  void Start(const std::string& s_key);
  bool CheckOver(const std::string& s_key, bool b_display=false) const;
  bool Over(const std::string& s_key, bool b_display=false) const;
  double CheckTime(const std::string& s_key) const;
  void ShowTime(const std::string& s_key) const;

private:
  std::map< std::string, Timer::Ptr > map_timer_ptr_;
};

} // namespace NVFR

#endif
