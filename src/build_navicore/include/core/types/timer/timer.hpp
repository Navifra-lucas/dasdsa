
/*
 * @file	: timer.hpp
 * @date	: Mar. 4, 2024
 * @author	: "HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: timer based on system clock
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2024 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_TIMER_HPP_
#define NAVIFRA_TIMER_HPP_

#include <chrono>
#include <string>
#include <map>
#include <memory>

namespace NaviFra {

class Timer
{
public:
  Timer(const char* c);
  ~Timer();

  void SetTimer(double duration_sec);
  void Start();
  bool Over(bool b_display=false);
  double CheckTime();
  void ShowTime();

private:
  double duration_sec_=0.0;
  std::chrono::steady_clock::time_point set_time_;
  char* c_name_=nullptr;
};

class LogTimer
{
public:
  LogTimer() { tp_log_time_ = std::chrono::steady_clock::now(); };
  ~LogTimer() {};

  void SetParam(double d_log_interval_time);
  void Reset();

  void Start();
  void ShowTime(const char* c);

private:
  bool b_init_=true;

  double d_log_interval_time_=5.0;
  std::chrono::steady_clock::time_point tp_log_time_;
  std::chrono::steady_clock::time_point tp_start_time_;

  int n_cnt_=0;
  int n_sum_duration_msec_=0;
  int n_min_duration_msec_=1e9;
  int n_max_duration_msec_=0;
};

class RapidTimer
{
public:
  RapidTimer() {};
  ~RapidTimer() { map_timer_ptr_.clear(); map_logtimer_ptr_.clear(); };

  bool Over(std::string s_key, double d_duration, bool b_display=false);

  void Reset(std::string s_key);
  void Start(std::string s_key);
  void ShowTime(std::string s_key, double d_interval_time_sec);

private:
  std::map< std::string, std::shared_ptr<Timer> > map_timer_ptr_;
  std::map< std::string, std::shared_ptr<LogTimer> > map_logtimer_ptr_;
};

} // namespace NaviFra

#endif
