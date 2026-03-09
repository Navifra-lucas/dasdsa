
/*
 * @file	: system_time_analysis.hpp
 * @date	: Mar. 4, 2024
 * @author	: "HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: timer based on system clock
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_TIME_ANALYSIS_HPP_
#define NAVIFRA_TIME_ANALYSIS_HPP_

#include <chrono>
#include <string>
#include <map>
#include <memory>

#include "utils/time_util.hpp"

namespace NVFR {

class SystemTimeAnalysis
{
public:
  using Ptr = std::shared_ptr<SystemTimeAnalysis>;

  SystemTimeAnalysis(std::string s_name);
  SystemTimeAnalysis(int n_interval_msec);
  SystemTimeAnalysis(double d_interval_sec);
  SystemTimeAnalysis(std::string s_name, int n_interval_msec);
  SystemTimeAnalysis(std::string s_name, double d_interval_sec);
  virtual ~SystemTimeAnalysis() = default;

  void SetName(const std::string& s_name);
  void SetInterval(int n_interval_msec);
  void SetInterval(double d_interval_sec);
  void Reset();
  void Start();
  void Analysis() const;

private:
  mutable bool b_init_=true;

  mutable int64_t n_cnt_=0;
  mutable int64_t n_sum_duration_msec_=0;
  mutable int64_t n_min_duration_msec_=static_cast<unsigned int>(1E6);
  mutable int64_t n_max_duration_msec_=0;

  std::string s_name_;
  int64_t n_interval_msec_;
  mutable std::chrono::steady_clock::time_point tp_start_;
  mutable std::chrono::steady_clock::time_point tp_timer_;
};

class RapidSystemTimeAnalysis
{
public:
  using Ptr = std::shared_ptr<RapidSystemTimeAnalysis>;

  RapidSystemTimeAnalysis() {};
  virtual ~RapidSystemTimeAnalysis();

  bool Regist(const std::string& s_key, double d_interval_sec);

  void SetInterval(const std::string& s_key, double d_interval_sec);
  void Reset(const std::string& s_key);
  void Start(const std::string& s_key);
  void Analysis(const std::string& s_key) const;

private:
  std::map< std::string, SystemTimeAnalysis::Ptr > map_sta_ptr_;
};

} // namespace NVFR

#endif
