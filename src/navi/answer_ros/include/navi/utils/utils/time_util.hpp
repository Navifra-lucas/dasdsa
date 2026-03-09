
/*
 * @file	: time_util.hpp
 * @date	: Mar. 4, 2024
 * @author	: "HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: system clock util
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_TIME_UTIL_HPP_
#define NAVIFRA_TIME_UTIL_HPP_

#include <chrono>

using std::chrono::milliseconds;
using std::chrono::microseconds;
using std::chrono::nanoseconds;
using std::chrono::steady_clock;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::time_point_cast;

namespace NVFR {
namespace TimeUtil {

typedef typename steady_clock::time_point _tp;

inline _tp GetNow()
{
  return steady_clock::now();
}

inline int64_t Sec2NanoSec(double d_sec) noexcept
{
  return static_cast<int64_t>(d_sec * 1E9);
}

inline int64_t Sec2MicroSec(double d_sec) noexcept
{
  return static_cast<int64_t>(d_sec * 1E6);
}

inline int64_t Sec2MilliSec(double d_sec) noexcept
{
  return static_cast<int64_t>(d_sec * 1E3);
}

inline double NanoSec2Sec(int64_t n_nano_sec) noexcept
{
  return static_cast<double>(n_nano_sec) / 1E9;
}

inline double MicroSec2Sec(int64_t n_micro_sec) noexcept
{
  return static_cast<double>(n_micro_sec) / 1E6;
}

inline double MilliSec2Sec(int64_t n_milli_sec) noexcept
{
  return static_cast<double>(n_milli_sec) / 1E3;
}

inline _tp MoveTpDNanoSec(const _tp& tp_start, int64_t n_nano_sec) noexcept
{
  duration<int64_t, std::nano> du(n_nano_sec);
  nanoseconds du_micro_sec = duration_cast<nanoseconds>(du);
  return time_point_cast<nanoseconds>(tp_start + du_micro_sec);
}

inline _tp MoveTpDMicroSec(const _tp& tp_start, int64_t n_micro_sec) noexcept
{
  duration<int64_t, std::micro> du(n_micro_sec);
  microseconds du_micro_sec = duration_cast<microseconds>(du);
  return time_point_cast<microseconds>(tp_start + du_micro_sec);
}

inline _tp MoveTpDMilliSec(const _tp& tp_start, int64_t n_mill_sec) noexcept
{
  duration<int64_t, std::milli> du(n_mill_sec);
  milliseconds du_micro_sec = duration_cast<milliseconds>(du);
  return time_point_cast<milliseconds>(tp_start + du_micro_sec);
}

inline _tp MoveTpDSec(const _tp& tp_start, double d_sec) noexcept
{
  duration<double> du(d_sec);
  milliseconds du_micro_sec = duration_cast<milliseconds>(du);
  return time_point_cast<milliseconds>(tp_start + du_micro_sec);
}

inline int64_t GetNanoSec(const _tp& tp_start) noexcept
{
  return duration_cast<nanoseconds>(steady_clock::now() - tp_start).count();
}

inline int64_t GetMicroSec(const _tp& tp_start) noexcept
{
  return duration_cast<microseconds>(steady_clock::now() - tp_start).count();
}

inline int64_t GetMilliSec(const _tp& tp_start) noexcept
{
  return duration_cast<milliseconds>(steady_clock::now() - tp_start).count();
}

inline double GetSec(const _tp& tp_start) noexcept
{
  int64_t n_milli_sec = GetMilliSec(tp_start);
  return MilliSec2Sec(n_milli_sec);
}

inline int64_t GetNanoSec(const _tp& tp_start, const _tp& tp_end) noexcept
{
  return duration_cast<nanoseconds>(tp_end - tp_start).count();
}

inline int64_t GetMicroSec(const _tp& tp_start, const _tp& tp_end) noexcept
{
  return duration_cast<microseconds>(tp_end - tp_start).count();
}

inline int64_t GetMilliSec(const _tp& tp_start, const _tp& tp_end) noexcept
{
  return duration_cast<milliseconds>(tp_end - tp_start).count();
}

inline double GetSec(const _tp& tp_start, const _tp& tp_end) noexcept
{
  return MilliSec2Sec(GetMilliSec(tp_start, tp_end));
}

} // namespace TimeUtil

} // namespace NVFR

#endif
