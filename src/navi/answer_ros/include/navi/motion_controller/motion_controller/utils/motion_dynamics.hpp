/*
 * @file	: motion_dynamics.hpp
 * @date	: Jan 10, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: motion_dynamics calculator (s, v, a(d), j, t)
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef MOTION_DYNAMICS_HPP_
#define MOTION_DYNAMICS_HPP_

#include "utils/pose.hpp"
#include "utils/common_math.hpp"

namespace NVFR {
namespace MotionDynamics {

// distance (S)

// velocity (V)
/*
  v1 = v0 + a*dt, where v1 <= v_max
*/
template <typename T>
inline T CalcVbyAT(T v0, T a, T dt, T v_max) {
  T v1 = v0 + a * dt;
  return (v1 > v_max) ? v_max : v1;
};
/*
  a1_max = a0 + j*dt, where a1_max <= a_max
  v1_max = v0 + (a1_max + a0)*dt / 2, where v1_max <= v_max
  vt <= v1_max
*/
template <typename T>
inline T FilterMaxVbyAJT(T tv, T v0, T a0, T v_max, T a_max, T j, T dt) {
  T a1_max = CalcAbyJT(a0, j, dt, a_max);
  T v1_max = v0 + static_cast<T>(0.5) * (a1_max + a0) * dt;
  if (v1_max > v_max) v1_max = v_max;
  return (tv > v1_max) ? v1_max : tv;
};
/*
  v1 - v0 < a^2 / (2*j)
*/
template <typename T>
inline bool IsNearDesiredVbyAJ(T dv, T a, T j) {
  return (dv < static_cast<T>(0.5) * a * a / j + static_cast<T>(1e-5));
};

// accel (A)
/*
  a1 = a0 + j*dt, where j > 0, a1 <= a_max
*/
template <typename T>
inline T CalcAbyJT(T a0, T j, T dt, T a_max) {
  T a1 = a0 + j * dt;
  return (a1 > a_max) ? a_max : a1;
};
/*
  a1 = a0 + j*dt, where j > 0, a1 <= a_max
*/
template <typename T>
inline T CalcAbySV(T ds, T v0, T v1) {
  return static_cast<T>(0.5) * (v1 + v0) * (v1 - v0) / ds;
};

// time (T)
/*
  v > v_min
  dt = (sqrt(v^2 + 2*a*ds) - v) / a, where a != 0, v != 0
  dt = ds / v, where a = 0
*/
template <typename T>
inline T CalcTbySVA(T ds, T v, T a) {
  T EPS = static_cast<T>(1e-5);
  if (v < EPS) v = EPS;
  if (a < EPS) return ds / v;
  return (std::sqrt(v*v + 2*a*ds) - v) / a;
};
/*
  s = (j/6)*t^3
*/
template <typename T>
inline T CalcTbyremainSJ(T s, T j) {
  return std::pow(static_cast<T>(6)*s/j, static_cast<T>(1.0/3.0));
};

// centri-effect
inline void CentriEffectForPath(Pose& o_curr_wp, const Pose& o_next_wp, double d_max_centri_acc, double d_max_centri_jrk) {
  double d_abs_curv = std::fabs(o_curr_wp.GetCurv());
  double d_dK = o_next_wp.GetCurv() - o_curr_wp.GetCurv();
  double d_ds = o_next_wp.GetSm() - o_curr_wp.GetSm();

  // centri-acc
  if (d_abs_curv > 0.001f) {
    double d_max_centri_acc_spd = std::sqrt(d_max_centri_acc / d_abs_curv);
    if (o_curr_wp.GetVX() > d_max_centri_acc_spd) o_curr_wp.SetVX(d_max_centri_acc_spd);
  }
  // centri-jrk
  if (d_dK*d_dK > 0.000001f && d_ds > 0.005f) {
    double d_max_centri_jrk_spd = std::pow(d_max_centri_jrk * d_ds / d_dK, 1.0/3.0);
    // if (o_curr_wp.GetVX() > d_max_centri_jrk_spd) o_curr_wp.SetVX(d_max_centri_jrk_spd);
  }
};

inline double CentriEffectForMotion(double d_curv, double d_max_centri_acc) {
  double d_abs_curv = std::fabs(d_curv);
  if (d_abs_curv > 0.001) d_abs_curv = 0.001;
  return std::sqrt(d_max_centri_acc / d_abs_curv);
};

inline void CurvSpeedFilter(Pose& o_curr_wp,
  double d_min_v_x, double d_max_v_x, double d_max_v_w) {
  // 1. consider linear min/max parameters
  double d_v_x = CM::LUBnd(o_curr_wp.GetVX(), d_min_v_x, d_max_v_x);
  double d_curv = std::abs(o_curr_wp.GetCurv());
  // 2. consider 
  if (d_curv > d_max_v_w / d_max_v_x) {
    d_v_x = CM::UBnd(d_v_x, d_max_v_w / d_curv);
  }
  o_curr_wp.SetVX(d_v_x);
};

inline void MotionSpeedFilter(double& d_v_x, double& d_v_w,
  double d_min_v_x, double d_max_v_x, double d_max_v_w, double d_max_curv) {
  int n_v_x_dir = d_v_x < 0.0 ? -1 : 1;
  int n_v_w_dir = d_v_w < 0.0 ? -1 : 1;
  d_v_x *= n_v_x_dir;
  d_v_w *= n_v_w_dir;
  // 0. exceptions
  if (d_v_x < 1e-5) return;
  // 1. calc curvature
  double d_curv = d_v_w / d_v_x;
  // 2. consider minimum linear speed
  if (d_v_x < d_min_v_x) {
    d_v_x = d_min_v_x;
    d_v_w = d_min_v_x * d_curv;
  }
  // 3. consider maximum parameters
  double d_r1 = 1.0;
  double d_r2 = 1.0;
  double d_r3 = 1.0;
  if (d_v_x > d_max_v_x) {
    d_r1 = d_max_v_x / d_v_x;
  }
  if (d_v_w > d_max_v_w) {
    d_r2 = d_max_v_w / d_v_w;
  }
  if (d_curv > d_max_v_w / d_max_v_x) {
    double d_max_tmp = d_max_v_w / d_curv;
    if (d_v_x > d_max_tmp) {
      d_r3 = d_max_tmp / d_v_x;
    }
  } else {
    double d_max_tmp = d_max_v_x * d_curv;
    if (d_v_w > d_max_tmp) {
      d_r3 = d_max_tmp / d_v_w;
    }
  }
  double d_r = std::min(1.0, d_r1);
  d_r = std::min(d_r, d_r2);
  d_r = std::min(d_r, d_r3);
  d_v_x *= n_v_x_dir * d_r;
  d_v_w *= n_v_w_dir * d_r;
};

} // namespace MotionDynamics
namespace MDS = MotionDynamics;
} // namespace NVFR

#endif
