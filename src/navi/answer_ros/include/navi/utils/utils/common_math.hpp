
/*
 * @file	: common_math.hpp
 * @date	: Mar. 4, 2024
 * @author	: "HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: (pos,vel) calculator for motion controller, path planner and decision maker
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_COMMON_MATH_HPP_
#define NAVIFRA_COMMON_MATH_HPP_

#include <cmath>
#include <vector>

namespace NVFR {
namespace CommonMath{

static constexpr double TWO_PI = 2.0 * M_PI;
static constexpr double Deg2Rad = M_PI / 180.0;
static constexpr double Rad2Deg = 180.0 / M_PI;

template <typename T>
inline int Sign(T val) { return (val > 0) - (val < 0); };
template <typename T>
inline T Square(T v) { return v*v; };
template <typename T>
inline T Square(T x, T y) { return x*x + y*y; };
template <typename T>
inline T Length(T x, T y) { return std::sqrt(x*x + y*y); };

template <typename T>
inline T Mean(const std::vector<T>& vec)
{
  T size = static_cast<T>(vec.size());
  T sum = 0;
  for (const auto& v : vec) {
    sum +=v;
  }
  return sum / size;
}

template <typename... Args>
inline double Mean(double first, Args... rest)
{
  double sum = first + (rest + ...);
  return sum / static_cast<double>(1 + sizeof...(rest));
}

template <typename T>
inline T RMS(const std::vector<T>& vec)
{
  T size = static_cast<T>(vec.size());
  T sum_sq = 0;
  for (const auto& v : vec) {
    sum_sq +=v*v;
  }
  return std::sqrt(sum_sq / size);
}

template <typename... Args>
inline double RMS(double first, Args... rest)
{
  double sum_sq = first * first + ((rest * rest) + ...);
  return std::sqrt(sum_sq / static_cast<double>(1 + sizeof...(rest)));
}

inline double NormRad(double rad)
{
  int n_sign = Sign(rad);
  return n_sign * ( fmod(n_sign * rad + M_PI, TWO_PI) - M_PI );
}

inline double ms2s(int n_msec)
{
  return static_cast<double>(n_msec) / 1000.0;
}

inline int s2ms(double d_sec)
{
  return static_cast<int>(1000 * d_sec);
}

/**
 * @brief calculate convertion from Euler to Quaternion
 * @param roll, pitch, yaw [rad]
 * @return Quaternion(x,y,z,w) [std::vector<double>, rad]
*/
inline std::vector<double> Euler2Quaternion(double roll, double pitch, double yaw)
{
  std::vector<double> Q(4, 0.0);
  double cr = cos(0.5 * roll);
  double sr = sin(0.5 * roll);
  double cp = cos(0.5 * pitch);
  double sp = sin(0.5 * pitch);
  double cy = cos(0.5 * yaw);
  double sy = sin(0.5 * yaw);
  Q[0] = sr * cp * cy - cr * sp * sy; // Q.x
  Q[1] = cr * sp * cy + sr * cp * sy; // Q.y
  Q[2] = cr * cp * sy - sr * sp * cy; // Q.z
  Q[3] = cr * cp * cy + sr * sp * sy; // Q.w
  return Q;
};

/**
 * @brief calculate convertion from Euler to Quaternion
 * @param roll, pitch, yaw [rad]
 * @return Quaternion(x,y,z,w) [std::vector<double>, rad]
*/
inline std::vector<double> Yaw2Quaternion(double yaw)
{
  std::vector<double> Q(4, 0.0);
  Q[0] = 0.0; // Q.x
  Q[1] = 0.0; // Q.y
  Q[2] = sin(0.5 * yaw); // Q.z
  Q[3] = cos(0.5 * yaw); // Q.w
  return Q;
};

/**
 * @brief calculate convertion from Quaternion to Euler
 * @param qx, qy, qz, qw [rad]
 * @return Euler(roll,pitch,yaw) [std::vector<double>, rad]
*/
inline std::vector<double> Quaternion2Euler(double qx, double qy, double qz, double qw)
{
  std::vector<double> E(3, 0.0);
  // roll
  double sinr_cosp = 2 * (qw * qx + qy * qz);
  double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
  E[0] = std::atan2(sinr_cosp, cosr_cosp);
  // pitch
  double sinp = std::sqrt(1 + 2 * (qw * qy - qx * qz));
  double cosp = std::sqrt(1 - 2 * (qw * qy - qx * qz));
  E[1] = 2 * std::atan2(sinp, cosp) - M_PI / 2;
  // yaw
  double siny_cosp = 2 * (qw * qz + qx * qy);
  double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
  E[2] = std::atan2(siny_cosp, cosy_cosp);
  return E;
};

/**
 * @brief calculate convertion from Quaternion to Roll of Euler
 * @param qx, qy, qz, qw [rad]
 * @return roll(Euler) [double, rad]
*/
inline double Quaternion2Roll(double qx, double qy, double qz, double qw)
{
  double sinr_cosp = 2 * (qw * qx + qy * qz);
  double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
  return std::atan2(sinr_cosp, cosr_cosp);
};

/**
 * @brief calculate convertion from Quaternion to Pitch of Euler
 * @param qx, qy, qz, qw [rad]
 * @return pitch(Euler) [double, rad]
*/
inline double Quaternion2Pitch(double qx, double qy, double qz, double qw)
{
  double sinp = std::sqrt(1 + 2 * (qw * qy - qx * qz));
  double cosp = std::sqrt(1 - 2 * (qw * qy - qx * qz));
  return 2 * std::atan2(sinp, cosp) - M_PI / 2;
};

/**
 * @brief calculate convertion from Quaternion to Yaw of Euler
 * @param qx, qy, qz, qw [rad]
 * @return yaw(Euler) [double, 1/m]
*/
inline double Quaternion2Yaw(double qx, double qy, double qz, double qw)
{
  double siny_cosp = 2 * (qw * qz + qx * qy);
  double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
  return std::atan2(siny_cosp, cosy_cosp);
};

/**
 * @brief check value is lower than comparison
 * @param val variable to evaluate
 * @param cmp comparison value
 * @return true (val < cmp) / false
*/
template <typename T>
inline bool IsLower(T val, T cmp)
{
  return val < cmp;
};

/**
 * @brief check value is upper than comparison
 * @param val variable to evaluate
 * @param cmp comparison value
 * @return true (val > cmp) / false
*/
template <typename T>
inline bool IsUpper(T val, T cmp)
{
  return val > cmp;
};

/**
 * @brief check value is out of range from minimum to maximum
 * @param val variable to evaluate
 * @param min minimum value
 * @param max maximum value
 * @note must be [max] > [min]
 * @return true (val > max || val < min) / false
*/
template <typename T>
inline bool IsOutOfRange(T val, T min, T max)
{
  return val > max || val < min;
};

/**
 * @brief check absolute value is lower than comparison
 * @param val variable to evaluate
 * @param cmp comparison value > 0
 * @return true (abs(val) < cmp) / false
*/
template <typename T>
inline bool IsAbsLower(T val, T cmp)
{
  return val*val < cmp*cmp;
};

/**
 * @brief check absolute value is upper than comparison
 * @param val variable to evaluate
 * @param cmp comparison value > 0
 * @return true (abs(val) > cmp) / false
*/
template <typename T>
inline bool IsAbsUpper(T val, T cmp)
{
  return val*val > cmp*cmp;
};

/**
 * @brief check absolute value is out of range from minimum to maximum
 * @param val variable to evaluate
 * @param min minimum value > 0
 * @param max maximum value > 0
 * @note [max] > [min]
 * @return true (abs(val) > max || abs(val) < min) / false
*/
template <typename T>
inline bool IsAbsOutOfRange(T val, T min, T max)
{
  return val*val > max*max || val*val < min*min;
};

/**
 * @brief lower boundary
 * @param val variable to evaluate
 * @param min minimum value
 * @note val >= min
*/
template <typename T>
inline T LBnd(T val, T min)
{
  return val < min ? min : val;
};

/**
 * @brief upper boundary
 * @param val variable to evaluate
 * @param max maximum value
 * @note val <= max
*/
template <typename T>
inline T UBnd(T val, T max)
{
  return val > max ? max : val;
};

/**
 * @brief lower & upper boundary
 * @param val variable to evaluate
 * @param max maximum value
 * @param min minimum value
 * @note 1) must be [max] > [min]
 * @note 2) min <= val <= max
*/
template <typename T>
inline T LUBnd(T val, T min, T max)
{
  return val > max ? max : val < min ? min : val;
};

/**
 * @brief absolute lower boundary
 * @param val variable to evaluate
 * @param min minimum value > 0
 * @note |val| >= min
*/
template <typename T>
inline T AbsLBnd(T val, T min)
{
  int n_sign_val = Sign(val);
  T abs_val = n_sign_val * val;
  abs_val = abs_val < min ? min : abs_val;
  return n_sign_val * abs_val;
};

/**
 * @brief absolute upper boundary
 * @param val variable to evaluate
 * @param max maximum value > 0
 * @note |val| <= max
*/
template <typename T>
inline T AbsUBnd(T val, T max)
{
  int n_sign_val = Sign(val);
  T abs_val = n_sign_val * val;
  abs_val = abs_val > max ? max : abs_val;
  return n_sign_val * abs_val;
};

/**
 * @brief absolute lower & upper boundary
 * @param val variable to evaluate
 * @param min minimum value > 0
 * @param max maximum value > 0
 * @note min <= |val| <= max
*/
template <typename T>
inline T AbsLUBnd(T val, T min, T max)
{
  int n_sign_val = Sign(val);
  T abs_val = n_sign_val * val;
  abs_val = abs_val > max ? max : abs_val < min ? min : abs_val;
  return n_sign_val * abs_val;
};

/**
 * @brief calculate ratio from min to max
 * @param val variable to evaluate
 * @param min minimum of range
 * @param max maximum of range
 * @note (flt_val - min) / (max - min), [0,1]
*/
template <typename T>
inline T LURatio(T val, T min, T max)
{
  T flt_val = LUBnd(val, min, max);
  return (flt_val - min) / (max - min);
};

} // namespace CommonMath
namespace CM = CommonMath;
} // namespace NVFR

#endif
