
/*
 * @file	: pos.hpp
 * @date	: Jan. 1, 2022
 * @author	: "Joongtae, Park(brain)" (founder@navifra.com)"
 * @brief	: x,y,z,roll,pitch,yaw 정보를 저장
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2022 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_SIMPLE_POS_HPP_
#define NAVIFRA_SIMPLE_POS_HPP_

#include <cmath>
#include <float.h>
#include <iostream>

namespace NaviFra {
class SimplePos // 가장 기초가 되는 pos자료클래스
{
public:
  float f_x_m_ = 0;
  float f_y_m_ = 0;
  float f_z_m_ = 0;

  float f_yaw_ = 0;
  float f_pitch_ = 0;
  float f_roll_ = 0;

  // SimplePos(){};
  // ~SimplePos(){};

  void SetXm(const float &f_x_m);
  void SetYm(const float &f_y_m);
  void SetZm(const float &f_z_m);

  float GetXm(void) const;
  float GetYm(void) const;
  float GetZm(void) const;

  float GetDeg(void) const;
  float GetDegDomain(void) const;
  float GetRadDomain(void) const;

  float GetDiffDegShort(const SimplePos &o_pos, const int &n_direct = 0) const;
  float GetRad(void) const;
  void SetDeg(const float &f_deg);

  void SetRad(const float &f_rad);

  float GetRollDeg() { return f_roll_; }
  float GetPitchDeg() { return f_pitch_; }
  float GetYawDeg() { return f_yaw_; }
  void SetQuaternion(const float &f_qw, const float &f_qx, const float &f_qy,
                     const float &f_qz);

  inline SimplePos operator+(const SimplePos &o_pos) const {
    SimplePos o_result;
    o_result.SetXm(f_x_m_ + o_pos.GetXm());
    o_result.SetYm(f_y_m_ + o_pos.GetYm());
    o_result.SetRad(f_yaw_ + o_pos.GetRad());
    return o_result;
  }

  inline SimplePos operator-(const SimplePos &o_pos) const {
    SimplePos o_result;
    o_result.SetXm(f_x_m_ - o_pos.GetXm());
    o_result.SetYm(f_y_m_ - o_pos.GetYm());
    o_result.SetRad(f_yaw_ - o_pos.GetRad());
    return o_result;
  }

  inline SimplePos operator*(const float f_value) const {
    SimplePos o_result;
    o_result.SetXm(f_x_m_ * f_value);
    o_result.SetYm(f_y_m_ * f_value);
    return o_result;
  }

  inline SimplePos operator/(const float f_value) const {
    SimplePos o_result;
    if (f_value == 0)
      return o_result;
    o_result.SetXm(f_x_m_ / f_value);
    o_result.SetYm(f_y_m_ / f_value);
    return o_result;
  }

  friend std::ostream &operator<<(std::ostream &o, SimplePos const &fred);

  SimplePos(float f_x_m = 0.f, float f_y_m = 0.f, float f_deg = 0.f);
  SimplePos(float f_x_m, float f_y_m, float f_z_m, float f_deg, float f_roll,
            float f_pitch);
  virtual ~SimplePos(){};

  friend SimplePos operator*(const SimplePos &a_to_b_pose,
                             const SimplePos &b_to_c_pose);
  SimplePos inv();
  void Init() {
    f_x_m_ = 0.f;
    f_y_m_ = 0.f;
    f_yaw_ = 0.f;
    SetDeg(0);
  }

  double Cross(const SimplePos &o_pos) const {
    return GetXm() * o_pos.GetYm() - GetYm() * o_pos.GetXm();
  }
};
} // namespace NaviFra

#endif
