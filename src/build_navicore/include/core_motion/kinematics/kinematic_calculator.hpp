#ifndef NAVIFRA_KINEMATIC_CALCULATOR_HPP
#define NAVIFRA_KINEMATIC_CALCULATOR_HPP
#include "cmd/cmd.hpp"
#include "simplepos/simplepos.hpp"
#include "util/ansi_color.h"

#include <math.h>
#include <stdio.h>

#include <cfloat>
#include <cmath>
#include <vector>

namespace NaviFra {
struct WheelData_t {
  float f_FL_scalar = 0;
  float f_FL_angle_rad = 0;

  float f_FR_scalar = 0;
  float f_FR_angle_rad = 0;

  float f_RL_scalar = 0;
  float f_RL_angle_rad = 0;

  float f_RR_scalar = 0;
  float f_RR_angle_rad = 0;
};

struct DD_Wheel_t {
  float f_FL_wheel_scalar = 0;
  float f_RR_wheel_scalar = 0;

  // param
  float f_wheel_base_m = 1;
};

struct Skid_Wheel_t {
  float f_FL_wheel_scalar = 0;
  float f_FR_wheel_scalar = 0;
  float f_RL_wheel_scalar = 0;
  float f_RR_wheel_scalar = 0;

  // param
  float f_FL_wheel_to_center_m = 0.5; // 로봇 중심 기준 x 방향 휠 위치
  float f_FL_wheel_offset = 0.2; // 로봇 중심 기준 y 방향 휠 위치
  float f_FR_wheel_to_center_m = 0.5;
  float f_FR_wheel_offset = -0.2;
  float f_RL_wheel_to_center_m = -0.5;
  float f_RL_wheel_offset = 0.2;
  float f_RR_wheel_to_center_m = -0.5;
  float f_RR_wheel_offset = -0.2;

  float f_wheel_max_speed = 0.5; // 휠의 최대 속도 [m/s]
};

struct SD_Wheel_t {
  float f_FL_wheel_scalar = 0;
  float f_FL_wheel_steer_angle_rad = 0;
  float f_FL_wheel_steer_angle_rad_pre = 0;

  // param
  float f_FL_wheel_to_center_m = 1; // 로봇 중심 기준 x 방향 휠 위치
  float f_FL_wheel_offset = 0;      // 로봇 중심 기준 y 방향 휠 위치
  float f_FL_wheel_axis_offset_m = 0.0;
};

struct SD_Nrmk_Wheel_t {
  float f_FL_wheel_scalar = 0;
  float f_FL_wheel_steer_angle_rad = 0;
  float f_FL_wheel_steer_angle_rad_pre = 0;

  float f_FR_wheel_scalar = 0;
  float f_FR_wheel_steer_angle_rad = 0;
  float f_FR_wheel_steer_angle_rad_pre = 0;

  float f_RR_wheel_steer_angle_rad = 0;
  float f_RR_wheel_steer_angle_rad_pre = 0;

  // param
  float f_FL_wheel_to_center_m = 1.0; // 로봇 중심 기준 x 방향 휠 위치
  float f_FL_wheel_offset = 0.3; // 로봇 중심 기준 y 방향 휠 위치
  float f_FL_wheel_axis_offset_m = 0.0;
  float f_FL_steer_max_angle_deg = 28.0;
  float f_FL_steer_min_angle_deg = -28.0;

  float f_FR_wheel_to_center_m = 1.0;
  float f_FR_wheel_offset = -0.3;
  float f_FR_wheel_axis_offset_m = 0.0;
  float f_FR_steer_max_angle_deg = 28.0;
  float f_FR_steer_min_angle_deg = -28.0;

  float f_RR_wheel_to_center_m = -1.0;
  float f_RR_wheel_offset = 0.0;
  float f_RR_wheel_axis_offset_m = 0.0;
  float f_RR_steer_max_angle_deg = 100.0;
  float f_RR_steer_min_angle_deg = -100.0;
};

struct QD_Wheel_t {
  float f_FL_wheel_scalar = 0;
  float f_FL_wheel_steer_angle_rad = 0;
  float f_FL_wheel_steer_angle_rad_pre = 0;

  float f_RR_wheel_scalar = 0;
  float f_RR_wheel_steer_angle_rad = 0;
  float f_RR_wheel_steer_angle_rad_pre = 0;

  // param
  float f_FL_wheel_to_center_m = 1; // 로봇 중심 기준 x 방향 휠 위치
  float f_FL_wheel_offset = 0;      // 로봇 중심 기준 y 방향 휠 위치
  float f_FL_steer_max_angle_deg = 110;
  float f_FL_steer_min_angle_deg = -110;

  float f_RR_wheel_to_center_m = 1;
  float f_RR_wheel_offset = 0;
  float f_RR_steer_max_angle_deg = 110;
  float f_RR_steer_min_angle_deg = -110;
};

struct QD_Oct_Wheel_t {
  float f_FL_wheel_scalar = 0;
  float f_FL_wheel_steer_angle_rad = 0;
  float f_FL_wheel_steer_angle_rad_pre = 0;

  float f_FR_wheel_scalar = 0;
  float f_FR_wheel_steer_angle_rad = 0;
  float f_FR_wheel_steer_angle_rad_pre = 0;

  float f_RL_wheel_scalar = 0;
  float f_RL_wheel_steer_angle_rad = 0;
  float f_RL_wheel_steer_angle_rad_pre = 0;

  float f_RR_wheel_scalar = 0;
  float f_RR_wheel_steer_angle_rad = 0;
  float f_RR_wheel_steer_angle_rad_pre = 0;

  // param
  float f_FL_wheel_to_center_m = 1; // 로봇 중심 기준 x 방향 휠 위치
  float f_FL_wheel_offset = 0;      // 로봇 중심 기준 y 방향 휠 위치
  float f_FL_steer_max_angle_deg = 110;
  float f_FL_steer_min_angle_deg = -110;

  float f_FR_wheel_to_center_m = 1;
  float f_FR_wheel_offset = 0;
  float f_FR_steer_max_angle_deg = 110;
  float f_FR_steer_min_angle_deg = -110;

  float f_RL_wheel_to_center_m = 1;
  float f_RL_wheel_offset = 0;
  float f_RL_steer_max_angle_deg = 110;
  float f_RL_steer_min_angle_deg = -110;

  float f_RR_wheel_to_center_m = 1;
  float f_RR_wheel_offset = 0;
  float f_RR_steer_max_angle_deg = 110;
  float f_RR_steer_min_angle_deg = -110;
};

class KinematicCalculator {
public:
  // backup용 데이터
  DD_Wheel_t st_dd_wheel_;
  Skid_Wheel_t st_skid_wheel_;
  SD_Wheel_t st_sd_wheel_;
  SD_Nrmk_Wheel_t st_sd_nrmk_wheel_;
  QD_Wheel_t st_qd_wheel_;
  QD_Oct_Wheel_t st_qd_oct_wheel_;
  int n_spinturn_steer_direction_ = 0;

  KinematicCalculator(){};
  ~KinematicCalculator(){};

  void SetDDInfo(float f_wheel_base_m);
  void SetSkidInfo(float f_FL_wheel_to_center_m, float f_FL_wheel_offset,
                   float f_FR_wheel_to_center_m, float f_FR_wheel_offset,
                   float f_RL_wheel_to_center_m, float f_RL_wheel_offset,
                   float f_RR_wheel_to_center_m, float f_RR_wheel_offset,
                   float f_wheel_max_speed);
  void SetSDInfo(float f_FL_wheel_to_center_m, float f_FL_wheel_offset);
  void SetSDInfo(float f_FL_wheel_to_center_m, float f_FL_wheel_offset,
                 float f_FL_wheel_axis_offset);
  void SetSDNrmkInfo(float f_FL_wheel_to_center_m, float f_FL_wheel_offset,
                     float f_FL_wheel_axis_offset, float f_FR_wheel_to_center_m,
                     float f_FR_wheel_offset, float f_FR_wheel_axis_offset,
                     float f_RR_wheel_to_center_m, float f_RR_wheel_offset,
                     float f_RR_wheel_axis_offset, float f_FL_max_angle,
                     float f_FR_max_angle, float f_RR_max_angle,
                     float f_FL_min_angle, float f_FR_min_angle,
                     float f_RR_min_angle);

  void SetQDInfo(float f_FL_wheel_to_center_m, float f_FL_wheel_offset,
                 float f_RR_wheel_to_center_m, float f_RR_wheel_offset,
                 float f_FL_max_angle, float f_RR_max_angle,
                 float f_FL_min_angle, float f_RR_min_angle);
  void SetQDOctInfo(float f_FL_wheel_to_center_m, float f_FL_wheel_offset,
                    float f_FR_wheel_to_center_m, float f_FR_wheel_offset,
                    float f_RL_wheel_to_center_m, float f_RL_wheel_offset,
                    float f_RR_wheel_to_center_m, float f_RR_wheel_offset,
                    float f_FL_max_angle, float f_FR_max_angle,
                    float f_RL_max_angle, float f_RR_max_angle,
                    float f_FL_min_angle, float f_FR_min_angle,
                    float f_RL_min_angle, float f_RR_min_angle);
  /**
   * @brief DD 기구 센서 값 -> v,w
   * @return NaviFra::SimplePos
   */
  NaviFra::SimplePos CalcCmdDDWheel(const WheelData_t &st_data);

  /**
   * @brief Skid 기구 센서 값 -> v,w
   * @return NaviFra::SimplePos
   */
  NaviFra::SimplePos CalcCmdSkidWheel(const WheelData_t &st_data);

  /**
   * @brief SD 기구 센서 값 -> v,w
   * @return NaviFra::SimplePos
   */
  NaviFra::SimplePos CalcCmdSDWheel(const WheelData_t &st_data);

  /**
   * @brief SD Nrmk 기구 센서 값 -> v,w
   * @return NaviFra::SimplePos
   */
  NaviFra::SimplePos CalcCmdSDNrmkWheel(const WheelData_t &st_data);

  /**
   * @brief QD 기구 센서 값 -> v,w
   * @return NaviFra::SimplePos
   */
  NaviFra::SimplePos CalcCmdQDWheel(const WheelData_t &st_data);

  /**
   * @brief QDOct 기구 센서 값 -> v,w
   * @return NaviFra::SimplePos
   */
  NaviFra::SimplePos CalcCmdQDOctWheel(const WheelData_t &st_data);

  /**
   * @brief DD 기구 센서 값 -> odom
   * @return NaviFra::SimplePos
   */
  NaviFra::SimplePos CalcDposDDWheel(const WheelData_t &st_data);

  /**
   * @brief Skid 기구 센서 값 -> odom
   * @return NaviFra::SimplePos
   */
  NaviFra::SimplePos CalcDposSkidWheel(const WheelData_t &st_data);

  /**
   * @brief SD 기구 센서 값 -> odom
   * @return NaviFra::SimplePos
   */
  NaviFra::SimplePos CalcDposSDWheel(const WheelData_t &st_data);

  /**
   * @brief SD Nrmk 기구 센서 값 -> odom
   * @return NaviFra::SimplePos
   */
  NaviFra::SimplePos CalcDposSDNrmkWheel(const WheelData_t &st_data);

  /**
   * @brief QD 기구 센서 값 -> odom
   * @return NaviFra::SimplePos
   */

  NaviFra::SimplePos CalcDposQDWheel(const WheelData_t &st_data);

  /**
   * @brief QDOct 기구 센서 값 -> odom
   * @return NaviFra::SimplePos
   */

  NaviFra::SimplePos CalcDposQDOctWheel(const WheelData_t &st_data);

  /**
   * @brief DD cmd -> 기구 제어
   */
  DD_Wheel_t ResolveDDCmd(const Cmd &o_cmd);

  /**
   * @brief Skid cmd -> 기구 제어
   */
  Skid_Wheel_t ResolveSkidCmd(const Cmd &o_cmd);

  /**
   * @brief SD cmd -> 기구 제어
   */
  SD_Wheel_t ResolveSDCmd(const Cmd &o_cmd);

  /**
   * @brief SD Nrmk cmd -> 기구 제어
   */
  SD_Nrmk_Wheel_t ResolveSDNrmkCmd(const Cmd &o_cmd);

  /**
   * @brief QD cmd -> 기구 제어
   */
  QD_Wheel_t ResolveQDCmd(const Cmd &o_cmd);

  /**
   * @brief QD Oct cmd -> 기구 제어
   */
  QD_Oct_Wheel_t ResolveQDOctCmd(const Cmd &o_cmd);

  void SetSpinturnSteerDirection(int n_data);
  float CalcWfromDV(float f_D_x_m, float f_D_y_m, float f_V_x_m, float f_V_y_m);
  NaviFra::SimplePos VTransform_RigidBody(float f_v_x, float f_v_y, float f_w,
                                          float f_related_d_x,
                                          float f_related_d_y);
  NaviFra::SimplePos CalcDPosfromVform(const NaviFra::SimplePos &o_Vform);
  NaviFra::SimplePos CalcDPosfromXm(float f_x_m, float f_yaw_rad);

  int sign(float val) { return (0 < val) - (val < 0); }

  float CalcAngleDomainRad_(float f_angle_rad) {
    return fmod((f_angle_rad + sign(f_angle_rad) * M_PI), 2.0 * M_PI) -
           sign(f_angle_rad) * M_PI;
  }

  int Sign(float a) {
    return isgreater(a, 0.0f) ? 1 : isless(a, 0.0f) ? -1 : 0;
  }
};
} // namespace NaviFra
#endif
