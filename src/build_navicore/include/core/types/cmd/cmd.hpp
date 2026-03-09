/*
 * @file	: Cmd.hpp
 * @date	: Dec 31, 2019
 * @author	:"Joongtae, Park(brain)" (founder@navifra.com)"
 * @brief	: 로봇 및 기타 object의 위치를 저장하는  class
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2022 NAVIFRA Coperation.
 * 	All Rights are Reserved.
 */

#ifndef CMD_HPP_
#define CMD_HPP_

#include <cmath>
#include <iostream>

namespace NaviFra {

class Cmd {
public:
    Cmd();
    virtual ~Cmd();

    void SetVelX(const float f_vel_x_m_s);
    void SetVelY(const float f_vel_y_m_s);
    void SetVelYaw(const float f_vel_yaw_rad_s);
    void SetDirectionFlag(const int n_direction_flag);
    void SetDGFlag(const bool b_diagonal_flag);
    void SetTurnTableVel(const float f_turn_table_deg_s);
    void SetStopFlag(const bool b_flag);

    float GetVelX(void);
    float GetVelY(void);
    float GetVelYaw(void);
    int GetDirectionFlag();
    bool GetDGFlag();
    float GetTurnTableVel(void);
    bool GetStopFlag();

    void Clear();

private:
    float f_linear_vel_x_s_ = 0.f;
    float f_linear_vel_y_s_ = 0.f;
    float f_anguler_vel_yaw_s_ = 0.f;
    int n_direction_flag_ = 0;  // 0 수동, 1 직진, 2 후진
    bool b_diagonal_flag_ = false;
    float f_turn_table_deg_s_ = 0;

    bool b_stop_flag_ = false;
};
}  // namespace NaviFra

#endif