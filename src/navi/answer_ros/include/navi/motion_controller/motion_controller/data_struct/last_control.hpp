/*
 * @file	: last_control.hpp
 * @date	: Dec 20, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: purepersuit motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef LAST_CONTROL_HPP_
#define LAST_CONTROL_HPP_

#include <deque>

namespace NVFR {

struct LastControl_t
{
  double d_linear_spd=0.0;
  double d_linear_acc=0.0;
  double d_linear_jrk=0.0;
  double d_angular_spd=0.0;
  double d_angular_acc=0.0;
  double d_angular_jrk=0.0;

  std::deque<double> dq_linear_acc;
  std::deque<double> dq_angular_acc;

  void Reset() {
    d_linear_spd=0.0;
    d_linear_acc=0.0;
    d_linear_jrk=0.0;
    d_angular_spd=0.0;
    d_angular_acc=0.0;
    d_angular_jrk=0.0;

    dq_linear_acc=std::deque<double>(3, 0.0);
    dq_angular_acc=std::deque<double>(3, 0.0);
  }
};

} // namespace NVFR

#endif
