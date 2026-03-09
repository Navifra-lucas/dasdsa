/*
 * @file	: pid_controller.hpp
 * @date	: Jan 9, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: PID controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include "utils/common_math.hpp"

namespace NVFR {

/*
 * < PID controller >

 * type : signed variable (ex. int, float, double, ect...)
 * parameters : Kp, Ki, Kd, cutoff(max of I gain)
 * input data :
    r : reference, desired
    x : current state
    dt: time interval
 * PID calculation :
    P = r - x
    I = prev_I + P*dt, abs(I) <= cutoff
    D = (P - prev_P) / dt

    y = Kp*P + Ki*I + Kd*D
*/
template <typename T>
class PID
{
public:
  PID()
    : Kp_(static_cast<T>(1.0))
    , Ki_(static_cast<T>(0.01))
    , Kd_(static_cast<T>(0.001))
    , cutoff_(static_cast<T>(5.0))
    , min_dt_(static_cast<T>(0.001))
    , prev_P_(static_cast<T>(0))
    , prev_I_(static_cast<T>(0))
  {
    //
  };

  void SetParam(T Kp, T Ki, T Kd, T cutoff, T min_dt=static_cast<T>(0.001))
  {
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
    cutoff_ = cutoff;
    min_dt_ = min_dt;
  };

  // reset pid states
  void Reset()
  {
    prev_P_ = static_cast<T>(0);
    prev_I_ = static_cast<T>(0);
  };

  void Reset(T prev_P)
  {
    prev_P_ = prev_P;
    prev_I_ = static_cast<T>(0);
  };

  // execute pid controller
  T Execute(T r, T x, T dt)
  {
    // time filter
    dt = CM::AbsLBnd(dt, min_dt_);

    T P = r - x;
    T I = prev_I_ + P*dt;
    I = CM::AbsUBnd(I, cutoff_);
    T D = (P - prev_P_) / dt;

    T y = Kp_*P + Ki_*I + Kd_*D;

    prev_P_ = P;
    prev_I_ = I;

    return y;
  };

private:
  // parameters
  T Kp_;
  T Ki_;
  T Kd_;
  T cutoff_;
  T min_dt_;

  // variables
  T prev_P_;
  T prev_I_;

};

} // namespace NVFR

#endif
