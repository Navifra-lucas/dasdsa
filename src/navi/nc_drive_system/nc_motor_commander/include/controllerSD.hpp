#ifndef DRIVER_CONTROLLER_SD_HPP_
#define DRIVER_CONTROLLER_SD_HPP_

#include "controller_base.hpp"
#include "core_calculator/core_calculator.hpp"
#include "math.h"

#include <chrono>
#include <functional>
#include <iostream>
#include <thread>

namespace NaviFra {
class ControllerSD : public Controller {
public:
    void Initialize(bool b_only_param_update);
    void WriteCommand(const Cmd& o_cmd);

    void SetQuadCmd(string& str_data);
    string GetQuadCmd();
    void SpinturnSteerDirection(int n_data);

    // void EncoderZero(string& str_data);
    // void SetMotorGain(string& str_data);
    // void SteerAlotOpen(bool b_data);
    bool b_control_off_ = false;

protected:
  void calculateOdom();
  void updateMotorData(const MotorDataMap& motor_data);
  
private:
    void calculateVelocity(const NaviFra::SimplePos& pos);
   
    void BrakeControl();
    bool isEnable();
    void Enable();

    std::mutex mtx_quad_cmd_;
    float f_steer_offse_deg_ = 0;
};
};  // namespace NaviFra
#endif
