#ifndef DRIVER_CONTROLLER_DD_HPP_
#define DRIVER_CONTROLLER_DD_HPP_

#include "controller_base.hpp"

namespace NaviFra {
class NaviCANCore;
class ControllerDD : public Controller {
public:
    ControllerDD(){};
    ~ControllerDD() {}
    void Initialize(bool b_only_param_update);
    void WriteCommand(const Cmd& o_cmd);


protected:
  void calculateOdom();
  void updateMotorData(const MotorDataMap& motor_data);

  private:

  
    void calculateVelocity(const NaviFra::SimplePos& pos);
    void BrakeControl();

    bool isEnable();
    void Enable();


};
};  // namespace NaviFra
#endif
