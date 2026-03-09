#ifndef NAVIFRA_NAVICAN_INTERFACE_MOTOR_DRIVER_BASE_H
#define NAVIFRA_NAVICAN_INTERFACE_MOTOR_DRIVER_BASE_H

#include "NaviCAN/MotorDriver/interface/IMotorInfo.h"
#include "NaviCAN/MotorDriver/BaseMotorDriver.h"

namespace NaviFra
{

// 모든 MotorDriver가 구현해야 하는 기본 인터페이스
class IMotorDriverBase : public BaseMotorDriver, public IMotorInfo {
public:
    explicit IMotorDriverBase(const BaseMotorDriverParams& params)
        : BaseMotorDriver(params) {}
        
    virtual ~IMotorDriverBase() = default;
};


}

#endif