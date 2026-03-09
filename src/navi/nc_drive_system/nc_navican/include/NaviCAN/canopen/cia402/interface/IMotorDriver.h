#ifndef NAVIFRA_NAVICAN_INTERFACE_MOTOR_DRAIVER_H
#define NAVIFRA_NAVICAN_INTERFACE_MOTOR_DRAIVER_H

#include "lely/coapp/driver.hpp"
#include "lely/coapp/master.hpp"

#include <string>

namespace NaviFra {

class IMotorDriver {
public:
    virtual ~IMotorDriver() = default;

    enum OperationMode
    {
        NO_MODE = 0,
        PROFILED_POSITION = 1,
        VELOCITY = 2,
        PROFILED_VELOCITY = 3,
        PROFILED_TORQUE = 4,
        RESERVED = 5,
        HOMING = 6,
        INTERPOLATED_POSITION = 7,
        CYCLIC_SYNCHRONOUS_POSITION = 8,
        CYCLIC_SYNCHRONOUS_VELOCITY = 9,
        CYCLIC_SYNCHRONOUS_TORQUE = 10,
    };

    virtual bool setTarget(double val) = 0;

    virtual bool enterModeAndWait(int8_t mode) = 0;

    virtual bool isModeSupported(int8_t mode) = 0;

    virtual int8_t getMode() = 0;

    virtual void registerDefaultModes() = 0;
};

using IMotorSharedPtr = std::shared_ptr<IMotorDriver>;

}  // namespace NaviFra

#endif
