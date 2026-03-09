#ifndef NAVIFRA_NAVICAN_MOTOR_DRIVER_REGISTRAR_H
#define NAVIFRA_NAVICAN_MOTOR_DRIVER_REGISTRAR_H

#include <string>
#include "NaviCAN/MotorDriver/MotorDriverFactory.h"

#include "util/logger.hpp"
namespace NaviFra
{

// Self-Registration 
template<typename DriverType, typename ParamsType = BaseMotorDriverParams>
class MotorDriverRegistrar {
public:
    MotorDriverRegistrar(const std::string& name) {
        NLOG(info) << "Registering Motor Driver: " << name;
        MotorDriverFactory::getInstance().registerMotorDriver<DriverType, ParamsType>(name);
    }
};

} // namespace NaviFra

#endif