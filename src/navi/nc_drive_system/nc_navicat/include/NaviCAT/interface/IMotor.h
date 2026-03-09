#ifndef NAVIFRA_NAVICAT_INTERFACE_MOTOR_H
#define NAVIFRA_NAVICAT_INTERFACE_MOTOR_H

#include <cstdint>
#include <string>

namespace NaviFra {
namespace NaviCAT {

class IMotor {
public:
    virtual ~IMotor() = default;
    /*
        Status
    */
    virtual int getState(uint8_t motorId) = 0;
    virtual bool isEnable(uint8_t motorId) = 0;
    virtual bool isError(uint8_t motorId) = 0;

    /*
        Command
    */
    virtual void setTarget(uint8_t motorId, double value) = 0;
    virtual double getPosition(uint8_t motorId) = 0;
    virtual double getSpeed(uint8_t motorId) = 0;
    virtual double getWheelRPM(uint8_t motorId) = 0;
    virtual double getMotorRPM(uint8_t motorId) = 0;
    virtual double getCurrent(uint8_t motorId) = 0;
    virtual double getVoltage(uint8_t motorId) = 0;
    virtual uint32_t getErrorCode(uint8_t motorId) = 0;
    virtual std::string getErrorMessage(uint8_t motorId) = 0;
    virtual uint16_t getStatus(uint8_t motorId) = 0;

    virtual bool enable(uint8_t motorId) = 0;
    virtual bool disable(uint8_t motorId) = 0;
    virtual bool resetError(uint8_t motorId) = 0;

    virtual double getLatestRpdoWriteTimeCount(uint8_t motorId) const = 0;
    virtual int getCanBusState() = 0;
};
    
}
}  // namespace NaviFra
#endif  // NAVIFRA_INTERFACE_MOTOR_H