#ifndef NAVIFRA_NAVICAN_INTERFACE_MOTOR_INFO_H
#define NAVIFRA_NAVICAN_INTERFACE_MOTOR_INFO_H

#include <cstdbool>
#include <cstdint>
#include <string>

namespace NaviFra {
class IMotorInfo {
public:
    ~IMotorInfo() = default;

    virtual double getSpeed(void) = 0;
    virtual bool setTargetInfo(double target) = 0;
    virtual double getPosition(void) = 0;
    virtual int8_t getOperationMode(void) = 0;
    virtual void preset(void) = 0;
    virtual bool isEnable(void) = 0;
    virtual bool isFault(void) = 0;

    virtual uint16_t getStatus(void) = 0;
    virtual int getState() = 0;
    virtual std::string getStateText() = 0;
};
}  // namespace NaviFra

#endif