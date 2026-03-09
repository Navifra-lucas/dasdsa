#ifndef NAVIFRA_NAVICAN_INTERFACE_MOTOR_CONROLLER_H
#define NAVIFRA_NAVICAN_INTERFACE_MOTOR_CONROLLER_H

#include "NaviCAN/EncoderDriver/interface/IExternalEncoder.h"
#include "NaviCAN/MotorDriver/MotorDriver.h"
#include "NaviCAN/MotorExtraInfo/interface/IMotorExtraInfo.h"

#include <chrono>
#include <memory>

namespace NaviFra {

class IMotorController {
public:
    virtual ~IMotorController() = default;

    virtual bool init() = 0;

    virtual void setTarget(double target) = 0;
    virtual double getTarget() = 0;
    virtual int8_t getOperationMode() = 0;
    virtual double getPosition() = 0;
    virtual double getSpeed() = 0;

    virtual double getCurrent() = 0;
    virtual double getVoltage() = 0;
    virtual uint32_t getErrorCode() = 0;
    virtual std::string getErrorMessage() = 0;
    virtual uint16_t getStatus() = 0;

    virtual int getState() = 0;
    virtual std::string getStateText() = 0;
    virtual uint16_t getSTOCode() = 0;
    virtual bool isEnable() = 0;
    virtual bool enable() = 0;
    virtual bool disable() = 0;
    virtual bool shutdown() = 0;
    virtual bool resetError() = 0;
    virtual bool homing(int8_t homing_method, std::chrono::milliseconds timeout) = 0;
    virtual bool isFault() = 0;
    virtual void presetEncoder() = 0;
};
}  // namespace NaviFra

#endif  // NAVIFRA_PIDCONTROLLER_H