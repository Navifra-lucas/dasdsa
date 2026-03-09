#ifndef NAVIFRA_NAVICAN_ERRROR_MOTOR_CONTROLLER
#define NAVIFRA_NAVICAN_ERRROR_MOTOR_CONTROLLER

#include "NaviCAN/EncoderDriver/interface/IExternalEncoder.h"
#include "NaviCAN/MotorController/interface/IMotorController.h"
#include "NaviCAN/MotorDriver/interface/IMotorInfo.h"
#include "NaviCAN/MotorDriver/interface/IMotorStateProcessor.h"
#include "NaviCAN/MotorExtraInfo/interface/IMotorExtraInfo.h"

namespace NaviFra {

class NullMotorController : public IMotorController {
public:
    NullMotorController(){};

    bool init() { return 0; }

    void setTarget(double target) override {}
    double getTarget() override { return 0.0; }
    int8_t getOperationMode() override { return 0; }
    double getPosition() override { return 0.0; }
    double getSpeed() override { return 0.0; }

    double getCurrent() override { return 0.01; }
    double getVoltage() override { return 0.0; }
    uint32_t getErrorCode() override { return 0; }
    std::string getErrorMessage() override { return "Wrong Motor Id"; }
    uint16_t getStatus() override { return 0; }
    uint16_t getSTOCode() override { return 0; }

    int getState() override { return -1; }
    std::string getStateText() override { return "NullMotorController"; }

    bool isEnable() override { return false; }
    bool enable() override { return false; }
    bool disable() override { return false; }
    bool shutdown() override { return false; }
    bool resetError() override { return false; }
    bool isFault() override { return true; }
    bool homing(int8_t homing_method, std::chrono::milliseconds timeout) override { return true; }
    void presetEncoder() override {}
};
}  // namespace NaviFra

#endif