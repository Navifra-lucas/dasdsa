#ifndef NAVIFRA_NAVICAN_MOTOR_DRIVER_CUSTOM_SWZM_H
#define NAVIFRA_NAVICAN_MOTOR_DRIVER_CUSTOM_SWZM_H

#include "NaviCAN/MotorDriver/interface/IMotorDriverBase.h"
#include "NaviCAN/Object/custom/swzm/ControlWord.h"
#include "NaviCAN/Object/custom/swzm/StatusByte.h"

namespace NaviFra {
namespace NaviCAN {
namespace MotorDriver {
namespace Custom {
namespace swzm {

// 파라미터
struct MotorDriverParams : public NaviFra::BaseMotorDriverParams {
};

class MotorDriver : public NaviFra::IMotorDriverBase {
public:
    using ControlWord = NaviFra::NaviCAN::Object::Custom::swzm::ControlWord;
    using StatusByte = NaviFra::NaviCAN::Object::Custom::swzm::StatusByte;

    MotorDriver(const MotorDriverParams& params)
        : IMotorDriverBase(params)
    {
    }

    bool handleInit() override;
    void handleRead() override;
    void handleWrite() override;
    bool handleShutdown() override;
    bool handleHalt() override;
    bool handleRecover() override;

    bool setTargetInfo(double target) override;
    double getSpeed() override { return static_cast<double>(ActualSpeed()); }
    bool isFault() override { return (Fault() || SafeStopActive()); }
    double getPosition() override { return static_cast<double>(MotorEncoderPulseCounter()); }
    int8_t getOperationMode(void) override { return 3; }  // velocity mode
    bool isEnable() override { return PowerStageActive(); }
    uint16_t getStatus() override { return static_cast<uint16_t>(status_byte_.getValue()); }
    int getState() override { return PowerStageActive(); }
    std::string getStateText() override { return status_byte_.isPowerStageActive() ? "PowerStageActive" : "PowerStageInactive"; }
    void preset() override {}

private:
    auto DriveEnable(bool value) { setTpdoMapped<bool>(0x2100, 0x00, value); }
    auto MainContactorEnable(bool value) { setTpdoMapped<bool>(0x2101, 0x00, value); }
    auto BrakeRelease(bool value) { setTpdoMapped<bool>(0x2102, 0x00, value); }
    auto DigitalOutput1(bool value) { setTpdoMapped<bool>(0x2103, 0x00, value); }
    auto DigitalOutput2(bool value) { setTpdoMapped<bool>(0x2104, 0x00, value); }
    auto DigitalOutput3(bool value) { setTpdoMapped<bool>(0x2105, 0x00, value); }
    auto StopFunction(bool value) { setTpdoMapped<bool>(0x2106, 0x00, value); }
    auto ResetFault(bool value) { setTpdoMapped<bool>(0x210f, 0x00, value); }

    auto SetSpeed(int16_t value) { setTpdoMapped<int16_t>(0x2110, 0x00, value); }
    auto TorqueFeedForward(int16_t value) { setTpdoMapped<int16_t>(0x2111, 0x00, value); }
    auto SetCurrentProportionalValue1(int16_t value) { setTpdoMapped<int16_t>(0x2112, 0x00, value); }
    auto SetCurrentProportionalValue2(int16_t value) { setTpdoMapped<int16_t>(0x2113, 0x00, value); }

    bool PowerStageActive() { return getRpdoMapped<bool>(0x2114, 0x00); }
    bool Fault() { return getRpdoMapped<bool>(0x2115, 0x00); }
    auto TorqueMode() { return getRpdoMapped<bool>(0x2116, 0x00); }
    auto AnalogCan() { return getRpdoMapped<bool>(0x2117, 0x00); }
    auto CwCanEnable() { return getRpdoMapped<bool>(0x2118, 0x00); }
    auto Ccw() { return getRpdoMapped<bool>(0x2119, 0x00); }
    bool SafeStopActive() { return getRpdoMapped<bool>(0x211a, 0x00); }
    auto ToggleBit() { return getRpdoMapped<bool>(0x211b, 0x00); }

    auto MotorLoad() { return getRpdoMapped<int8_t>(0x211c, 0x00); }
    int16_t ActualSpeed() { return getRpdoMapped<int16_t>(0x211d, 0x00); }
    auto ActualTorque() { return getRpdoMapped<int16_t>(0x211e, 0x00); }
    int16_t MotorEncoderPulseCounter() { return getRpdoMapped<int16_t>(0x211f, 0x00); }
    auto FaultWarningCode() { return getRpdoMapped<uint16_t>(0x2120, 0x00); }
    auto BDILevel() { return getRpdoMapped<uint8_t>(0x2121, 0x00); }
    auto InverterTemperature() { return getRpdoMapped<int8_t>(0x2122, 0x00); }
    auto MotorTemperature() { return getRpdoMapped<int16_t>(0x2123, 0x00); }
    auto BatteryVoltage() { return getRpdoMapped<uint16_t>(0x2124, 0x00); }

    StatusByte status_byte_;
    ControlWord control_word_;

    double target_ = 0.0;
    bool is_target_ = false;

    constexpr static auto DRIVE_DISABLE_DELAY = 100ms;
};

}  // namespace swzm
}  // namespace Custom
}  // namespace MotorDriver
}  // namespace NaviCAN
}  // namespace NaviFra
#endif