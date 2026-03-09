#ifndef NAVIFRA_NAVICAN_MOTOR_DRIVER_CUSTOM_CURTIS_THEPRIME_RTV3000_H
#define NAVIFRA_NAVICAN_MOTOR_DRIVER_CUSTOM_CURTIS_THEPRIME_RTV3000_H

#include "NaviCAN/MotorDriver/interface/IMotorDriverBase.h"

namespace NaviFra {
namespace NaviCAN {
namespace MotorDriver {
namespace Custom {
namespace curtis {
namespace ThePrime {
namespace RTV3000 {

// 파라미터
struct MotorDriverParams : public NaviFra::BaseMotorDriverParams {
    uint8_t steer_index;
};

class MotorDriver : public NaviFra::IMotorDriverBase {
public:
    MotorDriver(const MotorDriverParams& params)
        : IMotorDriverBase(params)
        , steer_index_(params.steer_index)
    {
        SteeringAngle(0x1234);
    }

    bool handleInit() override;
    void handleRead() override;
    void handleWrite() override;
    bool handleShutdown() override;
    bool handleHalt() override;
    bool handleRecover() override;

    bool setTargetInfo(double target) override;
    double getSpeed() override { return static_cast<double>(ActualVelocity()); }
    int8_t getOperationMode() override { return (motor_id_ <= 4) ? 3 : 1; }
    bool isFault() override { return false; }
    double getPosition() override { return static_cast<double>(ActualSteeringAngle(steer_index_)); }
    bool isEnable() override { return true; }  // 항상 Enabled
    uint16_t getStatus() override { return 0; }
    int getState() override { return 0; }
    std::string getStateText() override { return ""; }
    void preset() override {}

    // tpdo 1
    uint16_t ActualVelocity() { return getRpdoMapped<uint16_t>(0x606c, 0); }
    auto ActualCurrent() { return getRpdoMapped<uint16_t>(0x6078, 0); }
    auto TractionErrorCode() { return getRpdoMapped<uint8_t>(0x603f, 1); }
    auto Voltage() { return getRpdoMapped<uint8_t>(0x3001, 0); }
    auto Temperature() { return getRpdoMapped<uint8_t>(0x3002, 0); }

    // tpdo 2
    uint16_t ActualSteeringAngle(uint8_t steer_index) { return getRpdoMapped<uint16_t>(0x6064, steer_index); }
    auto J24Input9() { return getRpdoMapped<bool>(0x3012, 0); }

    // tpdo 3
    auto Accumulation(int index) { return getRpdoMapped<uint8_t>(0x3010, index - 1); }
    auto SteeringErrorCode() { return getRpdoMapped<uint8_t>(0x603f, steer_index_); }

    // rpdo 1
    void Interlock(bool value) { setTpdoMapped<bool>(0x2000, 0, value); }
    void Forward(bool value) { setTpdoMapped<bool>(0x2001, 0, value); }
    void Backward(bool value) { setTpdoMapped<bool>(0x2002, 0, value); }
    void J19Driver6(bool value) { setTpdoMapped<bool>(0x2003, 0, value); }
    void J4Driver3(bool value) { setTpdoMapped<bool>(0x2004, 0, value); }
    void J3Driver4(bool value) { setTpdoMapped<bool>(0x2005, 0, value); }
    void AccumulatorReset(bool value) { setTpdoMapped<bool>(0x2006, 0, value); }
    void J20Driver7(bool value) { setTpdoMapped<bool>(0x2007, 0, value); }

    void TargetVelocity(uint16_t value) { setTpdoMapped<uint16_t>(0x60ff, 0, value); }
    void Acceleration(uint8_t value) { setTpdoMapped<uint8_t>(0x6083, 0, value); }
    void Deceleration(uint8_t value) { setTpdoMapped<uint8_t>(0x6084, 0, value); }

    // rpdo 2
    void SteeringAngle(uint16_t value) { setTpdoMapped<uint16_t>(0x607a, steer_index_, value); }
    void Valve(uint16_t value) { setTpdoMapped<uint16_t>(0x2012, 0, value); }

private:
    const uint8_t steer_index_;
};

struct HydraulicsDriverParams : public NaviFra::BaseMotorDriverParams {
};

class HydraulicsDriver : public NaviFra::IMotorDriverBase {
public:
    HydraulicsDriver(const HydraulicsDriverParams& params)
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
    double getSpeed() override { return 0.0; }
    int8_t getOperationMode() override { return (motor_id_ <= 4) ? 3 : 1; }
    bool isFault() override { return false; }
    double getPosition() override { return 0.0; }
    bool isEnable() override { return true; }  // 항상 Enabled
    uint16_t getStatus() override { return 0; }
    int getState() override { return 0; }
    std::string getStateText() override { return ""; }
    void preset() override {}

    // tpdo 1

    // 유압 드라이버 포트 J9 Input 상태읽기
    auto J9Input5() { return getRpdoMapped<uint8_t>(0x3000, 0); }

    // 유압 모터 속도 피드백
    auto ActualVelocity() { return getRpdoMapped<uint16_t>(0x606c, 0); }

    // 유압 모터 전류 피드백
    auto ActualCurrent() { return getRpdoMapped<uint16_t>(0x6078, 0); }

    // 유압 드라이버 오류 코드 읽기 (0x12 ~ 0xff)
    auto ErrorCode() { return getRpdoMapped<uint8_t>(0x603f, 0); }

    // 유압 모터 온도 피드백 (0 ~ 255sms -55도 ~ 200도)
    auto Temperature() { return getRpdoMapped<uint8_t>(0x3002, 0); }

    // rpdo 1
    // 유압드라이버 상호 장금 신호 (1: 활성화, 0: 잠금 해제)
    void Interlock(bool value) { setTpdoMapped<bool>(0x2000, 0, value); }

    // 유압드라이버 전동기 주행 신호 활성화(1: 활성화, 0: 잠금 해제)
    void DriveSignal(bool value) { setTpdoMapped<bool>(0x2001, 0, value); }

    // 유압 드라이버 포트 J19 -> Drive 6 활성화, 전압 강하 불가
    void J19Driver6(bool value) { setTpdoMapped<bool>(0x2002, 0, value); }

    // 유압 드라이버 포트 J3 -> Drive 3 활성화, PWM 비율 조정 가능 (포크 폭 조절)
    void J3Driver3(bool value) { setTpdoMapped<bool>(0x2003, 0, value); }

    // 유압 드라이버 포트 J4 -> Drive 4 활성화, PWM 비율 조정 가능 (포크 틸팅)
    void J4Driver4(bool value) { setTpdoMapped<bool>(0x2004, 0, value); }

    // 유압 드라이버 포트 J5 -> Drive 2 활성화, PWM 비율 조정 가능
    void J5Driver2(bool value) { setTpdoMapped<bool>(0x2005, 0, value); }

    // 0: 정회전, 1:역회전, (주의: 모터만 해당)
    void MotorDirection(bool value) { setTpdoMapped<bool>(0x2006, 0, value); }

    // 유압 드라이버 포트 J20 -> Drive 7 활성화, 전압 강하 불가
    void J20Driver7(bool value) { setTpdoMapped<bool>(0x2007, 0, value); }

    // 유압 드라이버 속도
    void TargetVelocity(uint16_t value) { setTpdoMapped<uint16_t>(0x60ff, 0, value); }

    // 비례 밸브: 값 (0 ~ 32767)은 비례 드라이버 PWM 출력전류 0 ~ 2A, 출력은 J2 비례 주행 포트
    void Valve(uint16_t value) { setTpdoMapped<uint16_t>(0x2008, 0, value); }

    // 유압 드라이버 가속(2 ~ 255 -> 0.2 ~ 25.5s)
    // !!!값이 작을 수록 가속 빠름!!!!
    void Acceleration(uint8_t value) { setTpdoMapped<uint8_t>(0x6083, 0, value); }

    // 유압 드라이버 가속(2 ~ 255 -> 0.2 ~ 25.5s)
    //  !!!값이 작을 수록 가속 빠름!!!!
    void Deceleration(uint8_t value) { setTpdoMapped<uint8_t>(0x6084, 0, value); }
};

}  // namespace RTV3000
}  // namespace ThePrime
}  // namespace curtis
}  // namespace Custom
}  // namespace MotorDriver
}  // namespace NaviCAN
}  // namespace NaviFra
#endif