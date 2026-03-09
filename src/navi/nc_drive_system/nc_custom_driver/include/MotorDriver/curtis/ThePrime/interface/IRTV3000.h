#ifndef NAVIFRA_CUSTOM_MOTOR_DRIVER_RTV3000
#define NAVIFRA_CUSTOM_MOTOR_DRIVER_RTV3000

#include <cstdint>

namespace NaviFra {
namespace MotorDriver {
namespace curtis {
namespace ThePrime {

enum class TractionMotorId : uint8_t
{
    A = 0,
    B = 1
};

enum class HydraulicsMotorId : uint8_t
{
    H = 0
};

enum class SteerMotorId : uint8_t
{
    S1 = 0,  // AS/BS 조향
    S2 = 1  // SA/SB 조향
};

class ITraction {
public:
    virtual ~ITraction() = default;

    // Feedback - 주행 모터 상태
    virtual uint16_t FeedbackSpeed(TractionMotorId motor_id) = 0;
    virtual uint16_t FeedbackCurrent(TractionMotorId motor_id) = 0;
    virtual uint8_t ErrorCode(TractionMotorId motor_id) = 0;
    virtual uint8_t BatteryVoltage(TractionMotorId motor_id) = 0;
    virtual int8_t Temperature(TractionMotorId motor_id) = 0;
    virtual bool EmergencyStopState(TractionMotorId motor_id) = 0;

    // Feedback - 조향 상태
    virtual int16_t SteeringAngle(TractionMotorId motor_id, SteerMotorId steer_id) = 0;
    virtual uint8_t SteeringErrorCode(TractionMotorId motor_id) = 0;
    virtual bool PortJ24Input9(TractionMotorId motor_id) = 0;

    // Control - 주행 제어
    virtual void EnableInterlock(TractionMotorId motor_id, bool enable) = 0;
    virtual void EnableForward(TractionMotorId motor_id, bool enable) = 0;
    virtual void EnableBackward(TractionMotorId motor_id, bool enable) = 0;
    virtual void EnableDriverJ19(TractionMotorId motor_id, bool enable) = 0;  // Driver6
    virtual void EnableForkDown(TractionMotorId motor_id, bool enable) = 0;  // Driver3 (J4)
    virtual void EnableForkUp(TractionMotorId motor_id, bool enable) = 0;  // Driver4 (J3)
    virtual void ResetAccumulator(TractionMotorId motor_id, bool enable) = 0;
    virtual void EnableDriverJ20(TractionMotorId motor_id, bool enable) = 0;  // Driver7

    virtual void SetTargetSpeed(TractionMotorId motor_id, uint16_t speed_rpm) = 0;
    virtual void SetAcceleration(TractionMotorId motor_id, uint8_t accel_time) = 0;  // 0.2~25.5s
    virtual void SetDeceleration(TractionMotorId motor_id, uint8_t decel_time) = 0;  // 0.2~25.5s

    // Control - 조향 제어
    virtual void SetSteeringAngle(TractionMotorId motor_id, SteerMotorId steer_id, int16_t angle) = 0;
    virtual void SetProportionalValve(TractionMotorId motor_id, uint16_t valve_value) = 0;  // 0~32767
};

class IHydraulics {
public:
    virtual ~IHydraulics() = default;

    // Feedback - 유압 시스템 상태
    virtual uint16_t FeedbackSpeed(HydraulicsMotorId motor_id) = 0;
    virtual uint16_t FeedbackCurrent(HydraulicsMotorId motor_id) = 0;
    virtual uint8_t ErrorCode(HydraulicsMotorId motor_id) = 0;
    virtual int8_t Temperature(HydraulicsMotorId motor_id) = 0;
    virtual bool PortJ9Input5() = 0;

    // Control - 유압 제어
    virtual void EnableInterlock(HydraulicsMotorId motor_id, bool enable) = 0;
    virtual void EnableForward(HydraulicsMotorId motor_id, bool enable) = 0;
    virtual void EnableBackward(HydraulicsMotorId motor_id, bool enable) = 0;
    virtual void EnableForkDown(HydraulicsMotorId motor_id, bool enable) = 0;
    virtual void EnableForkUp(HydraulicsMotorId motor_id, bool enable) = 0;
    virtual void EnableDriverJ19(HydraulicsMotorId motor_id, bool enable) = 0;
    virtual void EnableDriverJ20(HydraulicsMotorId motor_id, bool enable) = 0;

    virtual void SetTargetSpeed(HydraulicsMotorId motor_id, uint16_t speed_rpm) = 0;
    virtual void SetProportionalValve(HydraulicsMotorId motor_id, uint16_t valve_value) = 0;
    virtual void SetAcceleration(HydraulicsMotorId motor_id, uint8_t accel_time) = 0;
    virtual void SetDeceleration(HydraulicsMotorId motor_id, uint8_t decel_time) = 0;
};

// 통합 인터페이스
class IRTV3000
    : public ITraction
    , public IHydraulics {
public:
    virtual ~IRTV3000() = default;

    // 시스템 제어
    virtual bool Initialize() = 0;
    virtual bool Start() = 0;
    virtual void Stop() = 0;
    virtual void EmergencyStop() = 0;

    // 상태 조회
    virtual bool IsConnected() = 0;
    virtual bool HasError() = 0;
    virtual std::string GetLastError() = 0;
};

}  // namespace ThePrime
}  // namespace curtis
}  // namespace MotorDriver
}  // namespace NaviFra

#endif