#pragma once

#include "MotorDriver/curtis/ThePrime/data/RTV3000Data.h"
#include "MotorDriver/curtis/ThePrime/interface/IRTV3000.h"

#include <nc_can_handler/Can.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>

namespace NaviFra {
namespace MotorDriver {
namespace curtis {
namespace ThePrime {
namespace RTV3000 {

class MotorDriver : public IRTV3000 {
public:
    explicit MotorDriver(const std::string& can_interface = "can0")
    {
        can_interface_ = std::make_unique<CanHandler::Can>(can_interface);
        setupCANCallbacks();
    }

    ~MotorDriver() override { Stop(); }

    // Traction - Feedback
    uint16_t FeedbackSpeed(TractionMotorId motor_id) override { return getTractionData(motor_id, &TractionData::speed); }
    uint16_t FeedbackCurrent(TractionMotorId motor_id) override { return getTractionData(motor_id, &TractionData::current); }
    uint8_t ErrorCode(TractionMotorId motor_id) override { return getTractionData(motor_id, &TractionData::error_code); }
    uint8_t BatteryVoltage(TractionMotorId motor_id) override { return getTractionData(motor_id, &TractionData::voltage); }
    int8_t Temperature(TractionMotorId motor_id) override { return getTractionData(motor_id, &TractionData::temperature); }
    bool EmergencyStopState(TractionMotorId motor_id) override { return getTractionData(motor_id, &TractionData::emergency_stop); }
    uint8_t SteeringErrorCode(TractionMotorId motor_id) override { return getTractionData(motor_id, &TractionData::steering_error); }
    bool PortJ24Input9(TractionMotorId motor_id) override { return getTractionData(motor_id, &TractionData::port_j24_input9); }

    int16_t SteeringAngle(TractionMotorId motor_id, SteerMotorId steer_id) override
    {
        return threadSafeGet<int16_t>([this, motor_id, steer_id]() {
            return traction_data_[static_cast<size_t>(motor_id)].steering_angles[static_cast<size_t>(steer_id)];
        });
    }

    // 트랙션 제어 - Target
    void EnableInterlock(TractionMotorId motor_id, bool enable) override { setTractionControlBit(motor_id, 0, enable); }
    void EnableForward(TractionMotorId motor_id, bool enable) override { setTractionControlBit(motor_id, 1, enable, true, 2); }
    void EnableBackward(TractionMotorId motor_id, bool enable) override { setTractionControlBit(motor_id, 2, enable, true, 1); }
    void EnableDriverJ19(TractionMotorId motor_id, bool enable) override { setTractionControlBit(motor_id, 3, enable); }
    void EnableForkDown(TractionMotorId motor_id, bool enable) override { setTractionControlBit(motor_id, 4, enable); }
    void EnableForkUp(TractionMotorId motor_id, bool enable) override { setTractionControlBit(motor_id, 5, enable); }
    void EnableDriverJ20(TractionMotorId motor_id, bool enable) override { setTractionControlBit(motor_id, 7, enable); }
    void ResetAccumulator(TractionMotorId motor_id, bool enable) override
    {
        // 리셋 후 클리어 필요한지 확인
        setTractionControlBit(motor_id, 6, enable);
        if (enable)
            setTractionControlBit(motor_id, 6, false);
    }

    void SetTargetSpeed(TractionMotorId motor_id, uint16_t speed_rpm) override
    {
        setTractionValue(motor_id, &TractionControl::target_speed, speed_rpm, static_cast<uint16_t>(6000));
    }

    void SetAcceleration(TractionMotorId motor_id, uint8_t accel_time) override
    {
        setTractionValue(motor_id, &TractionControl::accel_rate, accel_time, static_cast<uint8_t>(2), static_cast<uint8_t>(255));
    }

    void SetDeceleration(TractionMotorId motor_id, uint8_t decel_time) override
    {
        setTractionValue(motor_id, &TractionControl::decel_rate, decel_time, static_cast<uint8_t>(2), static_cast<uint8_t>(255));
    }

    void SetSteeringAngle(TractionMotorId motor_id, SteerMotorId steer_id, int16_t angle) override
    {
        setTractionArray(
            motor_id, &TractionControl::target_angles, static_cast<size_t>(steer_id),
            std::max(static_cast<int16_t>(-16000), std::min(static_cast<int16_t>(16000), angle)));
        sendSteeringCommand(motor_id);
    }

    void SetProportionalValve(TractionMotorId motor_id, uint16_t valve_value) override
    {
        setTractionValue(motor_id, &TractionControl::proportional_valve, valve_value, static_cast<uint16_t>(32767));
    }

    // Hydraulics - 이미 한 줄로 간소화됨
    uint16_t FeedbackSpeed(HydraulicsMotorId motor_id) override { return getHydraulicsData(&HydraulicsData::speed); }
    uint16_t FeedbackCurrent(HydraulicsMotorId motor_id) override { return getHydraulicsData(&HydraulicsData::current); }
    uint8_t ErrorCode(HydraulicsMotorId motor_id) override { return getHydraulicsData(&HydraulicsData::error_code); }
    int8_t Temperature(HydraulicsMotorId motor_id) override { return getHydraulicsData(&HydraulicsData::temperature); }
    bool PortJ9Input5() override { return getHydraulicsData(&HydraulicsData::port_j9_input5); }
    void EnableInterlock(HydraulicsMotorId motor_id, bool enable) override { setHydraulicsBit(0, enable); }
    void EnableForward(HydraulicsMotorId motor_id, bool enable) override { setHydraulicsBit(1, enable, true, 2); }
    void EnableBackward(HydraulicsMotorId motor_id, bool enable) override { setHydraulicsBit(2, enable, true, 1); }
    void EnableForkDown(HydraulicsMotorId motor_id, bool enable) override { setHydraulicsBit(3, enable); }
    void EnableForkUp(HydraulicsMotorId motor_id, bool enable) override { setHydraulicsBit(4, enable); }
    void EnableDriverJ19(HydraulicsMotorId motor_id, bool enable) override { setHydraulicsBit(5, enable); }
    void EnableDriverJ20(HydraulicsMotorId motor_id, bool enable) override { setHydraulicsBit(6, enable); }

    void SetTargetSpeed(HydraulicsMotorId motor_id, uint16_t speed_rpm) override
    {
        setHydraulicsValue(&HydraulicsControl::target_speed, speed_rpm, static_cast<uint16_t>(6000));
    }

    void SetProportionalValve(HydraulicsMotorId motor_id, uint16_t valve_value) override
    {
        setHydraulicsValue(&HydraulicsControl::proportional_valve, valve_value, static_cast<uint16_t>(32767));
    }

    void SetAcceleration(HydraulicsMotorId motor_id, uint8_t accel_time) override
    {
        setHydraulicsValue(&HydraulicsControl::accel_rate, accel_time, static_cast<uint8_t>(2), static_cast<uint8_t>(255));
    }

    void SetDeceleration(HydraulicsMotorId motor_id, uint8_t decel_time) override
    {
        setHydraulicsValue(&HydraulicsControl::decel_rate, decel_time, static_cast<uint8_t>(2), static_cast<uint8_t>(255));
    }

    // IRTV3000 인터페이스 구현
    bool Initialize() override;
    bool Start() override;
    void Stop() override;
    void EmergencyStop() override;
    bool IsConnected() override { return is_connected_.load(); }
    bool HasError() override;
    std::string GetLastError() override { return last_error_; }
    void setupCANCallbacks();

private:
    std::unique_ptr<CanHandler::Can> can_interface_;
    mutable std::mutex data_mutex_;
    std::atomic<bool> is_connected_{false};
    std::string last_error_;

    std::array<RTV3000::TractionData, 2> traction_data_;
    std::array<RTV3000::TractionControl, 2> traction_ctrl_;
    RTV3000::HydraulicsData hydraulics_data_;
    RTV3000::HydraulicsControl hydraulics_ctrl_;

    template <typename T, typename F>
    T threadSafeGet(F&& getter) const
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return getter();
    }

    template <typename F>
    void threadSafeSet(F&& setter)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        setter();
    }

    void setBitFlag(uint8_t& flags, int bit_pos, bool enable)
    {
        if (enable)
            flags |= (1 << bit_pos);
        else
            flags &= ~(1 << bit_pos);
    }

    // 트랙션 헬퍼들
    template <typename T>
    T getTractionData(TractionMotorId motor_id, T TractionData::*member) const
    {
        return threadSafeGet<T>([this, motor_id, member]() { return traction_data_[static_cast<size_t>(motor_id)].*member; });
    }

    void setTractionControlBit(TractionMotorId motor_id, int bit_pos, bool enable, bool disable_conflict = false, int conflict_bit = -1)
    {
        auto idx = static_cast<size_t>(motor_id);
        setBitFlag(traction_ctrl_[idx].control_flags, bit_pos, enable);
        if (enable && disable_conflict && conflict_bit >= 0) {
            setBitFlag(traction_ctrl_[idx].control_flags, conflict_bit, false);
        }
        sendTractionCommand(motor_id);
    }

    template <typename T>
    void setTractionValue(TractionMotorId motor_id, T TractionControl::*member, const T& value, const T& max_val)
    {
        traction_ctrl_[static_cast<size_t>(motor_id)].*member = std::min(value, max_val);
        sendTractionCommand(motor_id);
    }

    template <typename T>
    void setTractionValue(TractionMotorId motor_id, T TractionControl::*member, const T& value, const T& min_val, const T& max_val)
    {
        traction_ctrl_[static_cast<size_t>(motor_id)].*member = std::max(min_val, std::min(value, max_val));
        sendTractionCommand(motor_id);
    }

    
    template <typename T, size_t N>
    void setTractionArray(TractionMotorId motor_id, std::array<T, N> TractionControl::*member, size_t index, const T& value)
    {
        (traction_ctrl_[static_cast<size_t>(motor_id)].*member)[index] = value;
    }

    // 유압 헬퍼들
    template <typename T>
    T getHydraulicsData(T HydraulicsData::*member) const
    {
        return threadSafeGet<T>([this, member]() { return hydraulics_data_.*member; });
    }

    void setHydraulicsBit(int bit_pos, bool enable, bool disable_conflict = false, int conflict_bit = -1)
    {
        setBitFlag(hydraulics_ctrl_.control_flags, bit_pos, enable);
        if (enable && disable_conflict && conflict_bit >= 0) {
            setBitFlag(hydraulics_ctrl_.control_flags, conflict_bit, false);
        }
        sendHydraulicsCommand();
    }

    template <typename T>
    void setHydraulicsValue(T HydraulicsControl::*member, const T& value, const T& max_val)
    {
        hydraulics_ctrl_.*member = std::min(value, max_val);
        sendHydraulicsCommand();
    }

    template <typename T>
    void setHydraulicsValue(T HydraulicsControl::*member, const T& value, const T& min_val, const T& max_val)
    {
        hydraulics_ctrl_.*member = std::max(min_val, std::min(value, max_val));
        sendHydraulicsCommand();
    }

    void sendTractionCommand(TractionMotorId motor_id);
    void sendSteeringCommand(TractionMotorId motor_id);
    void sendHydraulicsCommand();
    void sendAllCommands();
};

}  // namespace RTV3000
}  // namespace ThePrime
}  // namespace curtis
}  // namespace MotorDriver
}  // namespace NaviFra