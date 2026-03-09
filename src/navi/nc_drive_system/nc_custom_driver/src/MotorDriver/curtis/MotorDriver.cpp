#include "MotorDriver/curtis/ThePrime/MotorDriver.h"

using namespace NaviFra::MotorDriver::curtis::ThePrime::RTV3000;

void MotorDriver::setupCANCallbacks()
{
    // A주행 상태 (0x183) - 타입 자동 추론 활용
    can_interface_->registerCallback(
        0x183, [this](uint8_t _, uint16_t speed, uint16_t current, uint8_t error_code, uint8_t voltage, int8_t temperature) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            auto& data = traction_data_[static_cast<size_t>(TractionMotorId::A)];
            data.speed = speed;
            data.current = current;
            data.error_code = error_code;
            data.voltage = voltage;
            data.temperature = temperature;
        });

    // B주행 상태 (0x184)
    can_interface_->registerCallback(
        0x184, [this](uint8_t _, uint16_t speed, uint16_t current, uint8_t error_code, uint8_t voltage, int8_t temperature) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            auto& data = traction_data_[static_cast<size_t>(TractionMotorId::B)];
            data.speed = speed;
            data.current = current;
            data.error_code = error_code;
            data.voltage = voltage;
            data.temperature = temperature;
        });

    // A 조향 각도 (0x283)
    can_interface_->registerCallback(0x283, [this](int16_t angle_as, int16_t angle_sa, uint16_t _, uint8_t emergency_flags, uint8_t __) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        auto& data = traction_data_[static_cast<size_t>(TractionMotorId::A)];
        data.steering_angles[0] = angle_as;
        data.steering_angles[1] = angle_sa;
        data.emergency_stop = (emergency_flags & 0x01) != 0;
        data.port_j24_input9 = (emergency_flags & 0x01) != 0;
    });

    // B 조향 각도 (0x284)
    can_interface_->registerCallback(0x284, [this](int16_t angle_bs, int16_t angle_sb, uint16_t _, uint8_t emergency_flags, uint8_t __) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        auto& data = traction_data_[static_cast<size_t>(TractionMotorId::B)];
        data.steering_angles[0] = angle_bs;
        data.steering_angles[1] = angle_sb;
        data.emergency_stop = (emergency_flags & 0x01) != 0;
        data.port_j24_input9 = (emergency_flags & 0x01) != 0;
    });

    // A 조향 에러 (0x383)
    can_interface_->registerCallback(0x383, [this](uint8_t acc0, uint8_t acc1, uint8_t acc2, uint8_t steer_error, uint16_t _) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        traction_data_[static_cast<size_t>(TractionMotorId::A)].steering_error = steer_error;
    });

    // B 조향 에러 (0x384)
    can_interface_->registerCallback(0x384, [this](uint8_t acc0, uint8_t acc1, uint8_t acc2, uint8_t steer_error, uint16_t _) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        traction_data_[static_cast<size_t>(TractionMotorId::B)].steering_error = steer_error;
    });

    // 유압 상태 (0x186)
    can_interface_->registerCallback(
        0x186, [this](uint8_t flags, uint16_t speed, uint16_t current, uint8_t error_code, uint8_t _, int8_t temperature) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            hydraulics_data_.port_j9_input5 = (flags & 0x01) != 0;
            hydraulics_data_.speed = speed;
            hydraulics_data_.current = current;
            hydraulics_data_.error_code = error_code;
            hydraulics_data_.temperature = temperature;
        });
}
void MotorDriver::sendTractionCommand(TractionMotorId motor_id)
{
    auto idx = static_cast<size_t>(motor_id);
    const auto& ctrl = traction_ctrl_[idx];

    uint32_t can_id = (motor_id == TractionMotorId::A) ? 0x203 : 0x204;

    can_interface_->sendMessage(
        can_id,
        ctrl.control_flags,  // uint8_t
        ctrl.target_speed,  // uint16_t (자동으로 Low/High byte 처리)
        ctrl.accel_rate,  // uint8_t
        ctrl.decel_rate,  // uint8_t
        uint16_t(0),  // uint16_t 보류 (2바이트)
        uint8_t(0)  // uint8_t 보류 (1바이트)
    );
}

void MotorDriver::sendSteeringCommand(TractionMotorId motor_id)
{
    auto idx = static_cast<size_t>(motor_id);
    const auto& ctrl = traction_ctrl_[idx];

    uint32_t can_id = (motor_id == TractionMotorId::A) ? 0x303 : 0x304;

    can_interface_->sendMessage(
        can_id,
        ctrl.target_angles[0],  // int16_t (자동으로 Low/High byte 처리)
        ctrl.target_angles[1],  // int16_t
        uint16_t(0),  // uint16_t 보류 (2바이트)
        ctrl.proportional_valve  // uint16_t
    );
}

void MotorDriver::sendHydraulicsCommand()
{
    can_interface_->sendMessage(
        0x206,
        hydraulics_ctrl_.control_flags,  // uint8_t
        hydraulics_ctrl_.target_speed,  // uint16_t
        hydraulics_ctrl_.proportional_valve,  // uint16_t
        uint8_t(0),  // uint8_t 보류
        hydraulics_ctrl_.accel_rate,  // uint8_t
        hydraulics_ctrl_.decel_rate  // uint8_t
    );
}

void MotorDriver::sendAllCommands()
{
    sendTractionCommand(TractionMotorId::A);
    sendTractionCommand(TractionMotorId::B);
    sendSteeringCommand(TractionMotorId::A);
    sendSteeringCommand(TractionMotorId::B);
    sendHydraulicsCommand();
}

bool MotorDriver::Initialize()
{
    if (!can_interface_->initialize()) {
        last_error_ = "Failed to initialize CAN interface";
        return false;
    }
    return true;
}

bool MotorDriver::Start()
{
    is_connected_.store(false);
    if (can_interface_) {
        can_interface_->stop();
    }
    return true;
}

void MotorDriver::Stop()
{
    is_connected_.store(false);
    if (can_interface_) {
        can_interface_->stop();
    }
}

void MotorDriver::EmergencyStop()
{
    // 모든 모터 즉시 정지
    for (auto& ctrl : traction_ctrl_) {
        ctrl.control_flags = 0x01;  // 상호 잠금만 유지
        ctrl.target_speed = 0;
    }
    hydraulics_ctrl_.control_flags = 0x01;
    hydraulics_ctrl_.target_speed = 0;

    sendAllCommands();
}

bool MotorDriver::HasError()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (const auto& data : traction_data_) {
        if (data.error_code != 0)
            return true;
    }
    return hydraulics_data_.error_code != 0;
}