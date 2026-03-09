#pragma once

#include <array>
#include <cstdint>

namespace NaviFra {
namespace MotorDriver {
namespace curtis {
namespace ThePrime {
namespace RTV3000 {

// 상태 데이터 구조체
struct TractionData {
    uint16_t speed = 0;
    uint16_t current = 0;
    uint8_t error_code = 0;
    uint8_t voltage = 0;
    int8_t temperature = 0;
    bool emergency_stop = false;

    // 조향 데이터
    std::array<int16_t, 2> steering_angles = {0, 0};  // [Steer1, Steer2]
    uint8_t steering_error = 0;
    bool port_j24_input9 = false;
};

struct HydraulicsData {
    uint16_t speed = 0;
    uint16_t current = 0;
    uint8_t error_code = 0;
    int8_t temperature = 0;
    bool port_j9_input5 = false;
};

// 제어 데이터 구조체
struct TractionControl {
    uint8_t control_flags = 0;
    uint16_t target_speed = 0;
    uint8_t accel_rate = 10;
    uint8_t decel_rate = 10;

    // 조향 제어
    std::array<int16_t, 2> target_angles = {0, 0};
    uint16_t proportional_valve = 0;
};

struct HydraulicsControl {
    uint8_t control_flags = 0;
    uint16_t target_speed = 0;
    uint16_t proportional_valve = 0;
    uint8_t accel_rate = 10;
    uint8_t decel_rate = 10;
};

}  // namespace RTV3000
}  // namespace ThePrime
}  // namespace curtis
}  // namespace MotorDriver
}  // namespace NaviFra
