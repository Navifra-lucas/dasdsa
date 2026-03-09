#ifndef NAVIFRA_NAVICAN_CANOPEN_INDEX_CALCULATOR_H
#define NAVIFRA_NAVICAN_CANOPEN_INDEX_CALCULATOR_H

#include <array>
#include <cstdint>

namespace NaviFra {
namespace NaviCAN {
namespace Utils {

/**
 * @brief CANopen Index Calculator
 *
 * Base addresses per channel:
 * - Channel 0: 0x6000
 * - Channel 1: 0x6800
 * - Channel 2: 0x7000
 * - Channel 3: 0x7800
 * - Channel 4: 0x8000
 * - Channel 5: 0x8800
 * - Channel 6: 0x9000
 * - Channel 7: 0x9800
 */
class CANopenIndexCalculator {
public:
    static constexpr std::array<uint16_t, 8> BASE_OFFSETS = {0x6000, 0x6800, 0x7000, 0x7800, 0x8000, 0x8800, 0x9000, 0x9800};

    static constexpr uint16_t DEFAULT_BASE = 0x6000;

    static constexpr uint16_t getBase(uint8_t index) { return (index < BASE_OFFSETS.size()) ? BASE_OFFSETS[index] : DEFAULT_BASE; }

    static constexpr uint16_t calculate(uint8_t channel_index, uint16_t offset) { return getBase(channel_index) + offset; }

    struct CIA402Offsets {
        static constexpr uint16_t CONTROL_WORD = 0x0040;
        static constexpr uint16_t STATUS_WORD = 0x0041;
        static constexpr uint16_t OPERATION_MODE = 0x0060;
        static constexpr uint16_t OPERATION_MODE_DISPLAY = 0x0061;
        static constexpr uint16_t POSITION_ACTUAL_VALUE = 0x0064;
        static constexpr uint16_t VELOCITY_ACTUAL_VALUE = 0x006C;
        static constexpr uint16_t TORQUE_TARGET = 0x0071;
        static constexpr uint16_t TARGET_POSITION = 0x007A;
        static constexpr uint16_t HOMING_METHOD = 0x0098;
        static constexpr uint16_t INTERPOLATION_DATA_RECORD = 0x00C1;
        static constexpr uint16_t TARGET_VELOCITY = 0x00FF;
        static constexpr uint16_t VL_TARGET_VELOCITY = 0x0042;
    };

    static constexpr uint16_t getControlWordIndex(uint8_t channel_index) { return calculate(channel_index, CIA402Offsets::CONTROL_WORD); }

    static constexpr uint16_t getStatusWordIndex(uint8_t channel_index) { return calculate(channel_index, CIA402Offsets::STATUS_WORD); }

    static constexpr uint16_t getOperationModeIndex(uint8_t channel_index)
    {
        return calculate(channel_index, CIA402Offsets::OPERATION_MODE);
    }

    static constexpr uint16_t getOperationModeDisplayIndex(uint8_t channel_index)
    {
        return calculate(channel_index, CIA402Offsets::OPERATION_MODE_DISPLAY);
    }

    static constexpr uint16_t getPositionActualValueIndex(uint8_t channel_index)
    {
        return calculate(channel_index, CIA402Offsets::POSITION_ACTUAL_VALUE);
    }

    static constexpr uint16_t getVelocityActualValueIndex(uint8_t channel_index)
    {
        return calculate(channel_index, CIA402Offsets::VELOCITY_ACTUAL_VALUE);
    }
};

}  // namespace Utils
}  // namespace NaviCAN
}  // namespace NaviFra

#endif  // NAVIFRA_NAVICAN_CANOPEN_INDEX_CALCULATOR_H
