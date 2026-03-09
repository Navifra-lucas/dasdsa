#include "NaviCAN/Object/custom/swzm/StatusByte.h"

namespace NaviFra {
namespace NaviCAN {
namespace Object {
namespace Custom {
namespace swzm {

StatusByte::StatusByte(uint8_t value)
    : BitField<uint8_t>(value)
{
}

bool StatusByte::isPowerStageActive() const
{
    return (getBit(POWER_STAGE_ACTIVE));
}

bool StatusByte::isFault() const
{
    return (getBit(FAULT));
}

bool StatusByte::isGeneratorMode() const
{
    return (getBit(TORQUE_MODE));
}

bool StatusByte::isAnalogModeActive() const
{
    return (getBit(MODE_ANALOG));
}

bool StatusByte::isCANEnabled() const
{
    return (getBit(ENABLE_INPUT));
}

bool StatusByte::isCcw() const
{
    return (getBit(CCW_INPUT));
}

bool StatusByte::isSafeStopActive() const
{
    return (getBit(SAFE_STOP_ACTIVE));
}

bool StatusByte::isTOgglebitSet() const
{
    return (getBit(TOGGLE_BIT));
}

} // namespace swzm
} // namespace Custom
} // namespace Object
} // namespace NaviCAN
} // namespace NaviFra