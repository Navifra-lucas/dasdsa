#ifndef NAVIFRA_NAVICAN_MOTOR_DRIVER_CUSTOM_WELCON_H
#define NAVIFRA_NAVICAN_MOTOR_DRIVER_CUSTOM_WELCON_H

#include "NaviCAN/MotorDriver/MotorDriver.hpp"

namespace NaviFra {
namespace NaviCAN {
namespace MotorDriver {
namespace Custom {
namespace welcon {

class MotorDriver : public NaviFra::MotorDriver {
public:
    void preset(int32_t required_position = 0)
    {
        auto current_position = getPosition();
        auto current_offset = getActaulPositionOffset();
        setActualPositionOffset(current_position + current_offset + required_position);
    }

private:
    void setActualPositionOffset(int32_t value) { setValue<int32_t>(actual_position_offset_index, 0x00, value); }
    int32_t getActaulPositionOffset() { getValue<int32>(actual_position_offset_index, 0x00); }
    const uint16_t actual_position_offset_index = 0x20a5;
};

}  // namespace welcon
}  // namespace Custom
}  // namespace MotorDriver
}  // namespace NaviCAN
}  // namespace NaviFra

#endif