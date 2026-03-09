#include "NaviCAN/Object/custom/swzm/ControlWord.h"

namespace NaviFra {
namespace NaviCAN {
namespace Object {
namespace Custom {
namespace swzm {

ControlWord::ControlWord(uint16_t value)
    : BitField<uint16_t>(value)
{
}

void ControlWord::enableDrive(bool value)
{
    update([value](uint16_t current) -> uint16_t {
        return value ? (current | (1 << DRIVE_ENABLE)) : (current & ~(1 << DRIVE_ENABLE));
    });
}

void ControlWord::enableMainContactor(bool value)
{
    update([value](uint16_t current) -> uint16_t {
        return value ? (current | (1 << MAIN_CONTACTOR_ENABLE)) : (current & ~(1 << MAIN_CONTACTOR_ENABLE));
    });
}

void ControlWord::enableBrake(bool value)
{
    update([value](uint16_t current) -> uint16_t {
        return value ? (current | (1 << BRAKE_RELEASE)) : (current & ~(1 << BRAKE_RELEASE));
    });
}

void ControlWord::enableSafeStop(bool value)
{
    update([value](uint16_t current) -> uint16_t {
        return value ? (current | (1 << SAFE_STOP_FUNCTION)) : (current & ~(1 << SAFE_STOP_FUNCTION));
    });
}

void ControlWord::faultReset()
{
    update([](uint16_t current) -> uint16_t {
        // 비트 8 (FAULT_RESET) 설정
        return current | (1 << RESET_FAULT);
    });
}

}  // namespace swzm
}  // namespace Custom
}  // namespace Object
}  // namespace NaviCAN
}  // namespace NaviFra