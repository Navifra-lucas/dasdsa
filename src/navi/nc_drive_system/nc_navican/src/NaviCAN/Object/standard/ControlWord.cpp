#include "NaviCAN/Object/standard/ControlWord.hpp"

namespace NaviFra {
namespace NaviCAN {
namespace Object {
namespace Standard {


ControlWord::ControlWord(uint16_t value)
    : BitField<uint16_t>(value)
{
}

void ControlWord::shutdown()
{
    update([](uint16_t current) -> uint16_t {
        // 비트 패턴: xxxx xxxx x0xx 0110
        return (current & 0xFF00) | 0x0006;
    });
}

void ControlWord::switchOn()
{
    update([](uint16_t current) -> uint16_t {
        // 비트 패턴: xxxx xxxx x0xx 0111
        return (current & 0xFF00) | 0x0007;
    });
}

void ControlWord::disableVoltage()
{
    update([](uint16_t current) -> uint16_t {
        // 비트 패턴: xxxx xxxx x0xx 0000
        return (current & 0xFF00);
    });
}

void ControlWord::QuickStop()
{
    update([](uint16_t current) -> uint16_t {
        // 비트 패턴: xxxx xxxx x0xx 0010
        return (current & 0xFF00) | 0x0002;
    });
}

void ControlWord::disableOperation()
{
    update([](uint16_t current) -> uint16_t {
        // 비트 패턴: xxxx xxxx x0xx 0111
        return (current & 0xFF00) | 0x0007;
    });
}

void ControlWord::enableOperation()
{
    update([](uint16_t current) -> uint16_t {
        // 비트 패턴: xxxx xxxx x0xx 1111
        return (current & 0xFF00) | 0x000F;
    });
}

void ControlWord::faultReset(bool value)
{
    update([value](uint16_t current) -> uint16_t {
        // 비트 8 (FAULT_RESET) 설정
        if(value ==true) {
            return current | (1 << FAULT_RESET);
        } else {
            return current & ~(1 << FAULT_RESET);
        }

    });
}

}  // namespace Standard
}  // namespace Object
}  // namespace NaviCAN
}  // namespace NaviFra