#ifndef NAVIFRA_NAVICAN_CANOPEN_CIA402_HOMING_MOTOR_MODE_H
#define NAVIFRA_NAVICAN_CANOPEN_CIA402_HOMING_MOTOR_MODE_H

#include "NaviCAN/Object/standard/ControlWord.hpp"
#include "NaviCAN/Object/standard/StatusWord.hpp"
#include "NaviCAN/canopen/cia402/Mode.h"

#include <atomic>
#include <chrono>

using namespace NaviFra::NaviCAN::Object::Standard;

namespace NaviFra {
namespace NaviCAN {
namespace Canopen {
namespace CIA402 {

class HomingMode : public Mode {
public:
    virtual bool homing(std::chrono::milliseconds timeout) = 0;
    virtual bool setHomingMethod(int8_t homing_method) = 0;

protected:
    HomingMode(int8_t mode_id)
        : Mode(mode_id)
    {
    }

    enum SWBits
    {
        HomingAttained = StatusWord::Bits::OPERATION_MODE_SPECIFIC0,
        HomingError = StatusWord::Bits::OPERATION_MODE_SPECIFIC1,
        HomingTargetReached = StatusWord::Bits::TARGET_REACHED,

        OperationEnabled = 0x0037
    };

    enum CWBits
    {
        HomingOperationStart = ControlWord::Bits::OPERATION_MODE_SPECIFIC0,
    };
};

}  // namespace CIA402
}  // namespace Canopen
}  // namespace NaviCAN
}  // namespace NaviFra

#endif