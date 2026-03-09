#ifndef NAVIFRA_NAVICAN_CANOPEN_CIA402_MODE_H
#define NAVIFRA_NAVICAN_CANOPEN_CIA402_MODE_H

#include "NaviCAN/Object/standard/ControlWord.hpp"
#include "NaviCAN/Utils/WordAccesser.h"
#include "NaviCAN/canopen/cia402/Command.h"
#include "NaviCAN/canopen/cia402/StateHandler.h"

#include <cstdint>
#include <memory>

using namespace NaviFra::NaviCAN::Object::Standard;

namespace NaviFra {
namespace NaviCAN {
namespace Canopen {
namespace CIA402 {

class Mode {
public:
    const int8_t mode_id_;
    Mode(int8_t id)
        : mode_id_(id)
    {
    }
    ~Mode() = default;

    using OpModeAccesser = WordAccessor<
        (1 << ControlWord::Bits::OPERATION_MODE_SPECIFIC0) | (1 << ControlWord::Bits::OPERATION_MODE_SPECIFIC1) |
        (1 << ControlWord::Bits::OPERATION_MODE_SPECIFIC2)>;

    virtual bool start() = 0;
    virtual bool read(const uint16_t& sw) = 0;
    virtual bool write(OpModeAccesser& cw) = 0;
    virtual bool setTarget(const double& val) { return false; }
};

using ModeSharedPtr = std::shared_ptr<Mode>;

}  // namespace CIA402
}  // namespace Canopen
}  // namespace NaviCAN
}  // namespace NaviFra

#endif  // NAVIFRA_MOTOR_MODE_HPP
