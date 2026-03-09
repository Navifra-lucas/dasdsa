#ifndef NAVIFRA_NAVICAN_MODE_FORWARD_HELPER_H
#define NAVIFRA_NAVICAN_MODE_FORWARD_HELPER_H

#include "NaviCAN/NaviCANDriver.h"
#include "NaviCAN/canopen/cia402/ModeTagetHelper.h"

#include <cstdint>
#include <memory>

namespace NaviFra {
namespace NaviCAN {
namespace Canopen {
namespace CIA402 {

template <int8_t ID, typename TYPE, uint16_t OBJ, uint8_t SUB, uint16_t CW_MASK>
class ModeForwardHelper : public ModeTargetHelper<TYPE> {
    std::shared_ptr<NaviCANDriver> driver;

public:
    ModeForwardHelper(std::shared_ptr<NaviCANDriver> driver)
        : ModeTargetHelper<TYPE>(ID)
        , driver(driver)
    {
    }

    bool read(const uint16_t& sw) override
    {
        // Add logic here to read state or status if needed.
        return true;
    }

    bool write(Mode::OpModeAccesser& cw) override
    {
        if (this->hasTarget()) {
            cw = cw.get() | CW_MASK;  // Set control word for the mode

            // Write the target value to the CANopen object dictionary
            driver->universal_set_value<TYPE>(OBJ, SUB, this->getTarget());
            return true;
        }
        else {
            cw = cw.get() & ~CW_MASK;  // Clear control word for the mode
            return false;
        }
    }
};
}  // namespace CIA402
}  // namespace Canopen
}  // namespace NaviCAN

}  // namespace NaviFra

#endif  // MODE_FORWARD_HELPER_HPP