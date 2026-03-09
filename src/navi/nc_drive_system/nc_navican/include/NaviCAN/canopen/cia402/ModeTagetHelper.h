#ifndef NAVIFRA_NAVICAN_MODE_TARGET_HELPER_H
#define NAVIFRA_NAVICAN_MODE_TARGET_HELPER_H

#include "NaviCAN/canopen/cia402/Mode.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>

namespace NaviFra {
namespace NaviCAN {
namespace Canopen {
namespace CIA402 {

template <typename TYPE>
class ModeTargetHelper : public Mode {
    TYPE target_;
    std::atomic<bool> has_target_;

public:
    ModeTargetHelper(int8_t mode)
        : Mode(mode)
        , has_target_(false)
    {
    }

    bool hasTarget() const { return has_target_; }

    TYPE getTarget() const { return target_; }

    virtual bool setTarget(const double& val)
    {
        if (std::isnan(val)) {
            std::cout << "Target command is not a number" << std::endl;
            return false;
        }

        // Clamp the value to the valid range
        double min_val = static_cast<double>(std::numeric_limits<TYPE>::lowest());
        double max_val = static_cast<double>(std::numeric_limits<TYPE>::max());

        double clamped = std::clamp(val, min_val, max_val);

        if (clamped != val) {
            std::cout << "Value " << val << " out of range, clamped to " << clamped << std::endl;
        }

        target_ = static_cast<TYPE>(clamped);
        has_target_ = true;
        return true;
    }

    virtual bool start()
    {
        has_target_ = false;
        return true;
    }
};
}  // namespace CIA402
}  // namespace Canopen
}  // namespace NaviCAN
}  // namespace NaviFra

#endif  // NAVIFRA_MODE_TARGET_HELPER_H