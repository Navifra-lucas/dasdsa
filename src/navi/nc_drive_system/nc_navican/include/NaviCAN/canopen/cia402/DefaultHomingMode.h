#ifndef NAVIFRA_NAVICAN_CANOPEN_CIA402_DEFAULT_HOMING_MOTOR_MODE_H
#define NAVIFRA_NAVICAN_CANOPEN_CIA402_DEFAULT_HOMING_MOTOR_MODE_H

#include "NaviCAN/NaviCANDriver.h"
#include "NaviCAN/Object/standard/StatusWord.hpp"
#include "NaviCAN/canopen/cia402/HomingMode.h"

#include <chrono>
#include <memory>
#include <mutex>

using namespace NaviFra::NaviCAN::Object::Standard;
using namespace std::chrono_literals;

namespace NaviFra {
namespace NaviCAN {
namespace Canopen {
namespace CIA402 {

template <int8_t ID, typename TYPE, uint16_t OBJ, uint8_t SUB, uint16_t CW_MASK>
class DefaultHomingMode : public HomingMode {
public:
    DefaultHomingMode(std::shared_ptr<NaviCANDriver> driver)
        : HomingMode(ID)
        , driver_(driver)
    {
    }

    bool start() override { return true; }

    bool read(const uint16_t& sw) override
    {
        sw_.store(sw);
        return true;
    }

    bool write(Mode::OpModeAccesser& cw) override
    {
        if (execute_) {
            cw.set(CWBits::HomingOperationStart);
        }
        else {
            cw.reset(CWBits::HomingOperationStart);
        }
        return true;
    }

    bool setHomingMethod(int8_t homing_method) override
    {
        homing_method_ = homing_method;
        if (driver_->sync_sdo_write_typed<TYPE>(OBJ, SUB, homing_method, 1s) == false) {
            NLOG(info) << "cannot set homing method, OBJ :" << std::hex << (int)OBJ << ", homing_method=" << (int)homing_method;
            return false;
        }

        return true;
    }

    bool homing(std::chrono::milliseconds timeout) override
    {
        // HOMING: CHECK BEFOR START
        std::this_thread::sleep_for(100ms);
        // if (!(sw_ & (1 << SWBits::HomingTargetReached))) {
        //     NLOG(info) << "homing: HomingTargetReached bit is not set, cannot start homing";
        //     return false;
        // }

        // HOMING: START
        execute_ = true;

        auto start_time = std::chrono::steady_clock::now();
        auto deadline = start_time + timeout;

        while (std::chrono::steady_clock::now() < deadline) {
            uint16_t sw = sw_.load();

            // Check if homing is completed successfully
            if (sw & (1 << SWBits::HomingAttained)) {
                execute_ = false;
                NLOG(info) << "homing: completed successfully";
                return true;
            }

            if ((sw & 0x003F) != SWBits::OperationEnabled) {
                execute_ = false;
                NLOG(info) << "homing: operation not enabled";
                return false;
            }

            // Check if homing error occurred
            if (sw & (1 << SWBits::HomingError)) {
                execute_ = false;
                NLOG(info) << "homing: error occurred";
                return false;
            }

            std::this_thread::sleep_for(1ms);
        }

        // HOMING : END (timeout)
        execute_ = false;
        NLOG(info) << "homing: timeout reached (" << timeout.count() << "ms)";
        return false;
    }

private:
    std::shared_ptr<NaviCANDriver> driver_;
    std::atomic<bool> execute_{false};

    std::atomic<uint16_t> sw_{0};
    std::atomic<TYPE> homing_method_{0};
};

}  // namespace CIA402
}  // namespace Canopen
}  // namespace NaviCAN
}  // namespace NaviFra

#endif
