#ifndef NAVIFRA_NAVICAN_PROFILED_POSITION_MODE_H
#define NAVIFRA_NAVICAN_PROFILED_POSITION_MODE_H

#include "NaviCAN/NaviCANDriver.h"
#include "NaviCAN/Object/standard/ControlWord.hpp"
#include "NaviCAN/Object/standard/StatusWord.hpp"
#include "NaviCAN/canopen/cia402/Command.h"
#include "NaviCAN/canopen/cia402/Mode.h"
#include "NaviCAN/canopen/cia402/ModeTagetHelper.h"
#include "NaviCAN/canopen/cia402/interface/IMotorDriver.h"

#include <cstdint>
#include <limits>
#include <memory>

using namespace NaviFra::NaviCAN::Object::Standard;

namespace NaviFra {
namespace NaviCAN {
namespace Canopen {
namespace CIA402 {
template <
    int8_t ID, typename TYPE, uint16_t TARGET_POS_OBJ, uint8_t TARGET_POS_SUB, uint16_t ACTUAL_POS_OBJ, uint8_t ACTUAL_POS_SUB,
    uint16_t CW_MASK>
class DefaultProfiledPositionMode : public ModeTargetHelper<TYPE> {
    std::shared_ptr<NaviCANDriver> driver;

    TYPE last_target_{0};
    uint16_t sw_{0};

    enum class State
    {
        IDLE,
        SETTING_TARGET,
        WAITING_ACKNOWLEDGE,
        MOTION_ACTIVE,
        TARGET_REACHED
    };
    State state_{State::IDLE};
    State prev_state_{State::IDLE};

    void setState(State new_state)
    {
        if (state_ != new_state) {
            prev_state_ = state_;
            state_ = new_state;

            const char* state_names[] = {"IDLE", "SETTING_TARGET", "WAITING_ACKNOWLEDGE", "MOTION_ACTIVE", "TARGET_REACHED"};

            NLOG(info) << "State changed: " << state_names[static_cast<int>(prev_state_)] << " -> "
                       << state_names[static_cast<int>(state_)];
        }
    }

public:
    DefaultProfiledPositionMode(std::shared_ptr<NaviCANDriver> driver)
        : ModeTargetHelper<TYPE>(ID)
        , driver(driver)
    {
    }
    enum SWMasks
    {
        MASK_REACHED = (1 << StatusWord::Bits::TARGET_REACHED),
        MASK_ACKNOWLEDGED = (1 << StatusWord::Bits::OPERATION_MODE_SPECIFIC0),
        MASK_ERROR = (1 << StatusWord::Bits::OPERATION_MODE_SPECIFIC1),
        MASK_OPERATION_ENABLED = 0x0037  // Operation Enabled 상태
    };

    enum CWBits
    {
        CW_NEW_POINT = ControlWord::Bits::OPERATION_MODE_SPECIFIC0,  // bit 4
        CW_IMMEDIATE = ControlWord::Bits::OPERATION_MODE_SPECIFIC1,  // bit 5
        CW_ABSOLUTE_POSITION = ControlWord::Bits::OPERATION_MODE_SPECIFIC2,  // bit 6
    };

    bool hasTarget() const { return true; }

    virtual bool start()
    {
        setState(State::IDLE);
        last_target_ = driver->universal_get_value<int32_t>(ACTUAL_POS_OBJ, ACTUAL_POS_SUB);
        this->setTarget(last_target_);
        return ModeTargetHelper<TYPE>::start();
    }

    virtual bool read(const uint16_t& sw)
    {
        sw_ = sw;

        // Operation Enabled 상태 확인
        if ((sw_ & 0x003F) != MASK_OPERATION_ENABLED) {
            // Not in Operation Enabled state
            return false;
        }

        return true;
    }

    virtual bool write(Mode::OpModeAccesser& cw)
    {
        if (!this->hasTarget()) {
            setState(State::IDLE);
            return false;
        }

        if ((sw_ & 0x003F) != MASK_OPERATION_ENABLED) {
            cw.set(CW_NEW_POINT);
            cw.set(CW_IMMEDIATE);  // bit 5: Change set immediately
            cw.set(CW_ABSOLUTE_POSITION);  // bit 6: Absolute position

            setState(State::IDLE);
            return false;
        }

        TYPE target = this->getTarget();
        auto target_position = static_cast<int32_t>(target);

        switch (state_) {
            case State::IDLE:
                if (target != last_target_) {
                    // 새로운 목표 위치가 있음
                    setState(State::SETTING_TARGET);
                }
                break;

            case State::SETTING_TARGET:

                driver->universal_set_value<int32_t>(TARGET_POS_OBJ, TARGET_POS_SUB, target_position);
                std::this_thread::sleep_for(1s);

                // 2. New Setpoint bit = 1
                cw.set(CW_NEW_POINT);
                cw.set(CW_IMMEDIATE);  // bit 5: Change set immediately
                cw.set(CW_ABSOLUTE_POSITION);  // bit 6: Absolute position

                last_target_ = target;
                NLOG(info) << "new target set: " << target_position;
                setState(State::WAITING_ACKNOWLEDGE);
                break;

            case State::WAITING_ACKNOWLEDGE:
                // Set-point Acknowledge 대기
                if (sw_ & MASK_ACKNOWLEDGED) {
                    // Acknowledge 받음 → bit 4를 0으로
                    cw.reset(CW_NEW_POINT);
                    cw.reset(CW_IMMEDIATE);
                    cw.reset(CW_ABSOLUTE_POSITION);

                    NLOG(info) << "target acknowledged: " << target_position;
                    setState(State::MOTION_ACTIVE);
                }
                else {
                    // 아직 Acknowledge 안됨 → bit 4 유지
                    cw.set(CW_NEW_POINT);
                }
                break;

            case State::MOTION_ACTIVE:
                // bit 4는 0 유지
                cw.reset(CW_NEW_POINT);
                cw.reset(CW_IMMEDIATE);
                cw.reset(CW_ABSOLUTE_POSITION);
                // Target Reached 확인
                if (sw_ & MASK_REACHED) {
                    setState(State::TARGET_REACHED);
                }

                // 새로운 목표가 있으면 다시 시작
                if (target != last_target_) {
                    setState(State::SETTING_TARGET);
                }
                break;

            case State::TARGET_REACHED:
                cw.reset(CW_NEW_POINT);
                cw.reset(CW_IMMEDIATE);
                cw.reset(CW_ABSOLUTE_POSITION);

                // 새로운 목표가 있으면 다시 시작
                if (target != last_target_) {
                    setState(State::SETTING_TARGET);
                }
                else {
                    setState(State::IDLE);
                }
                break;
        }

        return true;
    }

    bool isTargetReached() const { return state_ == State::TARGET_REACHED; }

    State getState() const { return state_; }
};

}  // namespace CIA402
}  // namespace Canopen
}  // namespace NaviCAN
}  // namespace NaviFra

#endif  // NAVIFRA_PROFILED_POSITION_MODE_H