#ifndef NAVIFRA_NAVICAN_MOTOR_STATE_HANDLER_H
#define NAVIFRA_NAVICAN_MOTOR_STATE_HANDLER_H

#include "NaviCAN/Object/standard/StatusWord.hpp"

#include <chrono>
#include <condition_variable>
#include <mutex>

namespace NaviFra {
namespace NaviCAN {
namespace Canopen {
namespace CIA402 {

class StateHandler {
public:
    StateHandler();

    enum State
    {
        UNKNOWN = 0,
        START = 0,
        NOT_READY_TO_SWITCH_ON = 1,
        SWITCH_ON_DISABLED = 2,
        READY_TO_SWITCH_ON = 3,
        SWITCHED_ON = 4,
        OPERATION_ENABLE = 5,
        QUICK_STOP_ACTIVE = 6,
        FAULT_REACTION_ACTIVE = 7,
        FAULT = 8,
    };

    State getState();
    State updateState(NaviFra::NaviCAN::Object::Standard::StatusWord& sw);
    std::string getStateText();
    bool waitForNewState(const std::chrono::steady_clock::time_point& abstime, State& state);

private:
    std::condition_variable cond_;
    std::mutex mutex_;
    State state_;
};

}  // namespace CIA402
}  // namespace Canopen
}  // namespace NaviCAN
}  // namespace NaviFra

#endif  // NAVIFRA_MOTOR_STATE_H
