#include "NaviCAN/canopen/cia402/StateHandler.h"

#include "NaviCAN/Object/standard/StatusWord.hpp"

#include <mutex>

using namespace NaviFra;
using namespace NaviFra::NaviCAN::Canopen::CIA402;
using namespace NaviFra::NaviCAN::Object::Standard;

StateHandler::StateHandler()
    : state_(State::UNKNOWN)
{
}

StateHandler::State StateHandler::getState()
{
    std::scoped_lock lock(mutex_);
    return state_;
}

StateHandler::State StateHandler::updateState(NaviFra::NaviCAN::Object::Standard::StatusWord& sw)
{
    static const uint16_t r = (1 << StatusWord::Bits::READY_TO_SWITCH_ON);
    static const uint16_t s = (1 << StatusWord::Bits::SWITCHED_ON);
    static const uint16_t o = (1 << StatusWord::Bits::OPERATION_ENABLED);
    static const uint16_t f = (1 << StatusWord::Bits::FAULT);
    static const uint16_t q = (1 << StatusWord::Bits::QUICK_STOP);
    static const uint16_t d = (1 << StatusWord::Bits::SWITCH_ON_DISABLED);

    State new_state = State::UNKNOWN;
    auto value = sw.getValue();
    uint16_t state = value & (d | q | f | o | s | r);
    switch (state) {
        case (0 | 0 | 0 | 0 | 0 | 0):
        case (0 | q | 0 | 0 | 0 | 0):
            new_state = NOT_READY_TO_SWITCH_ON;
            break;

        case (d | 0 | 0 | 0 | 0 | 0):
        case (d | q | 0 | 0 | 0 | 0):
            new_state = SWITCH_ON_DISABLED;
            break;

        case (0 | q | 0 | 0 | 0 | r):
            new_state = READY_TO_SWITCH_ON;
            break;

        case (0 | q | 0 | 0 | s | r):
            new_state = SWITCHED_ON;
            break;

        case (0 | q | 0 | o | s | r):
            new_state = OPERATION_ENABLE;
            break;

        case (0 | 0 | 0 | o | s | r):
            new_state = QUICK_STOP_ACTIVE;
            break;

        case (0 | 0 | f | o | s | r):
        case (0 | q | f | o | s | r):
            new_state = FAULT_REACTION_ACTIVE;
            break;

        case (0 | 0 | f | 0 | 0 | 0):
        case (0 | q | f | 0 | 0 | 0):
            new_state = FAULT;
            break;

        default:
            break;
    }

    std::scoped_lock lock(mutex_);
    if (new_state != state_) {
        state_ = new_state;
        cond_.notify_all();
    }
    return state_;
}

std::string StateHandler::getStateText()
{
    auto state = getState();
    switch (state) {
        case State::START:
            return "START";
        case State::NOT_READY_TO_SWITCH_ON:
            return "NOT_READY_TO_SWITCH_ON";
        case State::SWITCH_ON_DISABLED:
            return "SWITCH_ON_DISABLED";
        case State::READY_TO_SWITCH_ON:
            return "READY_TO_SWITCH_ON";
        case State::SWITCHED_ON:
            return "SWITCHED_ON";
        case State::OPERATION_ENABLE:
            return "OPERATION_ENABLE";
        case State::QUICK_STOP_ACTIVE:
            return "QUICK_STOP_ACTIVE";
        case State::FAULT_REACTION_ACTIVE:
            return "FAULT_REACTION_ACTIVE";
        case State::FAULT:
            return "FAULT";
        default:
            return "UNKNOWN";
    }
}

bool StateHandler::waitForNewState(const std::chrono::steady_clock::time_point& abstime, StateHandler::State& state)
{
    std::unique_lock lock(mutex_);
    while (state_ == state && cond_.wait_until(lock, abstime) == std::cv_status::no_timeout) {}
    bool res = state != state_;
    state = state_;
    return res;
}
