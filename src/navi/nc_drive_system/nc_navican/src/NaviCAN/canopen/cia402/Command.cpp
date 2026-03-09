#include "NaviCAN/canopen/cia402/Command.h"

#include "util/logger.hpp"

#include <bitset>
#include <sstream>
#include <stdexcept>

using namespace NaviFra;
using namespace NaviFra::NaviCAN::Canopen::CIA402;

const Command::TransitionTable Command::transitions_;

using ControlWord = NaviFra::NaviCAN::Object::Standard::ControlWord;

Command::TransitionTable::TransitionTable()
{
    typedef StateHandler::State s;

    transitions_.reserve(32);

    Op<ControlWord> disable_voltage(0, (1 << ControlWord::Bits::FAULT_RESET) | (1 << ControlWord::Bits::ENABLE_VOLTAGE));
    /* 7*/ add(s::READY_TO_SWITCH_ON, s::SWITCH_ON_DISABLED, disable_voltage);
    /* 9*/ add(s::OPERATION_ENABLE, s::SWITCH_ON_DISABLED, disable_voltage);
    /*10*/ add(s::SWITCHED_ON, s::SWITCH_ON_DISABLED, disable_voltage);
    /*12*/ add(s::QUICK_STOP_ACTIVE, s::SWITCH_ON_DISABLED, disable_voltage);

    Op<ControlWord> automatic(0, 0);
    /* 0*/ add(s::START, s::NOT_READY_TO_SWITCH_ON, automatic);
    /* 1*/ add(s::NOT_READY_TO_SWITCH_ON, s::SWITCH_ON_DISABLED, automatic);
    /*14*/ add(s::FAULT_REACTION_ACTIVE, s::FAULT, automatic);

    Op<ControlWord> shutdown(
        (1 << ControlWord::Bits::QUICK_STOP) | (1 << ControlWord::Bits::ENABLE_VOLTAGE),
        (1 << ControlWord::Bits::FAULT_RESET) | (1 << ControlWord::Bits::SWITCH_ON));
    /* 2*/ add(s::SWITCH_ON_DISABLED, s::READY_TO_SWITCH_ON, shutdown);
    /* 6*/ add(s::SWITCHED_ON, s::READY_TO_SWITCH_ON, shutdown);
    /* 8*/ add(s::OPERATION_ENABLE, s::READY_TO_SWITCH_ON, shutdown);

    Op<ControlWord> switch_on(
        (1 << ControlWord::Bits::QUICK_STOP) | (1 << ControlWord::Bits::ENABLE_VOLTAGE) | (1 << ControlWord::Bits::SWITCH_ON),
        (1 << ControlWord::Bits::FAULT_RESET) | (1 << ControlWord::Bits::ENABLE_OPERATION));
    /* 3*/ add(s::READY_TO_SWITCH_ON, s::SWITCHED_ON, switch_on);
    /* 5*/ add(s::OPERATION_ENABLE, s::SWITCHED_ON, switch_on);

    Op<ControlWord> enable_operation(
        (1 << ControlWord::Bits::QUICK_STOP) | (1 << ControlWord::Bits::ENABLE_VOLTAGE) | (1 << ControlWord::Bits::SWITCH_ON) |
            (1 << ControlWord::Bits::ENABLE_OPERATION),
        (1 << ControlWord::Bits::FAULT_RESET));
    /* 4*/ add(s::SWITCHED_ON, s::OPERATION_ENABLE, enable_operation);
    /*16*/ add(s::QUICK_STOP_ACTIVE, s::OPERATION_ENABLE, enable_operation);

    Op<ControlWord> quickstop(
        (1 << ControlWord::Bits::ENABLE_VOLTAGE), (1 << ControlWord::Bits::FAULT_RESET) | (1 << ControlWord::Bits::QUICK_STOP));
    /* 7*/ add(s::READY_TO_SWITCH_ON, s::QUICK_STOP_ACTIVE, quickstop);  // transit to SWITCH_ON_DISABLD
    /*10*/ add(s::SWITCHED_ON, s::QUICK_STOP_ACTIVE, quickstop);  // transit to SWITCH_ON_DISABLED
    /*11*/ add(s::OPERATION_ENABLE, s::QUICK_STOP_ACTIVE, quickstop);

    // fault reset
    Op<ControlWord> fault_reset((1 << ControlWord::Bits::FAULT_RESET), 0);
    /*15*/ add(s::FAULT, s::SWITCH_ON_DISABLED, fault_reset);
}

StateHandler::State Command::nextStateForDisabling(StateHandler::State state)
{
    switch (state) {
        case StateHandler::State::START:
            return StateHandler::State::NOT_READY_TO_SWITCH_ON;

        case StateHandler::State::FAULT:
        case StateHandler::State::NOT_READY_TO_SWITCH_ON:
            return StateHandler::State::SWITCH_ON_DISABLED;

        case StateHandler::State::OPERATION_ENABLE:
            return StateHandler::State::SWITCHED_ON;
        case StateHandler::State::SWITCHED_ON:
            return StateHandler::State::READY_TO_SWITCH_ON;
        case StateHandler::State::READY_TO_SWITCH_ON:
            return StateHandler::State::SWITCH_ON_DISABLED;
        case StateHandler::State::SWITCH_ON_DISABLED:
            return StateHandler::State::SWITCH_ON_DISABLED;

        case StateHandler::State::FAULT_REACTION_ACTIVE:
            return StateHandler::State::FAULT;
        default:
            throw std::out_of_range("state value is illegal");
            break;
    }
}
StateHandler::State Command::nextStateForEnabling(StateHandler::State state)
{
    switch (state) {
        case StateHandler::State::START:
            return StateHandler::State::NOT_READY_TO_SWITCH_ON;

        case StateHandler::State::FAULT:
        case StateHandler::State::NOT_READY_TO_SWITCH_ON:
            return StateHandler::State::SWITCH_ON_DISABLED;

        case StateHandler::State::SWITCH_ON_DISABLED:
            return StateHandler::State::READY_TO_SWITCH_ON;

        case StateHandler::State::READY_TO_SWITCH_ON:
            return StateHandler::State::SWITCHED_ON;

        case StateHandler::State::SWITCHED_ON:
        case StateHandler::State::QUICK_STOP_ACTIVE:
        case StateHandler::State::OPERATION_ENABLE:
            return StateHandler::State::OPERATION_ENABLE;

        case StateHandler::State::FAULT_REACTION_ACTIVE:
            return StateHandler::State::FAULT;
        default:
            throw std::out_of_range("state value is illegal");
            break;
    }
}

bool Command::setTransition(ControlWord& cw, const StateHandler::State& from, const StateHandler::State& to, StateHandler::State* next)
{
    try {
        if (from != to) {
            StateHandler::State hop = to;
            if (next) {
                if (to == StateHandler::State::OPERATION_ENABLE) {
                    hop = nextStateForEnabling(from);
                }

                if (to == StateHandler::State::SWITCH_ON_DISABLED) {
                    hop = nextStateForDisabling(from);
                }
                *next = hop;
            }
            transitions_.get(from, hop)(cw);
            NLOG(info) << "Transition from " << static_cast<int>(from) << " to " << static_cast<int>(to) << " cw= " << std::hex
                       << cw.getValue() << " next=" << static_cast<int>(hop);
        }
        return true;
    }
    catch (...) {
        // Handle error logging if needed
        NLOG(error) << "Error in setTransition: from " << static_cast<int>(from) << " to " << static_cast<int>(to);
    }
    return false;
}