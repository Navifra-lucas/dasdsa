#ifndef NAVIFRA_NAVICAN_CANOPEN_CIA402_COMMAND_H
#define NAVIFRA_NAVICAN_CANOPEN_CIA402_COMMAND_H

#include "NaviCAN/Object/standard/ControlWord.hpp"
#include "NaviCAN/canopen/cia402/StateHandler.h"

#include <boost/container/flat_map.hpp>

#include <cstdint>
#include <type_traits>
#include <utility>

namespace NaviFra {
namespace NaviCAN {
namespace Canopen {
namespace CIA402 {

class Command {
private:
    using ControlWord = NaviFra::NaviCAN::Object::Standard::ControlWord;

    template <typename ControlWordType>
    struct Op {
    private:
        using value_type = typename std::decay_t<decltype(std::declval<ControlWordType>().getValue())>;

        value_type to_set_;
        value_type to_reset_;

    public:
        Op(value_type to_set, value_type to_reset)
            : to_set_(to_set)
            , to_reset_(to_reset)
        {
        }

        void operator()(ControlWordType& cw) const
        {
            cw.update([this](value_type current) { return (current & ~to_reset_) | to_set_; });
        }
    };

    class TransitionTable {
    private:
        boost::container::flat_map<std::pair<StateHandler::State, StateHandler::State>, Op<ControlWord>> transitions_;

        void add(const StateHandler::State& from, const StateHandler::State& to, const Op<ControlWord>& op)
        {
            transitions_.insert(std::make_pair(std::make_pair(from, to), op));
        }

    public:
        TransitionTable();
        const Op<ControlWord>& get(const StateHandler::State& from, const StateHandler::State& to) const
        {
            return transitions_.at(std::make_pair(from, to));
        }
    };

    static const TransitionTable transitions_;
    static StateHandler::State nextStateForEnabling(StateHandler::State state);
    static StateHandler::State nextStateForDisabling(StateHandler::State state);

public:
    Command();

    static bool setTransition(ControlWord& cw, const StateHandler::State& from, const StateHandler::State& to, StateHandler::State* next);
};

}  // namespace CIA402
}  // namespace Canopen
}  // namespace NaviCAN
}  // namespace NaviFra

#endif  // NAVIFRA_MOTOR_COMMAND_H
