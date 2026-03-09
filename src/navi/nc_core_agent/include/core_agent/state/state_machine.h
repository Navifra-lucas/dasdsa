#ifndef NAVIFRA_STATE_MACHINE_H
#define NAVIFRA_STATE_MACHINE_H

#include "core/util/logger.hpp"

#include <functional>
#include <unordered_map>

namespace NaviFra {
using Action = std::function<void()>;
using Guard = std::function<bool()>;

struct Transition {
    Action action;
    Guard guard;
};

template <typename STATE>
class StateMachine {
public:
    StateMachine(STATE initialState)
        : currentState(initialState)
    {
    }

    virtual ~StateMachine() {}

public:
    void addTransition(
        std::vector<STATE> froms, STATE to, Action action, Guard guard = []() { return true; })
    {
        for (const auto& state : froms) {
            addTransition(state, to, action, guard);
        }
    }

    void addTransition(
        STATE from, STATE to, Action action, Guard guard = []() { return true; })
    {
        transitions[from][to] = {action, guard};
    }

    bool setState(STATE nextState)
    {
        if (transitions[currentState].find(nextState) != transitions[currentState].end()) {
            auto& transition = transitions[currentState][nextState];
            Action& action = transition.first;
            Guard& guard = transition.second;

            if (guard()) {
                action();
                currentState = nextState;
                return true;
            }
            else {
                LOG_DEBUG(
                    "Guard condition failed for transition from %s to ", stateToString(currentState).c_str(),
                    stateToString(nextState).c_str());
            }
        }
        else {
            LOG_DEBUG("Invalid transition from %s to  %s", stateToString(currentState).c_str(), stateToString(nextState).c_str());
        }

        return false;
    }

    STATE getState() const { return currentState; }

protected:
    STATE currentState;

    std::unordered_map<STATE, std::unordered_map<STATE, std::pair<Action, Guard>>> transitions;

protected:
    virtual std::string implStateToString(STATE state) const { return ""; }
    std::string stateToString(STATE state) const { return implStateToString(state); }
};
}  // namespace NaviFra
#endif