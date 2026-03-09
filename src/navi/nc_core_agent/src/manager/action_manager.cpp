#include "core_agent/core_agent.h"

#include <core_agent/manager/action_manager.h>
#include <core_agent/message/message_broker.h>

using namespace NaviFra;

ActionManager::ActionManager()
{
}

ActionManager::~ActionManager()
{
}

void ActionManager::onAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        if (!obj->isNull("action")) {
            std::string action = obj->get("action").convert<std::string>();
            if (actions_.find(action) != actions_.end()) {
                actions_[action]->onAction(source, obj);
            }
            else {
                // MessageBroker::instance().publish(source, obj->get("uuid").extract<std::string>(), "response", "[ " + action + " ]
                // not_found");
                // sendResponseSuccess(source , obj->get("uuid").extract<std::string>(), "fail", "[ " + action + " ] not_found");
            }
        }
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
    }
}

void ActionManager::initialize(NaviFra::ActionType type)
{
    implInitialize(type);
}

void ActionManager::implInitialize(NaviFra::ActionType type)
{
    const auto& registeredActions = ActionRegistry::instance().getActions();
    auto it = registeredActions.find(type);
    if (it != registeredActions.end()) {
        for (const auto& actionPair : it->second) {
            actions_[actionPair.first] = actionPair.second();
        }
    }
}

void ActionManager::printActions()
{
    for (auto action : actions_) {
        LOG_INFO("Action : [ %s ],  Handler [ %s ]", action.first.c_str(), action.second->name().c_str());
    }
}