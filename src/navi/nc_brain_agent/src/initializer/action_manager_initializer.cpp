#include "nc_brain_agent/initializer/action_manager_initializer.h"

#include "core_agent/manager/action_manager.h"
#include "core_agent/util/config.h"
#include "util/logger.hpp"

namespace NaviFra {

void ActionManagerInitializer::initialize()
{
    std::string agent_type = Config::instance().getString("agent_type", "base");

    // agent_type에 따라 ActionManager 초기화
    ActionType actionType = (agent_type == "volvo") ? ActionType::VOLVO : ActionType::DEFAULT;

    ActionManager::instance().initialize(actionType);
    ActionManager::instance().printActions();

    LOG_INFO("ActionManager initialized with agent_type: %s", agent_type.c_str());
}

}  // namespace NaviFra
