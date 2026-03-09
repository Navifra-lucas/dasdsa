#include "nc_wia_agent/util/task_builder/Handler/chargeActionHandler.h"

#include "nc_wia_agent/util/task_builder/action_param.h"

namespace NaviFra {

ChargeActionHandler::ChargeActionHandler() = default;
ChargeActionHandler::~ChargeActionHandler() = default;

std::string ChargeActionHandler::getTypeName(Poco::JSON::Object::Ptr work) const
{
    return "charging";
}

void ChargeActionHandler::process(Poco::JSON::Object::Ptr work, Poco::JSON::Object::Ptr& currentEntry, ActionData& action_data)
{
    try {
        action_data.goal_info.push_back({action_data.o_last_pos, "charging"});
    }
    catch (std::exception& e) {
        NLOG(error) << "error in ChargeActionHandler: " << e.what();
    }
}
}  // namespace NaviFra