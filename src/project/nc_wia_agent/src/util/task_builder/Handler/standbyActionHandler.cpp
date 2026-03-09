#include "nc_wia_agent/util/task_builder/Handler/standbyActionHandler.h"

#include "nc_wia_agent/util/task_builder/action_param.h"

namespace NaviFra {

StandbyActionHandler::StandbyActionHandler() = default;
StandbyActionHandler::~StandbyActionHandler() = default;

std::string StandbyActionHandler::getTypeName(Poco::JSON::Object::Ptr work) const
{
    return "wait";
}

void StandbyActionHandler::process(Poco::JSON::Object::Ptr work, Poco::JSON::Object::Ptr& currentEntry, ActionData& action_data)
{
    try {
        auto params = StandbyParams::from(work);

        currentEntry->set("wait_seconds", params.n_standby_time);
        currentEntry->set("wait_id", params.s_standby_id);

        NLOG(info) << "Standby id -> " << params.s_standby_id << " / seconds -> " << params.n_standby_time;

        action_data.goal_info.push_back({action_data.o_last_pos, "standby"});
    }
    catch (std::exception& e) {
        NLOG(error) << "error in standbyActionHandler: " << e.what();
    }
}
}  // namespace NaviFra