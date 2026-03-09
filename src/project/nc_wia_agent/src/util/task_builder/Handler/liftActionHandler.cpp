#include "nc_wia_agent/util/task_builder/Handler/liftActionHandler.h"

#include "nc_wia_agent/util/task_builder/action_param.h"

namespace NaviFra {

LiftActionHandler::LiftActionHandler() = default;
LiftActionHandler::~LiftActionHandler() = default;

std::string LiftActionHandler::getTypeName(Poco::JSON::Object::Ptr work) const
{
    Poco::JSON::Array::Ptr args = work->getArray("action_args");
    float lift_arg = args->getElement<float>(0);

    return (lift_arg > 0) ? "loading" : "unloading";
}

void LiftActionHandler::process(Poco::JSON::Object::Ptr work, Poco::JSON::Object::Ptr& currentEntry, ActionData& action_data)
{
    try {
        auto params = LiftParams::from(work);

        action_data.goal_info.push_back({action_data.o_last_pos, "lift"});
    }
    catch (std::exception& e) {
        NLOG(error) << "error in liftActionHandler: " << e.what();
    }
}
}  // namespace NaviFra