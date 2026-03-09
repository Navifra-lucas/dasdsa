#include "nc_wia_agent/util/task_builder/Handler/spinActionHandler.h"

#include "nc_wia_agent/util/task_builder/action_param.h"

namespace NaviFra {

SpinActionHandler::SpinActionHandler() = default;
SpinActionHandler::~SpinActionHandler() = default;

std::string SpinActionHandler::getTypeName(Poco::JSON::Object::Ptr work) const
{
    return "spin";
}

void SpinActionHandler::process(Poco::JSON::Object::Ptr work, Poco::JSON::Object::Ptr& currentEntry, ActionData& action_data)
{
    try {
        Poco::JSON::Array::Ptr vec;
        if (currentEntry->has("vec_move_data")) {
            vec = currentEntry->getArray("vec_move_data");
        }
        else {
            vec = new Poco::JSON::Array;
            currentEntry->set("vec_move_data", vec);
        }
        auto params = SpinParams::from(work);

        Poco::JSON::Object::Ptr waypoint = new Poco::JSON::Object();
        waypoint->set("f_x_m", action_data.o_last_pos.GetXm());
        waypoint->set("f_y_m", action_data.o_last_pos.GetYm());
        waypoint->set("f_angle_deg", params.f_goal_deg);
        vec->add(waypoint);
        currentEntry->set("b_arrive_align", true);

        NLOG(info) << "Target Spin degree -> " << params.f_goal_deg;

        action_data.o_last_pos.SetDeg(params.f_goal_deg);

        action_data.goal_info.push_back({action_data.o_last_pos, "turn"});
    }
    catch (std::exception& e) {
        NLOG(error) << "error in spinActionHandler: " << e.what();
    }
}
}  // namespace NaviFra