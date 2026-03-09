#include "nc_wia_agent/util/task_builder/Handler/undockingActionHandler.h"

#include "nc_wia_agent/util/task_builder/action_param.h"
namespace NaviFra {

UndockingActionHandler::UndockingActionHandler() = default;
UndockingActionHandler::~UndockingActionHandler() = default;

std::string UndockingActionHandler::getTypeName(Poco::JSON::Object::Ptr work) const
{
    return "undocking";
}

void UndockingActionHandler::process(Poco::JSON::Object::Ptr work, Poco::JSON::Object::Ptr& currentEntry, ActionData& action_data)
{
    try {
        auto params = UndockingParams::from(work);

        currentEntry->set("start_node_name", params.s_name);
        currentEntry->set("end_node_name", params.s_name);
        currentEntry->set("s_name", params.s_name);
        currentEntry->set("velocity", params.f_dockout_speed);
        currentEntry->set("docking_dist", params.f_docking_dist);
        currentEntry->set("drive_type", params.n_drive_type);
        currentEntry->set("list_f_target_obstacle", params.list_f_target_obstacle_margin);

        action_data.o_last_pos.SetXm(params.o_goal_pos.GetXm());
        action_data.o_last_pos.SetYm(params.o_goal_pos.GetYm());
        action_data.o_last_pos.SetDeg(params.o_goal_pos.GetDeg());
        action_data.f_last_curve_radius = 0.0f;
        action_data.last_drive_direction = params.n_drive_type;
        action_data.goal_info.push_back({action_data.o_last_pos, "undocking"});
    }
    catch (std::exception& e) {
        NLOG(error) << "error in standbyActionHandler: " << e.what();
    }
}
}  // namespace NaviFra