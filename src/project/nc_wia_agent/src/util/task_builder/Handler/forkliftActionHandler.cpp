#include "nc_wia_agent/util/task_builder/Handler/forkliftActionHandler.h"

#include "nc_wia_agent/util/task_builder/action_param.h"
namespace NaviFra {

ForkliftActionHandler::ForkliftActionHandler() = default;
ForkliftActionHandler::~ForkliftActionHandler() = default;

std::string ForkliftActionHandler::getTypeName(Poco::JSON::Object::Ptr work) const
{
    return "forklift";
}

void ForkliftActionHandler::process(Poco::JSON::Object::Ptr work, Poco::JSON::Object::Ptr& currentEntry, ActionData& action_data)
{
    try {
        auto params = ForkLiftParams::from(work);

        Poco::JSON::Array::Ptr vec = new Poco::JSON::Array();

        Poco::JSON::Object::Ptr obj = new Poco::JSON::Object();

        obj->set("current_node_id", params.s_current_node_id);
        obj->set("target_node_id", params.s_target_node_id);
        obj->set("current_pos", posToJson(params.o_current_pos));
        obj->set("target_pos", posToJson(params.o_target_pos));
        // obj->set("offset_pos", posToJson(params.o_offset_pos));
        obj->set("rack_level", params.n_rack_level);
        obj->set("target_level", params.n_target_level);
        obj->set("target_height", params.n_target_height);
        obj->set("drive_type", params.n_drive_type);
        obj->set("rack_type", params.n_rack_type);
        obj->set("pallet_type", params.n_pallet_type);

        vec->add(obj);

        currentEntry->set("fork_lift_data", vec);

        action_data.o_last_pos = params.o_target_pos;
        action_data.goal_info.push_back({params.o_target_pos, "forklift", params.n_rack_type, params.n_drive_type});
    }
    catch (std::exception& e) {
        NLOG(error) << "error in ForkliftActionHandler: " << e.what();
        throw;
    }
}
}  // namespace NaviFra