#pragma once

#include "nc_wia_agent/util/task_builder/action_param.h"
#include "nc_wia_agent/util/task_builder/common_types.h"

#include <Poco/JSON/Object.h>

#include <iomanip>
#include <sstream>
#include <string>

namespace NaviFra {

class ActionHandler {
public:
    virtual ~ActionHandler();

    virtual void process(Poco::JSON::Object::Ptr work, Poco::JSON::Object::Ptr& currentEntry, ActionData& action_data) = 0;
    virtual std::string getTypeName(Poco::JSON::Object::Ptr work) const = 0;

protected:
    Poco::JSON::Object::Ptr posToJson(const Pos& pos)
    {
        Poco::JSON::Object::Ptr o = new Poco::JSON::Object;
        o->set("x", pos.GetXm());
        o->set("y", pos.GetYm());
        o->set("deg", pos.GetDeg());
        return o;
    }

    Poco::JSON::Object::Ptr createBasicWaypoint(MoveParams& params)
    {
        Poco::JSON::Object::Ptr waypoint = new Poco::JSON::Object();

        waypoint->set("s_name", params.s_name);
        waypoint->set("n_drive_type", params.n_drive_type);
        waypoint->set("f_speed_ms", params.f_max_trans_vel);
        waypoint->set("f_x_m", params.o_goal_pos.GetXm());
        waypoint->set("f_y_m", params.o_goal_pos.GetYm());
        waypoint->set("f_angle_deg", params.o_goal_pos.GetDeg());

        waypoint->set("f_lift_height", params.n_lift_height);
        waypoint->set("n_lidar_field", params.n_lidar_field);
        waypoint->set("n_camera_field", params.n_camera_field);
        waypoint->set("n_ossd_field", params.n_ossd_field);
        NLOG(info) << "Drive type -> " << params.n_drive_type << " / speed -> " << params.f_max_trans_vel;
        return waypoint;
    }
};
}  // namespace NaviFra