#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <core_agent/data/robot_status.h>
#include <math.h>
#include <nc_brain_agent/action/nc_action_docking.h>
#include <nc_brain_agent/data/nc_brain_map.h>
#include <nc_brain_agent/nc_robot_agent.h>
#include <tf/tf.h>

using namespace NaviFra;

NcActionDocking::NcActionDocking()
{
}

NcActionDocking::~NcActionDocking()
{
}

void NcActionDocking::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();

    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    if (robotInfo->getStatus() != "idle") {
        LOG_ERROR("robot status is not idle. can not action on %s", action.c_str());
        sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail");
        // sendResponseSuccess(source , obj->get("uuid").convert<std::string>(), "fail", "The robot cannot perform the task in running
        // status.");
        return;
    }
    NLOG(info) << "implonAction : " << action;
    node_action();
    sendResponseSuccess(source, obj->get("uuid").convert<std::string>());


    // Poco::JSON::Object::Ptr data = obj->getObject("data");

    // bool b_get_goal_position = false;
    // bool b_get_start_position = false;
    // bool b_get_link_drive_data = false;

    // std::string strNodeId = data->has("node_id") ? data->get("node_id").convert<std::string>() : "";
    // if (strNodeId.compare("") == 0) {
    //     LOG_ERROR("request data is not received on %s", action.c_str());
    //     sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail");
    // }
    // else {
    //     if (!InMemoryRepository::instance()
    //              .get<RobotStatus>(RobotStatus::KEY)
    //              ->isJobActive()) {  // manual_state idle 확인에서 robot status가 jobactive가 아닐때로 변경
    //         std::string goal_node_id = strNodeId;
    //         float goal_node_x = 0;
    //         float goal_node_y = 0;
    //         float goal_node_deg = 0;

    //         b_get_goal_position = InMemoryRepository::instance()
    //                                   .get<NcBrainMap>(NcBrainMap::KEY)
    //                                   ->getNodePositionByNodeid(goal_node_id, goal_node_x, goal_node_y, goal_node_deg);

    //         if (!b_get_goal_position) {
    //             sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail");
    //             return;
    //         }

    //         std::string start_node_id = "";
    //         float start_node_x = 0;
    //         float start_node_y = 0;
    //         float start_node_deg = 0;

    //         b_get_start_position = InMemoryRepository::instance()
    //                                    .get<NcBrainMap>(NcBrainMap::KEY)
    //                                    ->getNodePositionNearbyRobot(start_node_id, start_node_x, start_node_y, start_node_deg);

    //         if (!b_get_start_position) {
    //             sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail");
    //             return;
    //         }

    //         float linear_speed = 0;
    //         int drive_type = 0;

    //         b_get_link_drive_data = InMemoryRepository::instance()
    //                                     .get<NcBrainMap>(NcBrainMap::KEY)
    //                                     ->getLinkDriveData(start_node_id, goal_node_id, linear_speed, drive_type);

    //         if (!b_get_link_drive_data) {
    //             sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail");
    //             return;
    //         }

    //         DockingNode start, end;
    //         DockingType type;

    //         start.f_x = start_node_x;
    //         start.f_y = start_node_y;
    //         start.f_angle_deg = start_node_deg;
    //         start.f_linear_speed = linear_speed;
    //         start.n_drive_type = drive_type;

    //         end.f_x = goal_node_x;
    //         end.f_y = goal_node_y;
    //         end.f_angle_deg = goal_node_deg;

    //         type.b_start_pause = false;
    //         type.b_flag = false;
    //         type.b_in = true;
    //         type.b_state = true;

    //         docking(start, end, type);
    //         sendResponseSuccess(source, obj->get("uuid").convert<std::string>());
    //     }
    //     else
    //         sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail", "fail! job is active!");
    // }
}

std::string NcActionDocking::implName()
{
    return "NcActionDocking";
}
