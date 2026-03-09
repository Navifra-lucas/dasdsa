#include "nc_wia_agent/util/task_builder/Handler/dockingActionHandler.h"

#include "nc_wia_agent/util/task_builder/action_param.h"
namespace NaviFra {

DockingActionHandler::DockingActionHandler() = default;
DockingActionHandler::~DockingActionHandler() = default;

std::string DockingActionHandler::getTypeName(Poco::JSON::Object::Ptr work) const
{
    return "docking";
}

void DockingActionHandler::process(Poco::JSON::Object::Ptr work, Poco::JSON::Object::Ptr& currentEntry, ActionData& action_data)
{
    try {
        auto params = DockingParams::from(work);

        static const std::unordered_map<int, std::string> docktype_map = {
            {6, "aruco"},
            {7, "slam"},
            {8, "aruco charge"},
        };
        std::string s_dock_type;
        if (params.n_dock_type == 9) {
            s_dock_type = docktype_map.count(params.n_dock_type) ? docktype_map.at(params.n_dock_type) : "error";
        }

        NLOG(info) << "Docking Pos x: " << params.o_goal_pos.GetXm() << ", y: " << params.o_goal_pos.GetYm()
                   << ", deg: " << params.o_goal_pos.GetDeg() << ", dock_dist: " << params.f_docking_dist;

        if (s_dock_type == "slam") {
            NLOG(info) << "SLAM docking.";

            Poco::JSON::Array::Ptr vec;
            if (currentEntry->has("vec_move_data")) {
                vec = currentEntry->getArray("vec_move_data");
            }
            else {
                vec = new Poco::JSON::Array;
                currentEntry->set("vec_move_data", vec);
            }

            int n_drive_type = (params.s_direction == "front") ? MOVE_FORWARD : MOVE_BACKWARD;

            MoveParams tmp_params;
            tmp_params.s_name = params.s_name;
            tmp_params.n_drive_type = params.n_drive_type;
            tmp_params.f_max_trans_vel = 0.5;
            tmp_params.o_goal_pos = params.o_goal_pos;

            Poco::JSON::Object::Ptr moveObj = createBasicWaypoint(tmp_params);
            vec->add(moveObj);

            action_data.o_last_pos.SetXm(params.o_goal_pos.GetXm());
            action_data.o_last_pos.SetYm(params.o_goal_pos.GetYm());
            action_data.o_last_pos.SetDeg(params.o_goal_pos.GetDeg());
            action_data.goal_info.push_back({params.o_goal_pos, "slam_docking"});

            return;
        }

        Poco::JSON::Object::Ptr dataObj = new Poco::JSON::Object();

        // 시작 좌표 (도킹 시작점 = 이전 move 좌표)
        Poco::JSON::Object::Ptr startObj = new Poco::JSON::Object();
        startObj->set("x", action_data.o_last_pos.GetXm());
        startObj->set("y", action_data.o_last_pos.GetYm());
        startObj->set("deg", action_data.o_last_pos.GetDeg());

        // 끝 좌표 (도킹 완료 지점)
        Poco::JSON::Object::Ptr endObj = new Poco::JSON::Object();

        float f_goal_rad = params.o_goal_pos.GetDeg() * M_PI / 180.0;
        float f_docking_end_x = params.o_goal_pos.GetXm() - params.f_docking_dist * cos(f_goal_rad);
        float f_docking_end_y = params.o_goal_pos.GetYm() - params.f_docking_dist * sin(f_goal_rad);
        params.o_goal_pos.SetXm(f_docking_end_x);
        params.o_goal_pos.SetYm(f_docking_end_y);

        endObj->set("x", params.o_goal_pos.GetXm());
        endObj->set("y", params.o_goal_pos.GetYm());
        endObj->set("deg", params.o_goal_pos.GetDeg());

        dataObj->set("start", startObj);
        dataObj->set("end", endObj);
        currentEntry->set("data", dataObj);

        currentEntry->set("start_node_name", params.s_name);
        currentEntry->set("end_node_name", params.s_name);
        currentEntry->set("direction", params.s_direction);
        currentEntry->set("docking_type", s_dock_type);
        currentEntry->set("pallet_exist", params.n_pallet_exist);
        currentEntry->set("goal_offset_x", params.o_goal_offset.GetXm());
        currentEntry->set("goal_offset_y", params.o_goal_offset.GetYm());
        currentEntry->set("goal_offset_deg", params.o_goal_offset.GetDeg());
        currentEntry->set("list_f_camera_roi", params.list_f_camera_roi);
        currentEntry->set("list_f_camera_roi2", params.list_f_camera_roi2);
        currentEntry->set("lift_f_move_obstacle", params.list_f_move_obstacle);

        if (params.n_dock_type == 6) {
            action_data.goal_info.push_back({params.o_goal_pos, "docking"});
        }
        else if (params.n_dock_type == 8) {
            action_data.goal_info.push_back({params.o_goal_pos, "charge_docking"});
        }
    }
    catch (std::exception& e) {
        NLOG(error) << "error in dockingActionHandler: " << e.what();
    }
}
}  // namespace NaviFra