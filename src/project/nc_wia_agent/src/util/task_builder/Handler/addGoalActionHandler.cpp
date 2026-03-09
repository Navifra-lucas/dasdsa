#include "nc_wia_agent/util/task_builder/Handler/addGoalActionHandler.h"

#include "nc_wia_agent/util/task_builder/action_param.h"
#define TESTMAX 100000

namespace NaviFra {

AddGoalActionHandler::AddGoalActionHandler() = default;
AddGoalActionHandler::~AddGoalActionHandler() = default;

std::string AddGoalActionHandler::getTypeName(Poco::JSON::Object::Ptr work) const
{
    return "move";
}

double AddGoalActionHandler::getYawFromOrientation(const Orientation& o)
{
    tf2::Quaternion quat(o.x, o.y, o.z, o.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;  // 라디안
}

// 곡선경로 진입점, 종료 지점 계산용 함수
// 시작 좌표, 끝 좌표를 기준으로 진행 방향을 계산, 곡선경로 시작지점을 도출
NaviFra::Pos AddGoalActionHandler::calcPosfromPassingDist(Pos o_start_pos, Pos o_end_pos, float f_passing_dist)
{
    float f_angle_rad = std::atan2(o_end_pos.GetYm() - o_start_pos.GetYm(), o_end_pos.GetXm() - o_start_pos.GetXm());

    float new_x, new_y;

    new_x = o_end_pos.GetXm() - f_passing_dist * std::cos(f_angle_rad);
    new_y = o_end_pos.GetYm() - f_passing_dist * std::sin(f_angle_rad);

    Pos o_tmp_pos;
    o_tmp_pos.SetXm(new_x);
    o_tmp_pos.SetYm(new_y);
    o_tmp_pos.SetDeg(f_passing_dist < 0 ? o_start_pos.GetDeg() : o_end_pos.GetDeg());
    NLOG(info) << "Make new Pos. X : " << o_tmp_pos.GetXm() << ", Y : " << o_tmp_pos.GetYm() << ", Deg : " << o_tmp_pos.GetDeg();

    return o_tmp_pos;
}

Poco::JSON::Object::Ptr AddGoalActionHandler::addMoveData(MoveParams& action_params, Poco::JSON::Object::Ptr obj)
{
    float half_length = action_params.f_target_lenth / 2.0f;
    float half_width = action_params.f_target_width / 2.0f;
    auto vec_lccs_margin = action_params.vec_lccs_field;

    Poco::JSON::Array::Ptr margins = new Poco::JSON::Array;
    for (const float& margin : action_params.vec_lccs_field) {
        margins->add(margin);
    }
    // JSON에 추가
    obj->set("list_f_obstacle_margin", margins);
    obj->set("b_lccs_off", action_params.b_lccs_off);
    obj->set("b_arrive_align", true);
    return obj;
}

void AddGoalActionHandler::process(Poco::JSON::Object::Ptr work, Poco::JSON::Object::Ptr& currentEntry, ActionData& action_data)
{
    try {
        auto params = MoveParams::from(work);

        // vec_move 초기화
        Poco::JSON::Array::Ptr vec;
        if (currentEntry->has("vec_move_data")) {
            vec = currentEntry->getArray("vec_move_data");
        }
        else {
            vec = new Poco::JSON::Array;
            currentEntry->set("vec_move_data", vec);
        }
        currentEntry->set("uuid", params.node_id);

        bool b_initial_task = false;

        // 이전 액션이 무브가 아니었다면, vec size 는 0
        if (vec->size() == 0) {
            auto robotPose = InMemoryRepository::instance().get<RobotPose>(RobotPose::KEY);
            Pos o_curr_pos;
            o_curr_pos.SetXm(robotPose->getPosition().x);
            o_curr_pos.SetYm(robotPose->getPosition().y);
            o_curr_pos.SetDeg(getYawFromOrientation(robotPose->getOrientation()) * 180.0 / M_PI);

            bool b_need_update = false;
            // last_pos가 초기값이면 현재 위치로 설정
            if (action_data.o_last_pos.GetXm() == TESTMAX) {
                action_data.o_last_pos = o_curr_pos;
                action_data.last_drive_direction = params.n_drive_type;
                b_need_update = true;
                b_initial_task = true;
            }
            else {
                float dist =
                    std::hypot(action_data.o_last_pos.GetXm() - o_curr_pos.GetXm(), action_data.o_last_pos.GetYm() - o_curr_pos.GetYm());
                if (dist > 0.2) {
                    b_need_update = true;
                }
            }

            if (b_need_update) {
                auto tmp_params = params;
                tmp_params.o_goal_pos = action_data.o_last_pos;
                Poco::JSON::Object::Ptr startPosObj = createBasicWaypoint(tmp_params);
                vec->add(startPosObj);
            }
        }
        else if (vec->size() > 0) {
            Poco::JSON::Object::Ptr last_entry = vec->getObject(vec->size() - 1);
            last_entry->set("n_drive_type", params.n_drive_type);
        }

        float f_fnode_dist = std::hypot(
            action_data.o_last_pos.GetXm() - params.o_goal_pos.GetXm(), action_data.o_last_pos.GetYm() - params.o_goal_pos.GetYm());

        if (params.f_passing_dist > 0.0f && params.f_curve_speed > 0.0f) {
            if (f_fnode_dist + 1e-3 <= params.f_passing_dist) {
                //이전 노드의 주행 속도, curve_radius 교체
                if (vec->size() > 0) {
                    Poco::JSON::Object::Ptr last_entry = vec->getObject(vec->size() - 1);
                    last_entry->set("f_curve_radius", params.f_passing_dist);
                    last_entry->set("f_max_trans_vel", params.f_curve_speed);
                }
            }
            else {
                auto tmp_pos = calcPosfromPassingDist(action_data.o_last_pos, params.o_goal_pos, params.f_passing_dist);

                params.f_max_trans_vel = params.f_curve_speed;
                auto tmp_params = params;
                tmp_params.o_goal_pos = tmp_pos;

                try {
                    int id = std::stoi(params.action_id);
                    int prev_id = std::max(0, id - 1);
                    tmp_params.action_id = std::to_string(prev_id);

                    std::ostringstream ss;
                    ss << std::setw(4) << std::setfill('0') << prev_id;
                    tmp_params.s_name = ss.str();
                }
                catch (...) {
                    NLOG(info) << "fail to convert action_id to int";
                }

                NLOG(info) << "Add goal / " << tmp_pos.GetXm() << " / " << tmp_pos.GetYm();

                Poco::JSON::Object::Ptr tmpPosObj = createBasicWaypoint(tmp_params);
                tmpPosObj = addMoveData(params, tmpPosObj);
                vec->add(tmpPosObj);

                action_data.goal_info.push_back({tmp_pos, "move"});
            }
        }
        NLOG(info) << "Add goal / " << params.o_goal_pos.GetXm() << " / " << params.o_goal_pos.GetYm();

        Poco::JSON::Object::Ptr goalPosObj = createBasicWaypoint(params);
        goalPosObj = addMoveData(params, goalPosObj);
        vec->add(goalPosObj);

        action_data.goal_info.push_back({params.o_goal_pos, "move"});
        action_data.o_last_pos.SetXm(params.o_goal_pos.GetXm());
        action_data.o_last_pos.SetYm(params.o_goal_pos.GetYm());
        action_data.o_last_pos.SetDeg(params.o_goal_pos.GetDeg());
        action_data.last_drive_direction = params.n_drive_type;
    }
    catch (std::exception& e) {
        // NLOG(error) << "error in AddGoalActionHandler: " << e.what();
    }
}
}  // namespace NaviFra