#include "nc_wia_agent/util/task_builder/Handler/moveActionHandler.h"

#include "nc_wia_agent/util/task_builder/action_param.h"
#define TESTMAX 100000

namespace NaviFra {

MoveActionHandler::MoveActionHandler() = default;
MoveActionHandler::~MoveActionHandler() = default;

std::string MoveActionHandler::getTypeName(Poco::JSON::Object::Ptr work) const
{
    return "move";
}

double MoveActionHandler::getYawFromOrientation(const Orientation& o)
{
    tf2::Quaternion quat(o.x, o.y, o.z, o.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;  // 라디안
}

// 곡선경로 진입점, 종료 지점 계산용 함수
// 시작 좌표, 끝 좌표를 기준으로 진행 방향을 계산, 곡선경로 시작지점을 도출
NaviFra::Pos MoveActionHandler::calcPosfromPassingDist(Pos o_start_pos, Pos o_end_pos, float f_passing_dist)
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

// 현재 위치 부터 링크까지의 거리 계산 함수
float MoveActionHandler::calculateDistanceToLine(const Pos& point, const Pos& line_start, const Pos& line_end)
{
    float px = point.GetXm();
    float py = point.GetYm();
    float x1 = line_start.GetXm();
    float y1 = line_start.GetYm();
    float x2 = line_end.GetXm();
    float y2 = line_end.GetYm();

    // 선분의 길이의 제곱지
    float line_len_sq = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    // 선분의 길이가 0이면 점까지의 거리 반환
    if (line_len_sq < 1e-6) {
        return std::hypot(px - x1, py - y1);
    }

    // 점을 선분에 투영했을 때의 매개변수 t (0 <= t <= 1이면 선분 위)
    float t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / line_len_sq;
    t = std::max(0.0f, std::min(1.0f, t));  // clamp to [0, 1]

    // 선분 위의 가장 가까운 점
    float closest_x = x1 + t * (x2 - x1);
    float closest_y = y1 + t * (y2 - y1);

    // 거리 계산
    return std::hypot(px - closest_x, py - closest_y);
}

Poco::JSON::Object::Ptr MoveActionHandler::addMoveData(MoveParams& action_params, Poco::JSON::Object::Ptr obj)
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

void MoveActionHandler::process(Poco::JSON::Object::Ptr work, Poco::JSON::Object::Ptr& currentEntry, ActionData& action_data)
{
    try {
        auto params = MoveParams::from(work);
        NaviFra::Pos o_represent_pos;
        bool b_has_represent_pos = false;

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
            }

            // ACS 명령 받을 시, 종류 별 내 동작 처리
            // Case 1. 무브미션이 노드 한개만 들어오는 경우.
            // 내 위치 찍고, 골 포즈 찍기

            // Case 2. 연속적인 무브 미션이 들어오는데, 내 위치가 링크(1->2)에서 먼 경우.
            // 내 위치 찍고, 골 포즈 찍기

            // Case 3/4. 연속적인 무브 미션이 들어오는데, 내 위치가 링크(1->2)에 가까운 경우.
            // 골 포즈만 찍기

            if (action_data.b_has_next_goal) {
                // 연속 move: 선분까지 거리만 체크
                float f_dist_to_line = calculateDistanceToLine(o_curr_pos, params.o_goal_pos, action_data.o_next_pos);
                NLOG(info) << "Consecutive MOVE. Distance to line (node1->node2): " << f_dist_to_line;

                if (f_dist_to_line > 0.2) {
                    // Case 2: 선분에서 멂 -> 내 위치를 먼저 추가
                    NLOG(info) << "Case 2: Robot is far from path. Add current position first.";
                    b_need_update = true;
                }
                else {
                    // Case 3/4: 선분에 가까움 -> 골 노드 그대로 추가
                    NLOG(info) << "Case 3/4: Robot is close to path. Add goal_pos only.";
                }
            }
            else {
                // 다음이 move가 아님 또는 다음 노드 정보 없음: 점 거리 체크
                NLOG(info) << "Next is NOT MOVE or no next goal info. Distance to node";
                b_need_update = true;
            }

            if (b_need_update) {
                auto tmp_params = params;
                tmp_params.o_goal_pos = action_data.o_last_pos;
                Poco::JSON::Object::Ptr startPosObj = createBasicWaypoint(tmp_params);
                startPosObj = addMoveData(tmp_params, startPosObj);
                vec->add(startPosObj);
            }
            else {
                params.o_goal_pos = o_curr_pos;
            }
        }
        else {
            Poco::JSON::Object::Ptr last_entry = vec->getObject(vec->size() - 1);
            last_entry->set("n_drive_type", params.n_drive_type);
        }

        float f_fnode_dist = 0;
        NLOG(info) << action_data.b_curve_pending << " / " << action_data.f_last_curve_radius << " / " << action_data.f_last_curve_speed;

        if (action_data.b_curve_pending && vec->size() > 0 && action_data.o_last_pos.GetXm() != TESTMAX) {
            float f_prev_deg = action_data.o_last_pos.GetDeg();
            float f_curr_deg = params.o_goal_pos.GetDeg();

            float f_diff = f_curr_deg - f_prev_deg;
            if (std::abs(f_diff) > 180.0f) {
                f_diff = (f_curr_deg - f_prev_deg) > 0 ? f_diff - 360.0f : f_diff + 360.0f;
            }

            NLOG(info) << f_diff;
            //임의로, 헤딩 각도 5도 이내면 직선이라고 판단. 스피드 변화 X
            if (fabs(f_diff) > 5.0f && fabs(f_diff) < 175.0f) {
                Poco::JSON::Object::Ptr last_entry = vec->getObject(vec->size() - 1);
                last_entry->set("f_speed_ms", action_data.f_last_curve_speed);
                last_entry->set("f_curve_radius", action_data.f_last_curve_radius);
                f_fnode_dist = std::hypot(
                    action_data.o_last_pos.GetXm() - params.o_goal_pos.GetXm(), action_data.o_last_pos.GetYm() - params.o_goal_pos.GetYm());

                if (action_data.f_last_curve_radius > 0.0f &&
                    f_fnode_dist > action_data.f_last_curve_radius + params.f_passing_dist + 0.1f) {
                    auto tmp_pos =
                        calcPosfromPassingDist(action_data.o_last_pos, params.o_goal_pos, f_fnode_dist - action_data.f_last_curve_radius);

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
                    tmpPosObj = addMoveData(tmp_params, tmpPosObj);
                    vec->add(tmpPosObj);
                }
            }
        }

        bool b_need_curve = false;
        if (params.f_passing_dist > 0.0f && params.f_curve_speed > 0.0f && action_data.o_last_pos.GetXm() != TESTMAX) {
            b_need_curve = true;
            if (f_fnode_dist != 0 && f_fnode_dist + 1e-3 <= params.f_passing_dist) {
                params.f_passing_dist = f_fnode_dist;
            }
            auto tmp_pos = calcPosfromPassingDist(action_data.o_last_pos, params.o_goal_pos, params.f_passing_dist);

            auto tmp_params = params;
            tmp_params.o_goal_pos = tmp_pos;

            o_represent_pos = tmp_pos;
            b_has_represent_pos = true;

            NLOG(info) << "Add goal / " << tmp_pos.GetXm() << " / " << tmp_pos.GetYm();

            Poco::JSON::Object::Ptr tmpPosObj = createBasicWaypoint(tmp_params);
            tmpPosObj = addMoveData(tmp_params, tmpPosObj);
            vec->add(tmpPosObj);
        }
        NLOG(info) << "Add goal / " << params.o_goal_pos.GetXm() << " / " << params.o_goal_pos.GetYm();

        Poco::JSON::Object::Ptr goalPosObj = createBasicWaypoint(params);
        goalPosObj = addMoveData(params, goalPosObj);
        vec->add(goalPosObj);

        if (b_has_represent_pos) {
            action_data.goal_info.push_back({o_represent_pos, "move"});
        }
        else {
            action_data.goal_info.push_back({params.o_goal_pos, "move"});
        }
        action_data.o_last_pos.SetXm(params.o_goal_pos.GetXm());
        action_data.o_last_pos.SetYm(params.o_goal_pos.GetYm());
        action_data.o_last_pos.SetDeg(params.o_goal_pos.GetDeg());
        action_data.f_last_curve_speed = (b_need_curve) ? params.f_curve_speed : 0;
        action_data.f_last_curve_radius = (b_need_curve) ? params.f_passing_dist : 0;
        action_data.b_curve_pending = (b_need_curve) ? true : false;
        action_data.last_drive_direction = params.n_drive_type;
    }
    catch (std::exception& e) {
        // NLOG(error) << "error in MoveActionHandler: " << e.what();
    }
}
}  // namespace NaviFra