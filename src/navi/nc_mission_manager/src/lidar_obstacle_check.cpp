#include "lidar_obstacle_check.hpp"

#include "core/util/logger.hpp"

using namespace std;

namespace NaviFra {

int LidarObstacle::CheckPointInPolygon(
    const std::vector<NaviFra::SimplePos>& vec_sensors_relative_robot, NaviFra::Polygon& o_polygon, bool b_arrive_check,
    bool b_docking_path_obs_check, bool b_start_obs_check)
{
    int n_detect_check = 0;
    int n_sensor_size = vec_sensors_relative_robot.size();
    static bool b_pre_detect = false;
    vector<NaviFra::Polygon> vec_target_obstacle_polygon = GetCheckPolygon(CheckPolygon::TARGET);
    vector<NaviFra::Polygon> vec_polygon_docking_path_check = GetCheckPolygon(CheckPolygon::MOVE);
    vector<NaviFra::Polygon> vec_polygon_start_check = GetCheckPolygon(CheckPolygon::START);
    for (int i = 0; i < n_sensor_size; i++) {
        if(vec_sensors_relative_robot.at(i).GetZm() == 4) {
            continue;
        }
        // 체크할 라이다 포인트
        NaviFra::Pos o_checking_pos(vec_sensors_relative_robot.at(i).GetXm(), vec_sensors_relative_robot.at(i).GetYm());
        // 해당 지점이 로봇에 충돌한 경우
        if (o_polygon.GetPointInPolygon(o_checking_pos)) {
            o_obs_pos_ = o_checking_pos;
            n_detect_check = 1;
            if (!b_pre_detect)
                LOG_INFO("[monitoring] OBSTACLE_COLLISION_DETECTION : %.3f", vec_sensors_relative_robot.at(i).GetZm());
            if (vec_sensors_relative_robot.at(i).GetZm() == 1)
                n_detect_check = 2;
            else if (vec_sensors_relative_robot.at(i).GetZm() == 2)
                n_detect_check = 3;
            break;
        }
        if (b_arrive_check) {
            for (int j = 0; j < vec_target_obstacle_polygon.size(); j++) {
                if (vec_target_obstacle_polygon.at(j).GetPointInPolygon(o_checking_pos)) {
                    o_obs_pos_ = o_checking_pos;
                    n_detect_check = 1;
                    if (!b_pre_detect)
                        LOG_INFO("[monitoring] TARGET %d_COLLISION_DETECTION : %.3f", j, vec_sensors_relative_robot.at(i).GetZm());
                    if (vec_sensors_relative_robot.at(i).GetZm() == 1)
                        n_detect_check = 2;
                    else if (vec_sensors_relative_robot.at(i).GetZm() == 2)
                        n_detect_check = 3;
                    break;
                }
            }
            if (n_detect_check != 0)
                break;
        }

        if (b_docking_path_obs_check) {
            for (int j = 0; j < vec_polygon_docking_path_check.size(); j++) {
                if (vec_polygon_docking_path_check.at(j).GetPointInPolygon(o_checking_pos)) {
                    o_obs_pos_ = o_checking_pos;
                    n_detect_check = 1;
                    if (!b_pre_detect)
                        LOG_INFO("[monitoring] MOVE_COLLISION_DETECTION : %.3f", vec_sensors_relative_robot.at(i).GetZm());
                    if (vec_sensors_relative_robot.at(i).GetZm() == 1)
                        n_detect_check = 2;
                    else if (vec_sensors_relative_robot.at(i).GetZm() == 2)
                        n_detect_check = 3;
                    break;
                }
            }
            if (n_detect_check != 0)
                break;
        }
        if (b_start_obs_check) {
            for (int j = 0; j < vec_polygon_start_check.size(); j++) {
                if (vec_polygon_start_check.at(j).GetPointInPolygon(o_checking_pos)) {
                    o_obs_pos_ = o_checking_pos;
                    n_detect_check = 1;
                    if (!b_pre_detect)
                        LOG_INFO("[monitoring] START_COLLISION_DETECTION : %.3f", vec_sensors_relative_robot.at(i).GetZm());
                    if (vec_sensors_relative_robot.at(i).GetZm() == 1)
                        n_detect_check = 2;
                    else if (vec_sensors_relative_robot.at(i).GetZm() == 2)
                        n_detect_check = 3;
                    break;
                }
            }
            if (n_detect_check != 0)
                break;
        }
    }
    if (n_detect_check != 0)
        b_pre_detect = true;
    else
        b_pre_detect = false;
    return n_detect_check;
}

void LidarObstacle::TargetObsPolyMaker(
    const Pos& o_start_pos, const Pos& o_end_pos, const Pos& o_robot_pos, const vector<Pos>& vec_pos_local_path,
    const Pos::DriveInfo_t& o_drive_info, const Pos::DriveInfo_t& o_goal_drive_info)
{
    std::vector<NaviFra::Polygon> vec_target_obstacle_polygon;
    Pos::DriveInfo_t o_drive_info_local = o_drive_info;
    Pos o_end_pos_local = o_end_pos;
    if (o_drive_info_local.f_docking_check_dist != 0.0)
        o_end_pos_local = vec_pos_local_path.at(vec_pos_local_path.size() - 1 - o_drive_info_local.f_docking_check_dist);
    NaviFra::Polygon o_arrive_polygon;
    NaviFra::Polygon o_arrive_polygon_local;

    o_arrive_polygon.AddVertex(Pos(-0.001, -0.001, 0));
    o_arrive_polygon.AddVertex(Pos(0.001, -0.001, 0));
    o_arrive_polygon.AddVertex(Pos(0.001, 0.001, 0));
    o_arrive_polygon.AddVertex(Pos(-0.001, 0.001, 0));

    o_arrive_polygon.UpdateVertexSizePlusFLRR(
        o_goal_drive_info.f_target_obstacle_margin_front, o_goal_drive_info.f_target_obstacle_margin_rear,
        o_goal_drive_info.f_target_obstacle_margin_left, o_goal_drive_info.f_target_obstacle_margin_right);
    float f_global_angle = atan2(o_end_pos_local.GetYm() - o_start_pos.GetYm(), o_end_pos_local.GetXm() - o_start_pos.GetXm());
    if (o_drive_info_local.f_docking_check_dist != 0.0)
        f_global_angle = atan2(o_end_pos_local.GetYm() - o_start_pos.GetYm(), o_end_pos_local.GetXm() - o_start_pos.GetXm()) + M_PI;

    Pos o_gloabl_gap = Pos(o_end_pos_local.GetXm() - o_robot_pos.GetXm(), o_end_pos_local.GetYm() - o_robot_pos.GetYm(), 0);
    Pos o_local_gap = CoreCalculator::TransformRotationDeg_(o_gloabl_gap, -o_robot_pos.GetDeg());
    for (int i = 0; i < o_arrive_polygon.o_polygon_vertexs_.size(); i++) {
        o_arrive_polygon_local.AddVertex(CoreCalculator::TransformRotationDeg_(
            o_arrive_polygon.o_polygon_vertexs_.at(i), f_global_angle * RADtoDEG - o_robot_pos.GetDeg()));
        o_arrive_polygon_local.o_polygon_vertexs_.at(i) = NaviFra::Pos(
            o_arrive_polygon_local.o_polygon_vertexs_.at(i).GetXm() + o_local_gap.GetXm(),
            o_arrive_polygon_local.o_polygon_vertexs_.at(i).GetYm() + o_local_gap.GetYm(), 0);
    }
    vec_target_obstacle_polygon.emplace_back(o_arrive_polygon_local);
    SetCheckPolygon(CheckPolygon::TARGET, vec_target_obstacle_polygon);
}

void LidarObstacle::MoveObsPolyMaker(
    const Pos& o_start_pos, const Pos& o_end_pos, const Pos& o_robot_pos, const Pos::DriveInfo_t& o_goal_drive_info,
    const Parameters_t& st_param, NaviFra::Polygon& o_polygon, const vector<Pos>& vec_pos_local_path)
{
    // 체크영역
    std::vector<NaviFra::Polygon> vec_save_pose;
    std::vector<NaviFra::Polygon> vec_polygon_docking_path_check(4);
    int n_robot_path_idx = CoreCalculator::FindMinDistanceIdxFromPosVector_(o_robot_pos, vec_pos_local_path);
    float f_remain_path_dist = (vec_pos_local_path.size() - n_robot_path_idx) / 100.0;
    // 동적인 장애물 체크 폴리곤 만들기
    NaviFra::Polygon o_arrive_move_polygon;
    // NaviFra::Polygon o_arrive_move_polygon_local;

    o_arrive_move_polygon.AddVertex(Pos(-0.001, -0.001, 0));
    o_arrive_move_polygon.AddVertex(Pos(0.001, -0.001, 0));
    o_arrive_move_polygon.AddVertex(Pos(0.001, 0.001, 0));
    o_arrive_move_polygon.AddVertex(Pos(-0.001, 0.001, 0));
    o_arrive_move_polygon.UpdateVertexSizePlusFLRR(
        o_goal_drive_info.f_move_obstacle_margin_front, o_goal_drive_info.f_move_obstacle_margin_rear,
        o_goal_drive_info.f_move_obstacle_margin_left, o_goal_drive_info.f_move_obstacle_margin_right);
    float f_global_angle = atan2(o_end_pos.GetYm() - o_start_pos.GetYm(), o_end_pos.GetXm() - o_start_pos.GetXm());

    //기존 Move Obstacle Check Polygon Maker
    // Pos o_gloabl_gap = Pos(o_end_pos.GetXm() - o_robot_pos.GetXm(), o_end_pos.GetYm() - o_robot_pos.GetYm(), 0);
    // Pos o_local_gap = CoreCalculator::TransformRotationDeg_(o_gloabl_gap, -o_robot_pos.GetDeg());
    // float f_dist = CoreCalculator::CalcPosDistance_(o_local_gap);
    // if (f_dist > st_param.f_move_obstacle_dist_max && st_param.f_move_obstacle_dist_max > 0) {
    //     o_local_gap.SetXm(o_local_gap.GetXm() * st_param.f_move_obstacle_dist_max / f_dist);
    //     o_local_gap.SetYm(o_local_gap.GetYm() * st_param.f_move_obstacle_dist_max / f_dist);
    // }

    // // cout<<"local "<<o_local_gap.GetXm()<<" / "<<o_local_gap.GetYm()<<endl;
    // for (int i = 0; i < o_arrive_move_polygon.o_polygon_vertexs_.size(); i++) {
    //     o_arrive_move_polygon_local.AddVertex(CoreCalculator::TransformRotationDeg_(
    //         o_arrive_move_polygon.o_polygon_vertexs_.at(i), f_global_angle * RADtoDEG - o_robot_pos.GetDeg()));
    //     o_arrive_move_polygon_local.o_polygon_vertexs_.at(i) = NaviFra::Pos(
    //         o_arrive_move_polygon_local.o_polygon_vertexs_.at(i).GetXm() + o_local_gap.GetXm(),
    //         o_arrive_move_polygon_local.o_polygon_vertexs_.at(i).GetYm() + o_local_gap.GetYm(), 0);
    // }
    // vec_save_pose.emplace_back(o_polygon);
    // vec_save_pose.emplace_back(o_arrive_move_polygon_local);

    // for (int j = 0; j < vec_save_pose.size(); j++) {
    //     vec_polygon_docking_path_check.at(0).AddVertex(vec_save_pose.at(j).o_polygon_vertexs_.at(1));
    //     vec_polygon_docking_path_check.at(1).AddVertex(vec_save_pose.at(j).o_polygon_vertexs_.at(2));
    //     vec_polygon_docking_path_check.at(2).AddVertex(vec_save_pose.at(j).o_polygon_vertexs_.at(3));
    //     vec_polygon_docking_path_check.at(3).AddVertex(vec_save_pose.at(j).o_polygon_vertexs_.at(0));
    //     if (j == vec_save_pose.size() - 1) {
    //         for (int k = vec_save_pose.size() - 1; k >= 0; k--) {
    //             vec_polygon_docking_path_check.at(0).AddVertex(vec_save_pose.at(k).o_polygon_vertexs_.at(2));
    //             vec_polygon_docking_path_check.at(1).AddVertex(vec_save_pose.at(k).o_polygon_vertexs_.at(3));
    //             vec_polygon_docking_path_check.at(2).AddVertex(vec_save_pose.at(k).o_polygon_vertexs_.at(0));
    //             vec_polygon_docking_path_check.at(3).AddVertex(vec_save_pose.at(k).o_polygon_vertexs_.at(1));
    //         }
    //     }
    // }

    //신규 Move Obstacle Check Polygon Maker
    for (int j = (vec_pos_local_path.size()-1); j >= n_robot_path_idx; j-=20) {
        NaviFra::Polygon o_arrive_move_polygon_local;
        Pos o_gloabl_gap = Pos(vec_pos_local_path.at(j).GetXm() - o_robot_pos.GetXm(), vec_pos_local_path.at(j).GetYm() - o_robot_pos.GetYm(), 0);
        Pos o_local_gap = CoreCalculator::TransformRotationDeg_(o_gloabl_gap, -o_robot_pos.GetDeg());
        float f_dist = CoreCalculator::CalcPosDistance_(o_local_gap);
        if (f_dist > st_param.f_move_obstacle_dist_max && st_param.f_move_obstacle_dist_max > 0) {
            continue;
        }
        for (int i = 0; i < o_arrive_move_polygon.o_polygon_vertexs_.size(); i++) {
            o_arrive_move_polygon_local.AddVertex(CoreCalculator::TransformRotationDeg_(
                o_arrive_move_polygon.o_polygon_vertexs_.at(i), vec_pos_local_path.at(j).GetDeg() - o_robot_pos.GetDeg()));
            o_arrive_move_polygon_local.o_polygon_vertexs_.at(i) = NaviFra::Pos(
                o_arrive_move_polygon_local.o_polygon_vertexs_.at(i).GetXm() + o_local_gap.GetXm(),
                o_arrive_move_polygon_local.o_polygon_vertexs_.at(i).GetYm() + o_local_gap.GetYm(), 0);
        }
        // o_arrive_move_polygon
        vec_save_pose.emplace_back(o_arrive_move_polygon_local);
    }

    for (int j = vec_save_pose.size() - 1; j >= 0; j--) {
        vec_polygon_docking_path_check.at(0).AddVertex(vec_save_pose.at(j).o_polygon_vertexs_.at(1));
        vec_polygon_docking_path_check.at(1).AddVertex(vec_save_pose.at(j).o_polygon_vertexs_.at(2));
        vec_polygon_docking_path_check.at(2).AddVertex(vec_save_pose.at(j).o_polygon_vertexs_.at(3));
        vec_polygon_docking_path_check.at(3).AddVertex(vec_save_pose.at(j).o_polygon_vertexs_.at(0));
        if (j == 0) {
            for (int k = 0; k < vec_save_pose.size(); k++) {
                vec_polygon_docking_path_check.at(0).AddVertex(vec_save_pose.at(k).o_polygon_vertexs_.at(2));
                vec_polygon_docking_path_check.at(1).AddVertex(vec_save_pose.at(k).o_polygon_vertexs_.at(3));
                vec_polygon_docking_path_check.at(2).AddVertex(vec_save_pose.at(k).o_polygon_vertexs_.at(0));
                vec_polygon_docking_path_check.at(3).AddVertex(vec_save_pose.at(k).o_polygon_vertexs_.at(1));
            }
        }
    }
    vec_polygon_docking_path_check.at(0).AddVertex(vec_polygon_docking_path_check.at(0).o_polygon_vertexs_.front());

    SetCheckPolygon(CheckPolygon::MOVE, vec_polygon_docking_path_check);
}

void LidarObstacle::SetCheckPolygon(int n_polygon, const vector<NaviFra::Polygon>& o_polygon)
{
    std::lock_guard<std::mutex> lock(mtx_polygon_);
    if (n_polygon == CheckPolygon::MOVE)
        vec_polygon_docking_path_check_ = o_polygon;
    else if (n_polygon == CheckPolygon::TARGET)
        vec_target_obstacle_polygon_ = o_polygon;
    else if (n_polygon == CheckPolygon::START)
        vec_polygon_start_check_ = o_polygon;
}

vector<NaviFra::Polygon> LidarObstacle::GetCheckPolygon(int n_polygon)
{
    std::lock_guard<std::mutex> lock(mtx_polygon_);
    vector<NaviFra::Polygon> vec_polygon;
    if (n_polygon == CheckPolygon::MOVE)
        vec_polygon = vec_polygon_docking_path_check_;
    else if (n_polygon == CheckPolygon::TARGET)
        vec_polygon = vec_target_obstacle_polygon_;
    else if (n_polygon == CheckPolygon::START)
        vec_polygon = vec_polygon_start_check_;
    return vec_polygon;
}

void LidarObstacle::StartObsPolyMaker(
    const Pos& o_robot_pos, const vector<Pos>& vec_pos_local_path, const Parameters_t& st_param, NaviFra::Polygon& o_polygon)
{
    Pos o_differ_pos;
    std::vector<NaviFra::Polygon> vec_start_obs_save_pose;
    std::vector<NaviFra::Polygon> vec_polygon_start_check(4);
    float f_start_obs_step_m = st_param.f_start_obs_step_m;
    if (f_start_obs_step_m == 0)
        f_start_obs_step_m = 0.1;
    int n_robot_path_idx = CoreCalculator::FindMinDistanceIdxFromPosVector_(o_robot_pos, vec_pos_local_path);
    float f_start_check_dist = st_param.f_start_obs_check_m;
    if (f_start_check_dist * 100 >= vec_pos_local_path.size() - n_robot_path_idx)
        f_start_check_dist = float(vec_pos_local_path.size() - n_robot_path_idx) / 100.0;
    float f_check_step = f_start_check_dist / f_start_obs_step_m;
    for (int j = 0; j <= f_check_step; j++) {
        NaviFra::Polygon o_start_obs_check_polygon;
        int start_check_pos_idx = n_robot_path_idx + f_start_obs_step_m * j * 100;
        if (start_check_pos_idx >= vec_pos_local_path.size()) {
            break;
        }
        o_differ_pos = vec_pos_local_path.at(start_check_pos_idx) - vec_pos_local_path.at(n_robot_path_idx);
        o_differ_pos = CoreCalculator::TransformRotationDeg_(o_differ_pos, -o_robot_pos.GetDeg());
        NaviFra::Polygon o_polygon_tmp = o_polygon;
        std::vector<NaviFra::Pos> o_start_check_polygon_vertexs(4);

        float f_robot_rad = o_robot_pos.GetRad();
        if(vec_pos_local_path.at(start_check_pos_idx).GetConstDriveInfo().e_drive_type == Pos::DRIVE_TYPE::REAR || 
            vec_pos_local_path.at(start_check_pos_idx).GetConstDriveInfo().e_drive_type == Pos::DRIVE_TYPE::REAR_DIAGONAL)
            f_robot_rad -= M_PI;
        o_polygon_tmp.UpdateVertexRotate(vec_pos_local_path.at(start_check_pos_idx).GetRad() - f_robot_rad);

        for (int i = 0; i < o_polygon_tmp.o_polygon_vertexs_.size(); i++) {
            o_start_check_polygon_vertexs.at(i) = NaviFra::Pos(
                o_polygon_tmp.o_polygon_vertexs_.at(i).GetXm() + o_differ_pos.GetXm(),
                o_polygon_tmp.o_polygon_vertexs_.at(i).GetYm() + o_differ_pos.GetYm(), 0);
            o_start_obs_check_polygon.AddVertex(o_start_check_polygon_vertexs.at(i));
        }
        vec_start_obs_save_pose.emplace_back(o_start_obs_check_polygon);
    }

    for (int j = 0; j < vec_start_obs_save_pose.size(); j++) {
        vec_polygon_start_check.at(0).AddVertex(vec_start_obs_save_pose.at(j).o_polygon_vertexs_.at(1));
        vec_polygon_start_check.at(1).AddVertex(vec_start_obs_save_pose.at(j).o_polygon_vertexs_.at(2));
        vec_polygon_start_check.at(2).AddVertex(vec_start_obs_save_pose.at(j).o_polygon_vertexs_.at(3));
        vec_polygon_start_check.at(3).AddVertex(vec_start_obs_save_pose.at(j).o_polygon_vertexs_.at(0));
        if (j == vec_start_obs_save_pose.size() - 1) {
            for (int k = vec_start_obs_save_pose.size() - 1; k >= 0; k--) {
                vec_polygon_start_check.at(0).AddVertex(vec_start_obs_save_pose.at(k).o_polygon_vertexs_.at(2));
                vec_polygon_start_check.at(1).AddVertex(vec_start_obs_save_pose.at(k).o_polygon_vertexs_.at(3));
                vec_polygon_start_check.at(2).AddVertex(vec_start_obs_save_pose.at(k).o_polygon_vertexs_.at(0));
                vec_polygon_start_check.at(3).AddVertex(vec_start_obs_save_pose.at(k).o_polygon_vertexs_.at(1));
            }
        }
    }
    // vec_polygon_start_check_ = vec_polygon_start_check;
    SetCheckPolygon(CheckPolygon::START, vec_polygon_start_check);
}

float LidarObstacle::SideObstacleCheck(
    float f_path_vel, const Parameters_t& st_param, NaviFra::Polygon& o_polygon,
    const std::vector<NaviFra::SimplePos>& vec_sensors_relative_robot)
{
    NaviFra::Polygon o_side_check_polygon;
    float f_obs_target_speed_vel = f_path_vel;
    int n_sensor_size = vec_sensors_relative_robot.size();
    o_side_check_polygon = o_polygon;
    o_side_check_polygon.UpdateVertexSizePlusFLRR(
        st_param.f_side_check_margin, st_param.f_side_check_margin, st_param.f_side_check_margin, st_param.f_side_check_margin);
    for (int j = 0; j < n_sensor_size; j++) {
        // 체크할 라이다 포인트
        NaviFra::Pos o_checking_pos(vec_sensors_relative_robot.at(j).GetXm(), vec_sensors_relative_robot.at(j).GetYm());
        // 해당 지점이 로봇에 충돌한 경우
        if (o_side_check_polygon.GetPointInPolygon(o_checking_pos)) {
            f_obs_target_speed_vel = f_path_vel * st_param.f_side_target_speed_ratio;
            break;
        }
    }
    return f_obs_target_speed_vel;
}

void LidarObstacle::UpdatePolyBySpeed(NaviFra::Polygon& o_polygon, Parameters_t& st_param, float f_linear_speed_x, float f_angular_speed_degs, int n_detect_check)
{    
    float f_obs_margin_plus_front = 0.0;
    float f_obs_margin_plus_rear = 0.0;
    float f_obs_margin_plus_left = st_param.f_side_base_detect_dist;
    float f_obs_margin_plus_right = st_param.f_side_base_detect_dist;
    float f_spinturn_add_radius = fabs(st_param.f_spinturn_add_radius);
    if (n_detect_check == 0) {
        if (fabs(f_linear_speed_x) < 0.01f && fabs(f_angular_speed_degs) > 0.01f) {
            // 회전만 하는 경우: 정팔각형 구성
            double r = 0.0;
            for (const auto& vertex : o_polygon.o_polygon_vertexs_) {
                float dist = hypot(vertex.GetXm(), vertex.GetYm());
                r = std::max(r, static_cast<double>(dist));
            }
            if(f_spinturn_add_radius > 1){
                r += 1.0;
            }
            else {
                r += st_param.f_spinturn_add_radius;
            }
            o_polygon.Clear();
            for (int i = 0; i < 16; ++i) {
                double angle_rad = M_PI * 2.0 * i / 16.0;
                double x = r * cos(angle_rad);
                double y = r * sin(angle_rad);
                o_polygon.AddVertex(Pos(x, y, 0));
            }
        }
        else {
            if (f_linear_speed_x >= 0.01f) {
                f_obs_margin_plus_front = st_param.f_base_detect_dist;
                if (fabs(f_linear_speed_x) > 0.2f){
                    if (f_linear_speed_x <= 1.0f) {
                        // 0~1.0 구간: 0.2마다 0.1씩 증가
                        int n_add_margin = static_cast<int>(f_linear_speed_x / st_param.f_speed_step_std);
                        f_obs_margin_plus_front += n_add_margin * 0.1f;
                    } else {
                        // 1.0 이상 구간: 0.2마다 0.2씩 증가
                        int n_add_margin_low = static_cast<int>(1.0f / st_param.f_speed_step_std); // 0~1.0 구간 단계 수
                        int n_add_margin_high = static_cast<int>((f_linear_speed_x - 1.0f) / st_param.f_speed_step_std);
                        f_obs_margin_plus_front += n_add_margin_low*st_param.f_low_speed_step_add_dist + n_add_margin_high * st_param.f_high_speed_step_add_dist;
                    }
                }
                if (f_angular_speed_degs >= 5.0f) {
                    f_obs_margin_plus_left += st_param.f_side_add_dist;
                    // if (fabs(f_angular_speed_degs) > 5.0){
                        // int n_add_margin = fabs(f_angular_speed_degs)/ (f_detect_margin_angular_step_ * M_PI / 180.0);
                    // }
                }
                else if (f_angular_speed_degs <= -5.0f) {
                    f_obs_margin_plus_right += st_param.f_side_add_dist;
                    // if (fabs(f_angular_speed_degs) > 5.0){
                        // int n_add_margin = fabs(f_angular_speed_degs)/ (f_detect_margin_angular_step_ * M_PI / 180.0);
                    // }
                }
            }
            else if (f_linear_speed_x <= -0.01f) {
                f_obs_margin_plus_rear = st_param.f_base_detect_dist;
                if (fabs(f_linear_speed_x) > 0.2f){
                    if (fabs(f_linear_speed_x) <= 1.0f) {
                        // 0~1.0 구간: 0.2마다 0.1씩 증가
                        int n_add_margin = static_cast<int>(fabs(f_linear_speed_x) / st_param.f_speed_step_std);
                        f_obs_margin_plus_rear += n_add_margin * 0.1f;
                    } else {
                        // 1.0 이상 구간: 0.2마다 0.2씩 증가
                        int n_add_margin_low = static_cast<int>(1.0f / st_param.f_speed_step_std); // 0~1.0 구간 단계 수
                        int n_add_margin_high = static_cast<int>((fabs(f_linear_speed_x) - 1.0f) / st_param.f_speed_step_std);
                        f_obs_margin_plus_rear += n_add_margin_low*st_param.f_low_speed_step_add_dist + n_add_margin_high * st_param.f_high_speed_step_add_dist;
                    }
                }
                if (f_angular_speed_degs >= 5.0f) {
                    f_obs_margin_plus_right += st_param.f_side_add_dist;
                    // if (fabs(f_angular_speed_degs) > 5.0f){
                        // int n_add_margin = fabs(f_angular_speed_degs)/ (f_detect_margin_angular_step_ * M_PI / 180.0);
                        // f_obs_margin_plus_right += (side_base_add_margin*side_base_add_margin);
                    // }
                }
                else if (f_angular_speed_degs <= -5.0f) {
                    f_obs_margin_plus_left += st_param.f_side_add_dist;
                    // if (fabs(f_angular_speed_degs) > 5.0f){
                        // int n_add_margin = fabs(f_angular_speed_degs)/ (f_detect_margin_angular_step_ * M_PI / 180.0);
                        // f_obs_margin_plus_left += (side_base_add_margin*side_base_add_margin);
                    // }
                }
            }
            // 일반 마진 보정 적용
            o_polygon.UpdateVertexSizePlusFLRR(
                f_obs_margin_plus_front,
                f_obs_margin_plus_rear,
                f_obs_margin_plus_left,
                f_obs_margin_plus_right
            );
        }
    }
    else {
        // 감지 중일 땐 이전 폴리곤 유지
        o_polygon = o_polygon_pre_;
    }

    // 최종 폴리곤 백업
    o_polygon_pre_ = o_polygon;
}

float LidarObstacle::PredictObstacleCheck(
    const Parameters_t& st_param, NaviFra::Polygon& o_polygon, const std::vector<NaviFra::SimplePos>& vec_sensors_relative_robot,
    bool b_arrive_check, bool b_docking_path_obs_check, const CommandVelocity_t& regenerated_st_vel, bool b_diagonal_path)
{
    vector<NaviFra::Polygon> vec_target_obstacle_polygon = GetCheckPolygon(CheckPolygon::TARGET);
    vector<NaviFra::Polygon> vec_polygon_docking_path_check = GetCheckPolygon(CheckPolygon::MOVE);
    float f_obs_speed_vel = 1;
    bool b_predict_check = false;
    int n_max_predict_step_num = st_param.n_predict_step_num;
    int n_sensor_size = vec_sensors_relative_robot.size();
    for (int i = 0; i <= n_max_predict_step_num; i++)  // 실제 및 가상 변수 추가
    {
        NaviFra::Polygon o_predict_polygon;
        std::vector<NaviFra::Pos> o_polygon_vertexs(4);
        o_polygon_vertexs.resize(o_polygon.o_polygon_vertexs_.size());
        float predict_step = st_param.f_collision_predict_sec * i / st_param.n_predict_step_num;
        float predict_x_step = regenerated_st_vel.f_linear_speed_x_ms * predict_step;
        float predict_y_step = regenerated_st_vel.f_linear_speed_y_ms * predict_step;
        float predict_deg_step = regenerated_st_vel.f_angular_speed_degs * predict_step;

        // 사행 경로가 아닐때만 예측 감지영역 줄이기
        if (!b_diagonal_path)
        {
            if(regenerated_st_vel.f_angular_speed_degs < st_param.f_reduce_predict_deg_standard && regenerated_st_vel.f_angular_speed_degs > -st_param_.f_reduce_predict_deg_standard ){
                predict_deg_step = regenerated_st_vel.f_angular_speed_degs * predict_step / st_param.n_reduce_predict_deg_step;
                predict_y_step = regenerated_st_vel.f_linear_speed_y_ms * predict_step / st_param.n_reduce_predict_deg_step;
            }
            else{
                predict_deg_step = regenerated_st_vel.f_angular_speed_degs * predict_step;
                predict_y_step = regenerated_st_vel.f_linear_speed_y_ms * predict_step;
            }
        }

        for (int j = 0; j < o_polygon_vertexs.size(); j++) {
            o_polygon_vertexs.at(j) = NaviFra::Pos(
                o_polygon.o_polygon_vertexs_.at(j).GetXm() + predict_x_step, o_polygon.o_polygon_vertexs_.at(j).GetYm() + predict_y_step,
                0);
            o_predict_polygon.AddVertex(CoreCalculator::TransformRotationDeg_(o_polygon_vertexs.at(j), predict_deg_step));
        }

        for (int j = 0; j < n_sensor_size; j++) {
            if(vec_sensors_relative_robot.at(j).GetZm() == 4) {
                continue;
            }
            // 체크할 라이다 포인트
            NaviFra::Pos o_checking_pos(vec_sensors_relative_robot.at(j).GetXm(), vec_sensors_relative_robot.at(j).GetYm());
            // 해당 지점이 로봇에 충돌한 경우
            if (o_predict_polygon.GetPointInPolygon(o_checking_pos)) {
                b_predict_check = true;
                f_obs_speed_vel = float(i) / float(st_param.n_predict_step_num);
                break;
            }
        }

        if(b_predict_check) // 더 부드럽게 정지 예측하기 위해서
        {
            int n_find_step = st_param.n_predict_step_num;
            for (int k = 0; k <= n_find_step; k++)  // 실제 및 가상 변수 추가
            {
                o_predict_polygon.Clear();
                float predict_step = st_param.f_collision_predict_sec * float(i - 1/float(n_find_step)*k) / float(st_param.n_predict_step_num);
                float predict_x_step = regenerated_st_vel.f_linear_speed_x_ms * predict_step;
                float predict_y_step = 0;
                float predict_deg_step = 0;
        
        
                if(regenerated_st_vel.f_angular_speed_degs < st_param.f_reduce_predict_deg_standard && regenerated_st_vel.f_angular_speed_degs > -st_param_.f_reduce_predict_deg_standard ){
                    predict_deg_step = regenerated_st_vel.f_angular_speed_degs * predict_step / st_param.n_reduce_predict_deg_step;
                    predict_y_step = regenerated_st_vel.f_linear_speed_y_ms * predict_step / st_param.n_reduce_predict_deg_step;
                }
                else{
                    predict_deg_step = regenerated_st_vel.f_angular_speed_degs * predict_step;
                    predict_y_step = regenerated_st_vel.f_linear_speed_y_ms * predict_step;
                }
        
                for (int j = 0; j < o_polygon_vertexs.size(); j++) {
                    o_polygon_vertexs.at(j) = NaviFra::Pos(
                        o_polygon.o_polygon_vertexs_.at(j).GetXm() + predict_x_step, o_polygon.o_polygon_vertexs_.at(j).GetYm() + predict_y_step,
                        0);
                    o_predict_polygon.AddVertex(CoreCalculator::TransformRotationDeg_(o_polygon_vertexs.at(j), predict_deg_step));
                }
                bool b_find = false;
                for (int j = 0; j < n_sensor_size; j++) {
                    if(vec_sensors_relative_robot.at(j).GetZm() == 4) {
                        continue;
                    }
                    // 체크할 라이다 포인트
                    NaviFra::Pos o_checking_pos(vec_sensors_relative_robot.at(j).GetXm(), vec_sensors_relative_robot.at(j).GetYm());
                    // 해당 지점이 로봇에 충돌한 경우
                    if (o_predict_polygon.GetPointInPolygon(o_checking_pos)) {
                        b_find = true;
                        o_obs_pos_ = o_checking_pos;
                        break;
                    }
                }
                if(b_find == false)
                {
                    // NLOG(info)<<"kkkk "<<k;
                    f_obs_speed_vel = float(i - 1/float(n_find_step)*k) / float(st_param.n_predict_step_num);
                    break;
                }
            }
        }

        // arrive check polygon add
        if (b_arrive_check) {
            for (int j = 0; j < vec_target_obstacle_polygon.size(); j++) {
                o_predict_polygon.o_polygon_vertexs_.insert(
                    o_predict_polygon.o_polygon_vertexs_.end(), vec_target_obstacle_polygon.at(j).o_polygon_vertexs_.begin(),
                    vec_target_obstacle_polygon.at(j).o_polygon_vertexs_.end());
            }
        }
        if (b_docking_path_obs_check) {
            o_predict_polygon.o_polygon_vertexs_.insert(
                o_predict_polygon.o_polygon_vertexs_.end(), vec_polygon_docking_path_check.at(0).o_polygon_vertexs_.begin(),
                vec_polygon_docking_path_check.at(0).o_polygon_vertexs_.end());
            o_predict_polygon.o_polygon_vertexs_.insert(
                o_predict_polygon.o_polygon_vertexs_.end(), vec_polygon_docking_path_check.at(1).o_polygon_vertexs_.begin(),
                vec_polygon_docking_path_check.at(1).o_polygon_vertexs_.end());
            o_predict_polygon.o_polygon_vertexs_.insert(
                o_predict_polygon.o_polygon_vertexs_.end(), vec_polygon_docking_path_check.at(2).o_polygon_vertexs_.begin(),
                vec_polygon_docking_path_check.at(2).o_polygon_vertexs_.end());
            o_predict_polygon.o_polygon_vertexs_.insert(
                o_predict_polygon.o_polygon_vertexs_.end(), vec_polygon_docking_path_check.at(3).o_polygon_vertexs_.begin(),
                vec_polygon_docking_path_check.at(3).o_polygon_vertexs_.end());
        }
        if ((i == st_param.n_predict_step_num - 1 && !b_predict_check) || b_predict_check) {
            static ros::Publisher poly_collision_pub_ =
                node_handle_.advertise<geometry_msgs::PolygonStamped>("/NaviFra/visualize/robot_collision_predict", 5, true);
            geometry_msgs::PolygonStamped msg_collision = DrawPolygon(o_predict_polygon.o_polygon_vertexs_);
            poly_collision_pub_.publish(msg_collision);
        }
        if (b_predict_check) {
            break;
        }
    }
    return f_obs_speed_vel;
}

}  // namespace NaviFra