#include "lane_avoid_path.hpp"

#include "core/util/logger.hpp"
#include "debug/debug_visualizer.hpp"
using namespace std;
namespace NaviFra {
void LaneAvoidPath::SetParam(const Parameters_t& st_param)
{
    std::lock_guard<std::mutex> lock(mtx_param_);
    st_param_ = st_param;
    vec_avoid_sample_point_.clear();
    for (int i = st_param_.st_lane_avoidance_param.n_search_min_step; i <= st_param_.st_lane_avoidance_param.n_search_max_step; i++) {
        Avoid_sample_point o_tmp_sample_point;
        o_tmp_sample_point.n_s = i;
        o_tmp_sample_point.f_s_m = i * st_param_.st_lane_avoidance_param.f_search_step_size_m;
        o_tmp_sample_point.n_d = 0;
        o_tmp_sample_point.f_d_m = 0;
        vec_avoid_sample_point_.emplace_back(o_tmp_sample_point);

        for (int j = 1; j <= max(st_param_.st_lane_avoidance_param.n_left_num, st_param_.st_lane_avoidance_param.n_right_num); j++) {
            o_tmp_sample_point.n_d = j;
            o_tmp_sample_point.f_d_m = j * st_param_.st_lane_avoidance_param.f_interval_m;
            vec_avoid_sample_point_.emplace_back(o_tmp_sample_point);
            o_tmp_sample_point.n_d = -j;
            o_tmp_sample_point.f_d_m = -j * st_param_.st_lane_avoidance_param.f_interval_m;
            vec_avoid_sample_point_.emplace_back(o_tmp_sample_point);
        }
    }
    vec_avoid_sample_edge_.resize(vec_avoid_sample_point_.size() * vec_avoid_sample_point_.size());
    for (Avoid_sample_edge& e : vec_avoid_sample_edge_) {
        e.b_use = true;
        e.edge.clear();
    }
    LOG_INFO(
        "max_curvature : %f, n_search_max_step : %d, n_search_min_step : %d, f_search_step_size_m : %f",
        st_param_.f_polynominal_max_curvature_m, st_param_.st_lane_avoidance_param.n_search_max_step,
        st_param_.st_lane_avoidance_param.n_search_min_step, st_param_.st_lane_avoidance_param.f_search_step_size_m);

    LOG_INFO("[avoid2] sample point size: %d", (int)vec_avoid_sample_point_.size());
}

int LaneAvoidPath::GetSampleIdx(int n_s, int n_d)
{
    if (n_s < st_param_.st_lane_avoidance_param.n_search_min_step || n_s > st_param_.st_lane_avoidance_param.n_search_max_step)
        return -1;
    int n_max_lane_num = max(st_param_.st_lane_avoidance_param.n_left_num, st_param_.st_lane_avoidance_param.n_right_num);
    if (n_d < -n_max_lane_num || n_d > n_max_lane_num)
        return -1;
    int n_s_idx = n_s - st_param_.st_lane_avoidance_param.n_search_min_step;
    int n_d_idx = 0;
    if (n_d > 0)
        n_d_idx = 2 * n_d - 1;
    else
        n_d_idx = -2 * n_d;
    return n_s_idx * (2 * n_max_lane_num + 1) + n_d_idx;
}

void LaneAvoidPath::ResetSamples()
{
    for (int i = 0; i < (int)vec_avoid_sample_point_.size(); i++) {
        vec_avoid_sample_point_.at(i).b_use = false;
    }
    for (Avoid_sample_edge& e : vec_avoid_sample_edge_) {
        e.b_use = true;
        e.edge.clear();
    }
}

void LaneAvoidPath::SetSensorMsg(const SensorMsg_t& st_sensor_msgs)
{
    std::lock_guard<std::mutex> lock(mtx_sensor_msg_);
    st_sensor_msgs_ = st_sensor_msgs;
}

void LaneAvoidPath::SetForceComebackTargetIdx(int n_force_comeback_target_idx)
{
    n_force_comeback_target_idx_ = n_force_comeback_target_idx;
}

void LaneAvoidPath::SetBackwardFlag(bool b_move_backward_flag)
{
    b_move_backward_flag_ = b_move_backward_flag;
}

void LaneAvoidPath::SetCollisionDetectorPtr(std::shared_ptr<CollisionDetector> o_collision_detector_ptr)
{
    o_collision_detector_ptr_ = o_collision_detector_ptr;
}

std::vector<NaviFra::Avoid_sector_t> LaneAvoidPath::DevideMissionSector(const std::vector<NaviFra::Pos>& vec_path)
{
    std::vector<Avoid_sector_t> st_temp_avoid_status_list;
    Avoid_sector_t st_current_avoid_sector;
    Avoid_sector_t st_prev_avoid_sector;
    bool b_first_flag = true;
    bool b_last_flag = false;

    for (int i = 0; i < vec_path.size(); i++) {
        Pos::DriveInfo_t o_drive_info = vec_path.at(i).GetConstDriveInfo();
        // o_drive_info.b_avoidance_right << " step -> " <<  o_drive_info.n_avoidance_step << endl;
        if (i == int(vec_path.size()) - 1) {
            b_last_flag == true;
            st_current_avoid_sector.n_index = i;
            st_current_avoid_sector.n_avoid_type = o_drive_info.n_avoid_type;
            st_temp_avoid_status_list.emplace_back(st_current_avoid_sector);
            break;
        }

        st_current_avoid_sector.n_index = i;
        st_current_avoid_sector.n_avoid_type = o_drive_info.n_avoid_type;
        st_current_avoid_sector.n_avoid_step_l_end = o_drive_info.n_avoidance_step;
        if (o_drive_info.b_avoidance_left == false) {
            st_current_avoid_sector.n_avoid_step_l_end = 0;
        }
        st_current_avoid_sector.n_avoid_step_r_end = o_drive_info.n_avoidance_step;
        if (o_drive_info.b_avoidance_right == false) {
            st_current_avoid_sector.n_avoid_step_r_end = 0;
        }
        st_current_avoid_sector.e_curve_type = o_drive_info.e_curve_type;

        if (b_first_flag == true) {
            st_temp_avoid_status_list.emplace_back(st_current_avoid_sector);
            st_prev_avoid_sector.n_index = st_current_avoid_sector.n_index;
            st_prev_avoid_sector.n_avoid_type = st_current_avoid_sector.n_avoid_type;
            st_prev_avoid_sector.n_avoid_step_l_end = st_current_avoid_sector.n_avoid_step_l_end;
            st_prev_avoid_sector.n_avoid_step_r_end = st_current_avoid_sector.n_avoid_step_r_end;
            st_prev_avoid_sector.e_curve_type = st_current_avoid_sector.e_curve_type;
            b_first_flag = false;
        }
        else {
            if (st_current_avoid_sector != st_prev_avoid_sector) {
                st_temp_avoid_status_list.emplace_back(st_current_avoid_sector);
                st_prev_avoid_sector.n_index = st_current_avoid_sector.n_index;
                st_prev_avoid_sector.n_avoid_type = st_current_avoid_sector.n_avoid_type;
                st_prev_avoid_sector.n_avoid_step_l_end = st_current_avoid_sector.n_avoid_step_l_end;
                st_prev_avoid_sector.n_avoid_step_r_end = st_current_avoid_sector.n_avoid_step_r_end;
                st_prev_avoid_sector.e_curve_type = st_current_avoid_sector.e_curve_type;
            }
        }
    }
    return st_temp_avoid_status_list;
}

std::vector<Pos> LaneAvoidPath::MakeAvoidArcPath(const NaviFra::Pos& o_start_arc_node, const NaviFra::Pos& o_end_arc_node)
{
    vector<Pos> vec_result;

    bool b_clock_wise_dir = false;
    if (o_start_arc_node.GetConstDriveInfo().f_circle_angle < 0)
        b_clock_wise_dir = true;

    Pos s_pos = o_start_arc_node;
    if (b_clock_wise_dir == true) {
        s_pos.SetRad(
            atan2(
                o_start_arc_node.GetYm() - o_start_arc_node.GetConstDriveInfo().f_circle_pos_y,
                o_start_arc_node.GetXm() - o_start_arc_node.GetConstDriveInfo().f_circle_pos_x) -
            M_PI / 2);
    }
    else {
        s_pos.SetRad(
            atan2(
                o_start_arc_node.GetYm() - o_start_arc_node.GetConstDriveInfo().f_circle_pos_y,
                o_start_arc_node.GetXm() - o_start_arc_node.GetConstDriveInfo().f_circle_pos_x) +
            M_PI / 2);
    }
    Pos e_pos = o_end_arc_node;
    if (b_clock_wise_dir == true) {
        e_pos.SetRad(
            atan2(
                o_end_arc_node.GetYm() - o_start_arc_node.GetConstDriveInfo().f_circle_pos_y,
                o_end_arc_node.GetXm() - o_start_arc_node.GetConstDriveInfo().f_circle_pos_x) -
            M_PI / 2);
    }
    else {
        e_pos.SetRad(
            atan2(
                o_end_arc_node.GetYm() - o_start_arc_node.GetConstDriveInfo().f_circle_pos_y,
                o_end_arc_node.GetXm() - o_start_arc_node.GetConstDriveInfo().f_circle_pos_x) +
            M_PI / 2);
    }

    Pos center_pos(s_pos.GetDriveInfo().f_circle_pos_x, s_pos.GetDriveInfo().f_circle_pos_y);

    float f_dist_pos = hypot(s_pos.GetXm() - e_pos.GetXm(), s_pos.GetYm() - e_pos.GetYm());
    float f_radius_circle = f_dist_pos / 2 / sin(abs(s_pos.GetRad() - e_pos.GetRad()) / 2);

    if (f_radius_circle < 0.01)
        return vec_result;

    float f_deg_error = s_pos.GetDeg() - e_pos.GetDeg();
    if (f_deg_error > 180)
        f_deg_error -= 360;
    else if (f_deg_error < -180)
        f_deg_error += 360;

    float f_arc_length = fabs(f_deg_error) * M_PI * f_radius_circle / 180;
    int N = (int)(f_arc_length / 0.01);

    float f_x = s_pos.GetXm() - center_pos.GetXm();
    float f_y = s_pos.GetYm() - center_pos.GetYm();
    float f_a = s_pos.GetRad();

    // make arc path point
    for (int j = 0; j < N; j++) {
        Pos tmp(f_x, f_y);
        if (b_clock_wise_dir == true) {
            tmp = CoreCalculator::TransformRotationDeg_(tmp, -0.01 * 180 / M_PI / f_radius_circle * j);
            tmp.SetRad(atan2(tmp.GetYm(), tmp.GetXm()) - M_PI / 2);
        }
        else {
            tmp = CoreCalculator::TransformRotationDeg_(tmp, 0.01 * 180 / M_PI / f_radius_circle * j);
            tmp.SetRad(atan2(tmp.GetYm(), tmp.GetXm()) + M_PI / 2);
        }
        tmp.SetXm(tmp.GetXm() + center_pos.GetXm());
        tmp.SetYm(tmp.GetYm() + center_pos.GetYm());
        tmp.SetDriveInfo(s_pos.GetDriveInfo());
        tmp.GetDriveInfo().f_circle_pos_x = center_pos.GetXm();
        tmp.GetDriveInfo().f_circle_pos_y = center_pos.GetYm();
        tmp.GetDriveInfo().f_circle_angle = -f_deg_error;

        vec_result.emplace_back(tmp);
    }

    return vec_result;
}

Extend_path_result LaneAvoidPath::MakeRemainPath(
    const std::vector<NaviFra::Pos>& vec_tmp_path, int n_target_lane_num, Lane_info_t st_lane_info)
{
    std::vector<Pos> vec_result_path;
    Extend_path_result st_extend_result;
    bool b_extend_available = true;
    std::vector<Avoid_sector_t> vec_avoid_sector_list = DevideMissionSector(vec_tmp_path);
    // vec_avoid_sector_list.resize(vec_avoid_sector_list.size() - 1);

    // 직선 경로일 때
    for (int i = 0; i < vec_avoid_sector_list.size() - 1; i++) {
        // float f_path_angle_deg =
        //     atan2(
        //         vec_tmp_path.at(vec_avoid_sector_list.at(i).n_index + 3).GetYm() -
        //         vec_tmp_path.at(vec_avoid_sector_list.at(i).n_index).GetYm(), vec_tmp_path.at(vec_avoid_sector_list.at(i).n_index +
        //         3).GetXm() - vec_tmp_path.at(vec_avoid_sector_list.at(i).n_index).GetXm()) *
        //     RADtoDEG;
        float f_path_angle_deg = vec_tmp_path.at(vec_avoid_sector_list.at(i).n_index).GetDeg();
        // 왼쪽 회피 차선

        if (vec_avoid_sector_list.at(i).e_curve_type == Pos::LINE_TYPE::LINE) {
            if (n_target_lane_num == 0) {
                LOG_DEBUG("[LaneAvoidPath::MakeRemainPath] extend_path line target lane : 0")
                st_extend_result.vec_extend_path = vec_tmp_path;
                st_extend_result.b_extend_available = true;
                return st_extend_result;
            }
            else {
                LOG_DEBUG("[LaneAvoidPath::MakeRemainPath] extend_path line target lane : %d", n_target_lane_num)
                NaviFra::Pos o_avoid_pos_dm(0, st_lane_info.f_lane_dist * n_target_lane_num, 0);
                o_avoid_pos_dm = NaviFra::CoreCalculator::TransformRotationDeg_(o_avoid_pos_dm, f_path_angle_deg);
                NaviFra::Pos pos1 = vec_tmp_path.at(vec_avoid_sector_list.at(i).n_index);
                NaviFra::Pos pos2 = vec_tmp_path.at(vec_avoid_sector_list.at(i + 1).n_index);
                NaviFra::Pos dist_pos1 = pos1 + o_avoid_pos_dm;
                NaviFra::Pos dist_pos2 = pos2 + o_avoid_pos_dm;
                dist_pos1.SetDriveInfo(pos1.GetDriveInfo());
                dist_pos2.SetDriveInfo(pos2.GetDriveInfo());

                vector<NaviFra::Pos> avoid_path = CoreCalculator::GetRayPosToTarget_(dist_pos1, dist_pos2, 0.01);

                SetAvoidStatus(avoid_path, true);
                vec_result_path.insert(vec_result_path.end(), avoid_path.begin(), avoid_path.end());
            }
        }
        // 곡선 경로일 때
        else {
            if (n_target_lane_num == 0) {
                LOG_DEBUG("[LaneAvoidPath::MakeRemainPath] extend_path curve target lane : 0")
                st_extend_result.vec_extend_path = vec_tmp_path;
                st_extend_result.b_extend_available = true;
                return st_extend_result;
            }
            //   vec_tmp_path.at(vec_avoid_sector_list.at(i).n_index).GetDriveInfo().f_circle_angle < 0
            else {
                LOG_DEBUG("[LaneAvoidPath::MakeRemainPath] extend_path curve target lane : %d", n_target_lane_num)
                NaviFra::Pos o_avoid_pos_dm_start(0, st_lane_info.f_lane_dist * n_target_lane_num, 0);
                NaviFra::Pos o_avoid_pos_dm_end(0, st_lane_info.f_lane_dist * n_target_lane_num, 0);
                NaviFra::Pos pos1 = vec_tmp_path.at(vec_avoid_sector_list.at(i).n_index);
                NaviFra::Pos pos2 = vec_tmp_path.at(vec_avoid_sector_list.at(i + 1).n_index);
                bool b_clock_wise_dir = false;
                if (pos1.GetConstDriveInfo().f_circle_angle < 0)
                    b_clock_wise_dir = true;

                if (b_clock_wise_dir == true) {
                    pos1.SetRad(
                        atan2(
                            pos1.GetYm() - pos1.GetConstDriveInfo().f_circle_pos_y,
                            pos1.GetXm() - pos1.GetConstDriveInfo().f_circle_pos_x) -
                        M_PI / 2);
                }
                else {
                    pos1.SetRad(
                        atan2(
                            pos1.GetYm() - pos1.GetConstDriveInfo().f_circle_pos_y,
                            pos1.GetXm() - pos1.GetConstDriveInfo().f_circle_pos_x) +
                        M_PI / 2);
                }
                o_avoid_pos_dm_start = NaviFra::CoreCalculator::TransformRotationDeg_(o_avoid_pos_dm_start, pos1.GetDeg());
                if (b_clock_wise_dir == true) {
                    pos2.SetRad(
                        atan2(
                            pos2.GetYm() - pos1.GetConstDriveInfo().f_circle_pos_y,
                            pos2.GetXm() - pos1.GetConstDriveInfo().f_circle_pos_x) -
                        M_PI / 2);
                }
                else {
                    pos2.SetRad(
                        atan2(
                            pos2.GetYm() - pos1.GetConstDriveInfo().f_circle_pos_y,
                            pos2.GetXm() - pos1.GetConstDriveInfo().f_circle_pos_x) +
                        M_PI / 2);
                }
                o_avoid_pos_dm_end = NaviFra::CoreCalculator::TransformRotationDeg_(o_avoid_pos_dm_end, pos2.GetDeg());

                // 곡선 회피차선 제한
                // 반시계 왼쪽 회피 차선 제한
                if (b_clock_wise_dir == false) {
                    // float f_dist_pos = hypot(pos1.GetXm() - pos2.GetXm(), pos1.GetYm() - pos2.GetYm());
                    // float f_radius_circle = f_dist_pos / 2 / sin(abs(pos1.GetRad() - pos2.GetRad()) / 2);
                    float f_radius_circle = hypot(
                        pos1.GetConstDriveInfo().f_circle_pos_x - pos1.GetXm(), pos1.GetConstDriveInfo().f_circle_pos_y - pos1.GetYm());
                    // LOG_DEBUG("[LaneAvoidPath::MakeRemainPath] f_radius_circle counter_clock_wise_dir : %f", f_radius_circle);
                    if ((st_lane_info.f_lane_dist * n_target_lane_num) >= (f_radius_circle - st_lane_info.f_lane_dist / 2)) {
                        LOG_DEBUG("Can not make Arc Avoid Lane(counter_clockwise) / target_lane_num : %d", n_target_lane_num);
                        b_extend_available = false;
                        break;
                    }
                }  // 시계 오른쪽 회피 차선 제한
                else {
                    // float f_dist_pos = hypot(pos1.GetXm() - pos2.GetXm(), pos1.GetYm() - pos2.GetYm());
                    // float f_radius_circle = f_dist_pos / 2 / sin(abs(pos1.GetRad() - pos2.GetRad()) / 2);
                    float f_radius_circle = hypot(
                        pos1.GetConstDriveInfo().f_circle_pos_x - pos1.GetXm(), pos1.GetConstDriveInfo().f_circle_pos_y - pos1.GetYm());
                    // LOG_DEBUG("[LaneAvoidPath::MakeRemainPath] f_radius_circle clock_wise_dir : %f", f_radius_circle);
                    if ((-(st_lane_info.f_lane_dist * n_target_lane_num)) >= (f_radius_circle - st_lane_info.f_lane_dist / 2)) {
                        LOG_DEBUG("Can not make Arc Avoid Lane(clockwise) / target_lane_num : %d", n_target_lane_num);
                        b_extend_available = false;
                        break;
                    }
                }

                NaviFra::Pos dist_pos1 = pos1 + o_avoid_pos_dm_start;
                NaviFra::Pos dist_pos2 = pos2 + o_avoid_pos_dm_end;
                dist_pos1.SetDriveInfo(pos1.GetDriveInfo());
                // 반시계
                if (b_clock_wise_dir == false) {
                    dist_pos1.GetDriveInfo().f_curve_radius =
                        pos1.GetDriveInfo().f_curve_radius - (st_lane_info.f_lane_dist * n_target_lane_num);
                }  //시계
                else {
                    dist_pos1.GetDriveInfo().f_curve_radius =
                        pos1.GetDriveInfo().f_curve_radius + (st_lane_info.f_lane_dist * n_target_lane_num);
                }
                dist_pos2.SetDriveInfo(pos2.GetDriveInfo());

                vector<NaviFra::Pos> avoid_arc_path = MakeAvoidArcPath(dist_pos1, dist_pos2);
                SetAvoidStatus(avoid_arc_path, true);
                vec_result_path.insert(vec_result_path.end(), avoid_arc_path.begin(), avoid_arc_path.end());
            }
        }
    }

    st_extend_result.vec_extend_path = vec_result_path;
    st_extend_result.b_extend_available = b_extend_available;
    return st_extend_result;
}

std::vector<Pos> LaneAvoidPath::MakeDubinsPath(
    const NaviFra::Pos& o_dubins_start_pos, const NaviFra::Pos& o_dubins_end_pos, float f_radius, float f_linear_speed_ms,
    bool b_force_comeback)
{
    vector<NaviFra::Pos> vec_dubins_path;
    double q0[] = {(double)o_dubins_start_pos.GetXm(), (double)o_dubins_start_pos.GetYm(), (double)o_dubins_start_pos.GetRad()};
    // goal
    double q1[] = {(double)o_dubins_end_pos.GetXm(), (double)o_dubins_end_pos.GetYm(), (double)o_dubins_end_pos.GetRad()};
    // initialize the path
    NaviFra::DubinsPath path;
    // calculate the path
    float min_radius = f_radius;
    dubins_init(q0, q1, min_radius, &path);

    if (b_force_comeback == false) {
        if (path.param[0] > M_PI / 2 || path.param[2] > M_PI / 2) {
            LOG_DEBUG("Radius too big");
            return vec_dubins_path;
        }
    }
    else {
        if (path.param[0] > M_PI || path.param[2] > M_PI) {
            LOG_DEBUG("Radius too big");
            return vec_dubins_path;
        }
    }

    int i = 0;
    double x = 0.f;
    double length = dubins_path_length(&path);

    float prev_q[3];
    Pos::DriveInfo_t o_drive_info = o_dubins_start_pos.GetConstDriveInfo();
    int type = path.type;
    float f_s = 0;
    // avoid duplicate waypoint
    while (x < length) {
        double q[3];
        int type = 0;

        dubins_path_sample(&path, x, q, type);
        float f_angle_deg = (float)q[2] * RADtoDEG;
        f_angle_deg = CoreCalculator::CalcAngleDomainDeg_(f_angle_deg);
        NaviFra::Pos o_tmp;
        o_tmp.SetXm((float)q[0]);
        o_tmp.SetYm((float)q[1]);
        if (x == 0) {
            o_tmp.SetZm(0);
        }
        else {
            f_s += (float)hypot(q[0] - prev_q[0], q[1] - prev_q[1]);
            o_tmp.SetZm(f_s);
        }
        if (type == 0 || type == 2) {
            o_drive_info.e_curve_type = Pos::CURVE;
            o_drive_info.f_curve_radius = min_radius;
            o_drive_info.f_curvature = 1 / min_radius;
        }
        else {
            o_drive_info.e_curve_type = Pos::LINE;
            o_drive_info.f_curve_radius = 0;
            o_drive_info.f_curvature = 0;
        }

        o_tmp.SetDeg(f_angle_deg);
        o_tmp.SetDriveInfo(o_drive_info);

        vec_dubins_path.emplace_back(o_tmp);

        std::copy(q, q + 3, prev_q);
        x += 0.01;
        i++;
    }

    return vec_dubins_path;
}

vector<NaviFra::Pos> LaneAvoidPath::MakePolynominalPath(const NaviFra::Pos& o_start_pos, const NaviFra::Pos& o_end_pos)
{
    // Start point
    float x0 = o_start_pos.GetXm(), y0 = o_start_pos.GetYm(), a0 = o_start_pos.GetRad(), c0 = o_start_pos.GetConstDriveInfo().f_curvature,
          v0 = 1.0;
    // End point
    float x1 = o_end_pos.GetXm(), y1 = o_end_pos.GetYm(), a1 = o_end_pos.GetRad(), c1 = o_end_pos.GetConstDriveInfo().f_curvature, v1 = 1.0;

    if ((v0 + v1) == 0) {
        LOG_DEBUG("[LaneAvoidPath::MakePolynominalPath] Speed Zero")
        return std::vector<NaviFra::Pos>();
    }

    float f_distance = sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));

    for (int i = 0; i < 3; i++) {
        f_distance = PathDistanceOptimization(x0, y0, a0, c0, v0, x1, y1, a1, c1, v1, f_distance);
        if (f_distance == 0) {
            LOG_DEBUG("[LaneAvoidPath::MakePolynominalPath] Distance Zero")
            return std::vector<NaviFra::Pos>();
        }
    }

    float V_min = min(v0, v1);
    float V_max = max(v0, v1);
    // float T_min = 0.5 * sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0)) / V_max;
    // float T_max = 2 * sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0)) / V_min;
    // float T_mid = 2 * sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0)) / (V_min + V_max);

    float T_mid = 2 * f_distance / (V_min + V_max);

    float T = T_mid;

    // Polynomial function
    float T_5 = pow(T, 5);
    float T_4 = pow(T, 4);
    float T_3 = pow(T, 3);
    float T_2 = pow(T, 2);

    Eigen::Matrix3d mat_A;
    mat_A << T_5, T_4, T_3, 5 * T_4, 4 * T_3, 3 * T_2, 20 * T_3, 12 * T_2, 6 * T;

    if (mat_A.determinant() == 0) {
        LOG_DEBUG("[Error] (T= %f ) Det(A) == 0", T);
        return std::vector<NaviFra::Pos>();
    }

    Eigen::Matrix3d inv_A = mat_A.inverse();

    // x(t) coefficients
    Eigen::Vector3d vec_b;
    vec_b << x1 + 0.5 * c0 * v0 * v0 * sin(a0) * T_2 - v0 * cos(a0) * T - x0, v1 * cos(a1) + c0 * v0 * v0 * sin(a0) * T - v0 * cos(a0),
        c0 * v0 * v0 * sin(a0) - c1 * v1 * v1 * sin(a1);

    Eigen::Vector3d vec_p = inv_A * vec_b;
    PolynominalPath x_poly(vec_p(0), vec_p(1), vec_p(2), -0.5 * c0 * v0 * v0 * sin(a0), v0 * cos(a0), x0);

    // y(t) coefficients
    vec_b << y1 - 0.5 * c0 * v0 * v0 * cos(a0) * T_2 - v0 * sin(a0) * T - y0, v1 * sin(a1) - c0 * v0 * v0 * cos(a0) * T - v0 * sin(a0),
        c1 * v1 * v1 * cos(a1) - c0 * v0 * v0 * cos(a0);

    vec_p = inv_A * vec_b;

    PolynominalPath y_poly(vec_p(0), vec_p(1), vec_p(2), 0.5 * c0 * v0 * v0 * cos(a0), v0 * sin(a0), y0);
    std::vector<float> t_array;
    std::vector<float> vx_array;
    std::vector<float> vy_array;
    float t = 0.0;

    while (t < T) {
        t_array.emplace_back(t);
        vx_array.emplace_back(x_poly.GetVel(t_array.back()));
        vy_array.emplace_back(y_poly.GetVel(t_array.back()));
        t = t + 0.01 / (sqrt(pow(vx_array.back(), 2) + pow(vy_array.back(), 2)));
    }

    std::vector<float> x_array = x_poly.GetPosVector(t_array);
    std::vector<float> y_array = y_poly.GetPosVector(t_array);
    std::vector<float> ax_array = x_poly.GetAccArray(t_array);
    std::vector<float> ay_array = y_poly.GetAccArray(t_array);

    // Calculate curvature array
    std::vector<float> c_array = CalCurvArray(vx_array, vy_array, ax_array, ay_array);
    if (c_array.empty()) {
        LOG_ERROR("[LaneAvoidPath::MakePolynominalPath] Curve Array Size is Zero");
        return std::vector<NaviFra::Pos>();
    }

    std::vector<float> ang_arr, vel_arr;
    RecalAngVelAccArray(vx_array, vy_array, ax_array, ay_array, ang_arr, vel_arr);

    size_t n_array_size = x_array.size();

    if (n_array_size != y_array.size() || n_array_size != ang_arr.size() || n_array_size != vel_arr.size() ||
        n_array_size != c_array.size()) {
        LOG_ERROR("[LaneAvoidPath::MakePolynominalPath] Polynominal Array Size Not Same");
        return std::vector<NaviFra::Pos>();
    }

    std::vector<NaviFra::Pos> vec_polynomial_path;
    vec_polynomial_path.resize(n_array_size);
    vec_polynomial_path.at(0).SetZm(0);
    for (size_t i = 0; i < n_array_size; ++i) {
        if (fabs(c_array.at(i)) > st_param_.f_polynominal_max_curvature_m) {
            LOG_DEBUG("[LaneAvoidPath::MakePolynominalPath] Path Over Max Curvatuve %.3f", fabs(c_array.at(i)));
            return std::vector<NaviFra::Pos>();
        }

        vec_polynomial_path.at(i).SetXm(x_array.at(i));
        vec_polynomial_path.at(i).SetYm(y_array.at(i));
        vec_polynomial_path.at(i).SetRad(ang_arr.at(i));

        // Pos::DriveInfo_t o_drive_info = o_start_pos.GetConstDriveInfo();
        if (c_array.at(i) * c_array.at(i) < 0.00001) {
            vec_polynomial_path.at(i).GetDriveInfo().f_curvature = 0;
            vec_polynomial_path.at(i).GetDriveInfo().f_curve_radius = 0;
            vec_polynomial_path.at(i).GetDriveInfo().e_curve_type = Pos::LINE_TYPE::LINE;
            // o_drive_info.f_curvature = 0;
            // o_drive_info.f_curve_radius = 0;
            // o_drive_info.e_curve_type = Pos::LINE_TYPE::LINE;
        }
        else {
            vec_polynomial_path.at(i).GetDriveInfo().f_curvature = c_array.at(i);
            vec_polynomial_path.at(i).GetDriveInfo().f_curve_radius = 1.0f / c_array.at(i);
            vec_polynomial_path.at(i).GetDriveInfo().e_curve_type = Pos::LINE_TYPE::CURVE;
            // o_drive_info.f_curvature = c_array.at(i);
            // o_drive_info.f_curve_radius = 1/c_array.at(i);
            // o_drive_info.e_curve_type = Pos::LINE_TYPE::CURVE;
        }

        if (i != 0) {
            float f_ds = hypotf(
                vec_polynomial_path.at(i).GetXm() - vec_polynomial_path.at(i - 1).GetXm(),
                vec_polynomial_path.at(i).GetYm() - vec_polynomial_path.at(i - 1).GetYm());
            vec_polynomial_path.at(i).SetZm(vec_polynomial_path.at(i - 1).GetZm() + f_ds);
        }
    }

    SetAvoidStatus(vec_polynomial_path, true);
    return vec_polynomial_path;
}

std::vector<int> LaneAvoidPath::SetlaneSearchOrder(
    const std::vector<NaviFra::Pos>& vec_ref_path, const NaviFra::Pos& o_start_pos, int n_current_lane_num, Lane_info_t st_lane_info)
{
    std::vector<int> vec_search_order;
    int n_start_ref_idx = CoreCalculator::FindMinDistanceIdxFromPosVector_(o_start_pos, vec_ref_path);
    NaviFra::Pos o_ref_start_pos = vec_ref_path.at(n_start_ref_idx);
    float f_angle_diff = o_start_pos.GetDeg() - o_ref_start_pos.GetDeg();
    f_angle_diff = CoreCalculator::CalcAngleDomainDeg_(f_angle_diff);
    // Explore lanes in increasing distance from the current lane
    if (f_angle_diff > 5) {
        for (int i = n_current_lane_num + 1; i <= st_lane_info.n_left_max_num; i++) {
            vec_search_order.push_back(i);
        }
        for (int i = n_current_lane_num - 1; i >= -st_lane_info.n_right_max_num; i--) {
            vec_search_order.push_back(i);
        }
    }
    else if (f_angle_diff < -5) {
        for (int i = n_current_lane_num - 1; i >= -st_lane_info.n_right_max_num; i--) {
            vec_search_order.push_back(i);
        }
        for (int i = n_current_lane_num + 1; i <= st_lane_info.n_left_max_num; i++) {
            vec_search_order.push_back(i);
        }
    }
    else {
        if (n_current_lane_num >= 0) {
            for (int i = 1;
                 i <= std::max(st_lane_info.n_left_max_num - n_current_lane_num, st_lane_info.n_right_max_num + n_current_lane_num); ++i) {
                if (n_current_lane_num + i <= st_lane_info.n_left_max_num) {
                    vec_search_order.push_back(n_current_lane_num + i);
                }
                if (n_current_lane_num - i >= -st_lane_info.n_right_max_num) {
                    vec_search_order.push_back(n_current_lane_num - i);
                }
            }
        }
        else {
            for (int i = 1;
                 i <= std::max(st_lane_info.n_left_max_num - n_current_lane_num, st_lane_info.n_right_max_num + n_current_lane_num); ++i) {
                if (n_current_lane_num - i >= -st_lane_info.n_right_max_num) {
                    vec_search_order.push_back(n_current_lane_num - i);
                }
                if (n_current_lane_num + i <= st_lane_info.n_left_max_num) {
                    vec_search_order.push_back(n_current_lane_num + i);
                }
            }
        }
    }

    for (int i = 0; i < vec_search_order.size(); i++) {
        LOG_DEBUG("vec_search_order : %d ", vec_search_order.at(i));
    }

    return vec_search_order;
}

Avoid_path_result LaneAvoidPath::CheckAvoidPath(
    const std::vector<NaviFra::Pos>& vec_ref_path, const NaviFra::Pos& o_start_pos, Lane_info_t st_lane_info, int n_current_lane_num,
    float f_min_dist)
{
    LOG_INFO("[LaneAvoidPath::CheckAvoidPath] Start Finding Avoid Path");
    int n_over_curv_cnt = 0;
    int n_collision_cnt = 0;
    int n_success_cnt = 0;
    int n_min_cost = 1E6;
    Avoid_path_result st_avoid_path_result;
    Extend_path_result st_extend_result;
    vector<NaviFra::Pos> vec_avoid_path;
    int n_start_ref_idx = CoreCalculator::FindMinDistanceIdxFromPosVector_(o_start_pos, vec_ref_path);
    std::vector<int> vec_search_order = SetlaneSearchOrder(vec_ref_path, o_start_pos, n_current_lane_num, st_lane_info);

    for (int n_lane_num : vec_search_order) {
        for (int n_dist_step = st_param_.st_lane_avoidance_param.n_search_max_step;
             n_dist_step >= st_param_.st_lane_avoidance_param.n_search_min_step; n_dist_step--) {
            int n_front_dist_idx = 100 * st_param_.st_lane_avoidance_param.f_search_step_size_m * n_dist_step;

            if ((n_start_ref_idx + n_front_dist_idx) > n_force_comeback_target_idx_) {
                LOG_DEBUG("[LaneAvoidPath::CheckAvoidPath] Go over n_force_comeback_target_idx_");
                vec_avoid_path.clear();
                continue;
            }

            // 다른 lane으로 이동시 두 lane을 잇는 점 포인트
            NaviFra::Pos o_ref_curve_end_pos = vec_ref_path.at(n_start_ref_idx + n_front_dist_idx);
            int n_left_lane_max_num =
                o_ref_curve_end_pos.GetConstDriveInfo().b_avoidance_left * o_ref_curve_end_pos.GetConstDriveInfo().n_avoidance_step;
            int n_right_lane_max_num =
                o_ref_curve_end_pos.GetConstDriveInfo().b_avoidance_right * o_ref_curve_end_pos.GetConstDriveInfo().n_avoidance_step;
            if ((n_lane_num > n_left_lane_max_num) || (n_lane_num < -n_right_lane_max_num)) {
                vec_avoid_path.clear();
                continue;
            }

            NaviFra::Pos o_avoid_pos_curve_end_dm(0, st_lane_info.f_lane_dist * (n_lane_num), 0);
            o_avoid_pos_curve_end_dm =
                NaviFra::CoreCalculator::TransformRotationDeg_(o_avoid_pos_curve_end_dm, o_ref_curve_end_pos.GetDeg());
            NaviFra::Pos o_curve_end_pos = o_ref_curve_end_pos + o_avoid_pos_curve_end_dm;
            // LOG_DEBUG("[Noah_check] avoid_target_pos line_type : %d, curvature : %f, f_circle_angle : %f, curve_radius : %f, circle_pos_x
            // : %f, circle_pos_y: %f, radius : %f, cirecle_angle : %f", o_ref_curve_end_pos.GetConstDriveInfo().e_curve_type,
            // o_ref_curve_end_pos.GetConstDriveInfo().f_curvature, o_ref_curve_end_pos.GetConstDriveInfo().f_circle_angle,
            // o_ref_curve_end_pos.GetConstDriveInfo().f_curve_radius, o_ref_curve_end_pos.GetConstDriveInfo().f_circle_pos_x ,
            // o_ref_curve_end_pos.GetConstDriveInfo().f_circle_pos_y, hypot(o_ref_curve_end_pos.GetConstDriveInfo().f_circle_pos_x -
            // o_ref_curve_end_pos.GetXm(),o_ref_curve_end_pos.GetConstDriveInfo().f_circle_pos_y - o_ref_curve_end_pos.GetYm()));
            if (o_ref_curve_end_pos.GetConstDriveInfo().e_curve_type == Pos::LINE_TYPE::CURVE) {
                bool b_clock_wise_dir = false;
                if (o_ref_curve_end_pos.GetConstDriveInfo().f_circle_angle < 0)
                    b_clock_wise_dir = true;

                if (b_clock_wise_dir == false) {
                    // float f_dist_pos = hypot(pos1.GetXm() - pos2.GetXm(), pos1.GetYm() - pos2.GetYm());
                    // float f_radius_circle = f_dist_pos / 2 / sin(abs(pos1.GetRad() - pos2.GetRad()) / 2);
                    float f_radius_circle = hypot(
                        o_ref_curve_end_pos.GetConstDriveInfo().f_circle_pos_x - o_ref_curve_end_pos.GetXm(),
                        o_ref_curve_end_pos.GetConstDriveInfo().f_circle_pos_y - o_ref_curve_end_pos.GetYm());
                    // LOG_INFO("[Noah_check] f_radius_circle counter_clock_wise_dir : %f", f_radius_circle);
                    if ((st_lane_info.f_lane_dist * n_lane_num) >= (f_radius_circle - st_lane_info.f_lane_dist / 2)) {
                        // NLOG(info) << "Can not make Arc Avoid Lane(counter_clockwise) / target_lane_num : " << n_lane_num;
                        vec_avoid_path.clear();
                        continue;
                    }
                }  // 시계 오른쪽 회피 차선 제한
                else {
                    // float f_dist_pos = hypot(pos1.GetXm() - pos2.GetXm(), pos1.GetYm() - pos2.GetYm());
                    // float f_radius_circle = f_dist_pos / 2 / sin(abs(pos1.GetRad() - pos2.GetRad()) / 2);
                    float f_radius_circle = hypot(
                        o_ref_curve_end_pos.GetConstDriveInfo().f_circle_pos_x - o_ref_curve_end_pos.GetXm(),
                        o_ref_curve_end_pos.GetConstDriveInfo().f_circle_pos_y - o_ref_curve_end_pos.GetYm());
                    // LOG_INFO("[Noah_check] f_radius_circle clock_wise_dir : %f", f_radius_circle);
                    if ((-(st_lane_info.f_lane_dist * n_lane_num)) >= (f_radius_circle - st_lane_info.f_lane_dist / 2)) {
                        // NLOG(info) << "Can not make Arc Avoid Lane(clockwise) / target_lane_num : " <<  n_lane_num;
                        vec_avoid_path.clear();
                        continue;
                    }
                }
            }
            // o_start_pos 와 o_curve_end_pos 잇기
            vector<NaviFra::Pos> vec_temp_path_1;
            // for (float f_radius_step = st_param_.f_dubins_normal_max_curve_radius_m; f_radius_step >=
            // st_param_.f_dubins_normal_min_curve_radius_m ; f_radius_step -= st_param_.f_dubins_normal_curve_interval_m)
            // {
            //     vec_temp_path_1 = MakeDubinsPath(o_start_pos, o_curve_end_pos, f_radius_step,
            //     vec_ref_path.front().GetConstDriveInfo().f_linear, false); if (vec_temp_path_1.size() != 0)
            //     {
            //         break;
            //     }
            // }

            vec_temp_path_1 = MakePolynominalPath(o_start_pos, o_curve_end_pos);

            if (vec_temp_path_1.size() == 0) {
                LOG_DEBUG("[LaneAvoidPath::CheckAvoidPath] Polynominal Path Size = 0");
                vec_avoid_path.clear();
                continue;
            }
            SetAvoidStatus(vec_temp_path_1, true);
            vec_avoid_path.insert(vec_avoid_path.end(), vec_temp_path_1.begin(), vec_temp_path_1.end());

            // extend avoid path
            if (vec_ref_path.at(n_start_ref_idx + n_front_dist_idx).GetZm() - vec_ref_path.at(n_start_ref_idx).GetZm() < f_min_dist) {
                vector<NaviFra::Pos> vec_temp_path_2;
                int n_idx_dist = 0;
                for (int i = n_start_ref_idx + n_front_dist_idx; i <= n_force_comeback_target_idx_; i++) {
                    float f_plus_dist = vec_ref_path.at(i).GetZm() - vec_ref_path.at(n_start_ref_idx + n_front_dist_idx).GetZm();
                    n_idx_dist = i - (n_start_ref_idx + n_front_dist_idx);
                    if (f_plus_dist + vec_avoid_path.back().GetZm() > f_min_dist) {
                        break;
                    }
                }

                int a = 1;
                while (true) {
                    vector<NaviFra::Pos> vec_parted_ref_path;
                    if (n_idx_dist == 0) {
                        break;
                    }
                    int n_extend_dist_idx = n_idx_dist * a;
                    if ((n_start_ref_idx + n_front_dist_idx + n_extend_dist_idx) >= n_force_comeback_target_idx_) {
                        n_extend_dist_idx = n_force_comeback_target_idx_ - (n_start_ref_idx + n_front_dist_idx);
                    }
                    std::copy(
                        vec_ref_path.begin() + n_start_ref_idx + n_front_dist_idx,
                        vec_ref_path.begin() + n_start_ref_idx + n_front_dist_idx + (n_extend_dist_idx) + 1,
                        std::back_inserter(vec_parted_ref_path));

                    st_extend_result = MakeRemainPath(vec_parted_ref_path, n_lane_num, st_lane_info);
                    vec_temp_path_2 = std::move(st_extend_result.vec_extend_path);

                    if (vec_temp_path_2.empty())
                        break;

                    float f_ds = hypot(
                        vec_temp_path_2.at(0).GetXm() - vec_avoid_path.back().GetXm(),
                        vec_temp_path_2.at(0).GetYm() - vec_avoid_path.back().GetYm());
                    vec_temp_path_2.at(0).SetZm(f_ds + vec_avoid_path.back().GetZm());
                    // set S(거리, Zm)
                    for (int i = 1; i < (int)vec_temp_path_2.size(); i++) {
                        f_ds = hypot(
                            vec_temp_path_2.at(i).GetXm() - vec_temp_path_2.at(i - 1).GetXm(),
                            vec_temp_path_2.at(i).GetYm() - vec_temp_path_2.at(i - 1).GetYm());
                        vec_temp_path_2.at(i).SetZm(f_ds + vec_temp_path_2.at(i - 1).GetZm());
                    }

                    if (((int)vec_temp_path_2.size() > 0) &&
                        ((vec_temp_path_2.back().GetZm() > f_min_dist) ||
                         (n_extend_dist_idx == (n_force_comeback_target_idx_ - (n_start_ref_idx + n_front_dist_idx))))) {
                        int n_cut_idx = 0;
                        for (; n_cut_idx < (int)vec_temp_path_2.size(); n_cut_idx++) {
                            // Pos o_global_gap = Pos(vec_temp_path_2.at(n_cut_idx).GetXm() - vec_avoid_path.back().GetXm(),
                            // vec_temp_path_2.at(n_cut_idx).GetYm() - vec_avoid_path.back().GetYm(), 0); Pos o_local_gap =
                            // CoreCalculator::TransformRotationRad_(o_global_gap, -vec_avoid_path.back().GetRad());
                            float f_global_gap = hypot(
                                vec_temp_path_2.at(n_cut_idx).GetXm() - vec_avoid_path.back().GetXm(),
                                vec_temp_path_2.at(n_cut_idx).GetYm() - vec_avoid_path.back().GetYm());
                            if (fabs(f_global_gap) > 0.005)
                            // if(o_local_gap.GetXm() > 0.005)
                            {
                                break;
                            }
                        }

                        if (n_cut_idx != 0) {
                            vec_temp_path_2.erase(vec_temp_path_2.begin(), vec_temp_path_2.begin() + n_cut_idx);
                        }
                        // if (n_lane_num != 0)
                        //     SetAvoidStatus(vec_temp_path_2, true);
                        // else
                        //     SetAvoidStatus(vec_temp_path_2, false);
                        SetAvoidStatus(vec_temp_path_2, true);
                        vec_avoid_path.insert(vec_avoid_path.end(), vec_temp_path_2.begin(), vec_temp_path_2.end());
                        break;
                    }
                    else {
                        vec_temp_path_2.clear();
                    }
                    a += 1;
                }
            }

            bool b_extend_available = st_extend_result.b_extend_available;
            // check_curvature
            // SetFullSm(vec_avoid_path);
            SetCurvature(vec_avoid_path);
            if (b_extend_available == false) {
                vec_avoid_path.clear();
                continue;
            }
            for (int i = 0; i < (int)vec_avoid_path.size(); i++) {
                if (fabs(vec_avoid_path.at(i).GetConstDriveInfo().f_curvature) > st_param_.f_polynominal_max_curvature_m) {
                    LOG_TRACE(
                        "[LaneAvoidPath::CheckAvoidPath] OverCurvature >> lane_num=%d, dist_step=%d, index=%d, x=%.2f, y=%.2f, deg=%.1f, k=%.2f",
                        n_lane_num, n_dist_step, i, vec_avoid_path.at(i).GetXm(), vec_avoid_path.at(i).GetYm(),
                        vec_avoid_path.at(i).GetDeg(), vec_avoid_path.at(i).GetConstDriveInfo().f_curvature);
                    ++n_over_curv_cnt;
                    vec_avoid_path.clear();
                    break;
                }
            }

            if ((int)vec_avoid_path.size() <= 0) {
                continue;
            }

            // 경로 collision check 및 경로 전달
            // st_avoid_path_result.vec_avoid_path = vec_avoid_path;
            // st_avoid_path_result.n_target_lane_num = n_lane_num;
            // return st_avoid_path_result;
            int n_collision_idx =
                o_collision_detector_ptr_->CheckPathCollision(vec_avoid_path, st_param_.f_avoid_margin_m, b_move_backward_flag_, 0);
            if (n_collision_idx == -1) {
                ++n_success_cnt;
                int n_nowCost = CalcCost(vec_avoid_path, n_lane_num);
                if (n_min_cost > n_nowCost) {
                    n_min_cost = n_nowCost;
                    st_avoid_path_result.vec_avoid_path = vec_avoid_path;
                    st_avoid_path_result.n_target_lane_num = n_lane_num;
                }
            }
            else {
                LOG_TRACE(
                    "[LaneAvoidPath::CheckAvoidPath] Collision >> lane_num=%d, dist_step=%d, index=%d, x=%.2f, y=%.2f, deg=%.1f",
                    n_lane_num, n_dist_step, n_collision_idx, vec_avoid_path.at(n_collision_idx).GetXm(),
                    vec_avoid_path.at(n_collision_idx).GetYm(), vec_avoid_path.at(n_collision_idx).GetDeg());
                ++n_collision_cnt;
            }
            //--------------------------------

            vec_avoid_path.clear();
        }
    }

    if (st_avoid_path_result.vec_avoid_path.empty()) {
        LOG_WARNING("[LaneAvoidPath::CheckAvoidPath] Finding AvoidPath Failed");
    }
    else {
        LOG_INFO("[LaneAvoidPath::CheckAvoidPath] Finding AvoidPath Success");
    }
    LOG_DEBUG(
        "[LaneAvoidPath::CheckAvoidPath] Summerize >> success path num=%d, collision path num=%d, over curvature path num=%d",
        n_success_cnt, n_collision_cnt, n_over_curv_cnt);
    return st_avoid_path_result;
}

Avoid_path_result LaneAvoidPath::CheckAvoidPathTwice(
    const std::vector<NaviFra::Pos>& vec_ref_path, const NaviFra::Pos& o_start_pos, Lane_info_t st_lane_info, int n_current_lane_num,
    float f_min_dist)
{
    LOG_DEBUG("[LaneAvoidPath::CheckAvoidPathTwice] Start Finding Avoid Path Twice");
    std::chrono::steady_clock::time_point time_start;
    std::chrono::duration<double> check_sec;
    int n_cnt = 0;

    time_start = std::chrono::steady_clock::now();

    Avoid_path_result st_avoid_path_result;
    Extend_path_result st_extend_result;
    vector<NaviFra::Pos> vec_avoid_path;
    ResetSamples();
    int spSize = (int)vec_avoid_sample_point_.size();

    check_sec = std::chrono::steady_clock::now() - time_start;
    LOG_DEBUG("[LaneAvoidPath::CheckAvoidPathTwice] (ResetSamples) Timer : %.3lf", check_sec.count());
    time_start = std::chrono::steady_clock::now();

    int n_left_num = st_param_.st_lane_avoidance_param.n_left_num;
    int n_right_num = st_param_.st_lane_avoidance_param.n_right_num;
    int n_start_idx = CoreCalculator::FindMinDistanceIdxFromPosVector_(o_start_pos, vec_ref_path);
    for (int i = n_start_idx; i < (int)vec_ref_path.size(); i++) {
        int n_step = vec_ref_path.at(i).GetConstDriveInfo().n_avoidance_step;
        if (!vec_ref_path.at(i).GetConstDriveInfo().b_avoidance_left)
            n_left_num = 0;
        else if (n_left_num > n_step)
            n_left_num = n_step;
        if (!vec_ref_path.at(i).GetConstDriveInfo().b_avoidance_right)
            n_right_num = 0;
        else if (n_right_num > n_step)
            n_right_num = n_step;
        if ((vec_ref_path.at(i).GetZm() - vec_ref_path.at(n_start_idx).GetZm()) > f_min_dist)
            break;
    }

    check_sec = std::chrono::steady_clock::now() - time_start;
    LOG_DEBUG("[LaneAvoidPath::CheckAvoidPathTwice] (reference path) Timer : %.3lf", check_sec.count());
    time_start = std::chrono::steady_clock::now();

    for (std::vector<Avoid_sample_point>::iterator it = vec_avoid_sample_point_.begin(); it != vec_avoid_sample_point_.end(); ++it) {
        // 0. check lane num
        if (it->n_d > n_left_num || it->n_d < -n_right_num)
            continue;

        // 1. calc global x,y
        float _f_min_dist = vec_ref_path.back().GetZm();
        int n_ref_idx;
        //-------------------------------------------------------------------------------------------------------------------
        for (int i = n_start_idx; i < (int)vec_ref_path.size(); i++) {
            float f_dist = fabs(it->f_s_m - (vec_ref_path.at(i).GetZm() - vec_ref_path.at(n_start_idx).GetZm()));
            if (_f_min_dist > f_dist) {
                _f_min_dist = f_dist;
                n_ref_idx = i;
            }
        }

        // 다른 lane으로 이동시 두 lane을 잇는 점 포인트
        NaviFra::Pos o_ref_curve_end_pos = vec_ref_path.at(n_ref_idx);
        NaviFra::Pos o_avoid_pos_curve_end_dm(0, it->f_d_m, 0);
        o_avoid_pos_curve_end_dm = NaviFra::CoreCalculator::TransformRotationDeg_(o_avoid_pos_curve_end_dm, o_ref_curve_end_pos.GetDeg());
        NaviFra::Pos o_curve_end_pos = o_ref_curve_end_pos + o_avoid_pos_curve_end_dm;
        o_curve_end_pos.SetDeg(o_ref_curve_end_pos.GetDeg());

        //---------------------------------------------------------------------------------------------------------
        // 2. collision detect
        if (!o_collision_detector_ptr_->CheckPosCollision(o_curve_end_pos, st_param_.f_avoid_margin_m, b_move_backward_flag_)) {
            it->b_use = true;
            it->o_pos = o_curve_end_pos;
        }
    }

    std::vector<NaviFra::Pos> points;
    for (const Avoid_sample_point& sp : vec_avoid_sample_point_) {
        if (sp.b_use)
            points.emplace_back(sp.o_pos);
    }

    check_sec = std::chrono::steady_clock::now() - time_start;
    LOG_DEBUG("[LaneAvoidPath::CheckAvoidPathTwice] (sample collision) Timer : %.3lf", check_sec.count());
    time_start = std::chrono::steady_clock::now();

    std::queue<std::vector<int>> qu;

    for (int n_idx0 = 0; n_idx0 < spSize - st_param_.st_lane_avoidance_param.n_search_min_step; n_idx0++) {
        if (!vec_avoid_sample_point_.at(n_idx0).b_use)
            continue;

        int n_s0 = vec_avoid_sample_point_.at(n_idx0).n_s;
        int n_d1 = vec_avoid_sample_point_.at(n_idx0).n_d;

        for (int n_s1 = n_s0;
             n_s1 <= st_param_.st_lane_avoidance_param.n_search_max_step - st_param_.st_lane_avoidance_param.n_search_min_step; ++n_s1) {
            int n_idx1 = (n_s0 == n_s1) ? n_idx0 : GetSampleIdx(n_s1, n_d1);
            if (!vec_avoid_sample_point_.at(n_idx1).b_use)
                break;

            for (int n_idx2 = GetSampleIdx(n_s1 + st_param_.st_lane_avoidance_param.n_search_min_step, 0); n_idx2 < spSize; n_idx2++) {
                ++n_cnt;
                if (n_d1 != vec_avoid_sample_point_.at(n_idx2).n_d && vec_avoid_sample_point_.at(n_idx2).b_use) {
                    qu.push(std::vector<int>({n_idx0, n_idx1, n_idx2}));
                }
            }
        }
    }

    check_sec = std::chrono::steady_clock::now() - time_start;
    LOG_DEBUG("[LaneAvoidPath::CheckAvoidPathTwice] (triple queue) Size : %d", (int)qu.size());
    LOG_DEBUG("[LaneAvoidPath::CheckAvoidPathTwice] (triple queue) Count : %d", n_cnt);
    LOG_DEBUG("[LaneAvoidPath::CheckAvoidPathTwice] (triple queue) Timer : %.3lf", check_sec.count());
    time_start = std::chrono::steady_clock::now();
    n_cnt = 0;

    int n_min_cost = 1E6;
    std::map<int, char> map_n_d_collision;

    while (!qu.empty()) {
        int n_idx0 = qu.front().at(0);
        int n_idx1 = qu.front().at(1);
        int n_idx2 = qu.front().at(2);
        qu.pop();
        const Avoid_sample_point& st_sp0 = vec_avoid_sample_point_.at(n_idx0);
        const Avoid_sample_point& st_sp1 = vec_avoid_sample_point_.at(n_idx1);
        const Avoid_sample_point& st_sp2 = vec_avoid_sample_point_.at(n_idx2);
        Avoid_sample_edge& st_edge0 = vec_avoid_sample_edge_.at(n_idx0 + 0 * spSize);
        Avoid_sample_edge& st_edge1 = vec_avoid_sample_edge_.at(n_idx1 + n_idx0 * spSize);
        Avoid_sample_edge& st_edge2 = vec_avoid_sample_edge_.at(n_idx2 + n_idx1 * spSize);

        if (!st_edge0.b_use || !st_edge1.b_use || !st_edge2.b_use)
            continue;
        if (map_n_d_collision.find(vec_avoid_sample_point_.at(n_idx2).n_d) != map_n_d_collision.end())
            continue;

        if (st_edge0.edge.empty()) {
            st_edge0.edge = MakePolynominalPath(o_start_pos, st_sp0.o_pos);
            if (st_edge0.edge.empty()) {
                st_edge0.b_use = false;
                continue;
            }
            if (-1 != o_collision_detector_ptr_->CheckPathCollision(st_edge0.edge, st_param_.f_avoid_margin_m, b_move_backward_flag_, 0)) {
                st_edge0.b_use = false;
                st_edge0.edge.clear();
                continue;
            }
        }
        if (n_idx0 != n_idx1 && st_edge1.edge.empty()) {
            st_edge1.edge = MakePolynominalPath(st_sp0.o_pos, st_sp1.o_pos);
            if (st_edge1.edge.empty()) {
                st_edge1.b_use = false;
                continue;
            }
            if (-1 != o_collision_detector_ptr_->CheckPathCollision(st_edge1.edge, st_param_.f_avoid_margin_m, b_move_backward_flag_, 0)) {
                st_edge1.b_use = false;
                st_edge1.edge.clear();
                continue;
            }
        }
        if (st_edge2.edge.empty()) {
            st_edge2.edge = MakePolynominalPath(st_sp1.o_pos, st_sp2.o_pos);
            if (st_edge2.edge.empty()) {
                st_edge2.b_use = false;
                continue;
            }
            if (-1 != o_collision_detector_ptr_->CheckPathCollision(st_edge2.edge, st_param_.f_avoid_margin_m, b_move_backward_flag_, 0)) {
                st_edge2.b_use = false;
                st_edge2.edge.clear();
                continue;
            }
        }
        ++n_cnt;

        vec_avoid_path.clear();
        vec_avoid_path.reserve(st_edge0.edge.size() + st_edge1.edge.size() + st_edge2.edge.size());
        vec_avoid_path.insert(vec_avoid_path.end(), st_edge0.edge.begin(), st_edge0.edge.end());
        if (n_idx0 != n_idx1)
            vec_avoid_path.insert(vec_avoid_path.end(), st_edge1.edge.begin(), st_edge1.edge.end());
        vec_avoid_path.insert(vec_avoid_path.end(), st_edge2.edge.begin(), st_edge2.edge.end());

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // extend avoid path
        int n_origin_edge_size = vec_avoid_path.size();
        st_extend_result.b_extend_available = true;
        if (!vec_avoid_path.empty() && st_sp2.f_s_m < f_min_dist) {
            SetFullSm(vec_avoid_path);
            vector<NaviFra::Pos> vec_temp_path_4;
            int n_extend_start_idx = CoreCalculator::FindMinDistanceIdxFromPosVector_(vec_avoid_path.back(), vec_ref_path);
            int n_idx_dist = 0;
            for (int i = n_extend_start_idx; i <= n_force_comeback_target_idx_; i++) {
                float f_plus_dist = vec_ref_path.at(i).GetZm() - vec_ref_path.at(n_extend_start_idx).GetZm();
                n_idx_dist = i - n_extend_start_idx;
                if (f_plus_dist + vec_avoid_path.back().GetZm() > f_min_dist) {
                    break;
                }
            }

            int a = 1;
            while (true) {
                vector<NaviFra::Pos> vec_parted_ref_path;
                if (n_idx_dist == 0) {
                    break;
                }
                int n_extend_dist_idx = n_idx_dist * a;
                if ((n_extend_start_idx + n_extend_dist_idx) >= n_force_comeback_target_idx_) {
                    n_extend_dist_idx = n_force_comeback_target_idx_ - (n_extend_start_idx);
                }
                vec_parted_ref_path.reserve(n_extend_dist_idx + 1);
                vec_parted_ref_path.insert(
                    vec_parted_ref_path.end(), vec_ref_path.begin() + n_extend_start_idx + 1,
                    vec_ref_path.begin() + n_extend_start_idx + 1 + n_extend_dist_idx + 1);

                st_extend_result = MakeRemainPath(vec_parted_ref_path, vec_avoid_sample_point_.at(n_idx2).n_d, st_lane_info);
                vec_temp_path_4 = std::move(st_extend_result.vec_extend_path);

                if (vec_temp_path_4.empty())
                    break;

                float f_ds = hypot(
                    vec_temp_path_4.at(0).GetXm() - vec_avoid_path.back().GetXm(),
                    vec_temp_path_4.at(0).GetYm() - vec_avoid_path.back().GetYm());
                vec_temp_path_4.at(0).SetZm(f_ds + vec_avoid_path.back().GetZm());
                // set S(거리, Zm)
                for (int i = 1; i < (int)vec_temp_path_4.size(); i++) {
                    f_ds = hypot(
                        vec_temp_path_4.at(i).GetXm() - vec_temp_path_4.at(i - 1).GetXm(),
                        vec_temp_path_4.at(i).GetYm() - vec_temp_path_4.at(i - 1).GetYm());
                    vec_temp_path_4.at(i).SetZm(f_ds + vec_temp_path_4.at(i - 1).GetZm());
                }

                if ((!vec_temp_path_4.empty()) &&
                    ((vec_temp_path_4.back().GetZm() > f_min_dist) ||
                     (n_extend_dist_idx == (n_force_comeback_target_idx_ - (n_extend_start_idx))))) {
                    int n_cut_idx = 0;
                    for (; n_cut_idx < (int)vec_temp_path_4.size(); n_cut_idx++) {
                        float f_global_gap = hypot(
                            vec_temp_path_4.at(n_cut_idx).GetXm() - vec_avoid_path.back().GetXm(),
                            vec_temp_path_4.at(n_cut_idx).GetYm() - vec_avoid_path.back().GetYm());
                        if (fabs(f_global_gap) > 0.005) {
                            break;
                        }
                    }

                    if (n_cut_idx != 0) {
                        vec_temp_path_4.erase(vec_temp_path_4.begin(), vec_temp_path_4.begin() + n_cut_idx);
                    }
                    // if (vec_avoid_sample_point_.at(n_idx2).n_d != 0)
                    //     SetAvoidStatus(vec_temp_path_4, true);
                    // else
                    //     SetAvoidStatus(vec_temp_path_4, false);
                    SetAvoidStatus(vec_temp_path_4, true);
                    vec_avoid_path.insert(vec_avoid_path.end(), vec_temp_path_4.begin(), vec_temp_path_4.end());
                    break;
                }
                else {
                    vec_temp_path_4.clear();
                }
                a += 1;
            }
        }
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool b_extend_available = st_extend_result.b_extend_available;
        if (b_extend_available == false) {
            vec_avoid_path.clear();
            continue;
        }
        // SetFullSm(vec_avoid_path);
        SetCurvature(vec_avoid_path);
        for (int i = 0; i < (int)vec_avoid_path.size(); i++) {
            if (fabs(vec_avoid_path.at(i).GetConstDriveInfo().f_curvature) > st_param_.f_polynominal_max_curvature_m) {
                vec_avoid_path.clear();
                break;
            }
        }

        if (vec_avoid_path.empty()) {
            vec_avoid_path.clear();
            continue;
        }

        // 경로 collision check 및 경로 전달
        if (-1 ==
            o_collision_detector_ptr_->CheckPathCollision(
                vec_avoid_path, st_param_.f_avoid_margin_m, b_move_backward_flag_, n_origin_edge_size)) {
            int n_nowCost = CalcCost(vec_avoid_path, vec_avoid_sample_point_.at(n_idx2).n_d);
            if (n_min_cost > n_nowCost) {
                n_min_cost = n_nowCost;
                st_avoid_path_result.vec_avoid_path = vec_avoid_path;
                st_avoid_path_result.n_target_lane_num = vec_avoid_sample_point_.at(n_idx2).n_d;
            }
        }
        if (o_collision_detector_ptr_->CheckPosCollision(vec_avoid_path.back(), st_param_.f_avoid_margin_m, b_move_backward_flag_)) {
            map_n_d_collision.at(vec_avoid_sample_point_.at(n_idx2).n_d) = static_cast<char>(1);
        }
        vec_avoid_path.clear();
    }

    check_sec = std::chrono::steady_clock::now() - time_start;
    LOG_DEBUG("[LaneAvoidPath::CheckAvoidPathTwice] (while-while-for) Count : %d", n_cnt);
    LOG_DEBUG("[LaneAvoidPath::CheckAvoidPathTwice] (while-while-for) Timer : %.3lf", check_sec.count());

    if (st_avoid_path_result.vec_avoid_path.empty()) {
        LOG_WARNING("[LaneAvoidPath::CheckAvoidPathTwice] Finding AvoidPath Failed");
    }
    else {
        LOG_INFO("[LaneAvoidPath::CheckAvoidPathTwice] Finding AvoidPath Success");
    }
    return st_avoid_path_result;
}

std::vector<NaviFra::Pos> LaneAvoidPath::FindComebackPath(
    const std::vector<NaviFra::Pos>& vec_ref_path, const NaviFra::Pos& o_start_pos, Lane_info_t st_lane_info, int n_current_lane_num,
    float f_min_dist)
{
    LOG_DEBUG("[LaneAvoidPath::FindComebackPath] Start Finding ComebackPath");
    vector<NaviFra::Pos> vec_avoid_path;
    Extend_path_result st_extend_result;
    int n_start_ref_idx = CoreCalculator::FindMinDistanceIdxFromPosVector_(o_start_pos, vec_ref_path);

    int check_max_size = min(int(vec_ref_path.size()), n_start_ref_idx + 300);

    std::vector<NaviFra::Pos> sliced_path(vec_ref_path.begin() + n_start_ref_idx, vec_ref_path.begin() + check_max_size - 1);
    int check_collision = o_collision_detector_ptr_->CheckPathCollision(sliced_path, st_param_.f_avoid_margin_m, b_move_backward_flag_, 0);

    if (check_collision != -1)
        return vector<Pos>();

    if(n_force_comeback_target_idx_ > vec_ref_path.size())
    {
        LOG_INFO("n_force_comeback_target_idx_: %d, vec_ref_path size : %d", n_force_comeback_target_idx_, vec_ref_path.size());
        n_force_comeback_target_idx_ = vec_ref_path.size();
    }

    for (int n_dist_step = st_param_.st_lane_avoidance_param.n_search_max_step;
         n_dist_step >= st_param_.st_lane_avoidance_param.n_search_min_step; n_dist_step--) {
        int n_front_dist_idx = 100 * st_param_.st_lane_avoidance_param.f_search_step_size_m * n_dist_step;
        if ((n_start_ref_idx + n_front_dist_idx) > n_force_comeback_target_idx_) {
            LOG_DEBUG("[LaneAvoidPath::FindComebackPath] Go over n_force_comeback_target_idx_");
            vec_avoid_path.clear();
            continue;
        }

        // 다른 lane으로 이동시 두 lane을 잇는 점 포인트
        NaviFra::Pos o_curve_end_pos = vec_ref_path.at(n_start_ref_idx + n_front_dist_idx);
        // o_start_pos 와 o_curve_end_pos 잇기
        vector<NaviFra::Pos> vec_temp_path_1;
        // for (float f_radius_step = st_param_.f_dubins_normal_max_curve_radius_m; f_radius_step >=
        // st_param_.f_dubins_normal_min_curve_radius_m ; f_radius_step -= st_param_.f_dubins_normal_curve_interval_m)
        // {
        //     vec_temp_path_1 = MakeDubinsPath(o_start_pos, o_curve_end_pos, f_radius_step,
        //     vec_ref_path.front().GetConstDriveInfo().f_linear, false); if (vec_temp_path_1.size() != 0)
        //     {
        //         break;
        //     }
        // }

        vec_temp_path_1 = MakePolynominalPath(o_start_pos, o_curve_end_pos);

        if (vec_temp_path_1.size() == 0) {
            LOG_DEBUG("[LaneAvoidPath::FindComebackPath] Polynominal Path Size = 0");
            vec_avoid_path.clear();
            continue;
        }
        SetAvoidStatus(vec_temp_path_1, true);
        vec_avoid_path.insert(vec_avoid_path.end(), vec_temp_path_1.begin(), vec_temp_path_1.end());
        // o_curve_end_pos와 refernce path의 target_idx까지 잇기
        // vector<NaviFra::Pos> vec_parted_ref_path;
        // std::copy(vec_ref_path.begin() + n_start_ref_idx + n_front_dist_idx,
        //             vec_ref_path.begin() + n_target_idx + 1,
        //             std::back_inserter(vec_parted_ref_path));

        // vec_avoid_path.insert(vec_avoid_path.end(), vec_parted_ref_path.begin(), vec_parted_ref_path.end());

        // extend avoid path
        if (vec_ref_path.at(n_start_ref_idx + n_front_dist_idx).GetZm() - vec_ref_path.at(n_start_ref_idx).GetZm() < f_min_dist) {
            vector<NaviFra::Pos> vec_temp_path_2;
            int n_idx_dist = 0;
            for (int i = n_start_ref_idx + n_front_dist_idx; i <= n_force_comeback_target_idx_; i++) {
                float f_plus_dist = vec_ref_path.at(i).GetZm() - vec_ref_path.at(n_start_ref_idx + n_front_dist_idx).GetZm();
                n_idx_dist = i - (n_start_ref_idx + n_front_dist_idx);
                if (f_plus_dist + vec_avoid_path.back().GetZm() > f_min_dist) {
                    break;
                }
            }
            int a = 1;
            while (true) {
                vector<NaviFra::Pos> vec_parted_ref_path;
                if (n_idx_dist == 0) {
                    break;
                }
                int n_extend_dist_idx = n_idx_dist * a;
                if ((n_start_ref_idx + n_front_dist_idx + n_extend_dist_idx) >= n_force_comeback_target_idx_) {
                    n_extend_dist_idx = n_force_comeback_target_idx_ - (n_start_ref_idx + n_front_dist_idx);
                }
                std::copy(
                    vec_ref_path.begin() + n_start_ref_idx + n_front_dist_idx,
                    vec_ref_path.begin() + n_start_ref_idx + n_front_dist_idx + (n_extend_dist_idx) + 1,
                    std::back_inserter(vec_parted_ref_path));
                st_extend_result = MakeRemainPath(vec_parted_ref_path, 0, st_lane_info);
                vec_temp_path_2 = std::move(st_extend_result.vec_extend_path);
                if (vec_temp_path_2.empty())
                    break;

                float f_ds = hypot(
                    vec_temp_path_2.at(0).GetXm() - vec_avoid_path.back().GetXm(),
                    vec_temp_path_2.at(0).GetYm() - vec_avoid_path.back().GetYm());
                vec_temp_path_2.at(0).SetZm(f_ds + vec_avoid_path.back().GetZm());
                // set S(거리, Zm)
                for (int i = 1; i < (int)vec_temp_path_2.size(); i++) {
                    f_ds = hypot(
                        vec_temp_path_2.at(i).GetXm() - vec_temp_path_2.at(i - 1).GetXm(),
                        vec_temp_path_2.at(i).GetYm() - vec_temp_path_2.at(i - 1).GetYm());
                    vec_temp_path_2.at(i).SetZm(f_ds + vec_temp_path_2.at(i - 1).GetZm());
                }

                if (((int)vec_temp_path_2.size() > 0) &&
                    ((vec_temp_path_2.back().GetZm() > f_min_dist) ||
                     (n_extend_dist_idx == (n_force_comeback_target_idx_ - (n_start_ref_idx + n_front_dist_idx))))) {
                    int n_cut_idx = 0;
                    for (; n_cut_idx < (int)vec_temp_path_2.size(); n_cut_idx++) {
                        Pos o_global_gap =
                            Pos(vec_temp_path_2.at(n_cut_idx).GetXm() - vec_avoid_path.back().GetXm(),
                                vec_temp_path_2.at(n_cut_idx).GetYm() - vec_avoid_path.back().GetYm(), 0);
                        Pos o_local_gap = CoreCalculator::TransformRotationRad_(o_global_gap, -vec_avoid_path.back().GetRad());
                        if (o_local_gap.GetXm() > 0.005) {
                            break;
                        }
                    }

                    if (n_cut_idx != 0) {
                        vec_temp_path_2.erase(vec_temp_path_2.begin(), vec_temp_path_2.begin() + n_cut_idx);
                    }

                    SetAvoidStatus(vec_temp_path_2, true);
                    vec_avoid_path.insert(vec_avoid_path.end(), vec_temp_path_2.begin(), vec_temp_path_2.end());
                    break;
                }
                else {
                    vec_temp_path_2.clear();
                }
                a += 1;
            }
        }
        // 경로 collision check 및 경로 전달
        // return vec_avoid_path;
        int n_collision_idx =
            o_collision_detector_ptr_->CheckPathCollision(vec_avoid_path, st_param_.f_avoid_margin_m, b_move_backward_flag_, 0);
        if (n_collision_idx == -1 && vec_avoid_path.size() > 50) {
            LOG_INFO("[LaneAvoidPath::FindComebackPath] Finding ComebackPath Success");
            return vec_avoid_path;
        }
        //--------------------------------

        vec_avoid_path.clear();
    }

    // LOG_INFO("[LaneAvoidPath::FindComebackPath] Finding ComebackPath Failed");
    return vector<Pos>();
}

std::vector<NaviFra::Pos> LaneAvoidPath::ForceComebackPath(
    const std::vector<NaviFra::Pos>& vec_ref_path, const NaviFra::Pos& o_start_pos, const NaviFra::Pos& o_end_pos, Lane_info_t st_lane_info,
    int n_current_lane_num, float f_min_dist)
{
    LOG_DEBUG("[LaneAvoidPath::ForceComebackPath] Force Comeback Path Start");
    LOG_INFO(
        "[LaneAvoidPath::ForceComebackPath] start: %.3f, %.3f, %.3f, end: %.3f, %.3f, %.3f", o_start_pos.GetXm(), o_start_pos.GetYm(),
        o_start_pos.GetDeg(), o_end_pos.GetXm(), o_end_pos.GetYm(), o_end_pos.GetDeg());
    Avoid_path_result st_avoid_path_result;
    int n_start_ref_idx = CoreCalculator::FindMinDistanceIdxFromPosVector_(o_start_pos, vec_ref_path);
    int n_end_ref_idx = CoreCalculator::FindMinDistanceIdxFromPosVector_(o_end_pos, vec_ref_path);

    vector<NaviFra::Pos> vec_avoid_path;

    NaviFra::Pos o_curve_end_pos = o_end_pos;

    LOG_INFO("[LaneAvoidPath::ForceComebackPath] n_force_comeback_target_idx :  %d", n_force_comeback_target_idx_);
    // 다른 lane으로 이동시 두 lane을 잇는 점 포인트
    if (n_start_ref_idx >= n_force_comeback_target_idx_) {
        LOG_WARNING("[LaneAvoidPath::ForceComebackPath] start pos get over n_force_comeback_target_idx");
    }

    int n_dist_step = 1;
    while (st_param_.st_lane_avoidance_param.f_search_step_size_m > 0) {
        int n_front_dist_idx = 100 * st_param_.st_lane_avoidance_param.f_search_step_size_m * n_dist_step;
        int n_end_idx = n_start_ref_idx + n_front_dist_idx;
        if (n_front_dist_idx > 200) {
            if (n_start_ref_idx + n_front_dist_idx >= n_force_comeback_target_idx_) {
                n_end_idx = n_force_comeback_target_idx_;
                o_curve_end_pos = vec_ref_path.at(n_end_idx);
            }
            else {
                o_curve_end_pos = vec_ref_path.at(n_end_idx);
            }

            // o_start_pos 와 o_curve_end_pos 잇기
            vector<NaviFra::Pos> vec_temp_path_1;
            // for (float f_radius_step = st_param_.f_dubins_force_max_curve_radius_m; f_radius_step >=
            // st_param_.f_dubins_force_min_curve_radius_m ; f_radius_step -= st_param_.f_dubins_force_curve_interval_m)
            // {
            //     vec_temp_path_1 = MakeDubinsPath(o_start_pos, o_curve_end_pos, f_radius_step,
            //     vec_ref_path.front().GetConstDriveInfo().f_linear, true); if (vec_temp_path_1.size() != 0)
            //     {
            //         break;
            //     }
            // }
            vec_temp_path_1 = MakePolynominalPath(o_start_pos, o_curve_end_pos);
            SetAvoidStatus(vec_temp_path_1, true);

            if (n_end_idx == n_force_comeback_target_idx_) {
                vec_avoid_path.insert(vec_avoid_path.end(), vec_temp_path_1.begin(), vec_temp_path_1.end());
                break;
            }

            if ((int)vec_temp_path_1.size() > 0) {
                vec_avoid_path.insert(vec_avoid_path.end(), vec_temp_path_1.begin(), vec_temp_path_1.end());
                break;
            }
        }
        n_dist_step++;
    }

    if ((int)vec_avoid_path.size() > 0) {
        LOG_INFO("[LaneAvoidPath::ForceComebackPath] Finding ForceComebackPath Success");
    }
    else {
        LOG_INFO("[LaneAvoidPath::ForceComebackPath] Finding ForceComebackPath Failed.. Generate Straight path");
        vec_avoid_path = CoreCalculator::GetRayPosToTarget_(o_start_pos, o_end_pos, 0.01);
    }

    return vec_avoid_path;
}

std::vector<NaviFra::Pos> LaneAvoidPath::ForceFindPath(
    const std::vector<NaviFra::Pos>& vec_ref_path, const NaviFra::Pos& o_start_pos, const NaviFra::Pos& o_end_pos, Lane_info_t st_lane_info,
    int n_current_lane_num, float f_min_dist)
{
    LOG_DEBUG("[LaneAvoidPath::ForceFindPath] Force Find Path Start");
    LOG_INFO(
        "[LaneAvoidPath::ForceFindPath] start: %.3f, %.3f, %.3f, end: %.3f, %.3f, %.3f", o_start_pos.GetXm(), o_start_pos.GetYm(),
        o_start_pos.GetDeg(), o_end_pos.GetXm(), o_end_pos.GetYm(), o_end_pos.GetDeg());
    Avoid_path_result st_avoid_path_result;
    int n_start_ref_idx = CoreCalculator::FindMinDistanceIdxFromPosVector_(o_start_pos, vec_ref_path);
    int n_end_ref_idx = CoreCalculator::FindMinDistanceIdxFromPosVector_(o_end_pos, vec_ref_path);

    vector<NaviFra::Pos> vec_avoid_path;

    NaviFra::Pos o_curve_end_pos = o_end_pos;

    LOG_INFO("[LaneAvoidPath::ForceFindPath] n_force_comeback_target_idx :  %d", n_force_comeback_target_idx_);
    // 다른 lane으로 이동시 두 lane을 잇는 점 포인트
    if (n_start_ref_idx >= n_force_comeback_target_idx_) {
        LOG_WARNING("[LaneAvoidPath::ForceFindPath] start pos get over n_force_comeback_target_idx");
    }

    if (n_end_ref_idx > n_force_comeback_target_idx_) {
        o_curve_end_pos = vec_ref_path.at(n_force_comeback_target_idx_);
        LOG_WARNING(
            "[LaneAvoidPath::ForceFindPath] target pos get over n_force_comeback_target_idx -> change target pos to force comeback target pos");
    }

    // o_start_pos 와 o_curve_end_pos 잇기
    vector<NaviFra::Pos> vec_temp_path_1;
    // for (float f_radius_step = st_param_.f_dubins_force_max_curve_radius_m; f_radius_step >= st_param_.f_dubins_force_min_curve_radius_m
    // ; f_radius_step -= st_param_.f_dubins_force_curve_interval_m)
    // {
    //     vec_temp_path_1 = MakeDubinsPath(o_start_pos, o_curve_end_pos, f_radius_step, vec_ref_path.front().GetConstDriveInfo().f_linear,
    //     true); if (vec_temp_path_1.size() != 0)
    //     {
    //         break;
    //     }
    // }

    vec_temp_path_1 = MakePolynominalPath(o_start_pos, o_curve_end_pos);

    if (vec_temp_path_1.empty()) {
        LOG_INFO("[LaneAvoidPath::ForceFindPath] Finding ForceFindPath Failed");
        return vector<Pos>();
    }

    SetAvoidStatus(vec_temp_path_1, true);
    vec_avoid_path.insert(vec_avoid_path.end(), vec_temp_path_1.begin(), vec_temp_path_1.end());

    LOG_INFO("[LaneAvoidPath::ForceFindPath] Finding ForceFindPath Success");
    return vec_avoid_path;
}

Avoid_path_result LaneAvoidPath::ExtendAvoidPath(
    const std::vector<NaviFra::Pos>& vec_ref_path, const std::vector<NaviFra::Pos>& vec_current_path, int n_current_path_idx,
    Lane_info_t st_lane_info, int n_current_lane_num, float f_min_dist)
{
    LOG_DEBUG("[LaneAvoidPath::ExtendAvoidPath] Extend avoid path start");
    vector<NaviFra::Pos> vec_avoid_path;
    vector<NaviFra::Pos> vec_parted_ref_path;
    Avoid_path_result st_avoid_path_result;
    Extend_path_result st_extend_result;

    float f_dist = vec_current_path.back().GetZm() - vec_current_path[n_current_path_idx].GetZm();
    float remain_dist = f_min_dist - f_dist;
    if (remain_dist <= 0) {
        LOG_DEBUG("[LaneAvoidPath::ExtendAvoidPath] ExtendAvoidPath remain_dist <= 0");
        st_avoid_path_result.n_target_lane_num = n_current_lane_num;
        return st_avoid_path_result;
    }
    int n_remain_start_idx = CoreCalculator::FindMinDistanceIdxFromPosVector_(vec_current_path.back(), vec_ref_path);
    LOG_DEBUG(
        "[LaneAvoidPath::ExtendAvoidPath] Extend start ref path index : %d  x :  %f y :  %f", n_remain_start_idx,
        vec_ref_path.at(n_remain_start_idx).GetXm(), vec_ref_path.at(n_remain_start_idx).GetYm());
    // std::copy(vec_ref_path.begin() + n_end_avoid_idx, vec_ref_path.begin() + n_target_idx + 1, std::back_inserter(vec_tmp_path));

    int n_idx_dist = 0;
    for (int i = n_remain_start_idx; i <= n_force_comeback_target_idx_; i++) {
        float f_plus_dist = vec_ref_path.at(i).GetZm() - vec_ref_path.at(n_remain_start_idx).GetZm();
        n_idx_dist = i - (n_remain_start_idx);
        if (f_plus_dist > remain_dist) {
            break;
        }
    }

    int a = 1;
    while (true) {
        vector<NaviFra::Pos> vec_parted_ref_path;
        if (n_idx_dist == 0) {
            break;
        }
        int n_extend_dist_idx = n_idx_dist * a;
        if ((n_remain_start_idx + n_extend_dist_idx) >= n_force_comeback_target_idx_) {
            n_extend_dist_idx = n_force_comeback_target_idx_ - n_remain_start_idx;
        }
        std::copy(
            vec_ref_path.begin() + n_remain_start_idx, vec_ref_path.begin() + n_remain_start_idx + (n_extend_dist_idx) + 1,
            std::back_inserter(vec_parted_ref_path));

        st_extend_result = MakeRemainPath(vec_parted_ref_path, n_current_lane_num, st_lane_info);
        vec_avoid_path = std::move(st_extend_result.vec_extend_path);
        LOG_DEBUG("[LaneAvoidPath::ExtendAvoidPath] st_extend_result.b_extend_available : %d", st_extend_result.b_extend_available);
        LOG_DEBUG("[LaneAvoidPath::ExtendAvoidPath] extend vec_avoid_path : %d", (int)vec_avoid_path.size());
        if (vec_avoid_path.empty())
            break;
        if (st_extend_result.b_extend_available == false)
            break;

        SetFullSm(vec_avoid_path);

        if (((int)vec_avoid_path.size() > 0) &&
            ((vec_avoid_path.back().GetZm() > remain_dist) || (n_extend_dist_idx == (n_force_comeback_target_idx_ - n_remain_start_idx)))) {
            int n_cut_idx = 0;
            for (; n_cut_idx < (int)vec_avoid_path.size(); n_cut_idx++) {
                // Pos o_global_gap = Pos(vec_avoid_path.at(n_cut_idx).GetXm() - vec_current_path.back().GetXm(),
                // vec_avoid_path.at(n_cut_idx).GetYm() - vec_current_path.back().GetYm(), 0); Pos o_local_gap =
                // CoreCalculator::TransformRotationRad_(o_global_gap, -vec_current_path.back().GetRad());
                float f_global_gap = hypot(
                    vec_avoid_path.at(n_cut_idx).GetXm() - vec_current_path.back().GetXm(),
                    vec_avoid_path.at(n_cut_idx).GetYm() - vec_current_path.back().GetYm());
                if (fabs(f_global_gap) > 0.005) {
                    break;
                }
            }

            if (n_cut_idx != 0) {
                vec_avoid_path.erase(vec_avoid_path.begin(), vec_avoid_path.begin() + n_cut_idx);
            }
            break;
        }
        else {
            vec_avoid_path.clear();
        }
        a += 1;
    }

    // if (n_current_lane_num != 0)
    //     SetAvoidStatus(vec_avoid_path, true);
    // else
    //     SetAvoidStatus(vec_avoid_path, false);

    SetAvoidStatus(vec_avoid_path, true);

    bool b_extend_available = st_extend_result.b_extend_available;
    // check_curvature
    vector<NaviFra::Pos> vec_check_curvature_path;
    vec_check_curvature_path = vec_current_path;
    vec_check_curvature_path.insert(vec_check_curvature_path.end(), vec_avoid_path.begin(), vec_avoid_path.end());
    SetFullSm(vec_check_curvature_path);
    SetCurvature(vec_check_curvature_path);
    for (int i = 0; i < (int)vec_check_curvature_path.size(); i++)

    {
        if (fabs(vec_check_curvature_path.at(i).GetConstDriveInfo().f_curvature) > st_param_.f_polynominal_max_curvature_m) {
            vec_avoid_path.clear();
            LOG_INFO(
                "[LaneAvoidPath::ExtendAvoidPath] b_extend_available point index : %d f_curvature : %f param : %f ", i,
                vec_check_curvature_path.at(i).GetConstDriveInfo().f_curvature, st_param_.f_polynominal_max_curvature_m);
            b_extend_available = false;
            break;
        }
    }

    if (b_extend_available == false) {
        LOG_INFO("[LaneAvoidPath::ExtendAvoidPath] extend force_comback");
        st_avoid_path_result.vec_avoid_path = ForceComebackPath(
            vec_ref_path, vec_current_path.at(n_current_path_idx), vec_ref_path.at(n_force_comeback_target_idx_), st_lane_info,
            n_current_lane_num, 0);
        st_avoid_path_result.n_target_lane_num = 0;
    }
    else {
        st_avoid_path_result.vec_avoid_path = vec_avoid_path;
        st_avoid_path_result.n_target_lane_num = n_current_lane_num;
    }
    // vec_tmp_path = MakeRemainPath(vec_parted_ref_path, n_current_lane_num, st_lane_info);
    // vec_avoid_path.insert(vec_avoid_path.end(), vec_tmp_path.begin(), vec_tmp_path.end());

    // st_avoid_path_result.vec_avoid_path = vec_avoid_path;
    // st_avoid_path_result.n_target_lane_num = n_current_lane_num;

    return st_avoid_path_result;
}

void LaneAvoidPath::SetCurvature(vector<Pos>& vec_path)
{
    for (int i = 0; i < (int)vec_path.size() - 1; ++i) {
        float ds = vec_path.at(i + 1).GetZm() - vec_path.at(i).GetZm();
        if (ds < 0.00001)
            continue;
        float da = CoreCalculator::CalcAngleDomainRad_(vec_path.at(i + 1).GetRad() - vec_path.at(i).GetRad());
        vec_path.at(i).GetDriveInfo().f_curvature = da / ds;
    }
    vec_path.back().GetDriveInfo().f_curvature = vec_path.at(vec_path.size() - 2).GetDriveInfo().f_curvature;
}

void LaneAvoidPath::SetFullSm(vector<Pos>& vec_path)
{
    vec_path.at(0).SetZm(0.f);
    for (int i = 1; i < (int)vec_path.size(); ++i) {
        float dx = vec_path.at(i).GetXm() - vec_path.at(i - 1).GetXm();
        float dy = vec_path.at(i).GetYm() - vec_path.at(i - 1).GetYm();
        vec_path.at(i).SetZm(vec_path.at(i - 1).GetZm() + std::sqrt(dx * dx + dy * dy));
    }
}

void LaneAvoidPath::SetAvoidStatus(vector<Pos>& vec_path, bool b_state)
{
    if (b_state == true) {
        for (int i = 1; i < (int)vec_path.size(); ++i) {
            vec_path.at(i).GetDriveInfo().b_avoid_status = true;
        }
    }
    else {
        for (int i = 1; i < (int)vec_path.size(); ++i) {
            vec_path.at(i).GetDriveInfo().b_avoid_status = false;
        }
    }
}

int LaneAvoidPath::CalcCost(const std::vector<NaviFra::Pos>& path, int n_lane_num)
{
    double d_cost_k = 0;
    for (const NaviFra::Pos& wp : path)
        d_cost_k += wp.GetConstDriveInfo().f_curvature * wp.GetConstDriveInfo().f_curvature;
    int n_cost_k = static_cast<int>(d_cost_k);
    int n_cost_m = o_collision_detector_ptr_->CalcCost(path, st_param_.f_avoid_margin_m, false) / 10;
    int n_cost_d = n_lane_num * n_lane_num;
    int n_totalCost = n_cost_k + n_cost_m + n_cost_d;
    LOG_DEBUG("[LaneAvoidPath::CalcCost] totalCost=%d, cost(k)=%d, cost(m)=%d, cost(d)=%d", n_totalCost, n_cost_k, n_cost_m, n_cost_d);
    return n_totalCost;
}

}  // namespace NaviFra
