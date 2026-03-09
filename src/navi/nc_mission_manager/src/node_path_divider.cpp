#include "node_path_divider.hpp"

#include "core/util/logger.hpp"

namespace NaviFra {
NodePathDivider::NodePathDivider()
{
}

NodePathDivider::~NodePathDivider()
{
}

std::vector<NaviFra::Pos> NodePathDivider::MakeDigonalPath(std::vector<NaviFra::Pos>& vec_path, Parameters_t& st_parameter)
{
    std::vector<Pos> vec_diagonal;
    // normal + diagonal path, make polyfit smooth path for diagonal motion.
    vec_diagonal.emplace_back(vec_path.at(0));

    for (int i = 1; i < vec_path.size(); i++) {
        vec_diagonal.emplace_back(vec_path.at(i));

        if (vec_path.at(i).GetConstDriveInfo().e_drive_type == Pos::DRIVE_TYPE::FRONT_DIAGONAL ||
            vec_path.at(i).GetConstDriveInfo().e_drive_type == Pos::DRIVE_TYPE::REAR_DIAGONAL) {
            vec_diagonal.back().SetRad(vec_diagonal.at(vec_diagonal.size() - 2).GetRad());
        }

        if (vec_path.at(i).GetConstDriveInfo().e_drive_type == vec_path.at(i - 1).GetConstDriveInfo().e_drive_type) {
            continue;
        }

        if (i > 10 && i < vec_path.size() - 10) {
            float f_rad1 = atan2(vec_path.at(i + 5).GetYm() - vec_path.at(i).GetYm(), vec_path.at(i + 5).GetXm() - vec_path.at(i).GetXm());
            float f_rad2 = atan2(vec_path.at(i).GetYm() - vec_path.at(i - 5).GetYm(), vec_path.at(i).GetXm() - vec_path.at(i - 5).GetXm());

            float f_diff_angel = CoreCalculator::WrapAnglePiToPiDeg_((f_rad1 - f_rad2) * RADtoDEG);
            NLOG(info) << "f_diff_angel " << f_diff_angel;
            if (fabs(f_diff_angel) < 1)  // 각도가 1도이하 꺾여있으면 스킵
            {
                continue;
            }
        }

        string front_nownode_name = vec_path.at(i).GetNowNodeName();
        string front_nownode_id = vec_path.at(i).GetNowNodeID();
        string front_nextnode_name = vec_path.at(i).GetNextNodeName();
        string front_nextnode_id = vec_path.at(i).GetNextNodeID();
        string rear_nownode_name;
        string rear_nownode_id;
        string rear_nextnode_name;
        string rear_nextnode_id;
        for (int j = i; j > 0; j--) {
            // if(front_nownode_name != vec_path.at(j).GetNowNodeName())
            if (front_nownode_id != vec_path.at(j).GetNowNodeID()) {
                rear_nownode_name = vec_path.at(j).GetNowNodeName();
                rear_nownode_id = vec_path.at(j).GetNowNodeID();
                rear_nextnode_name = vec_path.at(j).GetNextNodeName();
                rear_nextnode_id = vec_path.at(j).GetNextNodeID();
                break;
            }
        }
        LOG_INFO("Diagonal Path node %s->%s", front_nownode_name.c_str(), rear_nownode_name.c_str());

        float dist_checking;
        Pos::DriveInfo_t driveinfo;

        bool b_end_diagonal = false;
        float f_end_angle = vec_path.at(i).GetRad();
        if (vec_path.at(i).GetConstDriveInfo().e_drive_type == Pos::DRIVE_TYPE::FRONT_DIAGONAL ||
            vec_path.at(i).GetConstDriveInfo().e_drive_type == Pos::DRIVE_TYPE::REAR_DIAGONAL) {
            dist_checking = vec_path.at(i).GetConstDriveInfo().f_diagonal_curve;
            driveinfo = vec_path.at(i).GetConstDriveInfo();
        }
        if (vec_path.at(i - 1).GetConstDriveInfo().e_drive_type == Pos::DRIVE_TYPE::FRONT_DIAGONAL ||
            vec_path.at(i - 1).GetConstDriveInfo().e_drive_type == Pos::DRIVE_TYPE::REAR_DIAGONAL) {
            dist_checking = vec_path.at(i - 1).GetConstDriveInfo().f_diagonal_curve;
            driveinfo = vec_path.at(i - 1).GetConstDriveInfo();
            b_end_diagonal = true;
        }

        // make arc path for diagonal motion
        int idx_forward = CoreCalculator::FindIndexAtDistance_(vec_path, i, dist_checking);
        int idx_backword = CoreCalculator::FindIndexAtDistance_(vec_path, i, dist_checking, true);
        int idx = min(idx_forward - i, i - idx_backword);
        vec_diagonal.erase(vec_diagonal.end() - idx, vec_diagonal.end());
        Pos s_pos = vec_path.at(i - idx);
        Pos e_pos = vec_path.at(i + idx);

        float f_dist_pos = hypot(s_pos.GetXm() - e_pos.GetXm(), s_pos.GetYm() - e_pos.GetYm());
        float f_radius_circle = f_dist_pos / 2 / sin(abs(s_pos.GetRad() - e_pos.GetRad()) / 2);

        float fd2 = f_dist_pos / 2;
        float fOffset = sqrt(f_radius_circle * f_radius_circle - fd2 * fd2);
        float fplusx = fOffset * (e_pos.GetYm() - s_pos.GetYm()) / f_dist_pos;
        float fplusy = fOffset * (e_pos.GetXm() - s_pos.GetXm()) / f_dist_pos;
        Pos center_pos;
        Pos center_pos_1;
        center_pos_1.SetXm((s_pos.GetXm() + e_pos.GetXm()) / 2 - fplusx);
        center_pos_1.SetYm((s_pos.GetYm() + e_pos.GetYm()) / 2 + fplusy);
        Pos center_pos_2;
        center_pos_2.SetXm((s_pos.GetXm() + e_pos.GetXm()) / 2 + fplusx);
        center_pos_2.SetYm((s_pos.GetYm() + e_pos.GetYm()) / 2 - fplusy);

        Pos dir_s(cos(s_pos.GetRad()), sin(s_pos.GetRad()));
        Pos dir_v1(center_pos_1.GetXm() - s_pos.GetXm(), center_pos_1.GetYm() - s_pos.GetYm());
        Pos dir_v2(center_pos_2.GetXm() - s_pos.GetXm(), center_pos_2.GetYm() - s_pos.GetYm());
        // dot product for find center of circle
        bool b_clock_wise_dir = false;
        if (fabs(CoreCalculator::CalcPosDotProduct_(dir_s, dir_v1)) < 0.1) {
            center_pos = center_pos_1;
            b_clock_wise_dir = false;
        }
        else if (fabs(CoreCalculator::CalcPosDotProduct_(dir_s, dir_v2)) < 0.1) {
            center_pos = center_pos_2;
            b_clock_wise_dir = true;
        }

        float f_deg_error = s_pos.GetDeg() - e_pos.GetDeg();
        if (f_deg_error > 180)
            f_deg_error -= 360;
        else if (f_deg_error < -180)
            f_deg_error += 360;

        float f_arc_length = fabs(f_deg_error) * M_PI * f_radius_circle / 180;
        int N = (int)(f_arc_length / 0.01);
        int n_nownode_std = int(N / 2 - st_parameter.f_current_node_change_before_m * 100);
        if (n_nownode_std < 1)
            n_nownode_std = 1;

        float f_x = s_pos.GetXm() - center_pos.GetXm();
        float f_y = s_pos.GetYm() - center_pos.GetYm();
        float f_a = vec_diagonal.back().GetRad();

        // 사행 곡률에 따른 속도 적용
        float f_diagonal_speed = driveinfo.f_linear;
        // 사행 곡률에 따른 속도 적용
        driveinfo.f_linear = dist_checking / 2;
        LOG_INFO("1driveinfo %.2f", driveinfo.f_linear);

        driveinfo.f_linear = driveinfo.f_linear * st_parameter.f_diagonal_curve_speed_ratio;
        LOG_INFO("3driveinfo %.2f", driveinfo.f_linear);

        if (driveinfo.f_linear < 0.05)
            driveinfo.f_linear = 0.05;
        LOG_INFO("4driveinfo %.2f", driveinfo.f_linear);

        if (driveinfo.f_linear > f_diagonal_speed)
            driveinfo.f_linear = f_diagonal_speed;
        LOG_INFO("5driveinfo %.2f", driveinfo.f_linear);

        // make arc path point
        for (int j = 1; j < N; j++) {
            Pos tmp(f_x, f_y);
            if (b_clock_wise_dir == true) {
                tmp = CoreCalculator::TransformRotationDeg_(tmp, -0.01 * 180 / M_PI / f_radius_circle * j);
            }
            else {
                tmp = CoreCalculator::TransformRotationDeg_(tmp, 0.01 * 180 / M_PI / f_radius_circle * j);
            }
            tmp.SetXm(tmp.GetXm() + center_pos.GetXm());
            tmp.SetYm(tmp.GetYm() + center_pos.GetYm());
            if (b_end_diagonal) {
                tmp.SetRad(CoreCalculator::WrapAnglePiToPiRad_(f_end_angle - f_a) * (j) / (N - 1) + f_a);
            }
            else {
                tmp.SetRad(f_a);
            }
            tmp.SetDriveInfo(driveinfo);
            if (j > n_nownode_std) {
                tmp.SetNowNodeName(front_nownode_name);
                tmp.SetNowNodeID(front_nownode_id);
                tmp.SetNextNodeName(front_nextnode_name);
                tmp.SetNextNodeID(front_nextnode_id);
            }
            else {
                tmp.SetNowNodeName(rear_nownode_name);
                tmp.SetNowNodeID(rear_nownode_id);
                tmp.SetNextNodeName(rear_nextnode_name);
                tmp.SetNextNodeID(rear_nextnode_id);
            }
            vec_diagonal.emplace_back(tmp);
        }

        i += idx;
    }

    return vec_diagonal;
}

std::vector<NaviFra::Pos> NodePathDivider::PostProcessDigonalPath(const std::vector<NaviFra::Pos>& vec_path)
{
    std::vector<Pos> vec_diagonal = vec_path;

    float f_max_side_move_deg = 85;  // 도 이상일때 사행 주행시 로봇의 각을 조절해주어 steer 가용범위 유지한다.
    for (int i = 1; i < vec_diagonal.size(); i++) {
        if (vec_diagonal.at(i).GetConstDriveInfo().e_drive_type == Pos::DRIVE_TYPE::FRONT_DIAGONAL ||
            vec_diagonal.at(i).GetConstDriveInfo().e_drive_type == Pos::DRIVE_TYPE::REAR_DIAGONAL) {
            float f_vec_rad = atan2(
                vec_diagonal.at(i).GetYm() - vec_diagonal.at(i - 1).GetYm(), vec_diagonal.at(i).GetXm() - vec_diagonal.at(i - 1).GetXm());
            if (f_vec_rad < 0)
                f_vec_rad += 2 * M_PI;
            float f_path_rad = vec_diagonal.at(i).GetRad();
            if (f_path_rad < 0)
                f_path_rad += 2 * M_PI;
            float f_gap_rad = abs(f_vec_rad - f_path_rad);
            if (f_gap_rad > M_PI)
                f_gap_rad -= 2 * M_PI;
            f_gap_rad = abs(f_gap_rad);

            if (f_gap_rad * RADtoDEG > f_max_side_move_deg) {
                float f_direction = vec_diagonal.at(i).GetRad() - f_vec_rad;
                if (f_direction > M_PI)
                    f_direction -= 2 * M_PI;
                if (f_direction < -M_PI)
                    f_direction += 2 * M_PI;

                float f_correction_deg = f_gap_rad * RADtoDEG - f_max_side_move_deg;
                if (f_direction >= 0 && f_direction < M_PI) {
                    vec_diagonal.at(i).SetDeg(vec_diagonal.at(i).GetDeg() - f_correction_deg);
                }
                else {
                    vec_diagonal.at(i).SetDeg(vec_diagonal.at(i).GetDeg() + f_correction_deg);
                }
            }
        }
    }

    return vec_diagonal;
}

std::vector<std::vector<Pos>> NodePathDivider::DevidePath(const std::vector<Pos>& vec_path, float f_angle_dividing_the_path_deg)
{
    if (vec_path.size() <= 1)
        return std::vector<std::vector<Pos>>({vec_path});

    std::vector<std::vector<Pos>> result_path_stack;
    int n_size = vec_path.size();
    std::vector<Pos> tmp_path_stack;
    tmp_path_stack.emplace_back(vec_path.at(0));
    for (int i = 1; i < n_size - 1; i++) {
        tmp_path_stack.emplace_back(vec_path.at(i));
        float f_diff_angel = CoreCalculator::WrapAnglePiToPiDeg_(vec_path.at(i).GetDeg() - vec_path.at(i - 1).GetDeg());
        bool b_angle = (fabs(f_diff_angel) > f_angle_dividing_the_path_deg);  // 각도가 10도이상 비선형적으로 꺾일 때
        bool b_direct = false;  // 로봇의 진행방향이 바뀔 때
        bool b_digoanl = false;  // 사행 모션과 비사행 모션으로 바뀔 때

        if (vec_path.at(i).GetConstDriveInfo().e_drive_type != vec_path.at(i - 1).GetConstDriveInfo().e_drive_type) {
            if (vec_path.at(i).GetConstDriveInfo().e_drive_type == Pos::DRIVE_TYPE::FRONT_DIAGONAL &&
                vec_path.at(i - 1).GetConstDriveInfo().e_drive_type == Pos::DRIVE_TYPE::FRONT) {
                b_digoanl = true;
                if (vec_path.at(i).GetConstDriveInfo().f_diagonal_curve >= 0.1)
                    b_direct = false;
                else
                    b_direct = true;
            }
            else if (
                vec_path.at(i).GetConstDriveInfo().e_drive_type == Pos::DRIVE_TYPE::FRONT &&
                vec_path.at(i - 1).GetConstDriveInfo().e_drive_type == Pos::DRIVE_TYPE::FRONT_DIAGONAL) {
                b_digoanl = true;
                if (vec_path.at(i - 1).GetConstDriveInfo().f_diagonal_curve >= 0.1)
                    b_direct = false;
                else
                    b_direct = true;
            }
            else if (
                vec_path.at(i).GetConstDriveInfo().e_drive_type == Pos::DRIVE_TYPE::REAR_DIAGONAL &&
                vec_path.at(i - 1).GetConstDriveInfo().e_drive_type == Pos::DRIVE_TYPE::REAR) {
                b_digoanl = true;
                if (vec_path.at(i).GetConstDriveInfo().f_diagonal_curve >= 0.1)
                    b_direct = false;
                else
                    b_direct = true;
            }
            else if (
                vec_path.at(i).GetConstDriveInfo().e_drive_type == Pos::DRIVE_TYPE::REAR &&
                vec_path.at(i - 1).GetConstDriveInfo().e_drive_type == Pos::DRIVE_TYPE::REAR_DIAGONAL) {
                b_digoanl = true;
                if (vec_path.at(i - 1).GetConstDriveInfo().f_diagonal_curve >= 0.1)
                    b_direct = false;
                else
                    b_direct = true;
            }
            else {
                b_direct = true;
            }
        }
        // LOG_INFO("%d %.2f", i, f_diff_angel);
        if (b_angle == true) {
            LOG_INFO(
                "Path divided %s->%s %.2f", vec_path.at(i).GetNowNodeName().c_str(), vec_path.at(i).GetNextNodeName().c_str(),
                f_diff_angel);
        }
        if ((b_direct || (b_angle && b_digoanl && b_direct) || (b_angle && !b_digoanl)) &&
            i > 10)  // divide condition (direction not same, )
        {
            NLOG(info) << b_angle << " " << b_direct << " / " << vec_path.at(i).GetConstDriveInfo().e_drive_type << " "
                       << vec_path.at(i - 1).GetConstDriveInfo().e_drive_type << " / "
                       << fabs(CoreCalculator::WrapAnglePiToPiDeg_(vec_path.at(i).GetDeg() - vec_path.at(i - 1).GetDeg()));
            tmp_path_stack.pop_back();
            result_path_stack.push_back(tmp_path_stack);
            tmp_path_stack.clear();
            tmp_path_stack.emplace_back(vec_path.at(i));
        }
        else if ((b_direct || (b_angle && b_digoanl && b_direct) || (b_angle && !b_digoanl)) && i < 10) {
            tmp_path_stack.clear();
            tmp_path_stack.emplace_back(vec_path.at(i));
        }
    }
    tmp_path_stack.emplace_back(vec_path.back());
    result_path_stack.push_back(tmp_path_stack);
    return result_path_stack;
}

void NodePathDivider::ApplyDividedPath(
    std::vector<std::vector<NaviFra::Pos>>&& path_stack, float yaw_bias, const NaviFra::Pos& o_robot_Pos,
    std::vector<PathDescription>& vec_out_path_desc_stack, Parameters_t& st_parameter)
{
    int vec_path_desc_stack_size = vec_out_path_desc_stack.size();
    int vec_path_desc_stack_start_idx = std::max(vec_path_desc_stack_size - 1, 0);
    if (vec_path_desc_stack_size != 0) {
        if (vec_out_path_desc_stack.back().vec_path.size() == 1)  // just spin turn
            vec_path_desc_stack_start_idx++;
    }
    vec_out_path_desc_stack.resize(vec_path_desc_stack_start_idx + path_stack.size());
    for (int i = 0; i < path_stack.size(); i++) {
        int idx_thistime = vec_path_desc_stack_start_idx + i;
        NaviFra::PathDescription& path_desc = vec_out_path_desc_stack.at(idx_thistime);

        // set path info
        path_desc.vec_path = std::move(path_stack.at(i));
        path_desc.s_start_name = path_desc.vec_path.front().GetNowNodeName();
        path_desc.s_start_node_id = path_desc.vec_path.front().GetNowNodeID();
        NaviFra::Pos goal_pos = path_desc.vec_path.back();
        path_desc.vec_path = MakeDigonalPath(path_desc.vec_path, st_parameter);  // 사행 패쓰는 부드럽게 이어준다.

        // path_desc.vec_path = PostProcessDigonalPath(path_desc.vec_path); // 기구학을 고려해 사행 각도를 후처리한다.

        // set yaw bias
        path_desc.f_yaw_bias = yaw_bias;
        if (int(vec_out_path_desc_stack.at(idx_thistime).vec_path.front().GetConstDriveInfo().e_drive_type) >= 2000) {
            path_desc.f_yaw_bias = CoreCalculator::WrapAnglePiToPiRad_(path_desc.f_yaw_bias + M_PI);
            NLOG(info) << "path apply::=" << path_desc.f_yaw_bias * 180 / 3.14;
        }

        // apply goal
        path_desc.o_goal_pos = goal_pos;
        NLOG(info) << "path_desc.o_goal_pos deg : " << path_desc.o_goal_pos.GetDeg();
        NLOG(info) << "path_desc.o_goal_pos zone type : " << path_desc.o_goal_pos.GetType();
    }
}
}  // namespace NaviFra
