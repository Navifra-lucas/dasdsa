#include "core_calculator.hpp"

#include <cfloat>
namespace NaviFra {

/**
 * @brief x, y, Deg값이 모두 0인경우 True
 */

float CoreCalculator::CalcPosDistance_(const Pos& o_pos)
{
    return hypot(o_pos.GetXm(), o_pos.GetYm());
}

float CoreCalculator::CalcPosDistance_(const Pos& o_pos, const Pos& o_pos2)
{
    return hypot(o_pos.GetXm() - o_pos2.GetXm(), o_pos.GetYm() - o_pos2.GetYm());
}

int CoreCalculator::FindMinDistanceIdxFromPosVector_(const Pos& o_pos, const vector<Pos>& vec_pos)
{
    float f_min_dist_m = std::numeric_limits<float>::max();
    int n_result_index = 0;

    if (vec_pos.empty()) {
        return 0;
    }

    for (int i = 0; i < (int)vec_pos.size(); ++i) {
        float f_x_delta = vec_pos.at(i).GetXm() - o_pos.GetXm();
        float f_y_delta = vec_pos.at(i).GetYm() - o_pos.GetYm();
        float f_distance = hypot(f_x_delta, f_y_delta);
        // if (f_distance > 1.0f)  // 경로 포인트와 로봇 거리가 5m 이상 차이나면 당분간
        //                         // 검사할 의미가 없음..
        // {
        //     i += 80;
        //     if (i >= vec_pos.size()) {
        //         i = vec_pos.size() - 1;
        //     }
        // }

        if (f_min_dist_m > f_distance) {
            f_min_dist_m = f_distance;
            n_result_index = i;
        }
    }
    return n_result_index;
}

int CoreCalculator::FindMinDistanceIdxFromPosVectorFarPoint_(const Pos& o_pos, const vector<Pos>& vec_pos)
{
    float f_min_dist_m = std::numeric_limits<float>::max();
    int n_result_index = 0;

    int n_data_size = vec_pos.size();
    if (n_data_size <= 1) {
        return 0;
    }

    for (int i = 0; i < n_data_size; i++) {
        float f_x_delta = vec_pos.at(i).GetXm() - o_pos.GetXm();
        float f_y_delta = vec_pos.at(i).GetYm() - o_pos.GetYm();
        float f_distance = hypot(f_x_delta, f_y_delta);
        if (f_distance < f_min_dist_m + 0.01 && i - n_result_index > 3)  // Update if the distance is less than 10mm different, otherwise
                                                                         // back indices are searched first by Nate.
        {
            f_min_dist_m = f_distance;
            n_result_index = i;
        }
    }
    return n_result_index;
}

int CoreCalculator::FindIndexAtDistance_(
    const std::vector<Pos>& vec_path_pos, int n_start_idx, float f_apart_length, const bool b_is_reverse)
{
    if (n_start_idx < 0) {
        n_start_idx = 0;
    }
    else if (vec_path_pos.size() <= n_start_idx) {
        n_start_idx = vec_path_pos.size() - 1;
    }

    float f_checked_path_length = 0;
    int n_result_idx = n_start_idx;
    int f_ex_idx = n_start_idx;

    for (n_result_idx = n_start_idx; ((0 <= n_result_idx) && (n_result_idx < vec_path_pos.size()));
         (b_is_reverse ? n_result_idx-- : n_result_idx++)) {
        // 검사 거리 판단
        if (n_result_idx == n_start_idx) {
            f_checked_path_length += CoreCalculator::CalcPosDistance_(vec_path_pos.at(n_start_idx), vec_path_pos.at(n_result_idx));
        }
        else {
            f_checked_path_length += CoreCalculator::CalcPosDistance_(vec_path_pos.at(f_ex_idx), vec_path_pos.at(n_result_idx));
        }
        f_ex_idx = n_result_idx;

        if ((f_apart_length < f_checked_path_length))  // 검사 거리가 limit 거리보다 크다
        {
            break;
        }
    }
    // 인덱스 예외처리
    if (n_result_idx < 0) {
        n_result_idx = 0;
    }
    else if (vec_path_pos.size() <= n_result_idx) {
        n_result_idx = vec_path_pos.size() - 1;
    }

    return n_result_idx;
}

std::vector<Pos> CoreCalculator::GetRayPosToTarget_(const Pos& o_start_pos, const Pos& o_target_pos, float f_gap_dist_m)
{
    const float MIN_DISTANCE_THRESHOLD_M = 0.01;
    vector<Pos> vec_ray_pos;
    if (CalcPosDistance_(o_start_pos, o_target_pos) < MIN_DISTANCE_THRESHOLD_M)
        return vec_ray_pos;

    Pos o_current_pos = o_start_pos;
    Pos o_pos_difference(o_target_pos.GetXm() - o_start_pos.GetXm(), o_target_pos.GetYm() - o_start_pos.GetYm(), 0);
    o_pos_difference.SetRad(atan2(o_pos_difference.GetYm(), o_pos_difference.GetXm()));
    float f_current_distance_m = 0;
    float f_sum_distance_m = 0;
    float f_gradient_x = cos(o_pos_difference.GetRad());
    float f_gradient_y = sin(o_pos_difference.GetRad());
    float f_start_x_m = o_start_pos.GetXm();
    float f_start_y_m = o_start_pos.GetYm();

    o_current_pos.SetRad(o_pos_difference.GetRad());
    vec_ray_pos.emplace_back(o_current_pos);

    float f_target_distance_m = hypot(o_pos_difference.GetXm(), o_pos_difference.GetYm());
    while (f_current_distance_m < f_target_distance_m - f_gap_dist_m) {
        float f_ray_x_m = f_sum_distance_m * f_gradient_x;
        float f_ray_y_m = f_sum_distance_m * f_gradient_y;
        f_current_distance_m = hypot(f_ray_x_m, f_ray_y_m);
        f_sum_distance_m = f_sum_distance_m + f_gap_dist_m;
        Pos o_ray_pos(f_start_x_m + f_ray_x_m, f_start_y_m + f_ray_y_m, o_pos_difference.GetDeg());
        // o_ray_pos.SetDriveInfo(o_current_pos.GetConstDriveInfo());
        vec_ray_pos.emplace_back(o_ray_pos);
    }

    // 마지막에 도착 노드를 제대로 넣어줘야 함. 안그러면 정밀도 이슈 나올수 있음
    // // 네이트
    if (vec_ray_pos.size() > 0)
        vec_ray_pos.back() = o_target_pos;
    return vec_ray_pos;
}

float CoreCalculator::CalcDistanceFromDotToLine_(const Pos& o_line_s, const Pos& o_line_e, const Pos& o_target_pos)
{
    Pos o_line_vec = o_line_e - o_line_s;
    Pos o_sub_vec = o_target_pos - o_line_s;
    float o_line_dist = hypot(o_line_vec.GetXm(), o_line_vec.GetYm());
    if (o_line_dist > 0)
        return fabs((o_line_vec.GetXm() * o_sub_vec.GetYm() - o_line_vec.GetYm() * o_sub_vec.GetXm()) / o_line_dist);
    else
        return 0;
}

Pos CoreCalculator::CalcPosDotOnLine_(const Pos& o_line_s, const Pos& o_line_e, const Pos& o_target_pos)
{
    float f_distance_m = CoreCalculator::CalcPosDistance_(o_line_e, o_line_s);
    if (f_distance_m < FLT_EPSILON)
        return Pos(0, 0, 0);

    Pos o_unit_vector = (o_line_e - o_line_s) / f_distance_m;
    Pos o_diff_pos = o_target_pos - o_line_s;
    Pos o_add_pos = o_unit_vector * (o_unit_vector.GetXm() * o_diff_pos.GetXm() + o_unit_vector.GetYm() * o_diff_pos.GetYm());
    Pos o_result = o_line_s + o_add_pos;
    return o_result;
}

Pos CoreCalculator::TransformRotationRad_(const Pos& o_pos, float f_rad)
{
    float f_rot_x = o_pos.GetXm() * cos(f_rad) - o_pos.GetYm() * sin(f_rad);
    float f_rot_y = o_pos.GetXm() * sin(f_rad) + o_pos.GetYm() * cos(f_rad);
    return Pos(f_rot_x, f_rot_y);
}

Pos CoreCalculator::TransformRotationDeg_(const Pos& o_pos, float f_deg)
{
    return TransformRotationRad_(o_pos, f_deg * DEGtoRAD);
}

float CoreCalculator::CalcPosDotProduct_(const Pos& o_pos1, const Pos& o_pos2)
{
    return o_pos1.GetXm() * o_pos2.GetXm() + o_pos1.GetYm() * o_pos2.GetYm();
}

Pos CoreCalculator::CalcUnitVector_(const float& f_target_distance_m, const float& f_radian)
{
    Pos o_unit_vector_pos(f_target_distance_m * cos(f_radian), f_target_distance_m * sin(f_radian));
    return o_unit_vector_pos;
}

float CoreCalculator::WrapAnglePiToPiRad_(float f_angle_rad)
{
    float f_rad = fmod((f_angle_rad + CoreCalculator::ToSign_(f_angle_rad) * M_PI), 2.0 * M_PI);
    return f_rad - CoreCalculator::ToSign_(f_angle_rad) * M_PI;
}

float CoreCalculator::WrapAnglePiToPiDeg_(float f_angle_deg)
{
    float f_deg = fmod((f_angle_deg + CoreCalculator::ToSign_(f_angle_deg) * 180.0), 360.0);
    return f_deg - CoreCalculator::ToSign_(f_angle_deg) * 180.0;
}

float CoreCalculator::CalcVectorPosDistance_(const std::vector<Pos>& vec_pos)
{
    int n_vec_size = vec_pos.size();
    float f_target_distance_m = 0;
    for (int i = 0; i < n_vec_size - 1; i++) {
        f_target_distance_m += CalcPosDistance_(vec_pos.at(i), vec_pos.at(i + 1));
    }
    return f_target_distance_m;
}

Pos CoreCalculator::TransformPos_(const Pos& target, const Pos& relation)
{
    const float transform_rad = relation.GetRad();
    const float transform_x = relation.GetXm();
    const float transform_y = relation.GetYm();
    const float target_x = target.GetXm();
    const float target_y = target.GetYm();

    const float cos_transform = std::cos(transform_rad);
    const float sin_transform = std::sin(transform_rad);

    const float transformed_x = cos_transform * target_x - sin_transform * target_y + transform_x;
    const float transformed_y = sin_transform * target_x + cos_transform * target_y + transform_y;
    const float transformed_deg = target.GetDeg() + relation.GetDeg();
    const float final_deg = CoreCalculator::WrapAnglePiToPiDeg_(transformed_deg);

    return Pos(transformed_x, transformed_y, final_deg);
}

int CoreCalculator::sign(float val)
{
    return (0 < val) - (val < 0);
}

float CoreCalculator::CalcAngleDomainRad_(float f_angle_rad)
{
    return fmod((f_angle_rad + CoreCalculator::sign(f_angle_rad) * M_PI), 2.0 * M_PI) - CoreCalculator::sign(f_angle_rad) * M_PI;
}

float CoreCalculator::CalcAngleDomainDeg_(float f_angle_deg)
{
    return fmod((f_angle_deg + CoreCalculator::sign(f_angle_deg) * 180.0), 360.0) - CoreCalculator::sign(f_angle_deg) * 180.0;
}

}  // namespace NaviFra
