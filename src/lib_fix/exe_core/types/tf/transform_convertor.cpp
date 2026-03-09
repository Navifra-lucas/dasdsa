#include "transform_convertor.hpp"

namespace NaviFra {
NaviFra::Pos TransformConvertor::ConvertToGlobalPosition_(const NaviFra::Pos& o_local_pos, const NaviFra::Pos& o_origin_pos)
{
    NaviFra::Pos o_result_pos;
    o_result_pos.SetDeg(o_origin_pos.GetDeg() + o_local_pos.GetDeg());

    float f_x_m =
        o_origin_pos.GetXm() + o_local_pos.GetXm() * cos(o_origin_pos.GetRad()) - o_local_pos.GetYm() * sin(o_origin_pos.GetRad());
    float f_y_m =
        o_origin_pos.GetYm() + o_local_pos.GetXm() * sin(o_origin_pos.GetRad()) + o_local_pos.GetYm() * cos(o_origin_pos.GetRad());
    float f_z_m = o_local_pos.GetZm();

    o_result_pos.SetXm(f_x_m);
    o_result_pos.SetYm(f_y_m);
    o_result_pos.SetZm(f_z_m);
    return o_result_pos;
}

NaviFra::Pos TransformConvertor::ConvertToLocalPosition_(const NaviFra::Pos& o_origin_pos, const NaviFra::Pos& o_global_pos)
{
    NaviFra::Pos o_result_pos;
    o_result_pos.SetRad(-o_origin_pos.GetRad() + o_global_pos.GetRad());

    float f_delta_x_m = o_global_pos.GetXm() - o_origin_pos.GetXm();
    float f_delta_y_m = o_global_pos.GetYm() - o_origin_pos.GetYm();
    float f_z_m = o_origin_pos.GetZm();

    o_result_pos.SetXm(f_delta_x_m * cos(o_origin_pos.GetRad()) + f_delta_y_m * sin(o_origin_pos.GetRad()));
    o_result_pos.SetYm(-f_delta_x_m * sin(o_origin_pos.GetRad()) + f_delta_y_m * cos(o_origin_pos.GetRad()));
    o_result_pos.SetZm(f_z_m);
    return o_result_pos;
}

std::vector<NaviFra::Pos> TransformConvertor::ConvertToGlobalPositionVector_(
    const std::vector<NaviFra::Pos>& vec_local_pos, const NaviFra::Pos& o_origin_pos)
{
    int n_vec_size = vec_local_pos.size();
    std::vector<NaviFra::Pos> vec_result_pos(n_vec_size);
    for (int i = 0; i < n_vec_size; i++) {
        vec_result_pos[i] = ConvertToGlobalPosition_(vec_local_pos[i], o_origin_pos);
    }
    return vec_result_pos;
}

std::vector<NaviFra::Pos> TransformConvertor::ConvertToLocalPositionVector_(
    const std::vector<NaviFra::Pos>& vec_global_pos, const NaviFra::Pos& o_origin_pos)
{
    int n_vec_size = vec_global_pos.size();
    std::vector<NaviFra::Pos> vec_result_pos(n_vec_size);

    for (int i = 0; i < n_vec_size; i++) {
        vec_result_pos[i] = ConvertToLocalPosition_(o_origin_pos, vec_global_pos[i]);
    }
    return vec_result_pos;
}

std::vector<SimplePos> TransformConvertor::ConvertScanToPosition_(const Scan_t& st_laser_scan, float f_scan_angle_inc, float f_min_dist)
{
    std::vector<SimplePos> vec_result_pos;
    if (st_laser_scan.vec_data_m.empty())
        return vec_result_pos;

    ExtrinsicParameter_t st_extrinsic_parameter = st_laser_scan.st_extrinsic_parameter;
    float f_roll_cos = cos(st_extrinsic_parameter.f_roll_rad);
    float f_roll_sin = sin(st_extrinsic_parameter.f_roll_rad);
    float f_pitch_cos = cos(st_extrinsic_parameter.f_pitch_rad);
    float f_pitch_sin = sin(st_extrinsic_parameter.f_pitch_rad);
    float f_yaw_cos = cos(st_extrinsic_parameter.f_yaw_rad);
    float f_yaw_sin = sin(st_extrinsic_parameter.f_yaw_rad);

    float f_scan_angle_inc_deg = f_scan_angle_inc;
    if (f_scan_angle_inc_deg < st_laser_scan.f_angle_increment_deg)
        f_scan_angle_inc_deg = st_laser_scan.f_angle_increment_deg;

    float f_angle_rad = 0.f;
    int n_inc = static_cast<int>(std::round(f_scan_angle_inc_deg / st_laser_scan.f_angle_increment_deg));
    for (int i = 0; i < st_laser_scan.vec_data_m.size(); i += n_inc) {
        SimplePos o_transformed_result_pos;
        if ((st_laser_scan.vec_data_m[i] > st_laser_scan.f_max_range_m) || (st_laser_scan.vec_data_m[i] <= st_laser_scan.f_min_range_m) ||
            std::isnan(st_laser_scan.vec_data_m[i])) {
            f_angle_rad += st_laser_scan.f_angle_increment_deg * DEGtoRAD;
            continue;
        }

        float f_dist_m = st_laser_scan.vec_data_m[i];

        if (std::isnan(f_dist_m) != 0)
            continue;
        if (f_dist_m == std::numeric_limits<float>::infinity())
            continue;

        f_angle_rad = (st_laser_scan.f_angle_min_deg + i * st_laser_scan.f_angle_increment_deg) * DEGtoRAD;

        // transformed range(dis) to x, y, z coordinates
        float f_dist_to_x_m = f_dist_m * cos(f_angle_rad);
        float f_dist_to_y_m = f_dist_m * sin(f_angle_rad);
        float f_dist_to_z_m = 0.0f;

        // transformed xyz to lidar extrinsic transform
        float f_x_m = st_extrinsic_parameter.f_x_m + f_dist_to_x_m * (f_pitch_cos * f_yaw_cos) +
            f_dist_to_y_m * (-f_roll_cos * f_yaw_sin + f_roll_sin * f_pitch_sin * f_yaw_cos) +
            f_dist_to_z_m * (f_roll_sin * f_yaw_sin + f_roll_cos * f_pitch_sin * f_yaw_cos);
        float f_y_m = st_extrinsic_parameter.f_y_m + f_dist_to_x_m * (f_pitch_cos * f_yaw_sin) +
            f_dist_to_y_m * (f_roll_cos * f_yaw_cos + f_roll_sin * f_pitch_sin * f_yaw_sin) +
            f_dist_to_z_m * (-f_roll_sin * f_yaw_cos + f_roll_cos * f_pitch_sin * f_yaw_sin);
        float f_z_m = st_extrinsic_parameter.f_z_m + f_dist_to_x_m * (-f_pitch_sin) + f_dist_to_y_m * (f_roll_sin * f_pitch_cos) +
            f_dist_to_z_m * (f_roll_cos * f_pitch_cos);

        o_transformed_result_pos.SetXm(f_x_m);
        o_transformed_result_pos.SetYm(f_y_m);
        o_transformed_result_pos.SetZm(f_z_m);

        vec_result_pos.emplace_back(o_transformed_result_pos);
    }

    return vec_result_pos;
}
}  // namespace NaviFra
