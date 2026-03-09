#ifndef NAVIFRA_PARAM_MSG_H_
#define NAVIFRA_PARAM_MSG_H_

#include "motion_parameter.hpp"
#include "polygon/polygon.hpp"

#include <map>

namespace NaviFra {

struct LaneAvoidance_t {
    float f_interval_m = 0;
    int n_left_num = 0;
    int n_right_num = 0;
    float f_search_step_size_m = 0;
    int n_search_min_step = 0;
    int n_search_max_step = 0;
};

struct MPCParameters_t {
    // normal
    float f_motion_steps = 0;
    float f_motion_ref_cte = 0;
    float f_motion_vel = 0;
    float f_motion_ref_etheta = 0;
    float f_motion_w_cte = 0;
    float f_motion_w_etheta = 0;
    float f_motion_w_vel = 0;
    float f_motion_w_angvel = 0;
    float f_motion_w_angvel_d = 0;
    float f_motion_w_accel = 0;
    float f_motion_w_accel_d = 0;
    float f_motion_throttle = 0;
    float f_motion_bound_value = 0;
    float f_motion_front_path_idx = 0;
    float f_motion_back_path_idx = 0;
    float f_motion_curve_front_path_idx = 0;
    float f_motion_curve_back_path_idx = 0;

    float f_motion_start_dist = 0;
    float f_motion_start_speed = 0;
    float f_motion_start_min_speed = 0;
    float f_motion_start_duration = 0;
    bool b_start_control_dist_flag = false;

    // low speed
    float f_motion_low_speed_std = 0;
    float f_motion_low_speed_w_cte = 0;
    float f_motion_low_speed_w_etheta = 0;
    float f_motion_low_speed_w_angvel = 0;

    float f_motion_high_speed_std = 0;
    float f_motion_high_speed_w_angvel = 0;
    float f_motion_high_w_accel = 0;
    float f_motion_high_w_accel_d = 0;
    float f_motion_high_throttle = 0;

    float f_motion_high_curve_speed_std = 0;
    float f_motion_high_curve_speed_w_angvel = 0;
    float f_motion_curve_w_accel = 0;
    float f_motion_curve_w_accel_d = 0;
    float f_motion_curve_throttle = 0;
    float f_motion_curve_w_angvel = 0;
    float f_motion_curve_w_angvel_d = 0;

    float f_motion_arrive_p_gain = 0;
    float f_motion_arrive_d_gain = 0;

    int n_arc_determined_distance_index = 0;
    MPCParameters_t()
    {
        f_motion_steps = 40.0;
        f_motion_ref_cte = 0.0;
        f_motion_vel = 1.0;
        f_motion_ref_etheta = 0.0;
        f_motion_w_cte = 100.0;
        f_motion_w_etheta = 50.0;
        f_motion_w_vel = 1.0;
        f_motion_w_angvel = 100.0;
        f_motion_w_angvel_d = 10.0;
        f_motion_w_accel = 50.0;
        f_motion_w_accel_d = 10.0;
        f_motion_throttle = 1.0;
        f_motion_bound_value = 1000.0;
        f_motion_front_path_idx = 50.0;
        f_motion_back_path_idx = 50.0;
        f_motion_arrive_p_gain = 0.5;
        f_motion_arrive_d_gain = 0.01;
        f_motion_curve_front_path_idx = 40.0;
        f_motion_curve_back_path_idx = 20.0;
        f_motion_start_duration = 4.0;
        b_start_control_dist_flag = true;
    }
};
struct LocalMissionParameters_t {
    int updated_frequency_ms = 1000;
    float map_size_x_m = 5.0f;
    float map_size_y_m = 5.0f;
    float map_res_m = 0.01f;
    float padding_size_m = 0.5f;
};

struct Parameters_t {
    bool b_release_mode = false;  // true면 디버그용 토픽 발행 안함
    bool b_live_topology_mode = false;
    float f_move_deleay_time_sec = 0.03;

    LaneAvoidance_t st_lane_avoidance_param;

    MotionLimitParameter_t st_motion_param;
    MotionMoveParameter_t st_motion_move_param;  // new version
    SafetyMotion_t st_safety_motion_param;
    AlignMotion_t st_align_param;  // new version

    NavCondition_t st_nav_cond_param;  // new version

    MPCParameters_t st_mpc_param;

    LocalMissionParameters_t st_local_mission_param;

    // behavior
    float f_avoid_speed_max = 0.5;
    float f_avoid_speed_ratio = 0.7;
    float f_avoid_arc_speed_ratio = 0.7;
    float f_avoid_margin_m = 0.4;

    float f_avoid_detect_obs_dir_margin_m = 1.0;

    float f_origin_detect_obs_dir_margin_m = 1.0;
    float f_origin_path_detect_obs_margin_m = 0.1;
    int n_origin_path_detect_obs_dist_m = 2;
    bool b_use_remain_obs = false;
    float f_sec_remain_obs_check = 10;

    float f_align_motion_range_accel_deg = 30.0f;
    float f_align_motion_range_decel_deg = 30.0f;
    float f_align_motion_max_rot_degs = 35.0f;
    float f_align_motion_min_rot_degs = 5.0f;
    float f_align_motion_min_rot_end_degs = 5.0f;
    float f_align_motion_min_rot_deg_range = 5.0f;
    float f_align_motion_threshold_for_start_deg = 10.0f;
    float f_align_motion_threshold_for_end_normal_deg = 0.5;
    float f_align_motion_threshold_for_end_fine_deg = 0.5;
    int n_align_motion_enable_start = 1;
    int n_align_motion_enable_end = 1;
    float f_arrived_boundary_dist_x_m = 0.005;

    float f_avoid_vel_ms = 0.8;
    float f_avoid_check_vel_ms = 0.2;

    float f_static_obs_check_time_sec = 1.0;
    float f_static_obs_check_dist_m = 0.1;

    float f_align_p_gain = 1;
    float f_align_d_gain = 0;
    float f_align_i_gain = 0;
    float f_align_pid_control_deg = 10;

    float f_goal_arrive_decel_m_ss = 0.2f;
    float f_goal_arrive_min_vel_dist_m = 0.1f;
    float f_goal_arrive_min_vel_default_dist_m = 0.03f;
    float f_goal_arrive_min_vel_dist_degs_ratio = 0.3f;
    float f_goal_arrive_min_vel_dist_diagonal_ratio = 0.1f;
    float f_near_goal_dist_thres = 0.5f;
    float f_near_goal_speed_thres = 0.1f;
    float f_linear_max_decel_mss = 1.0f;
    float f_linear_max_accel_mss = 1.0f;

    bool b_use_start_obs_check_flag = false;
    float f_start_obs_check_m = 0;
    float f_start_obs_step_m = 0.1;
    int n_start_obs_check_sec = 1;
    float f_move_obstacle_dist_max = 3;
    float f_restart_obs_check_m = 0;

    bool b_use_camera_predict_mode_flag = false;
    float f_camera_collision_predict_sec = 0;
    float f_camera_collision_margin_rotate = 0;
    float f_camera_collision_margin_reducted = 0;
    float f_dist_to_reduct_camera_margin = 0;

    bool b_use_lccs = false;
    float f_base_detect_dist = 0.0;
    float f_side_base_detect_dist = 0.0;
    float f_speed_step_std = 0.0;
    float f_low_speed_step_add_dist = 0.0;
    float f_high_speed_step_add_dist = 0.0;
    float f_side_add_dist = 0.0;
    float f_spinturn_add_radius = 0.1;

    bool b_camera_obs_clear = false;
    bool b_set_area_ratio = false;
    bool b_use_dynamic_camera = false;
    float f_set_area_ratio_time_sec = 0;
    float f_camera_suspendmission_dist_m = 0;
    float f_camera_obs_clear_time_sec = 0;
    float f_docking_camera_check_sec = 0;
    float f_camera_sto_off_m = 0;
    float f_camera_path_check_m = 0;
    float f_camera_path_check_step = 0;
    float f_camera_path_check_side_margin = 0;

    bool b_use_detection_mode_flag = true;
    bool b_use_predict_mode_flag = true;
    bool b_use_side_check_flag = true;
    int n_collision_estop_period_ms = 0;
    int n_unlock_suspending_duration_ms = 0;
    int n_predict_step_num = 10;
    int n_reduce_predict_deg_step = 10;
    float f_reduce_predict_deg_standard = 5.0;
    float f_safe_far_from_path_to_robot_distance = 0.5;
    float f_control_velocity_far_from_path_m = 0.2;
    float f_obstacle_change_delay = 0.1;
    float f_detect_margin = 0.0;
    float f_rotation_margin_m = 0.0;
    float f_resume_start_delay = 0.1;
    float f_side_check_margin = 0.1;
    float f_side_target_speed_ratio = 1.0;
    bool b_use_obs_alarm_separate = false;

    float f_collision_predict_sec = 0;

    float f_collision_extension_x_m = 0;
    float f_collision_extension_y_m = 0;
    float f_collision_extension_max_vel_ms = 0;
    float f_collision_extension_min_vel_ms = 0;

    // RS path planning
    float f_goal_arrive_turning_radius_m = 0;
    float f_goal_arrive_path_step_size_m = 0;
    float f_goal_arrive_straight_line_ratio = 0;

    std::map<std::string, Polygon> map_polygon_robot_collision_;

    int n_goal_arrive_start_align_index = 0;
    bool b_allways_align_flag = false;
    // Motion Constraints End
    float f_motion_control_period = 0.1;
    float f_vel_percent_enable_standard_ms = 0;

    int n_kinematics_type = 0;  // 0: DD, 1: Quad , 2: fork, 3: car

    // node based plan
    float f_on_path_node_threshold_m = 0.2;
    float f_on_path_edge_threshold_m = 0.25;

    // mission
    float f_angle_dividing_the_path_deg = 10.f;

    // time gap
    float f_goal_receive_time_gap_sec = 0.3;

    // 모터 에러든 센서 타임아웃이든 예외 상황에서 abort할지 말지 결정하는 플래그
    bool b_navi_error_cancel_flag = true;

    float f_diagonal_linear_lateral_w = 0.3;
    float f_diagonal_low_speed = 0.21;
    float f_diagonal_low_speed_w = 4;
    float f_diagonal_high_speed_w = 15;

    float f_diagonal_angle_kp = 1.2;
    float f_diagonal_angle_kd = 0.1;
    float f_diagonal_angle_ki = 0;
    float f_diagonal_curve_speed_ratio = 1;

    int n_icp_error_ratio_threshold = 50;
    int n_icp_error_count_threshold = 1;
    float f_current_node_change_before_m = 0.0;
    int n_hacs_curve_speed_up_dist_cm = 120;

    // path plan
    int n_path_planner_period_ms = 100;
    bool b_avoid_permission = false;
    float f_min_dist_for_forward_path_m = 0;

    // avoid
    int n_detection_period_ms = 0;
    int n_avoidance_period_ms = 0;
    float f_force_comeback_start_dist_m = 0;
    float f_force_comeback_target_dist_m = 0;
    float f_force_comeback_in_curve_added_start_dist_m = 0;
    float f_dubins_normal_max_curve_radius_m = 0;
    float f_dubins_normal_min_curve_radius_m = 0;
    float f_dubins_normal_curve_interval_m = 0;
    float f_dubins_force_max_curve_radius_m = 0;
    float f_dubins_force_min_curve_radius_m = 0;
    float f_dubins_force_curve_interval_m = 0;
    float f_polynominal_max_curvature_m = 0;
    float f_avoid_imposible_sec_s = 0;

    // public motion
    float f_toward_goal_check_thres_deg = 5.0f;

    // upper
    bool b_upper_use = false;
    // mappingbot
    bool b_control_motor_directly = true;

    bool b_avoidance_auto_comeback_path = false;

    // docking
    float f_double_check_dist_m = 0;
    float f_straight_path_dist_m = 0;

    // waypoint
    float f_start_thr_dist_m = 0.25f;
};

}  // namespace NaviFra
#endif
