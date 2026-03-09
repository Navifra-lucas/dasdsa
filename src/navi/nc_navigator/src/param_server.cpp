#include "nc_navigator/nc_navigator_pch.h"

#include <nc_navigator/param_server.hpp>

ParamServer::ParamServer()
{
    UpdateParam();
    param_update_sub_ = node_handle_.subscribe("navifra/param_update", 10, &ParamServer::ParamCmd, this);
    navi_state_sub_ = node_handle_.subscribe("navifra/status", 1, &ParamServer::CheckState, this);

    b_robot_state_ = false;
    b_robot_init_ = true;
}
void ParamServer::GetParam()
{
    Notify("param_callback", st_param_);
    LOG_INFO("UpdateParam Done");
}

vector<string> ParamServer::split(string input, char delimiter)
{
    vector<string> answer;
    stringstream ss(input);
    string temp;

    while (getline(ss, temp, delimiter)) {
        answer.push_back(temp);
    }

    return answer;
}

bool ParamServer::Notify(const std::string& str_cbf_name, const boost::any& any_type_var)
{
    if (map_callback_pt_.find(str_cbf_name) == map_callback_pt_.end()) {
        return false;
    }
    else {
        map_callback_pt_[str_cbf_name](any_type_var);
        return true;
    }
}

bool ParamServer::RegisteCallbackFunc(const std::string& str_cbf_name, const std::function<void(const boost::any&)>& pt_func)
{
    if (map_callback_pt_.find(str_cbf_name) == map_callback_pt_.end()) {
        map_callback_pt_[str_cbf_name] = pt_func;
        return true;
    }
    else {
        return false;
    }
}

void ParamServer::CheckState(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data != "running") {
        b_robot_state_ = true;
    }
    else {
        b_robot_state_ = false;
    }
}

void ParamServer::ParamCmd(const std_msgs::String::ConstPtr& msg)
{
    while (ros::ok()) {
        NLOG(info) << "wait param idle";

        if (b_robot_state_) {
            NLOG(info) << "ParamCmd";
            UpdateParam();
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void ParamServer::UpdateParam()
{
    LOG_INFO("UpdateParam");

    ros::param::param<bool>("navigation/b_release_mode", st_param_.b_release_mode, true);
    ros::param::param<bool>("navigation/b_live_topology_mode", st_param_.b_live_topology_mode, false);
    ros::param::param<float>("motion_base/f_move_deleay_time_sec", st_param_.f_move_deleay_time_sec, 0.05);

    //---------------------------------VEL------------------------------------
    ros::param::param<float>("motion_base/f_linear_speed_max_ms", st_param_.st_motion_param.f_linear_speed_max_ms, 1.f);
    ros::param::param<float>("motion_base/f_linear_speed_min_ms", st_param_.st_motion_param.f_linear_speed_min_ms, 0.1f);
    ros::param::param<float>("motion_base/f_angular_speed_max_degs", st_param_.st_motion_param.f_angular_speed_max_degs, 35);
    ros::param::param<float>("motion_base/f_angular_speed_min_degs", st_param_.st_motion_param.f_angular_speed_min_degs, 5);
    ros::param::param<bool>("motion_diagonal/b_diagonal_control", st_param_.st_motion_param.b_diagonal_control, true);
    //------------------------------------------------------------------------

    //----------------------------SAFETY MOTION-------------------------------
    ros::param::param<float>("motion_safe/f_safety_dist_from_obs_m", st_param_.st_safety_motion_param.f_safety_dist_from_obs_m, 1.5f);
    ros::param::param<float>("motion_arrive/f_goal_decel_mss", st_param_.st_safety_motion_param.f_goal_decel_mss, 0.2f);

    //--------------------------------SAFE------------------------------------
    // 장애물 감지 관련
    ros::param::param<bool>("obstacle/b_use_detection_mode_flag", st_param_.b_use_detection_mode_flag, true);
    ros::param::param<bool>("obstacle/b_use_predict_mode_flag", st_param_.b_use_predict_mode_flag, true);
    ros::param::param<bool>("obstacle/b_use_side_check_flag", st_param_.b_use_side_check_flag, true);
    ros::param::param<bool>("obstacle/b_use_obs_alarm_separate", st_param_.b_use_obs_alarm_separate, true);
    ros::param::param<int>("obstacle/n_predict_step_num", st_param_.n_predict_step_num, 10);
    ros::param::param<int>("obstacle/n_reduce_predict_deg_step", st_param_.n_reduce_predict_deg_step, 10);
    ros::param::param<float>("obstacle/f_reduce_predict_deg_standard", st_param_.f_reduce_predict_deg_standard, 5.0);
    ros::param::param<float>("obstacle/f_obstacle_change_delay", st_param_.f_obstacle_change_delay, 0.1);
    ros::param::param<float>("obstacle/f_resume_start_delay", st_param_.f_resume_start_delay, 0.1);
    ros::param::param<float>("obstacle/f_collision_predict_sec", st_param_.f_collision_predict_sec, 4.0);
    ros::param::param<bool>("obstacle/b_use_start_obs_check_flag", st_param_.b_use_start_obs_check_flag, false);
    ros::param::param<float>("obstacle/f_start_obs_check_m", st_param_.f_start_obs_check_m, 0);
    ros::param::param<float>("obstacle/f_restart_obs_check_m", st_param_.f_restart_obs_check_m, 0);
    ros::param::param<float>("obstacle/f_start_obs_step_m", st_param_.f_start_obs_step_m, 1);
    ros::param::param<float>("obstacle/f_side_check_margin", st_param_.f_side_check_margin, 0);
    ros::param::param<float>("obstacle/f_side_target_speed_ratio", st_param_.f_side_target_speed_ratio, 1);
    ros::param::param<float>("obstacle/f_detect_margin", st_param_.f_detect_margin, 0);
    ros::param::param<int>("obstacle/n_start_obs_check_sec", st_param_.n_start_obs_check_sec, 1);
    ros::param::param<float>("obstacle/f_move_obstacle_dist_max", st_param_.f_move_obstacle_dist_max, 3);

    ros::param::param<float>("obstacle/f_camera_collision_predict_sec", st_param_.f_camera_collision_predict_sec, 0);
    ros::param::param<bool>("obstacle/b_use_camera_predict_mode_flag", st_param_.b_use_camera_predict_mode_flag, false);
    ros::param::param<bool>("obstacle/b_camera_obs_clear", st_param_.b_camera_obs_clear, false);
    ros::param::param<bool>("obstacle/b_set_area_ratio", st_param_.b_set_area_ratio, false);
    ros::param::param<bool>("obstacle/b_use_dynamic_camera", st_param_.b_use_dynamic_camera, false);
    ros::param::param<float>("obstacle/f_set_area_ratio_time_sec", st_param_.f_set_area_ratio_time_sec, 0);
    ros::param::param<float>("obstacle/f_camera_suspendmission_dist_m", st_param_.f_camera_suspendmission_dist_m, 0);
    ros::param::param<float>("obstacle/f_camera_obs_clear_time_sec", st_param_.f_camera_obs_clear_time_sec, 10);
    ros::param::param<float>("obstacle/f_camera_collision_margin_rotate", st_param_.f_camera_collision_margin_rotate, 0);
    ros::param::param<float>("obstacle/f_camera_collison_margin_reducted", st_param_.f_camera_collision_margin_reducted, 0);
    ros::param::param<float>("obstacle/f_dist_to_reduct_camera_margin", st_param_.f_dist_to_reduct_camera_margin, 0);
    ros::param::param<float>("obstacle/f_docking_camera_check_sec", st_param_.f_docking_camera_check_sec, 0);
    ros::param::param<float>("obstacle/f_camera_sto_off_m", st_param_.f_camera_sto_off_m, 1.0);
    ros::param::param<float>("obstacle/f_camera_path_check_m", st_param_.f_camera_path_check_m, 0);
    ros::param::param<float>("obstacle/f_camera_path_check_step", st_param_.f_camera_path_check_step, 0);
    ros::param::param<float>("obstacle/f_camera_path_check_side_margin", st_param_.f_camera_path_check_side_margin, 0);

    ros::param::param<float>("obstacle/f_origin_detect_obs_dir_margin_m", st_param_.f_origin_detect_obs_dir_margin_m, 1.0);
    ros::param::param<float>("obstacle/f_origin_path_detect_obs_margin_m", st_param_.f_origin_path_detect_obs_margin_m, 0.1);
    ros::param::param<int>("obstacle/n_origin_path_detect_obs_dist_m", st_param_.n_origin_path_detect_obs_dist_m, 0);

    ros::param::param<bool>("obstacle/b_use_remain_obs", st_param_.b_use_remain_obs, false);
    ros::param::param<float>("obstacle/f_sec_remain_obs_check", st_param_.f_sec_remain_obs_check, 10);

    ros::param::param<bool>("lccs/b_use_lccs", st_param_.b_use_lccs, true);
    ros::param::param<float>("lccs/f_base_detect_dist", st_param_.f_base_detect_dist, 1.0);
    ros::param::param<float>("lccs/f_side_base_detect_dist", st_param_.f_side_base_detect_dist, 0.2);
    ros::param::param<float>("lccs/f_speed_step_std", st_param_.f_speed_step_std, 0.2);
    ros::param::param<float>("lccs/f_low_speed_step_add_dist", st_param_.f_low_speed_step_add_dist, 0.1);
    ros::param::param<float>("lccs/f_high_speed_step_add_dist", st_param_.f_high_speed_step_add_dist, 0.2);
    ros::param::param<float>("lccs/f_side_add_dist", st_param_.f_side_add_dist, 0.2);
    ros::param::param<float>("lccs/f_spinturn_add_radius", st_param_.f_spinturn_add_radius, 0.2);
    
    //------------------------------------------------------------------------
    //-------------------------------MOTION-----------------------------------
    ros::param::param<float>("motion_base/f_motion_control_period", st_param_.f_motion_control_period, 0.03f);

    // motion angle align algorithm
    ros::param::param<float>("motion_align/f_align_motion_range_accel_deg", st_param_.f_align_motion_range_accel_deg, 30.0);
    ros::param::param<float>("motion_align/f_align_motion_range_decel_deg", st_param_.f_align_motion_range_decel_deg, 30.0);
    ros::param::param<float>("motion_align/f_align_motion_max_rot_degs", st_param_.f_align_motion_max_rot_degs, 35.0);
    ros::param::param<float>("motion_align/f_align_motion_min_rot_degs", st_param_.f_align_motion_min_rot_degs, 3.0);
    ros::param::param<float>("motion_align/f_align_motion_min_rot_end_degs", st_param_.f_align_motion_min_rot_end_degs, 3.0);
    ros::param::param<float>("motion_align/f_align_motion_min_rot_deg_range", st_param_.f_align_motion_min_rot_deg_range, 5.0);
    ros::param::param<float>("motion_align/f_align_motion_threshold_for_start_deg", st_param_.f_align_motion_threshold_for_start_deg, 10.0);
    ros::param::param<float>(
        "motion_align/f_align_motion_threshold_for_end_normal_deg", st_param_.f_align_motion_threshold_for_end_normal_deg, 0.5);
    ros::param::param<float>(
        "motion_align/f_align_motion_threshold_for_end_fine_deg", st_param_.f_align_motion_threshold_for_end_fine_deg, 0.5);

    ros::param::param<float>("motion_align/f_align_p_gain", st_param_.f_align_p_gain, 0.2);
    ros::param::param<float>("motion_align/f_align_d_gain", st_param_.f_align_d_gain, 0.0);
    ros::param::param<float>("motion_align/f_align_i_gain", st_param_.f_align_i_gain, 0.0);
    ros::param::param<float>("motion_align/f_align_pid_control_deg", st_param_.f_align_pid_control_deg, 0.0);

    ros::param::param<float>("motion_base/f_vel_percent_enable_standard_ms", st_param_.f_vel_percent_enable_standard_ms, 0.3f);
    ros::param::param<float>("motion_arrive/f_goal_arrive_min_vel_dist_m", st_param_.f_goal_arrive_min_vel_dist_m, 0.05f);
    ros::param::param<float>("motion_arrive/f_goal_arrive_min_vel_default_dist_m", st_param_.f_goal_arrive_min_vel_default_dist_m, 0.03f);
    ros::param::param<float>("motion_arrive/f_goal_arrive_min_vel_dist_degs_ratio", st_param_.f_goal_arrive_min_vel_dist_degs_ratio, 0.1f);
    ros::param::param<float>(
        "motion_arrive/f_goal_arrive_min_vel_dist_diagonal_ratio", st_param_.f_goal_arrive_min_vel_dist_diagonal_ratio, 0.1f);
    ros::param::param<float>("motion_arrive/f_near_goal_speed_thres", st_param_.f_near_goal_speed_thres, 0.2f);
    ros::param::param<float>("motion_arrive/f_near_goal_dist_thres", st_param_.f_near_goal_dist_thres, 0.5f);
    ros::param::param<float>("motion_arrive/f_goal_arrive_decel_m_ss", st_param_.f_goal_arrive_decel_m_ss, 0.2f);
    ros::param::param<float>("motion_arrive/f_goal_arrive_turning_radius_m", st_param_.f_goal_arrive_turning_radius_m, 2);
    ros::param::param<float>("motion_base/f_linear_max_decel_mss", st_param_.f_linear_max_decel_mss, 0.5);
    ros::param::param<float>("motion_base/f_linear_max_accel_mss", st_param_.f_linear_max_accel_mss, 0.5);

    // goal curve align
    // align angle path planner
    ros::param::param<int>("planner_base/n_path_planner_period_ms", st_param_.n_path_planner_period_ms, 100);
    ros::param::param<bool>("planner_base/b_avoid_permission", st_param_.b_avoid_permission, true);
    ros::param::param<float>("planner_base/f_min_dist_for_forward_path_m", st_param_.f_min_dist_for_forward_path_m, 7.0);
    ros::param::param<float>("planner_base/f_goal_arrive_path_step_size_m", st_param_.f_goal_arrive_path_step_size_m, 0.01);
    ros::param::param<float>("planner_base/f_goal_arrive_straight_line_ratio", st_param_.f_goal_arrive_straight_line_ratio, 0.2);
    ros::param::param<int>("planner_base/n_goal_arrive_start_align_index", st_param_.n_goal_arrive_start_align_index, 150);
    ros::param::param<bool>("planner_base/b_allways_align_flag", st_param_.b_allways_align_flag, false);
    ros::param::param<float>("planner_base/f_goal_arrive_turning_radius_m", st_param_.f_goal_arrive_turning_radius_m, 2);

    // Parameter for MPC solver
    ros::param::param<float>("motion_track/f_motion_steps", st_param_.st_mpc_param.f_motion_steps, 40.0);
    ros::param::param<float>("motion_track/f_motion_ref_cte", st_param_.st_mpc_param.f_motion_ref_cte, 0.0);
    ros::param::param<float>("motion_track/f_motion_vel", st_param_.st_mpc_param.f_motion_vel, 1.0);
    ros::param::param<float>("motion_track/f_motion_ref_etheta", st_param_.st_mpc_param.f_motion_ref_etheta, 0.0);
    ros::param::param<float>("motion_track/f_motion_w_cte", st_param_.st_mpc_param.f_motion_w_cte, 100.0);
    ros::param::param<float>("motion_track/f_motion_w_etheta", st_param_.st_mpc_param.f_motion_w_etheta, 50.0);
    ros::param::param<float>("motion_track/f_motion_w_vel", st_param_.st_mpc_param.f_motion_w_vel, 1.0);
    ros::param::param<float>("motion_track/f_motion_w_angvel", st_param_.st_mpc_param.f_motion_w_angvel, 100.0);
    ros::param::param<float>("motion_track/f_motion_w_angvel_d", st_param_.st_mpc_param.f_motion_w_angvel_d, 10.0);
    ros::param::param<float>("motion_track/f_motion_w_accel", st_param_.st_mpc_param.f_motion_w_accel, 50.0);
    ros::param::param<float>("motion_track/f_motion_w_accel_d", st_param_.st_mpc_param.f_motion_w_accel_d, 10.0);
    ros::param::param<float>("motion_track/f_motion_throttle", st_param_.st_mpc_param.f_motion_throttle, 1.0);  // Maximal throttle accel
    ros::param::param<float>(
        "motion_track/f_motion_bound_value", st_param_.st_mpc_param.f_motion_bound_value, 1000.0);  // Bound value for other variables
    ros::param::param<float>("motion_track/f_motion_front_path_idx", st_param_.st_mpc_param.f_motion_front_path_idx, 60.0);
    ros::param::param<float>("motion_track/f_motion_back_path_idx", st_param_.st_mpc_param.f_motion_back_path_idx, 40.0);

    ros::param::param<float>("motion_track/f_motion_curve_front_path_idx", st_param_.st_mpc_param.f_motion_curve_front_path_idx, 40.0);
    ros::param::param<float>("motion_track/f_motion_curve_back_path_idx", st_param_.st_mpc_param.f_motion_curve_back_path_idx, 20.0);

    ros::param::param<float>("motion_track/f_motion_start_dist", st_param_.st_mpc_param.f_motion_start_dist, 0.15);
    ros::param::param<float>("motion_track/f_motion_start_speed", st_param_.st_mpc_param.f_motion_start_speed, 0.15);
    ros::param::param<float>("motion_track/f_motion_start_min_speed", st_param_.st_mpc_param.f_motion_start_min_speed, 0.05);
    ros::param::param<bool>("motion_base/b_start_control_dist_flag", st_param_.st_mpc_param.b_start_control_dist_flag, true);
    ros::param::param<float>("motion_track/f_motion_start_duration", st_param_.st_mpc_param.f_motion_start_duration, 3.0);

    ros::param::param<float>("motion_track/f_motion_low_speed_std", st_param_.st_mpc_param.f_motion_low_speed_std, 0.2);
    ros::param::param<float>("motion_track/f_motion_low_speed_w_cte", st_param_.st_mpc_param.f_motion_low_speed_w_cte, 10.0);
    ros::param::param<float>("motion_track/f_motion_low_speed_w_etheta", st_param_.st_mpc_param.f_motion_low_speed_w_etheta, 5.0);
    ros::param::param<float>("motion_track/f_motion_low_speed_w_angvel", st_param_.st_mpc_param.f_motion_low_speed_w_angvel, 5.0);

    ros::param::param<float>("motion_track/f_motion_high_speed_std", st_param_.st_mpc_param.f_motion_high_speed_std, 2.0);
    ros::param::param<float>("motion_track/f_motion_high_speed_w_angvel", st_param_.st_mpc_param.f_motion_high_speed_w_angvel, 5);
    ros::param::param<float>("motion_track/f_motion_high_w_accel", st_param_.st_mpc_param.f_motion_high_w_accel, 0.01);
    ros::param::param<float>("motion_track/f_motion_high_w_accel_d", st_param_.st_mpc_param.f_motion_high_w_accel_d, 0.01);
    ros::param::param<float>("motion_track/f_motion_high_throttle", st_param_.st_mpc_param.f_motion_high_throttle, 20.0);

    ros::param::param<float>("motion_track/f_motion_high_curve_speed_std", st_param_.st_mpc_param.f_motion_high_curve_speed_std, 2.0);
    ros::param::param<float>(
        "motion_track/f_motion_high_curve_speed_w_angvel", st_param_.st_mpc_param.f_motion_high_curve_speed_w_angvel, 5);
    ros::param::param<float>("motion_track/f_motion_curve_w_accel", st_param_.st_mpc_param.f_motion_curve_w_accel, 0.01);
    ros::param::param<float>("motion_track/f_motion_curve_w_accel_d", st_param_.st_mpc_param.f_motion_curve_w_accel_d, 0.01);
    ros::param::param<float>("motion_track/f_motion_curve_throttle", st_param_.st_mpc_param.f_motion_curve_throttle, 20.0);

    ros::param::param<float>("motion_arrive/f_motion_arrive_p_gain", st_param_.st_mpc_param.f_motion_arrive_p_gain, 0.5);
    ros::param::param<float>("motion_arrive/f_motion_arrive_d_gain", st_param_.st_mpc_param.f_motion_arrive_d_gain, 0.01);

    ros::param::param<float>("motion_track/f_motion_curve_w_angvel", st_param_.st_mpc_param.f_motion_curve_w_angvel, 2);
    ros::param::param<float>("motion_track/f_motion_curve_w_angvel_d", st_param_.st_mpc_param.f_motion_curve_w_angvel_d, 2);

    ros::param::param<float>(
        "motion_safe/f_safe_far_from_path_to_robot_distance", st_param_.f_safe_far_from_path_to_robot_distance,
        0.5);  // 패쓰로부터 최대 로봇 떨어진 허용거리
    ros::param::param<float>(
        "motion_safe/f_control_velocity_far_from_path_m", st_param_.f_control_velocity_far_from_path_m,
        0.1);  // 패쓰로부터 이상으로 떨어지면 저속제어

    ros::param::param<int>("motion_base/n_kinematics_type", st_param_.n_kinematics_type, 0);
    ros::param::param<float>("motion_arrive/f_arrived_boundary_dist_x_m", st_param_.f_arrived_boundary_dist_x_m, 0.005);
    ros::param::param<float>("planner/f_angle_dividing_the_path_deg", st_param_.f_angle_dividing_the_path_deg, 10.f);

    // 예외처리
    ros::param::param<float>("navigation/f_goal_receive_time_gap_sec", st_param_.f_goal_receive_time_gap_sec, 0.3);

    // QUAD
    ros::param::param<float>("motion_diagonal/f_diagonal_linear_lateral_w", st_param_.f_diagonal_linear_lateral_w, 0.3);
    ros::param::param<float>("motion_diagonal/f_diagonal_low_speed", st_param_.f_diagonal_low_speed, 0.21);
    ros::param::param<float>("motion_diagonal/f_diagonal_low_speed_w", st_param_.f_diagonal_low_speed_w, 15.0);
    ros::param::param<float>("motion_diagonal/f_diagonal_high_speed_w", st_param_.f_diagonal_high_speed_w, 4.0);
    ros::param::param<float>("motion_diagonal/f_diagonal_angle_kp", st_param_.f_diagonal_angle_kp, 1.2);
    ros::param::param<float>("motion_diagonal/f_diagonal_angle_kd", st_param_.f_diagonal_angle_kd, 0.1);
    ros::param::param<float>("motion_diagonal/f_diagonal_angle_ki", st_param_.f_diagonal_angle_ki, 0);
    ros::param::param<float>("motion_diagonal/f_diagonal_curve_speed_ratio", st_param_.f_diagonal_curve_speed_ratio, 1.0);

    ros::param::param<int>("navigation/n_icp_error_ratio_threshold", st_param_.n_icp_error_ratio_threshold, 50);
    ros::param::param<int>("navigation/n_icp_error_count_threshold", st_param_.n_icp_error_count_threshold, 1);
    ros::param::param<float>("navigation/f_current_node_change_before_m", st_param_.f_current_node_change_before_m, 0.0);
    ros::param::param<int>("motion_track/n_hacs_curve_speed_up_dist_cm", st_param_.n_hacs_curve_speed_up_dist_cm, 120);

    //--------------------------------LANE AVOIDANCE------------------------------------

    ros::param::param<float>("motion_avoid/f_avoid_margin_m", st_param_.f_avoid_margin_m, 0.4);
    ros::param::param<float>("motion_avoid/f_avoid_detect_obs_dir_margin_m", st_param_.f_avoid_detect_obs_dir_margin_m, 1.0);
    ros::param::param<bool>("motion_avoid/b_avoidance_auto_comeback_path", st_param_.b_avoidance_auto_comeback_path, true);
    ros::param::param<float>("motion_avoid/f_avoid_vel_ms", st_param_.f_avoid_vel_ms, 1.0);
    ros::param::param<float>("motion_avoid/f_avoid_check_vel_ms", st_param_.f_avoid_check_vel_ms, 0.2);
    ros::param::param<float>("motion_avoid/f_static_obs_check_time_sec", st_param_.f_static_obs_check_time_sec, 1.0);
    ros::param::param<float>("motion_avoid/f_static_obs_check_dist_m", st_param_.f_static_obs_check_dist_m, 0.1);

    //--------------------------------LOCAL MISSION------------------------------------
    ros::param::param<int>("planner_local/updated_frequency_ms", st_param_.st_local_mission_param.updated_frequency_ms, 1000);
    ros::param::param<float>("planner_local/map_size_x_m", st_param_.st_local_mission_param.map_size_x_m, 14.0);
    ros::param::param<float>("planner_local/map_size_y_m", st_param_.st_local_mission_param.map_size_y_m, 14.0);
    ros::param::param<float>("planner_local/map_res_m", st_param_.st_local_mission_param.map_res_m, 0.1);
    ros::param::param<float>("planner_local/padding_size_m", st_param_.st_local_mission_param.padding_size_m, 0.1);

    //--------------------------------LANE AVOIDANCE------------------------------------
    ros::param::param<int>("planner_avoid/n_detection_period_ms", st_param_.n_detection_period_ms, 700);
    ros::param::param<int>("planner_avoid/n_avoidance_period_ms", st_param_.n_avoidance_period_ms, 2000);
    ros::param::param<float>("planner_avoid/f_force_comeback_start_dist_m", st_param_.f_force_comeback_start_dist_m, 2.0);
    ros::param::param<float>("planner_avoid/f_force_comeback_target_dist_m", st_param_.f_force_comeback_target_dist_m, 1.0);
    ros::param::param<float>(
        "planner_avoid/f_force_comeback_in_curve_added_start_dist_m", st_param_.f_force_comeback_in_curve_added_start_dist_m, 4.0);
    ros::param::param<float>("planner_avoid/f_polynominal_max_curvature_m", st_param_.f_polynominal_max_curvature_m, 1.0);
    ros::param::param<float>("planner_avoid/f_avoid_imposible_sec_s", st_param_.f_avoid_imposible_sec_s, 5.0);

    ros::param::param<float>("planner_avoid/f_interval_m", st_param_.st_lane_avoidance_param.f_interval_m, 1.0);
    ros::param::param<int>("planner_avoid/n_left_num", st_param_.st_lane_avoidance_param.n_left_num, 2);
    ros::param::param<int>("planner_avoid/n_right_num", st_param_.st_lane_avoidance_param.n_right_num, 2);
    ros::param::param<float>("planner_avoid/f_search_step_size_m", st_param_.st_lane_avoidance_param.f_search_step_size_m, 1.0);
    ros::param::param<int>("planner_avoid/n_search_min_step", st_param_.st_lane_avoidance_param.n_search_min_step, 4);
    ros::param::param<int>("planner_avoid/n_search_max_step", st_param_.st_lane_avoidance_param.n_search_max_step, 6);
    ros::param::param<float>("planner_avoid/f_dubins_normal_max_curve_radius_m", st_param_.f_dubins_normal_max_curve_radius_m, 1.0);
    ros::param::param<float>("planner_avoid/f_dubins_normal_min_curve_radius_m", st_param_.f_dubins_normal_min_curve_radius_m, 1.0);
    ros::param::param<float>("planner_avoid/f_dubins_normal_curve_interval_m", st_param_.f_dubins_normal_curve_interval_m, 1.0);
    ros::param::param<float>("planner_avoid/f_dubins_force_max_curve_radius_m", st_param_.f_dubins_force_max_curve_radius_m, 1.0);
    ros::param::param<float>("planner_avoid/f_dubins_force_min_curve_radius_m", st_param_.f_dubins_force_min_curve_radius_m, 1.0);
    ros::param::param<float>("planner_avoid/f_dubins_force_curve_interval_m", st_param_.f_dubins_force_curve_interval_m, 1.0);

    // Upper
    ros::param::param<bool>("upper/b_upper_use", st_param_.b_upper_use, false);
    // mappingbot
    ros::param::param<bool>("mappingbot/b_control_motor_directly", st_param_.b_control_motor_directly, true);

    std::vector<float> vec_polygon_vertexes;
    std::vector<string> vec_polygon_list;

    node_handle_.getParam("obstacle/poly_list", vec_polygon_list);
    for (int i = 0; i < vec_polygon_list.size(); i++) {
        vec_polygon_vertexes.clear();
        try {
            NLOG(info) << "vec_polygon_list.at(i) get " << vec_polygon_list.at(i).c_str();
            node_handle_.getParam("obstacle/" + vec_polygon_list.at(i), vec_polygon_vertexes);

            if (st_param_.map_polygon_robot_collision_.find(vec_polygon_list.at(i)) != st_param_.map_polygon_robot_collision_.end()) {
                LOG_INFO("map_polygon_robot_collision_ find %s", vec_polygon_list.at(i).c_str());
                st_param_.map_polygon_robot_collision_[vec_polygon_list.at(i)].Clear();
                st_param_.map_polygon_robot_collision_[vec_polygon_list.at(i)].AddVertexList(vec_polygon_vertexes);
            }
            else {
                LOG_INFO("map_polygon_robot_collision_ not find %s", vec_polygon_list.at(i).c_str());
                st_param_.map_polygon_robot_collision_.insert(std::make_pair(vec_polygon_list.at(i), Polygon()));
                st_param_.map_polygon_robot_collision_[vec_polygon_list.at(i)].AddVertexList(vec_polygon_vertexes);
            }
            for (int j = 0; j < vec_polygon_vertexes.size(); j++) {
                LOG_INFO("polygon %d : %f", j, vec_polygon_vertexes[j]);
            }
        }
        catch (const std::exception& e) {
            std::cerr << "obstacle error " << e.what() << '\n';
            continue;
        }
    }

    NaviFra::ParamRepository::GetInstance()->NotifyParam(st_param_);
    Notify("param_callback", st_param_);

    if (!b_robot_init_) {
        LOG_INFO("//-------------------PRINT ALL NAVIGATOR PARAM--------------------//\n");
        LOG_INFO("navigation/b_release_mode %d\n", st_param_.b_release_mode);
        LOG_INFO("navigation/b_live_topology_mode %d\n", st_param_.b_live_topology_mode);
        LOG_INFO("motion_base/f_move_deleay_time_sec %.3f\n", st_param_.f_move_deleay_time_sec);

        LOG_INFO("//----------------------------VEL------------------------------------//\n");
        LOG_INFO("motion_base/f_linear_speed_max_ms %.3f\n", st_param_.st_motion_param.f_linear_speed_max_ms);
        LOG_INFO("motion_base/f_linear_speed_min_ms %.3f\n", st_param_.st_motion_param.f_linear_speed_min_ms);
        LOG_INFO("motion_base/f_angular_speed_max_degs %.3f\n", st_param_.st_motion_param.f_angular_speed_max_degs);
        LOG_INFO("motion_base/f_angular_speed_min_degs %.3f\n", st_param_.st_motion_param.f_angular_speed_min_degs);
        LOG_INFO("motion_base/b_diagonal_control %d\n", st_param_.st_motion_param.b_diagonal_control);

        LOG_INFO("//------------------------SAFETY MOTION------------------------------//\n");
        LOG_INFO("motion_safe/f_safety_dist_from_obs_m %.3f\n", st_param_.st_safety_motion_param.f_safety_dist_from_obs_m);
        LOG_INFO("motion_arrive/f_goal_decel_mss %.3f\n", st_param_.st_safety_motion_param.f_goal_decel_mss);

        LOG_INFO("//-------------------------------SAFE--------------------------------//\n");
        LOG_INFO("obstacle/b_use_detection_mode_flag %d\n", st_param_.b_use_detection_mode_flag);
        LOG_INFO("obstacle/b_use_predict_mode_flag %d\n", st_param_.b_use_predict_mode_flag);
        LOG_INFO("obstacle/b_use_side_check_flag %d\n", st_param_.b_use_side_check_flag);
        LOG_INFO("obstacle/b_use_obs_alarm_separate %d\n", st_param_.b_use_obs_alarm_separate);
        LOG_INFO("obstacle/n_predict_step_num %d\n", st_param_.n_predict_step_num);
        LOG_INFO("obstacle/f_obstacle_change_delay %.3f\n", st_param_.f_obstacle_change_delay);
        LOG_INFO("obstacle/f_resume_start_delay %.3f\n", st_param_.f_resume_start_delay);
        LOG_INFO("obstacle/f_collision_predict_sec %.3f\n", st_param_.f_collision_predict_sec);
        LOG_INFO("obstacle/b_use_start_obs_check_flag %d\n", st_param_.b_use_start_obs_check_flag);
        LOG_INFO("obstacle/f_start_obs_check_m %.3f\n", st_param_.f_start_obs_check_m);
        LOG_INFO("obstacle/f_restart_obs_check_m %.3f\n", st_param_.f_restart_obs_check_m);
        LOG_INFO("obstacle/f_start_obs_step_m %.3f\n", st_param_.f_start_obs_step_m);
        LOG_INFO("obstacle/f_side_check_margin %.3f\n", st_param_.f_side_check_margin);
        LOG_INFO("obstacle/f_side_target_speed_ratio %.3f\n", st_param_.f_side_target_speed_ratio);
        LOG_INFO("obstacle/f_detect_margin %.3f\n", st_param_.f_detect_margin);
        LOG_INFO("obstacle/n_start_obs_check_sec %d\n", st_param_.n_start_obs_check_sec);
        LOG_INFO("obstacle/f_move_obstacle_dist_max %.3f\n", st_param_.f_move_obstacle_dist_max);

        LOG_INFO("obstacle/f_camera_collision_predict_sec %.3f\n", st_param_.f_camera_collision_predict_sec);
        LOG_INFO("obstacle/b_use_camera_predict_mode_flag %d\n", st_param_.b_use_camera_predict_mode_flag);
        LOG_INFO("obstacle/b_camera_obs_clear %d\n", st_param_.b_camera_obs_clear);
        LOG_INFO("obstacle/b_set_area_ratio %d\n", st_param_.b_set_area_ratio);
        LOG_INFO("obstacle/b_use_dynamic_camera %d\n", st_param_.b_use_dynamic_camera);
        LOG_INFO("obstacle/f_set_area_ratio_time_sec %.3f\n", st_param_.f_set_area_ratio_time_sec);
        LOG_INFO("obstacle/f_camera_suspendmission_dist_m %.3f\n", st_param_.f_camera_suspendmission_dist_m);
        LOG_INFO("obstacle/f_camera_obs_clear_time_sec %.3f\n", st_param_.f_camera_obs_clear_time_sec);
        LOG_INFO("obstacle/f_camera_collision_margin_rotate %.3f\n", st_param_.f_camera_collision_margin_rotate);
        LOG_INFO("obstacle/f_camera_collison_margin_reducted %.3f\n", st_param_.f_camera_collision_margin_reducted);
        LOG_INFO("obstacle/f_dist_to_reduct_camera_margin %.3f\n", st_param_.f_dist_to_reduct_camera_margin);
        LOG_INFO("obstacle/f_docking_camera_check_sec %.3f\n", st_param_.f_docking_camera_check_sec);
        LOG_INFO("obstacle/f_camera_sto_off_m %.3f\n", st_param_.f_camera_sto_off_m);
        LOG_INFO("obstacle/f_camera_path_check_m %.3f\n", st_param_.f_camera_path_check_m);
        LOG_INFO("obstacle/f_camera_path_check_step %.3f\n", st_param_.f_camera_path_check_step);
        LOG_INFO("obstacle/f_camera_path_check_side_margin %.3f\n", st_param_.f_camera_path_check_side_margin);

        LOG_INFO("obstacle/f_origin_detect_obs_dir_margin_m %.3f\n", st_param_.f_origin_detect_obs_dir_margin_m);
        LOG_INFO("obstacle/f_origin_path_detect_obs_margin_m %.3f\n", st_param_.f_origin_path_detect_obs_margin_m);
        LOG_INFO("obstacle/n_origin_path_detect_obs_dist_m %d\n", st_param_.n_origin_path_detect_obs_dist_m);

        LOG_INFO("obstacle/b_use_remain_obs %d\n", st_param_.b_use_remain_obs);
        LOG_INFO("obstacle/f_sec_remain_obs_check %.3f\n", st_param_.f_sec_remain_obs_check);

        LOG_INFO("//----------------------------MOTION---------------------------------//\n");
        LOG_INFO("obstacle/f_motion_control_period %.3f\n", st_param_.f_motion_control_period);

        LOG_INFO("motion_align/f_align_motion_range_accel_deg %.3f\n", st_param_.f_align_motion_range_accel_deg);
        LOG_INFO("motion_align/f_align_motion_range_decel_deg %.3f\n", st_param_.f_align_motion_range_decel_deg);
        LOG_INFO("motion_align/f_align_motion_max_rot_degs %.3f\n", st_param_.f_align_motion_max_rot_degs);
        LOG_INFO("motion_align/f_align_motion_min_rot_degs %.3f\n", st_param_.f_align_motion_min_rot_degs);
        LOG_INFO("motion_align/f_align_motion_min_rot_end_degs %.3f\n", st_param_.f_align_motion_min_rot_end_degs);
        LOG_INFO("motion_align/f_align_motion_min_rot_deg_range %.3f\n", st_param_.f_align_motion_min_rot_deg_range);
        LOG_INFO("motion_align/f_align_motion_threshold_for_start_deg %.3f\n", st_param_.f_align_motion_threshold_for_start_deg);
        LOG_INFO("motion_align/f_align_motion_threshold_for_end_normal_deg %.3f\n", st_param_.f_align_motion_threshold_for_end_normal_deg);
        LOG_INFO("motion_align/f_align_motion_threshold_for_end_fine_deg %.3f\n", st_param_.f_align_motion_threshold_for_end_fine_deg);

        LOG_INFO("motion_align/f_align_p_gain %.3f\n", st_param_.f_align_p_gain);
        LOG_INFO("motion_align/f_align_d_gain %.3f\n", st_param_.f_align_d_gain);
        LOG_INFO("motion_align/f_align_i_gain %.3f\n", st_param_.f_align_i_gain);
        LOG_INFO("motion_align/f_align_pid_control_deg %.3f\n", st_param_.f_align_pid_control_deg);

        LOG_INFO("motion_base/f_vel_percent_enable_standard_ms %.3f\n", st_param_.f_vel_percent_enable_standard_ms);
        LOG_INFO("motion_arrive/f_goal_arrive_min_vel_dist_m %.3f\n", st_param_.f_goal_arrive_min_vel_dist_m);
        LOG_INFO("motion_arrive/f_goal_arrive_min_vel_default_dist_m %.3f\n", st_param_.f_goal_arrive_min_vel_default_dist_m);
        LOG_INFO("motion_arrive/f_goal_arrive_min_vel_dist_degs_ratio %.3f\n", st_param_.f_goal_arrive_min_vel_dist_degs_ratio);
        LOG_INFO("motion_arrive/f_goal_arrive_min_vel_dist_diagonal_ratio %.3f\n", st_param_.f_goal_arrive_min_vel_dist_diagonal_ratio);
        LOG_INFO("motion_arrive/f_near_goal_speed_thres %.3f\n", st_param_.f_near_goal_speed_thres);
        LOG_INFO("motion_arrive/f_near_goal_dist_thres %.3f\n", st_param_.f_near_goal_dist_thres);
        LOG_INFO("motion_arrive/f_goal_arrive_decel_m_ss %.3f\n", st_param_.f_goal_arrive_decel_m_ss);
        LOG_INFO("motion_arrive/f_goal_arrive_turning_radius_m %.3f\n", st_param_.f_goal_arrive_turning_radius_m);
        LOG_INFO("motion_base/f_linear_max_decel_mss %.3f\n", st_param_.f_linear_max_decel_mss);
        LOG_INFO("motion_base/f_linear_max_accel_mss %.3f\n", st_param_.f_linear_max_accel_mss);

        LOG_INFO("planner_base/n_path_planner_period_ms %d\n", st_param_.n_path_planner_period_ms);
        LOG_INFO("planner_base/b_avoid_permission %d\n", st_param_.b_avoid_permission);
        LOG_INFO("planner_base/f_min_dist_for_forward_path_m %.3f\n", st_param_.f_min_dist_for_forward_path_m);
        LOG_INFO("planner_base/f_goal_arrive_path_step_size_m %.3f\n", st_param_.f_goal_arrive_path_step_size_m);
        LOG_INFO("planner_base/f_goal_arrive_straight_line_ratio %.3f\n", st_param_.f_goal_arrive_straight_line_ratio);
        LOG_INFO("planner_base/n_goal_arrive_start_align_index %d\n", st_param_.n_goal_arrive_start_align_index);
        LOG_INFO("planner_base/b_allways_align_flag %d\n", st_param_.b_allways_align_flag);
        LOG_INFO("planner_base/f_goal_arrive_turning_radius_m %.3f\n", st_param_.f_goal_arrive_turning_radius_m);

        LOG_INFO("//----------------------Parameter for MPC solver---------------------//\n");
        LOG_INFO("motion_track/f_motion_steps %.3f\n", st_param_.st_mpc_param.f_motion_steps);
        LOG_INFO("motion_track/f_motion_ref_cte %.3f\n", st_param_.st_mpc_param.f_motion_ref_cte);
        LOG_INFO("motion_track/f_motion_vel %.3f\n", st_param_.st_mpc_param.f_motion_vel);
        LOG_INFO("motion_track/f_motion_ref_etheta %.3f\n", st_param_.st_mpc_param.f_motion_ref_etheta);
        LOG_INFO("motion_track/f_motion_w_cte %.3f\n", st_param_.st_mpc_param.f_motion_w_cte);
        LOG_INFO("motion_track/f_motion_w_etheta %.3f\n", st_param_.st_mpc_param.f_motion_w_etheta);
        LOG_INFO("motion_track/f_motion_w_vel %.3f\n", st_param_.st_mpc_param.f_motion_w_vel);
        LOG_INFO("motion_track/f_motion_w_angvel %.3f\n", st_param_.st_mpc_param.f_motion_w_angvel);
        LOG_INFO("motion_track/f_motion_w_angvel_d %.3f\n", st_param_.st_mpc_param.f_motion_w_angvel_d);
        LOG_INFO("motion_track/f_motion_w_accel %.3f\n", st_param_.st_mpc_param.f_motion_w_accel);
        LOG_INFO("motion_track/f_motion_w_accel_d %.3f\n", st_param_.st_mpc_param.f_motion_w_accel_d);
        LOG_INFO("motion_track/f_motion_throttle %.3f\n", st_param_.st_mpc_param.f_motion_throttle);
        LOG_INFO("motion_track/f_motion_bound_value %.3f\n", st_param_.st_mpc_param.f_motion_bound_value);
        LOG_INFO("motion_track/f_motion_front_path_idx %.3f\n", st_param_.st_mpc_param.f_motion_front_path_idx);
        LOG_INFO("motion_track/f_motion_back_path_idx %.3f\n", st_param_.st_mpc_param.f_motion_back_path_idx);

        LOG_INFO("motion_track/f_motion_curve_front_path_idx %.3f\n", st_param_.st_mpc_param.f_motion_curve_front_path_idx);
        LOG_INFO("motion_track/f_motion_curve_back_path_idx %.3f\n", st_param_.st_mpc_param.f_motion_curve_back_path_idx);

        LOG_INFO("motion_track/f_motion_start_dist %.3f\n", st_param_.st_mpc_param.f_motion_start_dist);
        LOG_INFO("motion_track/f_motion_start_speed %.3f\n", st_param_.st_mpc_param.f_motion_start_speed);
        LOG_INFO("motion_track/f_motion_start_min_speed %.3f\n", st_param_.st_mpc_param.f_motion_start_min_speed);
        LOG_INFO("motion_base/b_start_control_dist_flag %d\n", st_param_.st_mpc_param.b_start_control_dist_flag);
        LOG_INFO("motion_track/f_motion_start_duration %.3f\n", st_param_.st_mpc_param.f_motion_start_duration);

        LOG_INFO("motion_track/f_motion_low_speed_std %.3f\n", st_param_.st_mpc_param.f_motion_low_speed_std);
        LOG_INFO("motion_track/f_motion_low_speed_w_cte %.3f\n", st_param_.st_mpc_param.f_motion_low_speed_w_cte);
        LOG_INFO("motion_track/f_motion_low_speed_w_etheta %.3f\n", st_param_.st_mpc_param.f_motion_low_speed_w_etheta);
        LOG_INFO("motion_track/f_motion_low_speed_w_angvel %.3f\n", st_param_.st_mpc_param.f_motion_low_speed_w_angvel);

        LOG_INFO("motion_track/f_motion_high_speed_std %.3f\n", st_param_.st_mpc_param.f_motion_high_speed_std);
        LOG_INFO("motion_track/f_motion_high_speed_w_angvel %.3f\n", st_param_.st_mpc_param.f_motion_high_speed_w_angvel);
        LOG_INFO("motion_track/f_motion_high_w_accel %.3f\n", st_param_.st_mpc_param.f_motion_high_w_accel);
        LOG_INFO("motion_track/f_motion_high_w_accel_d %.3f\n", st_param_.st_mpc_param.f_motion_high_w_accel_d);
        LOG_INFO("motion_track/f_motion_high_throttle %.3f\n", st_param_.st_mpc_param.f_motion_high_throttle);

        LOG_INFO("motion_track/f_motion_high_curve_speed_std %.3f\n", st_param_.st_mpc_param.f_motion_high_curve_speed_std);
        LOG_INFO("motion_track/f_motion_high_curve_speed_w_angvel %.3f\n", st_param_.st_mpc_param.f_motion_high_curve_speed_w_angvel);
        LOG_INFO("motion_track/f_motion_curve_w_accel %.3f\n", st_param_.st_mpc_param.f_motion_curve_w_accel);
        LOG_INFO("motion_track/f_motion_curve_w_accel_d %.3f\n", st_param_.st_mpc_param.f_motion_curve_w_accel_d);
        LOG_INFO("motion_track/f_motion_curve_throttle %.3f\n", st_param_.st_mpc_param.f_motion_curve_throttle);

        LOG_INFO("motion_arrive/f_motion_arrive_p_gain %.3f\n", st_param_.st_mpc_param.f_motion_arrive_p_gain);
        LOG_INFO("motion_arrive/f_motion_arrive_d_gain %.3f\n", st_param_.st_mpc_param.f_motion_arrive_d_gain);

        LOG_INFO("motion_track/f_motion_curve_w_angvel %.3f\n", st_param_.st_mpc_param.f_motion_curve_w_angvel);

        LOG_INFO("motion_safe/f_safe_far_from_path_to_robot_distance %.3f\n", st_param_.f_safe_far_from_path_to_robot_distance);
        LOG_INFO("motion_safe/f_control_velocity_far_from_path_m %.3f\n", st_param_.f_control_velocity_far_from_path_m);

        LOG_INFO("motion_base/n_kinematics_type %d\n", st_param_.n_kinematics_type);
        LOG_INFO("motion_arrive/f_arrived_boundary_dist_x_m %.3f\n", st_param_.f_arrived_boundary_dist_x_m);
        LOG_INFO("planner/f_angle_dividing_the_path_deg %.3f\n", st_param_.f_angle_dividing_the_path_deg);

        // 예외처리
        LOG_INFO("navigation/f_goal_receive_time_gap_sec %.3f\n", st_param_.f_goal_receive_time_gap_sec);

        LOG_INFO("//-------------------------------QUAD--------------------------------//\n");
        LOG_INFO("motion_diagonal/f_diagonal_linear_lateral_w %.3f\n", st_param_.f_diagonal_linear_lateral_w);
        LOG_INFO("motion_diagonal/f_diagonal_low_speed %.3f\n", st_param_.f_diagonal_low_speed);
        LOG_INFO("motion_diagonal/f_diagonal_low_speed_w %.3f\n", st_param_.f_diagonal_low_speed_w);
        LOG_INFO("motion_diagonal/f_diagonal_high_speed_w %.3f\n", st_param_.f_diagonal_high_speed_w);
        LOG_INFO("motion_diagonal/f_diagonal_angle_kp %.3f\n", st_param_.f_diagonal_angle_kp);
        LOG_INFO("motion_diagonal/f_diagonal_angle_kd %.3f\n", st_param_.f_diagonal_angle_kd);
        LOG_INFO("motion_diagonal/f_diagonal_angle_ki %.3f\n", st_param_.f_diagonal_angle_ki);
        LOG_INFO("motion_diagonal/f_diagonal_curve_speed_ratio %.3f\n", st_param_.f_diagonal_curve_speed_ratio);

        LOG_INFO("navigation/n_icp_error_ratio_threshold %d\n", st_param_.n_icp_error_ratio_threshold);
        LOG_INFO("navigation/n_icp_error_count_threshold %d\n", st_param_.n_icp_error_count_threshold);
        LOG_INFO("navigation/f_current_node_change_before_m %.3f\n", st_param_.f_current_node_change_before_m);
        LOG_INFO("motion_track/n_hacs_curve_speed_up_dist_cm %d\n", st_param_.n_hacs_curve_speed_up_dist_cm);

        LOG_INFO("//--------------------------LANE AVOIDANCE---------------------------//\n");
        LOG_INFO("motion_avoid/f_avoid_margin_m %.3f\n", st_param_.f_avoid_margin_m);
        LOG_INFO("motion_avoid/f_avoid_detect_obs_dir_margin_m %.3f\n", st_param_.f_avoid_detect_obs_dir_margin_m);
        LOG_INFO("motion_avoid/b_avoidance_auto_comeback_path %d\n", st_param_.b_avoidance_auto_comeback_path);
        LOG_INFO("motion_avoid/f_avoid_vel_ms %.3f\n", st_param_.f_avoid_vel_ms);
        LOG_INFO("motion_avoid/f_avoid_check_vel_ms %.3f\n", st_param_.f_avoid_check_vel_ms);
        LOG_INFO("motion_avoid/f_static_obs_check_time_sec %.3f\n", st_param_.f_static_obs_check_time_sec);
        LOG_INFO("motion_avoid/f_static_obs_check_dist_m %.3f\n", st_param_.f_static_obs_check_dist_m);

        LOG_INFO("planner_local/updated_frequency_ms %d\n", st_param_.st_local_mission_param.updated_frequency_ms);
        LOG_INFO("planner_local/map_size_x_m %.3f\n", st_param_.st_local_mission_param.map_size_x_m);
        LOG_INFO("planner_local/map_size_y_m %.3f\n", st_param_.st_local_mission_param.map_size_y_m);
        LOG_INFO("planner_local/map_res_m %.3f\n", st_param_.st_local_mission_param.map_res_m);
        LOG_INFO("planner_local/padding_size_m %.3f\n", st_param_.st_local_mission_param.padding_size_m);

        LOG_INFO("planner_avoid/n_detection_period_ms %d\n", st_param_.n_detection_period_ms);
        LOG_INFO("planner_avoid/n_avoidance_period_ms %d\n", st_param_.n_avoidance_period_ms);
        LOG_INFO("planner_avoid/f_force_comeback_start_dist_m %.3f\n", st_param_.f_force_comeback_start_dist_m);
        LOG_INFO("planner_avoid/f_force_comeback_target_dist_m %.3f\n", st_param_.f_force_comeback_target_dist_m);
        LOG_INFO(
            "planner_avoid/f_force_comeback_in_curve_added_start_dist_m %.3f\n", st_param_.f_force_comeback_in_curve_added_start_dist_m);
        LOG_INFO("planner_avoid/f_polynominal_max_curvature_m %.3f\n", st_param_.f_polynominal_max_curvature_m);
        LOG_INFO("planner_avoid/f_avoid_imposible_sec_s %.3f\n", st_param_.f_avoid_imposible_sec_s);

        LOG_INFO("planner_avoid/f_interval_m %.3f\n", st_param_.st_lane_avoidance_param.f_interval_m);
        LOG_INFO("planner_avoid/n_left_num %d\n", st_param_.st_lane_avoidance_param.n_left_num);
        LOG_INFO("planner_avoid/n_right_num %d\n", st_param_.st_lane_avoidance_param.n_right_num);
        LOG_INFO("planner_avoid/f_search_step_size_m %.3f\n", st_param_.st_lane_avoidance_param.f_search_step_size_m);
        LOG_INFO("planner_avoid/n_search_min_step %d\n", st_param_.st_lane_avoidance_param.n_search_min_step);
        LOG_INFO("planner_avoid/n_search_max_step %d\n", st_param_.st_lane_avoidance_param.n_search_max_step);
        LOG_INFO("planner_avoid/f_dubins_normal_max_curve_radius_m %.3f\n", st_param_.f_dubins_normal_max_curve_radius_m);
        LOG_INFO("planner_avoid/f_dubins_normal_min_curve_radius_m %.3f\n", st_param_.f_dubins_normal_min_curve_radius_m);
        LOG_INFO("planner_avoid/f_dubins_normal_curve_interval_m %.3f\n", st_param_.f_dubins_normal_curve_interval_m);
        LOG_INFO("planner_avoid/f_dubins_force_max_curve_radius_m %.3f\n", st_param_.f_dubins_force_max_curve_radius_m);
        LOG_INFO("planner_avoid/f_dubins_force_min_curve_radius_m %.3f\n", st_param_.f_dubins_force_min_curve_radius_m);
        LOG_INFO("planner_avoid/f_dubins_force_curve_interval_m %.3f\n", st_param_.f_dubins_force_curve_interval_m);

        LOG_INFO("upper/b_upper_use %d\n", st_param_.b_upper_use);
        LOG_INFO("mappingbot/b_control_motor_directly %d\n", st_param_.b_control_motor_directly);
    }

    LOG_INFO("UpdateParam Done");
}
