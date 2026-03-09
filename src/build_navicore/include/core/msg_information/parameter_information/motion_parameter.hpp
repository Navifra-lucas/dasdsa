#ifndef MOTION_PARAMETER_HPP_
#define MOTION_PARAMETER_HPP_
#include <string>
#include <vector>

namespace NaviFra {
struct CommandVelocity_t {
    int n_status;
    float f_linear_speed_x_ms;
    float f_linear_speed_y_ms;
    float f_angular_speed_degs;
    bool b_reverse_motion;
    bool b_return;
    CommandVelocity_t()
    {
        n_status = 0;
        f_linear_speed_x_ms = 0.f;
        f_linear_speed_y_ms = 0.f;
        f_angular_speed_degs = 0.f;
        b_reverse_motion = false;
        b_return = false;
    }
};

struct MotionLimitParameter_t {
    float f_linear_speed_max_ms;
    float f_linear_speed_min_ms;
    float f_angular_speed_min_degs;
    float f_angular_speed_max_degs;
    bool b_diagonal_control;
    MotionLimitParameter_t()
    {
        f_linear_speed_max_ms = 0.5f;
        f_linear_speed_min_ms = 0.1f;
        f_angular_speed_min_degs = 35.f;
        f_angular_speed_max_degs = 35.f;
        b_diagonal_control = true;
    }
};

struct MotionMoveParameter_t {
    float f_linear_speed_max_ms;
    float f_linear_speed_min_ms;
    float f_angular_speed_min_degs;
    float f_angular_speed_max_degs;
    float f_linear_acc_mss;
    float f_linear_dec_mss;
    MotionMoveParameter_t()
    {
        f_linear_speed_max_ms = 0.5f;
        f_linear_speed_min_ms = 0.1f;
        f_angular_speed_min_degs = 35.f;
        f_angular_speed_max_degs = 35.f;
        f_linear_acc_mss = 1.0f;
        f_linear_dec_mss = 1.0f;
    }
};

struct SafetyMotion_t {
    float f_safety_dist_from_obs_m = 0;
    float f_goal_decel_mss = 0;
};

struct AlignMotion_t {
    float f_accel_deg = 30.0f;
    float f_decel_deg = 30.0f;
    float f_max_rot_deg_s = 25.0f;
    float f_min_rot_deg_s = 2.0f;
    float f_end_thres_deg = 1;  // end align degree threshold
    float f_start_thres_deg = 1;  // start align degree threshold
    float f_min_rot_deg_range = 5.0f;
};

struct NavCondition_t {
    float f_goal_arrived_boundary_m = 0.01f;  // goal arrived boundary to finish motion [m]
    float f_near_goal_dist_thres = 0.5f;  // distance below this value means robot is arriving at goal [m]
    float f_near_goal_speed_thres = 0.1f;  // desired velocity when arriving at goal [m/s]
    float f_curve_min_vel_ms = 0.2f;  // min velocity on curvature [m/s]
    float f_curve_max_vel_ms = 1.0f;  // max velocity on curvature [m/s]
    float f_far_from_path_m = 0.5f;  // beyond this value will turn status [m]
};

struct MotionInfo_t {
    int n_motion_status;  // 0: normal, 1: obstacle, 2: heading_aligning, 3: arrived_at_goal, 4: away_from_path
    int n_obs_control;  // 장애물 감지 감속여부 0: none 1: 예측 감속 2: 정지
    float f_obs_speed_ratio;  // 장애물 감속시 감속 비율
    float f_path_velocity;  // path의 경로 속도
    float f_mpc_input_velocity;  // 전처리가 끝나고 mpc에 들어가는 input 속도
    float f_mpc_output_linear_velocity;  // mpc 결과 선속도
    float f_mpc_output_angular_velocity;  // mpc 결과 각속도
    float f_linear_speed_x_ms;
    float f_linear_speed_y_ms;
    float f_angular_speed_degs;
    float f_align_diff_angle_deg;  // align시 남은 각도
    float f_execution_time_sec;  // GenerateCommandVelocity 함수 수행 시간
    bool b_reverse_motion;  // 후진 주행 여부
    std::string s_avoid_status;
    std::string s_mpc_param_mode;  // mpc에 적용되는 param, 고속, 저속, 곡선 등
    std::vector<std::string> vec_motion_mode;
    MotionInfo_t()
    {
        n_motion_status = 0;
        n_obs_control = 0;
        f_obs_speed_ratio = 0.f;
        f_path_velocity = 0.f;
        f_mpc_input_velocity = 0.f;
        f_mpc_output_linear_velocity = 0.f;
        f_mpc_output_angular_velocity = 0.f;
        f_linear_speed_x_ms = 0.f;
        f_linear_speed_y_ms = 0.f;
        f_angular_speed_degs = 0.f;
        f_align_diff_angle_deg = 0.f;
        f_execution_time_sec = 0.f;
        b_reverse_motion = false;
        s_avoid_status = "";
        s_mpc_param_mode = "";
        vec_motion_mode.clear();
    }
    MotionInfo_t(const MotionInfo_t& other)
    {
        n_motion_status = other.n_motion_status;
        n_obs_control = other.n_obs_control;
        f_obs_speed_ratio = other.f_obs_speed_ratio;
        f_path_velocity = other.f_path_velocity;
        f_mpc_input_velocity = other.f_mpc_input_velocity;
        f_mpc_output_linear_velocity = other.f_mpc_output_linear_velocity;
        f_mpc_output_angular_velocity = other.f_mpc_output_angular_velocity;
        f_linear_speed_x_ms = other.f_linear_speed_x_ms;
        f_linear_speed_y_ms = other.f_linear_speed_y_ms;
        f_angular_speed_degs = other.f_angular_speed_degs;
        f_align_diff_angle_deg = other.f_align_diff_angle_deg;
        f_execution_time_sec = other.f_execution_time_sec;
        b_reverse_motion = other.b_reverse_motion;
        s_avoid_status = other.s_avoid_status;
        s_mpc_param_mode = other.s_mpc_param_mode;
        vec_motion_mode = other.vec_motion_mode;
    }
};
}  // namespace NaviFra

#endif
