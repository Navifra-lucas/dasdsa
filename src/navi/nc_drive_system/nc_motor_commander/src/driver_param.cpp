#include "driver_param.hpp"

namespace NaviFra {

void DriverParam::UpdateParam(
    ros::NodeHandle& nh, ros::NodeHandle& nhp, InterfaceParameters_t& st_interface_param_, DriverParameters_t& st_driver_param_)
{
    // driver param------------------------------------------
    nhp.param<bool>("b_sim_use", st_interface_param_.b_sim_use, false);
    ros::param::param<int>("motion_base/n_kinematics_type", st_driver_param_.n_kinematics_type, 0);  // 0: DD, 1: QD , 2: SD , 3:QDOct
    ros::param::param<float>("motion_base/f_linear_speed_max_ms", st_driver_param_.f_max_linear_vel_ms, 1);
    ros::param::param<float>("motion_base/f_angular_speed_max_degs", st_driver_param_.f_max_angular_vel_degs, 15);
    ros::param::param<float>("motion_base/f_linear_max_accel_mss", st_driver_param_.f_linear_max_accel_mss, 0.5);
    ros::param::param<float>("motion_base/f_linear_max_decel_mss", st_driver_param_.f_linear_max_decel_mss, 0.5);
    ros::param::param<float>("motion_base/f_rot_max_accdecel_degss", st_driver_param_.f_rot_max_accdecel_degss, 50.0);
    ros::param::param<bool>("driver_base/b_imuodom_enable", st_driver_param_.b_imuodom_enable, false);
    ros::param::param<bool>("driver_base/b_accdecel_enable", st_driver_param_.b_accdecel_enable, true);
    ros::param::param<bool>("driver_base/b_use_brake", st_driver_param_.b_use_brake, false);
    ros::param::param<bool>("driver_base/b_external_brake_control", st_driver_param_.b_external_brake_control, false);
    ros::param::param<bool>("driver_base/b_link_brake_and_servo", st_driver_param_.b_link_brake_and_servo, false);
    ros::param::param<float>("driver_base/f_brake_release_time_ms", st_driver_param_.f_brake_release_time_ms, 200);
    ros::param::param<float>("driver_base/f_brake_lock_wait_time_ms", st_driver_param_.f_brake_lock_wait_time_ms, 2000);
    ros::param::param<float>("driver_base/f_brake_threshold_rpm", st_driver_param_.f_brake_threshold_rpm, 5);
    ros::param::param<std::string>("driver_base/s_target_frame_id", st_driver_param_.s_target_frame_id, "odom");
    ros::param::param<std::string>("driver_base/s_child_frame_id", st_driver_param_.s_child_frame_id, "base_link");
    ros::param::param<bool>("driver_base/b_tf_enable", st_driver_param_.b_tf_enable, true);
    ros::param::param<float>("driver_base/f_heartbeat_ms", st_driver_param_.f_heartbeat_ms, 300);
    ros::param::param<float>("driver_base/f_control_period_ms", st_driver_param_.f_control_period_ms, 20);

    // wheel info
    ros::param::param<float>("driver_wheel/f_FL_wheel_x_m", st_driver_param_.st_wheel_param.f_FL_wheel_x_m, 0.3);
    ros::param::param<float>("driver_wheel/f_FR_wheel_x_m", st_driver_param_.st_wheel_param.f_FR_wheel_x_m, 0.3);
    ros::param::param<float>("driver_wheel/f_RL_wheel_x_m", st_driver_param_.st_wheel_param.f_RL_wheel_x_m, -0.3);
    ros::param::param<float>("driver_wheel/f_RR_wheel_x_m", st_driver_param_.st_wheel_param.f_RR_wheel_x_m, -0.3);
    ros::param::param<float>("driver_wheel/f_FL_wheel_y_m", st_driver_param_.st_wheel_param.f_FL_wheel_y_m, 0.3);
    ros::param::param<float>("driver_wheel/f_FR_wheel_y_m", st_driver_param_.st_wheel_param.f_FR_wheel_y_m, -0.3);
    ros::param::param<float>("driver_wheel/f_RL_wheel_y_m", st_driver_param_.st_wheel_param.f_RL_wheel_y_m, 0.3);
    ros::param::param<float>("driver_wheel/f_RR_wheel_y_m", st_driver_param_.st_wheel_param.f_RR_wheel_y_m, -0.3);
    ros::param::param<float>("driver_wheel/f_FL_wheel_axis_offset_m", st_driver_param_.st_wheel_param.f_FL_wheel_axis_offset_m, 0.0);
    ros::param::param<float>("driver_wheel/f_FR_wheel_axis_offset_m", st_driver_param_.st_wheel_param.f_FR_wheel_axis_offset_m, 0.0);
    ros::param::param<float>("driver_wheel/f_RL_wheel_axis_offset_m", st_driver_param_.st_wheel_param.f_RL_wheel_axis_offset_m, 0.0);
    ros::param::param<float>("driver_wheel/f_RR_wheel_axis_offset_m", st_driver_param_.st_wheel_param.f_RR_wheel_axis_offset_m, 0.0);
    ros::param::param<float>("driver_wheel/f_FL_wheel_diameter_m", st_driver_param_.st_wheel_param.f_FL_wheel_diameter_m, 0.15);
    ros::param::param<float>("driver_wheel/f_FR_wheel_diameter_m", st_driver_param_.st_wheel_param.f_FR_wheel_diameter_m, 0.15);
    ros::param::param<float>("driver_wheel/f_RL_wheel_diameter_m", st_driver_param_.st_wheel_param.f_RL_wheel_diameter_m, 0.15);
    ros::param::param<float>("driver_wheel/f_RR_wheel_diameter_m", st_driver_param_.st_wheel_param.f_RR_wheel_diameter_m, 0.15);

    // traction param
    ros::param::param<bool>(
        "driver_traction/b_FL_traction_target_rpmd", st_driver_param_.st_traction_param.b_FL_traction_target_rpmd, true);
    ros::param::param<bool>(
        "driver_traction/b_FR_traction_target_rpmd", st_driver_param_.st_traction_param.b_FR_traction_target_rpmd, true);
    ros::param::param<bool>(
        "driver_traction/b_RL_traction_target_rpmd", st_driver_param_.st_traction_param.b_RL_traction_target_rpmd, true);
    ros::param::param<bool>(
        "driver_traction/b_RR_traction_target_rpmd", st_driver_param_.st_traction_param.b_RR_traction_target_rpmd, true);
    ros::param::param<bool>(
        "driver_traction/b_FL_traction_feedback_rpmd", st_driver_param_.st_traction_param.b_FL_traction_feedback_rpmd, true);
    ros::param::param<bool>(
        "driver_traction/b_FR_traction_feedback_rpmd", st_driver_param_.st_traction_param.b_FR_traction_feedback_rpmd, true);
    ros::param::param<bool>(
        "driver_traction/b_RL_traction_feedback_rpmd", st_driver_param_.st_traction_param.b_RL_traction_feedback_rpmd, true);
    ros::param::param<bool>(
        "driver_traction/b_RR_traction_feedback_rpmd", st_driver_param_.st_traction_param.b_RR_traction_feedback_rpmd, true);
    ros::param::param<bool>("driver_traction/b_FL_traction_encoderd", st_driver_param_.st_traction_param.b_FL_traction_encoderd, true);
    ros::param::param<bool>("driver_traction/b_FR_traction_encoderd", st_driver_param_.st_traction_param.b_FR_traction_encoderd, true);
    ros::param::param<bool>("driver_traction/b_RL_traction_encoderd", st_driver_param_.st_traction_param.b_RL_traction_encoderd, true);
    ros::param::param<bool>("driver_traction/b_RR_traction_encoderd", st_driver_param_.st_traction_param.b_RR_traction_encoderd, true);
    ros::param::param<float>("driver_traction/f_FL_traction_gear_ratio", st_driver_param_.st_traction_param.f_FL_traction_gear_ratio, 1.0);
    ros::param::param<float>("driver_traction/f_FR_traction_gear_ratio", st_driver_param_.st_traction_param.f_FR_traction_gear_ratio, 1.0);
    ros::param::param<float>("driver_traction/f_RL_traction_gear_ratio", st_driver_param_.st_traction_param.f_RL_traction_gear_ratio, 1.0);
    ros::param::param<float>("driver_traction/f_RR_traction_gear_ratio", st_driver_param_.st_traction_param.f_RR_traction_gear_ratio, 1.0);
    ros::param::param<float>(
        "driver_traction/f_FL_traction_encoder_pulse", st_driver_param_.st_traction_param.f_FL_traction_encoder_pulse, 4096.0);
    ros::param::param<float>(
        "driver_traction/f_FR_traction_encoder_pulse", st_driver_param_.st_traction_param.f_FR_traction_encoder_pulse, 4096.0);
    ros::param::param<float>(
        "driver_traction/f_RL_traction_encoder_pulse", st_driver_param_.st_traction_param.f_RL_traction_encoder_pulse, 4096.0);
    ros::param::param<float>(
        "driver_traction/f_RR_traction_encoder_pulse", st_driver_param_.st_traction_param.f_RR_traction_encoder_pulse, 4096.0);
    ros::param::param<float>("driver_traction/f_FL_traction_rpm_min", st_driver_param_.st_traction_param.f_FL_traction_rpm_min, 0);
    ros::param::param<float>("driver_traction/f_FR_traction_rpm_min", st_driver_param_.st_traction_param.f_FR_traction_rpm_min, 0);
    ros::param::param<float>("driver_traction/f_RL_traction_rpm_min", st_driver_param_.st_traction_param.f_RL_traction_rpm_min, 0);
    ros::param::param<float>("driver_traction/f_RR_traction_rpm_min", st_driver_param_.st_traction_param.f_RR_traction_rpm_min, 0);
    ros::param::param<float>("driver_traction/f_FL_traction_rpm_max", st_driver_param_.st_traction_param.f_FL_traction_rpm_max, 2000);
    ros::param::param<float>("driver_traction/f_FR_traction_rpm_max", st_driver_param_.st_traction_param.f_FR_traction_rpm_max, 2000);
    ros::param::param<float>("driver_traction/f_RL_traction_rpm_max", st_driver_param_.st_traction_param.f_RL_traction_rpm_max, 2000);
    ros::param::param<float>("driver_traction/f_RR_traction_rpm_max", st_driver_param_.st_traction_param.f_RR_traction_rpm_max, 2000);
    ros::param::param<float>(
        "driver_traction/f_traction_overcurrent_amp_std", st_driver_param_.st_traction_param.f_traction_overcurrent_amp_std, 130);
    ros::param::param<float>(
        "driver_traction/f_traction_overcurrent_time_std", st_driver_param_.st_traction_param.f_traction_overcurrent_time_std, 3);
    ros::param::param<std::string>("driver_traction/s_target_unit", st_driver_param_.st_traction_param.s_target_unit, "rpm");
    ros::param::param<int>("driver_traction/n_target_pulse", st_driver_param_.st_traction_param.n_target_pulse, 1);

    // steer param
    ros::param::param<bool>("driver_steer/b_FL_target_steer_angled", st_driver_param_.st_steer_param.b_FL_target_steer_angled, true);
    ros::param::param<bool>("driver_steer/b_FR_target_steer_angled", st_driver_param_.st_steer_param.b_FR_target_steer_angled, true);
    ros::param::param<bool>("driver_steer/b_RL_target_steer_angled", st_driver_param_.st_steer_param.b_RL_target_steer_angled, true);
    ros::param::param<bool>("driver_steer/b_RR_target_steer_angled", st_driver_param_.st_steer_param.b_RR_target_steer_angled, true);
    ros::param::param<bool>("driver_steer/b_FL_feedback_steer_angled", st_driver_param_.st_steer_param.b_FL_feedback_steer_angled, true);
    ros::param::param<bool>("driver_steer/b_FR_feedback_steer_angled", st_driver_param_.st_steer_param.b_FR_feedback_steer_angled, true);
    ros::param::param<bool>("driver_steer/b_RL_feedback_steer_angled", st_driver_param_.st_steer_param.b_RL_feedback_steer_angled, true);
    ros::param::param<bool>("driver_steer/b_RR_feedback_steer_angled", st_driver_param_.st_steer_param.b_RR_feedback_steer_angled, true);
    ros::param::param<float>("driver_steer/f_steer_stop_thr_deg", st_driver_param_.st_steer_param.f_steer_stop_thr_deg, 5.0);
    ros::param::param<float>("driver_steer/f_steer_start_thr_deg", st_driver_param_.st_steer_param.f_steer_start_thr_deg, 10.0);
    ros::param::param<float>("driver_steer/f_FL_steer_gear_ratio", st_driver_param_.st_steer_param.f_FL_steer_gear_ratio, 1.0);
    ros::param::param<float>("driver_steer/f_FR_steer_gear_ratio", st_driver_param_.st_steer_param.f_FR_steer_gear_ratio, 1.0);
    ros::param::param<float>("driver_steer/f_RL_steer_gear_ratio", st_driver_param_.st_steer_param.f_RL_steer_gear_ratio, 1.0);
    ros::param::param<float>("driver_steer/f_RR_steer_gear_ratio", st_driver_param_.st_steer_param.f_RR_steer_gear_ratio, 1.0);
    ros::param::param<float>("driver_steer/f_FL_steer_encoder_pulse", st_driver_param_.st_steer_param.f_FL_steer_encoder_pulse, 4096.0);
    ros::param::param<float>("driver_steer/f_FR_steer_encoder_pulse", st_driver_param_.st_steer_param.f_FR_steer_encoder_pulse, 4096.0);
    ros::param::param<float>("driver_steer/f_RL_steer_encoder_pulse", st_driver_param_.st_steer_param.f_RL_steer_encoder_pulse, 4096.0);
    ros::param::param<float>("driver_steer/f_RR_steer_encoder_pulse", st_driver_param_.st_steer_param.f_RR_steer_encoder_pulse, 4096.0);
    ros::param::param<float>("driver_steer/f_FL_steer_max_angle_deg", st_driver_param_.st_steer_param.f_FL_steer_max_angle_deg, 120);
    ros::param::param<float>("driver_steer/f_FR_steer_max_angle_deg", st_driver_param_.st_steer_param.f_FR_steer_max_angle_deg, 120);
    ros::param::param<float>("driver_steer/f_RL_steer_max_angle_deg", st_driver_param_.st_steer_param.f_RL_steer_max_angle_deg, 120);
    ros::param::param<float>("driver_steer/f_RR_steer_max_angle_deg", st_driver_param_.st_steer_param.f_RR_steer_max_angle_deg, 120);
    ros::param::param<float>("driver_steer/f_FL_steer_min_angle_deg", st_driver_param_.st_steer_param.f_FL_steer_min_angle_deg, -120);
    ros::param::param<float>("driver_steer/f_FR_steer_min_angle_deg", st_driver_param_.st_steer_param.f_FR_steer_min_angle_deg, -120);
    ros::param::param<float>("driver_steer/f_RL_steer_min_angle_deg", st_driver_param_.st_steer_param.f_RL_steer_min_angle_deg, -120);
    ros::param::param<float>("driver_steer/f_RR_steer_min_angle_deg", st_driver_param_.st_steer_param.f_RR_steer_min_angle_deg, -120);
    ros::param::param<float>("driver_steer/f_FL_steer_rpm_min", st_driver_param_.st_steer_param.f_FL_steer_rpm_min, 0.0);
    ros::param::param<float>("driver_steer/f_FR_steer_rpm_min", st_driver_param_.st_steer_param.f_FR_steer_rpm_min, 0.0);
    ros::param::param<float>("driver_steer/f_RL_steer_rpm_min", st_driver_param_.st_steer_param.f_RL_steer_rpm_min, 0.0);
    ros::param::param<float>("driver_steer/f_RR_steer_rpm_min", st_driver_param_.st_steer_param.f_RR_steer_rpm_min, 0.0);
    ros::param::param<float>("driver_steer/f_FL_steer_rpm_max", st_driver_param_.st_steer_param.f_FL_steer_rpm_max, 250.0);
    ros::param::param<float>("driver_steer/f_FR_steer_rpm_max", st_driver_param_.st_steer_param.f_FR_steer_rpm_max, 250.0);
    ros::param::param<float>("driver_steer/f_RL_steer_rpm_max", st_driver_param_.st_steer_param.f_RL_steer_rpm_max, 250.0);
    ros::param::param<float>("driver_steer/f_RR_steer_rpm_max", st_driver_param_.st_steer_param.f_RR_steer_rpm_max, 250.0);
    ros::param::param<float>("driver_steer/f_steer_pid_convergence_deg", st_driver_param_.st_steer_param.f_steer_pid_convergence_deg, 0.02);
    ros::param::param<float>("driver_steer/f_steer_speed_std_ms", st_driver_param_.st_steer_param.f_steer_speed_std_ms, 0.5);
    ros::param::param<float>(
        "driver_steer/f_steer_low_speed_pid_kp_gain", st_driver_param_.st_steer_param.f_steer_low_speed_pid_kp_gain, 10.0);
    ros::param::param<float>(
        "driver_steer/f_steer_low_speed_pid_ki_gain", st_driver_param_.st_steer_param.f_steer_low_speed_pid_ki_gain, 0.0);
    ros::param::param<float>(
        "driver_steer/f_steer_low_speed_pid_kd_gain", st_driver_param_.st_steer_param.f_steer_low_speed_pid_kd_gain, 0.0);
    ros::param::param<float>(
        "driver_steer/f_steer_high_speed_pid_kp_gain", st_driver_param_.st_steer_param.f_steer_high_speed_pid_kp_gain, 10.0);
    ros::param::param<float>(
        "driver_steer/f_steer_high_speed_pid_ki_gain", st_driver_param_.st_steer_param.f_steer_high_speed_pid_ki_gain, 0.0);
    ros::param::param<float>(
        "driver_steer/f_steer_high_speed_pid_kd_gain", st_driver_param_.st_steer_param.f_steer_high_speed_pid_kd_gain, 0.0);
    ros::param::param<float>("driver_steer/f_steer_overcurrent_amp_std", st_driver_param_.st_steer_param.f_steer_overcurrent_amp_std, 130);
    ros::param::param<float>("driver_steer/f_steer_overcurrent_time_std", st_driver_param_.st_steer_param.f_steer_overcurrent_time_std, 3);
    ros::param::param<int>("driver_steer/n_steer_control_mode", st_driver_param_.st_steer_param.n_steer_control_mode, 0);
    ros::param::param<std::string>("driver_steer/s_target_unit", st_driver_param_.st_steer_param.s_target_unit, "rpm");
    ros::param::param<int>("driver_steer/n_target_pulse", st_driver_param_.st_steer_param.n_target_pulse, 1);

    // upper param
    ros::param::param<bool>("driver_upper/b_turntable_use", st_driver_param_.st_upper_param.b_turntable_use, false);
    ros::param::param<bool>(
        "driver_upper/b_turntable_reference_position_is_initial_position",
        st_driver_param_.st_upper_param.b_turntable_reference_position_is_initial_position, false);
    ros::param::param<int>("driver_upper/n_turntable_reference_position", st_driver_param_.st_upper_param.n_turntable_reference_position, 0);
    ros::param::param<float>("driver_upper/f_turntable_ppr", st_driver_param_.st_upper_param.f_turntable_ppr, 10000.0);
    ros::param::param<float>("driver_upper/f_turntable_gear_ratio", st_driver_param_.st_upper_param.f_turntable_gear_ratio, 1.0);

    ros::param::param<float>(
        "driver_upper/f_turntable_overcurrent_amp_std", st_driver_param_.st_upper_param.f_turntable_overcurrent_amp_std, 130.0);
    ros::param::param<float>(
        "driver_upper/f_turntable_overcurrent_time_std", st_driver_param_.st_upper_param.f_turntable_overcurrent_time_std, 3.0);

    // etc param
    ros::param::param<bool>("driver_etc/b_abs_steer_encoder_use", st_driver_param_.st_etc_param.b_abs_steer_encoder_use, false);
    ros::param::param<bool>("driver_etc/b_FL_steer_absfeedback_angled", st_driver_param_.st_etc_param.b_FL_steer_absfeedback_angled, true);
    ros::param::param<bool>("driver_etc/b_FR_steer_absfeedback_angled", st_driver_param_.st_etc_param.b_FR_steer_absfeedback_angled, true);
    ros::param::param<bool>("driver_etc/b_RL_steer_absfeedback_angled", st_driver_param_.st_etc_param.b_RL_steer_absfeedback_angled, true);
    ros::param::param<bool>("driver_etc/b_RR_steer_absfeedback_angled", st_driver_param_.st_etc_param.b_RR_steer_absfeedback_angled, true);
    ros::param::param<float>("driver_etc/f_FL_abssteer_offset", st_driver_param_.st_etc_param.f_FL_abssteer_offset, 0.0);
    ros::param::param<float>("driver_etc/f_FR_abssteer_offset", st_driver_param_.st_etc_param.f_FR_abssteer_offset, 0.0);
    ros::param::param<float>("driver_etc/f_RL_abssteer_offset", st_driver_param_.st_etc_param.f_RL_abssteer_offset, 0.0);
    ros::param::param<float>("driver_etc/f_RR_abssteer_offset", st_driver_param_.st_etc_param.f_RR_abssteer_offset, 0.0);
    ros::param::param<float>("driver_etc/f_FL_abssteer_pulse", st_driver_param_.st_etc_param.f_FL_abssteer_pulse, 0.0);
    ros::param::param<float>("driver_etc/f_FR_abssteer_pulse", st_driver_param_.st_etc_param.f_FR_abssteer_pulse, 0.0);
    ros::param::param<float>("driver_etc/f_RL_abssteer_pulse", st_driver_param_.st_etc_param.f_RL_abssteer_pulse, 0.0);
    ros::param::param<float>("driver_etc/f_RR_abssteer_pulse", st_driver_param_.st_etc_param.f_RR_abssteer_pulse, 0.0);
    ros::param::param<float>("driver_etc/f_FL_abssteer_gearratio", st_driver_param_.st_etc_param.f_FL_abssteer_gearratio, 1.0);
    ros::param::param<float>("driver_etc/f_FR_abssteer_gearratio", st_driver_param_.st_etc_param.f_FR_abssteer_gearratio, 1.0);
    ros::param::param<float>("driver_etc/f_RL_abssteer_gearratio", st_driver_param_.st_etc_param.f_RL_abssteer_gearratio, 1.0);
    ros::param::param<float>("driver_etc/f_RR_abssteer_gearratio", st_driver_param_.st_etc_param.f_RR_abssteer_gearratio, 1.0);

    // etc warn param
    ros::param::param<int>("driver_etc/n_rpm_warning_std", st_driver_param_.st_etc_param.n_rpm_warning_std, 100);
    ros::param::param<int>("driver_etc/n_rpm_warning_count_std", st_driver_param_.st_etc_param.n_rpm_warning_count_std, 30);

    // -------------------------------------------------interface-------------------------------------------------
    ros::param::param<bool>("pcan/b_pcan_use", st_interface_param_.b_pcan_use, false);
    ros::param::param<bool>("can/b_can_use", st_interface_param_.b_can_use, false);

    // Serial
    ros::param::param<bool>("serial/b_serial_use", st_interface_param_.b_serial_use, false);
    ros::param::param<std::string>("serial/s_serial_tty", st_interface_param_.st_serial_param.s_serial_tty, "/dev/ttyUSB_Serial");
    ros::param::param<int>("serial/n_serial_mode", st_interface_param_.st_serial_param.n_serial_mode, SERIAL_MODE::SERIAL_COMODO);
    ros::param::param<int>(
        "serial/n_serial_baudrate", st_interface_param_.st_serial_param.n_serial_baudrate, SERIAL_BAUDRATE::NC_SERIAL_BAUD_57600);
    ros::param::param<int>("serial/n_serial_rec_buf", st_interface_param_.st_serial_param.n_serial_rec_buf, 40);

    // MB5U
    ros::param::param<bool>("MB5U/b_MB5U_use", st_interface_param_.b_MB5U_use, false);
    ros::param::param<int>("MB5U/n_MB5U_baudrate", st_interface_param_.st_MB5U_param.n_MB5U_baudrate, MB5U_BAUDRATE::NC_MB5U_BAUD_B9600);
    ros::param::param<std::string>("MB5U/s_FL_MB5U_serial_num", st_interface_param_.st_MB5U_param.s_FL_MB5U_serial_num, "0000");
    ros::param::param<std::string>("MB5U/s_RR_MB5U_serial_num", st_interface_param_.st_MB5U_param.s_RR_MB5U_serial_num, "0000");

    // MB5U dd
    ros::param::param<int>("MB5U/n_dd_FL_MB5U_channel", st_interface_param_.st_MB5U_param.n_dd_FL_MB5U_channel, 1);
    ros::param::param<int>("MB5U/n_dd_RR_MB5U_channel", st_interface_param_.st_MB5U_param.n_dd_RR_MB5U_channel, 2);

    // MB5U qd
    ros::param::param<int>("MB5U/n_qd_FL_MB5U_channel", st_interface_param_.st_MB5U_param.n_qd_FL_MB5U_channel, 1);
    ros::param::param<int>("MB5U/n_qd_RR_MB5U_channel", st_interface_param_.st_MB5U_param.n_qd_RR_MB5U_channel, 2);

    // MB5U sd
    ros::param::param<int>("MB5U/n_sd_FL_MB5U_channel", st_interface_param_.st_MB5U_param.n_sd_FL_MB5U_channel, 1);
}

};  // namespace NaviFra
