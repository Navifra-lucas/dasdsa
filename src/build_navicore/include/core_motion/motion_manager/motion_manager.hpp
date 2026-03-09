#ifndef NAVIFRA_MOTION_PARENT_H
#define NAVIFRA_MOTION_PARENT_H

#include "msg_information/parameter_information/motion_parameter.hpp"
#include "msg_information/parameter_information/param_msg.hpp"
#include "msg_information/sensor_information/scan.hpp"
#include "msg_information/sensor_information/sensor_msg.hpp"
#include "param_repository/param_repository.hpp"

#include <functional>
#include <iostream>

namespace NaviFra {
class MotionManager {
public:
    enum MOTION_STATUS
    {
        NORMAL = 0,
        OBSTACLE,
        HEADING_ALIGNING,
        ARRIVED_AT_GOAL,
        AWAY_FROM_PATH
    };
    enum NODE_ALIGNMENT_TYPE
    {
        NONE = 0,
        SPIN,  // self positioning
        CURVE  // turning toward goal
    };

    MotionManager() {}
    virtual ~MotionManager() {}

    /**
     * @brief Called in 'AutoMoveMission::MoveToGoal'
     * Main velocity command genration
     */
    virtual CommandVelocity_t GenerateCommandVelocity(
        const Pos& o_robot_pos, const Pos& o_goal_pos, const vector<SimplePos>& vec_local_pc_all_data) = 0;

    virtual void Initialize(const vector<Pos>& vec_path, int n_robot_idx = 0, bool b_addgoal_flag = false, bool b_align_flag = true);

    /**
     * @brief Called in 'AutoMoveMission::StopMotion' (luna)
     */
    virtual void InitializeOnlyParameter() = 0;

    /**
     * @brief Called in 'AutoMoveMission::ResumeMission, AutoMoveMission::MoveToGoal' (luna)
     */
    virtual void StartMotion() = 0;

    /**
     * @brief Stop tracking motion
     * Sets command to zero and n_status ARRIVED_GOAL
     * Called in 'AutoMoveMission::StopMotion' (luna)
     */
    virtual void StopMotion() = 0;

    /**
     * @brief Get Mpc Motion Info (Joy)
     *
     * @return MotionInfo_t
     */
    virtual MotionInfo_t GetMotionInfo() = 0;

    /* Set functions */
    /**
     * @brief Called in 'AutoMoveMission::SetNaviParam'
     * Override this function
     * @param st_param parameter
     */
    virtual void SetParameter(const Parameters_t& st_param) = 0;

    virtual void SetLocalPath(const vector<Pos> vec_path) = 0;

    /**
     * @brief Set sensor msgs to private global sensor variable (luna)
     * @param st_all_sensor_msg sensor msg from upper class
     */
    virtual void SetSensorMsg(const SensorMsg_t& st_all_sensor_msg) = 0;

    virtual void SetMaxLinearVelocity(float f_speed){};
    virtual void SetDeceleration(float f_speed_percent){};
    virtual void SetTurnDeceleration(float f_turn_percent){};
    virtual void SetArriveBoundary(float f_arrive_boundary_m){};

    void SetPath(const std::vector<NaviFra::Pos>& vec_path);
    void SetAlignStartHeadingFlag(NODE_ALIGNMENT_TYPE e_flag);
    void SetAlignStartHeadingFlag(bool b_flag);
    virtual void SetCurvature(vector<Pos>& vec_path){};
    virtual void SetMaxVel(float f_speed){};
    virtual void SetVelPercent(float f_speed_percent){};
    bool GetAlignFlagState();
    NODE_ALIGNMENT_TYPE GetAlignGoalHeadingFlag();
    NODE_ALIGNMENT_TYPE GetAlignStartHeadingFlag();
    static CommandVelocity_t RegenerateVelByYawBias_(
        const CommandVelocity_t& st_vel, const vector<Pos>& vec_path, const int n_robot_idx, const Pos& o_robot_pos, const Pos& o_front_pos,
        const Pos& o_back_pos, float f_yaw_bias_rad, int n_slow_control, const Parameters_t& st_param);
    void InitAlignGoal(const NaviFra::Pos& o_start_pos, const NaviFra::Pos& o_target_pos, const float& f_start_w_vel = 0);
    CommandVelocity_t AllignGoalDeg(
        const NaviFra::Pos& o_robot_pos, const NaviFra::Pos& o_target_pos, const float& f_allign_deg, bool b_goal_align = false);
    Parameters_t GetParameters() { return st_parameter_; }

    /* robot type */
    int n_robot_type = 0;

    /* align related */
    NaviFra::Pos o_diff_goal_heading_pos_;
    NODE_ALIGNMENT_TYPE b_need_to_start_align_ = NODE_ALIGNMENT_TYPE::NONE;
    NODE_ALIGNMENT_TYPE b_need_to_end_align_ = NODE_ALIGNMENT_TYPE::NONE;
    float f_align_threshold_for_start_deg_ = 1.0f;
    float f_align_threshold_for_end_deg_ = 1.0f;

    /* global & local pointclouds */
    vector<SimplePos> vec_global_pc;

    /* global & local path */
    vector<Pos> vec_global_path_, vec_local_path_;
    std::vector<NaviFra::Pos> vec_path_m_;

    bool b_spinturn_flag_ = false;
    float f_start_w_vel_;

    SensorMsg_t st_sensor_msg_;  // include sensor related information
    struct AlignMotion_t {
        bool b_start_flag = false;
        float f_accel_deg = 30.0f;
        float f_decel_deg = 30.0f;
        float f_max_rot_deg_s = 25.0f;
        float f_min_rot_deg_s = 2.0f;
        float f_min_rot_end_deg_s = 2.0f;
        float f_arrive_deg = 1;
        const float f_max_angle = 180.0f;
        float f_min_angle = 10.0f;
    } st_align_motion_;

private:
    MotionLimitParameter_t st_motion_param_;  // limitations of actual robot
    float f_obs_min_dist_;  // (avoid mode) obstacle minimum distance

    /* protect data */
    mutex mtx_param_, mtx_path_;

    Parameters_t st_parameter_;

    /* time checking */
    std::chrono::steady_clock::time_point tp_time_;
    bool b_start_delay_flag_ = true;
    float f_time_dur_ = 0.0f;
};

}  // namespace NaviFra
#endif /*NAVIFRA_MOTION_PARENT_H*/
