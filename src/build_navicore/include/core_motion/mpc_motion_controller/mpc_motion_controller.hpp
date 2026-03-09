#ifndef __MPC_MOTION_CONTROLLER_H__
#define __MPC_MOTION_CONTROLLER_H__

#include "mpc/MPC.hpp"
#include "core_calculator/core_calculator.hpp"
#include "motion_manager/motion_manager.hpp"
#include "param_repository/param_repository.hpp"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <iostream>
namespace NaviFra {

class MpcMotionController : public MotionManager {
public:
    enum class MPC_PARAM
    {
        NORMAL = 0,
        LOW,
        HIGH,
        HIGH_CURVE,
        CURVE
    };
    MpcMotionController();
    virtual ~MpcMotionController();

    /**
     * @brief Make Mpc V, W (Nate)
     *
     * @param o_robot_pos
     * @param o_goal_pos
     * @param vec_sensors_relative_robot
     * @return CommandVelocity_t
     */
    CommandVelocity_t GenerateCommandVelocity(
        const Pos& o_robot_pos, const Pos& o_goal_pos, const vector<SimplePos>& vec_sensors_relative_robot);

    /**
     * @brief goal arrive control
     *
     * @param st_vel
     * @param o_goal_pos
     * @param f_remain_path_dist
     * @param f_now_target_speed
     * @param f_now_speed_percent
     * @return CommandVelocity_t
     */
    CommandVelocity_t SetArriveVelocity(
        const CommandVelocity_t& st_vel, const Pos& o_goal_pos, const float& f_remain_path_dist, const float& f_now_target_speed,
        const float& f_now_speed_percent, const float& theta);
    /**
     * @brief Diagonal Control
     *
     * @param st_vel
     * @return CommandVelocity_t
     */
    CommandVelocity_t SetDiagonalVelocity(const CommandVelocity_t& st_vel, const double& theta);

    /**
     * @brief Start Control set reference velocity
     *
     * @param f_ref_vel
     * @return float
     */
    float SetStartVelocity(const float& f_ref_vel);

    /**
     * @brief Align check && control
     *
     * @param st_vel
     * @return CommandVelocity_t
     */
    CommandVelocity_t SetAlignVelocity(const CommandVelocity_t& st_vel);

    /**
     * @brief in goal check && align control
     *
     * @param st_vel
     * @return CommandVelocity_t
     */
    CommandVelocity_t SetGoalAlignVelocity(const CommandVelocity_t& st_vel, const Pos& o_goal_pos);

    /**
     * @brief Safe Control
     *
     * @param st_vel
     * @param f_robot_path_dist
     * @return CommandVelocity_t
     */
    CommandVelocity_t SetSafeVelocity(const CommandVelocity_t& st_vel, const float& f_robot_path_dist, const float& f_now_target_speed);

    /**
     * @brief adjust Speed For Heading Bias
     *
     * @param st_vel
     * @param f_heading_bias
     * @return CommandVelocity_t
     */
    CommandVelocity_t SetHeadingBiasVelocity(const CommandVelocity_t& st_vel, const float& f_heading_bias);

    /**
     * @brief limit speed
     *
     * @param st_vel
     * @return CommandVelocity_t
     */
    CommandVelocity_t LimitVelocity(const CommandVelocity_t& st_vel);

    /**
     * @brief check away from path
     *
     * @param st_vel
     * @param f_robot_path_dist
     * @return CommandVelocity_t
     */
    CommandVelocity_t CheckAwayFromPath(const CommandVelocity_t& st_vel, const float& f_robot_path_dist);

    /**
     * @brief make diagonal theta for mpc
     *
     * @return double
     */
    double SetDiagonalAngle();

    /**
     * @brief set reference speed && arc check
     *
     * @return float
     */
    float SetCurveVelocity();

    /**
     * @brief mpc param set
     *
     * @param param_mode
     */
    void SetMPCParam(const int& param_mode);

    /**
     * @brief remain path distance
     *
     * @return float
     */
    float GetRemainDistance();

    /**
     * @brief mpc state set
     *
     * @param vec_mpc_path
     * @param theta
     * @param throttle
     * @param f_ref_vel
     * @return std::vector<double>
     */
    std::vector<double> SolveMPC(const double& theta, const double& throttle, const float& f_ref_vel);
    /**
     * @brief Set the Param object to rs planner (Nate)
     *
     * @param st_parameter
     */
    void SetParameter(const Parameters_t& st_parameter);

    void SetLocalPath(const vector<Pos> vec_path);  //{vec_path_ = vec_path;}

    /**
     * @brief init before moving  (Nate)
     *
     * @param vec_path
     * @param n_current_robot_location_index
     */
    void Initialize(
        const std::vector<NaviFra::Pos>& vec_path, int n_current_robot_location_index, bool b_is_add_goal, bool b_is_align_heading = true);

    /**
     * @brief 일시 정지 후 재 출발할때 flag 리셋 용
     *
     */
    void InitializeOnlyParameter();

    /**
     * @brief Stop signal  (Nate)
     *
     */
    void StopMotion() { b_stop_flag_ = true; }

    /**
     * @brief Start signal  (Nate)
     *
     */
    void StartMotion() { b_stop_flag_ = false; }

    /**
     * @brief make Polyfit mpc path  (Nate)
     *
     * @param xvals
     * @param yvals
     * @param order
     * @return Eigen::VectorXd
     */
    Eigen::VectorXd Polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

    /**
     * @brief cal Polyeval  (Nate)
     *
     * @param coeffs
     * @param x
     * @return double
     */
    double Polyeval(Eigen::VectorXd coeffs, double x);

    /**
     * @brief Create a Reference Path object for mpc  (Nate)
     *
     * @param vec_original_path
     * @param n_current_robot_location_index
     * @param n_back_idx
     * @param n_front_idx
     * @return std::vector<NaviFra::Pos>
     */
    std::vector<NaviFra::Pos> CreateReferencePath(
        const std::vector<NaviFra::Pos>& vec_original_path, int n_current_robot_location_index, int n_back_idx, int n_front_idx);

    /**
     * @brief Create a Goal Align Path object (Nate)
     *
     * @param vec_path
     * @param o_goal_pos
     * @param n_path_size_cm
     * @param b_diagonal
     * @return true
     * @return false
     */

    void SetMaxLinearVelocity(float f_speed);
    void SetDeceleration(float f_speed_percent);
    void SetTurnDeceleration(float f_turn_percent);
    void SetCmdPercentage(CommandVelocity_t& st_cmd);
    float SetTargetVelMs(const vector<Pos>& vec_path, const int& n_robot_path_idx);

    void SetSensorMsg(const SensorMsg_t& st_all_sensor_msg);

    void SetArriveBoundary(float f_arrive_boundary_m);

    /**
     * @brief Make Mpc Motion Info (Joy)
     *
     * @return MotionInfo_t
     */
    MotionInfo_t GetMotionInfo() { return st_motion_info_; };

private:
    Parameters_t st_param_;
    MPCparam_t m_mpc_params_;
    MotionInfo_t st_motion_info_;
    MotionInfo_t st_info_;

    std::vector<NaviFra::Pos> vec_path_;
    std::vector<NaviFra::Pos> vec_global_path_;

    MPC _mpc;

    void SetMotionInfo(const MotionInfo_t st_info) { st_motion_info_ = st_info; };

    /* navigation structures */
    SensorMsg_t st_sensor_msg_;  // include sensor related information

    NaviFra::Pos o_front_pos_;
    NaviFra::Pos o_back_pos_;
    NaviFra::Pos o_acr_front_;
    NaviFra::Pos o_acr_front2_;

    Pos o_robot_pos_cal_;
    Pos o_start_align_pos_;

    double _w;  // radian/sec, angular velocity
    double _throttle;  // acceleration
    double _speed;  // speed
    bool b_stop_flag_;
    bool b_arrive_control_;
    bool b_arrive_low_control_;
    bool b_arrive_lowlow_control_;
    bool b_start_control_;
    bool b_diagonal_align_path_;

    int n_start_idx_;
    float f_start_dist_ = 0;
    bool b_entered_start_align_once_;
    bool b_arc_, b_goal_arrive_;
    bool b_speed_change_control_;
    float f_path_speed_;
    std::mutex mtx_param_;
    std::mutex mtx_path_;
    std::mutex mtx_sensor_;

    int n_robot_path_idx_;
    int n_predict_pos_idx_const_;
    float f_motion_start_speed_;
    float f_end_speed_;
    float f_total_dist_;
    float f_target_speed_ = 0;

    float f_arrive_boundary_m_ = 0;
    float f_max_speed_ = 3.0f;
    float f_speed_percent_ = 1.0f;
    float f_turn_percent_ = 1.0f;

    bool b_start_init_ = false;
    NaviFra::Pos o_robot_pos_cal_pre_;

    int n_global_path_idx_ = 0;
};
}  // namespace NaviFra
#endif
