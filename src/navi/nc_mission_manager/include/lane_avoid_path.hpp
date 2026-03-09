#ifndef LANE_AVOID_PATH_HPP
#define LANE_AVOID_PATH_HPP

#include "avoid_structure.hpp"
#include "collision_detector.hpp"
#include "core_calculator/core_calculator.hpp"
#include "debug/debug_visualizer.hpp"
#include "dubins/dubins.hpp"
#include "msg_information/parameter_information/param_msg.hpp"
#include "msg_information/sensor_information/sensor_msg.hpp"
#include "polynominal_path.hpp"

#include <chrono>
#include <queue>

namespace NaviFra {
class LaneAvoidPath {
public:
    LaneAvoidPath(){};
    virtual ~LaneAvoidPath(){};

    void SetParam(const Parameters_t& st_param);
    void SetSensorMsg(const SensorMsg_t& st_sensor_msgs);
    void SetForceComebackTargetIdx(int n_force_comeback_target_idx);

    Avoid_path_result CheckAvoidPath(
        const std::vector<NaviFra::Pos>& vec_ref_path, const NaviFra::Pos& o_start_pos, Lane_info_t st_lane_info, int n_current_lane_num,
        float f_min_dist);
    Avoid_path_result CheckAvoidPathTwice(
        const std::vector<NaviFra::Pos>& vec_ref_path, const NaviFra::Pos& o_start_pos, Lane_info_t st_lane_info, int n_current_lane_num,
        float f_min_dist);
    std::vector<NaviFra::Pos> FindComebackPath(
        const std::vector<NaviFra::Pos>& vec_ref_path, const NaviFra::Pos& o_start_pos, Lane_info_t st_lane_info, int n_current_lane_num,
        float f_min_dist);
    std::vector<NaviFra::Pos> ForceComebackPath(
        const std::vector<NaviFra::Pos>& vec_ref_path, const NaviFra::Pos& o_start_pos, const NaviFra::Pos& o_end_pos,
        Lane_info_t st_lane_info, int n_current_lane_num, float f_min_dist);
    std::vector<NaviFra::Pos> ForceFindPath(
        const std::vector<NaviFra::Pos>& vec_ref_path, const NaviFra::Pos& o_start_pos, const NaviFra::Pos& o_end_pos,
        Lane_info_t st_lane_info, int n_current_lane_num, float f_min_dist);
    Avoid_path_result ExtendAvoidPath(
        const std::vector<NaviFra::Pos>& vec_ref_path, const std::vector<NaviFra::Pos>& vec_current_path, int n_current_path_idx,
        Lane_info_t st_lane_info, int n_current_lane_num, float f_min_dist);
    std::vector<NaviFra::Avoid_sector_t> DevideMissionSector(const std::vector<NaviFra::Pos>& vec_path);
    std::vector<NaviFra::Pos> MakeAvoidArcPath(const NaviFra::Pos& o_start_arc_node, const NaviFra::Pos& o_end_arc_node);
    Extend_path_result MakeRemainPath(const std::vector<NaviFra::Pos>& vec_tmp_path, int n_target_lane_num, Lane_info_t st_lane_info);
    void SetCurvature(vector<Pos>& vec_path);
    void SetFullSm(vector<Pos>& vec_path);
    void SetAvoidStatus(vector<Pos>& vec_path, bool b_state);
    std::vector<NaviFra::Pos> MakeDubinsPath(
        const NaviFra::Pos& o_dubins_start_pos, const NaviFra::Pos& o_dubins_end_pos, float f_radius, float f_linear_speed_ms,
        bool b_force_comeback);
    std::vector<NaviFra::Pos> MakePolynominalPath(const NaviFra::Pos& o_start_pos, const NaviFra::Pos& o_end_pos);
    std::vector<int> SetlaneSearchOrder(
        const std::vector<NaviFra::Pos>& vec_ref_path, const NaviFra::Pos& o_start_pos, int n_current_lane_num, Lane_info_t st_lane_info);
    void SetCollisionDetectorPtr(std::shared_ptr<CollisionDetector> o_collision_detector_ptr);
    void SetBackwardFlag(bool b_move_backward_flag);
    int GetSampleIdx(int n_s, int n_d);
    void ResetSamples();
    int CalcCost(const std::vector<NaviFra::Pos>& path, int n_lane_num);

private:
    Parameters_t st_param_;
    SensorMsg_t st_sensor_msgs_;
    NaviFra::Pos o_path_start_pos_;
    std::mutex mtx_param_;
    std::mutex mtx_sensor_msg_;
    std::mutex mtx_remain_obs_;
    float f_diagonal_deg_ = 0;
    float f_robot_width_ = 0;
    int n_force_comeback_target_idx_ = 0;
    std::vector<NaviFra::Pos> vec_prev_avoid_path_;
    std::vector<NaviFra::Pos> vec_remain_obs_;
    std::vector<std::chrono::steady_clock::time_point> vec_detect_obs_time_;
    std::vector<NaviFra::Pos> vec_current_avoid_path_;
    std::shared_ptr<CollisionDetector> o_collision_detector_ptr_;
    bool b_move_backward_flag_ = false;
    std::vector<Avoid_sample_point> vec_avoid_sample_point_;
    std::vector<Avoid_sample_edge> vec_avoid_sample_edge_;
};
}  // namespace NaviFra
#endif