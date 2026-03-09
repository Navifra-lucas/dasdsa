/*
 * @file	: mission_manager.hpp
 * @date	: Jan. 1, 2022
 * @author	: "Joongtae, Park(brain)" (founder@navifra.com)"
 * @brief	: 최종 목적지 정보만 가지고 있는 명령을 여러개의 미션으로 분할하여 관리함
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2022 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_MISSION_MANAGER_H_
#define NAVIFRA_MISSION_MANAGER_H_

#include "auto_move_mission.hpp"
#include "core/util/logger.hpp"
#include "core_msgs/MapDB.h"
#include "debug/debug_visualizer.hpp"
#include "map/localmap/localmap.h"
#include "map_based_path_planner/src/globalmap.hpp"
#include "mission_manager/mission.hpp"
#include "mission_manager/mission_repository.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include "node/navi_node.hpp"
#include "node_path_divider.hpp"
#include "param_repository/param_repository.hpp"
#include "pos/pos.hpp"

#include <iostream>

namespace NaviFra {
class MissionManager {
public:
    enum MISSION_MSG
    {
        MSG_NAVI_GLOBAL_MAP = 0,
        MSG_NAVIGATION_PARAM,
        MSG_TOPOLOGICAL_MAP,
        MSG_ODOM,
        MSG_STATUS,
        MSG_POLYGON_LIST,
        MSG_COSTMAP_BUILDER,
        MSG_NAVI_GLOBAL_MAP_NO_BLOCKING,
        MSG_TWIST,
        MSG_SPEED_PERCENT,
        MSG_TURN_PERCENT,
        MSG_NAVI_LOCAL_MAP,
        MSG_ARRIVE_BOUNDARY
    };

public:
    const int ERR_PATH_PLAN_FAILURE = -1;
    const int ERR_MISSION_GENERATION_FAILURE = -2;
    const float MIN_DIST_BT_GOAL_ROBOT_M = 0.5;

    MissionManager();
    virtual ~MissionManager();

    void ResumeMission();
    void SuspendMission();
    void TerminateMission(bool b_clear_mission = true);
    void ClearMission();
    std::vector<NaviFra::PathDescription> MakeDevidedPath(const vector<NaviNode>& o_msg);

    int StartMission(const vector<NaviNode>& o_msg);
    void PathStack(
        std::vector<NaviFra::PathDescription>& vec_mission_stack, std::vector<NaviFra::Pos>& vec_path, string goal_node_id,
        vector<NaviNode> vec_node_origin, string goal_name);

    /**
     * @brief 외부로부터 계산된 로봇 위치값을 설정
     *
     * @param o_robot_pos
     */
    void SetRobotPos(const Pos& o_robot_pos);

    /**
     * @brief Set the Msg object
     *
     * @param msg_id
     * @param any_type_var
     */
    void SetMsg(int msg_id, const boost::any& any_type_var);

    /**
     * @brief Set the Sensor Msg object
     *
     * @param st_sensor_msg
     */
    void SetSensorMsg(const SensorMsg_t& st_sensor_msg);

    void SetLoadState(const bool& b_msg);
    void SetForkPosition(int n_pos);
    void SetUseLccs(const bool& b_msg);
    void SetGoalArrivedFlag(const bool& b_msg);
    /**
     * @brief
     *
     * @param s_cb_func_name
     * @param pt_func_
     * @return true
     * @return false
     */
    bool RegistCbFunc(const std::string& s_cb_func_name, const std::function<void(const boost::any&)>& pt_func);
    bool RegistCbFunc(int n_cb_name, const std::function<void(const boost::any&)>& pt_func);

    void SetLocalMapPtr(std::shared_ptr<const std::vector<int8_t>> map_ptr);

    bool GetAvoidPermission() { return o_auto_move_mission_.GetAvoidPermission(); };
    void SetAvoidPermission(bool b_permission) { o_auto_move_mission_.SetAvoidPermission(b_permission); };

private:
    void SetParameter(const boost::any& any_type_var);

    /**
     * @brief 입력받은 미션 stack을 하나씩 처리하는 함수
     *
     * @param vec_mission_stack
     */
    void CarryoutMission();

    /**
     * @brief 미션을 수행하는 모듈에서 임무수행에 대한 완료 메세지를 받음
     *
     * @param any_type_var
     */
    void RecvMissionCompleteCbMsg(const boost::any& any_type_var);

    std::vector<Pos> MakeArcPath(const NaviFra::Pos& pos_1, const NaviFra::Pos& pos_2, const NaviFra::Pos& pos_3);
    void BuildLocalMap();
    void RecvMapDB(const core_msgs::MapDB::ConstPtr& msg);
    void RobotSpeedCallback(const nav_msgs::Odometry::ConstPtr& msg);

private:
    NaviFra::MissionRepository mission_repo_;
    NaviFra::AutoMoveMission o_auto_move_mission_;
    bool b_mission_complete_flag_;

    Pos o_robot_pos_;
    Pos o_goal_pos_pre_;
    vector<Pos> vec_global_path_;

    Parameters_t st_param_;
    ros::NodeHandle node_handle_;
    ros::Subscriber map_db_sub_;
    ros::Subscriber robot_speed_sub_;

    ros::Publisher globalmap_pub_;
    ros::Publisher static_localmap_pub_;
    ros::Publisher dynamic_localmap_pub_;
    ros::Publisher localmap_pub_;

    std::mutex mtx_mission_wait_;  //미션 끝날때까지 대기하는 루틴을 위한 변수
    std::condition_variable cond_var_mission_wait_;  //미션 끝날때까지 대기하는 루틴을 위한 변수

    std::mutex mtx_termination_wait_;
    std::condition_variable cond_var_termination_wait_;  // termination이 끝날때까지 대기하는 루틴을 위한 변수
    bool b_termiation_completed_flag_;

    NaviFra::NodePathDivider node_path_divider_;

    int n_now_mission_num_ = 0;

    std::string str_goal_id_pre_;
    bool b_addgoal_flag_;

    bool b_thread_on_ = false;
    std::mutex mtx_thread_on_;
    vector<NaviFra::NaviNode> vec_last_topological_map_;

    boost::thread th1_;
    boost::thread th_costmap_builder_;
    bool b_arrive_flag_ = false;
    bool b_start_flag_ = false;

    std::chrono::steady_clock::time_point tp_th_time_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_teminate_time_ = std::chrono::steady_clock::now();

    NaviFra::Pos o_goal_pos_;
    // NaviFra::Pos o_goal_pos_pre_;
    AutoMoveMission::MISSION_STATUS n_mission_state_ = AutoMoveMission::MISSION_STATUS::IDLE;  // for making local map only in travel status

    std::vector<Pos> vec_local_path_;

    bool b_docking_status_ = false;

    struct LocaMission_t {
        int update_frequency_ms = 500;
        float map_size_x_m = 5.0f;
        float map_size_y_m = 5.0f;
        float map_res_m = 0.1f;
        float padding_size_m = 0.3f;
    } st_local_mission_;
    NaviFra::GlobalMap o_globalmap_;
    nav_msgs::OccupancyGrid ros_grid_map_;

    // nav_msgs::OccupancyGrid o_original_global_map_;
    NaviFra::Map o_static_global_map_;
    NaviFra::Map o_dynamic_local_map_;
    NaviFra::GlobalMap o_new_static_global_map_;
    int n_global_map_size_;
    LocalMap o_inflation_localmap_;
    nav_msgs::OccupancyGrid o_loc_added_glob_map_;

    vector<NaviFra::SimplePos> o_data_;
    bool b_init_localmap_params_recvd_ = false;
    bool b_init_localmap_gend_ = false;
    nav_msgs::OccupancyGrid ros_map_msg_;
    bool b_map_db_received_ = false;
    bool b_globalmap_set_ = false;
    bool b_localmap_set_ = false;
};
}  // namespace NaviFra

#endif
