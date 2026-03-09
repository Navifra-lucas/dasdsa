#ifndef CAMERA_OBSTACLE_CHECK_HPP
#define CAMERA_OBSTACLE_CHECK_HPP

#include "core_msgs/CameraCmd.h"
#include "msg_information/parameter_information/param_msg.hpp"
#include "nav_msgs/Odometry.h"
#include "pos/pos.hpp"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int64.h"
#include "std_msgs/String.h"

#include <chrono>
#include <memory>
#include <mutex>
#include <shared_mutex>

namespace NaviFra {

class CameraObstacle {
public:
    enum class MISSION_STATUS
    {
        IDLE = 0,
        RUNNING,
        SUSPENDING,
        END
    };

    CameraObstacle();
    virtual ~CameraObstacle(){};
    void SetNaviParam(const Parameters_t& st_param);
    bool CameraObstacleCheck(
        bool b_pre_detect, bool b_spin_turn, bool b_docking_check, bool b_back_flag, bool b_obs_camera_check, bool b_is_out_docking,
        int n_status, int n_local_path_idx, const Pos& o_robot_pos, const vector<Pos>& vec_pos_local_path,
        const Pos::DriveInfo_t& o_drive_info, const Pos::DriveInfo_t& o_goal_drive_info, NaviFra::Polygon& o_polygon,
        const std::vector<NaviFra::SimplePos>& vec_sensors_vision_robot);

    // 로봇 속도 받아오기
    ros::NodeHandle node_handle_;
    ros::Subscriber robot_speed_sub_;
    void RobotSpeedCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void CameraDistRedCallback(const ros::TimerEvent& event);
    void SetCameraState(bool mode);
    Pos o_obs_pos_;
    Pos GetObsPos(){return o_obs_pos_;};

    int robot_pre_speed_check_ = 1;
    int n_local_path_idx_ = 0;

    float robot_speed_ = 0.0f;
    float robot_max_speed_ = 0.0f;
    float f_camera_suspendmission_dist_m_ = 0.0;
    float f_camera_collison_margin_front_ = 0.0;
    float f_camera_collison_margin_rear_ = 0.0;
    float f_camera_collison_margin_left_ = 0.0;
    float f_camera_collison_margin_right_ = 0.0;

    bool b_is_pre_docking_ = false;
    bool b_camera_detect_time_ = false;
    bool b_camera_detect_check_ = false;
    bool b_camera_off_check_ = false;
    bool b_camera_sto_check_ = false;
    bool b_pre_camera_sto_ = true;
    bool b_camera_clear_flag_ = false;
    bool b_ready_to_sto_ = false;
    bool b_docking_check_ = false;

    ros::Publisher set_area_ratio_pub_;
    ros::Publisher avoid_state_pub_;
    ros::Publisher current_margin_pub_;
    ros::Subscriber obstacle_sub_;
    ros::Timer camera_dist_reduct_timer_;
    ros::ServiceClient camera_cmd_req_;
    ros::Publisher camera_sto_pub_;
    ros::Publisher clear_obstacle_pub_;

    std::chrono::steady_clock::time_point tp_camera_clear_time_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_camera_detect_time_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_set_area_ratio_time_ = std::chrono::steady_clock::now();

private:
    std::mutex mtx_param_;
    std::mutex mtx_robot_speed_;
    std::mutex mtx_local_path_;
    std::mutex mtx_path_idx_;

    Parameters_t st_param_;
    std::vector<Pos> vec_pos_local_path_;  // 지역경로 변수
};

}  // namespace NaviFra
#endif