#ifndef LIDAR_OBSTACLE_CHECK_HPP
#define LIDAR_OBSTACLE_CHECK_HPP

#include "debug/debug_visualizer.hpp"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PolygonStamped.h"
#include "msg_information/parameter_information/param_msg.hpp"
#include "msg_information/sensor_information/sensor_msg.hpp"
#include "pos/pos.hpp"
#include "ros/ros.h"

#include <chrono>
#include <memory>
#include <mutex>
#include <shared_mutex>

namespace NaviFra {

enum CheckPolygon
{
    MOVE = 0,
    TARGET,
    START
};

class LidarObstacle {
public:
    LidarObstacle()
    {
        vec_target_obstacle_polygon_.clear();
        vec_polygon_docking_path_check_.clear();
        vec_polygon_start_check_.clear();
        o_polygon_pre_.AddVertex(Pos(0.001, 0.001, 0));
        o_polygon_pre_.AddVertex(Pos(0.001, -0.001, 0));
        o_polygon_pre_.AddVertex(Pos(-0.001, -0.001, 0));
        o_polygon_pre_.AddVertex(Pos(-0.001, 0.001, 0));
    };
    virtual ~LidarObstacle(){};
    void TargetObsPolyMaker(
        const Pos& o_start_pos, const Pos& o_end_pos, const Pos& o_robot_pos, const vector<Pos>& vec_pos_local_path,
        const Pos::DriveInfo_t& o_drive_info, const Pos::DriveInfo_t& o_goal_drive_info);
    void MoveObsPolyMaker(
        const Pos& o_start_pos, const Pos& o_end_pos, const Pos& o_robot_pos, const Pos::DriveInfo_t& o_goal_drive_info,
        const Parameters_t& st_param, NaviFra::Polygon& o_polygon, const vector<Pos>& vec_pos_local_path);
    void StartObsPolyMaker(
        const Pos& o_robot_pos, const vector<Pos>& vec_pos_local_path, const Parameters_t& st_param, NaviFra::Polygon& o_polygon);
    int CheckPointInPolygon(
        const std::vector<NaviFra::SimplePos>& vec_sensors_relative_robot, NaviFra::Polygon& o_polygon, bool b_arrive_check,
        bool b_docking_path_obs_check, bool b_start_obs_check);
    void UpdatePolyBySpeed(NaviFra::Polygon& o_polygon, Parameters_t& st_param, float f_linear_speed_x, float f_angular_speed_degs, int n_detect_check);

    float SideObstacleCheck(
        float f_path_vel, const Parameters_t& st_param, NaviFra::Polygon& o_polygon,
        const std::vector<NaviFra::SimplePos>& vec_sensors_relative_robot);
    float PredictObstacleCheck(
        const Parameters_t& st_param, NaviFra::Polygon& o_polygon, const std::vector<NaviFra::SimplePos>& vec_sensors_relative_robot,
        bool b_arrive_check, bool b_docking_path_obs_check, const CommandVelocity_t& regenerated_st_vel, bool b_diagonal_path);

    std::vector<NaviFra::Polygon> vec_target_obstacle_polygon_;
    std::vector<NaviFra::Polygon> vec_polygon_docking_path_check_;
    std::vector<NaviFra::Polygon> vec_polygon_start_check_;
    std::chrono::steady_clock::time_point tp_start_obs_detect_time_ = std::chrono::steady_clock::now();
    std::mutex mtx_polygon_;
    std::mutex mtx_param_;
    Pos GetObsPos(){return o_obs_pos_;};

private:
    Parameters_t st_param_;
    int n_test_ = 0;
    void SetCheckPolygon(int n_polygon, const vector<NaviFra::Polygon>& o_polygon);
    vector<NaviFra::Polygon> GetCheckPolygon(int n_polygon);
    ros::NodeHandle node_handle_;
    Pos o_obs_pos_;
    NaviFra::Polygon o_polygon_pre_;

    geometry_msgs::PolygonStamped DrawPolygon(const vector<NaviFra::Pos>& vec_pos)
    {
        geometry_msgs::PolygonStamped poly_msg;
        geometry_msgs::Point32 p;

        poly_msg.header.frame_id = "base_link";
        int n_vec_size = vec_pos.size();
        for (int i = 0; i < n_vec_size; i++) {
            p.x = vec_pos[i].GetXm();
            p.y = vec_pos[i].GetYm();
            poly_msg.polygon.points.push_back(p);
        }
        return poly_msg;
    }
};

}  // namespace NaviFra
#endif