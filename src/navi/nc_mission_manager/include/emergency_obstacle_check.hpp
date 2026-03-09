// #ifndef LIDAR_OBSTACLE_CHECK_HPP
// #define LIDAR_OBSTACLE_CHECK_HPP

// #include "debug/debug_visualizer.hpp"
// #include "geometry_msgs/Point32.h"
// #include "geometry_msgs/PolygonStamped.h"
// #include "msg_information/parameter_information/param_msg.hpp"
// #include "msg_information/sensor_information/sensor_msg.hpp"
// #include "pos/pos.hpp"
// #include "ros/ros.h"
// #include <std_msgs/Bool.h>


// #include <vector>
// #include <chrono>
// #include <memory>
// #include <mutex>
// #include <shared_mutex>

// namespace NaviFra {

// enum CheckPolygon
// {
//     MOVE = 0,
//     TARGET,
//     START
// };

// class EMOObstacle {
// public:
//     EMOObstacle();

//     virtual ~EMOObstacle(){};
//     int CheckPointInEMOPolygon(
//         const std::vector<NaviFra::SimplePos>& vec_sensors_relative_robot, NaviFra::Polygon& o_polygon);
        
//     float PredictObstacleCheck(
//         const Parameters_t& st_param, NaviFra::Polygon& o_polygon, const std::vector<NaviFra::SimplePos>& vec_sensors_relative_robot,
//         bool b_arrive_check, bool b_docking_path_obs_check, const CommandVelocity_t& regenerated_st_vel, bool b_diagonal_path);
//     static void makeLinearInflatedPolygon_(
//         NaviFra::Polygon& o_polygon_emo,
//         float f_v_lin_mps,
//         float f_front_base, float f_rear_base, float f_side_base,
//         float f_gain, std::vector<NaviFra::Polygon> o_out_polygon);
//     int CheckPointInEMOPolygonLinear(
//         const std::vector<NaviFra::SimplePos>& vec_sensors_relative_robot,
//         NaviFra::Polygon& o_polygon,
//         float f_v_lin_mps,
//         float f_front_base, float f_rear_base, float f_side_base, float f_gain);
        


//     std::vector<NaviFra::Polygon> vec_target_obstacle_polygon_;
//     std::vector<NaviFra::Polygon> vec_polygon_docking_path_check_;
//     std::vector<NaviFra::Polygon> vec_polygon_start_check_;
//     std::chrono::steady_clock::time_point tp_start_obs_detect_time_ = std::chrono::steady_clock::now();
//     std::mutex mtx_polygon_;
//     std::mutex mtx_param_;
//     Pos GetObsPos(){return o_obs_pos_;};
//     ros::Publisher pub_estop_;
//     ros::Publisher pub_estop_state_;

// private:
//     Parameters_t st_param_;
//     int n_test_ = 0;
//     void SetCheckEMOPolygon(int n_polygon, const std::vector<NaviFra::Polygon>& o_polygon);
//     std::vector<NaviFra::Polygon> GetCheckEMOPolygon(int n_polygon);
//     ros::NodeHandle nh_;
//     Pos o_obs_pos_;

//     geometry_msgs::PolygonStamped DrawPolygon(const std::vector<NaviFra::Pos>& vec_pos)
//     {
//         geometry_msgs::PolygonStamped poly_msg;
//         geometry_msgs::Point32 p;

//         poly_msg.header.frame_id = "base_link";
//         int n_vec_size = vec_pos.size();
//         for (int i = 0; i < n_vec_size; i++) {
//             p.x = vec_pos[i].GetXm();
//             p.y = vec_pos[i].GetYm();
//             poly_msg.polygon.points.push_back(p);
//         }
//         return poly_msg;
//     }
// };

// }  // namespace NaviFra
// #endif