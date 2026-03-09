// #include "emergency_obstacle_check.hpp"

// #include "core/util/logger.hpp"

// using namespace std;

// namespace NaviFra {

//     EMOObstacle::EMOObstacle()
// {
//     pub_estop_state_ = nh_.advertise<std_msgs::Bool>("estop_state", 5);
//     pub_estop_ = nh_.advertise<std_msgs::Bool>("emergency", 5);

//     LOG_INFO("Constructor", 1);
//     vec_target_obstacle_polygon_.clear();
//     vec_polygon_docking_path_check_.clear();

// }
    

// int EMOObstacle::CheckPointInEMOPolygon(
//     const std::vector<NaviFra::SimplePos>& vec_sensors_relative_robot, NaviFra::Polygon& o_polygon)
// {
//     int n_detect_check = 0;
//     int n_sensor_size = vec_sensors_relative_robot.size();
//     static bool b_pre_detect = false;           
//     static bool s_prev_emergency = false;       // emergency 퍼블리시 상태 추적
//     bool b_emergency_now = false;               // 이번 호출에서 감지 여부

//     std::vector<NaviFra::Polygon> vec_polygon_docking_path_check = GetCheckEMOPolygon(CheckPolygon::MOVE);

//     for (int i = 0; i < n_sensor_size; i++) {
//         NaviFra::Pos o_checking_pos(vec_sensors_relative_robot.at(i).GetXm(), vec_sensors_relative_robot.at(i).GetYm());

//         // 로봇 본체 폴리곤 충돌
//         if (o_polygon.GetPointInPolygon(o_checking_pos)) {
//             o_obs_pos_ = o_checking_pos;
//             n_detect_check = 1;
//             if (!b_pre_detect)
//                 LOG_INFO("[monitoring] OBSTACLE_COLLISION_DETECTION : %.3f", vec_sensors_relative_robot.at(i).GetZm());
//             if (vec_sensors_relative_robot.at(i).GetZm() == 1)
//                 n_detect_check = 2;
//             else if (vec_sensors_relative_robot.at(i).GetZm() == 2)
//                 n_detect_check = 3;

//             b_emergency_now = true;    // 충돌 감지
//             break;
//         }
//     }

//     // 로그 에지 처리
//     if (n_detect_check != 0)
//         b_pre_detect = true;
//     else
//         b_pre_detect = false;

//     // emergency 퍼블리시: 에지(상승/하강)에서만 발행
//     if (b_emergency_now != s_prev_emergency) {
//         std_msgs::Bool msg;
//         msg.data = b_emergency_now;               // true: 감지됨, false: 해제됨
//         pub_estop_.publish(msg);                  // "emergency"
//         pub_estop_state_.publish(msg);            // "estop_state"도 함께 갱신(원하면 유지)
//         s_prev_emergency = b_emergency_now;
//     }

//     return n_detect_check;
// }

// // 속도 기반으로 폴리곤을 선형 증가시킴
// //  - 전진: front = base + gain * |v|
// //  - 후진: rear  = base + gain * |v|
// //  - 좌우  : side = base + gain * |v|
// static void EMOObstacle::makeLinearInflatedPolygon_(
//     NaviFra::Polygon& o_polygon_emo,
//     float f_v_lin_mps,
//     float f_front_base, float f_rear_base, float f_side_base,
//     float f_gain, NaviFra::Polygon& o_out_polygon)
// {
//     float f_v_abs = std::fabs(f_v_lin_mps);

//     float f_front = f_front_base;
//     float f_rear  = f_rear_base;
//     float f_side  = f_side_base + f_gain * f_v_abs;

//     if (f_v_lin_mps >= 0.0f) {
//         f_front = f_front_base + f_gain * f_v_abs;   // 전진 시 전방 확대
//     } else {
//         f_rear  = f_rear_base  + f_gain  * f_v_abs;   // 후진 시 후방 확대
//     }

//     o_out_polygon = o_polygon_emo;
//     o_out_polygon.UpdateVertexSizePlusFLRR(f_front, f_rear, f_side_base, f_side_base);
// }

// //속도로 선형 인플레이트 후 기존 CheckPointInEMOPolygon() 호출
// int EMOObstacle::CheckPointInEMOPolygonLinear(
//     const std::vector<NaviFra::SimplePos>& vec_sensors_relative_robot,
//     NaviFra::Polygon& o_polygon,
//     float f_v_lin_mps,
//     float f_front_base, float f_rear_base, float f_side_base, float f_gain)
// {
//     NaviFra::Polygon o_dyn_polygon;
//     makeLinearInflatedPolygon_(o_polygon,
//                                f_v_lin_mps,
//                                f_front_base, f_rear_base, f_side_base,
//                                f_gain,
//                                o_dyn_polygon);

//     // 기존 함수는 그대로 사용 (내부 퍼블리시/에지 트리거 유지)
//     return CheckPointInEMOPolygon(vec_sensors_relative_robot, o_dyn_polygon);
// }


// void EMOObstacle::SetCheckEMOPolygon(int n_polygon, const std::vector<NaviFra::Polygon>& o_polygon)
// {
//     std::lock_guard<std::mutex> lock(mtx_polygon_);
//     if (n_polygon == CheckPolygon::MOVE)
//         vec_polygon_docking_path_check_ = o_polygon;
// }

// std::vector<NaviFra::Polygon> EMOObstacle::GetCheckEMOPolygon(int n_polygon)
// {
//     std::lock_guard<std::mutex> lock(mtx_polygon_);
//     std::vector<NaviFra::Polygon> vec_polygon;
//     if (n_polygon == CheckPolygon::MOVE)
//         vec_polygon = vec_polygon_docking_path_check_;
//     return vec_polygon;
// }

// }  // namespace NaviFra