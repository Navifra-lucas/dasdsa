#include "camera_obstacle_check_ros.hpp"

#include "util/logger.hpp"

using namespace std;

namespace NaviFra {

CameraObstacle::CameraObstacle()
{
    robot_speed_sub_ = node_handle_.subscribe("odom", 10, &CameraObstacle::RobotSpeedCallback, this);  // 속도 m/s
    camera_dist_reduct_timer_ = node_handle_.createTimer(ros::Duration(0.1), &CameraObstacle::CameraDistRedCallback, this);

    avoid_state_pub_ = node_handle_.advertise<std_msgs::Int64>("/avoid_state", 5);
    current_margin_pub_ = node_handle_.advertise<std_msgs::String>("/current_margin", 5);
    camera_cmd_req_ = node_handle_.serviceClient<core_msgs::CameraCmd>("/nc_obstacle_detector/set_camera");
    camera_sto_pub_ = node_handle_.advertise<std_msgs::Int64>("/camera_sto", 5);
    clear_obstacle_pub_ = node_handle_.advertise<std_msgs::Bool>("/nc_obstacle_detector/clear_obstacle", 5);
    set_area_ratio_pub_ = node_handle_.advertise<std_msgs::Bool>("/nc_obstacle_detector/set_area_ratio", 5);

    LOG_INFO("Constructor", 1);
}

bool CameraObstacle::CameraObstacleCheck(
    bool b_pre_detect, bool b_spin_turn, bool b_docking_check, bool b_back_flag, bool b_obs_camera_check, bool b_is_out_docking,
    int n_status, int n_local_path_idx, const Pos& o_robot_pos, const vector<Pos>& vec_pos_local_path, const Pos::DriveInfo_t& o_drive_info,
    const Pos::DriveInfo_t& o_goal_drive_info, NaviFra::Polygon& o_polygon, const std::vector<NaviFra::SimplePos>& vec_sensors_vision_robot)
{
    b_docking_check_ = b_docking_check;
    {
        std::lock_guard<std::mutex> lock(mtx_path_idx_);
        n_local_path_idx_ = n_local_path_idx;
    }
    {
        std::lock_guard<std::mutex> lock(mtx_path_idx_);
        n_local_path_idx_ = n_local_path_idx;
        vec_pos_local_path_ = vec_pos_local_path;
    }
    Parameters_t st_param;
    {
        std::lock_guard<std::mutex> lock(mtx_param_);
        st_param = st_param_;
    }

    bool b_detect_check = false;

    bool b_diagonal = false;
    int n_vision_size = vec_sensors_vision_robot.size();

    if (n_local_path_idx < vec_pos_local_path.size()) {
        auto e_drive_type = vec_pos_local_path.at(n_local_path_idx).GetConstDriveInfo().e_drive_type;
        if ((e_drive_type == Pos::DRIVE_TYPE::FRONT_DIAGONAL) || (e_drive_type == Pos::DRIVE_TYPE::REAR_DIAGONAL)) {
            b_diagonal = true;
        }
    }

    bool b_is_normal = false;

    bool b_camera_sto = false;
    float f_obstacle_distnace = 0;
    const NaviFra::Pos zero_pos(0, 0, 0);

    // vision camera margin up collision detect
    NaviFra::Polygon o_polygon_vision = o_polygon;
    if (b_spin_turn == false && b_diagonal == false) {
        if (!b_docking_check || b_is_out_docking) {
            if (b_is_pre_docking_ == false) {
                Pos o_differ_pos_camera;
                b_is_normal = true;
                int n_robot_path_idx = CoreCalculator::FindMinDistanceIdxFromPosVector_(o_robot_pos, vec_pos_local_path);
                float f_camera_path_check_dist = st_param.f_camera_path_check_m;
                if (o_drive_info.obstacle_move_reduction) {
                    f_camera_path_check_dist = st_param.f_restart_obs_check_m;
                }
                if (f_camera_path_check_dist * 100 >= vec_pos_local_path.size() - n_robot_path_idx)
                    f_camera_path_check_dist = (vec_pos_local_path.size() - n_robot_path_idx) / 100;
                float f_check_step = f_camera_path_check_dist / st_param.f_camera_path_check_step;
                for (int j = 0; j <= f_check_step; j++) {
                    o_polygon_vision.Clear();
                    NaviFra::Polygon o_start_obs_check_polygon;
                    int camera_path_check_idx = n_robot_path_idx + st_param.f_camera_path_check_step * j * 100;
                    if (camera_path_check_idx >= vec_pos_local_path.size()) {
                        LOG_ERROR("index error %d %d", camera_path_check_idx, vec_pos_local_path.size());
                        camera_path_check_idx = vec_pos_local_path.size() - 1;
                    }

                    o_differ_pos_camera = vec_pos_local_path.at(camera_path_check_idx) - vec_pos_local_path.at(n_robot_path_idx);
                    o_differ_pos_camera = CoreCalculator::TransformRotationDeg_(o_differ_pos_camera, -o_robot_pos.GetDeg());
                    NaviFra::Polygon o_polygon_tmp = o_polygon;
                    o_polygon_tmp.UpdateVertexSizePlusFLRR(
                        0, 0, st_param_.f_camera_path_check_side_margin, st_param_.f_camera_path_check_side_margin);

                    std::vector<NaviFra::Pos> o_camera_path_check_vertexs(4);
                    float f_robot_rad = o_robot_pos.GetRad();
                    if (vec_pos_local_path.at(camera_path_check_idx).GetConstDriveInfo().e_drive_type == Pos::DRIVE_TYPE::REAR ||
                        vec_pos_local_path.at(camera_path_check_idx).GetConstDriveInfo().e_drive_type == Pos::DRIVE_TYPE::REAR_DIAGONAL)
                        f_robot_rad -= M_PI;
                    o_polygon_tmp.UpdateVertexRotate(vec_pos_local_path.at(camera_path_check_idx).GetRad() - f_robot_rad);

                    for (int i = 0; i < o_polygon_tmp.o_polygon_vertexs_.size(); i++) {
                        o_camera_path_check_vertexs.at(i) = NaviFra::Pos(
                            o_polygon_tmp.o_polygon_vertexs_.at(i).GetXm() + o_differ_pos_camera.GetXm(),
                            o_polygon_tmp.o_polygon_vertexs_.at(i).GetYm() + o_differ_pos_camera.GetYm(), 0);
                        o_polygon_vision.AddVertex(o_camera_path_check_vertexs.at(i));
                    }
                    for (int k = 0; k < n_vision_size; k++) {
                        // 체크할 카메라 포인트
                        NaviFra::Pos o_checking_pos(vec_sensors_vision_robot.at(k).GetXm(), vec_sensors_vision_robot.at(k).GetYm());
                        // 해당 지점이 로봇에 충돌한 경우
                        if (o_polygon_vision.GetPointInPolygon(o_checking_pos)) {
                            o_obs_pos_ = o_checking_pos;
                            if (!b_pre_detect) {
                                LOG_INFO(
                                    "[monitoring] CAMERA_COLLISION_DETECTION .at(PATH_CHECK) x : %.3f, y : %.3f", o_checking_pos.GetXm(),
                                    o_checking_pos.GetYm());
                            }
                            f_obstacle_distnace =
                                fabs(CoreCalculator::CalcPosDistance_(zero_pos, o_checking_pos)) - o_polygon.GetRobotHeigth();
                            if (f_obstacle_distnace < st_param_.f_camera_sto_off_m) {
                                b_camera_sto = true;
                            }
                            b_detect_check = true;
                            b_camera_detect_check_ = true;
                            break;
                        }
                    }
                    if (b_detect_check)
                        break;
                    else {
                        b_camera_detect_check_ = false;
                    }
                }
            }
            else {
                f_camera_collison_margin_right_ = 0;
                f_camera_collison_margin_left_ = 0;

                float f_plt_x = 0.0;
                float f_plt_y = 0.0;

                if (o_drive_info.f_plt_type_x != 0 && o_drive_info.f_plt_type_y != 0) {
                    f_plt_x = o_drive_info.f_plt_type_x / 2;
                    f_plt_y = o_drive_info.f_plt_type_y / 2;
                }

                std::vector<float> vec_vision_polygon_vertexes;

                vec_vision_polygon_vertexes.emplace_back(0.001);
                vec_vision_polygon_vertexes.emplace_back(-0.001);
                vec_vision_polygon_vertexes.emplace_back(-0.001);
                vec_vision_polygon_vertexes.emplace_back(0.001);

                o_polygon_vision.Clear();

                o_polygon_vision.AddVertexList(vec_vision_polygon_vertexes);
                o_polygon_vision.UpdateVertexSizePlusFLRR(f_plt_x, f_plt_x, f_plt_y, f_plt_y);

                if (st_param_.map_polygon_robot_collision_.find("outline") != st_param_.map_polygon_robot_collision_.end()) {
                    NaviFra::Polygon o_polygon_outline = st_param_.map_polygon_robot_collision_.at("outline");
                    o_polygon_vision.UpdateVertexSizeMax(o_polygon_outline);
                }
                if (false == b_back_flag) {
                    f_camera_collison_margin_rear_ = 0;
                    f_camera_collison_margin_front_ = st_param_.f_camera_collision_margin_reducted;
                }
                else {
                    f_camera_collison_margin_front_ = 0;
                    f_camera_collison_margin_rear_ = st_param_.f_camera_collision_margin_reducted;
                }
            }
        }
        else {
            f_camera_collison_margin_right_ = 0;
            f_camera_collison_margin_left_ = 0;

            float f_plt_x = 0.0;
            float f_plt_y = 0.0;

            if (o_drive_info.f_plt_type_x != 0 && o_drive_info.f_plt_type_y != 0) {
                f_plt_x = o_drive_info.f_plt_type_x / 2 - 0.1;
                f_plt_y = o_drive_info.f_plt_type_y / 2 - 0.1;
            }

            // if (f_plt_y < f_outline_width_m)
            //     f_plt_y = f_outline_width_m;
            // if (f_plt_x < f_outline_height_m)
            //     f_plt_x = f_outline_height_m;

            int n_robot_path_idx = CoreCalculator::FindMinDistanceIdxFromPosVector_(o_robot_pos, vec_pos_local_path);

            Pos o_differ_pos = vec_pos_local_path.at(n_robot_path_idx) - o_robot_pos;
            o_differ_pos =
                CoreCalculator::TransformRotationDeg_(o_differ_pos, -vec_pos_local_path.at(vec_pos_local_path.size() - 1).GetDeg());

            std::vector<float> vec_vision_polygon_vertexes;

            vec_vision_polygon_vertexes.emplace_back(0.001);
            vec_vision_polygon_vertexes.emplace_back(-0.001);
            vec_vision_polygon_vertexes.emplace_back(-0.001);
            vec_vision_polygon_vertexes.emplace_back(0.001);

            o_polygon_vision.Clear();
            o_polygon_vision.AddVertexList(vec_vision_polygon_vertexes);
            o_polygon_vision.UpdateVertexSizePlusFLRR(
                f_plt_x, f_plt_x, f_plt_y + st_param_.f_camera_path_check_side_margin, f_plt_y + st_param_.f_camera_path_check_side_margin);

            NaviFra::Polygon o_polygon_vision_tmp;
            std::vector<NaviFra::Pos> o_vision_check_polygon_vertexs(4);

            for (int i = 0; i < o_polygon_vision.o_polygon_vertexs_.size(); i++) {
                o_vision_check_polygon_vertexs.at(i) = NaviFra::Pos(
                    o_polygon_vision.o_polygon_vertexs_.at(i).GetXm() + o_differ_pos.GetXm(),
                    o_polygon_vision.o_polygon_vertexs_.at(i).GetYm() + o_differ_pos.GetYm(), 0);
                o_polygon_vision_tmp.AddVertex(o_vision_check_polygon_vertexs.at(i));
            }

            o_polygon_vision = o_polygon_vision_tmp;

            float gap_x = vec_pos_local_path.at(vec_pos_local_path.size() - 1).GetXm() - vec_pos_local_path.at(n_robot_path_idx).GetXm();
            float gap_y = vec_pos_local_path.at(vec_pos_local_path.size() - 1).GetYm() - vec_pos_local_path.at(n_robot_path_idx).GetYm();

            if (false == b_back_flag) {
                f_camera_collison_margin_rear_ = 0;
                f_camera_collison_margin_front_ = sqrt(gap_x * gap_x + gap_y * gap_y);
            }
            else {
                f_camera_collison_margin_front_ = 0;
                f_camera_collison_margin_rear_ = sqrt(gap_x * gap_x + gap_y * gap_y);
            }
        }
    }
    else {
        f_camera_collison_margin_right_ = 0;
        f_camera_collison_margin_left_ = 0;
        f_camera_collison_margin_front_ = st_param_.f_camera_collision_margin_rotate;
        f_camera_collison_margin_rear_ = st_param_.f_camera_collision_margin_rotate;
    }
    // LOG_INFO("f_camera %.3f %.3f %.3f %.3f",f_camera_collison_margin_front_, f_camera_collison_margin_rear_,
    // f_camera_collison_margin_left_,
    //     f_camera_collison_margin_right_);

    if (!b_is_normal) {
        o_polygon_vision.UpdateVertexSizePlusFLRR(
            f_camera_collison_margin_front_, f_camera_collison_margin_rear_, f_camera_collison_margin_left_,
            f_camera_collison_margin_right_);
    }
    if (b_spin_turn == false && b_diagonal == false && b_docking_check) {
        float diff_rad =
            CoreCalculator::CalcAngleDomainRad_(o_robot_pos.GetRad() - vec_pos_local_path.at(vec_pos_local_path.size() - 1).GetRad());

        if (diff_rad > M_PI_2 && diff_rad <= -M_PI_2) {
            // LOG_INFO("backward angle");
            diff_rad = CoreCalculator::CalcAngleDomainRad_(M_PI + diff_rad);
            o_polygon_vision.UpdateVertexRotate(-diff_rad);
            o_polygon_vision.UpdateVertexInversionH();
        }
        else
            o_polygon_vision.UpdateVertexRotate(-diff_rad);
    }

    if (!b_is_normal) {
        for (int i = 0; i < n_vision_size; i++) {
            // 체크할 카메라 포인트
            NaviFra::Pos o_checking_pos(vec_sensors_vision_robot.at(i).GetXm(), vec_sensors_vision_robot.at(i).GetYm());
            // 해당 지점이 로봇에 충돌한 경우
            if (o_polygon_vision.GetPointInPolygon(o_checking_pos)) {
                if (!b_pre_detect)
                    LOG_INFO(
                        "[monitoring] CAMERA_COLLISION_DETECTION .at(INNORMAL) x : %.3f, y : %.3f", vec_sensors_vision_robot.at(i).GetXm(),
                        vec_sensors_vision_robot.at(i).GetYm());
                f_obstacle_distnace = fabs(CoreCalculator::CalcPosDistance_(zero_pos, o_checking_pos)) - o_polygon.GetRobotHeigth();
                // if (f_obstacle_distnace < st_param_.f_camera_sto_off_m && st_param_.f_camera_sto_off_m != 0){
                //     b_camera_sto = true;
                // }
                b_detect_check = true;
                b_camera_detect_check_ = true;
                break;
            }
            else {
                b_camera_detect_check_ = false;
            }
        }

        if (b_docking_check && !b_is_out_docking) {
            if (o_drive_info.f_camera_docking_roi_y_m > 0 && o_drive_info.f_camera_docking_roi_z_m > 0) {
                if (!b_camera_detect_time_) {
                    tp_camera_detect_time_ = std::chrono::steady_clock::now();
                    b_camera_detect_time_ = true;
                }

                if (b_camera_detect_check_) {
                    tp_camera_detect_time_ = std::chrono::steady_clock::now();
                }
                std::chrono::duration<float> sec_camera_check = std::chrono::steady_clock::now() - tp_camera_detect_time_;
                float time_check = sec_camera_check.count();
                if (sec_camera_check.count() > st_param_.f_docking_camera_check_sec && n_vision_size != 0) {
                    if (!b_camera_off_check_) {
                        core_msgs::CameraCmd srv5;
                        srv5.request.b_disable = true;
                        srv5.request.index = -1;
                        if (camera_cmd_req_.call(srv5)) {
                            LOG_INFO("DOCKING CHECK SUCCESS, CAMERA OFF : %.3f", time_check);
                            b_camera_off_check_ = true;
                        }
                        else {
                            LOG_ERROR("Failed to call service set_camera.");
                        }
                    }
                }
            }
            else {
                if (!b_camera_off_check_) {
                    core_msgs::CameraCmd srv5;
                    srv5.request.b_disable = true;
                    srv5.request.index = -1;

                    if (camera_cmd_req_.call(srv5)) {
                        LOG_INFO("NO USE CAMERA, CAMERA OFF");
                        b_camera_off_check_ = true;
                    }
                    else {
                        LOG_ERROR("Failed to call service set_camera.");
                    }
                }
            }
        }
    }
    else {
        b_camera_detect_time_ = false;
    }

    if (b_camera_detect_check_) {
        b_ready_to_sto_ = true;
    }

    if (b_pre_camera_sto_ != b_camera_sto) {
        LOG_INFO("CAMERA Obstacle distance : %.3f", f_obstacle_distnace);
        LOG_INFO("CAMERA STO distnace : %.3f", st_param_.f_camera_sto_off_m);
        LOG_INFO("CAMERA STO robot speed : %.3f", robot_speed_);

        if (!b_camera_sto) {
            b_ready_to_sto_ = false;
        }
    }
    b_pre_camera_sto_ = b_camera_sto;

    if (st_param.b_set_area_ratio) {
        static bool b_set_area_ratio = false;
        if (n_status == int(MISSION_STATUS::SUSPENDING) && (b_obs_camera_check || b_camera_detect_check_)) {
            std::chrono::duration<double> sec_set_area_ratio = std::chrono::steady_clock::now() - tp_set_area_ratio_time_;

            // set area ratio 시간이 지정된 시간보다 큰 경우
            if (!b_set_area_ratio && sec_set_area_ratio.count() > st_param.f_set_area_ratio_time_sec) {
                LOG_WARNING("CAMERA SET AREA RATIO : %.3f", sec_set_area_ratio.count());
                std_msgs::Bool bool_msg;
                bool_msg.data = true;
                set_area_ratio_pub_.publish(bool_msg);
                b_set_area_ratio = true;
            }
        }
        else {
            // 파리미터 조정 시간 기준 및 reser flag 초기화
            tp_set_area_ratio_time_ = std::chrono::steady_clock::now();
        }

        // IDLE, RESUME, END 일 시 즉시 디폴트값 설정,
        if (b_set_area_ratio && n_status != int(MISSION_STATUS::SUSPENDING)) {
            std_msgs::Bool bool_msg;
            bool_msg.data = false;
            set_area_ratio_pub_.publish(bool_msg);
            b_set_area_ratio = false;
        }
    }

    if (st_param.b_camera_obs_clear && n_status == int(MISSION_STATUS::SUSPENDING) && (b_obs_camera_check || b_camera_detect_check_)) {
        std::chrono::duration<double> sec_camera_clear = std::chrono::steady_clock::now() - tp_camera_clear_time_;

        // 클리어 시간이 지정된 시간보다 큰 경우
        if (sec_camera_clear.count() > st_param.f_camera_obs_clear_time_sec) {
            LOG_WARNING("CAMERA CLEAR : %.3f", sec_camera_clear.count());

            // 클리어 후 상태 유지 플래그 설정
            b_camera_clear_flag_ = true;
            // pub날리면 clear하는 형식

            // 클리어 시간 기준 초기화
            tp_camera_clear_time_ = std::chrono::steady_clock::now();
        }
    }
    else {
        // 카메라 감지 상태가 해제되었을 때는 클리어 시간 업데이트 하지 않음
        if (!b_camera_clear_flag_) {
            tp_camera_clear_time_ = std::chrono::steady_clock::now();
        }
    }

    // 카메라 클리어 후에도 로봇 상태와 속도를 유지하도록 처리
    if (b_camera_clear_flag_) {
        // 클리어 후 상태를 2초 동안 유지
        std::chrono::duration<double> sec_since_clear = std::chrono::steady_clock::now() - tp_camera_clear_time_;
        if (sec_since_clear.count() <= 2.0) {
            LOG_WARNING("[Resume Mission} by Camera Clear: %d", b_camera_clear_flag_);
            return 0;  // 2초 동안 미션 재개하지 않음
        }

        // 2초 후 상태 플래그 초기화
        if (sec_since_clear.count() > 2.0) {
            b_camera_clear_flag_ = false;
        }
    }

    return b_detect_check;
}

void CameraObstacle::SetNaviParam(const Parameters_t& st_param)
{
    LOG_INFO("SetNaviParam");
    {
        std::lock_guard<std::mutex> lock(mtx_param_);
        st_param_ = st_param;
    }
}

void CameraObstacle::RobotSpeedCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    int robot_speed_check;
    std::lock_guard<std::mutex> lock(mtx_robot_speed_);
    {
        robot_speed_ = sqrt(pow(msg->twist.twist.linear.x, 2) + pow(msg->twist.twist.linear.y, 2));
    }
    if (robot_speed_ <= 0.6 && !b_docking_check_) {
        robot_speed_check = 1;
    }
    else if (robot_speed_ <= 1.0 && robot_speed_ > 0.6 && !b_docking_check_) {
        robot_speed_check = 2;
    }
    else if (robot_speed_ <= 1.4 && robot_speed_ > 1.0 && !b_docking_check_) {
        robot_speed_check = 3;
    }
    else if (robot_speed_ <= 2.0 && robot_speed_ > 1.4 && !b_docking_check_) {
        robot_speed_check = 4;
    }
    else if (robot_speed_ = 0 && !b_docking_check_) {
        robot_speed_check = 5;
    }

    float f_robot_speed_sto_m = 0;
    if (!b_ready_to_sto_) {
        if (robot_speed_check == 1) {
            f_robot_speed_sto_m = 0.6;
            st_param_.f_camera_sto_off_m = f_robot_speed_sto_m;
        }
        else if (robot_speed_check == 2) {
            f_robot_speed_sto_m = 1.0;
            st_param_.f_camera_sto_off_m = f_robot_speed_sto_m;
        }
        else if (robot_speed_check == 3) {
            f_robot_speed_sto_m = 1.4;
            st_param_.f_camera_sto_off_m = f_robot_speed_sto_m;
        }
        else if (robot_speed_check == 4) {
            f_robot_speed_sto_m = 1.8;
            st_param_.f_camera_sto_off_m = f_robot_speed_sto_m;
        }
        else {
            f_robot_speed_sto_m = 0;
            st_param_.f_camera_sto_off_m = f_robot_speed_sto_m;
        }
    }
}

void CameraObstacle::CameraDistRedCallback(const ros::TimerEvent& event)
{
    int n_local_path_idx;
    std::vector<Pos> vec_pos_local_path;
    {
        std::lock_guard<std::mutex> lock(mtx_path_idx_);
        n_local_path_idx = n_local_path_idx_;
        vec_pos_local_path = vec_pos_local_path_;
    }

    Pos::DriveInfo_t& o_drive_info = vec_pos_local_path[n_local_path_idx].GetDriveInfo();

    if (n_local_path_idx < vec_pos_local_path.size()) {
        if (vec_pos_local_path.size() - n_local_path_idx <= st_param_.f_dist_to_reduct_camera_margin * 100 &&
            o_drive_info.camera_dist_reduction == true)
            b_is_pre_docking_ = true;
        else
            b_is_pre_docking_ = false;
    }
}

void CameraObstacle::SetCameraState(bool mode)
{
    b_camera_off_check_ = mode;
    if (!mode) {
        core_msgs::CameraCmd srv5;
        srv5.request.b_disable = false;
        srv5.request.index = -1;
        if (camera_cmd_req_.call(srv5)) {
            LOG_INFO("CAMERA ON");
        }
        else {
            LOG_ERROR("Failed to call service set_camera ON.");
        }
    }
    NLOG(info) << "b_camera_off_check reset !!!!!!!! " << mode;
}

}  // namespace NaviFra