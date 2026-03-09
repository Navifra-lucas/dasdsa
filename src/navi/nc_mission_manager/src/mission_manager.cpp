#include "mission_manager.hpp"

namespace NaviFra {
MissionManager::MissionManager()
    : b_mission_complete_flag_(false)
    , b_termiation_completed_flag_(false)
{
    LOG_INFO("Constructor");

    globalmap_pub_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("globalmap", 1, true);
    static_localmap_pub_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("static_localmap", 1, true);
    dynamic_localmap_pub_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("dynamic_localmap", 1, true);
    localmap_pub_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("localmap", 1, true);

    robot_speed_sub_ =
        node_handle_.subscribe("odom", 10, &MissionManager::RobotSpeedCallback, this, ros::TransportHints().tcpNoDelay(true));
    map_db_sub_ = node_handle_.subscribe("/map/db", 1, &MissionManager::RecvMapDB, this);

    th1_ = boost::thread(boost::bind(&MissionManager::CarryoutMission, this));
    th_costmap_builder_ = boost::thread(boost::bind(&MissionManager::BuildLocalMap, this));
    NaviFra::ParamRepository::GetInstance()->RegisterCbFunc(
        "MissionManager_Param", std::bind(&MissionManager::SetParameter, this, std::placeholders::_1));
    // 미션이 끝날때 마다 호출되는 콜백함수를 등록한다.
    o_auto_move_mission_.RegistCbFunc(
        static_cast<int>(AutoMoveMission::NOTIFY_MSG::MISSION),
        std::bind(&MissionManager::RecvMissionCompleteCbMsg, this, std::placeholders::_1));
}

MissionManager::~MissionManager()
{
    if (th1_.joinable()) {
        th1_.join();
    }
    if (th_costmap_builder_.joinable()) {
        th_costmap_builder_.join();
    }
    LOG_INFO("Destructor");
}

void MissionManager::SetLoadState(const bool& b_msg)
{
    // NLOG(info)<<"Set Load : "<<b_msg;
    o_auto_move_mission_.SetLoadState(b_msg);
}

void MissionManager::SetForkPosition(int n_pos)
{
    o_auto_move_mission_.SetForkPosition(n_pos);
}

void MissionManager::SetUseLccs(const bool& b_msg)
{
    NLOG(info)<<"Set LCCS : "<<b_msg;
    o_auto_move_mission_.SetUseLccs(b_msg);
}

void MissionManager::SetGoalArrivedFlag(const bool& b_msg)
{
    NLOG(info)<<"Set GoalArrived : "<<b_msg;
    o_auto_move_mission_.SetGoalArrivedFlag(b_msg);
}

void MissionManager::RobotSpeedCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    Pos o_robot_speed;
    o_robot_speed.SetXm(msg->twist.twist.linear.x);
    o_robot_speed.SetYm(msg->twist.twist.linear.y);
    o_robot_speed.SetRad(msg->twist.twist.angular.z);
    o_auto_move_mission_.SetSpeed(o_robot_speed);
}

void MissionManager::BuildLocalMap()
{
    float f_map_resolution_mpx = 0.0;
    NaviFra::Map o_local_map;

    float f_localmap_x_m = 0.0;
    float f_localmap_y_m = 0.0;
    int n_localmap_size = 0;
    int n_ros_map_width = 0;
    int n_ros_map_height = 0;
    Pos o_robot_pos;

    vector<int8_t> vec_local_map_data;

    while (ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(st_local_mission_.update_frequency_ms));

        if (false == b_map_db_received_) {
            continue;
        }
        if (false == b_init_localmap_params_recvd_) {
            continue;
        }
        if (n_now_mission_num_ == 0) {
            continue;
        }
        if (false == b_globalmap_set_) {
            // Set global map
            b_globalmap_set_ = true;
            f_map_resolution_mpx = st_local_mission_.map_res_m;
            o_globalmap_.SetMapInfo(f_map_resolution_mpx, st_local_mission_.padding_size_m);
            o_globalmap_.SetGlobalMap(ros_grid_map_);
        }

        if (false == b_localmap_set_) {
            b_localmap_set_ = true;
            f_localmap_x_m = st_local_mission_.map_size_x_m;
            f_localmap_y_m = st_local_mission_.map_size_y_m;
            o_local_map.SetMapInfo(f_localmap_x_m, f_localmap_y_m, f_map_resolution_mpx);
            n_localmap_size = o_local_map.GetMap().size();
            n_ros_map_width = o_local_map.GetXpx();
            n_ros_map_height = o_local_map.GetYpx();
        }
        {
            o_robot_pos = o_robot_pos_;
        }

        o_globalmap_.GetLocalMap(o_local_map, o_robot_pos);

        nav_msgs::OccupancyGrid ros_grid_map;
        ros_grid_map.header.frame_id = "map";
        ros_grid_map.header.stamp = ros::Time::now();
        ros_grid_map.info.resolution = f_map_resolution_mpx;
        ros_grid_map.info.origin.position.x = o_local_map.GetXOriginM();
        ros_grid_map.info.origin.position.y = o_local_map.GetYOriginM();
        ros_grid_map.info.width = n_ros_map_width;
        ros_grid_map.info.height = n_ros_map_height;
        ros_grid_map.data = o_local_map.GetMap();

        if (false == st_param_.b_release_mode) {
            static_localmap_pub_.publish(ros_grid_map);
        }

        std::vector<NaviFra::SimplePos> o_data;
        {
            o_data = o_data_;
        }

        o_globalmap_.UpdateGridCellUisngLiDAR(o_local_map, o_data, o_robot_pos);
        ros_grid_map.data = o_local_map.GetMap();
        if (false == st_param_.b_release_mode) {
            dynamic_localmap_pub_.publish(ros_grid_map);
        }
        o_globalmap_.InflateMap(n_ros_map_width, n_ros_map_height, o_local_map, st_local_mission_.padding_size_m);
        ros_grid_map.data = o_local_map.GetMap();

        SetLocalMapPtr(std::make_shared<const std::vector<int8_t>>(ros_grid_map.data));

        if (false == st_param_.b_release_mode) {
            for (int i = 0; i < ros_grid_map.data.size(); i++) {
                if (ros_grid_map.data.at(i) == 100)
                    continue;
                ros_grid_map.data.at(i) = -ros_grid_map.data.at(i);
            }
            localmap_pub_.publish(ros_grid_map);
        }
    }
}

bool MissionManager::RegistCbFunc(const std::string& s_cb_func_name, const std::function<void(const boost::any&)>& pt_func)
{
    bool b_ret_val = o_auto_move_mission_.RegistCbFunc(s_cb_func_name, pt_func);
    if (true == b_ret_val) {
        LOG_INFO(" %s is registered as a callback function ", s_cb_func_name.c_str());
    }
    else {
    }
    return b_ret_val;
}

bool MissionManager::RegistCbFunc(int n_cb_name, const std::function<void(const boost::any&)>& pt_func)
{
    bool b_ret = o_auto_move_mission_.RegistCbFunc(n_cb_name, pt_func);
    return b_ret;
}

void MissionManager::SetMsg(int msg_id, const boost::any& any_type_var)
{
    switch (msg_id) {
        case MISSION_MSG::MSG_NAVIGATION_PARAM:
            SetParameter(any_type_var);
            break;
        case MISSION_MSG::MSG_TWIST:
            o_auto_move_mission_.SetTwist(boost::any_cast<float>(any_type_var));
            break;
        case MISSION_MSG::MSG_SPEED_PERCENT:
            o_auto_move_mission_.SetSpeedPercent(boost::any_cast<float>(any_type_var));
            break;
        case MISSION_MSG::MSG_TURN_PERCENT:
            o_auto_move_mission_.SetTurnPercent(boost::any_cast<float>(any_type_var));
            break;
        case MISSION_MSG::MSG_ARRIVE_BOUNDARY:
            o_auto_move_mission_.SetArriveBoundary(boost::any_cast<float>(any_type_var));
            break;
    }
}

void MissionManager::SetParameter(const boost::any& any_type_var)
{
    LOG_INFO("SetNaviParam");
    st_param_ = boost::any_cast<Parameters_t>(any_type_var);

    /* local mission parameter updated*/
    st_local_mission_.update_frequency_ms = st_param_.st_local_mission_param.updated_frequency_ms;
    st_local_mission_.map_size_x_m = st_param_.st_local_mission_param.map_size_x_m;
    st_local_mission_.map_size_y_m = st_param_.st_local_mission_param.map_size_y_m;
    st_local_mission_.map_res_m = st_param_.st_local_mission_param.map_res_m;
    st_local_mission_.padding_size_m = st_param_.st_local_mission_param.padding_size_m;

    if (st_param_.map_polygon_robot_collision_.find("collision") != st_param_.map_polygon_robot_collision_.end()) {
        float f_robot_half_width = st_param_.map_polygon_robot_collision_.at("collision").GetRobotWidth() / 2;
        float f_robot_half_height = st_param_.map_polygon_robot_collision_.at("collision").GetRobotHeigth() / 2;
        st_local_mission_.padding_size_m = st_param_.st_local_mission_param.padding_size_m + hypot(f_robot_half_width, f_robot_half_height);
    }
    b_init_localmap_params_recvd_ = true;

    LOG_INFO("SetNaviParam Done");
}

void MissionManager::RecvMapDB(const core_msgs::MapDB::ConstPtr& msg)
{
    if (msg->map_db.poses.size() < 1) {
        LOG_ERROR("RecvMapDB ERROR db size : %d", msg->map_db.poses.size());
        return;
    }
    int n_map_db_size = msg->map_db.poses.size();

    if (false == b_init_localmap_params_recvd_) {
        LOG_ERROR("Local Map Params Not Read Yet !");
        return;
        // [Kevin] Need to deal with this case somehow..
    }

    // unit : m
    NaviFra::SimplePos o_map_element;
    std::vector<NaviFra::SimplePos> vec_map_db;
    for (int i = 0; i < n_map_db_size; i++) {
        o_map_element.SetXm(msg->map_db.poses.at(i).position.x);
        o_map_element.SetYm(msg->map_db.poses.at(i).position.y);
        vec_map_db.emplace_back(o_map_element);
    }

    // Get this info from map.json
    // unit : m
    float f_width_m = msg->map_width;
    float f_height_m = msg->map_height;

    float f_map_resolution_mpx = st_local_mission_.map_res_m;

    float f_origin_x_m = msg->map_origin_x / f_map_resolution_mpx;
    float f_origin_y_m = msg->map_origin_y / f_map_resolution_mpx;

    // unit : pix
    int n_size_x_px = int((f_width_m) / f_map_resolution_mpx);
    int n_size_y_px = int((f_height_m) / f_map_resolution_mpx);

    int n_map_size_px2 = n_size_x_px * n_size_y_px;
    vector<int8_t> vec_global_map_data;

    vec_global_map_data.resize(n_map_size_px2, 0);

    for (int i = 0; i < n_map_db_size; i++) {
        int n_idx_x = float((vec_map_db.at(i).GetXm()) + f_width_m / 2) / f_map_resolution_mpx;
        int n_idx_y = float((vec_map_db.at(i).GetYm()) + f_height_m / 2) / f_map_resolution_mpx;

        int n_idx = n_size_x_px * n_idx_y + n_idx_x;
        if (n_idx_x < 0 || n_idx_x >= n_size_x_px || n_idx_y < 0 || n_idx_y >= n_size_y_px) {
            continue;
        }
        vec_global_map_data.at(n_idx) = 100;
    }

    ros_grid_map_.header.frame_id = "map";
    ros_grid_map_.header.stamp = ros::Time::now();
    ros_grid_map_.info.resolution = f_map_resolution_mpx;
    ros_grid_map_.info.origin.position.x = -f_width_m / 2;
    ros_grid_map_.info.origin.position.y = -f_height_m / 2;
    ros_grid_map_.info.width = n_size_x_px;
    ros_grid_map_.info.height = n_size_y_px;
    ros_grid_map_.data = vec_global_map_data;

    if (false == st_param_.b_release_mode) {
        globalmap_pub_.publish(ros_grid_map_);
    }
    b_map_db_received_ = true;
    b_globalmap_set_ = false;
}

void MissionManager::SetSensorMsg(const SensorMsg_t& st_sensor_msg)
{
    o_auto_move_mission_.SetSensorMsg(st_sensor_msg);
    o_data_ = st_sensor_msg.vec_sensors_relative_robot;
}

void MissionManager::SetRobotPos(const Pos& o_robot_pos)
{
    o_robot_pos_ = o_robot_pos;
    o_auto_move_mission_.SetRobotPos(o_robot_pos_);
}

void MissionManager::ResumeMission()
{
    o_auto_move_mission_.ResumeUser();
}

void MissionManager::SuspendMission()
{
    LOG_INFO("MissionManager is paused.");
    o_auto_move_mission_.PauseUser();
}
void MissionManager::TerminateMission(bool b_clear_mission)
{
    // 미션 repo에 남겨진 잔여 미션들을 모두 정리한다.
    o_auto_move_mission_.TerminateMission();
    b_arrive_flag_ = false;
    b_start_flag_ = false;
    n_now_mission_num_ = 0;
    if (b_clear_mission) {
        LOG_INFO("Before clear the mission stack, remaining mission number::%d ", mission_repo_.GetRemainingMissions());
        mission_repo_.ClearMissionStack();
        mission_repo_.SetTermiateState(true);
        LOG_INFO("After clear, Mission stack size::%d ", mission_repo_.GetRemainingMissions());
    }
    else {
        LOG_INFO("Do not Clear Mision size : %d ", mission_repo_.GetRemainingMissions());
    }

    // ResumeMission();
}

void MissionManager::ClearMission()
{
    // 1. 기존 수행 중인 미션 및 미션 stack을 모두 클리어 한다.
    LOG_WARNING("******************************Important message(not error)*************************");
    LOG_WARNING("*          start function will be watied until previous mission terminated!!!     *");
    LOG_WARNING("*   Always make sure that function recv_mission_termination_cb_msg is called!!!!! *");
    LOG_WARNING("******************************Important message end********************************");
    TerminateMission(true);  // 기존 미션이 남아 있을 수 있기 때무에 항상 호출하고 시작
    LOG_INFO("mtx_termination_wait_ %d", b_termiation_completed_flag_);
    LOG_INFO("%d", b_mission_complete_flag_);
    LOG_INFO("%d", b_termiation_completed_flag_);
    LOG_INFO("Previous mission is terminated cleary!!! New mission will be stated!!");
    boost::this_thread::sleep_for(boost::chrono::milliseconds(20));
}

std::vector<NaviFra::PathDescription> MissionManager::MakeDevidedPath(const vector<NaviNode>& o_msg)  // vector<node> [ node, node , ...]
{
    std::vector<NaviFra::PathDescription> vec_mission_stack;
    vector<NaviNode> vec_node_origin;

    std::vector<Pos> vec_path;
    for (int i = 1; i < o_msg.size(); i++) {
        NaviNode o_start_node = o_msg.at(i - 1);
        NaviNode o_end_node = o_msg.at(i);
        Pos::DriveInfo_t o_drive_info = o_start_node.GetDriveInfo();
        Pos::DriveInfo_t o_drive_info_end = o_end_node.GetDriveInfo();
        LOG_INFO("%d o_msg   start %d end %d", i, o_drive_info.e_drive_type, o_drive_info_end.e_drive_type);

        bool b_stop_node = false;

        if (o_msg.at(i).GetConstDriveInfo().f_curve_radius > 0) {
            LOG_INFO("%d f_curve_radiusd   , %f", i, o_msg.at(i).GetConstDriveInfo().f_curve_radius);
            LOG_INFO("f_curve_radiusd use");
            vec_node_origin.emplace_back(o_msg.at(i));  // curve 노드만 추가
        }

        // 각도 틀어짐 체크, 커브 안주고 각도 꺾여있으면 나눠서감
        if (i != o_msg.size() - 1 && o_msg.at(i).GetConstDriveInfo().f_curve_radius == 0) {
            LOG_INFO("angle check %d", i);
            NaviNode node1 = o_msg.at(i - 1);
            NaviNode node2 = o_msg.at(i);
            NaviNode node3 = o_msg.at(i + 1);

            float f_angle1 = atan2(node1.GetYm() - node2.GetYm(), node1.GetXm() - node2.GetXm());
            float f_angle2 = atan2(node2.GetYm() - node3.GetYm(), node2.GetXm() - node3.GetXm());
            float f_angle = CoreCalculator::CalcAngleDomainRad_(f_angle1 - f_angle2) * RADtoDEG;

            NLOG(info) << "f_angle1 : " << f_angle1 << "f_angle2 : " << f_angle2 << "f_angle : " << f_angle;

            if (fabs(f_angle) > 10) {
                NLOG(info) << "f_angle " << f_angle;
                b_stop_node = true;
            }
        }

        // avoid type
        bool b_avoid_type_is_difer = false;

        if (o_drive_info.n_avoid_type != o_drive_info_end.n_avoid_type) {
            b_avoid_type_is_difer = true;
        }

        NLOG(info) << i << " o_drive_info " << o_drive_info.e_drive_type << " / " << o_drive_info.f_linear
                   << ", avoid type: " << o_drive_info.n_avoid_type << " o_drive_info_end.n_avoid_type " << o_drive_info_end.n_avoid_type;

        std::vector<Pos> vec_temp_path = CoreCalculator::GetRayPosToTarget_(o_start_node, o_end_node, 0.01);
        LOG_INFO("vec_temp_path: %d", vec_temp_path.size());
        if (vec_temp_path.size() == 0)
            continue;
        if(i == o_msg.size() - 1) { // 마지막 패스에서
            vec_temp_path.back() = o_end_node;
        }
        bool b_diagonal =
            (o_drive_info.e_drive_type == Pos::DRIVE_TYPE::FRONT_DIAGONAL || o_drive_info.e_drive_type == Pos::DRIVE_TYPE::REAR_DIAGONAL)
            ? true
            : false;
        if (b_diagonal) {
            NLOG(info) << i << " b_diagonal " << o_end_node.GetDeg();
            if (vec_temp_path.size() > 0) {
                if (o_drive_info.e_drive_type == Pos::DRIVE_TYPE::REAR_DIAGONAL) {
                    vec_temp_path.at(0).SetDeg(o_end_node.GetDeg() + 180);
                    NLOG(info) << "REAR " << vec_temp_path.at(0).GetDeg();
                }
                else {
                    vec_temp_path.at(0).SetDeg(o_end_node.GetDeg());
                    NLOG(info) << "FRONT " << vec_temp_path.at(0).GetDeg();
                }
            }
            for (int idx = 0; idx < vec_temp_path.size(); idx++) {
                if (o_drive_info.e_drive_type == Pos::DRIVE_TYPE::REAR_DIAGONAL) {
                    vec_temp_path.at(idx).SetDeg(o_end_node.GetDeg() + 180);
                }
                else {
                    vec_temp_path.at(idx).SetDeg(o_end_node.GetDeg());
                }
            }
        }

        for (int j = 0; j < vec_temp_path.size(); j++) {
            vec_temp_path.at(j).SetNowNodeName(o_start_node.GetName());
            vec_temp_path.at(j).SetNowNodeID(o_start_node.GetID());
            vec_temp_path.at(j).SetNextNodeName(o_end_node.GetName());
            vec_temp_path.at(j).SetNextNodeID(o_end_node.GetID());
            vec_temp_path.at(j).SetDriveInfo(o_drive_info);
        }

        vec_temp_path.back().SetNowNodeName(o_end_node.GetName());
        vec_temp_path.back().SetNowNodeID(o_end_node.GetID());
        if (i < o_msg.size() - 1) {
            vec_temp_path.back().SetNextNodeName(o_msg.at(i + 1).GetName());
            vec_temp_path.back().SetNextNodeID(o_msg.at(i + 1).GetID());
        }
        else {
            vec_temp_path.back().SetNextNodeName(o_msg.back().GetName());
            vec_temp_path.back().SetNextNodeID(o_msg.back().GetID());
        }
        vec_path.insert(vec_path.end(), vec_temp_path.begin(), vec_temp_path.end());

        bool b_differ_direction = ((int(o_drive_info.e_drive_type) < 2000 && int(o_drive_info_end.e_drive_type) < 2000) ||
                                   (int(o_drive_info.e_drive_type) >= 2000 && int(o_drive_info_end.e_drive_type) >= 2000))
            ? false
            : true;  // 다르면 true, 같은 방향성이면 false

        if (o_drive_info.e_drive_type == o_drive_info_end.e_drive_type) {
            // 연속된 사행 path에 대한 예외 처리
            if ((o_drive_info.e_drive_type == Pos::DRIVE_TYPE::FRONT_DIAGONAL &&
                    o_drive_info_end.e_drive_type == Pos::DRIVE_TYPE::FRONT_DIAGONAL) ||
                (o_drive_info.e_drive_type == Pos::DRIVE_TYPE::REAR_DIAGONAL &&
                    o_drive_info_end.e_drive_type == Pos::DRIVE_TYPE::REAR_DIAGONAL)) {
                // 도착 각도가 다르고 다음이 곡선 노드가 아닌 경우 path를 나눠 align하도록 함
                b_differ_direction =
                    (fabs(o_start_node.GetDeg() - o_end_node.GetDeg()) > 10 && o_drive_info_end.f_curve_radius == 0) ? true : false;

                // 곡선 가상노드에 대해서 path divide된 경우 예외처리
                if (o_drive_info_end.f_curve_radius != 0) {
                    NLOG(info) << "b_stop_node : "<< b_stop_node;
                    b_stop_node = false;
                }

                // 다음 노드와 각도가 다르다면 align하고 갈수 있도록 예외처리 (사행 곡선을 위함)
                if (i != o_msg.size() - 1) {
                    NaviNode node2 = o_msg.at(i);
                    NaviNode node3 = o_msg.at(i + 1);

                    if (fabs(node2.GetDeg() - node3.GetDeg()) > 10) {
                        NLOG(info) << "f_angle " << fabs(node2.GetDeg() - node3.GetDeg());
                        b_stop_node = true;
                    }
                }
            }
        }
        else
        {
            if ((o_drive_info.e_drive_type == Pos::DRIVE_TYPE::FRONT &&
                    o_drive_info_end.e_drive_type == Pos::DRIVE_TYPE::FRONT_DIAGONAL) ||
                (o_drive_info.e_drive_type == Pos::DRIVE_TYPE::FRONT_DIAGONAL &&
                    o_drive_info_end.e_drive_type == Pos::DRIVE_TYPE::FRONT) ||
                (o_drive_info.e_drive_type == Pos::DRIVE_TYPE::REAR &&
                    o_drive_info_end.e_drive_type == Pos::DRIVE_TYPE::REAR_DIAGONAL) ||
                (o_drive_info.e_drive_type == Pos::DRIVE_TYPE::REAR_DIAGONAL &&
                    o_drive_info_end.e_drive_type == Pos::DRIVE_TYPE::REAR)){

                // 다음 노드와 각도가 다르다면 align하고 갈수 있도록 예외처리 (사행 곡선을 위함)
                if (i < o_msg.size() - 1) {
                    NaviNode node3 = o_msg.at(i + 1);

                    float f_angle1 = atan2(o_start_node.GetYm() - o_end_node.GetYm(), o_start_node.GetXm() - o_end_node.GetXm());
                    float f_angle2 = atan2(o_end_node.GetYm() - node3.GetYm(), o_end_node.GetXm() - node3.GetXm());

                    float f_cur_deg = fabs(o_start_node.GetDeg() - o_end_node.GetDeg());
                    if (o_drive_info.e_drive_type ==  Pos::DRIVE_TYPE::FRONT_DIAGONAL
                        || o_drive_info.e_drive_type == Pos::DRIVE_TYPE::REAR_DIAGONAL) f_cur_deg = o_end_node.GetDeg();
                    float f_next_deg = fabs(o_end_node.GetDeg() - node3.GetDeg());
                    if (o_drive_info_end.e_drive_type ==  Pos::DRIVE_TYPE::FRONT_DIAGONAL
                        || o_drive_info_end.e_drive_type == Pos::DRIVE_TYPE::REAR_DIAGONAL) f_next_deg = node3.GetDeg();

                    b_differ_direction =
                        (fabs(f_cur_deg - f_next_deg) > 10 && o_drive_info_end.f_curve_radius == 0) ? true : false;

                    NLOG(info) << "b_differ_direction : "<< b_differ_direction;
                }
            }
        }

        NLOG(info) << "b_differ_direction : "<< b_differ_direction << "b_stop_node : "<< b_stop_node;

        if (b_differ_direction || b_stop_node || b_avoid_type_is_difer) {
            if (b_differ_direction)
                NLOG(info) << "b_differ_direction " << int(o_drive_info.e_drive_type) << " / " << int(o_drive_info_end.e_drive_type);
            NLOG(info) << "DEVIDE! b_differ_direction " << b_differ_direction << " b_stop_node " << b_stop_node << " b_avoid_type_is_difer "
                       << b_avoid_type_is_difer;
            PathStack(vec_mission_stack, vec_path, o_msg.back().GetID(), vec_node_origin, o_msg.back().GetName());
            vec_mission_stack.back().o_goal_pos = o_end_node;

            // for camera roi, insert start node drive info
            vec_mission_stack.back().o_goal_pos.GetDriveInfo().n_camera_index = o_start_node.GetConstDriveInfo().n_camera_index;
            vec_mission_stack.back().o_goal_pos.GetDriveInfo().d_camera_roi_x_m = o_start_node.GetConstDriveInfo().d_camera_roi_x_m;
            vec_mission_stack.back().o_goal_pos.GetDriveInfo().d_camera_roi_y_m = o_start_node.GetConstDriveInfo().d_camera_roi_y_m;
            vec_mission_stack.back().o_goal_pos.GetDriveInfo().d_camera_roi_z_m = o_start_node.GetConstDriveInfo().d_camera_roi_z_m;
            vec_mission_stack.back().o_goal_pos.GetDriveInfo().d_camera_roi2_x_m = o_start_node.GetConstDriveInfo().d_camera_roi2_x_m;
            vec_mission_stack.back().o_goal_pos.GetDriveInfo().d_camera_roi2_y_m = o_start_node.GetConstDriveInfo().d_camera_roi2_y_m;
            vec_mission_stack.back().o_goal_pos.GetDriveInfo().d_camera_roi2_z_m = o_start_node.GetConstDriveInfo().d_camera_roi2_z_m;
            vec_node_origin.clear();
        }
    }

    if (vec_path.size() > 0)  // 경로가 남아 있다면
    {
        NLOG(info) << "vec_path.size() > 0";
        PathStack(vec_mission_stack, vec_path, o_msg.back().GetID(), vec_node_origin, o_msg.back().GetName());
        vec_mission_stack.back().o_goal_pos = o_msg.back();
    }
    return vec_mission_stack;
}

void MissionManager::PathStack(
    std::vector<NaviFra::PathDescription>& vec_mission_stack, std::vector<NaviFra::Pos>& vec_path, string goal_node_id,
    vector<NaviNode> vec_node_origin, string goal_name)
{
    try
    {
        // 현재노드 다음노드 바꾸는 기준을 파라미터로 조절 // nate
        vector<int> vec_arcend_idx;
        vector<float> vec_arcend_speed;
        vector<float> vec_origin_speed;
        if (vec_path.size() > 10) {
            for (int i = 10; i < vec_path.size() - 10; i++)  // 곡선 경로 만들기..
            {
                for (int j = 0; j < vec_node_origin.size(); j++) {
                    if (CoreCalculator::CalcPosDistance_(vec_path.at(i), vec_node_origin.at(j)) <= 0.02)  // 커브인 노드에서만 체크
                    {
                        // float f_deg = fabs(vec_path.at(i + 5).GetDeg() - vec_path.at(i - 5).GetDeg());
                        // if (f_deg > 180)
                        //     f_deg = fabs(f_deg - 360);
                        // LOG_INFO("%d f_deg  %.3f", j, f_deg);
                        float f_deg_1 =
                            atan2(vec_path.at(i).GetYm() - vec_path.at(i - 5).GetYm(), vec_path.at(i).GetXm() - vec_path.at(i - 5).GetXm()) *
                            RADtoDEG;
                        float f_deg_2 =
                            atan2(vec_path.at(i + 5).GetYm() - vec_path.at(i).GetYm(), vec_path.at(i + 5).GetXm() - vec_path.at(i).GetXm()) *
                            RADtoDEG;
                        float f_deg_diff = fabs(f_deg_2 - f_deg_1);
                        if (f_deg_diff > 180)
                            f_deg_diff = fabs(f_deg_diff - 360);
                        LOG_INFO("%d f_deg_diff  %.3f", j, f_deg_diff);

                        if (f_deg_diff > 5 && f_deg_diff < 175.0)  // 현재노드가 바뀌는 곳이나, 마지막 노드체크
                        {
                            NLOG(info) << "curve angle" << f_deg_diff;
                            NLOG(info) << i << " curve " << vec_path.at(i).GetDeg() << " / " << vec_path.at(i - 1).GetDeg() << " / "
                                    << vec_node_origin.at(j).GetDriveInfo().f_curve_radius;
                            Pos::DriveInfo_t o_front_drive = vec_path.at(i).GetDriveInfo();
                            // o_front_drive.n_avoid_type = 0;
                            // o_front_drive.b_avoidance_left = false;
                            // o_front_drive.b_avoidance_right = false;

                            // if (vec_path.at(i).GetDriveInfo().e_drive_type == Pos::DRIVE_TYPE::FRONT ||
                            //     vec_path.at(i).GetDriveInfo().e_drive_type == Pos::DRIVE_TYPE::REAR) {
                            NLOG(info) << "curve in";
                            int n_front_idx = i + int(vec_node_origin.at(j).GetDriveInfo().f_curve_radius * 100);
                            int n_rear_idx = i - int(vec_node_origin.at(j).GetDriveInfo().f_curve_radius * 100);
                            if (vec_path.size() <= n_front_idx)
                                n_front_idx = vec_path.size() - 1;
                            if (n_rear_idx < 0)
                                n_rear_idx = 0;

                            int n_front_gap = n_front_idx - i;
                            int n_rear_gap = i - n_rear_idx;
                            NLOG(info) << "curve n_front_gap " << i << " /// " << n_front_idx << " / n_rear_gap " << n_rear_idx;

                            int n_small_gap = std::min(n_front_gap, n_rear_gap);
                            if (n_small_gap <= 0)
                                continue;
                            NLOG(info) << "curve n_small_gap " << n_small_gap;
                            vector<Pos> vec_arc = MakeArcPath(vec_path.at(i - n_small_gap), vec_path.at(i), vec_path.at(i + n_small_gap));
                            NLOG(info) << "curve vec_arc " << vec_arc.size();

                            o_front_drive.f_linear = vec_node_origin.at(j).GetDriveInfo().f_linear;
                            if (o_front_drive.f_linear > 1.2)
                                o_front_drive.f_linear = 1.2;
                            o_front_drive.f_circle_pos_x = vec_arc.front().GetDriveInfo().f_circle_pos_x;
                            o_front_drive.f_circle_pos_y = vec_arc.front().GetDriveInfo().f_circle_pos_y;
                            o_front_drive.f_circle_angle = vec_arc.front().GetDriveInfo().f_circle_angle;
                            for (int k = 0; k < vec_arc.size(); k++) {
                                vec_arc.at(k).SetDriveInfo(o_front_drive);
                                vec_arc.at(k).GetDriveInfo().e_curve_type = Pos::CURVE;
                                vec_arc.at(k).GetDriveInfo().f_curve_radius = vec_node_origin.at(j).GetDriveInfo().f_curve_radius;
                                vec_arc.at(k).SetCurveType(Pos::CURVE);
                                if (vec_path.at(i).GetDriveInfo().e_drive_type == Pos::DRIVE_TYPE::FRONT_DIAGONAL ||
                                    vec_path.at(i).GetDriveInfo().e_drive_type == Pos::DRIVE_TYPE::REAR_DIAGONAL) {
                                    vec_arc.at(k).SetDeg(vec_path.at(i + n_small_gap).GetDeg());
                                }
                                if (k < vec_arc.size() / 2) {
                                    vec_arc.at(k).SetNowNodeName(vec_path.at(i).GetNowNodeName());
                                    vec_arc.at(k).SetNowNodeID(vec_path.at(i).GetNowNodeID());
                                    vec_arc.at(k).SetNextNodeName(vec_path.at(i).GetNextNodeName());
                                    vec_arc.at(k).SetNextNodeID(vec_path.at(i).GetNextNodeID());
                                }
                                else {
                                    vec_arc.at(k).SetNowNodeName(vec_path.at(i + 2).GetNowNodeName());
                                    vec_arc.at(k).SetNowNodeID(vec_path.at(i + 2).GetNowNodeID());
                                    vec_arc.at(k).SetNextNodeName(vec_path.at(i + 2).GetNextNodeName());
                                    vec_arc.at(k).SetNextNodeID(vec_path.at(i + 2).GetNextNodeID());
                                }
                            }
                            NLOG(info) << "curve 1 vec_path " << vec_path.size();
                            vec_path.erase(vec_path.begin() + i - n_small_gap, vec_path.begin() + i + n_small_gap);
                            NLOG(info) << "curve 2 vec_path " << vec_path.size();
                            vec_path.insert(vec_path.begin() + i - n_small_gap, vec_arc.begin(), vec_arc.end());
                            NLOG(info) << "curve 3 vec_path " << vec_path.size();
                            vec_arcend_idx.emplace_back(i - n_small_gap + vec_arc.size());
                            vec_arcend_speed.emplace_back(o_front_drive.f_linear);
                            vec_origin_speed.emplace_back(vec_node_origin.at(j).GetDriveInfo().f_linear);
                            i += n_small_gap;
                            // }
                        }
                    }
                }
            }
        }

        for (int i = 0; i < vec_arcend_idx.size(); i++) {
            LOG_INFO("arcnate vec_arcend_idx %d %d", vec_arcend_idx.at(i), vec_path.size());
            int n_acc_dist = st_param_.n_hacs_curve_speed_up_dist_cm;

            if (vec_arcend_idx.at(i) < vec_path.size() - (n_acc_dist + 30)) {
                float f_start = vec_arcend_speed.at(i);
                float f_end = vec_origin_speed.at(i);
                LOG_INFO("arcnate %f %f", f_start, f_end);

                for (int j = 0; j < n_acc_dist; j++)  // n_acc_distcm 서서히 가속
                {
                    if(vec_arcend_idx.at(i) + j >= vec_path.size())
                        break;
                    Pos::DriveInfo_t o_drive = vec_path.at(vec_arcend_idx.at(i) + j).GetDriveInfo();
                    o_drive.f_linear = f_start + (f_end - f_start) / float(n_acc_dist) * float(j);
                    if (o_drive.f_linear > f_end)
                        o_drive.f_linear = f_start;
                    vec_path.at(vec_arcend_idx.at(i) + j).SetDriveInfo(o_drive);
                }
            }
        }

        int n_change_idx = int(st_param_.f_current_node_change_before_m * 100);  // m*100 -> index로 변환됨, 1cm -> 1idx
        if (n_change_idx > 0) {
            for (int i = 1; i < vec_path.size(); i++) {
                if (i - n_change_idx > 0) {
                    if (vec_path.at(i).GetNowNodeID() != vec_path.at(i - 1).GetNowNodeID() ||
                        i == vec_path.size() - 1)  // 현재노드가 바뀌는 곳이나, 마지막 노드일 경우 노드이름 앞으로 당겨주기
                    {
                        string s_node_name = vec_path.at(i).GetNowNodeName();
                        string s_node_id = vec_path.at(i).GetNowNodeID();
                        for (int j = 1; j < n_change_idx; j++) {
                            vec_path.at(i - j).SetNowNodeName(s_node_name);
                            vec_path.at(i - j).SetNowNodeID(s_node_id);
                        }
                    }
                    // if (vec_path.at(i).GetNextNodeName() != vec_path.at(i - 1).GetNextNodeName()) // 다음노드가 바뀌는 곳일 경우 노드이름
                    // 앞으로 당겨주기
                    if (vec_path.at(i).GetNextNodeID() !=
                        vec_path.at(i - 1).GetNextNodeID())  // 다음노드가 바뀌는 곳일 경우 노드이름 앞으로 당겨주기
                    {
                        string s_node_name = vec_path.at(i).GetNextNodeName();
                        string s_node_id = vec_path.at(i).GetNextNodeID();
                        for (int j = 1; j < n_change_idx; j++) {
                            vec_path.at(i - j).SetNextNodeName(s_node_name);
                            vec_path.at(i - j).SetNextNodeID(s_node_id);
                        }
                    }
                }
            }
        }

        NaviFra::PathDescription o_mission;
        // PathSmoothing o_path_smoothing;
        // o_path_smoothing.SetParam(0.01, 50);
        LOG_INFO("vec_path %d", vec_path.size());
        // std::vector<NaviFra::Pos> vec_smooth_path = o_path_smoothing.smoothingCubicSplinePath(vec_path);
        // LOG_INFO("vec_path %d smooth %d", vec_path.size(), vec_smooth_path.size());
        // o_mission.vec_path = vec_smooth_path;
        o_mission.vec_path = vec_path;
        o_mission.o_goal_pos = vec_path.back();
        o_mission.s_goal_name = vec_path.back().GetGoalNodeName();
        if (o_mission.s_goal_name.size() == 0) {
            o_mission.s_goal_name = goal_name;
        }
        o_mission.s_goal_node_id = goal_node_id;
        if (o_mission.vec_path.front().GetDriveInfo().e_drive_type == NaviFra::Pos::DRIVE_TYPE::REAR ||
            o_mission.vec_path.front().GetDriveInfo().e_drive_type == NaviFra::Pos::DRIVE_TYPE::REAR_DIAGONAL) {
            o_mission.f_yaw_bias = CoreCalculator::WrapAnglePiToPiRad_(o_mission.f_yaw_bias + M_PI);
            NLOG(info) << "path apply::=" << o_mission.f_yaw_bias * 180 / 3.14;
        }
        vec_mission_stack.emplace_back(o_mission);
        vec_path.clear();
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("Exception in PathStack: %s", e.what());
    }
}

int MissionManager::StartMission(const vector<NaviNode>& o_msg)
{
    std::chrono::duration<double> sec_teminate = std::chrono::steady_clock::now() - tp_teminate_time_;
    if (sec_teminate.count() < 0.3) {
        LOG_WARNING("mission terminate running... wait 200ms");
        boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
        LOG_WARNING("mission terminate running... wait done");
    }
    b_addgoal_flag_ = o_auto_move_mission_.GetRunningStatus();

    ClearMission();

    std::vector<NaviFra::PathDescription> vec_mission_stack;

    if (o_msg.size() > 1)
        vec_mission_stack = MakeDevidedPath(o_msg);
    else {
        // spinturn 처리
        NaviFra::PathDescription o_path_description;

        std::vector<Pos> vec_path;
        vec_path.emplace_back(o_msg.back());
        vec_path.back().SetNowNodeName(o_msg.at(0).GetName());
        vec_path.back().SetNowNodeID(o_msg.at(0).GetID());
        vec_path.back().SetNextNodeName(o_msg.at(0).GetName());
        vec_path.back().SetNextNodeID(o_msg.at(0).GetID());
        Pos o_goal_pos = o_msg.at(0);
        string s_start_name = o_msg.at(0).GetName();
        string s_start_node_id = o_msg.at(0).GetID();
        string s_goal_name = o_msg.at(0).GetName();
        string s_goal_node_id = o_msg.at(0).GetID();
        int n_start_align_flag = 1;
        int n_end_align_flag = 1;

        o_path_description.vec_path = vec_path;
        o_path_description.o_goal_pos = o_goal_pos;
        o_path_description.s_start_name = s_start_name;
        o_path_description.s_start_node_id = s_start_node_id;
        o_path_description.s_goal_name = s_goal_name;
        o_path_description.s_goal_node_id = s_goal_node_id;
        o_path_description.n_start_align_flag = n_start_align_flag;
        o_path_description.n_end_align_flag = n_end_align_flag;
        vec_mission_stack.emplace_back(o_path_description);
    }

    LOG_INFO("Number of vec_mission_stack::%d", vec_mission_stack.size());

    // 로봇이 있는 경로의 마지막에 가까운 path만 남기고 삭제
    int n_close_idx = -1;
    float f_min_dist = 1000;
    for (int i = 0; i < vec_mission_stack.size(); i++) {
        if (vec_mission_stack.at(i).vec_path.size() > 0) {
            int n_idx = CoreCalculator::FindMinDistanceIdxFromPosVector_(o_robot_pos_, vec_mission_stack.at(i).vec_path);
            float f_dist = CoreCalculator::CalcPosDistance_(o_robot_pos_, vec_mission_stack.at(i).vec_path.at(n_idx));
            if (f_dist < f_min_dist) {
                f_min_dist = f_dist;
                n_close_idx = i;
            }
        }
    }

    if (n_close_idx != -1 && f_min_dist < 0.2) {  // 가장 가까운 path 전의 path는 삭제 (불필요)
        NLOG(info) << "n_close_idx " << n_close_idx << " f_min_dist " << f_min_dist;
        vec_mission_stack.erase(vec_mission_stack.begin(), vec_mission_stack.begin() + n_close_idx);
    }
    else {
        LOG_ERROR("No path is close to the robot position");
        // 복귀 경로 만들던가 해야할듯....
        NLOG(info) << "n_close_idx " << n_close_idx << " f_min_dist " << f_min_dist;
    }

    if (true == vec_mission_stack.empty()) {
        LOG_ERROR("Mission is empty!!! Input path is not empty!! Function bug!! Function bug!! Function bug!!!!");
        o_auto_move_mission_.NotifyCbFunc("navi_alarm_callback", static_cast<int>(AutoMoveMission::NAVIGATION_ALARM::PATH_NOT_CREATED));
        return ERR_MISSION_GENERATION_FAILURE;
    }
    else {
        LOG_INFO("vec_mission_stack.back().o_goal_pos::%d, %f, %f, %f", 
            vec_mission_stack.back().o_goal_pos.GetType(), 
            vec_mission_stack.back().o_goal_pos.GetXm(), 
            vec_mission_stack.back().o_goal_pos.GetYm(), 
            vec_mission_stack.back().o_goal_pos.GetDeg());

        // except progress for delete passed mission. erase mission stack in pass mission num
        LOG_INFO("Number of missions to accomplish::%d", vec_mission_stack.size());

        mission_repo_.SetMissionBundle(vec_mission_stack);
        b_start_flag_ = true;
    }
    vec_global_path_.clear();
    for (auto& path : vec_mission_stack) {
        vec_global_path_.insert(vec_global_path_.end(), path.vec_path.begin(), path.vec_path.end());
    }

    if (vec_global_path_.size() > 1) {
        nav_msgs::Path path;
        path.header.seq = 0;
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();

        static float f_ui_dist = 0;
        {
            for (size_t i = 1; i < vec_global_path_.size(); i++) {
                f_ui_dist += hypot(
                    vec_global_path_.at(i).GetXm() - vec_global_path_.at(i - 1).GetXm(),
                    vec_global_path_.at(i).GetYm() - vec_global_path_.at(i - 1).GetYm());
                if (f_ui_dist > 0.05 || (vec_global_path_.at(i).GetDriveInfo().e_curve_type == Pos::CURVE && f_ui_dist > 0.05)) {
                    f_ui_dist = 0;
                    geometry_msgs::PoseStamped pose;
                    pose.pose.position.x = vec_global_path_.at(i).GetXm();
                    pose.pose.position.y = vec_global_path_.at(i).GetYm();
                    path.poses.emplace_back(pose);
                }
            }

            static ros::Publisher localpath_pub_ = node_handle_.advertise<nav_msgs::Path>("/NaviFra/visualize/ui_global_path", 50, true);
            localpath_pub_.publish(path);
        }
    }
    return 0;
}

void MissionManager::RecvMissionCompleteCbMsg(const boost::any& any_type_var)
{
    int n_var_data = boost::any_cast<int>(any_type_var);
    switch (n_var_data) {
        case AutoMoveMission::CB_MSG_MISSION_COMPLETED:  // mission 정상 수행 완료(목적지 도착 경우)
        {
            LOG_INFO("Mission is compeleted successfully!! Just wait for all proc termination!!!");
            tp_teminate_time_ = std::chrono::steady_clock::now();
            TerminateMission(false);  // behavior proc을 종료하라고 명령
            b_arrive_flag_ = true;
            break;
        }
    }
}

void MissionManager::CarryoutMission()
{
    static int n_mission_cnt = 0;
    while (ros::ok()) {
        boost::this_thread::sleep(boost::posix_time::millisec(10));
        if (mission_repo_.GetRemainingMissions() != 0 && (b_arrive_flag_ == true || b_start_flag_ == true)) {
            b_arrive_flag_ = false;
            b_start_flag_ = false;
            LOG_INFO(" mission_repo_.GetRemainingMissions()=%d", mission_repo_.GetRemainingMissions());
            n_now_mission_num_ = mission_repo_.GetRemainingMissions();

            // 현재 수행해야할 미션을 받아온다.
            NaviFra::PathDescription current_mission = mission_repo_.GetCurrentMission();

            // if not final local path, goal type remove
            if (mission_repo_.GetRemainingMissions() > 1)
                current_mission.o_goal_pos.SetType(Pos::NODE_TYPE::NONE);
            LOG_WARNING("******************************Important message(not error)*************************");
            LOG_WARNING("*          carryout function will be watied until mission complete!!!             *");
            LOG_WARNING("*   Always make sure that function RecvMissionCompleteCbMsg is called!!!!!!   *");
            LOG_WARNING("******************************Important message end********************************");

            if (current_mission.vec_path.size() > 0) {
                o_auto_move_mission_.SetBehaviorMission(current_mission, b_addgoal_flag_);  // behavior에 미션 설정
                o_auto_move_mission_.ExecuteJob();  // behavior 미션 실행
            }
            else {
                LOG_ERROR("Mission path is empty!!! Function bug!! Function bug!! Function bug!!!!");
                o_auto_move_mission_.NotifyCbFunc(
                    "navi_alarm_callback", static_cast<int>(AutoMoveMission::NAVIGATION_ALARM::PATH_NOT_CREATED));
            }

            LOG_INFO("mission_repo_.GetTermiateState() %d", mission_repo_.GetTermiateState());

            // goal 도착시 현재 스택의 goal이 맞는지 확인 후 다음 스택으로 넘기기 nate
            int n_remaining_mission_cnt = mission_repo_.CompleteMission();  // 미션 리포에서 끝난 미션은 삭제
            LOG_INFO(" in while n_remaining_mission_cnt=%d, n_mission_cnt=%d", n_remaining_mission_cnt, n_mission_cnt);
        }
        else if (mission_repo_.GetRemainingMissions() == 0 && b_arrive_flag_ == true) {
            b_arrive_flag_ = false;
            n_mission_cnt = 0;
            n_now_mission_num_ = 0;
            LOG_INFO("All missions have been completed. arrived at : %s", str_goal_id_pre_.c_str());
            o_auto_move_mission_.NotifyCbFunc("navi_alarm_callback", static_cast<int>(AutoMoveMission::NAVIGATION_ALARM::GOAL_ARRIVED));
            vec_global_path_.clear();
            str_goal_id_pre_ = "";
        }
    }
}

std::vector<Pos> MissionManager::MakeArcPath(const NaviFra::Pos& pos_1, const NaviFra::Pos& pos_2, const NaviFra::Pos& pos_3)
{
    vector<Pos> vec_result;
    // make arc path for diagonal motion

    Pos s_pos = pos_1;
    s_pos.SetRad(atan2(pos_2.GetYm() - pos_1.GetYm(), pos_2.GetXm() - pos_1.GetXm()));
    Pos e_pos = pos_3;
    e_pos.SetRad(atan2(pos_3.GetYm() - pos_2.GetYm(), pos_3.GetXm() - pos_2.GetXm()));
    // NLOG(info)<<"dist12 "<<CoreCalculator::CalcPosDistance_(pos_1, pos_2)<<" dist23 "<<CoreCalculator::CalcPosDistance_(pos_2,
    // pos_3)<<endl; NLOG(info)<<"s_pos "<<s_pos.GetDeg()<<" e_pos "<<e_pos.GetDeg()<<endl;
    float f_dist_pos = hypot(s_pos.GetXm() - e_pos.GetXm(), s_pos.GetYm() - e_pos.GetYm());
    float f_radius_circle = f_dist_pos / 2 / sin(abs(s_pos.GetRad() - e_pos.GetRad()) / 2);
    if (f_radius_circle < 0.01)
        return vec_result;
    // NLOG(info)<<__LINE__<<endl;

    float fd2 = f_dist_pos / 2;
    float fOffset = sqrt(f_radius_circle * f_radius_circle - fd2 * fd2);
    float fplusx = fOffset * (e_pos.GetYm() - s_pos.GetYm()) / f_dist_pos;
    float fplusy = fOffset * (e_pos.GetXm() - s_pos.GetXm()) / f_dist_pos;
    Pos center_pos;
    Pos center_pos_1;
    center_pos_1.SetXm((s_pos.GetXm() + e_pos.GetXm()) / 2 - fplusx);
    center_pos_1.SetYm((s_pos.GetYm() + e_pos.GetYm()) / 2 + fplusy);
    Pos center_pos_2;
    center_pos_2.SetXm((s_pos.GetXm() + e_pos.GetXm()) / 2 + fplusx);
    center_pos_2.SetYm((s_pos.GetYm() + e_pos.GetYm()) / 2 - fplusy);

    Pos dir_s(cos(s_pos.GetRad()), sin(s_pos.GetRad()));
    Pos dir_v1(center_pos_1.GetXm() - s_pos.GetXm(), center_pos_1.GetYm() - s_pos.GetYm());
    Pos dir_v2(center_pos_2.GetXm() - s_pos.GetXm(), center_pos_2.GetYm() - s_pos.GetYm());
    // dot product for find center of circle
    bool b_clock_wise_dir = false;
    if (fabs(CoreCalculator::CalcPosDotProduct_(dir_s, dir_v1)) < 0.1) {
        center_pos = center_pos_1;
        b_clock_wise_dir = false;
    }
    else if (fabs(CoreCalculator::CalcPosDotProduct_(dir_s, dir_v2)) < 0.1) {
        center_pos = center_pos_2;
        b_clock_wise_dir = true;
    }

    float f_deg_error = s_pos.GetDeg() - e_pos.GetDeg();
    if (f_deg_error > 180)
        f_deg_error -= 360;
    else if (f_deg_error < -180)
        f_deg_error += 360;

    float f_arc_length = fabs(f_deg_error) * M_PI * f_radius_circle / 180;
    int N = (int)(f_arc_length / 0.01);

    float f_x = s_pos.GetXm() - center_pos.GetXm();
    float f_y = s_pos.GetYm() - center_pos.GetYm();
    float f_a = s_pos.GetRad();
    // NLOG(info)<<__LINE__<<endl;

    // make arc path point
    for (int j = 0; j < N; j++) {
        Pos tmp(f_x, f_y);
        if (b_clock_wise_dir == true) {
            tmp = CoreCalculator::TransformRotationDeg_(tmp, -0.01 * 180 / M_PI / f_radius_circle * j);
            tmp.SetRad(atan2(tmp.GetYm(), tmp.GetXm()) - M_PI / 2);
        }
        else {
            tmp = CoreCalculator::TransformRotationDeg_(tmp, 0.01 * 180 / M_PI / f_radius_circle * j);
            tmp.SetRad(atan2(tmp.GetYm(), tmp.GetXm()) + M_PI / 2);
        }
        tmp.SetXm(tmp.GetXm() + center_pos.GetXm());
        tmp.SetYm(tmp.GetYm() + center_pos.GetYm());
        tmp.GetDriveInfo().f_circle_pos_x = center_pos.GetXm();
        tmp.GetDriveInfo().f_circle_pos_y = center_pos.GetYm();
        tmp.GetDriveInfo().f_circle_angle = -f_deg_error;
        // NLOG(info)<<j<<" tmp "<<tmp.GetDeg()<<endl;
        // if (vec_result.size() > 0)
        // {
        //   float f_angle = atan2(tmp.GetYm() - vec_result.back().GetYm(), tmp.GetXm() - vec_result.back().GetXm());
        //   tmp.SetRad(f_angle);
        // }
        // else
        // {
        //   tmp.SetRad(f_a);
        // }
        vec_result.emplace_back(tmp);
    }

    // NLOG(info)<<__LINE__<<endl;

    // NLOG(info)<<"f_radius_circle "<<f_radius_circle<<" N "<<N<<" vec_result "<<vec_result.size()<<endl;

    return vec_result;
}

void MissionManager::SetLocalMapPtr(std::shared_ptr<const std::vector<int8_t>> map_ptr)
{
    o_auto_move_mission_.SetLocalMapPtr(map_ptr);
}

}  // namespace NaviFra
