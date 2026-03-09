#include "ros1/explorer/explorer_ros1.hpp"

#include "ros/package.h"

namespace NVFR {

ExplorerRos1::ExplorerRos1(const std::string &node_name)
    : s_node_name_(node_name)
    , s_image_directory_(
          std::string(getenv("HOME")) +
          "/navifra_solution/navicore/configs/explorer/cv_image/")
    , b_initialized_(false)
    , visualizer_(nh_, "Expr")
{
    // initialize
    Initialize();

    if (!WaitDataSet()) {
        Terminator();
        return;
    }

    if (!FirstMovement()) {
        Terminator();
        return;
    }

    b_initialized_ = true;
}

ExplorerRos1::~ExplorerRos1()
{
    Terminator();
    o_explorer_ptr_->Terminator();
    o_explorer_ptr_.reset();
    LOG_INFO("Destruct ExplorerRos1");
    std::this_thread::sleep_for(std::chrono::milliseconds(20UL));
    logger_->TerminateLogger();
    logger_.reset();
}

void ExplorerRos1::Terminator()
{
    for (auto &sub : subscriptions_) {
        sub.second->shutdown();
    }
    for (auto &pub : publishers_) {
        pub.second->shutdown();
    }
    for (auto &server : srv_servers_) {
        server.second->shutdown();
    }
    for (auto &client : srv_clients_) {
        client.second->shutdown();
    }

    ros::shutdown();
}

void ExplorerRos1::Initialize()
{
    // set directory of configs
    ANSWER::Path::GetInstance().SetAllPaths(
        ros::package::getPath("answer"));

    // load interface param
    ANSWER::Configurator::GetInstance().LoadParameters("ros");
    ANSWER::Configurator::GetInstance().LoadParameters("expr_interface");

    // set logger
    auto log_level = ANSWER::Configurator::GetInstance()
        .GetParamValue("expr_interface", "base", "log_level")
        .convert<std::string>();
    auto base_path = ANSWER::Configurator::GetInstance()
        .GetParamValue("ros", "base", "base_path")
        .convert<std::string>();
    logger_ = std::make_shared<ANSWER::Logger>(
        s_node_name_, log_level, base_path);

    // check directory
    if (!FS::CreateDirectory(s_image_directory_))
        exit(1);

    // explorer
    o_explorer_ptr_ = std::make_unique<Explorer>();

    // set param
    LoadParam();

    o_explorer_ptr_->RegistCbFuncVoid(KEY::XT, &ExplorerRos1::Terminator, this);
    o_explorer_ptr_->RegistCbFunc(KEY::XD, &ExplorerRos1::Draw, this);

    // pub
    publishers_[KEY::TOPICS::EXPR_MISSION_STATUS] =
        std::make_shared<ros::Publisher>(nh_.advertise<std_msgs::Int32>(
            NAME::TOPICS::EXPR_MISSION_STATUS, 10));
    o_explorer_ptr_->RegistCbFunc(KEY::XS, &ExplorerRos1::SendStatus, this);
    publishers_[KEY::TOPICS::NAVI_MISSION_ALIGN] =
        std::make_shared<ros::Publisher>(
            nh_.advertise<answer_msgs::MissionAlign>(
                NAME::TOPICS::NAVI_MISSION_ALIGN, 10));
    o_explorer_ptr_->RegistCbFunc(
        KEY::XR, &ExplorerRos1::SendAlignMission, this);
    publishers_[KEY::TOPICS::NAVI_MISSION_NODE_PATH] =
        std::make_shared<ros::Publisher>(
            nh_.advertise<answer_msgs::MissionNodePath>(
                NAME::TOPICS::NAVI_MISSION_NODE_PATH, 10));
    o_explorer_ptr_->RegistCbFunc(
        KEY::XP, &ExplorerRos1::SendPathMission, this);
    publishers_[KEY::TOPICS::NAVI_MISSION_EXPLORE] =
        std::make_shared<ros::Publisher>(
            nh_.advertise<answer_msgs::MissionExplore>(
                NAME::TOPICS::NAVI_MISSION_EXPLORE, 10));
    o_explorer_ptr_->RegistCbFunc(
        KEY::XE, &ExplorerRos1::SendExploreMission, this);

    // service-client
    srv_clients_[KEY::SERVICES::GLOBAL_MAP] =
        std::make_shared<ros::ServiceClient>(
            nh_.serviceClient<answer_msgs::GlobalMap>(
                ANSWER::SERVICES::GLOBAL_MAP, 10));
    o_explorer_ptr_->RegistCbFuncVoid(
        KEY::XM, &ExplorerRos1::ReqMapImage, this);

    // sub
    std::string s_msg_type = "";
    std::string s_topic_name = "";

    s_topic_name = ANSWER::Configurator::GetInstance().GetParam<std::string>(
        "navi_interface", "topic_name", ANSWER::TOPICS::ROBOT_POSE,
        ANSWER::TOPICS::ROBOT_POSE);
    subscriptions_[KEY::TOPICS::ROBOT_POSE] = std::make_shared<ros::Subscriber>(
        nh_.subscribe(s_topic_name, 10, &ExplorerRos1::GetRobotPose, this));

    subscriptions_[KEY::TOPICS::NAVI_MISSION_RESPONSE] =
        std::make_shared<ros::Subscriber>(nh_.subscribe(
            NAME::TOPICS::NAVI_MISSION_RESPONSE, 10,
            &ExplorerRos1::GetNaviMissionResponse, this));
    subscriptions_[KEY::TOPICS::NAVI_STATUS] =
        std::make_shared<ros::Subscriber>(nh_.subscribe(
            NAME::TOPICS::NAVI_STATUS, 10, &ExplorerRos1::GetNaviStatus, this));
    subscriptions_[KEY::TOPICS::NAVI_ERROR] =
        std::make_shared<ros::Subscriber>(nh_.subscribe(
            NAME::TOPICS::NAVI_ERROR, 10, &ExplorerRos1::GetNaviError, this));
    subscriptions_[KEY::TOPICS::POSE_GRAPH_NODE] =
        std::make_shared<ros::Subscriber>(nh_.subscribe(
            ANSWER::TOPICS::POSE_GRAPH_NODE, 10, &ExplorerRos1::GetSlamNodes,
            this));
    subscriptions_[KEY::TOPICS::PARAM_READ_ALL] =
        std::make_shared<ros::Subscriber>(nh_.subscribe(
            NAME::TOPICS::PARAM_READ_ALL, 10, &ExplorerRos1::GetParamReadSignal,
            this));
    subscriptions_[KEY::TOPICS::PARAM_READ_NODE] =
        std::make_shared<ros::Subscriber>(nh_.subscribe(
            NAME::TOPICS::PARAM_READ_NODE + s_node_name_, 10,
            &ExplorerRos1::GetParamReadSignal, this));
    subscriptions_[KEY::TOPICS::LOG_LV_ALL] =
        std::make_shared<ros::Subscriber>(nh_.subscribe(
            NAME::TOPICS::LOG_LV_ALL, 10, &ExplorerRos1::GetLogLv, this));
    subscriptions_[KEY::TOPICS::LOG_LV_NODE] =
        std::make_shared<ros::Subscriber>(nh_.subscribe(
            NAME::TOPICS::LOG_LV_NODE + s_node_name_, 10,
            &ExplorerRos1::GetLogLv, this));
}

bool ExplorerRos1::WaitDataSet()
{
    // robot pose
    while (ros::ok()) {
        ros::spinOnce();
        if (o_explorer_ptr_->CheckRobot()) {
            Pose o_start_pose = o_explorer_ptr_->GetRobotPose();
            o_explorer_ptr_->SetStartPose(o_start_pose);
            LOG_INFO("Get Robot Pose");
            break;
        }
        LOG_TRACE(
            "waiting for the topic: /{} ...",
            ANSWER::TOPICS::ROBOT_POSE.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(100UL));
    }
    if (!ros::ok()) {
        LOG_WARN(
            "Interrupted while waiting for the topic: /{}",
            ANSWER::TOPICS::ROBOT_POSE.c_str());
        return false;
    }

    LOG_INFO("Ok (data set)");
    return true;
}

bool ExplorerRos1::FirstMovement()
{
    bool b_res = false;
    while (!b_res && ros::ok()) {
        o_service_client_handler_.Request("FirstMission");
        o_explorer_ptr_->StartExplore();
        ros::spinOnce();
        b_res = o_service_client_handler_.WaitResponseWithTimer(
            "FirstMission", 1000UL);
    }

    if (b_res) {
        LOG_INFO("First movement");
    }
    else {
        LOG_WARN("Interrupted while waiting for first path mission");
    }
    return b_res;
}

void ExplorerRos1::LoadParam()
{
    // load & read expr param
    ExprParam_t st_param = ExprParamManager::LoadExprParam();
    o_explorer_ptr_->SetParam(st_param);
}

void ExplorerRos1::Draw(const std::string &s_name, const cv::Mat &cv_image)
{
    cv::imwrite(s_image_directory_ + s_name + ".png", cv_image);
}

void ExplorerRos1::SendStatus(const int &n_status)
{
    std_msgs::Int32 msg;
    msg.data = n_status;
    publishers_[KEY::TOPICS::EXPR_MISSION_STATUS]->publish(msg);
}

void ExplorerRos1::SendAlignMission(const double &d_yaw_rad)
{
    answer_msgs::MissionAlign msg;
    msg.uuid = "uuid-explore-align";
    msg.n_align_direction = answer_msgs::MissionAlign::AUTO;
    msg.d_align_target_deg = d_yaw_rad * CM::Rad2Deg;
    publishers_[KEY::TOPICS::NAVI_MISSION_ALIGN]->publish(msg);
}

void ExplorerRos1::SendPathMission(
    const Pose &o_start_pose, const Pose &o_goal_pose)
{
    answer_msgs::MissionNodePath msg;
    msg.uuid = "uuid-explore-path";

    answer_msgs::EdgeInfo edge_info;
    edge_info.o_from_node.s_name = "__ES__";
    edge_info.o_to_node.s_name = "__EF__";
    edge_info.o_drive_info.n_move_dir = answer_msgs::DriveInfo::FORWARD;
    edge_info.o_drive_info.n_drive_type = answer_msgs::DriveInfo::BIKE;
    edge_info.o_drive_info.n_start_type = answer_msgs::DriveInfo::NORMAL;
    edge_info.o_drive_info.n_goal_type = answer_msgs::DriveInfo::NORMAL;
    edge_info.o_from_node.s_name = "__ES__";
    edge_info.o_to_node.s_name = "__EF__";
    edge_info.o_from_node.o_pose =
        MsgConverter::Convert<Pose, geometry_msgs::Pose2D>(o_start_pose);
    edge_info.o_to_node.o_pose =
        MsgConverter::Convert<Pose, geometry_msgs::Pose2D>(o_goal_pose);
    edge_info.n_path_type = answer_msgs::EdgeInfo::LINE;
    edge_info.d_target_speed = 0.5;
    msg.edge_array.emplace_back(edge_info);

    publishers_[KEY::TOPICS::NAVI_MISSION_NODE_PATH]->publish(msg);
}

void ExplorerRos1::SendExploreMission(const Pose &o_target_pose)
{
    LOG_INFO(
        "Explore target pose: ({:.3f},{:.3f})", o_target_pose.GetXm(),
        o_target_pose.GetYm());
    answer_msgs::MissionExplore msg;
    msg.uuid = "uuid-explore-mission";
    msg.d_target_speed = 0.5;
    msg.o_target_pose =
        MsgConverter::Convert<Pose, geometry_msgs::Pose2D>(o_target_pose);
    publishers_[KEY::TOPICS::NAVI_MISSION_EXPLORE]->publish(msg);
}

void ExplorerRos1::ReqMapImage()
{
    answer_msgs::GlobalMap srv;
    srv.request.header.frame_id = s_node_name_;
    while (ros::ok()) {
        srv.request.header.stamp = ros::Time::now();
        if (srv_clients_[KEY::SERVICES::GLOBAL_MAP]->call(srv)) {
            if (srv.response.success) {
                MapInfo_t st_map_info =
                    MsgConverter::Convert<nav_msgs::MapMetaData, MapInfo_t>(
                        srv.response.map.info);
                auto [b_success, cv_image] =
                    ICVT::mapToImage(st_map_info, srv.response.map.data);
                if (b_success) {
                    o_explorer_ptr_->SetImage(
                        cv_image, st_map_info.d_resolution_m,
                        -st_map_info.d_origin_x_m, -st_map_info.d_origin_y_m);
                }
            }
            if (o_explorer_ptr_->CheckMap()) {
                LOG_INFO("Receive Map");
                break;
            }
            else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000UL));
            }
        }
        else {
            // LOG_WARN(
            //     "waiting for the service: /{} ...",
            //     NAME::SERVICES::GLOBAL_MAP);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000UL));
        }
    }
    if (!ros::ok()) {
        // LOG_WARN(
        //     "Interrupted while waiting for the service: /{}",
        //     NAME::SERVICES::GLOBAL_MAP);
    }
}

}  // namespace NVFR
