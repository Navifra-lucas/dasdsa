#include "ros1/graph_map/graph_map_ros1.hpp"

#include "ros/package.h"

namespace NVFR {

GraphMapRos1::GraphMapRos1(
    const std::string &node_name)
    : s_node_name_(node_name)
    , b_initialized_(false)
{
    // initialize
    Initialize();

}

GraphMapRos1::~GraphMapRos1()
{
    Terminator();
    o_graph_map_ptr_.reset();
    LOG_INFO("Destruct GraphMapRos1");
    std::this_thread::sleep_for(std::chrono::milliseconds(20UL));
    logger_->TerminateLogger();
    logger_.reset();
}

void GraphMapRos1::Terminator()
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

    ros::shutdown();
}

void GraphMapRos1::Initialize()
{
    // set directory of configs
    ANSWER::Path::GetInstance().SetAllPaths(
        ros::package::getPath("answer"));

    // load interface param
    ANSWER::Configurator::GetInstance().LoadParameters("ros");
    ANSWER::Configurator::GetInstance().LoadParameters("graph_map_interface");

    // set logger
    auto log_level = ANSWER::Configurator::GetInstance()
        .GetParamValue("graph_map_interface", "base", "log_level")
        .convert<std::string>();
    auto base_path = ANSWER::Configurator::GetInstance()
        .GetParamValue("ros", "base", "base_path")
        .convert<std::string>();
    logger_ = std::make_shared<ANSWER::Logger>(
        s_node_name_, log_level, base_path);

    std::string s_map_file;
    if (!RequestMapDirectory(s_map_file))
    {
        LOG_ERROR("Fail to request map directory");
        return;
    }

    o_graph_map_ptr_ = std::make_unique<GraphMap>(s_map_file);
    if (!o_graph_map_ptr_->LoadMap())
    {
        LOG_ERROR("Fail to load map");
        return;
    }

    // set param
    LoadParam();

    // regist pub/sub
    RegistPublisher();
    RegistSubscriber();
    RegistServiceServer();

    // success initialization
    b_initialized_ = true;
    LOG_INFO("Success initialization");

}

void GraphMapRos1::LoadParam()
{
    // load & read navigator param
    // NavigatorParam_t st_param = NaviParamManager::LoadNaviParam();
    o_graph_map_ptr_->SetParam();
}

void GraphMapRos1::RegistPublisher()
{
    // mission node path
    publishers_[KEY::TOPICS::NAVI_MISSION_NODE_PATH] =
        std::make_shared<ros::Publisher>(
            nh_.advertise<answer_msgs::MissionNodePath>(
                NAME::TOPICS::NAVI_MISSION_NODE_PATH, 10));
    // mission response
    publishers_[KEY::TOPICS::NAVI_MISSION_RESPONSE] =
        std::make_shared<ros::Publisher>(
            nh_.advertise<answer_msgs::MissionResponse>(
                NAME::TOPICS::NAVI_MISSION_RESPONSE, 10));

}

void GraphMapRos1::RegistSubscriber()
{
    // robot pose
    subscriptions_[KEY::TOPICS::ROBOT_POSE] =
        std::make_shared<ros::Subscriber>(
            nh_.subscribe(
                ANSWER::TOPICS::ROBOT_POSE, 10,
                &GraphMapRos1::GetRobotPose, this));

    // others
    subscriptions_[KEY::TOPICS::PARAM_READ_ALL] =
        std::make_shared<ros::Subscriber>(
            nh_.subscribe(
                NAME::TOPICS::PARAM_READ_ALL, 10,
                &GraphMapRos1::GetParamReadSignal, this));
    subscriptions_[KEY::TOPICS::PARAM_READ_NODE] =
        std::make_shared<ros::Subscriber>(
            nh_.subscribe(
                NAME::TOPICS::PARAM_READ_NODE + s_node_name_, 10,
                &GraphMapRos1::GetParamReadSignal, this));
    subscriptions_[KEY::TOPICS::LOG_LV_ALL] =
        std::make_shared<ros::Subscriber>(
            nh_.subscribe(
                NAME::TOPICS::LOG_LV_ALL, 10,
                &GraphMapRos1::GetLogLv, this));
    subscriptions_[KEY::TOPICS::LOG_LV_NODE] =
        std::make_shared<ros::Subscriber>(
            nh_.subscribe(
                NAME::TOPICS::LOG_LV_NODE + s_node_name_, 10,
                &GraphMapRos1::GetLogLv, this));

}

void GraphMapRos1::RegistServiceServer()
{
    srv_servers_[KEY::SERVICES::GRAPH_MAP_LOAD_MAP] =
        std::make_shared<ros::ServiceServer>(
            nh_.advertiseService(
                NAME::SERVICES::GRAPH_MAP_LOAD_MAP,
                &GraphMapRos1::ResponseLoadMap, this));
    srv_servers_[KEY::SERVICES::GRAPH_MAP_LOCALIZATION] =
        std::make_shared<ros::ServiceServer>(
            nh_.advertiseService(
                NAME::SERVICES::GRAPH_MAP_LOCALIZATION,
                &GraphMapRos1::ResponseLocalization, this));
    srv_servers_[KEY::SERVICES::GRAPH_MAP_MISSION_NODE] =
        std::make_shared<ros::ServiceServer>(
            nh_.advertiseService(
                NAME::SERVICES::GRAPH_MAP_MISSION_NODE,
                &GraphMapRos1::ResponseMissionGraphNode, this));
    srv_servers_[KEY::SERVICES::GRAPH_MAP_MISSION_NODE_LIST] =
        std::make_shared<ros::ServiceServer>(
            nh_.advertiseService(
                NAME::SERVICES::GRAPH_MAP_MISSION_NODE_LIST,
                &GraphMapRos1::ResponseMissionGraphNodeList, this));

}

bool GraphMapRos1::RequestMapDirectory(std::string& s_map_file)
{
    ros::ServiceClient client = nh_.serviceClient<answer_msgs::CommonService>(NAME::SERVICES::GRAPH_MAP_DIRECTORY);

    answer_msgs::CommonService srv;
    srv.request.data = "";

    while (ros::ok()) {
        if (client.call(srv)) {
            s_map_file = srv.response.reason;
            LOG_INFO("Result: {}", s_map_file.c_str());
            return true;
        }
        LOG_ERROR("Failed to call service {}", NAME::SERVICES::GRAPH_MAP_DIRECTORY.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(1000UL));
    }

    LOG_WARN("Interrupted while waiting for the service: /{}",
        NAME::SERVICES::GRAPH_MAP_DIRECTORY.c_str());
    return false;
}

}  // namespace NVFR
