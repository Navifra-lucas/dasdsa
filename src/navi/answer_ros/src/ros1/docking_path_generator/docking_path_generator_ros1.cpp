#include "ros1/docking_path_generator/docking_path_generator_ros1.hpp"

#include "ros/package.h"

namespace NVFR {

DockingPathGeneratorRos1::DockingPathGeneratorRos1(
    const std::string &node_name)
    : s_node_name_(node_name)
    , b_initialized_(false)
    , visualizer_(nh_, "DPG")
    , n_mission_type_(DockingInfo::CMD::NONE)
{
    // initialize
    Initialize();

}

DockingPathGeneratorRos1::~DockingPathGeneratorRos1()
{
    Terminator();
    o_mission_ptr_.reset();
    LOG_INFO("Destruct DockingPathGeneratorRos1");
    std::this_thread::sleep_for(std::chrono::milliseconds(20UL));
    logger_->TerminateLogger();
    logger_.reset();
}

void DockingPathGeneratorRos1::Terminator()
{
    for (auto &sub : subscriptions_) {
        sub.second->shutdown();
    }
    for (auto &pub : publishers_) {
        pub.second->shutdown();
    }
    for (auto &srv : srv_servers_) {
        srv.second->shutdown();
    }

    ros::shutdown();
}

void DockingPathGeneratorRos1::Initialize()
{
    // set directory of configs
    ANSWER::Path::GetInstance().SetAllPaths(
        ros::package::getPath("answer"));

    // load interface param
    ANSWER::Configurator::GetInstance().LoadParameters("ros");
    ANSWER::Configurator::GetInstance().LoadParameters("dpg_interface");

    // set logger
    auto log_level = ANSWER::Configurator::GetInstance()
        .GetParamValue("dpg_interface", "base", "log_level")
        .convert<std::string>();
    auto base_path = ANSWER::Configurator::GetInstance()
        .GetParamValue("ros", "base", "base_path")
        .convert<std::string>();
    logger_ = std::make_shared<ANSWER::Logger>(
        s_node_name_, log_level, base_path);

    // frame_id of visualizer
    visualizer_.SetFrameId("map", "base_link");

    // set param
    LoadParam();

    // regist pub/sub
    RegistPublisher();
    RegistSubscriber();
    RegistServiceServer();
    RegistServiceClient();

    // success initialization
    b_initialized_ = true;
    LOG_INFO("Success initialization");

}

void DockingPathGeneratorRos1::LoadParam()
{
    // load & read docking param
    DPG_Param_t st_param = DPGParamManager::LoadDPGParam();
    st_dpg_param_.Move(st_param);
}

void DockingPathGeneratorRos1::RegistPublisher()
{
    publishers_[DOCKING_PATH] =
        std::make_shared<ros::Publisher>(
            nh_.advertise<geometry_msgs::PoseArray>(
                DOCKING_PATH, 10));
    PCM::RegistCbFunc(PCM::KEY::DOCKING_PATH, &DockingPathGeneratorRos1::SendDockingPath, this);
    publishers_[RETURN_PATH] =
        std::make_shared<ros::Publisher>(
            nh_.advertise<geometry_msgs::PoseArray>(
                RETURN_PATH, 10));
    PCM::RegistCbFunc(PCM::KEY::RETURN_PATH, &DockingPathGeneratorRos1::SendReturnPath, this);

}

void DockingPathGeneratorRos1::RegistSubscriber()
{
    // robot state
    subscriptions_[ANSWER::TOPICS::ROBOT_POSE] =
        std::make_shared<ros::Subscriber>(
            nh_.subscribe(
                ANSWER::TOPICS::ROBOT_POSE, 10,
                &DockingPathGeneratorRos1::GetRobotPose, this));
    subscriptions_[ANSWER::TOPICS::ODOM] =
        std::make_shared<ros::Subscriber>(
            nh_.subscribe(
                ANSWER::TOPICS::ODOM, 10,
                &DockingPathGeneratorRos1::GetOdom, this));

    // perception
    subscriptions_[CMD_DOCKING] =
        std::make_shared<ros::Subscriber>(
            nh_.subscribe(
                CMD_DOCKING, 10,
                &DockingPathGeneratorRos1::GetCmdDocking, this));
    subscriptions_[OBJECT_POSE] =
        std::make_shared<ros::Subscriber>(
            nh_.subscribe(
                OBJECT_POSE, 10,
                &DockingPathGeneratorRos1::GetObjectPose, this));

    // others
    subscriptions_[NAME::TOPICS::PARAM_READ_ALL] =
        std::make_shared<ros::Subscriber>(
            nh_.subscribe(
                NAME::TOPICS::PARAM_READ_ALL, 10,
                &DockingPathGeneratorRos1::GetParamReadSignal, this));
    subscriptions_[NAME::TOPICS::PARAM_READ_NODE] =
        std::make_shared<ros::Subscriber>(
            nh_.subscribe(
                NAME::TOPICS::PARAM_READ_NODE + s_node_name_, 10,
                &DockingPathGeneratorRos1::GetParamReadSignal, this));
    subscriptions_[NAME::TOPICS::LOG_LV_ALL] =
        std::make_shared<ros::Subscriber>(
            nh_.subscribe(
                NAME::TOPICS::LOG_LV_ALL, 10,
                &DockingPathGeneratorRos1::GetLogLv, this));
    subscriptions_[NAME::TOPICS::LOG_LV_NODE] =
        std::make_shared<ros::Subscriber>(
            nh_.subscribe(
                NAME::TOPICS::LOG_LV_NODE + s_node_name_, 10,
                &DockingPathGeneratorRos1::GetLogLv, this));
}

void DockingPathGeneratorRos1::RegistServiceServer()
{
    // srv_servers_[MISSION_DONE] =
    //     std::make_shared<ros::ServiceServer>(
    //         nh_.advertiseService(
    //             MISSION_DONE,
    //             &DockingPathGeneratorRos1::ResponseMissionDone, this));

}

void DockingPathGeneratorRos1::RegistServiceClient()
{
    // srv_clients_[CLIENT_NAME] =
    //     std::make_shared<ros::ServiceClient>(
    //         nh_.serviceClient<
    //             answer_msgs::CommonService>(
    //                 CLIENT));

}

void DockingPathGeneratorRos1::SendDockingPath(const Path& path)
{
    int n_path_size = path.size();
    LOG_INFO("[SendDockingPath] path size is {}", path.size());

    geometry_msgs::PoseArray msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    for (const auto& pose : path)
    {
        auto quat = CM::Yaw2Quaternion(pose.GetRad());
        geometry_msgs::Pose wp;
        wp.position.x = pose.GetXm();
        wp.position.y = pose.GetYm();
        wp.position.z = 0.0;
        wp.orientation.x = quat[0];
        wp.orientation.y = quat[1];
        wp.orientation.z = quat[2];
        wp.orientation.w = quat[3];
        msg.poses.emplace_back(wp);
    }

    publishers_[DOCKING_PATH]->publish(msg);

}

void DockingPathGeneratorRos1::SendReturnPath(const Path& path)
{
    int n_path_size = path.size();
    LOG_INFO("[SendReturnPath] path size is {}", path.size());

    geometry_msgs::PoseArray msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    for (const auto& pose : path)
    {
        auto quat = CM::Yaw2Quaternion(pose.GetRad());
        geometry_msgs::Pose wp;
        wp.position.x = pose.GetXm();
        wp.position.y = pose.GetYm();
        wp.position.z = 0.0;
        wp.orientation.x = quat[0];
        wp.orientation.y = quat[1];
        wp.orientation.z = quat[2];
        wp.orientation.w = quat[3];
        msg.poses.emplace_back(wp);
    }

    publishers_[RETURN_PATH]->publish(msg);

}

}  // namespace NVFR
