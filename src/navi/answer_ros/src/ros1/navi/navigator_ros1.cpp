#include "ros1/navi/navigator_ros1.hpp"

#include "ros/package.h"

namespace NVFR {

NavigatorRos1::NavigatorRos1(
    const std::string &node_name)
    : s_node_name_(node_name)
    , b_initialized_(false)
    , visualizer_(nh_, "NaviFra")
{
    // initialize
    if (!Initialize())
        return;

    // sub
    if (!RegistPrimarySub())
        return;
    if (!RegistSecondarySub())
        return;
    if (!RegistOtherSub())
        return;

    b_initialized_ = true;
}

NavigatorRos1::~NavigatorRos1()
{
    Terminator();
    o_navigator_ptr_->Terminator();
    o_navigator_ptr_.reset();
    LOG_INFO("Destruct NavigatorRos1");
    std::this_thread::sleep_for(std::chrono::milliseconds(20UL));
    logger_->TerminateLogger();
    logger_.reset();
}

void NavigatorRos1::Terminator()
{
    for (auto &sub : subscriptions_) {
        sub.second->shutdown();
    }
    for (auto &pub : publishers_) {
        pub.second->shutdown();
    }
    for (auto &client : srv_clients_) {
        client.second->shutdown();
    }

    ros::shutdown();
}

bool NavigatorRos1::Initialize()
{
    // set directory of configs
    ANSWER::Path::GetInstance().SetAllPaths(
        ros::package::getPath("answer"));

    // load interface param
    ANSWER::Configurator::GetInstance().LoadParameters("ros");
    ANSWER::Configurator::GetInstance().LoadParameters("navi_interface");

    // set logger
    auto log_level = ANSWER::Configurator::GetInstance()
        .GetParamValue("navi_interface", "base", "log_level")
        .convert<std::string>();
    auto base_path = ANSWER::Configurator::GetInstance()
        .GetParamValue("ros", "base", "base_path")
        .convert<std::string>();
    logger_ = std::make_shared<ANSWER::Logger>(
        s_node_name_, log_level, base_path);

    // frame_id of visualizer
    std::string s_fixed =
        ANSWER::Configurator::GetInstance().GetParam<std::string>(
            "navi_interface", "topic_name", ANSWER::TOPICS::MAP,
            ANSWER::TOPICS::MAP);
    std::string s_local =
        ANSWER::Configurator::GetInstance().GetParam<std::string>(
            "navi_interface", "tf_name", NAME::TFS::BASE_LINK,
            NAME::TFS::BASE_LINK);
    visualizer_.SetFrameId(s_fixed, s_local);

    // navigator
    int n_kinematics =
        ANSWER::Configurator::GetInstance()
            .GetParamValue("navi_interface", "navi_base", "kinematics")
            .convert<int>();
    int n_controller =
        ANSWER::Configurator::GetInstance()
            .GetParamValue("navi_interface", "navi_base", "controller")
            .convert<int>();
    int n_avoidance =
        ANSWER::Configurator::GetInstance()
            .GetParamValue("navi_interface", "navi_base", "avoidance")
            .convert<int>();
    if (MPTL::IsValid<MPTL::KINEMATICS>(n_kinematics) == false) {
        LOG_ERROR("KINEMATICS is wrong ({})", n_kinematics);
        return false;
    }
    if (MPTL::IsValid<MPTL::CONTROLLER>(n_controller) == false) {
        LOG_ERROR("CONTROLLER is wrong ({})", n_controller);
        return false;
    }
    if (MPTL::IsValid<MPTL::AVOID>(n_avoidance) == false) {
        LOG_ERROR("AVOID is wrong ({})", n_avoidance);
        return false;
    }
    o_navigator_ptr_ = std::make_unique<Navigator>(
        static_cast<MPTL::KINEMATICS>(n_kinematics),
        static_cast<MPTL::CONTROLLER>(n_controller),
        static_cast<MPTL::AVOID>(n_avoidance));

    // pub
    std::string s_msg_type = "";
    std::string s_topic_name = "";

    s_msg_type = ANSWER::Configurator::GetInstance().GetParam<std::string>(
        "navi_interface", "topic_type", NAME::TOPICS::CMD_VEL,
        NAME::MSG_TYPES::GEO_TWIST);
    s_topic_name = ANSWER::Configurator::GetInstance().GetParam<std::string>(
        "navi_interface", "topic_name", NAME::TOPICS::CMD_VEL,
        NAME::TOPICS::CMD_VEL);
    LOG_INFO("{} type: {}", s_topic_name.c_str(), s_msg_type.c_str());
    if (s_msg_type == NAME::MSG_TYPES::GEO_TWIST) {
        publishers_[KEY::TOPICS::CMD_VEL] = std::make_shared<ros::Publisher>(
            nh_.advertise<geometry_msgs::Twist>(s_topic_name, 10));
        o_navigator_ptr_->RegistCbFunc(
            KEY::MC, &NavigatorRos1::SendCmdVelT, this);
    }
    else if (s_msg_type == NAME::MSG_TYPES::GEO_TWIST_STMP) {
        publishers_[KEY::TOPICS::CMD_VEL] = std::make_shared<ros::Publisher>(
            nh_.advertise<geometry_msgs::TwistStamped>(s_topic_name, 10));
        o_navigator_ptr_->RegistCbFunc(
            KEY::MC, &NavigatorRos1::SendCmdVelTS, this);
    }
    else {
        LOG_ERROR(
            "Msg type of {} is wrong: {}", s_topic_name.c_str(),
            s_msg_type.c_str());
        return false;
    }

    publishers_[KEY::TOPICS::NAVI_MISSION_RESPONSE] =
        std::make_shared<ros::Publisher>(
            nh_.advertise<answer_msgs::MissionResponse>(
                NAME::TOPICS::NAVI_MISSION_RESPONSE, 10));
    publishers_[KEY::TOPICS::NAVI_STATUS] = std::make_shared<ros::Publisher>(
        nh_.advertise<std_msgs::Int32>(NAME::TOPICS::NAVI_STATUS, 10));
    o_navigator_ptr_->RegistCbFunc(
        KEY::NS, &NavigatorRos1::SendNaviStatus, this);
    publishers_[KEY::TOPICS::NAVI_ERROR] = std::make_shared<ros::Publisher>(
        nh_.advertise<std_msgs::Int32>(NAME::TOPICS::NAVI_ERROR, 10));
    o_navigator_ptr_->RegistCbFunc(
        KEY::NE, &NavigatorRos1::SendNaviError, this);
    publishers_[KEY::TOPICS::NAVI_INFO] = std::make_shared<ros::Publisher>(
        nh_.advertise<answer_msgs::NaviInfo>(NAME::TOPICS::NAVI_INFO, 10));
    o_navigator_ptr_->RegistCbFunc(KEY::NI, &NavigatorRos1::SendNaviInfo, this);

    // set param
    LoadParam();

    return true;
}

void NavigatorRos1::LoadParam()
{
    // load & read navigator param
    NavigatorParam_t st_param = NaviParamManager::LoadNaviParam();
    o_navigator_ptr_->WriteParam(st_param);
}

bool NavigatorRos1::RegistPrimarySub()
{
    std::string s_topic_name = "";

    s_topic_name = ANSWER::Configurator::GetInstance().GetParam<std::string>(
        "navi_interface", "topic_name", ANSWER::TOPICS::ROBOT_POSE,
        ANSWER::TOPICS::ROBOT_POSE);
    subscriptions_[KEY::TOPICS::ROBOT_POSE] = std::make_shared<ros::Subscriber>(
        nh_.subscribe(s_topic_name, 10, &NavigatorRos1::GetRobotPose, this));

    s_topic_name = ANSWER::Configurator::GetInstance().GetParam<std::string>(
        "navi_interface", "topic_name", ANSWER::TOPICS::ODOM,
        ANSWER::TOPICS::ODOM);
    subscriptions_[KEY::TOPICS::ODOM] = std::make_shared<ros::Subscriber>(
        nh_.subscribe(s_topic_name, 10, &NavigatorRos1::GetOdom, this));

    // wait standard subscriptions
    if (!WaitPrimaryDataSet()) {
        return false;
    }

    s_topic_name = ANSWER::Configurator::GetInstance().GetParam<std::string>(
        "navi_interface", "topic_name", ANSWER::TOPICS::MAP,
        ANSWER::TOPICS::MAP);
    subscriptions_[KEY::TOPICS::MAP] = std::make_shared<ros::Subscriber>(
        nh_.subscribe(s_topic_name, 10, &NavigatorRos1::GetGlobalMap, this));

    return true;
}

bool NavigatorRos1::RegistSecondarySub()
{
    std::string s_msg_type = "";
    std::string s_topic_name = "";

    s_msg_type = ANSWER::Configurator::GetInstance().GetParam<std::string>(
        "navi_interface", "topic_type", NAME::TOPICS::LIDAR,
        NAME::MSG_TYPES::SEN_PCD);
    s_topic_name = ANSWER::Configurator::GetInstance().GetParam<std::string>(
        "navi_interface", "topic_name", NAME::TOPICS::LIDAR,
        NAME::TOPICS::LIDAR);
    LOG_INFO("{} type: {}", s_topic_name.c_str(), s_msg_type.c_str());
    if (s_msg_type == NAME::MSG_TYPES::SEN_PCD) {
        subscriptions_[KEY::TOPICS::LIDAR] =
            std::make_shared<ros::Subscriber>(nh_.subscribe(
                s_topic_name, 10, &NavigatorRos1::GetPointCloud2D, this));
    }
    else if (s_msg_type == NAME::MSG_TYPES::SEN_LASERSCAN) {
        subscriptions_[KEY::TOPICS::LIDAR] =
            std::make_shared<ros::Subscriber>(nh_.subscribe(
                s_topic_name, 10, &NavigatorRos1::GetLidarScan, this));
    }
    else {
        LOG_ERROR(
            "Msg type of {} is wrong: {}", s_topic_name.c_str(),
            s_msg_type.c_str());
        return false;
    }

    // wait others (subscriptions)
    if (!WaitSecondaryDataSet()) {
        return false;
    }

    return true;
}

bool NavigatorRos1::RegistOtherSub()
{
    subscriptions_[KEY::TOPICS::NAVI_MISSION_ALIGN] =
        std::make_shared<ros::Subscriber>(nh_.subscribe(
            NAME::TOPICS::NAVI_MISSION_ALIGN, 10,
            &NavigatorRos1::GetMissionAlign, this));
    subscriptions_[KEY::TOPICS::NAVI_MISSION_NODE_PATH] =
        std::make_shared<ros::Subscriber>(nh_.subscribe(
            NAME::TOPICS::NAVI_MISSION_NODE_PATH, 10, &NavigatorRos1::GetMissionNodePath,
            this));
    subscriptions_[KEY::TOPICS::NAVI_MISSION_EXPLORE] =
        std::make_shared<ros::Subscriber>(nh_.subscribe(
            NAME::TOPICS::NAVI_MISSION_EXPLORE, 10,
            &NavigatorRos1::GetMissionExplore, this));
    subscriptions_[KEY::TOPICS::NAVI_CMD_SIGNAL] =
        std::make_shared<ros::Subscriber>(nh_.subscribe(
            NAME::TOPICS::NAVI_CMD_SIGNAL, 10, &NavigatorRos1::GetCmdSignal,
            this));
    subscriptions_[KEY::TOPICS::PARAM_READ_ALL] =
        std::make_shared<ros::Subscriber>(nh_.subscribe(
            NAME::TOPICS::PARAM_READ_ALL, 10,
            &NavigatorRos1::GetParamReadSignal, this));
    subscriptions_[KEY::TOPICS::PARAM_READ_NODE] =
        std::make_shared<ros::Subscriber>(nh_.subscribe(
            NAME::TOPICS::PARAM_READ_NODE + s_node_name_, 10,
            &NavigatorRos1::GetParamReadSignal, this));
    subscriptions_[KEY::TOPICS::LOG_LV_ALL] =
        std::make_shared<ros::Subscriber>(nh_.subscribe(
            NAME::TOPICS::LOG_LV_ALL, 10, &NavigatorRos1::GetLogLv, this));
    subscriptions_[KEY::TOPICS::LOG_LV_NODE] =
        std::make_shared<ros::Subscriber>(nh_.subscribe(
            NAME::TOPICS::LOG_LV_NODE + s_node_name_, 10,
            &NavigatorRos1::GetLogLv, this));

    return true;
}

bool NavigatorRos1::WaitPrimaryDataSet()
{
    // robot pose
    while (ros::ok()) {
        ros::spinOnce();
        if (o_navigator_ptr_->CheckDataSet() > 0) {
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

    // global map
    srv_clients_[KEY::SERVICES::GLOBAL_MAP] =
        std::make_shared<ros::ServiceClient>(
            nh_.serviceClient<answer_msgs::GlobalMap>(
                ANSWER::SERVICES::GLOBAL_MAP));
    answer_msgs::GlobalMap srv;
    srv.request.header.frame_id = s_node_name_;
    srv.request.header.stamp = ros::Time::now();
    while (ros::ok()) {
        if (srv_clients_[KEY::SERVICES::GLOBAL_MAP]->call(srv)) {
            GetGlobalMap(srv.response.map);
            LOG_INFO("Get Global Map");
            break;
        }
        else {
            LOG_WARN("Failed to call service, waiting again...");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000UL));
        }
    }
    if (!ros::ok()) {
        LOG_WARN(
            "Interrupted while waiting for the service: /{}",
            ANSWER::SERVICES::GLOBAL_MAP.c_str());
        return false;
    }

    LOG_INFO("Ok (primary data set)");
    return true;
}

bool NavigatorRos1::WaitSecondaryDataSet()
{
    std::string s_topic_name =
        ANSWER::Configurator::GetInstance().GetParam<std::string>(
            "navi_interface", "topic_name", NAME::TOPICS::LIDAR,
            NAME::TOPICS::LIDAR);

    while (ros::ok()) {
        ros::spinOnce();
        if (o_navigator_ptr_->CheckDataSet() > 2) {
            LOG_INFO("Get Local Map");
            break;
        }
        LOG_TRACE("waiting for the topic: /{} ...", s_topic_name.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(20UL));
    }
    if (!ros::ok()) {
        LOG_WARN(
            "Interrupted while waiting for the topic: /{}",
            s_topic_name.c_str());
        return false;
    }

    LOG_INFO("Ok (secondary data set)");
    return true;
}

void NavigatorRos1::SendCmdVelT(
    const Pose &o_control_input, const DriveInfo &o_drive_info)
{
    geometry_msgs::Twist msg;
    msg.linear.x = o_control_input.GetVX();
    msg.linear.y = o_control_input.GetVY();
    msg.angular.z = o_control_input.GetVW();
    // [henry] test
    msg.angular.x = o_control_input.GetXm();  // target spd
    msg.angular.y = o_control_input.GetYm();  // lateral errer
    msg.linear.z = o_control_input.GetZm();  // angle error
    publishers_[KEY::TOPICS::CMD_VEL]->publish(msg);
}

void NavigatorRos1::SendCmdVelTS(
    const Pose &o_control_input, const DriveInfo &o_drive_info)
{
    geometry_msgs::TwistStamped msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp = ros::Time::now();
    msg.twist.linear.x = o_control_input.GetVX();
    msg.twist.linear.y = o_control_input.GetVY();
    msg.twist.angular.z = o_control_input.GetVW();
    // [henry] test
    msg.twist.angular.x = o_control_input.GetXm();  // target spd
    msg.twist.angular.y = o_control_input.GetYm();  // lateral errer
    msg.twist.linear.z = o_control_input.GetZm();  // angle error
    publishers_[KEY::TOPICS::CMD_VEL]->publish(msg);
}

void NavigatorRos1::SendMissionResponse(
    const std::string &s_uuid, const bool &b_received)
{
    answer_msgs::MissionResponse msg;
    msg.uuid = s_uuid;
    msg.b_received = b_received;
    publishers_[KEY::TOPICS::NAVI_MISSION_RESPONSE]->publish(msg);
}

void NavigatorRos1::SendNaviStatus(const int &n_status)
{
    std_msgs::Int32 msg;
    msg.data = n_status;
    publishers_[KEY::TOPICS::NAVI_STATUS]->publish(msg);
}

void NavigatorRos1::SendNaviError(const int &n_error)
{
    std_msgs::Int32 msg;
    msg.data = n_error;
    publishers_[KEY::TOPICS::NAVI_ERROR]->publish(msg);
}

void NavigatorRos1::SendNaviInfo(const NaviInfo &o_navi_info)
{
    answer_msgs::NaviInfo msg =
        MsgConverter::Convert<NaviInfo, answer_msgs::NaviInfo>(o_navi_info);
    publishers_[KEY::TOPICS::NAVI_INFO]->publish(msg);
}

}  // namespace NVFR
