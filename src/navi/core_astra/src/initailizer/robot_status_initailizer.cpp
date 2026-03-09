
#include "core_astra/initializer/robot_status_initailizer.h"

#include "common.pb.h"
#include "core_agent/data/memory_repository.h"
#include "core_agent/data/robot_collision_info.h"
#include "core_agent/data/robot_info.h"
#include "core_agent/data/robot_pose.h"
#include "core_agent/data/robot_status.h"
#include "core_agent/manager/publish_manager.h"
#include "core_agent/util/config.h"
#include "core_astra/data/publish_channel.h"
#include "core_astra/zmq_handler.h"
#include "core_msgs/NavicoreStatus.h"
#include "robot_status.pb.h"
#include "util/logger.hpp"

#include <tf/transform_datatypes.h>
#include <zmq.hpp>

using namespace NaviFra;

RobotStatusInitializer::~RobotStatusInitializer()
{
    // 구독자 해제
    param_update_subscriber_.shutdown();
    task_info_subscriber_.shutdown();
    odometry_subscriber_.shutdown();
    navicore_status_subscriber_.shutdown();
    error_dist_subscriber_.shutdown();
}

std::vector<std::string> RobotStatusInitializer::splitString(const std::string& str, char delimiter)
{
    std::vector<std::string> result;
    std::stringstream ss(str);
    std::string token;

    while (std::getline(ss, token, delimiter)) {
        result.push_back(token);  // 빈 문자열도 허용
    }

    // 문자열이 구분자로 끝날 경우 마지막 빈 토큰 추가
    if (!str.empty() && str.back() == delimiter) {
        result.push_back("");
    }

    // 특수 케이스: 입력이 구분자만 하나일 때 (예: "/")
    if (str == std::string(1, delimiter)) {
        result = {"", ""};
    }

    return result;
}

void RobotStatusInitializer::onErrorDist(const std_msgs::Float64::ConstPtr msg)
{
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    robotInfo->setErrordist(msg->data);
}

void RobotStatusInitializer::onBmsInfo(const core_msgs::BmsInfo::ConstPtr msg)
{
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    robotInfo->setBattery(msg->f32_soc);
    robotInfo->setBatteryVol(msg->f32_pack_volt);
}

void RobotStatusInitializer::onPlcInfo(const core_msgs::PLCInfo::ConstPtr msg)
{
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    robotInfo->setIsManual(msg->io_mode_select_2);
}

void RobotStatusInitializer::onNavifrainfo(const core_msgs::NavicoreStatus::ConstPtr msg)
{
    try {
        static std::string current_node_id = "";

        auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);

        if (msg->s_current_node_id.size() > 1) {
            current_node_id = msg->s_current_node_id;
        }
        robotInfo->updateRobotInfo(
            {msg->f_robot_current_linear_vel_x, msg->f_robot_current_linear_vel_y, msg->f_robot_current_angular_vel_w},
            {current_node_id, msg->s_next_node_id, msg->s_goal_node_id, msg->f_path_progress}, msg->n_confidence);

        robotInfo->updateFootprint(msg->f_robot_pos_x_m, msg->f_robot_pos_y_m);
        robotInfo->updateErrorPose(msg->f_error_pos_x_m, msg->f_error_pos_y_m, msg->f_error_pos_deg);
        robotInfo->setStatus(msg->s_status);
        robotInfo->setErrordist(msg->f_path_error_dist_m);
        robotInfo->setWorkID(msg->s_goal_node);

        float robotOrientationDeg = msg->f_robot_pos_deg;
        tf::Quaternion q;
        q.setRPY(0, 0, robotOrientationDeg * 3.141592 / 180);
        std::vector<float> robotOrientationQuat = {0, 0, 0, 0};

        Position position;
        Orientation orientation;

        position.x = isnan(msg->f_robot_pos_x_m) ? 0 : msg->f_robot_pos_x_m;
        position.y = isnan(msg->f_robot_pos_y_m) ? 0 : msg->f_robot_pos_y_m;
        orientation.x = isnan(q.getX()) ? 0 : q.getX();
        orientation.y = isnan(q.getY()) ? 0 : q.getY();
        orientation.z = isnan(q.getZ()) ? 0 : q.getZ();
        orientation.w = isnan(q.getW()) ? 0 : q.getW();

        InMemoryRepository::instance().get<RobotPose>(RobotPose::KEY)->update(position, orientation);
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
    }
}

void RobotStatusInitializer::onOdometry(const nav_msgs::Odometry::ConstPtr msg)
{
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    robotInfo->updateVelocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.angular.z);
}

void RobotStatusInitializer::onNowTask(const std_msgs::String::ConstPtr msg)
{
    try {
        auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
        robotInfo->setNowTask(msg->data);
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
    }
}

void RobotStatusInitializer::onTaskInfo(const std_msgs::String::ConstPtr msg)
{
    static std::string s_previous_task_id = "";
    try {
        auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
        std::vector<std::string> vec_uuid = splitString(msg->data, '/');
        if (vec_uuid.size() != 2) {
            NLOG(error) << "Invalid task info format: " << msg->data;
            return;
        }
        if (vec_uuid[1] == "") {
            vec_uuid[1] = s_previous_task_id;
        }
        robotInfo->updateJob(vec_uuid[0], vec_uuid[1]);
        s_previous_task_id = vec_uuid[1];
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
    }
}

void RobotStatusInitializer::onParamUpdate(const std_msgs::String::ConstPtr msg)
{
    NLOG(info) << "Received ROS Param update";
    // // TODO update

    std::vector<float> robot_outline;
    std::vector<float> robot_collision;
    bool b_use_detection_mode_flag = false;
    ros::param::get("obstacle/outline", robot_outline);
    ros::param::get("obstacle/collision", robot_collision);
    ros::param::get("obstacle/b_use_detection_mode_flag", b_use_detection_mode_flag);

    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    auto collision = InMemoryRepository::instance().get<RobotCollisionInfo>(RobotCollisionInfo::KEY);

    float collision_offestX = 0.f;
    float collision_offestY = 0.f;
    if (robot_outline.size() == 4) {
        LOG_INFO("Read Robot Outline successfully.");
        collision->initShape(robot_outline[0], robot_outline[1], robot_outline[3], robot_outline[2], collision_offestX, collision_offestY);
    }
    if (robot_collision.size() == 4 && b_use_detection_mode_flag) {
        LOG_INFO("Read Robot collision successfully.");
        collision->initCollision(
            robot_collision[0], robot_collision[1], robot_collision[3], robot_collision[2], collision_offestX, collision_offestY);
    }
    else {
        LOG_INFO("Read Robot collision not read.");
        collision->initCollision(0, 0, 0, 0, collision_offestX, collision_offestY);
    }

    int n_kinematics = 0;
    float f_base = 0;
    ros::param::get("motion_base/n_kinematics_type", n_kinematics);
    if (n_kinematics == 0)  // dd
    {
        ros::param::get("driver_wheel/f_FL_wheel_y_m", f_base);
        f_base *= 2;
    }
    else if (n_kinematics == 1)  // qd
    {
        ros::param::get("driver_wheel/f_FL_wheel_x_m", f_base);
    }
    else if (n_kinematics == 2)  // sd
    {
        ros::param::get("driver_wheel/f_FL_wheel_x_m", f_base);
    }

    robotInfo->setRobotBase(f_base);
}

void RobotStatusInitializer::initialize()
{
    ros::NodeHandle nh;

    param_update_subscriber_ = nh.subscribe("navifra/param_update", 2, &RobotStatusInitializer::onParamUpdate, this);
    task_info_subscriber_ = nh.subscribe("navifra/sound", 20, &RobotStatusInitializer::onTaskInfo, this);
    odometry_subscriber_ = nh.subscribe("odom", 1, &RobotStatusInitializer::onOdometry, this);
    bms_subscriber_ = nh.subscribe("bms_info", 1, &RobotStatusInitializer::onBmsInfo, this);
    navicore_status_subscriber_ = nh.subscribe("navifra/info", 1, &RobotStatusInitializer::onNavifrainfo, this);
    error_dist_subscriber_ = nh.subscribe("NaviFra/error_dist", 1, &RobotStatusInitializer::onErrorDist, this);
    plc_info_subscriber_ = nh.subscribe("plc_info", 1, &RobotStatusInitializer::onPlcInfo, this);
    task_info_subscriber_ = nh.subscribe("navifra/sound", 1, &RobotStatusInitializer::onNowTask, this);

    PublisherManager::instance().addChannel(
        (int)PUBLISH_CHANNEL::CHANNEL_HEARBEAT,
        []() mutable {
            core_astra::Heartbeat hb;
            hb.set_id(Config::instance().getString("robot_id", "unknown"));
            hb.set_timestamp(std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1));  // ms 단위 timestamp
            hb.set_is_slam(false);
            hb.set_version("1.0.0");

            std::string payload = hb.SerializeAsString();

            ZMQHandler::instance().send("robot_heartbeat", payload);
        },
        500,  // 주기(ms)
        0);

    PublisherManager::instance().addChannel(
        (int)PUBLISH_CHANNEL::CHANNEL_INFO_STATUS,
        []() mutable {
            auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
            auto robotPose = InMemoryRepository::instance().get<RobotPose>(RobotPose::KEY);

            core_astra::RobotStatus status;

            // 기본 문자열 / 숫자
            status.set_id(Config::instance().getString("robot_id", "unknown"));
            status.set_robot_status(robotInfo->getNowTask());
            status.set_robot_alarm(robotInfo->getAlarmString());
            status.set_battery(robotInfo->getBattery());
            status.set_battery_voltage(robotInfo->getBatteryVol());
            status.set_error_dist(robotInfo->getErrordist());

            // ErrorPose
            auto ep = status.mutable_errors();
            ep->mutable_position()->set_x(0.02);
            ep->mutable_position()->set_y(-0.01);
            ep->mutable_position()->set_deg(1.5);

            // 로봇 위치
            auto position = status.mutable_robot_pose()->mutable_position();
            position->set_x(robotPose->getPosition().x);
            position->set_y(robotPose->getPosition().y);
            position->set_z(0.0);  // z축은 사용하지 않음
            auto orientation = status.mutable_robot_pose()->mutable_orientation();
            orientation->set_x(robotPose->getOrientation().x);
            orientation->set_y(robotPose->getOrientation().y);
            orientation->set_z(robotPose->getOrientation().z);
            orientation->set_w(robotPose->getOrientation().w);

            // 로봇 속도
            auto velocity = status.mutable_robot_linear_velocity();
            auto robotVelocity = robotInfo->getVelocity();

            velocity->Add(robotVelocity.at(0));  // x
            velocity->Add(robotVelocity.at(1));  // y

            status.set_robot_angular_velocity(robotVelocity.at(2));  // z

            // now, next, goal 노드
            status.set_now_node(robotInfo->getNowNodeString());
            status.set_next_node(robotInfo->getNextNodeString());
            status.set_goal_node(robotInfo->getGoalNodeString());
            status.set_now_task(robotInfo->getNowTask());
            status.set_work_id(robotInfo->getWorkID());

            // NLOG(info) << " Now Node : " << robotInfo->getNowNodeString() << " Next Node : " << robotInfo->getNextNodeString() << " Goal Node : " << robotInfo->getGoalNodeString();
            // NLOG(info) << " Now Node : " << status.now_node() << " Next Node : " << status.next_node() << " Goal Node : " << status.goal_node();

            // path 정보
            status.set_path_progress(robotInfo->getPathProgress());

            // map 일치율
            status.set_confidence(robotInfo->getConfidence());

            status.set_is_load(robotInfo->getIsLoad());

            // 로봇 사이즈
            auto size = status.mutable_robot_size();
            size->set_width(robotInfo->getSize().width);
            size->set_height(robotInfo->getSize().height);

            status.set_robot_base(robotInfo->getRobotBase());
            status.set_robot_wheeloffset(robotInfo->getRobotWheelOffset());

            auto collision = InMemoryRepository::instance().get<RobotCollisionInfo>(RobotCollisionInfo::KEY);
            auto mutable_collision = status.mutable_robot_collision_info();

            // shape 채우기
            for (auto& p : collision->getShapeVector()) {
                auto pos = mutable_collision->add_shape();
                pos->set_x(p.x);
                pos->set_y(p.y);
                pos->set_z(p.z);  // shape에도 z값 들어가면 그대로 반영
            }

            for (auto& p : collision->getCollisionVector()) {
                auto pos = mutable_collision->add_collision();
                pos->set_x(p.x);
                pos->set_y(p.y);
                pos->set_z(0);  // 현재 코드 기준으로 Z는 0
            }

            auto wheelInfoStore = InMemoryRepository::instance().get<RobotWheelInfoStore>(RobotWheelInfoStore::KEY);
            auto mutable_wheels = status.mutable_robot_wheels();  // robot_status.proto의 repeated RobotWheel

            for (const auto& pair : wheelInfoStore->getWheels()) {
                const auto& wheel = pair.second;

                auto wheelMsg = mutable_wheels->Add();
                wheelMsg->set_id(wheel.name());
                wheelMsg->set_diameter(wheel.getDiameter());
                wheelMsg->set_degree(wheel.getDegree());

                auto pos = wheelMsg->mutable_position();
                pos->set_x(wheel.getPosition().x);
                pos->set_y(wheel.getPosition().y);
                pos->set_z(0);  // 기존 코드에서 z는 항상 0
            }

            auto setting = status.mutable_setting();
            setting->set_map(robotInfo->getSetting().map);
            setting->set_total_nodes(robotInfo->getSetting().totalNodes);
            setting->set_teached_nodes(robotInfo->getSetting().teachedNodes);
            status.set_isrobotmanual(robotInfo->getIsManual());

            // JigInfo
            // auto jig = status->mutable_jiginfo();
            // jig->set_rotation(0);
            // jig->set_lift(1);
            // jig->set_clamp(true);
            // jig->set_hold(false);
            // jig->set_product(true);
            // jig->set_error(false);
            // jig->set_communication(true);

            //// Driving Info
            // auto drive = status.mutable_driving_info();
            // drive->set_max_linear_vel_of_path(1.0);
            // drive->set_liftcmd(0);
            // drive->set_piocmd(1);
            // drive->set_reset_cmd(false);
            // drive->set_batterystartcmd(0);
            //
            // status.set_isrobotmanual(false);
            // status.set_barcode_reader(true);
            //
            //// Error Table
            // status.add_errortable("NoError");
            // status.add_errortable("TestWarn");
            //
            // status.set_robottype("TestAGV");
            //
            //// 직렬화 및 송신
            std::string buffer;
            status.SerializeToString(&buffer);

            ZMQHandler::instance().send("robot_status", buffer);
        },
        100, 0);

    LOG_INFO("RobotStatusInitializer completed initialization");
}

void RobotStatusInitializer::finalize()
{
    param_update_subscriber_.shutdown();
    task_info_subscriber_.shutdown();
    odometry_subscriber_.shutdown();
    navicore_status_subscriber_.shutdown();
    error_dist_subscriber_.shutdown();
}
