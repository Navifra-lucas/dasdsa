#include "core_astra/initializer/event_initializer.h"

#include "core_agent/data/memory_repository.h"
#include "core_agent/data/motor_info.h"
#include "core_agent/data/robot_wheel_info.h"
#include "core_astra/zmq_handler.h"
#include "event.pb.h"  // EventMessage, EventType
#include "robot_pose.pb.h"  // Position, Orientation, Pose 재사용
#include "util/logger.hpp"

#include <Poco/DigestEngine.h>
#include <Poco/UUID.h>
#include <Poco/UUIDGenerator.h>
#include <core_msgs/TaskAlarm.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

using namespace NaviFra;
using namespace core_astra;

void EventInitializer::initialize()
{
    ros::NodeHandle nh;

    // Subscriber 등록
    sub_obstacle_ = nh.subscribe("NaviFra/visualize/ui_obstacle_pos", 2, &EventInitializer::onObstacle, this);
    sub_local_path_ = nh.subscribe("NaviFra/visualize/ui_local_path", 1, &EventInitializer::onLocalPath, this);
    sub_global_path_ = nh.subscribe("NaviFra/visualize/ui_global_path", 1, &EventInitializer::onGlobalPath, this);
    sub_predict_collision_ = nh.subscribe("NaviFra/visualize/ui_predict_collision", 1, &EventInitializer::onPredictCollision, this);
    sub_reflectors_ = nh.subscribe("NaviFra/reflectors", 1, &EventInitializer::onReflectors, this);
    sub_task_alarm_ = nh.subscribe("nc_task_manager/task_alarm", 20, &EventInitializer::onTaskAlarm, this);
    sub_task_response_ = nh.subscribe("nc_task_manager/task_response", 20, &EventInitializer::onTaskResponse, this);
    sub_navi_alarm_ = nh.subscribe("navifra/alarm", 20, &EventInitializer::onNaviAlarm, this);
    sub_mapping_progress_ = nh.subscribe("answer/mapping_progress", 1, &EventInitializer::onMappingProgress, this);
    sub_motor_info_ = nh.subscribe("motor_info", 1, &EventInitializer::onMotorInfo, this);
    sub_slam_graph_ = nh.subscribe("NaviFra/slam_graph", 1, &EventInitializer::onSLAMGraph, this);
    sub_cali_progress_ = nh.subscribe("NaviFra/cali_progress", 1, &EventInitializer::onCaliProgress, this);
    sub_hardware_info_ = nh.subscribe("NaviFra/hardware_info", 1, &EventInitializer::onHardwareInfo, this);
    sub_custom_ = nh.subscribe("etc_custom_message", 1, &EventInitializer::onCustom, this);

    LOG_INFO("EventInitializer completed initialization");
}

void EventInitializer::finalize()
{
    sub_obstacle_.shutdown();
    sub_local_path_.shutdown();
    sub_global_path_.shutdown();
    sub_predict_collision_.shutdown();
    sub_reflectors_.shutdown();
    sub_task_alarm_.shutdown();
    sub_task_response_.shutdown();
    sub_navi_alarm_.shutdown();
    sub_mapping_progress_.shutdown();
    sub_motor_info_.shutdown();
    sub_slam_graph_.shutdown();
    sub_cali_progress_.shutdown();
    sub_hardware_info_.shutdown();
    sub_custom_.shutdown();
}

void EventInitializer::onObstacle(const geometry_msgs::PolygonStamped::ConstPtr& msg)
{
    EventMessage event;
    event.set_type(EventType::EVENT_OBSTACLE);

    auto* payload = event.mutable_obstacle();
    payload->set_polygon_name("obstacle_pos");

    for (auto& point : msg->polygon.points) {
        auto* pos = payload->add_points();
        pos->set_x(point.x);
        pos->set_y(point.y);
        pos->set_z(0);
    }

    std::string buffer;
    event.SerializeToString(&buffer);
    ZMQHandler::instance().send("event", buffer);
}

void EventInitializer::onLocalPath(const nav_msgs::Path::ConstPtr& msg)
{
    EventMessage event;
    event.set_type(EventType::EVENT_LOCAL_PATH);

    auto* payload = event.mutable_local_path();
    for (auto& pose : msg->poses) {
        auto* p = payload->add_poses();
        p->mutable_position()->set_x(pose.pose.position.x);
        p->mutable_position()->set_y(pose.pose.position.y);
        p->mutable_position()->set_z(pose.pose.position.z);
        p->mutable_orientation()->set_x(pose.pose.orientation.x);
        p->mutable_orientation()->set_y(pose.pose.orientation.y);
        p->mutable_orientation()->set_z(pose.pose.orientation.z);
        p->mutable_orientation()->set_w(pose.pose.orientation.w);
    }

    std::string buffer;
    event.SerializeToString(&buffer);
    ZMQHandler::instance().send("event", buffer);
}

void EventInitializer::onGlobalPath(const nav_msgs::Path::ConstPtr& msg)
{
    EventMessage event;
    event.set_type(EventType::EVENT_GLOBAL_PATH);

    auto* payload = event.mutable_global_path();
    for (auto& pose : msg->poses) {
        auto* p = payload->add_poses();
        p->mutable_position()->set_x(pose.pose.position.x);
        p->mutable_position()->set_y(pose.pose.position.y);
        p->mutable_position()->set_z(pose.pose.position.z);
        p->mutable_orientation()->set_x(pose.pose.orientation.x);
        p->mutable_orientation()->set_y(pose.pose.orientation.y);
        p->mutable_orientation()->set_z(pose.pose.orientation.z);
        p->mutable_orientation()->set_w(pose.pose.orientation.w);
    }

    std::string buffer;
    event.SerializeToString(&buffer);
    ZMQHandler::instance().send("event", buffer);
}

void EventInitializer::onPredictCollision(const geometry_msgs::PolygonStamped::ConstPtr& msg)
{
    EventMessage event;
    event.set_type(EventType::EVENT_PREDICT_COLLISION);

    auto* payload = event.mutable_obstacle();  // 구조 동일하게 사용
    payload->set_polygon_name("predict_collision");

    for (auto& point : msg->polygon.points) {
        auto* pos = payload->add_points();
        pos->set_x(point.x);
        pos->set_y(point.y);
        pos->set_z(0);
    }

    std::string buffer;
    event.SerializeToString(&buffer);
    ZMQHandler::instance().send("event", buffer);
}

void EventInitializer::onReflectors(const std_msgs::String::ConstPtr& msg)
{
    EventMessage event;
    event.set_type(EventType::EVENT_REFLECTORS);

    auto* payload = event.mutable_reflector();
    payload->set_uuid(Poco::UUIDGenerator::defaultGenerator().createRandom().toString());
    payload->set_name("detected_reflector");
    payload->set_marker_type_cd("D40402");
    payload->mutable_position()->set_x(0);
    payload->mutable_position()->set_y(0);
    payload->mutable_position()->set_z(0);
    payload->mutable_orientation()->set_x(0);
    payload->mutable_orientation()->set_y(0);
    payload->mutable_orientation()->set_z(0);
    payload->mutable_orientation()->set_w(1);

    std::string buffer;
    event.SerializeToString(&buffer);
    ZMQHandler::instance().send("event", buffer);
}

void EventInitializer::onTaskAlarm(const core_msgs::TaskAlarm::ConstPtr msg)
{
    EventMessage event;
    event.set_type(EventType::EVENT_TASK_ALARM);

    auto* payload = event.mutable_task_alarm();
    payload->set_uuid(Poco::UUIDGenerator::defaultGenerator().createRandom().toString());
    payload->set_action("task_alarm");
    payload->set_from("task_manager");
    payload->set_result("");

    try {
        int alarmValue = std::stoi(msg->alarm);
        payload->set_alarm(alarmValue);
    }
    catch (...) {
        payload->set_alarm(0);
    }

    payload->set_task_id(msg->uuid);

    std::string buffer;
    event.SerializeToString(&buffer);
    ZMQHandler::instance().send("event", buffer);
}

void EventInitializer::onTaskResponse(const core_msgs::TaskAlarm::ConstPtr& msg)
{
    EventMessage event;
    event.set_type(EventType::EVENT_TASK_RESPONSE);

    auto* payload = event.mutable_task_response();
    payload->set_uuid(Poco::UUIDGenerator::defaultGenerator().createRandom().toString());
    payload->set_result("success");

    try {
        int alarmValue = std::stoi(msg->alarm);
        payload->set_alarm(alarmValue);
    }
    catch (...) {
        payload->set_alarm(0);
    }

    std::string buffer;
    event.SerializeToString(&buffer);
    ZMQHandler::instance().send("event", buffer);
}

void EventInitializer::onNaviAlarm(const core_msgs::NaviAlarm::ConstPtr& msg)
{
    EventMessage event;
    event.set_type(EventType::EVENT_NAVI_ALARM);

    auto* payload = event.mutable_navi_alarm();
    payload->set_alarm_id(msg->alarm);  // alarm_id → alarm

    std::string buffer;
    event.SerializeToString(&buffer);
    ZMQHandler::instance().send("event", buffer);
}

void EventInitializer::onMappingProgress(const std_msgs::Int16::ConstPtr& msg)
{
    core_astra::EventMessage event;
    event.set_type(core_astra::EventType::EVENT_MAPPING_PROGRESS);

    // Payload 설정 (0~100 -> 0.0~1.0)
    auto* payload = event.mutable_mapping_progress();
    payload->set_percent(static_cast<float>(msg->data) / 100.0f);

    std::string buffer;
    event.SerializeToString(&buffer);

    ZMQHandler::instance().send("event", buffer);
}

void EventInitializer::onMotorInfo(const core_msgs::MotorInfo::ConstPtr& msg)
{
    InMemoryRepository::instance().get<RobotWheelInfoStore>(RobotWheelInfoStore::KEY)->updateWheelInfo(msg);

    try {
        auto motorInfoStore = InMemoryRepository::instance().get<MotorInfoStore>(MotorInfoStore::KEY);
        motorInfoStore->append(msg);

        if (motorInfoStore->size() == 5) {
            EventMessage event;
            event.set_type(EventType::EVENT_MOTOR_INFO);

            // JSON Object → String 변환
            Poco::JSON::Object::Ptr obj = motorInfoStore->toObject();
            std::ostringstream oss;
            obj->stringify(oss);  // Poco JSON → 문자열
            std::string jsonString = oss.str();

            auto* payload = event.mutable_custom();
            payload->set_uuid(Poco::UUIDGenerator::defaultGenerator().createRandom().toString());
            payload->set_message(jsonString);

            std::string buffer;
            event.SerializeToString(&buffer);
            ZMQHandler::instance().send("event", buffer);

            motorInfoStore->clear();
        }
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << "onMotorInfo error : " << ex.displayText();
    }
}

void EventInitializer::onSLAMGraph(const std_msgs::String::ConstPtr& msg)
{
    EventMessage event;
    event.set_type(EventType::EVENT_SLAM_GRAPH);

    event.mutable_custom()->set_message(msg->data);

    std::string buffer;
    event.SerializeToString(&buffer);
    ZMQHandler::instance().send("event", buffer);
}

void EventInitializer::onCaliProgress(const std_msgs::Float32::ConstPtr& msg)
{
    /**
     * Calibration progress event 아직 구현 안됨 이거 쓸지 안쓸지 모름
     */
    EventMessage event;
    event.set_type(EventType::EVENT_CALI_PROGRESS);

    event.mutable_custom()->set_message("Calibration Progress: " + std::to_string(msg->data));

    std::string buffer;
    event.SerializeToString(&buffer);
    ZMQHandler::instance().send("event", buffer);
}

void EventInitializer::onHardwareInfo(const std_msgs::String::ConstPtr& msg)
{
    /**
     * Hardware info event 아직 구현 안됨 이거 쓸지 안쓸지 모름
     */
    EventMessage event;
    event.set_type(EventType::EVENT_HARDWARE_INFO);

    event.mutable_custom()->set_message(msg->data);

    std::string buffer;
    event.SerializeToString(&buffer);
    ZMQHandler::instance().send("event", buffer);
}

void EventInitializer::onCustom(const std_msgs::String::ConstPtr& msg)
{
    EventMessage event;
    event.set_type(EventType::EVENT_CUSTOM);

    auto* payload = event.mutable_custom();
    payload->set_uuid(Poco::UUIDGenerator::defaultGenerator().createRandom().toString());
    payload->set_message(msg->data);

    std::string buffer;
    event.SerializeToString(&buffer);
    ZMQHandler::instance().send("event", buffer);
}
