#include "nc_brain_agent/initializer/ros_initializer.h"

#include "core_agent/core/navicore_message.h"
#include "core_agent/data/lidar_merger.h"
#include "core_agent/data/memory_repository.h"
#include "core_agent/data/robot_collision_info.h"
#include "core_agent/data/robot_info.h"
#include "core_agent/data/robot_pose.h"
#include "core_agent/data/robot_status.h"
#include "core_agent/manager/alarm_manager.h"
#include "core_agent/manager/message_handler_manager.h"
#include "core_agent/manager/publish_manager.h"
#include "core_agent/message/message_broker.h"
#include "nc_brain_agent/data/nc_status_channel.h"
#include "nc_brain_agent/message/nc_agent_heartbeat.h"
#include "nc_brain_agent/message/nc_brain_message.h"
#include "util/logger.hpp"

#include <nc_brain_agent/message/nc_agent_heartbeat.h>
#include <nc_brain_agent/message/nc_brain_command.h>
#include <nc_brain_agent/message/nc_brain_message.h>
#include <nc_brain_agent/message/nc_message_alarm.h>
#include <nc_brain_agent/message/nc_message_cali_progress.h>
#include <nc_brain_agent/message/nc_message_custom.h>
#include <nc_brain_agent/message/nc_message_global_path.h>
#include <nc_brain_agent/message/nc_message_hardware_info.h>
#include <nc_brain_agent/message/nc_message_local_path.h>
#include <nc_brain_agent/message/nc_message_mapping.h>
#include <nc_brain_agent/message/nc_message_motor_info.h>
#include <nc_brain_agent/message/nc_message_polygon.h>
#include <nc_brain_agent/message/nc_message_reflectors.h>
#include <nc_brain_agent/message/nc_message_slam_node.h>
#include <nc_brain_agent/message/nc_message_task_alarm.h>
#include <nc_brain_agent/message/nc_message_task_response.h>

namespace NaviFra {

void ROSInitializer::initialize()
{
    // ---- Message Handlers 등록 ----
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_LOCAL_PATH, std::make_shared<NcMessageLocalPath>());
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_GLOBAL_PATH, std::make_shared<NcMessageGlobalPath>());
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_OBSTACLE, std::make_shared<NcMessagePolygon>());
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_PREDICT_COLLISION, std::make_shared<NcMessagePolygon>());
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_CUSTOM, std::make_shared<NcMessageCustom>());
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_TASK_ALAM, std::make_shared<NcMessageTaskAlarm>());
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_TASK_RESPONSE, std::make_shared<NcMessageTaskResponse>());
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_MAPING_PROGRESS, std::make_shared<NcMessageMapping>());
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESASGE_MOTOR_INFO, std::make_shared<NcMessageMotorInfo>());
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_SLAM_GRAPH, std::make_shared<NcMessageSLAMNode>());
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_CALI_PROGRESS, std::make_shared<NcMessageCaliProgress>());
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_HARDWARE_INFO, std::make_shared<NcMessageHardwareInfo>());
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_NAVI_ALARM, std::make_shared<NcMessageAlarm>());
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_SET_REFLECTORS, std::make_shared<NcMessageReflectors>());

    // ---- Publisher Channels 등록 ----
    std::string id = InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID();
    NcAgentHearbeat heartbeat;
    heartbeat.setID(id);

    // Heartbeat
    PublisherManager::instance().addChannel(
        (int)PUBLISH_CHANNEL::CHANNEL_HEARBEAT,
        [heartbeat, id]() mutable {
            Poco::Timestamp now;
            heartbeat.setTimestamp(now);

            auto robotStatus = InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY);
            heartbeat.setIsSlam(robotStatus->isSLAM());
            heartbeat.setVersion(robotStatus->getVersion());

            MessageBroker::instance().publish(NcBrainMessage::MESSAGE_ROBOT_HEARTBEAT_STATUS + id, heartbeat.toString());
        },
        500, 0);

    // Robot Info
    PublisherManager::instance().addChannel(
        (int)PUBLISH_CHANNEL::CHANNEL_INFO_STATUS,
        []() mutable {
            auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
            MessageBroker::instance().publish(NcBrainMessage::MESSAGE_ROBOT_STATUS_INFO + robotInfo->getID(), robotInfo->toString());

            // 이거 아래 예전에 그레이가 만든건데 일단 필요 없어서 삭제 함
            // updateRobotInfoPLC(robotInfo->getRobotDrivingInfoToString());
        },
        100, 0);

    // Lidar
    PublisherManager::instance().addChannel(
        (int)PUBLISH_CHANNEL::CHANNEL_LIDAR,
        []() mutable {
            auto lidar = InMemoryRepository::instance().get<LidarMerger>(LidarMerger::KEY);
            std::string json = Poco::format(
                "{\"id\":\"%s\",\"points\":%s}", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(),
                lidar->toString());

            MessageBroker::instance().publish(
                NcBrainMessage::MESSAGE_ROBOT_STATUS_SCAN + InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(),
                json);
        },
        1000, 120000);

    // Alarm Clear Channel
    PublisherManager::instance().addChannel(
        (int)PUBLISH_CHANNEL::CHANNEL_ALARM,
        []() mutable {
            std::string status = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY)->getStatus();
            if ((status == "running" || status == "idle") && AlarmManager::instance().size() > 0) {
                AlarmManager::instance().clearAllAlarms();
            }
        },
        200, 0);

    // Pose & Velocity
    PublisherManager::instance().addChannel(
        (int)PUBLISH_CHANNEL::CHANNEL_STATUS_POSE,
        []() mutable {
            auto pose = InMemoryRepository::instance().get<RobotPose>(RobotPose::KEY);
            auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
            
            Poco::JSON::Object::Ptr obj = new Poco::JSON::Object;
            obj->set("id", robotInfo->getID());
            obj->set("pose", pose->toObject());
            obj->set("velocity", robotInfo->getVelocity());
            obj->set("collision", InMemoryRepository::instance().get<RobotCollisionInfo>(RobotCollisionInfo::KEY)->getCollision());

            std::ostringstream oss;
            obj->stringify(oss);

            MessageBroker::instance().publish(
                "robot.status.pose:" + InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(), oss.str());
        },
        1000, 0);

    // 기본 비활성화 후 시작
    PublisherManager::instance().deactivate((int)PUBLISH_CHANNEL::CHANNEL_LIDAR);
    PublisherManager::instance().start();

    LOG_INFO("ROS Message handlers and publisher channels initialized successfully.");
}

}  // namespace NaviFra
