#include "nc_brain_agent/nc_brain_agent.h"

#include <Poco/Delegate.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <core_agent/core/navicore.h>
#include <core_agent/data/memory_repository.h>
#include <core_agent/data/robot_status.h>
#include <core_agent/message/message_broker.h>
#include <core_agent/message/message_publisher.h>
#include <core_agent/mqtt/mqtt_publisher.h>
#include <core_agent/mqtt/mqtt_subscriber.h>
#include <core_agent/util/config.h>
#include <nc_brain_agent/initializer/mqtt_initializer.h>
#include <nc_brain_agent/message/nc_brain_message.h>
#include <nc_brain_agent/data/nc_brain_map.h>

using Poco::JSON::Object;
using Poco::JSON::Parser;

using namespace NaviFra;

namespace {
void onMessage(const void*, MessageSubscriberArgs& args)
{
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    // NLOG(info) << "Recieved message on channel: " << args.channel() << " with message: " << args.message();

    if (args.channel() == NcBrainMessage::MESSAGE_ROBOT_REQUEST + robotInfo->getID() ||
        args.channel() == NcBrainMessage::MESSAGE_TASK_REQUEST + robotInfo->getID() ||
        args.channel() == NcBrainMessage::MESSAGE_ACS_RESPONSE) {
        try {
            LOG_INFO("Recieved %s : [ %s ]", args.channel().c_str(), args.message().c_str());

            Parser parser;
            Poco::Dynamic::Var result = parser.parse(args.message());
            Object::Ptr obj = result.extract<Object::Ptr>();

            if (obj->has("action")) {
                std::string action = obj->get("action").extract<std::string>();
                std::string uuid = obj->get("uuid").extract<std::string>();

                LOG_INFO("Recieved action of %s : [ %s ]", args.channel().c_str(), action.c_str());
                ActionManager::instance().onAction(args.source(), std::move(obj));
            }
        }
        catch (Poco::Exception ex) {
            LOG_ERROR("%s", ex.displayText().c_str());
        }
    }
    else if (args.channel() == NcBrainMessage::MESSAGE_BACKEND_REQUEST) {
        Parser parser;
        Poco::Dynamic::Var result = parser.parse(args.message());
        Object::Ptr obj = result.extract<Object::Ptr>();

        if (obj->get("id").convert<std::string>() == InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID()) {
            InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->setEnable(obj->get("is_enabled").convert<bool>());
            InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->setActive(obj->get("job_is_active").convert<bool>());
        }
    }

    if (args.channel() == "robot/request/" + robotInfo->getID() || args.channel() == "task_manager/request/" + robotInfo->getID() ||
        args.channel() == "acs/response") {
        try {
            LOG_INFO("Recieved %s : [ %s ]", args.channel().c_str(), args.message().c_str());

            Parser parser;
            Poco::Dynamic::Var result = parser.parse(args.message());
            Object::Ptr obj = result.extract<Object::Ptr>();
            std::string action = obj->get("action").extract<std::string>();
            std::string uuid = obj->get("uuid").extract<std::string>();

            LOG_INFO("Recieved action of %s : [ %s ]", args.channel().c_str(), action.c_str());
            ActionManager::instance().onAction(args.source(), std::move(obj));
        }
        catch (Poco::Exception ex) {
            LOG_ERROR("%s", ex.displayText().c_str());
        }
    }
    else if (args.channel() == "backend/status/robot") {
        Parser parser;
        Poco::Dynamic::Var result = parser.parse(args.message());
        Object::Ptr obj = result.extract<Object::Ptr>();

        if (obj->get("id").convert<std::string>() == InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID()) {
            InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->setEnable(obj->get("is_enabled").convert<bool>());
            InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->setActive(obj->get("job_is_active").convert<bool>());
        }
    }
    else if (args.channel() == "robot/status/pose/#") {
        // Robot Pose 정보 이면
        Parser parser;
        Poco::Dynamic::Var result = parser.parse(args.message());
        Object::Ptr obj = result.extract<Object::Ptr>();

        if (obj->has("id") &&
            obj->getValue<std::string>("id") != InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID()) {
            SendNearestRobotPose(args.message());
        }
    }
    else if( args.channel() == NcBrainMessage::MESSAGE_BACKEND_AREA || args.channel() == NcBrainMessage::MESSAGE_BACKEND_PORTAL
            || args.channel() == "backend/status/map/areas" || args.channel() == "backend/status/map/portal") {
        // Backend에서 Area 정보가 들어 온 경우
        NLOG(info) << "Recieved Backend AREA PORTAL " << args.channel() << " : [ " << args.message() << "]";
        InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->updateAreaPortal();
    }
}
}
void MqttInitializer::initialize()
{
    bool use_mqtt = Config::instance().getBool("use_mqtt", false);

    if (!use_mqtt) {
        LOG_INFO("MQTT disabled. Skipping initialization.");
        return;
    }

    // ---------------- Publisher 초기화 ----------------
    {
        MessagePublisherFactory pubFactory;
        auto publisher = pubFactory.createMessagePublisher<MQTTPublisher>();

        if (publisher && publisher->initialize()) {
            LOG_INFO("MQTTPublisher connected");
            MessageBroker::instance().addPublisher("mqtt", publisher);
        }
        else {
            LOG_ERROR("Failed to initialize MQTT Publisher");
        }
    }

    // ---------------- Subscriber 초기화 ----------------
    {
        LOG_INFO("MQTT enabled. Initializing subscriber...");
        MessageSubscriberFactory subFactory;
        auto subscriber = subFactory.createMessageSubscriber<MQTTSubscriber>();

        if (subscriber && subscriber->initialize()) {
            LOG_INFO("MQTT subscriber initialized successfully.");

            auto robotStatus = InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY);
            std::string robotId = robotStatus ? robotStatus->getID() : "UNKNOWN";
            subscriber->notify_ += Poco::delegate(onMessage);

            subscriber->subscribe(
                {NcBrainMessage::MESSAGE_ROBOT_REQUEST + robotId, NcBrainMessage::MESSAGE_TASK_REQUEST + robotId,
                 NcBrainMessage::MESSAGE_BACKEND_REQUEST, NcBrainMessage::MESSAGE_ACS_RESPONSE, //"robot.status.pose:*",
                 NcBrainMessage::MESSAGE_BACKEND_AREA, NcBrainMessage::MESSAGE_BACKEND_PORTAL});


            MessageBroker::instance().addSubscriber("mqtt", subscriber);
        }
        else {
            LOG_ERROR("Failed to initialize MQTT Subscriber");
        }
    }
}
