#include "nc_brain_agent/nc_brain_agent.h"

#include <Poco/Delegate.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <core_agent/core/navicore.h>
#include <core_agent/data/memory_repository.h>
#include <core_agent/data/robot_status.h>
#include <core_agent/message/message_broker.h>
#include <core_agent/message/message_publisher.h>
#include <core_agent/redis/redis_publisher.h>
#include <core_agent/redis/redis_subscriber.h>
#include <core_agent/util/config.h>
#include <nc_brain_agent/initializer/redis_initializer.h>
#include <nc_brain_agent/message/nc_brain_message.h>

using Poco::JSON::Object;
using Poco::JSON::Parser;

using namespace NaviFra;

namespace {
void onMessage(const void*, MessageSubscriberArgs& args)
{
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);

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
}
}

void RedisInitializer::initialize()
{
    bool use_redis = Config::instance().getBool("use_redis", false);

    if (!use_redis) {
        LOG_INFO("Redis disabled. Skipping initialization.");
        return;
    }

    // ---------- Publisher 초기화 ----------
    {
        MessagePublisherFactory pubFactory;
        auto publisher = pubFactory.createMessagePublisher<RedisPublisher>();

        if (publisher && publisher->initialize()) {
            LOG_INFO("RedisPublisher connected");
            MessageBroker::instance().addPublisher("redis", publisher);
        }
        else {
            LOG_ERROR("Failed to initialize RedisPublisher");
        }
    }

    // ---------- Subscriber 초기화 ----------
    {
        LOG_INFO("Redis enabled. Initializing subscriber...");
        MessageSubscriberFactory subFactory;
        auto subscriber = subFactory.createMessageSubscriber<RedisSubscriber>();

        if (subscriber && subscriber->initialize()) {
            LOG_INFO("Redis subscriber initialized successfully.");

            auto robotStatus = InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY);
            std::string robotId = robotStatus ? robotStatus->getID() : "UNKNOWN";

            subscriber->notify_ += Poco::delegate(onMessage);

            subscriber->subscribe(
                {NcBrainMessage::MESSAGE_ROBOT_REQUEST + robotId, NcBrainMessage::MESSAGE_TASK_REQUEST + robotId,
                 NcBrainMessage::MESSAGE_BACKEND_REQUEST, NcBrainMessage::MESSAGE_ACS_RESPONSE});

            MessageBroker::instance().addSubscriber("redis", subscriber);
        }
        else {
            LOG_ERROR("Failed to initialize RedisSubscriber");
        }
    }
}
