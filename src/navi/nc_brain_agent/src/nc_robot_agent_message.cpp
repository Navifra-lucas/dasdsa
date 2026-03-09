#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <core_agent/data/robot_info.h>
#include <core_agent/data/robot_status.h>
#include <core_agent/manager/action_manager.h>
#include <core_agent/message/message_publisher.h>
#include <nc_brain_agent/data/nc_agent_node_manager.h>
#include <nc_brain_agent/data/nc_agent_parameters.h>
#include <nc_brain_agent/data/nc_brain_map.h>
#include <nc_brain_agent/message/nc_brain_command.h>
#include <nc_brain_agent/message/nc_brain_message.h>
#include <nc_brain_agent/nc_robot_agent.h>
#include <nc_brain_agent/utils/nc_agent_utils.h>

using Poco::DynamicStruct;
using Poco::JSON::Object;
using Poco::JSON::Parser;

using namespace NaviFra;

bool NcRobotAgent::onBackendStatus(const std::string& message)
{
    Parser parser;
    Poco::Dynamic::Var result = parser.parse(message);
    Object::Ptr obj = result.extract<Object::Ptr>();

    if (obj->get("id").convert<std::string>() == InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID()) {
        InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->setEnable(obj->get("is_enabled").convert<bool>());
        InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->setActive(obj->get("job_is_active").convert<bool>());
    }

    return true;
}

bool NcRobotAgent::onRobotRequest(const std::string& message)
{
    try {
        Parser parser;
        Poco::Dynamic::Var result = parser.parse(message);
        Object::Ptr obj = result.extract<Object::Ptr>();
        std::string action = obj->get("action").extract<std::string>();
        std::string uuid = obj->get("uuid").extract<std::string>();

        LOG_INFO("Recieved action of robot.request : [ %s ]", action.c_str());
        // actionManager_->onAction(std::move(obj));
    }
    catch (Poco::Exception ex) {
        LOG_ERROR("%s", ex.displayText().c_str());
    }

    return true;
}

void NcRobotAgent::onUpdatedMap()
{
    updateBrainMap();
}

void NcRobotAgent::onMessage(const void*, MessageSubscriberArgs& args)
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
        onBackendStatus(args.message());
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
        onBackendStatus(args.message());
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
    else if( args.channel() == NcBrainMessage::MESSAGE_BACKEND_AREA || args.channel() == NcBrainMessage::MESSAGE_BACKEND_PORTAL) 
    {
        NLOG(info) << "Recieved Backend AREA PORTAL" << args.channel() << " : [ " << args.message() << "]";
    }

}