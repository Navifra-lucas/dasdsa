#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/data/memory_repository.h>
#include <core_agent/data/robot_status.h>
#include <core_agent/message/message_broker.h>
#include <nc_brain_agent/message/nc_brain_message.h>
#include <nc_brain_agent/message/nc_message_polygon.h>

using namespace NaviFra;

NcMessagePolygon::NcMessagePolygon()
{
}

void NcMessagePolygon::handleMessage(Poco::JSON::Object::Ptr message)
{
    std::string json(Poco::format(
        "{\"id\":\"%s\",\"polygon_name\":\"%s\",\"points\":%s}", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(),
        message->get("polygon_name").convert<std::string>(), message->get("points").convert<std::string>()));
    MessageBroker::instance().publish(
        NcBrainMessage::MESSAGE_ROBOT_STATUS_POLYGON + InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(), json);
    // NLOG(trace) << std::endl << oss.str();
}