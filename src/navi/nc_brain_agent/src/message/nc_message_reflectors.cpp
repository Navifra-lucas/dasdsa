#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/data/memory_repository.h>
#include <core_agent/data/robot_status.h>
#include <core_agent/message/message_broker.h>
#include <nc_brain_agent/message/nc_brain_message.h>
#include <nc_brain_agent/message/nc_message_reflectors.h>

using namespace NaviFra;

NcMessageReflectors::NcMessageReflectors()
{
}

void NcMessageReflectors::handleMessage(Poco::JSON::Object::Ptr message)
{
    // std::string json(Poco::format(
    //     "{\"id\":\"%s\",\"polygon_name\":\"%s\",\"points\":%s}",
    //     InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(), message->get("polygon_name").convert<std::string>(),
    //     message->get("points").convert<std::string>()));
    std::ostringstream oss;
    try {
        message->stringify(oss);
    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "Error ", e.what();
    }

    MessageBroker::instance().publish(
        NcBrainMessage::MESSAGE_ROBOT_STATUS_MARKER + InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(),
        oss.str());
    // NLOG(info) << std::endl << oss.str();
}