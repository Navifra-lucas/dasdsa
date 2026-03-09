#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/data/memory_repository.h>
#include <core_agent/data/robot_status.h>
#include <core_agent/manager/uuid_response_manager.h>
#include <core_agent/message/message_broker.h>
#include <nc_brain_agent/message/nc_brain_message.h>
#include <nc_brain_agent/message/nc_message_task_alarm.h>

using namespace NaviFra;

NcMessageTaskAlarm::NcMessageTaskAlarm()
{
}

void NcMessageTaskAlarm::handleMessage(Poco::JSON::Object::Ptr message)
{
    try {
        if (message->has("data"))
            message->getObject("data")->set("robot_id", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID());

        UUIDResponseManager::instance().registerUUID(message->get("uuid").convert<std::string>(), 5);  // timeout 5 초

        std::ostringstream oss;
        message->stringify(oss);
        MessageBroker::instance().publish(NcBrainMessage::MESSAGE_ACS_REQUEST, oss.str());
        NLOG(trace) << std::endl << oss.str();
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
    }
}