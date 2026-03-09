#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/data/memory_repository.h>
#include <core_agent/data/robot_status.h>
#include <core_agent/message/message_broker.h>
#include <nc_brain_agent/message/nc_brain_message.h>
#include <nc_brain_agent/message/nc_message_motor_info.h>

using namespace NaviFra;

NcMessageMotorInfo::NcMessageMotorInfo()
{
}

void NcMessageMotorInfo::handleMessage(Poco::JSON::Object::Ptr message)
{
    message->set("id", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID());

    std::ostringstream oss;
    message->stringify(oss);
    MessageBroker::instance().publish(
        NcBrainMessage::MESSAGE_ROBOT_STATUS_MOTOR + InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(), oss.str());
    NLOG(trace) << std::endl << oss.str();
}