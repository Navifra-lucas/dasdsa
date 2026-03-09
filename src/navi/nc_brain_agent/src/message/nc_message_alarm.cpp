#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/manager/alarm_manager.h>
#include <nc_brain_agent/message/nc_brain_message.h>
#include <nc_brain_agent/message/nc_message_alarm.h>

using namespace NaviFra;

NcMessageAlarm::NcMessageAlarm()
{
}

void NcMessageAlarm::handleMessage(Poco::JSON::Object::Ptr message)
{
    uint32_t alarm_id = message->get("alarm_id").convert<uint32_t>();
    std::string descrition = message->get("descrition").convert<std::string>();
    AlarmManager::instance().setAlarm(alarm_id, descrition);
}