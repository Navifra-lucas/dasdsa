#include "nc_brain_agent/nc_brain_agent.h"

#include <Poco/JSON/Object.h>
#include <core_agent/core/navicore.h>
#include <core_agent/data/robot_status.h>
#include <core_agent/message/message_broker.h>
#include <nc_brain_agent/action/nc_action_task_command.h>
#include <nc_brain_agent/message/nc_brain_message.h>

using namespace NaviFra;

NcActionTaskCommand::NcActionTaskCommand()
{
}

NcActionTaskCommand::~NcActionTaskCommand()
{
}

std::string NcActionTaskCommand::implName()
{
    return "NcActionTaskCommand";
}

void NcActionTaskCommand::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();

    auto status = InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY);

    Poco::JSON::Object msg, data;
    msg.set("uuid", obj->get("uuid"));
    msg.set("id", status->getID());
    msg.set("result", "success");
    msg.set("data", data);

    std::string command;
    auto reData = obj->getObject("data");
    command = reData->get("command").extract<std::string>();
    bool b_cancel = true;
    if (reData->has("all_clear")) {
        b_cancel = reData->get("all_clear").extract<bool>(); // true : cancel, false : pause
    }
    if(command == "cancel" && !b_cancel)
    {
        command = "pause";
    }
    taskCommand(command);

    std::ostringstream ostr;
    msg.stringify(ostr);

    MessageBroker::instance().publish(source, NcBrainMessage::MESSAGE_TASK_RESPONSE + status->getID(), ostr.str());
    MessageBroker::instance().publish(source, NcBrainMessage::MESSAGE_ROBOT_RESPONSE + status->getID(), ostr.str());
}