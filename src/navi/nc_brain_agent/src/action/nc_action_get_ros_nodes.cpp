#include "nc_brain_agent/nc_brain_agent.h"

#include <nc_brain_agent/action/nc_action_get_ros_nodes.h>
#include <nc_brain_agent/data/nc_agent_node_manager.h>

using namespace NaviFra;

NcActionGetROSNodes::NcActionGetROSNodes()
{
}

NcActionGetROSNodes::~NcActionGetROSNodes()
{
}

std::string NcActionGetROSNodes::implName()
{
    return "NcActionGetROSNodes";
}

void NcActionGetROSNodes::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();
    Poco::JSON::Object::Ptr data = obj->getObject("data");
    NLOG(info) << "NcActionGetROSNodes::implonAction action: " << action;
    sendResponseSuccessWithData(source, obj->get("uuid").convert<std::string>(), NcAgentNodeManager::get().getNodeStatus());
    NLOG(info) << "NcActionGetROSNodes::implonAction completed";
}
