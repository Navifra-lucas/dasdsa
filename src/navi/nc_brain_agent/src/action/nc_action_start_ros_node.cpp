#include "nc_brain_agent/nc_brain_agent.h"

#include <nc_brain_agent/action/async/nc_async_action_ros_node_start.h>
#include <nc_brain_agent/action/nc_action_start_ros_node.h>

using namespace NaviFra;

NcActionStartROSNode::NcActionStartROSNode()
{
}

NcActionStartROSNode::~NcActionStartROSNode()
{
}

std::string NcActionStartROSNode::implName()
{
    return "NcActionStartROSNode";
}

void NcActionStartROSNode::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    Poco::JSON::Object::Ptr data = obj->getObject("data");
    auto r = new NcAsyncActionROSNodeStart(obj->get("uuid").convert<std::string>(), data->get("ros_node_id").convert<std::string>());
    Poco::ThreadPool::defaultPool().start(*r);
}
