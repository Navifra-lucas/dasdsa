#include "nc_brain_agent/nc_brain_agent.h"

#include <nc_brain_agent/action/async/nc_async_action_ros_node_stop.h>
#include <nc_brain_agent/action/nc_action_stop_ros_node.h>

using namespace NaviFra;

NcActionStopROSNode::NcActionStopROSNode()
{
}

NcActionStopROSNode::~NcActionStopROSNode()
{
}

std::string NcActionStopROSNode::implName()
{
    return "NcActionStopROSNode";
}

void NcActionStopROSNode::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    Poco::JSON::Object::Ptr data = obj->getObject("data");

    auto r = new NcAsyncActionROSNodeStop(obj->get("uuid").convert<std::string>(), data->get("ros_node_id").convert<std::string>());
    Poco::ThreadPool::defaultPool().start(*r);
}
