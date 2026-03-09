#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/data/memory_repository.h>
#include <core_agent/data/robot_status.h>
#include <core_agent/message/message_broker.h>
#include <nc_brain_agent/data/nc_brain_map.h>
#include <nc_brain_agent/message/nc_brain_message.h>
#include <nc_brain_agent/message/nc_message_slam_node.h>

using namespace NaviFra;

NcMessageSLAMNode::NcMessageSLAMNode()
{
}

void NcMessageSLAMNode::handleMessage(Poco::JSON::Object::Ptr message)
{
    Poco::JSON::Object::Ptr slam = InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->getSlamMap();
    message->set("id", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID());
    message->set("can_save", true);

    if (slam.get()) {
        auto names = slam->getNames();
        for (const auto& name : names) {
            message->set(name, slam->get(name));
        }
    }

    std::ostringstream oss;
    message->stringify(oss);
    MessageBroker::instance().publish(
        NcBrainMessage::MESSAGE_ROBOT_STATUS_SLAM + InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(), oss.str());
    NLOG(trace) << std::endl << oss.str();
}