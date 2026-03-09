#include "nc_brain_agent/data/nc_brain_map.h"
#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/data/memory_repository.h>
#include <core_agent/data/robot_status.h>
#include <core_agent/message/message_broker.h>
#include <nc_brain_agent/message/nc_brain_message.h>
#include <nc_brain_agent/message/nc_message_mapping.h>

using namespace NaviFra;

NcMessageMapping::NcMessageMapping()
{
}

void NcMessageMapping::handleMessage(Poco::JSON::Object::Ptr message)
{
    message->set("id", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID());

    std::ostringstream oss;
    message->stringify(oss);
    MessageBroker::instance().publish(
        NcBrainMessage::MESSAGE_ROBOT_STATUS_SLAM_SAVE + InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(),
        oss.str());
    NLOG(trace) << std::endl << oss.str();
    double percent = message->has("percent") ? message->get("percent").convert<double>() : 0.0;

    if (message->has("percent") && fabs(percent - 100.0) < DBL_EPSILON) {
        LOG_INFO("SLAM SAVE COMPLETE");
        Poco::JSON::Object::Ptr slam = InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->getSlamMap();
        Poco::JSON::Object message;
        message.set("id", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID());
        message.set("can_save", true);

        if (slam.get()) {
            auto names = slam->getNames();
            for (const auto& name : names) {
                message.set(name, slam->get(name));
            }
        }

        std::ostringstream ossSLAM;
        message.stringify(ossSLAM);
        MessageBroker::instance().publish(
            NcBrainMessage::MESSAGE_ROBOT_STATUS_SLAM + InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(),
            ossSLAM.str());
    }
}