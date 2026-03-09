
#include "nc_brain_agent/nc_brain_agent.h"

#include <Poco/Delegate.h>
#include <core_agent/manager/alarm_manager.h>
#include <core_agent/manager/initializer_manager.h>
#include <core_agent/manager/publish_manager.h>
#include <core_agent/message/message_broker.h>
#include <core_agent/mqtt/mqtt_publisher.h>
#include <core_agent/redis/redis_commnder.h>
#include <core_agent/redis/redis_publisher.h>
#include <core_agent/util/config.h>
#include <nc_brain_agent/data/nc_agent_parameters.h>
#include <nc_brain_agent/message/nc_brain_command.h>
#include <nc_brain_agent/nc_robot_agent.h>
#include <nc_brain_agent/net/nc_rest_api_utils.h>
#include <nc_brain_agent/service/nc_playback_service.h>
#include <nc_brain_agent/service/nc_repeat_drive_service.h>
#include <nc_brain_agent/service/nc_web_service.h>
#include <nc_brain_agent/utils/nc_agent_utils.h>

using namespace NaviFra;
using namespace Poco::Redis;

using Poco::Net::HTTPResponse;

namespace {
static Poco::SingletonHolder<NcRobotAgent> sh;
}

NcRobotAgent::NcRobotAgent()
    : activity_(this, &NcRobotAgent::run)
{
}

NcRobotAgent::~NcRobotAgent()
{
    // PublisherManager::instance().stop();
}
NcRobotAgent& NcRobotAgent::get()
{
    return *sh.get();
}

bool NcRobotAgent::initialize()
{
    try {
        InitializerManager::instance().run();
        // initRobotStartUp();

#ifndef CPP_UNIT_TEST
        NcPlaybackService::get().initialize(nodeHandler_);
        NcRepeatDriveService::get().initialize(nodeHandler_);
        NcWEBService::get().initialize();

        LOG_INFO("NcRobotAgent initialize success");
        start();
#endif
    }
    catch (const Poco::Exception& ex) {
        LOG_ERROR("%s", ex.displayText().c_str());
        return false;
    }
    return true;
}

void NcRobotAgent::finalize()
{
    stop();

    if (redisReader_.get()) {
        redisReader_.reset();
    }

    if (DefaultSubscriber::instance().subscriber.get()) {
        DefaultSubscriber::instance().subscriber->notify_ -= Poco::delegate(this, &NcRobotAgent::onMessage);
    }
}

void NcRobotAgent::initRobotCalibrationBase()
{
    // TODO //Type 별로 수정 해야함
    std::string pos_key = NcAgentConfig::get().getRobotBasePos();
    std::string pos_offset = NcAgentConfig::get().getRobotBaseOffset();
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    robotInfo->updateRobotBase(
        NcAgentParameters::get().getParameters()[pos_key].as<double>(), NcAgentParameters::get().getParameters()[pos_offset].as<double>());
}