#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <core_agent/data/memory_repository.h>
#include <core_agent/data/robot_info.h>
#include <core_agent/data/robot_status.h>
#include <core_agent/message/message_publisher.h>
#include <nc_brain_agent/message/nc_agent_heartbeat.h>
#include <nc_brain_agent/message/nc_brain_command.h>
#include <nc_brain_agent/message/nc_brain_message.h>
#include <nc_brain_agent/nc_robot_agent.h>
#include <nc_brain_agent/utils/nc_agent_utils.h>

using namespace NaviFra;
using namespace Poco::Redis;

void NcRobotAgent::run()
{
    std::array<bool, 3> preRedisState, curRedisState;
    while (!activity_.isStopped()) {
        try {
            // if (!DefaultSubscriber::instance().subscriber->isConnected()) {
            //    preRedisState[0] = curRedisState[0];
            //    curRedisState[0] = false;
            //    if (preRedisState[0] != curRedisState[0])
            //        LOG_WARNING("redis subscriber client disconnected. try reconnect....");
            //    DefaultSubscriber::instance().subscriber->reconnect();
            //}
            // else {
            //    preRedisState[0] = curRedisState[0];
            //    curRedisState[0] = true;
            //}

            // if (!DefaultPublisher::instance().publisher->isConnected()) {
            //    preRedisState[1] = curRedisState[1];
            //    curRedisState[1] = false;
            //    if (preRedisState[1] != curRedisState[1])
            //        LOG_WARNING("redis writer client disconnected. try reconnect....");
            //    DefaultPublisher::instance().publisher->reconnect();
            //}
            // else {
            //    preRedisState[1] = curRedisState[1];
            //    curRedisState[1] = true;
            //}

            // if (!redisReader_->isConnected()) {
            //     preRedisState[2] = curRedisState[2];
            //     curRedisState[2] = false;
            //     if (preRedisState[2] != curRedisState[2])
            //         LOG_WARNING("redis reader client disconnected. try reconnect....");
            //     redisReader_->reconnect();
            // }
            // else {
            //     preRedisState[2] = curRedisState[2];
            //     curRedisState[2] = true;
            //     /// sssssss
            // }
        }
        catch (Poco::Exception& e) {
            LOG_ERROR("%s", e.what());
        }
        if (!activity_.isStopped())
            Poco::Thread::sleep(100);
    }
}