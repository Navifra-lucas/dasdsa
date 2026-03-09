#include "nc_brain_agent/nc_brain_agent.h"

#include <Poco/Stopwatch.h>
#include <core_agent/core/navicore.h>
#include <core_agent/data/robot_info.h>
#include <core_agent/data/robot_status.h>
#include <nc_brain_agent/action/async/nc_async_action_slam_stop.h>
#include <nc_brain_agent/message/nc_brain_message.h>
#include <ros/master.h>

using namespace NaviFra;
using Poco::JSON::Object;

enum STATE_SLAM
{
    STATE_SLAM_STOP_SLAM = 0,
    STATE_SLAM_WAIT_STOP_SLAM,
    STATE_SLAM_START_NAVI,
    STATE_SLAM_WAIT_START_NAVI,
    STATE_SLAM_STOP_DONE
};

NcAsyncActionSLAMStop::NcAsyncActionSLAMStop(std::string uuid)
    : uuid_(uuid)
{
}

NcAsyncActionSLAMStop::~NcAsyncActionSLAMStop()
{
}

void NcAsyncActionSLAMStop::run()
{
    Poco::Stopwatch sw;
    sw.start();
    STATE_SLAM state = STATE_SLAM_STOP_SLAM;
    bool bfound = false;
    std::vector<std::string> masterNodes;
    while (true) {
        switch (state) {
            case STATE_SLAM_STOP_SLAM:  // stop navi
                // block until slam is terminated
                stopMapping();
                state = STATE_SLAM_WAIT_STOP_SLAM;
                LOG_INFO("Stop SLAM Node");
                break;
            case STATE_SLAM_WAIT_STOP_SLAM:  // check node kill
            {
#if 0
                masterNodes.clear();

                ros::master::getNodes(masterNodes);

                auto not_npos = std::string::npos;
                for (auto node_name : masterNodes) {
                    not_npos = node_name.find("slam_node");
                    if (not_npos != std::string::npos) {
                        bfound = true;
                        break;
                    }
                    else {
                        bfound = false;
                    }
                }

                if (!bfound)
#endif
                {
                    state = STATE_SLAM_START_NAVI;
                    LOG_INFO("Stop Complete SLAM Node");
                    bfound = false;
                }
            } break;
            case STATE_SLAM_START_NAVI:
            {
                // block until localization is started
                startNavigation();
                state = STATE_SLAM_WAIT_START_NAVI;
                LOG_INFO("Start Navigator Node");
                bfound = false;
            } break;
            case STATE_SLAM_WAIT_START_NAVI:
            {
#if 0
                masterNodes.clear();
                ros::master::getNodes(masterNodes);

                auto not_npos = std::string::npos;
                for (auto node_name : masterNodes) {
                    not_npos = node_name.find("nc_navigator");
                    if (not_npos != std::string::npos) {
                        bfound = true;
                        break;
                    }
                    else {
                        bfound = false;
                    }
                }

                if (bfound)
#endif
                {
                    state = STATE_SLAM_STOP_DONE;
                    LOG_INFO("Started Navigator Node");
                    InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->setSLAM(false);
                    InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY)->setStatus("idle");
                    sendResponse();
                    break;
                }
            } break;
        }

        if (state == STATE_SLAM_STOP_DONE) {
            LOG_INFO("Task Stop SLAM success");
            break;
        }
        else if (sw.elapsedSeconds() > 30) {
            LOG_INFO("Started Navigator Node Fail");
            InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->setSLAM(false);
            sendResponse("fail");
            break;
        }

        Poco::Thread::sleep(1000);
    }

    delete this;
}

void NcAsyncActionSLAMStop::sendResponse(std::string result)
{
    // UUID 없이 끄는 경우 Response를 보내지 않는다.
    if (uuid_.empty())
        return;

    Object response, emptyData;
    response.set("uuid", uuid_);
    response.set("id", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID());
    response.set("result", result);
    response.set("data", emptyData);

    std::ostringstream oss;
    response.stringify(oss);
    MessageBroker::instance().publish(
        NcBrainMessage::MESSAGE_ROBOT_RESPONSE + InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(), oss.str());
}
