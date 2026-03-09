
#include "nc_brain_agent/nc_brain_agent.h"

#include <Poco/Stopwatch.h>
#include <core_agent/core/navicore.h>
#include <core_agent/data/robot_info.h>
#include <core_agent/data/robot_status.h>
#include <nc_brain_agent/action/async/nc_async_action_slam_start.h>
#include <nc_brain_agent/message/nc_brain_message.h>
#include <ros/master.h>
#include <ros/node_handle.h>

using namespace NaviFra;
using Poco::JSON::Object;

enum STATE_SLAM
{
    STATE_SLAM_STOP_NAVI = 0,
    STATE_SLAM_WAIT_STOP_NAVI,
    STATE_SLAM_START_SLAM,
    STATE_SLAM_WAIT_START_SLAM,
    STATE_SLAM_START_DONE
};

NcAsyncActionSLAMStart::NcAsyncActionSLAMStart(std::string uuid)
    : uuid_(uuid)
{
}

NcAsyncActionSLAMStart::~NcAsyncActionSLAMStart()
{
}

void NcAsyncActionSLAMStart::run()
{
    Poco::Stopwatch sw;
    sw.start();
    STATE_SLAM state = STATE_SLAM_STOP_NAVI;
    bool bfound = true;
    std::vector<std::string> masterNodes;
    std::vector<std::string> namespace_removed_nodes;
    while (true) {
        switch (state) {
            case STATE_SLAM_STOP_NAVI:  // stop navi
                // block until localization is terminated
                stopNavigation();
                state = STATE_SLAM_WAIT_STOP_NAVI;
                LOG_INFO("Stop Navigation Node");
                break;
            case STATE_SLAM_WAIT_STOP_NAVI:  // check node kill
            {
#if 0
                masterNodes.clear();
                ros::master::getNodes(masterNodes);
                auto not_npos = std::string::npos;
                for (auto node_name : masterNodes) {
                    not_npos = node_name.find("nc_localizer_ros");
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
                    state = STATE_SLAM_START_SLAM;
                    LOG_INFO("Stop Complete Navigation Node");
                    bfound = false;
                }
            } break;
            case STATE_SLAM_START_SLAM:
            {
                // block until slam is started
                startMapping();
                state = STATE_SLAM_WAIT_START_SLAM;
                LOG_INFO("Start SLAM Node");
                bfound = false;
            } break;
            case STATE_SLAM_WAIT_START_SLAM:
            {
                LOG_INFO("STATE_SLAM_WAIT_START_SLAM");
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
                if (bfound)
#endif
                {
                    state = STATE_SLAM_START_DONE;
                    LOG_INFO("Started SLAM Node");
                    InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->setSLAM(true);
                    InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY)->setStatus("MAPPING");

                    Object response, emptyData;
                    response.set("uuid", uuid_);
                    response.set("id", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID());
                    response.set("result", "success");
                    response.set("data", emptyData);

                    std::ostringstream oss;
                    response.stringify(oss);
                    MessageBroker::instance().publish(
                        NcBrainMessage::MESSAGE_ROBOT_RESPONSE + InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(),
                        oss.str());
                    break;
                }
            } break;
        }

        if (state == STATE_SLAM_START_DONE) {
            LOG_INFO("Task Start SLAM success");
            break;
        }
        else if (sw.elapsedSeconds() > 30) {
            LOG_INFO("Started SLAM Node Fail");
            InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->setSLAM(false);
            Object response, emptyData;
            response.set("uuid", uuid_);
            response.set("id", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID());
            response.set("result", "fail");
            response.set("data", emptyData);

            std::ostringstream oss;
            response.stringify(oss);
            MessageBroker::instance().publish(
                NcBrainMessage::MESSAGE_ROBOT_RESPONSE + InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(),
                oss.str());
            break;
        }

        Poco::Thread::sleep(1000);
    }

    delete this;
}
