
#include "nc_brain_agent/nc_brain_agent.h"

#include <Poco/Stopwatch.h>
#include <core_agent/core/navicore.h>
#include <core_agent/data/robot_info.h>
#include <core_agent/data/robot_status.h>
#include <core_agent/message/message_broker.h>
#include <nc_brain_agent/action/async/nc_async_action_ros_node_stop.h>
#include <nc_brain_agent/message/nc_brain_message.h>
#include <ros/master.h>

using namespace NaviFra;
using Poco::JSON::Object;

enum STATE_START_ROS
{
    STATE_ROS_NODE_STOP = 0,
    STATE_ROS_NODE_STOP_WAIT,
    STATE_ROS_NODE_STOP_DONE
};

NcAsyncActionROSNodeStop::NcAsyncActionROSNodeStop(std::string uuid, std::string ros_node_id)
    : uuid_(uuid)
    , ros_node_id_(ros_node_id)
{
}

NcAsyncActionROSNodeStop::~NcAsyncActionROSNodeStop()
{
}

void NcAsyncActionROSNodeStop::run()
{
    Poco::Stopwatch sw;
    sw.start();
    STATE_START_ROS state = STATE_ROS_NODE_STOP;
    bool bfound = true;
    std::vector<std::string> masterNodes;
    while (true) {
        switch (state) {
            case STATE_ROS_NODE_STOP:  // stop navi
                if (ros_node_id_.find("nc_navigator") != std::string::npos) {
                    stopNavigation();
                    state = STATE_ROS_NODE_STOP_WAIT;
                    LOG_TRACE("start %s Node", ros_node_id_.c_str());
                }
                else if (ros_node_id_.find("answer_node") != std::string::npos) {
                    stopMapping();
                    state = STATE_ROS_NODE_STOP_WAIT;
                    LOG_TRACE("start %s Node", ros_node_id_.c_str());
                }
                else {
                    state = STATE_ROS_NODE_STOP_DONE;
                    LOG_TRACE("start %s Node", ros_node_id_.c_str());
                }
                break;
            case STATE_ROS_NODE_STOP_WAIT:  // check node kill
            {
                masterNodes.clear();
                auto not_npos = std::string::npos;
                ros::master::getNodes(masterNodes);
                for (auto ros_node : masterNodes) {
                    not_npos = ros_node.find(ros_node_id_);
                    if (not_npos != std::string::npos) {
                        bfound = true;
                        break;
                    }
                    else {
                        bfound = false;
                    }
                }
                // auto it = std::find(masterNodes.begin(), masterNodes.end(), "/" + ros_node_id_);
                // if (it != masterNodes.end())
                //     bfound = true;
                // else
                //     bfound = false;

                if (!bfound) {
                    state = STATE_ROS_NODE_STOP_DONE;
                    LOG_TRACE("Stop Complete %s Node", ros_node_id_.c_str());
                }
            } break;
            case STATE_ROS_NODE_STOP_DONE:
            {
                LOG_TRACE("Started %s Node", ros_node_id_.c_str());
            } break;
        }

        if (state == STATE_ROS_NODE_STOP_DONE) {
            Object response, emptyData;
            response.set("uuid", uuid_);
            response.set("id", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID());
            response.set("result", "success");
            response.set("data", emptyData);

            if (ros_node_id_.compare("slam_node") == 0) {
                InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->setSLAM(false);
                InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY)->setStatus("idle");
            }

            std::ostringstream oss;
            response.stringify(oss);
            MessageBroker::instance().publish(
                NcBrainMessage::MESSAGE_ROBOT_RESPONSE + InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(),
                oss.str());
            break;
        }
        else if (sw.elapsedSeconds() > 10) {
            LOG_TRACE("Stop %s Node Fail ", ros_node_id_.c_str());
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
