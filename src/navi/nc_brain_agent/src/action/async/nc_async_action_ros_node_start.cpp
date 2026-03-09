
#include "nc_brain_agent/nc_brain_agent.h"

#include <Poco/Stopwatch.h>
#include <core_agent/core/navicore.h>
#include <core_agent/data/robot_info.h>
#include <core_agent/data/robot_status.h>
#include <core_agent/message/message_broker.h>
#include <nc_brain_agent/action/async/nc_async_action_ros_node_start.h>
#include <nc_brain_agent/message/nc_brain_message.h>
#include <ros/master.h>
#include <ros/ros.h>

using namespace NaviFra;
using Poco::JSON::Object;

enum STATE_START_ROS
{
    STATE_ROS_NODE_START = 0,
    STATE_ROS_NODE_START_WAIT,
    STATE_ROS_NODE_START_DONE
};

NcAsyncActionROSNodeStart::NcAsyncActionROSNodeStart(std::string uuid, std::string ros_node_id)
    : uuid_(uuid)
    , ros_node_id_(ros_node_id)
{
}

NcAsyncActionROSNodeStart::~NcAsyncActionROSNodeStart()
{
}

void NcAsyncActionROSNodeStart::run()
{
    Poco::Stopwatch sw;
    sw.start();
    STATE_START_ROS state = STATE_ROS_NODE_START;
    bool bfound = false;
    std::vector<std::string> masterNodes;
    while (true) {
        switch (state) {
            case STATE_ROS_NODE_START:  // stop navi
                if (ros_node_id_.find("nc_localizer_ros") != std::string::npos) {
                    startNavigation();
                    state = STATE_ROS_NODE_START_WAIT;
                    LOG_TRACE("start %s Node", ros_node_id_.c_str());
                }
                // else if (ros_node_id_.find("slam_node") != std::string::npos) {
                //     startMapping();
                //     state = STATE_ROS_NODE_START_WAIT;
                //     LOG_TRACE("start %s Node", ros_node_id_.c_str());
                // }
                else {
                    state = STATE_ROS_NODE_START_DONE;
                    LOG_TRACE("start %s Node", ros_node_id_.c_str());
                }
                break;
            case STATE_ROS_NODE_START_WAIT:  // check node kill
            {
                masterNodes.clear();
                ros::master::getNodes(masterNodes);
                auto not_npos = std::string::npos;
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

                if (bfound) {
                    state = STATE_ROS_NODE_START_DONE;
                    LOG_TRACE("Start Complete %s Node", ros_node_id_.c_str());
                }
            } break;
            case STATE_ROS_NODE_START_DONE:
            {
                state = STATE_ROS_NODE_START_DONE;
                LOG_TRACE("Started %s Node", ros_node_id_.c_str());
            } break;
        }

        if (state == STATE_ROS_NODE_START_DONE) {
            Object response, emptyData;
            response.set("uuid", uuid_);
            response.set("id", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID());
            response.set("result", "success");
            response.set("data", emptyData);

            if (ros_node_id_.compare("slam_node") == 0) {
                InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->setSLAM(true);
                InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY)->setStatus("MAPPING");
            }

            std::ostringstream oss;
            response.stringify(oss);
            MessageBroker::instance().publish(
                NcBrainMessage::MESSAGE_ROBOT_RESPONSE + InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(),
                oss.str());
            break;
        }
        else if (sw.elapsedSeconds() > 10) {
            LOG_TRACE("Started %s Node Fail ", ros_node_id_.c_str());
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