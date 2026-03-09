#include "core_astra/core_astra_node.h"

#include "core_astra/zmq_handler.h"

#include <ros/duration.h>
#include <ros/ros.h>
#include <ros/timer.h>
#include <std_msgs/String.h>

using namespace NaviFra;

CoreAstraNode::CoreAstraNode()
{
    ros_pub_ = nh_.advertise<std_msgs::String>("/agent_status", 100);
}

CoreAstraNode::~CoreAstraNode()
{
    // 소멸자에서 특별한 작업은 필요하지 않음
}
