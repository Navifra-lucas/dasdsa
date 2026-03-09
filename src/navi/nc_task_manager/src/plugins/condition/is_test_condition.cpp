#include "nc_task_manager/nc_task_manager_pch.h"

#include <core_msgs/TaskAlarm.h>
#include <nc_task_manager/plugins/condition/is_test_condition.h>
#include <std_msgs/String.h>

using namespace NaviFra;
IsTestCondition::IsTestCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    ros::NodeHandle nh;
    task_alarm_ = nh.advertise<core_msgs::TaskAlarm>("nc_task_manager/task_alarm", 1, false);
}

BT::NodeStatus IsTestCondition::tick()
{
    Task current_task;
    if (getInput("current_task", current_task)) {
        if (current_task.type() == "start_charging") {
            return BT::NodeStatus::SUCCESS;
        }
        else {
            return BT::NodeStatus::FAILURE;
        }
    }

    return BT::NodeStatus::FAILURE;
}

BT::PortsList IsTestCondition::providedPorts()
{
    return {
        BT::InputPort<Task>("current_task"),
        BT::InputPort<std::string>("current_node"),
    };
}
