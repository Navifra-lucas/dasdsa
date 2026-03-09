#include "nc_task_manager/nc_task_manager_pch.h"

#include <core_msgs/TaskAlarm.h>
#include <nc_task_manager/plugins/condition/is_action_condition.h>
#include <std_msgs/String.h>

using namespace NaviFra;
IsActionCondition::IsActionCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    ros::NodeHandle nh;
    task_alarm_ = nh.advertise<core_msgs::TaskAlarm>("nc_task_manager/task_alarm", 1, false);
}

BT::NodeStatus IsActionCondition::tick()
{
    Task current_task;
    if (getInput("current_task", current_task)) {
        if (current_task.type() == "done") {
            NLOG(info) << "Action Reached, done";
            return BT::NodeStatus::SUCCESS;
        }
        else {
            return BT::NodeStatus::FAILURE;
        }
    }

    return BT::NodeStatus::FAILURE;
}

BT::PortsList IsActionCondition::providedPorts()
{
    return {
        BT::InputPort<Task>("current_task"),
    };
}
