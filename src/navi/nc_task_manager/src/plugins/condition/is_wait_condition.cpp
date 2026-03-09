#include "nc_task_manager/nc_task_manager_pch.h"

#include <core_msgs/HacsNode.h>
#include <core_msgs/HacsNodeList.h>
#include <nc_task_manager/plugins/condition/is_wait_condition.h>
#include <std_msgs/String.h>
#include <core_msgs/TaskAlarm.h>

using namespace NaviFra;
IsWaitCondition::IsWaitCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    ros::NodeHandle nh;
    task_alarm_ = nh.advertise<core_msgs::TaskAlarm>("nc_task_manager/task_alarm", 1, false);
}

BT::NodeStatus IsWaitCondition::tick()
{
    Task current_task;
    std_msgs::String msg;
    if (getInput("current_task", current_task)) {
        if (current_task.type() == TYPE.WAIT) {
            core_msgs::TaskAlarm alarm;
            alarm.alarm = ALARM.START;
            alarm.uuid = current_task.uuid();
            alarm.type = current_task.type();
            NLOG(info) << "current_task :" << current_task.type();

            task_alarm_.publish(alarm);
            config().blackboard->set("wait_success", true);
            NLOG(info) << "task alarm start";

            return BT::NodeStatus::SUCCESS;
        }
        else {
            return BT::NodeStatus::FAILURE;
        }
    }
    return BT::NodeStatus::FAILURE;
}

BT::PortsList IsWaitCondition::providedPorts()
{
    return {BT::InputPort<Task>("current_task"),  BT::OutputPort<bool>("wait_success")};
}