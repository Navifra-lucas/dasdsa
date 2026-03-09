#include "nc_task_manager/nc_task_manager_pch.h"

#include <task_msgs/Loading.h>
#include <nc_task_manager/plugins/condition/is_loading_condition.h>
#include <std_msgs/String.h>
#include <core_msgs/TaskAlarm.h>

using namespace NaviFra;
IsLoadingCondition::IsLoadingCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    ros::NodeHandle nh;
    send_loading_ = nh.advertise<task_msgs::Loading>("/task_manager/lift_control", 10, false);
    task_alarm_ = nh.advertise<core_msgs::TaskAlarm>("nc_task_manager/task_alarm", 1, false);
}

BT::NodeStatus IsLoadingCondition::tick()
{
    Task current_task;
    task_msgs::Loading msg;
    if (getInput("current_task", current_task)) {
        auto level = current_task.level();
        if (current_task.type() == TYPE.LOADING || current_task.type() == TYPE.UNLOADING) {
            if (current_task.type() == TYPE.LOADING) {
                msg.type = "loading";
                msg.level = level;
                NLOG(info) << "Send To loading Command";
            }
            else if (current_task.type() == TYPE.UNLOADING) {
                msg.type = "unloading";
                msg.level = current_task.level();
                NLOG(info) << "Send To unloading Command";
            }
            NLOG(info) << "msg: " << msg;

            core_msgs::TaskAlarm alarm;
            alarm.alarm = ALARM.START;
            alarm.uuid = current_task.uuid();
            alarm.type = current_task.type();
            task_alarm_.publish(alarm);
            NLOG(info) << "Send To loading Command";
            
            send_loading_.publish(msg);
            return BT::NodeStatus::SUCCESS;
        }
        else {
            return BT::NodeStatus::FAILURE;
        }
    }
    return BT::NodeStatus::FAILURE;
}

BT::PortsList IsLoadingCondition::providedPorts()
{
    return {BT::InputPort<Task>("current_task")};
}