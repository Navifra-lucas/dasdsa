#include "nc_task_manager/nc_task_manager_pch.h"

#include <core_msgs/HacsNode.h>
#include <core_msgs/HacsNodeList.h>
#include <nc_task_manager/plugins/condition/is_checking_condition.h>
#include <std_msgs/String.h>
#include "std_msgs/Int32.h"
#include <core_msgs/TaskAlarm.h>
#include <std_msgs/Int16.h>

using namespace NaviFra;
IsCheckingCondition::IsCheckingCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    ros::NodeHandle nh;
    send_checking_ = nh.advertise<std_msgs::Int32>("/roi_setting", 10, false);
    task_alarm_ = nh.advertise<core_msgs::TaskAlarm>("nc_task_manager/task_alarm", 1, false);
    send_speaker_ = nh.advertise<std_msgs::Int16>("speaker_cmd", 1, false);
}
BT::NodeStatus IsCheckingCondition::tick()
{
    Task current_task;
    std_msgs::Int32 msg;
    std_msgs::Int16 mode;
    if (getInput("current_task", current_task)) {
        if (current_task.type() == TYPE.PICKUP_CHECKING || current_task.type() == TYPE.RETURN_CHECKING || current_task.type() == TYPE.TO_CHECKING) {
            msg.data = current_task.count();
            
            core_msgs::TaskAlarm alarm;
            alarm.alarm = ALARM.START;
            alarm.uuid = current_task.uuid();
            alarm.type = current_task.type();
            task_alarm_.publish(alarm);
            NLOG(info) << "Send To checking Command";
            send_checking_.publish(msg);
            mode.data = 4;
            NLOG(info) << "speaker : " <<mode.data;
            send_speaker_.publish(mode);
            return BT::NodeStatus::SUCCESS;
        }
        else {
            return BT::NodeStatus::FAILURE;
        }
    }
    return BT::NodeStatus::FAILURE;
}

BT::PortsList IsCheckingCondition::providedPorts()
{
    return {BT::InputPort<Task>("current_task")};
}