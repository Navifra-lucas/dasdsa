#include "nc_task_manager/nc_task_manager_pch.h"
#include <nc_task_manager/plugins/condition/is_charging_condition.h>

#include <task_msgs/Charging.h>
#include <core_msgs/TaskAlarm.h>

using namespace NaviFra;
IsChargingCondition::IsChargingCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    ros::NodeHandle nh;
    send_charging_ = nh.advertise<task_msgs::Charging>("/nc_task_manager/charging", 10, false);
    task_alarm_ = nh.advertise<core_msgs::TaskAlarm>("nc_task_manager/task_alarm", 1, false);
}

BT::NodeStatus IsChargingCondition::tick()
{
    Task current_task;
    task_msgs::Charging charging;
    if (getInput("current_task", current_task)) {
        if (current_task.type() == TYPE.CHARGING || current_task.type() == TYPE.UNCHARGING) {
            int charger_id = current_task.charger_id();
            if (current_task.type() == TYPE.CHARGING) {
                // NLOG(info) << "TYPE.CHARGING";
                charging.mode = "charging";

                core_msgs::TaskAlarm alarm;
                alarm.alarm = ALARM.START;
                alarm.uuid = current_task.uuid();
                alarm.type = current_task.type();
                task_alarm_.publish(alarm);

                NLOG(info) << "Send To charging Command";
            }
            else if (current_task.type() == TYPE.UNCHARGING) {
                // NLOG(info) << "TYPE.UNCHARGING";
                charging.mode = "uncharging";

                core_msgs::TaskAlarm alarm;
                alarm.alarm = ALARM.START;
                alarm.uuid = current_task.uuid();
                alarm.type = current_task.type();
                task_alarm_.publish(alarm);

                NLOG(info) << "Send To uncharging Command";
            }
            charging.id = charger_id;
            send_charging_.publish(charging);
            return BT::NodeStatus::SUCCESS;
        }
        else {
            return BT::NodeStatus::FAILURE;
        }
    }
    return BT::NodeStatus::FAILURE;
}

BT::PortsList IsChargingCondition::providedPorts()
{
    return {BT::InputPort<Task>("current_task")};
}