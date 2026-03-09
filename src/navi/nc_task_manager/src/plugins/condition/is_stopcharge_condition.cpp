#include "nc_task_manager/nc_task_manager_pch.h"

#include <core_msgs/TaskAlarm.h>
#include <nc_task_manager/plugins/condition/is_stopcharge_condition.h>
#include <task_msgs/Charging.h>

using namespace NaviFra;
IsStopChargeCondition::IsStopChargeCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    ros::NodeHandle nh;
    task_alarm_ = nh.advertise<core_msgs::TaskAlarm>("nc_task_manager/task_alarm", 1, false);
    send_stopcharge_ = nh.advertise<task_msgs::Charging>("nc_task_manager/charging", 1, false);  // 변경필요..
}

BT::NodeStatus IsStopChargeCondition::tick()
{
    Task current_task;
    bool b_uncharge_received, b_uncharge_trigger;

    if (!getInput("current_task", current_task)) {
        NLOG(info) << "current_task input missing.";
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("charge_stop_received", b_uncharge_received)) {
        NLOG(info) << "charge_stop_received input missing.";
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("charge_stop_trigger", b_uncharge_trigger)) {
        NLOG(info) << "charge_stop_trigger input missing.";
        return BT::NodeStatus::FAILURE;
    }

    if (b_uncharge_received) {
        return BT::NodeStatus::SUCCESS;
    }

    if (b_uncharge_trigger) {
        task_msgs::Charging msg;
        msg.mode = "uncharging";
        send_stopcharge_.publish(msg);
        config().blackboard->set("charge_stop_trigger", false);
        NLOG(info) << "Publish Stopcharge Command";
    }

    return BT::NodeStatus::FAILURE;
}

BT::PortsList IsStopChargeCondition::providedPorts()
{
    return {
        BT::InputPort<Task>("current_task"), BT::InputPort<bool>("charge_stop_received"), BT::InputPort<bool>("charge_stop_trigger"),
        BT::OutputPort<bool>("charge_stop_trigger")};
}
