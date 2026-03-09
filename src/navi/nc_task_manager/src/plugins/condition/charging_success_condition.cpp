#include "nc_task_manager/nc_task_manager_pch.h"

#include <core_msgs/TaskAlarm.h>
#include <nc_task_manager/plugins/condition/charging_success_condition.h>
#include <std_msgs/String.h>

using namespace NaviFra;
ChargingSuccessCondition::ChargingSuccessCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    ros::NodeHandle nh;
    task_alarm_ = nh.advertise<core_msgs::TaskAlarm>("nc_task_manager/task_alarm", 1, false);
}

BT::NodeStatus ChargingSuccessCondition::tick()
{
    std::string charge_start_received;
    if (getInput("charge_start_received", charge_start_received)) {
        if(charge_start_received == "true") {
            config().blackboard->set("charge_start_received", "false");
            Task current_task;
            if (getInput("current_task", current_task)) {
                core_msgs::TaskAlarm alarm;
                alarm.alarm = ALARM.DONE;
                alarm.uuid = current_task.uuid();
                alarm.type = current_task.type();
                task_alarm_.publish(alarm);
                NLOG(info) << "ChargingSuccess Received";
            }
            updateTask();
            return BT::NodeStatus::SUCCESS;
        }
        else if (charge_start_received == "error") {
            Task current_task;
            if (getInput("current_task", current_task)) {
                core_msgs::TaskAlarm alarm;
                alarm.alarm = ALARM.ERROR;
                alarm.uuid = current_task.uuid();
                alarm.type = current_task.type();
                task_alarm_.publish(alarm);
                NLOG(info) << "ChargingSuccess Error";
            }
            updateTask();
            return BT::NodeStatus::SUCCESS;
        }
    }
    return BT::NodeStatus::RUNNING;
}

BT::PortsList ChargingSuccessCondition::providedPorts()
{
    return {
        BT::InputPort<Task>("current_task"),
        BT::InputPort<BT::SharedQueue<Task>>("tasks"),
        BT::InputPort<std::string>("charge_start_received"),
        BT::OutputPort<std::string>("charge_start_received"),
        BT::OutputPort<Task>("current_task"),
        BT::OutputPort<BT::SharedQueue<Task>>("tasks")};
}

void ChargingSuccessCondition::updateTask()
{
    try {
        BT::SharedQueue<Task> tasks;
        if (getInput("tasks", tasks) && tasks->size() > 0) {
            tasks->pop_front();
            if (tasks->size() > 0) {
                config().blackboard->set("current_task", tasks->at(0));
                config().blackboard->set("tasks", tasks);
                NLOG(info) << "send_charging_ publish";
            }
            else {
                config().blackboard->set("tasks", tasks);
            }
        }
    }
    catch (std::exception& ex) {
        NLOG(error) << ex.what();
    }
}