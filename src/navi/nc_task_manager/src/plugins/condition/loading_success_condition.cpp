#include "nc_task_manager/nc_task_manager_pch.h"

#include <core_msgs/TaskAlarm.h>
#include <nc_task_manager/plugins/condition/loading_success_condition.h>
#include <std_msgs/String.h>

using namespace NaviFra;
LoadingSuccessCondition::LoadingSuccessCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    ros::NodeHandle nh;
    task_alarm_ = nh.advertise<core_msgs::TaskAlarm>("nc_task_manager/task_alarm", 1, false);
}

BT::NodeStatus LoadingSuccessCondition::tick()
{
    std::string loading_received, unloading_received;
    std::string lift_up, lift_down;

    if (getInput("loading_received", loading_received)) {
        if (loading_received == "true") {
            Task current_task;
            if (getInput("current_task", current_task)) {
                core_msgs::TaskAlarm alarm;
                alarm.alarm = ALARM.DONE;
                alarm.uuid = current_task.uuid();
                alarm.type = current_task.type();
                task_alarm_.publish(alarm);

                NLOG(info) << "LoadingSuccess DONE (LOAD)";
            }

            config().blackboard->set("loading_received", "false");
            updateTask();
            return BT::NodeStatus::SUCCESS;
        }
        else if (loading_received == "error") {
            Task current_task;
            if (getInput("current_task", current_task)) {
                core_msgs::TaskAlarm alarm;
                alarm.alarm = ALARM.ERROR;
                alarm.uuid = current_task.uuid();
                alarm.type = current_task.type();
                task_alarm_.publish(alarm);

                NLOG(info) << "LoadingSuccess ERROR (LOAD)";
            }

            config().blackboard->set("loading_received", "false");
            updateTask();
            return BT::NodeStatus::SUCCESS;
        }
    }

    // 언로딩 성공 체크
    if (getInput("unloading_received", unloading_received)) {
        if (unloading_received == "true") {
            Task current_task;
            if (getInput("current_task", current_task)) {
                core_msgs::TaskAlarm alarm;
                alarm.alarm = ALARM.DONE;
                alarm.uuid = current_task.uuid();
                alarm.type = current_task.type();
                task_alarm_.publish(alarm);

                NLOG(info) << "LoadingSuccess DONE (UNLOAD)";
            }

            config().blackboard->set("unloading_received", "false");
            updateTask();
            return BT::NodeStatus::SUCCESS;
        }
        else if (unloading_received == "error") {
            Task current_task;
            if (getInput("current_task", current_task)) {
                core_msgs::TaskAlarm alarm;
                alarm.alarm = ALARM.ERROR;
                alarm.uuid = current_task.uuid();
                alarm.type = current_task.type();
                task_alarm_.publish(alarm);

                NLOG(info) << "LoadingSuccess ERROR (UNLOAD)";
            }

            config().blackboard->set("unloading_received", "false");
            updateTask();
            return BT::NodeStatus::SUCCESS;
        }
    }

    return BT::NodeStatus::RUNNING;
}

BT::PortsList LoadingSuccessCondition::providedPorts()
{
    return {
        BT::InputPort<Task>("current_task"),
        BT::InputPort<BT::SharedQueue<Task>>("tasks"),
        BT::InputPort<std::string>("loading_received"),
        BT::InputPort<std::string>("unloading_received"),
        BT::InputPort<std::string>("lift_up"),
        BT::InputPort<std::string>("lift_down"),
        BT::OutputPort<std::string>("loading_received"),
        BT::OutputPort<std::string>("unloading_received"),
        BT::OutputPort<Task>("current_task"),
        BT::OutputPort<BT::SharedQueue<Task>>("tasks")};
}

void LoadingSuccessCondition::updateTask()
{
    try {
        BT::SharedQueue<Task> tasks;
        if (getInput("tasks", tasks) && tasks->size() > 0) {
            tasks->pop_front();
            if (tasks->size() > 0) {
                config().blackboard->set("current_task", tasks->at(0));
                config().blackboard->set("tasks", tasks);

                NLOG(info) << "send_loading_ publish";
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