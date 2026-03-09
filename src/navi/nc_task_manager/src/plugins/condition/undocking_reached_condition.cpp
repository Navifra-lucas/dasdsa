#include "nc_task_manager/nc_task_manager_pch.h"

#include <core_msgs/TaskAlarm.h>
#include <nc_task_manager/plugins/condition/undocking_reached_condition.h>

using namespace NaviFra;
UnDockingReachedCondition::UnDockingReachedCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    ros::NodeHandle nh;
    task_alarm_ = nh.advertise<core_msgs::TaskAlarm>("nc_task_manager/task_alarm", 1, false);
    send_startcharge_ = nh.advertise<std_msgs::String>("output_command", 1, false);
}

BT::NodeStatus UnDockingReachedCondition::tick()
{
    std::string undocking_reached;
    std_msgs::String mode;
    if (getInput("undocking_reached", undocking_reached)) {
        if (undocking_reached == "true") {
            config().blackboard->set("undocking_reached", "false");
            Task current_task;
            NLOG(info) << undocking_reached;
            if (getInput("current_task", current_task)) {
                core_msgs::TaskAlarm alarm;
                NLOG(info) << undocking_reached;
                alarm.alarm = ALARM.DONE;
                alarm.uuid = current_task.uuid();
                alarm.type = current_task.type();
                task_alarm_.publish(alarm);
                NLOG(info) << "Docking Reached";
            }
            updateTask();
            return BT::NodeStatus::SUCCESS;
        }
        else if (undocking_reached == "error") {
            Task current_task;
            if (getInput("current_task", current_task)) {
                core_msgs::TaskAlarm alarm;
                alarm.alarm = ALARM.ERROR;
                alarm.uuid = current_task.uuid();
                alarm.type = current_task.type();
                task_alarm_.publish(alarm);
                NLOG(info) << "Docking Error";
            }
            updateTask();
            return BT::NodeStatus::SUCCESS;
        }
    }
    return BT::NodeStatus::RUNNING;
}

BT::PortsList UnDockingReachedCondition::providedPorts()
{
    return {
        BT::InputPort<std::string>("undocking_reached"), BT::InputPort<Task>("current_task"), BT::InputPort<BT::SharedQueue<Task>>("tasks"),
        BT::OutputPort<Task>("current_task"), BT::OutputPort<BT::SharedQueue<Task>>("tasks")};
}

void UnDockingReachedCondition::updateTask()
{
    try {
        BT::SharedQueue<Task> tasks;
        if (getInput("tasks", tasks) && tasks->size() > 0) {
            tasks->pop_front();
            if (tasks->size() > 0) {
                NLOG(info) << " 123123";
                config().blackboard->set("current_task", tasks->at(0));
                config().blackboard->set("tasks", tasks);
            }
            else {
                NLOG(info) << " 123123";

                // config().blackboard->set("current_task", nullptr);
                config().blackboard->set("tasks", tasks);
            }
        }
    }
    catch (std::exception& ex) {
        NLOG(error) << ex.what();
    }
}