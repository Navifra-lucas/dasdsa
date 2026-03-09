#include "nc_task_manager/nc_task_manager_pch.h"

#include <core_msgs/TaskAlarm.h>
#include <nc_task_manager/plugins/condition/wait_for_manual_end_condition.h>

using namespace NaviFra;
WaitForManualCondition::WaitForManualCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    ros::NodeHandle nh;
    task_alarm_ = nh.advertise<core_msgs::TaskAlarm>("nc_task_manager/task_alarm", 1, false);
}

BT::NodeStatus WaitForManualCondition::tick()
{
    bool manualDoneReceived;
    if (getInput("driverDoneReceived", manualDoneReceived) && manualDoneReceived) {
        setOutput("driverDoneReceived", false);
        Task current_task;
        if (getInput("current_task", current_task)) {
            core_msgs::TaskAlarm alarm;
            alarm.alarm = ALARM.DONE;
            alarm.uuid = current_task.uuid();
            task_alarm_.publish(alarm);
            NLOG(info) << "Manual done received";
        }
        updateTask();
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

BT::PortsList WaitForManualCondition::providedPorts()
{
    return {BT::InputPort<Task>("current_task"),       BT::InputPort<BT::SharedQueue<Task>>("tasks"),
            BT::InputPort<bool>("driverDoneReceived"), BT::OutputPort<bool>("driverDoneReceived"),
            BT::OutputPort<Task>("current_task"),      BT::OutputPort<BT::SharedQueue<Task>>("tasks")};
}

void WaitForManualCondition::updateTask()
{
    try {
        BT::SharedQueue<Task> tasks;
        if (getInput("tasks", tasks) && tasks->size() > 0) {
            NLOG(info) << "GoalReached Update task";
            tasks->pop_front();
            if (tasks->size() > 0) {
                config().blackboard->set("current_task", tasks->at(0));
                config().blackboard->set("tasks", tasks);
                core_msgs::TaskAlarm alarm;
                alarm.alarm = ALARM.START;
                alarm.uuid = tasks->at(0).uuid();
                task_alarm_.publish(alarm);
            }
            else {
                // config().blackboard->set("current_task", nullptr);
                config().blackboard->set("tasks", tasks);
            }
        }
    }
    catch (std::exception& ex) {
        NLOG(error) << ex.what();
    }
}