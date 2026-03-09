#include "nc_task_manager/nc_task_manager_pch.h"

#include <core_msgs/TaskAlarm.h>
#include <nc_task_manager/plugins/condition/action_reached_condition.h>

using namespace NaviFra;
ActionReachedCondition::ActionReachedCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    ros::NodeHandle nh;
    task_alarm_ = nh.advertise<core_msgs::TaskAlarm>("nc_task_manager/task_alarm", 1, false);
}

BT::NodeStatus ActionReachedCondition::tick()
{
    bool actionReached;
    static bool b_action_check = false;
    if (getInput("action_reached", actionReached)){
        if(actionReached) {
            NLOG(info) << "Action Reached";
            config().blackboard->set("action_reached", false);
            Task current_task;
            if (getInput("current_task", current_task)) {
                // core_msgs::TaskAlarm alarm;
                // alarm.alarm = ALARM.DONE;
                // alarm.uuid = current_task.uuid();
                // task_alarm_.publish(alarm);
                NLOG(info) << "Action Reached";
            }
            updateTask();
            b_action_check = false;
            return BT::NodeStatus::SUCCESS;
        }
        else {
            if(!b_action_check) {
                tp_action_ = std::chrono::system_clock::now();
                b_action_check = true;
            }
            std::chrono::duration<double> sec = std::chrono::system_clock::now() - tp_action_;
            if(sec.count() > 0.5) {
                config().blackboard->set("action_reached", true);
            }
            return BT::NodeStatus::RUNNING;
        }
    }
    return BT::NodeStatus::RUNNING;
}

BT::PortsList ActionReachedCondition::providedPorts()
{
    return {
        BT::InputPort<bool>("action_reached"), 
        BT::InputPort<Task>("current_task"), 
        BT::InputPort<BT::SharedQueue<Task>>("tasks"),

        BT::OutputPort<bool>("action_reached"), 
        BT::OutputPort<Task>("current_task"), 
        BT::OutputPort<BT::SharedQueue<Task>>("tasks")
    };
}

void ActionReachedCondition::updateTask()
{
    try {
        BT::SharedQueue<Task> tasks;
        if (getInput("tasks", tasks) && tasks->size() > 0) {
            NLOG(info) << "ActionReached Update task";
            config().blackboard->set("previous_task", tasks->at(0));
            tasks->pop_front();
            if (tasks->size() > 0) {
                config().blackboard->set("current_task", tasks->at(0));
                config().blackboard->set("tasks", tasks);
                // core_msgs::TaskAlarm alarm;
                // alarm.alarm = ALARM.START;

                // alarm.uuid = tasks->at(0).uuid();
                // task_alarm_.publish(alarm);
            }
            else {
                config().blackboard->set("tasks", tasks);
                Task empty_task;
                config().blackboard->set("current_task", empty_task);
            }
        }
    }
    catch (std::exception& ex) {
        NLOG(error) << ex.what();
    }
}