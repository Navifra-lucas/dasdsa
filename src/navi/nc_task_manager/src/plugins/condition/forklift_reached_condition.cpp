#include "nc_task_manager/nc_task_manager_pch.h"

#include <core_msgs/TaskAlarm.h>
#include <nc_task_manager/plugins/condition/forklift_reached_condition.h>

using namespace NaviFra;
ForkLiftReachedCondition::ForkLiftReachedCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    ros::NodeHandle nh;
    task_alarm_ = nh.advertise<core_msgs::TaskAlarm>("nc_task_manager/task_alarm", 1, false);
}

BT::NodeStatus ForkLiftReachedCondition::tick()
{
    Task current_task;
    if (getInput("current_task", current_task)) {
        bool fork_lift_reached = false, wingbody_perception_reached = false, b_need_success = false;
        if (getInput("fork_lift_reached", fork_lift_reached) && fork_lift_reached) {
            NLOG(info) << "forklift reached : " << fork_lift_reached;
        }

        if (getInput("wingbody_perception_reached", wingbody_perception_reached) && wingbody_perception_reached) {
            NLOG(info) << "wingbody_perception_reached : " << wingbody_perception_reached;
        }

        if (current_task.forkLiftData().n_drive_type == 6) {
            if (fork_lift_reached && wingbody_perception_reached) {
                b_need_success = true;
            }
        }
        else {
            if (fork_lift_reached) {
                b_need_success = true;
            }
        }

        if (b_need_success) {
            core_msgs::TaskAlarm alarm;
            alarm.alarm = ALARM.DONE;
            alarm.uuid = current_task.uuid();
            task_alarm_.publish(alarm);
            NLOG(info) << "ForkLift Reached";
            updateTask();
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::RUNNING;
}

BT::PortsList ForkLiftReachedCondition::providedPorts()
{
    return {BT::InputPort<bool>("fork_lift_reached"),
            BT::InputPort<Task>("current_task"),
            BT::InputPort<bool>("wingbody_perception_reached"),
            BT::InputPort<BT::SharedQueue<Task>>("tasks"),
            BT::OutputPort<Task>("current_task"),
            BT::OutputPort<BT::SharedQueue<Task>>("tasks")};
}

void ForkLiftReachedCondition::updateTask()
{
    try {
        BT::SharedQueue<Task> tasks;
        if (getInput("tasks", tasks) && tasks->size() > 0) {
            NLOG(info) << "ForkLiftReached Update task";
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