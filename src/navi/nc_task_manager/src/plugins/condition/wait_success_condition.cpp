#include "nc_task_manager/nc_task_manager_pch.h"

#include <core_msgs/TaskAlarm.h>
#include <nc_task_manager/plugins/condition/wait_success_condition.h>
#include <std_msgs/String.h>

using namespace NaviFra;
WaitSuccessCondition::WaitSuccessCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    ros::NodeHandle nh;
    task_alarm_ = nh.advertise<core_msgs::TaskAlarm>("nc_task_manager/task_alarm", 1, false);
    tp_action_ = std::chrono::system_clock::now();
}

void WaitSuccessCondition::resetInternalState()
{
    b_action_check_ = false;
    wsc_ = false;

    wait_seconds_ = 0;
    s_wait_id_.clear();

    tp_action_ = std::chrono::system_clock::now();

    NLOG(info) << "[WAIT] Internal state reset";
}

BT::NodeStatus WaitSuccessCondition::tick()
{
    Task current_task;
    if (!getInput("current_task", current_task)) {
        wait_seconds_ = 0;
        s_wait_id_.clear();
        return BT::NodeStatus::RUNNING;
    }

    bool cancel_flag = false;
    if (config().blackboard->get("wait_cancel", cancel_flag) && cancel_flag) {
        NLOG(info) << "[WAIT] Reset triggered by external CANCEL";
        resetInternalState();
        config().blackboard->set("wait_cancel", false);
    }

    bool b_tmt_wsc = false;
    if (getInput("wait_success", b_tmt_wsc) && b_tmt_wsc == true) {
        NLOG(info) << "[WAIT] Cancel received, resetting timer without triggering wsc (tmt)";

        tp_action_ = std::chrono::system_clock::now();
        b_action_check_ = false;

        config().blackboard->set("wait_success", false);
    }

    if (wsc_) {
        NLOG(info) << "wsc is true";

        core_msgs::TaskAlarm alarm;
        alarm.alarm = ALARM.DONE;
        alarm.uuid = current_task.uuid();
        alarm.type = current_task.type();
        task_alarm_.publish(alarm);

        NLOG(info) << "WaitSuccessCondition: DONE alarm published (uuid=" << current_task.uuid() << ")";

        updateTask();
        wsc_ = false;

        return BT::NodeStatus::SUCCESS;
    }

    if (!b_action_check_) {
        wait_seconds_ = current_task.wait_seconds();
        s_wait_id_ = current_task.wait_id();

        NLOG(info) << "[WAIT] Init wait_id=" << s_wait_id_ << " wait_seconds=" << wait_seconds_;
        tp_action_ = std::chrono::system_clock::now();

        b_action_check_ = true;
    }

    std::string s_comp_wait_id;
    if (getInput("wait_id", s_comp_wait_id)) {
        if (s_comp_wait_id == s_wait_id_ && !s_wait_id_.empty()) {
            NLOG(info) << "[WAIT] Triggered by ID: " << s_wait_id_;
            config().blackboard->set("wait_id", "");
            wsc_ = true;
        }
    }

    auto elapsed = std::chrono::system_clock::now() - tp_action_;
    double sec = std::chrono::duration<double>(elapsed).count();

    if (wait_seconds_ != 0 && sec >= wait_seconds_) {
        NLOG(info) << "[WAIT] Triggered by TIMER (elapsed=" << sec << "s)";
        wsc_ = true;
    }

    return BT::NodeStatus::RUNNING;
}

BT::PortsList WaitSuccessCondition::providedPorts()
{
    return {BT::InputPort<Task>("current_task"),   BT::InputPort<BT::SharedQueue<Task>>("tasks"),  BT::InputPort<bool>("wait_success"),
            BT::InputPort<std::string>("wait_id"), BT::OutputPort<bool>("wait_success"),           BT::OutputPort<std::string>("wait_id"),
            BT::OutputPort<Task>("current_task"),  BT::OutputPort<BT::SharedQueue<Task>>("tasks"), BT::InputPort<bool>("wait_cancel")};
}

void WaitSuccessCondition::updateTask()
{
    try {
        BT::SharedQueue<Task> tasks;
        if (getInput("tasks", tasks) && tasks->size() > 0) {
            tasks->pop_front();
            if (tasks->size() > 0) {
                config().blackboard->set("current_task", tasks->at(0));
                config().blackboard->set("tasks", tasks);
                core_msgs::TaskAlarm alarm;
                alarm.alarm = ALARM.START;
                alarm.uuid = tasks->at(0).uuid();

                task_alarm_.publish(alarm);
                NLOG(info) << "WaitSuccessCondition task alarm start";
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