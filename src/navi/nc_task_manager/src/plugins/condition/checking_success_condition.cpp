#include "nc_task_manager/nc_task_manager_pch.h"

#include <core_msgs/TaskAlarm.h>
#include <nc_task_manager/plugins/condition/checking_success_condition.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>

using namespace NaviFra;
CheckingSuccessCondition::CheckingSuccessCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    ros::NodeHandle nh;
    task_alarm_ = nh.advertise<core_msgs::TaskAlarm>("nc_task_manager/task_alarm", 1, false);
    send_speaker_ = nh.advertise<std_msgs::Int16>("speaker_cmd", 1, false);

}

BT::NodeStatus CheckingSuccessCondition::tick()
{
    std::string checking_start_received;
    std::vector<int> object_in_roi_data;  // 값 초기화
    std_msgs::Int16 mode;

    if (getInput("checking_start_received", checking_start_received)) {
        if(checking_start_received == "true"){
            config().blackboard->set("checking_start_received", "false");
            Task current_task;
            if (getInput("current_task", current_task)) {

                core_msgs::TaskAlarm alarm;
                alarm.alarm = ALARM.DONE;
                alarm.uuid = current_task.uuid();
                alarm.type = current_task.type();
                auto keys = config().blackboard->getKeys();
                for (const auto& key : keys) {
                }
                
                if (config().blackboard->get("object_in_roi_data", object_in_roi_data)) {
                    NLOG(info) << "object_in_roi_data retrieved from blackboard, size: " << object_in_roi_data.size();
                    
                    std::ostringstream oss;
                    oss << "object_in_roi_data values: ";
                    for (const auto& value : object_in_roi_data) {
                        alarm.int_list.push_back(value);
                        oss << value << " ";
                    }
                    NLOG(info) << oss.str();
                } else {
                    NLOG(error) << "Failed to get object_in_roi_data from blackboard";
                }
                task_alarm_.publish(alarm);

                NLOG(info) << "Checking Success Received";
                mode.data = 0;
                NLOG(info) << "speaker : " <<mode.data;
                send_speaker_.publish(mode);
            }
            updateTask();
            return BT::NodeStatus::SUCCESS;
        }
        else if (checking_start_received == "error") {
            Task current_task;
            if (getInput("current_task", current_task)) {
                core_msgs::TaskAlarm alarm;
                alarm.alarm = ALARM.ERROR;
                alarm.uuid = current_task.uuid();
                alarm.type = current_task.type();
                task_alarm_.publish(alarm);
                mode.data = 0;
                NLOG(info) << "speaker : " <<mode.data;
                send_speaker_.publish(mode);
                NLOG(info) << "Checking Success Error";
            }
            updateTask();
            return BT::NodeStatus::SUCCESS;
        }
    }
    // object_in_roi_data = 0;  // RUNNING 상태일 때도 값 초기화
    return BT::NodeStatus::RUNNING;
}

BT::PortsList CheckingSuccessCondition::providedPorts()
{
    return {
        BT::InputPort<Task>("current_task"),          
        BT::InputPort<BT::SharedQueue<Task>>("tasks"),
        BT::InputPort<std::string>("checking_start_received"), 
        BT::OutputPort<std::string>("checking_start_received"),
        BT::OutputPort<Task>("current_task"),         
        BT::OutputPort<BT::SharedQueue<Task>>("tasks")};
}

void CheckingSuccessCondition::updateTask()
{
    try {
        BT::SharedQueue<Task> tasks;
        if (getInput("tasks", tasks) && tasks->size() > 0) {
            tasks->pop_front();
            if (tasks->size() > 0) {
                config().blackboard->set("current_task", tasks->at(0));
                config().blackboard->set("tasks", tasks);
    
                NLOG(info) << "send_checking_publish";
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