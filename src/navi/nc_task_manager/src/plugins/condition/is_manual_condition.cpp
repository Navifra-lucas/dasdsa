#include "nc_task_manager/nc_task_manager_pch.h"

#include <nc_task_manager/plugins/condition/is_manual_condition.h>
#include <std_msgs/String.h>

using namespace NaviFra;
IsManualCondition::IsManualCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    ros::NodeHandle nh;
    publiser_ = nh.advertise<std_msgs::String>("hwdriver/task", 10, false);
}

BT::NodeStatus IsManualCondition::tick()
{
    Task current_task;
    if (getInput("current_task", current_task)) {
        if (current_task.type() == TYPE.MANUAL) {
            if (current_task.uuid() != current_task_.uuid()) {
                current_task_ = current_task;
                std_msgs::String msg;
                msg.data = current_task.type();
                publiser_.publish(msg);
                NLOG(info) << "Started Manaul";
            }

            return BT::NodeStatus::SUCCESS;
        }
        else {
            return BT::NodeStatus::FAILURE;
        }
    }

    return BT::NodeStatus::FAILURE;
}

BT::PortsList IsManualCondition::providedPorts()
{
    return {BT::InputPort<Task>("current_task")};
}
