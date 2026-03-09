#include "nc_task_manager/nc_task_manager_pch.h"

#include <core_msgs/TaskAlarm.h>
#include <nc_task_manager/plugins/condition/is_turn_condition.h>
#include <std_msgs/String.h>

using namespace NaviFra;
IsTurnCondition::IsTurnCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    ros::NodeHandle nh;
    send_turn_ = nh.advertise<std_msgs::String>("/navifra/angle_turn", 1, false);
    task_alarm_ = nh.advertise<core_msgs::TaskAlarm>("nc_task_manager/task_alarm", 1, false);
}

BT::NodeStatus IsTurnCondition::tick()
{
    Task current_task;
    if (getInput("current_task", current_task)) {
        if (current_task.type() == TYPE.TURN) {
            std::string current_node;
            if (getInput("current_node", current_node)) {
                if (current_task.endNode() == current_node) {
                    std_msgs::String angle_turn_msg;
                    angle_turn_msg.data = std::to_string(current_task.finishAngle());
                    send_turn_.publish(angle_turn_msg);

                    return BT::NodeStatus::SUCCESS;
                }
                else {
                    return BT::NodeStatus::FAILURE;
                }
            }
        }
        else {
            return BT::NodeStatus::FAILURE;
        }
    }

    return BT::NodeStatus::FAILURE;
}

BT::PortsList IsTurnCondition::providedPorts()
{
    return {
        BT::InputPort<Task>("current_task"),
        BT::InputPort<std::string>("current_node"),
    };
}
