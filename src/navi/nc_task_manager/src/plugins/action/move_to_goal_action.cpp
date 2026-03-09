#include "nc_task_manager/nc_task_manager_pch.h"

#include <move_msgs/CoreCommand.h>
#include <nc_task_manager/plugins/action/move_to_goal_action.h>
#include <nc_task_manager/topic/nc_task_manager_topic.h>
#include <task_msgs/Charging.h>

using namespace NaviFra;
MoveToGoalAction::MoveToGoalAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    ros::NodeHandle nh;
    sendLivePath_ = nh.advertise<move_msgs::CoreCommand>("navifra/live_path", 1, false);
    send_charging_ = nh.advertise<task_msgs::Charging>("/nc_task_manager/charging", 10, false);
}

BT::NodeStatus MoveToGoalAction::tick()
{
    bool updated_goals;
    Task current_task;
    getInput("current_task", current_task);
    getInput("updated_goals", updated_goals);

    if (updated_goals) {
        NLOG(info) << "MoveToGoalAction current_task.type() " << current_task.type();
        if (current_task.type() == TYPE.MOVE || current_task.type() == TYPE.UNDOCKING) {
            NLOG(info) << "MoveToGoalAction current_task.type() == " << current_task.type();
            move_msgs::CoreCommand goalist;
            if (getInput("goals", goalist)) {
                for (size_t i = 0; i < goalist.list_waypoints.size(); i++) {
                    NLOG(info) << boost::format("Send Goal Node Name [ %1% ]") % goalist.list_waypoints.at(i).s_name;
                }
                sendLivePath_.publish(goalist);

                config().blackboard->set("updated_goals", false);
                config().blackboard->set("goal_reached", false);
                return BT::NodeStatus::SUCCESS;
            }
            else {
                NLOG(error) << "can't getInput goals blackboard goals is empty";
                return BT::NodeStatus::FAILURE;
            }
        }
    }

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList MoveToGoalAction::providedPorts()
{
    return {
        BT::InputPort<Task>("current_task"), BT::InputPort<move_msgs::CoreCommand>("goals"), BT::InputPort<bool>("updated_goals"),
        BT::InputPort<bool>("goal_reached")};
}