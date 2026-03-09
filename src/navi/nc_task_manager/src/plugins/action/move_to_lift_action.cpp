#include "nc_task_manager/nc_task_manager_pch.h"

#include <nc_task_manager/plugins/action/move_to_lift_action.h>

using namespace NaviFra;
MoveToLiftAction::MoveToLiftAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
}

BT::NodeStatus MoveToLiftAction::tick()
{
    return BT::NodeStatus::FAILURE;
}

BT::PortsList MoveToLiftAction::providedPorts()
{
    return {BT::InputPort<Task>("current_task"), BT::OutputPort<Task>("current_task")};
}