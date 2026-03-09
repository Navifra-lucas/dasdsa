#include "nc_task_manager/nc_task_manager_pch.h"
#include "std_msgs/Int32.h"

#include <core_msgs/ForkLift.h>
#include <core_msgs/TaskAlarm.h>
#include <nc_task_manager/plugins/condition/is_forklift_condition.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>

using namespace NaviFra;
IsForkLiftCondition::IsForkLiftCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    ros::NodeHandle nh;
    task_alarm_ = nh.advertise<core_msgs::TaskAlarm>("nc_task_manager/task_alarm", 1, false);
    fork_lift_ = nh.advertise<core_msgs::ForkLift>("nc_task_manager/fork_docking", 1, false);  // 임시 토픽
}
BT::NodeStatus IsForkLiftCondition::tick()
{
    Task current_task;
    if (getInput("current_task", current_task)) {
        if (current_task.type() == TYPE.FORKLIFT) {
            int drive_type = current_task.drive_type();
            Task::ForkLiftData data = current_task.forkLiftData();

            core_msgs::ForkLift fork_lift;

            fork_lift.s_current_node_id = data.s_current_node_id;
            fork_lift.s_target_node_id = data.s_target_node_id;

            fork_lift.f_current_x = data.f_current_x;
            fork_lift.f_current_y = data.f_current_y;
            fork_lift.f_current_deg = data.f_current_deg;
            fork_lift.f_target_x = data.f_target_x;
            fork_lift.f_target_y = data.f_target_y;
            fork_lift.f_target_deg = data.f_target_deg;

            fork_lift.n_rack_level = data.n_rack_level;
            fork_lift.n_target_level = data.n_target_level;
            fork_lift.n_target_height = data.n_target_height;
            fork_lift.n_rack_type = data.n_rack_type;
            fork_lift.n_pallet_type = data.n_pallet_type;

            fork_lift.n_drive_type = data.n_drive_type;  // drive_type 1 : up, -1 : down

            // if (fork_lift.n_drive_type != 1 && fork_lift.n_drive_type != -1)
            // {
            //     NLOG(error) << "Invalid Drive Type: " << fork_lift.n_drive_type;

            //     return BT::NodeStatus::FAILURE;
            // }

            fork_lift_.publish(fork_lift);

            NLOG(info) << "Send To ForkLift Command";

            return BT::NodeStatus::SUCCESS;
        }
        else {
            return BT::NodeStatus::FAILURE;
        }
    }
    return BT::NodeStatus::FAILURE;
}

BT::PortsList IsForkLiftCondition::providedPorts()
{
    return {BT::InputPort<Task>("current_task")};
}