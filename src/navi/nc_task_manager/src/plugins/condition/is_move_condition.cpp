#include "nc_task_manager/nc_task_manager_pch.h"
#include "pos/pos.hpp"

#include <core_msgs/TaskAlarm.h>
#include <move_msgs/CoreCommand.h>
#include <nc_task_manager/data/nc_task.h>
#include <nc_task_manager/plugins/condition/is_move_condition.h>
#include <std_msgs/String.h>

using namespace NaviFra;
IsMoveCondition::IsMoveCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    ros::NodeHandle nh;
    docking_state_pub_ = nh.advertise<std_msgs::String>("/docking_state", 1);
}

BT::NodeStatus IsMoveCondition::tick()
{
    static auto gen_time = std::chrono::steady_clock::now();

    Task current_task;
    bool b_arrive_align;
    bool b_uncharge_received;
    if (getInput("charge_stop_received", b_uncharge_received) && !b_uncharge_received) {
        return BT::NodeStatus::FAILURE;
    }

    if (getInput("current_task", current_task) && current_task.type() == TYPE.MOVE) {
        b_arrive_align = current_task.arrvieAlign();
        // config().blackboard->set("b_arrive_align", b_arrive_align);
        // NLOG(info) << "current_task.type() : " << current_task.type();
        bool need_update_goal = false;

        std::string s_robot_status = "";
        std::chrono::duration<double> sec_gen = std::chrono::steady_clock::now() - gen_time;
        if (getInput("s_robot_status", s_robot_status) && s_robot_status == "idle") {
            if (sec_gen.count() > 1.0) {
                need_update_goal = true;
            }
        }
        if (getInput("need_update_goal", need_update_goal) && need_update_goal) {
            generatedGoal();
            gen_time = std::chrono::steady_clock::now();
        }

        return BT::NodeStatus::SUCCESS;
    }
    else {
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::FAILURE;
}

void IsMoveCondition::generatedGoal()
{
    Task current_task;

    if (getInput("current_task", current_task)) {
        if (current_task.type() == TYPE.MOVE) {
            core_msgs::TaskAlarm alarm;
            alarm.alarm = ALARM.START;
            alarm.uuid = current_task.uuid();
            alarm.type = "move";
            task_alarm_.publish(alarm);
            NLOG(info) << "Send To move Command";
        }
    }
    NLOG(info) << "generatedGoal";
    BT::SharedQueue<Task> tasks;
    float f_last_x = -9999.0f;
    float f_last_y = -9999.0f;
    if (getInput("tasks", tasks)) {
        if (tasks->size() == 0) {
            NLOG(warning) << "No tasks in the queue";
            config().blackboard->set("need_update_goal", false);
            return;
        }
        bool b_loaded = false;
        if (getInput("load_state_received", b_loaded) && b_loaded) {
            NLOG(info) << "Pallet Loaded!, Change outline!!";
        }

        if (tasks->at(0).type() == TYPE.MOVE) {
            config().blackboard->set("current_task", tasks->at(0));
            // config().blackboard->get("b_arrive_align", b_arrive_align);
            // b_arrive_align = current_task.arrvieAlign();

            move_msgs::CoreCommand goals;
            for (const auto& task : *tasks) {
                if (task.type() != TYPE.MOVE && task.type() != TYPE.UNDOCKING)
                    break;

                std::vector<Task::MoveData> vec_data = task.moveData();
                NLOG(info) << "vec_data size : " << vec_data.size();
                if (vec_data.size() == 0) {
                    NLOG(info) << "not path error";
                    break;
                }
                for (const auto& data : vec_data) {
                    if (hypot(f_last_x - data.f_x_m, f_last_y - data.f_y_m) < 0.02f && !goals.list_waypoints.empty()) {
                        goals.list_waypoints.pop_back();
                        goals.list_lidar_obs.pop_back();
                    }

                    move_msgs::Waypoint waypoint;
                    waypoint.s_name = data.s_name;
                    waypoint.s_id = data.s_id;
                    waypoint.n_drive_type = data.n_drive_type;
                    waypoint.f_speed_ms = data.f_speed_ms;
                    waypoint.f_x_m = data.f_x_m;
                    waypoint.f_y_m = data.f_y_m;
                    waypoint.f_angle_deg = data.f_angle_deg;
                    waypoint.f_curve_radius = data.f_curve_radius;
                    waypoint.f_curvature = data.f_curvature;
                    waypoint.n_avoid_type = data.n_avoid_type;
                    waypoint.f_avoid_lanewidth = data.f_avoid_lanewidth;
                    waypoint.f_avoid_speed_ms = data.f_avoid_speed_ms;
                    waypoint.b_stop_quick = data.b_stop_quick;
                    waypoint.b_start_quick = data.b_start_quick;
                    waypoint.b_diagonal_align_skip = data.b_diagonal_align_skip;
                    waypoint.f_diagonal_heading_bias = data.f_diagonal_heading_bias;
                    waypoint.b_lccs = !data.b_lccs_off;

                    auto list_f_obstacle_margin = data.list_f_obstacle_margin;

                    if (list_f_obstacle_margin.size() == 4) {
                        //로드시 lccs 추가용...
                        if (b_loaded) {
                            list_f_obstacle_margin.at(0) += 0.0f;
                            list_f_obstacle_margin.at(1) += 2.2f;
                            list_f_obstacle_margin.at(2) += 0.5f;
                            list_f_obstacle_margin.at(3) += 0.5f;
                        }
                        move_msgs::LidarObstacle lidar_obs;
                        lidar_obs.list_f_obstacle_margin.resize(4);
                        lidar_obs.list_f_obstacle_margin.at(0) = list_f_obstacle_margin.at(0);
                        lidar_obs.list_f_obstacle_margin.at(1) = list_f_obstacle_margin.at(1);
                        lidar_obs.list_f_obstacle_margin.at(2) = list_f_obstacle_margin.at(2);
                        lidar_obs.list_f_obstacle_margin.at(3) = list_f_obstacle_margin.at(3);

                        NLOG(info) << lidar_obs.list_f_obstacle_margin.at(0);
                        NLOG(info) << lidar_obs.list_f_obstacle_margin.at(1);
                        NLOG(info) << lidar_obs.list_f_obstacle_margin.at(2);
                        NLOG(info) << lidar_obs.list_f_obstacle_margin.at(3);

                        goals.list_lidar_obs.emplace_back(lidar_obs);
                    }
                    else {
                        move_msgs::LidarObstacle lidar_obs;
                        lidar_obs.list_f_obstacle_margin.resize(4);
                        lidar_obs.list_f_obstacle_margin.at(0) = -1;
                        lidar_obs.list_f_obstacle_margin.at(1) = -1;
                        lidar_obs.list_f_obstacle_margin.at(2) = -1;
                        lidar_obs.list_f_obstacle_margin.at(3) = -1;

                        goals.list_lidar_obs.emplace_back(lidar_obs);
                    }

                    if (data.list_f_target_obstacle_margin.size() == 4) {
                        goals.list_f_target_obstacle_margin.clear();
                        goals.list_f_target_obstacle_margin.push_back(data.list_f_target_obstacle_margin.at(0));
                        goals.list_f_target_obstacle_margin.push_back(data.list_f_target_obstacle_margin.at(1));
                        goals.list_f_target_obstacle_margin.push_back(data.list_f_target_obstacle_margin.at(2));
                        goals.list_f_target_obstacle_margin.push_back(data.list_f_target_obstacle_margin.at(3));
                    }

                    if (data.list_f_move_obstacle_margin.size() == 4) {
                        waypoint.list_f_move_obstacle_margin.clear();
                        waypoint.list_f_move_obstacle_margin.push_back(data.list_f_move_obstacle_margin.at(0));
                        waypoint.list_f_move_obstacle_margin.push_back(data.list_f_move_obstacle_margin.at(1));
                        waypoint.list_f_move_obstacle_margin.push_back(data.list_f_move_obstacle_margin.at(2));
                        waypoint.list_f_move_obstacle_margin.push_back(data.list_f_move_obstacle_margin.at(3));
                    }

                    goals.b_start_pause = data.b_start_pause;
                    goals.list_waypoints.emplace_back(waypoint);
                    // goals.b_arrive_align = data.b_arrive_align;
                    f_last_x = data.f_x_m;
                    f_last_y = data.f_y_m;
                }
                goals.b_arrive_align = vec_data.back().b_arrive_align;
            }
            NLOG(info) << "goals size : " << goals.list_waypoints.size();

            NaviFra::Pos o_robot_pos;
            getInput("o_robot_pos", o_robot_pos);

            setOutput("goals", goals);
        }

        config().blackboard->set("updated_goals", true);
        config().blackboard->set("need_update_goal", false);
        // config().blackboard->set("b_arrive_align", false);
    }
}

BT::PortsList IsMoveCondition::providedPorts()
{
    return {BT::InputPort<Task>("current_task"),          BT::InputPort<bool>("need_update_goal"),
            BT::InputPort<std::string>("s_robot_status"), BT::InputPort<BT::SharedQueue<Task>>("tasks"),
            BT::InputPort<NaviFra::Pos>("o_robot_pos"),   BT::InputPort<bool>("charge_stop_received"),
            BT::InputPort<bool>("load_state_received"),   BT::OutputPort<bool>("updated_goals"),
            BT::OutputPort<Task>("current_task"),         BT::OutputPort<move_msgs::CoreCommand>("goals")};
}