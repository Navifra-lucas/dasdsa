#include "nc_task_manager/nc_task_manager_pch.h"
#include "pos/pos.hpp"

#include <core_msgs/TaskAlarm.h>
#include <move_msgs/CoreCommand.h>
#include <nc_task_manager/data/nc_task.h>
#include <nc_task_manager/plugins/condition/is_undocking_condition.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace NaviFra;

IsUnDockingCondition::IsUnDockingCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    ros::NodeHandle nh;
    task_alarm_ = nh.advertise<core_msgs::TaskAlarm>("nc_task_manager/task_alarm", 1, false);
    send_stopcharge_ = nh.advertise<std_msgs::String>("output_command", 1, false);
    sendLivePath_ = nh.advertise<move_msgs::CoreCommand>("navifra/live_path", 1, false);
}

BT::NodeStatus IsUnDockingCondition::tick()
{
    Task current_task;
    if (getInput("current_task", current_task)) {
        if (current_task.type() == TYPE.UNDOCKING) {
            static auto gen_time = std::chrono::steady_clock::now();
            Task current_task;
            bool b_uncharge_received = false;

            static auto tp_action_ = std::chrono::system_clock::now();

            // current_task 가져오기
            if (!getInput("current_task", current_task))
                return BT::NodeStatus::FAILURE;
            if (current_task.type() != TYPE.UNDOCKING)
                return BT::NodeStatus::FAILURE;

            // 로봇 상태 확인
            std::string s_robot_status;
            getInput("s_robot_status", s_robot_status);
            getInput("uncharge_received", b_uncharge_received);

            std::chrono::duration<double> sec = std::chrono::system_clock::now() - tp_action_;
            
            if (!b_uncharge_received ) {
                if(sec.count() >= 1.0)
                {
                    std_msgs::String msg;
                    msg.data = "charge_stop";
                    send_stopcharge_.publish(msg);
                    NLOG(info) << " charge Stop ";

                    tp_action_ = std::chrono::system_clock::now();
                }
                
                return BT::NodeStatus::RUNNING;
            }

            // 목표 생성
            move_msgs::CoreCommand goals;
            NaviFra::Pos o_robot_pos;
            getInput("o_robot_pos", o_robot_pos);

            double start_x = o_robot_pos.GetXm();
            double start_y = o_robot_pos.GetYm();
            double start_rad = o_robot_pos.GetRad();
            double start_deg = start_rad * 180.0 / M_PI;
            double docking_dist = current_task.docking_dist();
            double d_speed = current_task.velocity();
            int drive_type = current_task.drive_type();

            // 출발/도착 waypoint
            move_msgs::Waypoint start_node, goal_node;
            start_node.s_name = current_task.start_node_name();
            start_node.n_drive_type = drive_type;
            start_node.f_x_m = start_x;
            start_node.f_y_m = start_y;
            start_node.f_angle_deg = start_deg;
            start_node.f_speed_ms = d_speed;

            auto list_f_obstacle_margin = current_task.obstacle();
            auto list_f_target_obstacle_margin = current_task.target_obstacle();

            if (list_f_obstacle_margin.size() == 4) {
                move_msgs::LidarObstacle lidar_obs;
                lidar_obs.list_f_obstacle_margin.resize(4);
                lidar_obs.list_f_obstacle_margin.at(0) = list_f_obstacle_margin.at(0);
                lidar_obs.list_f_obstacle_margin.at(1) = list_f_obstacle_margin.at(1);
                lidar_obs.list_f_obstacle_margin.at(2) = list_f_obstacle_margin.at(2);
                lidar_obs.list_f_obstacle_margin.at(3) = list_f_obstacle_margin.at(3);

                goals.list_lidar_obs.emplace_back(lidar_obs);
            }
            else {
                move_msgs::LidarObstacle lidar_obs;
                lidar_obs.list_f_obstacle_margin.resize(4);
                lidar_obs.list_f_obstacle_margin.at(0) = -2;
                lidar_obs.list_f_obstacle_margin.at(1) = -2;
                lidar_obs.list_f_obstacle_margin.at(2) = -2;
                lidar_obs.list_f_obstacle_margin.at(3) = -2;

                goals.list_lidar_obs.emplace_back(lidar_obs);
            }

            if (list_f_target_obstacle_margin.size() == 4) {
                goals.list_f_target_obstacle_margin.clear();
                goals.list_f_target_obstacle_margin.push_back(list_f_target_obstacle_margin.at(0));
                goals.list_f_target_obstacle_margin.push_back(list_f_target_obstacle_margin.at(1));
                goals.list_f_target_obstacle_margin.push_back(list_f_target_obstacle_margin.at(2));
                goals.list_f_target_obstacle_margin.push_back(list_f_target_obstacle_margin.at(3));
            }
            goals.list_waypoints.emplace_back(start_node);

            double end_x = start_x + std::cos(start_rad) * docking_dist;
            double end_y = start_y + std::sin(start_rad) * docking_dist;

            goal_node.s_name = current_task.end_node_name();
            goal_node.n_drive_type = drive_type;
            goal_node.f_x_m = end_x;
            goal_node.f_y_m = end_y;
            goal_node.f_angle_deg = start_deg;
            goal_node.f_speed_ms = d_speed;

            if (list_f_obstacle_margin.size() == 4) {
                move_msgs::LidarObstacle lidar_obs;
                lidar_obs.list_f_obstacle_margin.resize(4);
                lidar_obs.list_f_obstacle_margin.at(0) = list_f_obstacle_margin.at(0);
                lidar_obs.list_f_obstacle_margin.at(1) = list_f_obstacle_margin.at(1);
                lidar_obs.list_f_obstacle_margin.at(2) = list_f_obstacle_margin.at(2);
                lidar_obs.list_f_obstacle_margin.at(3) = list_f_obstacle_margin.at(3);

                goals.list_lidar_obs.emplace_back(lidar_obs);
            }
            else {
                move_msgs::LidarObstacle lidar_obs;
                lidar_obs.list_f_obstacle_margin.resize(4);
                lidar_obs.list_f_obstacle_margin.at(0) = -2;
                lidar_obs.list_f_obstacle_margin.at(1) = -2;
                lidar_obs.list_f_obstacle_margin.at(2) = -2;
                lidar_obs.list_f_obstacle_margin.at(3) = -2;

                goals.list_lidar_obs.emplace_back(lidar_obs);
            }

            if (list_f_target_obstacle_margin.size() == 4) {
                goals.list_f_target_obstacle_margin.clear();
                goals.list_f_target_obstacle_margin.push_back(list_f_target_obstacle_margin.at(0));
                goals.list_f_target_obstacle_margin.push_back(list_f_target_obstacle_margin.at(1));
                goals.list_f_target_obstacle_margin.push_back(list_f_target_obstacle_margin.at(2));
                goals.list_f_target_obstacle_margin.push_back(list_f_target_obstacle_margin.at(3));
            }
            goals.list_waypoints.emplace_back(goal_node);

            if (current_task.type() == TYPE.UNDOCKING) {
                core_msgs::TaskAlarm alarm;
                alarm.alarm = ALARM.START;
                alarm.uuid = current_task.uuid();
                alarm.type = "move";
                task_alarm_.publish(alarm);
                NLOG(info) << "Send To move Command";
            }
            sendLivePath_.publish(goals);

            NLOG(info) << "[IsUnDockingCondition] Goal generated: start(" << start_x << "," << start_y << ") -> end(" << end_x << "," << end_y
                    << ")";
            gen_time = std::chrono::steady_clock::now();
            NLOG(info) << "return success";

            return BT::NodeStatus::SUCCESS;
        }
    }
    return BT::NodeStatus::FAILURE;
}

BT::PortsList IsUnDockingCondition::providedPorts()
{
    return {
        BT::InputPort<Task>("current_task"),
        BT::InputPort<std::string>("s_robot_status"),
        BT::InputPort<bool>("uncharge_received"),
        BT::InputPort<NaviFra::Pos>("o_robot_pos"),
        BT::OutputPort<move_msgs::CoreCommand>("goals"),
        BT::OutputPort<bool>("updated_goals"),
        BT::OutputPort<bool>("need_update_goal")};
}