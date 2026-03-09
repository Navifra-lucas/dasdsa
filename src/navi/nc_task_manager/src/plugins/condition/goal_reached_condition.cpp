#include "nc_task_manager/nc_task_manager_pch.h"
#include "pos/pos.hpp"

#include <core_msgs/NavicoreStatus.h>
#include <core_msgs/TaskAlarm.h>
#include <move_msgs/CoreCommand.h>
#include <nc_task_manager/plugins/condition/goal_reached_condition.h>
#include <nc_task_manager/topic/nc_task_manager_topic.h>
#include <std_msgs/String.h>

using namespace NaviFra;
GoalReachedCondition::GoalReachedCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    ros::NodeHandle nh;
    task_alarm_ = nh.advertise<core_msgs::TaskAlarm>("nc_task_manager/task_alarm", 10, false);
    docking_state_pub_ = nh.advertise<std_msgs::String>("/docking_state", 1);
}

BT::NodeStatus GoalReachedCondition::tick()
{
    if (getInput("goals", goals_) && goals_.list_waypoints.size() == 0) {
        NLOG(info) << "GoalReached";
        return BT::NodeStatus::SUCCESS;
    }
    else {
        Task current_task;
        std::string current_node;
        NaviFra::Pos o_robot_pos;
        if (getInput("current_node", current_node) && getInput("current_task", current_task)) {
            getInput("o_robot_pos", o_robot_pos);
            std::vector<Task::MoveData> vec_data = current_task.moveData();

            if (vec_data.size() > 0) {
                bool b_robot_passed_last_waypoint = false;
                bool b_goal_reached = false;

                float f_goal_dist = hypot(o_robot_pos.GetXm() - vec_data.back().f_x_m, o_robot_pos.GetYm() - vec_data.back().f_y_m);
                if (getInput("goal_reached", b_goal_reached) && b_goal_reached) {
                    // arrive alarm 받음
                    NLOG(info) << "b_goal_reached";
                    b_robot_passed_last_waypoint = true;
                }
                else if (vec_data.size() > 1) {
                    Pos o_start_pos(vec_data[vec_data.size() - 2].f_x_m, vec_data[vec_data.size() - 2].f_y_m, 0);
                    Pos o_end_pos(vec_data.back().f_x_m, vec_data.back().f_y_m, 0);
                    NaviFra::Pos diffpos_12 = o_end_pos - o_start_pos;

                    float f_dist_check = hypot(diffpos_12.GetXm(), diffpos_12.GetYm());

                    // 로봇이 마지막 웨이포인트를 지나쳤는지 검사
                    if (f_dist_check > 0.05) {
                        float theta = atan2(diffpos_12.GetYm(), diffpos_12.GetXm());
                        Pos o_now_pos(o_robot_pos.GetXm(), o_robot_pos.GetYm(), theta * 180 / 3.141592);
                        SimplePos o_local_remain = o_now_pos.inv() * o_end_pos;
                        float f_x_remain_dist = o_local_remain.GetXm();

                        if (f_x_remain_dist < 0.03 && f_goal_dist < 0.3 && current_node.size() > 0 &&
                            current_node == vec_data.back().s_id) {
                            NLOG(info) << "f_x_remain_dist " << f_x_remain_dist;
                            NLOG(info) << " current_node : " << current_node << "vec_data.back().s_id " << vec_data.back().s_id;
                            NLOG(info) << "b_robot_passed_last_waypoint";
                            b_robot_passed_last_waypoint = true;
                        }
                    }
                    else {
                        if (f_goal_dist < 0.3 && current_node.size() > 0 && current_node == vec_data.back().s_id) {
                            NLOG(info) << " current_node : " << current_node << "vec_data.back().s_id " << vec_data.back().s_id;
                            NLOG(info) << "b_robot_passed_last_waypoint";
                            b_robot_passed_last_waypoint = true;
                        }
                    }
                }

                if (b_robot_passed_last_waypoint) {
                    NLOG(info) << "Waypoint Done ";
                    updateGoal();
                    updateTask();
                    config().blackboard->set("goal_reached", false);
                    return BT::NodeStatus::SUCCESS;
                }
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList GoalReachedCondition::providedPorts()
{
    return {
        BT::InputPort<move_msgs::CoreCommand>("goals"),
        BT::InputPort<Task>("current_task"),
        BT::InputPort<std::string>("current_node"),
        BT::InputPort<BT::SharedQueue<Task>>("tasks"),
        BT::InputPort<NaviFra::Pos>("o_robot_pos"),
        BT::InputPort<bool>("goal_reached"),
        BT::InputPort<Task>("previous_task"),

        BT::OutputPort<Task>("previous_task"),
        BT::OutputPort<Task>("current_task"),
        BT::OutputPort<BT::SharedQueue<Task>>("tasks"),
        BT::OutputPort<move_msgs::CoreCommand>("goals"),
        BT::OutputPort<bool>("goal_reached")};
}

void GoalReachedCondition::updateGoal()
{
    try {
        Task current_task;
        if (getInput("current_task", current_task)) {
            // 삭제된 요소를 실제로 제거
            std::vector<Task::MoveData> vec_data = current_task.moveData();
            for (int i = 0; i < goals_.list_waypoints.size(); i++) {
                float f_dist = hypot(
                    vec_data.back().f_x_m - goals_.list_waypoints.at(i).f_x_m, vec_data.back().f_y_m - goals_.list_waypoints.at(i).f_y_m);
                NLOG(info) << "i : " << i << " f_dist " << f_dist;
                if (f_dist < 0.01) {
                    if (i == goals_.list_waypoints.size() - 1)  // 마지막 웨이포인트가 도착한 경우 전체 삭제
                        goals_.list_waypoints.clear();
                    else  // 마지막이 아니라면 출발전까지 삭제
                        goals_.list_waypoints.erase(goals_.list_waypoints.begin(), goals_.list_waypoints.begin() + i);
                    break;
                }
            }
            for (size_t i = 0; i < goals_.list_waypoints.size(); i++)
                NLOG(info) << boost::format("Stored Goal Name [ %1% ]") % goals_.list_waypoints.at(i).s_name;

            config().blackboard->set("goals", goals_);
        }
    }
    catch (std::exception& ex) {
        NLOG(error) << ex.what();
    }
}

void GoalReachedCondition::updateTask()
{
    try {
        BT::SharedQueue<Task> tasks;
        if (getInput("tasks", tasks) && tasks->size() > 0) {
            NLOG(info) << "GoalReached Update task";

            while (tasks->size() > 0) {
                NaviFra::Pos o_robot_pos;
                getInput("o_robot_pos", o_robot_pos);
                std::vector<Task::MoveData> vec_data = tasks->at(0).moveData();
                if (vec_data.size() == 0) {
                    NLOG(info) << "not path error";
                    break;
                }
                float f_goal_robot_dist = hypot(vec_data.back().f_x_m - o_robot_pos.GetXm(), vec_data.back().f_y_m - o_robot_pos.GetYm());

                core_msgs::TaskAlarm alarm;
                alarm.uuid = tasks->at(0).uuid();
                alarm.type = tasks->at(0).type();
                alarm.alarm = ALARM.DONE;
                NLOG(info) << "update goal done" << tasks->at(0).print();
                task_alarm_.publish(alarm);

                if (tasks->at(0).type() == TYPE.UNDOCKING) {
                    std_msgs::String msg;
                    msg.data = "DOCKINGOUT DONE";
                    docking_state_pub_.publish(msg);
                }

                tasks->pop_front();
                NLOG(info) << "f_goal_robot_dist : " << f_goal_robot_dist;
                if (f_goal_robot_dist < 0.3) {
                    NLOG(info) << "break!";
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            if (tasks->size() > 0) {
                config().blackboard->set("current_task", tasks->at(0));
                config().blackboard->set("tasks", tasks);
                core_msgs::TaskAlarm alarm;
                alarm.alarm = ALARM.START;
                alarm.uuid = tasks->at(0).uuid();
                alarm.type = "move";  // wendy. 가지고 있는 task검사해서 move나 turn으로만 구성되어 있을때 alarm 올림
                task_alarm_.publish(alarm);
                // NLOG("task alarm start");
            }
            else {
                config().blackboard->set("current_task", NaviFra::Task());
                config().blackboard->set("tasks", tasks);
            }
        }
    }

    catch (std::exception& ex) {
        NLOG(error) << ex.what();
    }
}