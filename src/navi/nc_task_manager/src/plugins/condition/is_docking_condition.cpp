#include "nc_task_manager/nc_task_manager_pch.h"

#include <core_msgs/HacsNode.h>
#include <core_msgs/HacsNodeList.h>
#include <core_msgs/TaskAlarm.h>
#include <nc_task_manager/plugins/condition/is_docking_condition.h>
#include <std_msgs/String.h>

using namespace NaviFra;
IsDockingCondition::IsDockingCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    ros::NodeHandle nh;
    send_docking_ = nh.advertise<core_msgs::HacsNodeList>("hacs_agent/register_task", 1, false);
    send_docking_aruco_ = nh.advertise<core_msgs::HacsNodeList>("navifra/target", 1, false);
    task_alarm_ = nh.advertise<core_msgs::TaskAlarm>("nc_task_manager/task_alarm", 1, false);
}

BT::NodeStatus IsDockingCondition::tick()
{
    Task current_task;
    std_msgs::String mode;
    std::string s_robot_status;
    if (getInput("current_task", current_task) && current_task.type() == TYPE.DOCKING ) {
        core_msgs::HacsNodeList hacsnodelist;
        core_msgs::HacsNode start_node;
        core_msgs::HacsNode goal_node;
        
        auto data = current_task.data();
        std::ostringstream oss;
        Poco::JSON::Stringifier::stringify(*data, oss);
        NLOG(info) << "data: " << oss.str();

        std::string docking_type = current_task.docking_type();
        NLOG(info) << "docking_type : " << docking_type;
        std::string direction = current_task.direction();
        float velocity = current_task.velocity();
        std::string satrt_node_name = current_task.start_node_name();
        std::string end_node_name = current_task.end_node_name();
        hacsnodelist.docking_type = docking_type;

        if (data->has("start") && data->has("end")) {
            auto start = data->getObject("start");
            auto end = data->getObject("end");

            start_node.node_uuid = current_task.startNode();
            start_node.name = satrt_node_name;
            start_node.x_m = start->get("x").convert<float>();
            start_node.y_m = start->get("y").convert<float>();
            start_node.angle_deg = start->get("deg").convert<float>();
            // start_node.speed = 0.4;
            start_node.speed = velocity;

            auto list_f_camera_roi = current_task.camera_roi();
            auto list_f_camera_roi2 = current_task.camera_roi2();
            auto list_f_move_obstacle_margin = current_task.move_obstacle();
            int pallet_exist = current_task.pallet_exist();

            std::vector<float> vec_offset = current_task.goal_offset();

            NLOG(info) << list_f_camera_roi.size();
            NLOG(info) << list_f_camera_roi2.size();

            start_node.camera_roi_y_m = list_f_camera_roi[0];
            start_node.camera_roi_z_m = list_f_camera_roi[1];
            start_node.camera_roi2_y_m = list_f_camera_roi2[0];
            start_node.camera_roi2_z_m = list_f_camera_roi2[1];

            //도킹 시, LCCS off
            start_node.obstacle_margin_front = -2;
            start_node.obstacle_margin_rear = -2;
            start_node.obstacle_margin_left = -2;
            start_node.obstacle_margin_right = -2;

            if (pallet_exist == 0) {
                goal_node.move_obstacle_margin_front = 0.8;
                goal_node.move_obstacle_margin_rear = 0.9;
                goal_node.move_obstacle_margin_left = 0.28;
                goal_node.move_obstacle_margin_right = 0.28;
            }
            else if (pallet_exist == 1) {
                goal_node.move_obstacle_margin_front = 1.2;
                goal_node.move_obstacle_margin_rear = 1.2;
                goal_node.move_obstacle_margin_left = 0.7;
                goal_node.move_obstacle_margin_right = 0.7;
            }

            goal_node.f_global_offset_x = vec_offset[0];
            goal_node.f_global_offset_y = vec_offset[1];
            goal_node.f_global_offset_deg = vec_offset[2];


            // goal_node.move_obstacle_margin_front = list_f_move_obstacle_margin[0];
            // goal_node.move_obstacle_margin_rear = list_f_move_obstacle_margin[1];
            // goal_node.move_obstacle_margin_left = list_f_move_obstacle_margin[2];
            // goal_node.move_obstacle_margin_right = list_f_move_obstacle_margin[3];

            // if (current_task.type() == TYPE.DOCKING) {
            if (direction == "front") {
                start_node.drive_type = 1;  //전진
            }

            // else if (current_task.type() == TYPE.UNDOCKING) {
            else if (direction == "rear") {
                start_node.drive_type = 8;  //후진
            }

            goal_node.node_uuid = current_task.endNode();
            goal_node.name = end_node_name;
            goal_node.x_m = end->get("x").convert<float>();
            goal_node.y_m = end->get("y").convert<float>();
            goal_node.angle_deg = end->get("deg").convert<float>();
            goal_node.drive_type = 10;  //정지

            hacsnodelist.nodes.push_back(start_node);
            hacsnodelist.nodes.push_back(goal_node);
            if (current_task.type() == TYPE.DOCKING) {
                hacsnodelist.b_docking_in = true;
                hacsnodelist.b_docking_state = false;
                hacsnodelist.b_start_pause = false;
                hacsnodelist.b_docking_flag = true;

                // send_docking_.publish(hacsnodelist);
            }
            // else if (current_task.type() == TYPE.UNDOCKING) {
            //     hacsnodelist.b_docking_in = false;
            //     hacsnodelist.b_docking_state = true;
            //     hacsnodelist.b_start_pause = false;
            //     hacsnodelist.b_docking_flag = true;
            // }
            if (docking_type == "aruco" || docking_type == "aruco charge") {
                core_msgs::TaskAlarm alarm;
                alarm.alarm = ALARM.START;
                alarm.uuid = current_task.uuid();
                alarm.type = current_task.type();
                task_alarm_.publish(alarm);
                if (current_task.type() == TYPE.DOCKING) {
                    mode.data = "top_on";
                    NLOG(info) << mode.data;
                    send_led_.publish(mode);
                }
                if (hacsnodelist.b_docking_in) {
                    send_docking_aruco_.publish(hacsnodelist);
                    NLOG(info) << "Send To Aruco Command";
                }
            }
            else {
                core_msgs::TaskAlarm alarm;
                alarm.alarm = ALARM.START;
                alarm.uuid = current_task.uuid();
                alarm.type = current_task.type();
                task_alarm_.publish(alarm);
                if (current_task.type() == TYPE.DOCKING) {
                    mode.data = "top_on";
                    NLOG(info) << mode.data;
                    send_led_.publish(mode);
                }

                send_docking_.publish(hacsnodelist);
                NLOG(info) << "Send To Docking Command";
            }

            return BT::NodeStatus::SUCCESS;
        }
        else {
            return BT::NodeStatus::FAILURE;
        }
    }
    return BT::NodeStatus::FAILURE;
}

BT::PortsList IsDockingCondition::providedPorts()
{
    return {
        BT::InputPort<Task>("current_task"),
        BT::InputPort<std::string>("current_node"),
        BT::InputPort<std::string>("s_robot_status"),
    };
}