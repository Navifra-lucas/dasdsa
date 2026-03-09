#include "nc_dcs/docking_control_node.h"

namespace nc_dcs {
bool DockingControlNode::errorCheck(SequenceStep& step)
{
    if (forkErrorCheck(step)) {
        return true;
    }
    else if (perceptionErrorCheck(step)) {
        return true;
    }
    else if(step.action_type == "fork_updown"){
        int n_current_fork_height = lift_status_.fork_up_down_position;
        static int n_pre_fork_height = n_current_fork_height;
        if(n_pre_fork_height != n_current_fork_height) {
            lift_move_time_ = ros::Time::now();
        }

        if(ros::Time::now() - lift_move_time_ > ros::Duration(5.0)) {
            LOG_ERROR("[DCS] Fork Updown Timeout %d", n_lift_up_down_retry_count_);
            step_initialized_ = false;
            n_lift_up_down_retry_count_++;
            if(n_lift_up_down_retry_count_ > MAX_LIFT_UP_DOWN_RETRY) {
                std_msgs::Int64 n_msg;
                n_msg.data = core_msgs::NaviAlarm::ERROR_LIFT_FORK_TRACTION_MOTOR_FEEDBACK_TIMEOUT;
                ros_interface_->getAlarmPub().publish(n_msg);
                return true;
            }
        }
        n_pre_fork_height = n_current_fork_height;
    }

    // Docking height interlock: stop if fork height error exceeds tolerance within 2.3m
    if (step.action_type == "move" && step.params.getString("type", "") == "docking") {
        int n_height_tolerance = fork_params_loader_.getDockingHeightToleranceMm();
        int n_target_height = current_mission_.n_real_target_height;
        int n_current_height = lift_status_.fork_up_down_position;
        int n_height_diff = std::abs(n_target_height - n_current_height);

        float f_remain_dist = hypot(
            target_pose_.pose.position.x - robot_pose_.pose.position.x,
            target_pose_.pose.position.y - robot_pose_.pose.position.y);

        if (f_remain_dist < 2.3) {
            LOG_INFO("[DCS] Docking height check: remain_dist=%.2f, target=%d, feedback=%d, diff=%d, tolerance=%d",
                     f_remain_dist, n_target_height, n_current_height, n_height_diff, n_height_tolerance);
        }

        if (f_remain_dist < 2.3 && n_height_diff >= n_height_tolerance) {
            LOG_ERROR("[DCS] Docking height interlock! remain_dist=%.2f, target=%d, feedback=%d, diff=%d, tolerance=%d",
                      f_remain_dist, n_target_height, n_current_height, n_height_diff, n_height_tolerance);
            std_msgs::Int64 n_msg;
            n_msg.data = core_msgs::NaviAlarm::ERROR_LIFT_FORK_TRACTION_MOTOR_FEEDBACK_TIMEOUT;
            ros_interface_->getAlarmPub().publish(n_msg);
            return true;
        }
    }

    return false;
}

bool DockingControlNode::perceptionErrorCheck(SequenceStep& step)
{
    if (step.action_type == "intensity" || step.action_type == "perception") {
        if (ros::Time::now() - perception_recv_time_ > ros::Duration(10.0)) {
            LOG_ERROR("[DCS] Perception Timeout");
            std_msgs::Int64 n_msg;
            n_msg.data = core_msgs::NaviAlarm::ERROR_DOCK_NOT_FIND;
            ros_interface_->getAlarmPub().publish(n_msg);
            return true;
        }
    }

    if (step.action_type == "perception") {
        if (n_perception_999_cnt_ >= 10) {
            LOG_ERROR("[DCS] Perception 999 count is over 10")
            std_msgs::Int64 n_msg;
            n_msg.data = core_msgs::NaviAlarm::ERROR_DOCK_NOT_FIND;
            ros_interface_->getAlarmPub().publish(n_msg);
            return true;
        }
    }
    return false;
}

bool DockingControlNode::forkErrorCheck(SequenceStep& step)
{
    // 지금 내 액션이 포크리프트 동작 액션인지 체크
    // 맞다면 for문 역으로 돌면서 포크리프트 관련 동작을 확인
    // 최초로 발견되는 포크리프트 동작이 내 동작과 다르다면
    // 포크리프트 관련 동작이 실제로 종료되었는지 체크 하고 return

    if (step.action_type != "fork_updown" && step.action_type != "fork_side" && step.action_type != "fork_tilt") {
        return false;
    }

    for (int i = current_step_index_ - 1; i >= 0; i--) {
        if (current_sequence_.steps[i].action_type == "fork_updown" || current_sequence_.steps[i].action_type == "fork_side" ||
            current_sequence_.steps[i].action_type == "fork_tilt") {
            if (current_sequence_.steps[i].action_type == step.action_type) {
                return false;
            }

            std::lock_guard<std::mutex> lock(lift_status_mutex_);

            int target_value = fork_params_loader_.getTargetValue(
                current_sequence_.steps[i].action_type, current_sequence_.steps[i].params.string_params, current_mission_.n_real_target_height);

            int n_current_value, n_tolerance;

            if (current_sequence_.steps[i].action_type == "fork_updown") {
                n_current_value = lift_status_.fork_up_down_position;
                n_tolerance = 25;
            }
            else if (current_sequence_.steps[i].action_type == "fork_side") {
                n_current_value = lift_status_.fork_width;
                n_tolerance = 0;
            }
            else if (current_sequence_.steps[i].action_type == "fork_tilt") {
                n_current_value = lift_status_.tilting_up_down;
                n_tolerance = 0;
            }

            bool b_error = (std::abs(n_current_value - target_value) > n_tolerance);

            if (b_error) {
                LOG_ERROR(
                    "[DCS] Fork action '%s' not completed. Target: %d, Current: %d", current_sequence_.steps[i].action_type.c_str(),
                    target_value, n_current_value);
                std_msgs::Int64 n_msg;
                // n_msg.data = core_msgs::NaviAlarm::ERROR_LIFT_NOT_FIND;
                ros_interface_->getAlarmPub().publish(n_msg);
            }
            return b_error;
        }
    }
    return false;
}

}
