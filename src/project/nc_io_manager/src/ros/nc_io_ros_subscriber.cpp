#include "nc_io_manager/controller/nc_io_controller.h"
#include "util/logger.hpp"

namespace NaviFra {

void IOController::liftControlCallback(const task_msgs::Loading::ConstPtr& msg)
{
    std::string s_lift_control = msg->type;
    if (s_lift_control == "loading") {  // 핀업
        b_last_pin_up_command_ = true;  // <--- 핀업 명령 flag 세팅
        if (sequence_controller_->startSequence("pin_up_3s")) {
            last_sequnce_name_ = "pin_up_3s";
        };
    }
    else if (s_lift_control == "unloading") {  // 핀다운
        b_last_pin_up_command_ = false;  // <--- 핀다운이면 flag 해제
        if (sequence_controller_->startSequence("pin_down")) {
            last_sequnce_name_ = "pin_down";
        };
    }
}

void IOController::RecvBrakeCmd(const std_msgs::Bool::ConstPtr& msg)
{
    // 이전 상태 저장
    static bool b_last_brake_on = false;
    static bool b_first_call = true;  // 첫 호출인지 체크

    bool b_brake_on = msg->data;
    // LOG_INFO("Received brake command: %s", b_brake_on ? "ON" : "OFF");

    if (data_handler_->getButtonBit(BUTTON::BRAKE)) {
        LOG_INFO("Brake release button pressed, ignoring command.");
        return;
    }

    bool b_brake_feedback = !data_handler_->getSafetyBit(SAFETY::BRAKE_RELEASE);
    if (b_brake_on == b_brake_feedback) {
        // LOG_INFO("Brake command equals feedback, no action.");
        return;
    }

    // 이전 값과 같으면 변화 없으면 return
    if (!b_first_call && b_brake_on == b_last_brake_on) {
        // LOG_INFO("Brake command unchanged, ignoring.");
        return;
    }

    // 값이 달라졌을 때만 처리
    if (b_brake_on) {
        data_handler_->setCommandBit(COMMAND::BRAKE, false);
    }
    else {
        data_handler_->setCommandBit(COMMAND::BRAKE, true);
    }

    b_last_brake_on = b_brake_on;
    b_first_call = false;
}

void IOController::RecvEmergency(const std_msgs::Bool::ConstPtr& msg)
{
    bool b_lccs_detect = msg->data;

    // if (b_lccs_detect){
    //     data_handler_ -> setCommandBit2(COMMAND2::QUICK_STOP, true);
    //     LOG_INFO("QUICK STOP TRUE");
    // }
    // else if(!b_lccs_detect){
    //     data_handler_ -> setCommandBit2(COMMAND2::QUICK_STOP, false);
    //     LOG_INFO("QUICK STOP FALSE");
    // }
}

void IOController::RecvUndockingStatus(const std_msgs::String::ConstPtr& msg)
{
    std::string s = msg->data;
    if (s == "undocking") {
        if (!undock_up_check_pending_) {
            undock_up_check_pending_ = true;
            undock_up_check_start_tp_ = std::chrono::steady_clock::now();
            LOG_INFO("[UNDOCK] start received -> will check UP PX after %.1f sec", undock_up_check_delay_sec_);
        }
    }
}
void IOController::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    f_linear_x_ = float(msg->linear.x);
}

void IOController::NaviAlarmCallback(const core_msgs::NaviAlarm::ConstPtr& msg)
{
    if (msg->alarm >= 2000) {
        data_handler_->setCommandBit(COMMAND::CHARGE_RELAY_REAR, false);
    }
}

void IOController::outputCommandCallback(const std_msgs::String::ConstPtr& msg)
{
    // 서버가 실행 중이고 클라이언트가 연결되어 있는지 확인
    if (!plc_comm_->isServerRunning() || !plc_comm_->isClientConnected()) {
        LOG_WARNING("Cannot execute command: PLC server not running or no client connected");
        return;
    }

    LOG_INFO("Received output command: %s", msg->data.c_str());

    try {
        std::string command = msg->data;
        parseAndExecuteCommand(command);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception while processing command: %s", e.what());
    }
}  // pin up 3초만 여러번 하고 멈추고 알람띄우지 않기

void IOController::RecvRobotPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_pos_);
    o_robot_pos_.SetXm(msg->pose.pose.position.x);
    o_robot_pos_.SetYm(msg->pose.pose.position.y);
}

void IOController::RecvARPose(const core_msgs::ARPoseList::ConstPtr& msg)
{
    if (msg->ar_poses.empty()) {
        b_ar_detect_ = false;
    }
    else {
        b_ar_detect_ = true;
    }
    // ar marker의 global 좌표
    NaviFra::Pos p, now_pos;
    p.SetXm(msg->ar_poses[0].f_x_m);
    p.SetYm(msg->ar_poses[0].f_y_m);
    p.SetDeg(msg->ar_poses[0].f_deg);
    {
        std::lock_guard<std::mutex> lock(mtx_pos_);
        now_pos = o_robot_pos_;
    }
    double dist = std::hypot(p.GetXm() - now_pos.GetXm(), p.GetYm() - now_pos.GetYm());
    if (dist > 6.0) {  //로봇 거리 감안해서... 나중에 수정할수도있음...
        NLOG(info) << "AR_marker is so far! return";
        b_ar_detect_ = false;
        return;
    }
    return;
}

void IOController::RecvBmsInfo(const core_msgs::BmsInfo::ConstPtr& msg)
{
    f_pack_current_ = msg->f32_pack_current;
}

}  // namespace NaviFra