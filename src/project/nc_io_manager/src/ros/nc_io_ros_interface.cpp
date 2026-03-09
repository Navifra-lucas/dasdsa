#include "nc_io_manager/controller/nc_io_controller.h"

#include "util/logger.hpp"

namespace NaviFra {

void IOController::setupROSInterface()
{
    // Subscriber 설정
    output_cmd_sub_ = nh_.subscribe("output_command", 5, &IOController::outputCommandCallback, this);
    lift_control_sub_ = nh_.subscribe("/task_manager/lift_control", 10, &IOController::liftControlCallback, this);// lift_control 에는 acs에서 주면 loading이면 올리고 unloading이면 내리고
    brake_cmd_sub_ = nh_.subscribe("motor_brakeon_target", 5, &IOController::RecvBrakeCmd, this);
    emergency_sub_ = nh_.subscribe("emergency", 5, &IOController::RecvEmergency, this);
    undocking_status_sub_ = nh_.subscribe("wia_agent/now_task", 5, &IOController::RecvUndockingStatus, this);
    sub_navi_alarm_ = nh_.subscribe("navifra/alarm", 10, &IOController::NaviAlarmCallback, this);
    ar_pose_sub_ = nh_.subscribe("/aruco_pos", 10, &IOController::RecvARPose, this);
    bms_info_sub_ = nh_.subscribe("bms_info", 10, &IOController::RecvBmsInfo, this);
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10, &IOController::CmdVelCallback, this);

    lift_status_pub_ = nh_.advertise<task_msgs::LiftInfo>("/plc/lift_info", 10);  // lift_info 에는 완료가 되면 lift_top 혹은 lift_bottom을 줘야함
    navi_alarm_pub_ = nh_.advertise<core_msgs::NaviAlarm>("navi_alarm", 10); //bms에서 충전 알아서 자체판단해서 하기, can으로 충전기한테 충전주기
    error_pub_ = nh_.advertise<std_msgs::Int64>("/navifra/error", 5);
    sequence_complete_pub_ = nh_.advertise<std_msgs::String>("sequence_complete", 10);
    motor_cmd_req_ = nh_.serviceClient<motor_msgs::MotorCmd>("motor_cmd");
    brake_pub_ = nh_.advertise<std_msgs::Bool>("motor_brakeon_feedback", 5); 
    cmd_pub_ = nh_.advertise<std_msgs::String>("navifra/cmd", 10);
    plc_info_pub_ = nh_.advertise<core_msgs::PLCInfo>("/plc_info", 5);
    charge_state_pub_ = nh_.advertise<std_msgs::Bool>("/charge_state", 5);
}

void IOController::loadParameters()
{
    ros::NodeHandle nhp("~");  // Private node handle for parameters

    // PLC 서버 설정 (기존 client 설정 대신)
    nhp.param<std::string>("server_bind_ip", server_bind_ip_, "0.0.0.0");  // 모든 인터페이스에서 리스닝
    nhp.param<int>("server_port", server_port_, 5000);  // 리스닝 포트

    // 재시도 설정 (서버에서는 연결 대기만)
    nhp.param<int>("max_accept_retry", max_retry_count_, 5);
    nhp.param<double>("accept_retry_delay_seconds", retry_delay_seconds_, 2.0);
    nhp.param<double>("server_restart_interval_seconds", reconnect_interval_seconds_, 10.0);
    nhp.param<double>("undocking_up_check_delay_sec", undock_up_check_delay_sec_, 3.0);

    // 통신 설정 (100ms 고정)
    communication_interval_ms_ = 100;  // 100ms 고정 주기
    nhp.param<int>("max_consecutive_failures", max_consecutive_failures_, 3);

    // 파라미터 로그 출력
    LOG_INFO("PLC Server Configuration:");
    LOG_INFO("  Bind IP: %s", server_bind_ip_.c_str());
    LOG_INFO("  Port: %d", server_port_);
    LOG_INFO("  Max Accept Retry: %d", max_retry_count_);
    LOG_INFO("  Accept Retry Delay: %.1f seconds", retry_delay_seconds_);
    LOG_INFO("  Server Restart Interval: %.1f seconds", reconnect_interval_seconds_);
    LOG_INFO("  Communication Interval: %d ms", communication_interval_ms_);
}

// --- 기존 plc_info_send 시퀀스 동작을 함수로 ---
void IOController::publishPlcInfoOnce_()
{
    // RESET 버튼 신호
    bool b_reset_switch = data_handler_->getButtonBit(BUTTON::RESET);

    // --- Bumper/Ems 원시 신호 ---
    bool raw_front_bumper = !data_handler_->getSafetyBit(SAFETY::FRONT_BUMPPER);
    bool raw_rear_bumper  = !data_handler_->getSafetyBit(SAFETY::REAR_BUMPER);
    bool raw_ems          = !data_handler_->getSafetyBit(SAFETY::EMERGENCY);

    // --- 래치 갱신 ---
    static bool latched_front_bumper = false;
    static bool latched_rear_bumper  = false;
    static bool latched_ems          = false;

    if (raw_front_bumper) latched_front_bumper = true;
    if (raw_rear_bumper)  latched_rear_bumper  = true;
    if (raw_ems)          latched_ems          = true;

    if (b_reset_switch) {
        latched_front_bumper = false;
        latched_rear_bumper  = false;
        latched_ems          = false;
        std_msgs::String alarm_clear_msg;
        alarm_clear_msg.data = "alarm_clear";
        cmd_pub_.publish(alarm_clear_msg);
    }

    // === 나머지 IO 원시 신호 ===
    bool b_ossd_signal       = data_handler_->getSafetyBit(SAFETY::OSSD_SIGNAL);
    bool b_sto_signal        = data_handler_->getSafetyBit(SAFETY::STO_SIGNAL);
    bool b_brake_release_fb  = data_handler_->getSafetyBit(SAFETY::BRAKE_RELEASE);
    bool b_manual_charge_on  = data_handler_->getSafetyBit(SAFETY::MANUAL_CHARGE);
    bool b_quick_stop_status = data_handler_->getSafetyBit(SAFETY::QUICK_STOP_SIGNAL);

    bool b_brake_release_sw  = data_handler_->getButtonBit(BUTTON::BRAKE);
    bool b_io_mode_sel1      = data_handler_->getButtonBit(BUTTON::MODE_SELECTE_1);
    bool b_io_mode_sel2      = data_handler_->getButtonBit(BUTTON::MODE_SELECTE_2);

    bool b_drive_power_relay = data_handler_->getSensorBit(SENSOR::DRIVE_POWER_RELAY_CHECK_SIGNAL);
    bool b_rear_charge_relay = data_handler_->getSensorBit(SENSOR::REAR_CHARGE_RELAY_CHECK_SIGNAL);

    bool b_front_up_px       = data_handler_->getPinBit(PIN::FRONT_UP_PX);
    bool b_front_down_px     = data_handler_->getPinBit(PIN::FRONT_DOWN_PX);
    bool b_rear_up_px        = data_handler_->getPinBit(PIN::REAR_UP_PX);
    bool b_rear_down_px      = data_handler_->getPinBit(PIN::REAR_DOWN_PX);
    bool b_front_motor_state = data_handler_->getPinBit(PIN::FRONT_MOTOR_ON);
    bool b_rear_motor_state  = data_handler_->getPinBit(PIN::REAR_MOTOR_ON);
    bool b_front_motor_alarm = data_handler_->getPinBit(PIN::FRONT_MOTOR_ALARM);
    bool b_rear_motor_alarm  = data_handler_->getPinBit(PIN::REAR_MOTOR_ALARM);

    int n_major = data_handler_->getInputValue(INPUT::VERSION_MAJOR);
    int n_minor = data_handler_->getInputValue(INPUT::VERSION_MINOR);
    int n_patch = data_handler_->getInputValue(INPUT::VERSION_PATCH);

    // --- 메시지 작성 ---
    core_msgs::PLCInfo plc_msg;
    plc_msg.front_bumper_switch     = latched_front_bumper;
    plc_msg.rear_bumper_switch      = latched_rear_bumper;
    plc_msg.emergency               = latched_ems;
    plc_msg.ossd_signal             = b_ossd_signal;
    plc_msg.sto_signal              = b_sto_signal;
    plc_msg.brake_release_feedback  = b_brake_release_fb;
    plc_msg.manual_charge_on        = b_manual_charge_on;
    plc_msg.quick_stop_status       = b_quick_stop_status;
    plc_msg.reset_switch            = b_reset_switch;
    plc_msg.brake_release_switch    = b_brake_release_sw;
    plc_msg.io_mode_select_1        = b_io_mode_sel1;
    plc_msg.io_mode_select_2        = b_io_mode_sel2;
    plc_msg.drive_power_relay_signal= b_drive_power_relay;
    plc_msg.rear_charge_relay_signal= b_rear_charge_relay;
    plc_msg.front_up_px             = b_front_up_px;
    plc_msg.front_down_px           = b_front_down_px;
    plc_msg.rear_up_px              = b_rear_up_px;
    plc_msg.rear_down_px            = b_rear_down_px;
    plc_msg.front_motor_state       = b_front_motor_state;
    plc_msg.rear_motor_state        = b_rear_motor_state;
    plc_msg.front_motor_alarm       = b_front_motor_alarm;
    plc_msg.rear_motor_alarm        = b_rear_motor_alarm;
    plc_msg.version_major           = static_cast<uint8_t>(n_major);
    plc_msg.version_minor           = static_cast<uint8_t>(n_minor);
    plc_msg.version_patch           = static_cast<uint8_t>(n_patch);
    plc_msg.plc_version             = std::to_string(n_major) + "." + std::to_string(n_minor) + "." + std::to_string(n_patch);

    plc_info_pub_.publish(plc_msg);
    // LOG_INFO("QUICK STOP STATUS: %d", b_quick_stop_status);

}

void IOController::scanAndPublishPinErrorsOnce_()
{
    int alarm_code = 0; 

    const bool front_motor_alarm = data_handler_->getPinBit(PIN::FRONT_MOTOR_ALARM);
    const bool rear_motor_alarm  = data_handler_->getPinBit(PIN::REAR_MOTOR_ALARM);
    const bool f_up = data_handler_->getPinBit(PIN::FRONT_UP_PX);
    const bool f_dn = data_handler_->getPinBit(PIN::FRONT_DOWN_PX);
    const bool r_up = data_handler_->getPinBit(PIN::REAR_UP_PX);
    const bool r_dn = data_handler_->getPinBit(PIN::REAR_DOWN_PX);

    const bool bumper_clear_mode = data_handler_->getCommandBit2(COMMAND2::BUMPER_CLEAR);

    const bool b_front_bumper = !data_handler_->getSafetyBit(SAFETY::FRONT_BUMPPER);
    const bool b_rear_bumper  = !data_handler_->getSafetyBit(SAFETY::REAR_BUMPER);

    if (!bumper_clear_mode) {
        if (b_front_bumper) {
            alarm_code = core_msgs::NaviAlarm::ERROR_FRONT_BUMPER_DETECTED;
        } else if (b_rear_bumper) {
            alarm_code = core_msgs::NaviAlarm::ERROR_REAR_BUMPER_DETECTED;
        }
    }

    if (alarm_code == 0) {
        if (front_motor_alarm) {
            alarm_code = core_msgs::NaviAlarm::ERROR_FRONT_PIN_MOTOR_ALARM;
        } else if (rear_motor_alarm) {
            alarm_code = core_msgs::NaviAlarm::ERROR_REAR_PIN_MOTOR_ALARM;
        } else if (f_up && f_dn) {
            alarm_code = core_msgs::NaviAlarm::ERROR_FRONT_PIN_SENSOR_BOTH_DETECTED;
        } else if (r_up && r_dn) {
            alarm_code = core_msgs::NaviAlarm::ERROR_REAR_PIN_SENSOR_BOTH_DETECTED;
        } else if ((f_up && r_dn) || (r_up && f_dn)) {
            alarm_code = core_msgs::NaviAlarm::ERROR_PIN_MISMATCH;
        }
    // 우선순위는 
    // 1. pin motor error
    // 2. front pin both detected error
    // 3. rear pin both detected error
    // 4. pin mismatch...가 맞나
    }

    if (alarm_code != 0) {
        if (!b_error_pending_ || pending_error_msg_.data != alarm_code) {
            s_error_.data = alarm_code;
            publishErrorWithDelay(s_error_, 0.5);
        }
    } else {
        b_error_first_published_ = false;
        b_error_pending_ = false;
    }
}


void IOController::publishErrorWithDelay(const std_msgs::Int64& error_msg, double delay_sec) {
    tp_error_start_ = Poco::Timestamp();
    pending_error_msg_ = error_msg;
    b_error_pending_ = true;
    wait_seconds_ = delay_sec;
    // LOG_INFO("[ErrorDelay] Queued error %ld, first publish in %.2f sec", error_msg.data, wait_seconds_);
}

void IOController::checkAndPublishDelayedErrors() {
    if (!b_error_pending_) return;

    Poco::Timestamp now;
    Poco::Timespan elapsed(now - tp_error_start_);
    double elapsed_sec = elapsed.totalSeconds();

    if (!b_error_first_published_) {
        if (elapsed_sec >= wait_seconds_) {
            error_pub_.publish(pending_error_msg_);
            NLOG(info) << "[ErrorDelay] First published error " << pending_error_msg_.data 
                       << " after " << elapsed_sec << " sec delay";
            b_error_first_published_ = true;  // 첫 발행 완료
        }
    }
    else {
        error_pub_.publish(pending_error_msg_);
        // NLOG(info) << "[ErrorDelay] Re-published error " << pending_error_msg_.data << " immediately";
    }
}

}