#include "nc_io_manager/controller/nc_io_controller.h"

#include "util/logger.hpp"

#include <chrono>

namespace NaviFra {

void IOController::RegisterControllerSequence()
{
    ManualModeOnSequence();
    ManualModeOffSequence();
    EquipmentSendSequence();
    OssdByPassOnSequence();
    OssdByPassOffSequence();
    LidarOssdEnableSequence();
    LidarOssdDisableSequence();
    PlcInfoSequence();

}

void IOController::ManualModeOnSequence()
{
     // manual mode 시퀀스 등록
    auto manual_mode_on_seq = std::make_shared<Sequence>("manual_mode_on");
    manual_mode_on_seq->addStep(SequenceStep(
        "manual_mode_on", [](std::shared_ptr<DataHandler> handle) { handle->setCommandBit(COMMAND::MANUAL_MODE_ON, true); },
        [](std::shared_ptr<DataHandler> handle) { return true; }, 5000,
        "waiting for manual mode on"));

    sequence_controller_->addSequence(manual_mode_on_seq); //plc 통신 끊겼을 때 어떻게 할건지
}

// manual_mode_on_seq -> manual_mode_off_seq로 변경 
void IOController::ManualModeOffSequence()
{
    auto manual_mode_off_seq = std::make_shared<Sequence>("manual_mode_off");
    manual_mode_off_seq->addStep(SequenceStep(
        "manual_mode_on", [](std::shared_ptr<DataHandler> handle) { handle->setCommandBit(COMMAND::MANUAL_MODE_ON, false); },
        [](std::shared_ptr<DataHandler> handle) { return true; }, 5000,
        "waiting for manual mode off"));

    sequence_controller_->addSequence(manual_mode_off_seq); //plc 통신 끊겼을 때 어떻게 할건지
}

void IOController::EquipmentSendSequence()
{
    // KIA EVO PJT 시퀀스 등록
    auto equipment_send_seq = std::make_shared<Sequence>("equipment_send");
    equipment_send_seq->addStep(SequenceStep(
        "equipment_send", [](std::shared_ptr<DataHandler> handle) { handle->setOutputValue(OUTPUT::EQUIPMENT1, 0x0B); },
        [](std::shared_ptr<DataHandler> handle) { return true; }, 5000,
        "waiting for equipment send"));

    sequence_controller_->addSequence(equipment_send_seq); //plc 통신 끊겼을 때 어떻게 할건지
}

void IOController::OssdByPassOnSequence()
{
    //ipc - plc 연결 끊겼을 때를 대비한 ossd bypass
    auto ossd_bypass_seq = std::make_shared<Sequence>("ossd_bypass_on");
    ossd_bypass_seq->addStep(SequenceStep(
        "ossd_bypass_on", [](std::shared_ptr<DataHandler> handle) { handle->setCommandBit2(COMMAND2::OSSD_BYPASS, true); },
        [](std::shared_ptr<DataHandler> handle) { return true; }, 5000,
        "waiting for ossd bypass on"));
    sequence_controller_->addSequence(ossd_bypass_seq);
}

void IOController::OssdByPassOffSequence()
{
    auto ossd_bypass_off_seq = std::make_shared<Sequence>("ossd_bypass_off");
    ossd_bypass_off_seq->addStep(SequenceStep(
        "ossd_bypass_off", [](std::shared_ptr<DataHandler> handle) { handle->setCommandBit2(COMMAND2::OSSD_BYPASS, false); },
        [](std::shared_ptr<DataHandler> handle) { return true; }, 5000,
        "waiting for ossd bypass off"));
    sequence_controller_->addSequence(ossd_bypass_off_seq);
}

void IOController::LidarOssdEnableSequence()
{
    // 통신 상관없는 ossd enable
    auto lidar_ossd_enable_seq = std::make_shared<Sequence>("lidar_ossd_enable");
    lidar_ossd_enable_seq->addStep(SequenceStep(
        "lidar_ossd_enable", [](std::shared_ptr<DataHandler> handle) { handle->setCommandBit2(COMMAND2::LIDAR_OSSD_ENABLE, true); },
        [](std::shared_ptr<DataHandler> handle) { return true; }, 5000,
        "waiting for lidar ossd enable on"));
    sequence_controller_->addSequence(lidar_ossd_enable_seq);
}

void IOController::LidarOssdDisableSequence()
{
    auto lidar_ossd_disable_seq = std::make_shared<Sequence>("lidar_ossd_disable");
    lidar_ossd_disable_seq->addStep(SequenceStep(
        "lidar_ossd_disable", [](std::shared_ptr<DataHandler> handle) { handle->setCommandBit2(COMMAND2::LIDAR_OSSD_ENABLE, false); },
        [](std::shared_ptr<DataHandler> handle) { return true; }, 5000,
        "waiting for lidar ossd enable off"));
    sequence_controller_->addSequence(lidar_ossd_disable_seq);
}

void IOController::PlcInfoSequence()
{
    auto plc_info_send_seq = std::make_shared<Sequence>("plc_info_send");
    plc_info_send_seq->addStep(SequenceStep(
        "plc_info_send",
        [this](std::shared_ptr<DataHandler> handle) {

            PlcInfoData info = handle->getAllPlcInfo();

            core_msgs::PLCInfo plc_msg;
            
            // 안전 / 비상
            plc_msg.front_bumper_switch     = info.front_bumper;
            plc_msg.rear_bumper_switch      = info.rear_bumper;
            plc_msg.emergency               = info.emergency_button;
            plc_msg.ossd_signal             = info.ossd_signal;
            plc_msg.sto_signal              = info.sto_signal;
            plc_msg.brake_release_feedback  = info.brake_feedback;
            plc_msg.manual_charge_on        = info.manual_charge_on;
            plc_msg.quick_stop_status       = info.quick_stop_status;

            // 버튼
            plc_msg.reset_switch            = info.reset_switch;
            plc_msg.brake_release_switch    = info.brake_switch;
            plc_msg.io_mode_select_1        = info.mode_select_1;
            plc_msg.io_mode_select_2        = info.mode_select_2;

            // 센서 / 릴레이
            plc_msg.drive_power_relay_signal = info.drive_power_relay;
            plc_msg.rear_charge_relay_signal = info.rear_charge_relay;

            // 핀 / 리프트
            plc_msg.front_up_px             = info.front_up_px;
            plc_msg.front_down_px           = info.front_down_px;
            plc_msg.rear_up_px              = info.rear_up_px;
            plc_msg.rear_down_px            = info.rear_down_px;
            plc_msg.front_motor_state       = info.front_motor_on;
            plc_msg.rear_motor_state        = info.rear_motor_on;
            plc_msg.front_motor_alarm       = info.front_motor_alarm;
            plc_msg.rear_motor_alarm        = info.rear_motor_alarm;

            // IO
            plc_msg.safety_input_io_ib0     = info.io_ib0;
            plc_msg.safety_input_io_qb6     = info.io_qb6;
            plc_msg.safety_input_io_qb12    = info.io_qb12;
            plc_msg.safety_input_io_qb18    = info.io_qb18;
            plc_msg.safety_input_io_ib24    = info.io_ib24;
            plc_msg.safety_input_io_ib25    = info.io_ib25;
            plc_msg.safety_input_io_qb26    = info.io_qb26;

            // 버전
            plc_msg.version_major           = info.version_major;
            plc_msg.version_minor           = info.version_minor;
            plc_msg.version_patch           = info.version_patch;
            plc_msg.plc_version = std::to_string(info.version_major) + "." + std::to_string(info.version_minor) + "." + std::to_string(info.version_patch);

            plc_info_pub_.publish(plc_msg);

            if (info.reset_switch) {
                std_msgs::String alarm_clear_msg;
                alarm_clear_msg.data = "alarm_clear";
                cmd_pub_.publish(alarm_clear_msg);
            }
        },
        [](std::shared_ptr<DataHandler> handle) { return true; }, 5000,
        "waiting for equipment send"
    ));

    sequence_controller_->addSequence(plc_info_send_seq);
}
}
