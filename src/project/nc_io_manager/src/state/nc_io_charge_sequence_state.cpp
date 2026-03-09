#include "nc_io_manager/controller/nc_io_controller.h"

#include "util/logger.hpp"

#include <chrono>

namespace NaviFra {

void IOController::RegisterChargingSequence()
{
    ChargingStartSequence(); // 충전 시작 시퀀스임..
    ChargingStopSequence(); // 충전 종료 시퀀스임..
    ChargingResetSequence(); // 충전 리셋 시퀀스임..
    ChargingResetFalseSequence(); // 충전 리셋 false? 이건 뭐임 시퀀스임..

}

void IOController::ChargingStartSequence()
{
    // 충전 시퀀스 등록
    auto charge_start_seq = std::make_shared<Sequence>("charge_start");
    charge_start_seq->addStep(SequenceStep(
        "charge_start", [this](std::shared_ptr<DataHandler> handle) { 
            handle->setCommandBit(COMMAND::CHARGE_RELAY_REAR, true);
            LOG_INFO("[CHARGE_START] Command Charge Start");
            },
        [](std::shared_ptr<DataHandler> handle) {
            bool value = handle->getSensorBit(SENSOR::REAR_CHARGE_RELAY_CHECK_SIGNAL);
            return value;
        },
        5000, "waiting for charging start"));

    charge_start_seq->addStep(SequenceStep(
        "charge_current_checking", [this](std::shared_ptr<DataHandler> handle) { 
            LOG_INFO("[CHARGE_CURRENT_START] Current Success");
            },
        [this](std::shared_ptr<DataHandler> handle) {
            if(f_pack_current_ > 10.0)
            {
                NLOG(info) << "[PP] Pack Current 10A 이상 흐르는중... " << f_pack_current_;
                return true;
            }
            return false;
        },
        40000, "waiting for charging current.."));

    sequence_controller_->addSequence(charge_start_seq);
}

void IOController::ChargingStopSequence()
{
    // --- charge_stop 시퀀스 ---
    auto charge_stop_seq = std::make_shared<Sequence>("charge_stop");

    // 1) 필요하면 끄기 명령
    charge_stop_seq->addStep(SequenceStep(
        "maybe_turn_off",
        [](std::shared_ptr<DataHandler> handle) {
            bool relay_on = handle->getSensorBit(SENSOR::REAR_CHARGE_RELAY_CHECK_SIGNAL);
            LOG_INFO("[CHARGE_STOP] step1 relay_on=%d", relay_on);
            if (relay_on) {
                handle->setCommandBit(COMMAND::CHARGE_RELAY_REAR, false);
                LOG_INFO("[CHARGE_STOP] command OFF sent");
            } else {
                LOG_INFO("[CHARGE_STOP] already OFF, will pass through");
            }
        },
        [](std::shared_ptr<DataHandler>) {
            return true; // 바로 다음 스텝으로
        },
        1000,   // 타임아웃 없앰!
        "issue OFF command if relay is ON"
    ));
    
    // 2) 릴레이 OFF 될 때까지 대기 (이미 OFF면 즉시 통과)
    charge_stop_seq->addStep(SequenceStep(
        "wait_relay_off",
        nullptr,
        [](std::shared_ptr<DataHandler> handle) {
            LOG_INFO("[CHARGE_STOP] step2 checking relay OFF...");
            return !handle->getSensorBit(SENSOR::REAR_CHARGE_RELAY_CHECK_SIGNAL);
        },
        5000,  // 필요시 조정
        "waiting for charging stop (relay OFF)"
    ));

    // 3) 완료 알림 퍼블리시
    charge_stop_seq->addStep(SequenceStep(
        "publish_charge_stop_done",
        [this](std::shared_ptr<DataHandler>) {
            LOG_INFO("[CHARGE_STOP] relay OFF confirmed -> publish charge_state=true");
        },
        [](std::shared_ptr<DataHandler>) {
            return true;
        },
        1000,
        "notify charge stop done"
    ));

    sequence_controller_->addSequence(charge_stop_seq);

}

void IOController::ChargingResetSequence()
{
    // 충전 시퀀스 등록
    auto reset_seq = std::make_shared<Sequence>("reset");
    reset_seq->addStep(SequenceStep(
        "reset", [](std::shared_ptr<DataHandler> handle) { handle->setCommandBit(COMMAND::REMOTE_RESET, true); },
        [](std::shared_ptr<DataHandler> handle) {
            return true;
        },
        5000, "waiting for charging start"));
        reset_seq->addStep(SequenceStep(
        "reset", [](std::shared_ptr<DataHandler> handle) { handle->setCommandBit(COMMAND::REMOTE_RESET, false); },
        [](std::shared_ptr<DataHandler> handle) {
            return true;
        },
        5000, "waiting for charging start"));
    
    sequence_controller_->addSequence(reset_seq);

}
void IOController::ChargingResetFalseSequence()
{
    auto reset_fasle_seq = std::make_shared<Sequence>("reset_false");
    reset_fasle_seq->addStep(SequenceStep(
        "reset_false", [](std::shared_ptr<DataHandler> handle) { handle->setCommandBit(COMMAND::REMOTE_RESET, false); },
        [](std::shared_ptr<DataHandler> handle) {
            return true;
        },
        5000, "waiting for charging start"));

    sequence_controller_->addSequence(reset_fasle_seq);

}
}
