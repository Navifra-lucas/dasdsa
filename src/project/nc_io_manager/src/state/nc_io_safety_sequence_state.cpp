#include "nc_io_manager/controller/nc_io_controller.h"

#include "util/logger.hpp"

#include <chrono>

namespace NaviFra {

void IOController::RegisterSafetySequence()
{
    BrakeReleaseOnSequence();
    BrakeReleaseOffSequence();
    QuickStopOnSequence();
    QuickStopOffSequence();
    StoCheckSequence();
    StoClearOnSequence();
    StoClearOffSequence();
    BumperClearOffSequence();
    BumperClearOnSequence();

}

void IOController::BrakeReleaseOnSequence()
{
    // brake release 시퀀스 등록
    auto brake_release_on_seq = std::make_shared<Sequence>("brake_release_on");
    brake_release_on_seq->addStep(SequenceStep(
        "brake_release_on", [](std::shared_ptr<DataHandler> handle) { handle->setCommandBit(COMMAND::BRAKE, true); },
        [](std::shared_ptr<DataHandler> handle) { return handle->getSafetyBit(SAFETY::BRAKE_RELEASE); }, 5000,
        "waiting for brake release on"));

    sequence_controller_->addSequence(brake_release_on_seq); //plc 통신 끊겼을 때 어떻게 할건지
}

void IOController::BrakeReleaseOffSequence()
{
    auto brake_release_off_seq = std::make_shared<Sequence>("brake_release_off");
    brake_release_off_seq->addStep(SequenceStep(
        "brake_release_off", [](std::shared_ptr<DataHandler> handle) { handle->setCommandBit(COMMAND::BRAKE, false); },
        [](std::shared_ptr<DataHandler> handle) { return !(handle->getSafetyBit(SAFETY::BRAKE_RELEASE)); }, 5000,
        "waiting for brake release off"));

    sequence_controller_->addSequence(brake_release_off_seq); 
}

void IOController::QuickStopOnSequence()
{
    // quick stop 시퀀스 등록
    auto quick_stop_on_seq = std::make_shared<Sequence>("quick_stop_on");
    quick_stop_on_seq->addStep(SequenceStep(
        "quick_stop_on", [](std::shared_ptr<DataHandler> handle) { handle->setCommandBit2(COMMAND2::QUICK_STOP, true); },
        [](std::shared_ptr<DataHandler> handle) { return handle->getSafetyBit(SAFETY::QUICK_STOP_SIGNAL); }, 5000,
        "waiting for quick stop on"));

    sequence_controller_->addSequence(quick_stop_on_seq); //plc 통신 끊겼을 때 어떻게 할건지
}

void IOController::QuickStopOffSequence()
{
    auto quick_stop_off_seq = std::make_shared<Sequence>("quick_stop_off");
    quick_stop_off_seq->addStep(SequenceStep(
        "quick_stop_off", [](std::shared_ptr<DataHandler> handle) { handle->setCommandBit2(COMMAND2::QUICK_STOP, false); },
        [](std::shared_ptr<DataHandler> handle) { return !(handle->getSafetyBit(SAFETY::QUICK_STOP_SIGNAL)); }, 5000,
        "waiting for quick stop off"));

    sequence_controller_->addSequence(quick_stop_off_seq); //plc 통신 끊겼을 때 어떻게 할건지
}

void IOController::StoCheckSequence()
{
    auto sto_check_seq = std::make_shared<Sequence>("sto_check");
    sto_check_seq->addStep(SequenceStep(
        "sto_check", [](std::shared_ptr<DataHandler> handle) { handle->setCommandBit2(COMMAND2::STO_CHECK, true); },
        [](std::shared_ptr<DataHandler> handle) { return true; }, 5000,
        "waiting for sto_check on"));

    sto_check_seq->addStep(SequenceStep(
        "sto_check", [](std::shared_ptr<DataHandler> handle) { handle->setCommandBit2(COMMAND2::STO_CHECK, false); },
        [](std::shared_ptr<DataHandler> handle) { return true; }, 5000,
        "waiting for sto_check off"));
    

    sequence_controller_->addSequence(sto_check_seq); //plc 통신 끊겼을 때 어떻게 할건지
}

// manual_mode_on_seq -> sto_clear_on_seq 으로 변경 
void IOController::StoClearOnSequence()
{
    // sto clear 시퀀스 등록
    auto sto_clear_on_seq = std::make_shared<Sequence>("sto_clear_on");
    sto_clear_on_seq->addStep(SequenceStep(
        "sto_clear_on", [](std::shared_ptr<DataHandler> handle) { handle->setCommandBit(COMMAND::STO_CLEAR, true); },
        [](std::shared_ptr<DataHandler> handle) { return handle->getSafetyBit(SAFETY::STO_SIGNAL); }, 5000,
        "waiting for manual mode on"));

    sequence_controller_->addSequence(sto_clear_on_seq); //plc 통신 끊겼을 때 어떻게 할건지
}

void IOController::StoClearOffSequence()
{
    auto sto_clear_off_seq = std::make_shared<Sequence>("sto_clear_off");
    sto_clear_off_seq->addStep(SequenceStep(
        "manual_mode_on", [](std::shared_ptr<DataHandler> handle) { handle->setCommandBit(COMMAND::STO_CLEAR, false); },
        [](std::shared_ptr<DataHandler> handle) { return !(handle->getSafetyBit(SAFETY::STO_SIGNAL)); }, 5000,
        "waiting for manual mode off"));

    sequence_controller_->addSequence(sto_clear_off_seq); //plc 통신 끊겼을 때 어떻게 할건지
}

void IOController::BumperClearOffSequence()
{
    auto bumper_clear_off_seq = std::make_shared<Sequence>("bumper_clear_off");
    bumper_clear_off_seq->addStep(SequenceStep(
        "bumper_clear_off", [](std::shared_ptr<DataHandler> handle) { handle->setCommandBit2(COMMAND2::BUMPER_CLEAR, false); },
        [](std::shared_ptr<DataHandler> handle) { 
            bool value1 = handle->getSafetyBit(SAFETY::FRONT_BUMPPER);
            bool value2 = handle->getSafetyBit(SAFETY::REAR_BUMPER);
            return !(value1 || value2); 
        }, 
        5000, "waiting for bumper clear mode off"));

    sequence_controller_->addSequence(bumper_clear_off_seq);
}


void IOController::BumperClearOnSequence()
{
    auto bumper_clear_on_seq = std::make_shared<Sequence>("bumper_clear_on");
    bumper_clear_on_seq->addStep(SequenceStep(
        "bumper_clear_on", [](std::shared_ptr<DataHandler> handle) { handle->setCommandBit2(COMMAND2::BUMPER_CLEAR, true); },
        [](std::shared_ptr<DataHandler> handle) { 
            bool value1 = handle->getSafetyBit(SAFETY::FRONT_BUMPPER);
            bool value2 = handle->getSafetyBit(SAFETY::REAR_BUMPER);
            return (value1 && value2); 
        }, 
        5000, "waiting for bumper clear mode off"));

    sequence_controller_->addSequence(bumper_clear_on_seq);
}

}
