#include "nc_io_manager/controller/nc_io_controller.h"

#include "util/logger.hpp"

#include <chrono>

namespace NaviFra {

void IOController::RegisterPinSequence()
{
    PinInitSequence(); // pin init 시퀀스임..
    PinDownSequence(); // pin down 시퀀스임..
    PinUpSequence(); // pin up 시퀀스임..
    Pin3secondAfterUpSequence(); // pin 3s up 시퀀스임..

}

void IOController::PinInitSequence()
{
    // 리프트 초기화 시퀀스 등록
    auto pin_init_seq = std::make_shared<Sequence>("pin_init");
    pin_init_seq->addStep(SequenceStep(
        "pin_up_start", [](std::shared_ptr<DataHandler> handle) { handle->setOutputValue(OUTPUT::PIN, 0x01); },
        [](std::shared_ptr<DataHandler> handle) {
            bool value1 = handle->getPinBit(PIN::FRONT_MOTOR_ON);  // 센서로 도킹 확인이 아니라면 그냥 return true로 해도 될듯
            bool value2 = handle->getPinBit(PIN::REAR_MOTOR_ON);
            return value1 && value2;
        },
        5000, "waiting for moving process"));
    pin_init_seq->addStep(SequenceStep(
        "monitoring pin sensor", nullptr,
        [](std::shared_ptr<DataHandler> handle) {
            bool sensor1 = handle->getPinBit(PIN::FRONT_UP_PX);
            bool sensor2 = handle->getPinBit(PIN::REAR_UP_PX);
            return sensor1 && sensor2;
        },
        15000, "waiting for pin up px sensor on"));
    pin_init_seq->addStep(SequenceStep(
        "stoping pin up", [](std::shared_ptr<DataHandler> handle) { handle->setOutputValue(OUTPUT::PIN, 0x00); },
        [](std::shared_ptr<DataHandler> handle) {
            bool value1 = handle->getPinBit(PIN::FRONT_MOTOR_ON);  
            bool value2 = handle->getPinBit(PIN::REAR_MOTOR_ON);

            return !(value1 && value2);
        }, 
        5000, "waiting for pin complete"));
    pin_init_seq->addStep(SequenceStep(
        "pin_down_start", [](std::shared_ptr<DataHandler> handle) { handle->setOutputValue(OUTPUT::PIN, 0x02); },
        [](std::shared_ptr<DataHandler> handle) {
            bool value1 = handle->getPinBit(PIN::FRONT_MOTOR_ON);  
            bool value2 = handle->getPinBit(PIN::REAR_MOTOR_ON);

            return value1 && value2;
        },
        5000, "waiting for moving process"));
    pin_init_seq->addStep(SequenceStep(
        "monitoring pin sensor", nullptr,
        [](std::shared_ptr<DataHandler> handle) {
            bool sensor1 = handle->getPinBit(PIN::FRONT_DOWN_PX);
            bool sensor2 = handle->getPinBit(PIN::REAR_DOWN_PX);
            return sensor1 && sensor2;
        },
        15000, "waiting for pin down px sensor on"));
    pin_init_seq->addStep(SequenceStep(
        "stoping pin down", [](std::shared_ptr<DataHandler> handle) { handle->setOutputValue(OUTPUT::PIN, 0x00); },
        [](std::shared_ptr<DataHandler> handle) {
            bool value1 = handle->getPinBit(PIN::FRONT_MOTOR_ON);  
            bool value2 = handle->getPinBit(PIN::REAR_MOTOR_ON);

            return !(value1 && value2);
        }, 
        5000, "waiting for pin complete"));
    sequence_controller_->addSequence(pin_init_seq);
}

void IOController::PinDownSequence()
{
    // 리프트 다운 시퀀스 등록
    auto pin_down_seq = std::make_shared<Sequence>("pin_down");
    pin_down_seq->addStep(SequenceStep(
        "pin_down_start", [](std::shared_ptr<DataHandler> handle) { handle->setOutputValue(OUTPUT::PIN, 0x02); },
        [](std::shared_ptr<DataHandler> handle) {
            return true;
        },
        5000, "waiting for moving process"));
    pin_down_seq->addStep(SequenceStep(
        "monitoring pin sensor", nullptr,
        [](std::shared_ptr<DataHandler> handle) {
            bool sensor1 = handle->getPinBit(PIN::FRONT_DOWN_PX);
            bool sensor2 = handle->getPinBit(PIN::REAR_DOWN_PX);
            return sensor1 && sensor2;
        },
        15000, "waiting for pin down px sensor on"));
    pin_down_seq->addStep(SequenceStep(
        "stoping pin down", [](std::shared_ptr<DataHandler> handle) { handle->setOutputValue(OUTPUT::PIN, 0x00); },
        [](std::shared_ptr<DataHandler> handle) { 
            bool value1 = handle->getPinBit(PIN::FRONT_MOTOR_ON);  
            bool value2 = handle->getPinBit(PIN::REAR_MOTOR_ON);

            return !(value1 && value2);
        }, 
        5000,"waiting for pin complete"));
    pin_down_seq->addStep(SequenceStep(
        "publish lift_bottom",
        [this](std::shared_ptr<DataHandler> ) {
            task_msgs::LiftInfo lift_info_msg;
            lift_info_msg.lift_bottom = true;
            lift_status_pub_.publish(lift_info_msg);
            LOG_INFO("[LIFT] pin_down COMPLETED -> publish lift_bottom=true");
        },
        [](std::shared_ptr<DataHandler>) {
            return true;  // 액션만 하고 바로 다음으로
        },
        10,"publish lift bottom info"));
        
    sequence_controller_->addSequence(pin_down_seq);
}

void IOController::PinUpSequence()
{
    // 리프트 업 시퀀스 등록
    auto pin_up_seq = std::make_shared<Sequence>("pin_up");
    pin_up_seq->addStep(SequenceStep(
        "pin_up_start", [](std::shared_ptr<DataHandler> handle) { handle->setOutputValue(OUTPUT::PIN, 0x01); },
        [](std::shared_ptr<DataHandler> handle) {
            bool value1 = handle->getPinBit(PIN::FRONT_MOTOR_ON);  
            bool value2 = handle->getPinBit(PIN::REAR_MOTOR_ON);
            return value1 && value2;
        },
        5000, "waiting for moving process"));
    pin_up_seq->addStep(SequenceStep(
        "monitoring pin sensor", nullptr,
        [](std::shared_ptr<DataHandler> handle) {
            bool sensor1 = handle->getPinBit(PIN::FRONT_UP_PX);
            bool sensor2 = handle->getPinBit(PIN::REAR_UP_PX);
            return sensor1 && sensor2;
        },
        15000, "waiting for pin up sensor on"));
    pin_up_seq->addStep(SequenceStep(
        "stoping pin up", [](std::shared_ptr<DataHandler> handle) { handle->setOutputValue(OUTPUT::PIN, 0x00); },
        [](std::shared_ptr<DataHandler> handle) {
            bool value1 = handle->getPinBit(PIN::FRONT_MOTOR_ON);  
            bool value2 = handle->getPinBit(PIN::REAR_MOTOR_ON);

            return !(value1 && value2);
        }, 
        5000, "waiting for pin complete"));
    pin_up_seq->addStep(SequenceStep(
        "publish lift_top",
        [this](std::shared_ptr<DataHandler> ) {
            task_msgs::LiftInfo lift_info_msg;
            lift_info_msg.lift_top = true;
            lift_status_pub_.publish(lift_info_msg);
            LOG_INFO("[LIFT] pin_up COMPLETED -> publish lift_top=true");
        },
        [](std::shared_ptr<DataHandler>) {
            return true;  // 액션만 하고 바로 다음으로
        },
        10,"publish lift top info"));
    
    sequence_controller_->addSequence(pin_up_seq);
}

void IOController::Pin3secondAfterUpSequence()
{
    auto pin_up_3s_seq = std::make_shared<Sequence>("pin_up_3s");
    
    // 1) 올리기 시작 + 시작시각 기록
    pin_up_3s_seq->addStep(SequenceStep(
        "pin_up_start",
        [this](std::shared_ptr<DataHandler> handle) {
            handle->setOutputValue(OUTPUT::PIN, 0x01);                 // 올림 ON
            pin_up_3s_start_tp_ = std::chrono::steady_clock::now();    // 시작시각 저장
            LOG_INFO("PIN UP 3s: start");
        },
        [this](std::shared_ptr<DataHandler> /*handle*/) {
            // 3초 경과되면 true 반환 -> 다음 스텝으로 진행
            auto now = std::chrono::steady_clock::now();
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - pin_up_3s_start_tp_).count();
            return ms >= 1700;
        },
        4000,  // 타임아웃은 3초보다 살짝 크게 (여유)
        "hold pin up for 3 seconds"
    ));
    
    // 2) 3초 후 정지
    pin_up_3s_seq->addStep(SequenceStep(
        "stopping pin up",
        [](std::shared_ptr<DataHandler> handle) {
            handle->setOutputValue(OUTPUT::PIN, 0x00);                 // 올림 OFF
            LOG_INFO("PIN UP 3s: stop");
        },
        [](std::shared_ptr<DataHandler> /*handle*/) {
            return true; // 바로 완료
        },
        500,
        "stop lifting pin"
    ));
    pin_up_3s_seq->addStep(SequenceStep(
        "publish lift_top",
        [this](std::shared_ptr<DataHandler> ) {
            task_msgs::LiftInfo lift_info_msg;
            lift_info_msg.lift_top = true;
            lift_status_pub_.publish(lift_info_msg);
            LOG_INFO("[LIFT] pin_up COMPLETED -> publish lift_top=true");
        },
        [](std::shared_ptr<DataHandler>) {
            return true;  // 액션만 하고 바로 다음으로
        },
        10,"publish lift top info"));

    
    sequence_controller_->addSequence(pin_up_3s_seq);
}

}