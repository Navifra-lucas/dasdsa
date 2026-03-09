#include "nc_io_manager/data/data_handler.h"

#include "util/logger.hpp"

#include <ros/ros.h>

namespace NaviFra {

DataHandler::DataHandler()
{
    // 입력 맵 초기화 (64개 채널)
    for (int i = 0; i < 64; ++i) {
        input_[i] = 0;
    }

    // 출력 맵 초기화 (64개 채널)
    for (int i = 0; i < 64; ++i) {
        output_[i] = 0;
    }

    LOG_INFO("DataHandler initialized");
}

DataHandler::~DataHandler()
{
    LOG_INFO("DataHandler destroyed");
}

std::array<u_int8_t, IO_DATA_SIZE> DataHandler::getInputArray() const
{
    return mapToArray(input_);
}

std::array<u_int8_t, IO_DATA_SIZE> DataHandler::getOutputArray() const
{
    return mapToArray(output_);
}

void DataHandler::setInputArray(const std::array<u_int8_t, IO_DATA_SIZE>& input)
{
    input_ = arrayToMap(input);
}

void DataHandler::setOutputArray(const std::array<u_int8_t, IO_DATA_SIZE>& output)
{
    output_ = arrayToMap(output);
}

bool DataHandler::getSafetyBit(SAFETY safetyBit)
{
    auto data = getInputValue(INPUT::SAFETY);
    return (data & (1 << static_cast<int>(safetyBit))) != 0;
}

bool DataHandler::getButtonBit(BUTTON buttonBit)
{
    auto data = getInputValue(INPUT::BUTTON);
    return (data & (1 << static_cast<int>(buttonBit))) != 0;
}

bool DataHandler::getHWStateBit(HW_STATE stateBit)
{
    auto data = getInputValue(INPUT::STATE_INFO);
    return (data & (1 << static_cast<int>(stateBit))) != 0;
}

bool DataHandler::getSensorBit(SENSOR sensorBit)
{
    u_int8_t data;
    int bitPosition = static_cast<int>(sensorBit);

    if (sensorBit < SENSOR::FRONT_MAGNET_SENSOR) {
        data = getInputValue(INPUT::SENSOR1);
    }
    else {
        data = getInputValue(INPUT::SENSOR2);
        bitPosition -= static_cast<int>(SENSOR::FRONT_MAGNET_SENSOR);
    }
    return (data & (1 << bitPosition)) != 0;
}

bool DataHandler::getPinBit(PIN pinBit)
{
    auto data = getInputValue(INPUT::PIN);
    return (data & (1 << static_cast<int>(pinBit))) != 0;
}

bool DataHandler::getLiftBit(LIFT liftBit)
{
    auto data = getInputValue(INPUT::LIFT);
    return (data & (1 << static_cast<int>(liftBit))) != 0;
}

bool DataHandler::getLiftTurnBit(LIFT_TURN liftTurnBit)
{
    auto data = getInputValue(INPUT::LIFT_TURN);
    return (data & (1 << static_cast<int>(liftTurnBit))) != 0;
}

bool DataHandler::getChargeRelayBit(CHARGE chargeRelayBit)
{
    auto data = getInputValue(INPUT::CHARGE_RELAY);
    return (data & (1 << static_cast<int>(chargeRelayBit))) != 0;
}

bool DataHandler::getConveyorBit(CONVEYOR conveyorBit)
{
    u_int8_t data;
    int bitPosition = static_cast<int>(conveyorBit);

    if (conveyorBit < CONVEYOR::CONVEYOR_ALARM) {
        data = getInputValue(INPUT::CONVEYOR1);
    }
    else if (conveyorBit >= CONVEYOR::CONVEYOR_ALARM && conveyorBit < CONVEYOR::LOAD_COMPLETE_FEEDBACK) {
        data = getInputValue(INPUT::CONVEYOR2);
        bitPosition -= static_cast<int>(CONVEYOR::CONVEYOR_ALARM);
    }
    else {
        data = getInputValue(INPUT::CONVEYOR3);
        bitPosition -= static_cast<int>(CONVEYOR::LOAD_COMPLETE_FEEDBACK);
    }
    return (data & (1 << bitPosition)) != 0;
}

void DataHandler::setCommandBit(COMMAND cmd, bool value)
{
    auto data = getOutputValue(OUTPUT::COMMAND);
    int bit_position = static_cast<int>(cmd);

    if (value) {
        // 비트 설정 (OR 연산)
        data |= (1 << bit_position);
    }
    else {
        // 비트 클리어 (AND + NOT 연산)
        data &= ~(1 << bit_position);
    }

    setOutputValue(OUTPUT::COMMAND, data);  // 수정된 값 다시 저장
}

void DataHandler::setCommandBit2(COMMAND2 cmd, bool value)
{
    auto data = getOutputValue(OUTPUT::COMMAND2);
    int bit_position = static_cast<int>(cmd);

    if (value) {
        // 비트 설정 (OR 연산)
        data |= (1 << bit_position);
    }
    else {
        // 비트 클리어 (AND + NOT 연산)
        data &= ~(1 << bit_position);
    }

    setOutputValue(OUTPUT::COMMAND2, data);  // 수정된 값 다시 저장
}

bool DataHandler::getCommandBit(COMMAND cmd)
{
    auto data = getOutputValue(OUTPUT::COMMAND);
    int n_bit_position = static_cast<int>(cmd);
    return (data & (1 << n_bit_position)) != 0;
}

bool DataHandler::getCommandBit2(COMMAND2 cmd)
{
    auto data = getOutputValue(OUTPUT::COMMAND2);
    int n_bit_position = static_cast<int>(cmd);
    return (data & (1 << n_bit_position)) != 0;
}

u_int8_t DataHandler::getInputValue(INPUT input_type)
{
    int channel = static_cast<int>(input_type);
    auto it = input_.find(channel);
    if (it != input_.end()) {
        return it->second;
    }
    return 0;
}

u_int8_t DataHandler::getOutputValue(OUTPUT output_type)
{
    int channel = static_cast<int>(output_type);
    auto it = output_.find(channel);
    if (it != output_.end()) {
        return it->second;
    }
    return 0;
}

void DataHandler::setInputValue(INPUT input_type, u_int8_t value)
{
    int channel = static_cast<int>(input_type);
    input_[channel] = value;
}

void DataHandler::setOutputValue(OUTPUT output_type, u_int8_t value)
{
    int channel = static_cast<int>(output_type);
    output_[channel] = value;
}

std::array<u_int8_t, IO_DATA_SIZE> DataHandler::mapToArray(const std::map<int, u_int8_t>& map) const
{
    std::array<u_int8_t, IO_DATA_SIZE> result;
    result.fill(0);  // 모든 바이트를 0으로 초기화

    for (const auto& pair : map) {
        int index = pair.first;
        uint8_t value = pair.second;

        // 범위 체크
        if (index >= 0 && index < IO_DATA_SIZE) {
            result[index] = value;
        }
    }

    return result;
}

std::map<int, u_int8_t> DataHandler::arrayToMap(const std::array<u_int8_t, IO_DATA_SIZE>& array)
{
    std::map<int, u_int8_t> result;

    for (size_t i = 0; i < IO_DATA_SIZE; ++i) {
        result[static_cast<int>(i)] = array[i];
    }

    return result;
}

PlcInfoData DataHandler::getAllPlcInfo() {
    PlcInfoData data;

    // 안전
    data.front_bumper      = getSafetyBit(SAFETY::FRONT_BUMPPER);
    data.rear_bumper       = getSafetyBit(SAFETY::REAR_BUMPER);
    data.emergency_button  = getSafetyBit(SAFETY::EMERGENCY);
    data.ossd_signal       = getSafetyBit(SAFETY::OSSD_SIGNAL);
    data.sto_signal        = getSafetyBit(SAFETY::STO_SIGNAL);
    data.brake_feedback    = getSafetyBit(SAFETY::BRAKE_RELEASE);
    data.manual_charge_on  = getSafetyBit(SAFETY::MANUAL_CHARGE);
    data.quick_stop_status = getSafetyBit(SAFETY::QUICK_STOP_SIGNAL);

    // 버튼
    data.reset_switch      = getButtonBit(BUTTON::RESET);
    data.brake_switch      = getButtonBit(BUTTON::BRAKE);
    data.mode_select_1     = getButtonBit(BUTTON::MODE_SELECTE_1);
    data.mode_select_2     = getButtonBit(BUTTON::MODE_SELECTE_2);

    // 센서 / 릴레이
    data.drive_power_relay = getSensorBit(SENSOR::DRIVE_POWER_RELAY_CHECK_SIGNAL);
    data.rear_charge_relay = getSensorBit(SENSOR::REAR_CHARGE_RELAY_CHECK_SIGNAL);

    // 핀 / 리프트
    data.front_up_px       = getPinBit(PIN::FRONT_UP_PX);
    data.front_down_px     = getPinBit(PIN::FRONT_DOWN_PX);
    data.rear_up_px        = getPinBit(PIN::REAR_UP_PX);
    data.rear_down_px      = getPinBit(PIN::REAR_DOWN_PX);
    data.front_motor_on    = getPinBit(PIN::FRONT_MOTOR_ON);
    data.rear_motor_on     = getPinBit(PIN::REAR_MOTOR_ON);
    data.front_motor_alarm = getPinBit(PIN::FRONT_MOTOR_ALARM);
    data.rear_motor_alarm  = getPinBit(PIN::REAR_MOTOR_ALARM);

    // IO
    data.io_ib0  = static_cast<uint8_t>(getInputValue(INPUT::IO1));
    data.io_qb6  = static_cast<uint8_t>(getInputValue(INPUT::IO2));
    data.io_qb12 = static_cast<uint8_t>(getInputValue(INPUT::IO3));
    data.io_qb18 = static_cast<uint8_t>(getInputValue(INPUT::IO4));
    data.io_ib24 = static_cast<uint8_t>(getInputValue(INPUT::IO5));
    data.io_ib25 = static_cast<uint8_t>(getInputValue(INPUT::IO6));
    data.io_qb26 = static_cast<uint8_t>(getInputValue(INPUT::IO7));

    // 버전
    data.version_major = static_cast<uint8_t>(getInputValue(INPUT::VERSION_MAJOR));
    data.version_minor = static_cast<uint8_t>(getInputValue(INPUT::VERSION_MINOR));
    data.version_patch = static_cast<uint8_t>(getInputValue(INPUT::VERSION_PATCH));

    return data;
}


}  // namespace NaviFra