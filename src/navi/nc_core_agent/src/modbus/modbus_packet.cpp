#include "core_agent/core_agent.h"
#include "util/logger.hpp"

#include <core_agent/modbus/modbus_packet.h>

using namespace NaviFra;

std::vector<uint8_t> ReadCoilsRequest::toByte() const
{
    std::vector<uint8_t> packet;

    // Unit ID (1 byte, Slave ID)
    packet.push_back(slave_address_);

    // PDU (Function Code + Starting Address + Number of Coils)
    packet.push_back(static_cast<uint8_t>(FunctionCode::READ_COILS));  // Function code (0x01 for Read Coils)
    packet.push_back((starting_address_ >> 8) & 0xFF);
    packet.push_back(starting_address_ & 0xFF);
    packet.push_back((n_of_coils_ >> 8) & 0xFF);
    packet.push_back(n_of_coils_ & 0xFF);

    return packet;
}

void ReadCoilsResponse::fromByte(const std::vector<uint8_t>& packet)
{
    // 최소 PDU 길이 확인 (Function Code + Byte Count = 2바이트 이상)
    if (packet.size() < 2) {
        throw std::runtime_error("Invalid packet size for Read Coils response");
    }

    // 함수 코드 확인 (0x01 - Read Coils)
    uint8_t functionCode = packet[0];
    if (functionCode != 0x01) {
        throw std::runtime_error("Invalid function code for Read Coils response");
    }

    // Byte Count (1바이트, 두 번째 바이트)
    uint8_t byteCount = packet[1];

    // Coil 상태 파싱
    coilStatus.clear();
    for (size_t i = 0; i < byteCount; ++i) {
        uint8_t byte = packet[2 + i];  // Coil 상태가 시작되는 위치부터 파싱
        for (int bit = 0; bit < 8; ++bit) {
            coilStatus.push_back((byte >> bit) & 0x01);  // 각 비트를 파싱하여 벡터에 저장
        }
    }

    // 필요한 개수만큼의 Coil 상태를 잘라내기
    coilStatus.resize(byteCount * 8);
}

std::vector<uint8_t> WriteSingleCoilRequest::toByte() const
{
    std::vector<uint8_t> packet;
    // Unit ID (1 byte, Slave ID)
    packet.push_back(slave_address_);

    // PDU (Function Code + Coil Address + Coil Value)
    packet.push_back(static_cast<uint8_t>(FunctionCode::WRITE_SINGLE_COIL));  // Function code (0x05 for Write Single Coil)
    packet.push_back((address_ >> 8) & 0xFF);  // Coil Address (high byte)
    packet.push_back(address_ & 0xFF);  // Coil Address (low byte)

    // Coil Value (0xFF00 for ON, 0x0000 for OFF)
    packet.push_back((value_ >> 8) & 0xFF);  // Coil Value (high byte)
    packet.push_back(value_ & 0xFF);  // Coil Value (low byte)

    return packet;
}

void WriteSingleCoilResponse::fromByte(const std::vector<uint8_t>& packet)
{
    // 최소 PDU 길이 확인 (Function Code + Address(2바이트) + Value(2바이트) = 5바이트 이상)
    if (packet.size() < 5) {
        throw std::runtime_error("Invalid packet size for Write Single Coil response");
    }

    // 함수 코드 확인 (0x05 - Write Single Coil)
    uint8_t functionCode = packet[0];
    if (functionCode != 0x05) {
        throw std::runtime_error("Invalid function code for Write Single Coil response");
    }

    // Coil Address와 Value 추출
    address_ = (packet[1] << 8) | packet[2];
    value_ = (packet[3] << 8) | packet[4];
}

std::vector<uint8_t> WriteMultipleCoilsRequest::toByte() const
{
    std::vector<uint8_t> packet;
    // Unit ID (1 byte, Slave ID)
    packet.push_back(slave_address_);

    // PDU (Function Code + Starting Address + Quantity of Coils + Byte Count + Coil Status)
    packet.push_back(static_cast<uint8_t>(FunctionCode::WRITE_MULTIPLE_COILS));  // Function code (0x0F for Write Multiple Coils)
    packet.push_back((starting_address_ >> 8) & 0xFF);  // Starting Address (high byte)
    packet.push_back(starting_address_ & 0xFF);  // Starting Address (low byte)

    // Quantity of Coils (2 bytes)
    packet.push_back((values_.size() >> 8) & 0xFF);  // Coil Count (high byte)
    packet.push_back(values_.size() & 0xFF);  // Coil Count (low byte)

    uint16_t byteCount = (values_.size() + 7) / 8;  // Byte Count to hold all bits
    // Byte Count (1 byte)
    packet.push_back(byteCount);

    // Coil Status - Pack bits into bytes
    for (size_t i = 0; i < byteCount; ++i) {
        uint8_t byte = 0;
        for (int bit = 0; bit < 8; ++bit) {
            if ((i * 8 + bit) < values_.size() && values_[i * 8 + bit]) {
                byte |= (1 << bit);
            }
        }
        packet.push_back(byte);
    }

    return packet;
}

void WriteMultipleCoilsResponse::fromByte(const std::vector<uint8_t>& packet)
{
    // 최소 PDU 길이 확인 (Function Code + Starting Address(2바이트) + Quantity of Coils(2바이트) = 5바이트 이상)
    if (packet.size() < 5) {
        throw std::runtime_error("Invalid packet size for Write Multiple Coils response");
    }

    // 함수 코드 확인 (0x0F - Write Multiple Coils)
    uint8_t functionCode = packet[0];
    if (functionCode != 0x0F) {
        throw std::runtime_error("Invalid function code for Write Multiple Coils response");
    }

    // Starting Address와 Quantity of Coils 추출
    starting_address_ = (packet[1] << 8) | packet[2];
    quantity_of_coils_ = (packet[3] << 8) | packet[4];
}

std::vector<uint8_t> ReadHoldingRegistersRequest::toByte() const
{
    std::vector<uint8_t> packet;
    packet.push_back(slave_address_);
    // Function Code
    packet.push_back(static_cast<uint8_t>(FunctionCode::READ_HOLDING_REGISTERS));

    // Starting Address (2 bytes)
    packet.push_back((starting_address_ >> 8) & 0xFF);
    packet.push_back(starting_address_ & 0xFF);

    // Quantity of Registers (2 bytes)
    packet.push_back((quantity_of_registers_ >> 8) & 0xFF);
    packet.push_back(quantity_of_registers_ & 0xFF);

    return packet;
}

void ReadHoldingRegistersResponse::fromByte(const std::vector<uint8_t>& packet)
{
    if (packet.size() < 2) {
        throw std::runtime_error("Invalid packet size for Read Holding Registers response");
    }

    // 첫 번째 바이트가 함수 코드(0x03)인지 확인
    uint8_t functionCode = packet[0];
    if (functionCode != 0x03) {
        throw std::runtime_error("Invalid function code for Read Holding Registers response");
    }

    // 두 번째 바이트는 데이터의 바이트 수 (byte count)
    uint8_t byteCount = packet[1];
    if (packet.size() < 2 + byteCount) {
        throw std::runtime_error("Packet size does not match byte count for Read Holding Registers response");
    }

    // 레지스터 값 파싱
    register_values_.clear();
    for (size_t i = 0; i < byteCount / 2; ++i) {
        uint16_t regValue = (packet[2 + i * 2] << 8) | packet[3 + i * 2];
        register_values_.push_back(regValue);
    }
}

std::vector<uint8_t> ReadInputRegistersRequest::toByte() const
{
    std::vector<uint8_t> packet;
    packet.push_back(slave_address_);  // Unit ID (슬레이브 주소)

    // PDU (Function Code, Starting Address, Quantity of Registers)
    packet.push_back(static_cast<uint8_t>(FunctionCode::READ_INPUT_REGISTERS));
    packet.push_back((starting_address_ >> 8) & 0xFF);  // Starting Address 상위 바이트
    packet.push_back(starting_address_ & 0xFF);  // Starting Address 하위 바이트
    packet.push_back((quantity_of_registers_ >> 8) & 0xFF);  // Quantity of Registers 상위 바이트
    packet.push_back(quantity_of_registers_ & 0xFF);  // Quantity of Registers 하위 바이트

    return packet;
}

void ReadInputRegistersResponse::fromByte(const std::vector<uint8_t>& packet)
{
    // PDU의 최소 길이 확인: 함수 코드(1바이트) + 바이트 수(1바이트) + 데이터
    if (packet.size() < 2) {
        throw std::runtime_error("Invalid packet size for Read Input Registers response");
    }

    // 첫 번째 바이트가 함수 코드(0x04)인지 확인
    uint8_t functionCode = packet[0];
    if (functionCode != 0x04) {
        throw std::runtime_error("Invalid function code for Read Input Registers response");
    }

    // 두 번째 바이트는 데이터의 바이트 수
    uint8_t byteCount = packet[1];
    if (packet.size() < 2 + byteCount) {
        throw std::runtime_error("Packet size does not match byte count for Read Input Registers response");
    }

    // 레지스터 값 파싱
    register_values_.clear();
    for (size_t i = 0; i < byteCount / 2; ++i) {
        uint16_t regValue = (packet[2 + i * 2] << 8) | packet[3 + i * 2];
        register_values_.push_back(regValue);
    }
}

std::vector<uint8_t> WriteSingleRegisterRequest::toByte() const
{
    std::vector<uint8_t> packet;
    packet.push_back(slave_address_);  // Unit ID (슬레이브 주소)

    // PDU (Function Code + Address + Value)
    packet.push_back(static_cast<uint8_t>(FunctionCode::WRITE_SINGLE_REGISTER));  // Function code (0x06)
    packet.push_back((address_ >> 8) & 0xFF);  // Address 상위 바이트
    packet.push_back(address_ & 0xFF);  // Address 하위 바이트
    packet.push_back((value_ >> 8) & 0xFF);  // Value 상위 바이트
    packet.push_back(value_ & 0xFF);  // Value 하위 바이트

    return packet;
}

void WriteSingleRegisterResponse::fromByte(const std::vector<uint8_t>& packet)
{
    if (packet.size() < 5) {  // 최소 길이 확인 (함수 코드, 주소, 값)
        throw std::runtime_error("Invalid packet size for Write Single Register response");
    }

    // 첫 번째 바이트는 함수 코드 확인
    uint8_t functionCode = packet[0];
    if (functionCode != 0x06) {
        throw std::runtime_error("Invalid function code for Write Single Register response");
    }

    // 주소와 값 파싱
    address_ = (packet[1] << 8) | packet[2];
    value_ = (packet[3] << 8) | packet[4];
}

std::vector<uint8_t> WriteMultipleRegistersRequest::toByte() const
{
    std::vector<uint8_t> packet;
    packet.push_back(slave_address_);  // Unit ID

    // PDU (기능 코드 + 시작 주소 + 레지스터 개수 + 바이트 수 + 레지스터 값들)
    packet.push_back(static_cast<uint8_t>(FunctionCode::WRITE_MULTIPLE_REGISTERS));  // 기능 코드 (0x10)
    packet.push_back((starting_address_ >> 8) & 0xFF);  // 시작 주소 상위 바이트
    packet.push_back(starting_address_ & 0xFF);  // 시작 주소 하위 바이트
    packet.push_back((values_.size() >> 8) & 0xFF);  // 레지스터 개수 상위 바이트
    packet.push_back(values_.size() & 0xFF);  // 레지스터 개수 하위 바이트

    uint8_t byteCount = static_cast<uint8_t>(values_.size() * 2);  // 바이트 수 (레지스터 값의 전체 길이)
    packet.push_back(byteCount);

    // 레지스터 값 추가
    for (uint16_t value : values_) {
        packet.push_back((value >> 8) & 0xFF);  // 레지스터 값 상위 바이트
        packet.push_back(value & 0xFF);  // 레지스터 값 하위 바이트
    }

    return packet;
}

void WriteMultipleRegistersResponse::fromByte(const std::vector<uint8_t>& packet)
{
    // 응답 패킷의 최소 길이 확인 (기능 코드 + 시작 주소 + 레지스터 개수)
    if (packet.size() < 5) {
        throw std::runtime_error("Invalid packet size for Write Multiple Registers response");
    }

    // 기능 코드 확인
    uint8_t functionCode = packet[0];
    if (functionCode != static_cast<uint8_t>(FunctionCode::WRITE_MULTIPLE_REGISTERS)) {
        NLOG(error)<<"Invalid function code for Write Multiple Registers response "<<functionCode;
        throw std::runtime_error("Invalid function code for Write Multiple Registers response");
    }

    // PDU 데이터 추출 (시작 주소와 레지스터 개수)
    starting_address_ = (packet[1] << 8) | packet[2];
    quantity_of_registers_ = (packet[3] << 8) | packet[4];
}

std::vector<uint8_t> ReadDiscreteInputsRequest::toByte() const
{
    std::vector<uint8_t> packet;
    packet.push_back(slave_address_);
    packet.push_back(static_cast<uint8_t>(FunctionCode::READ_DISCRETE_INPUTS));  // Function Code 0x02
    packet.push_back((starting_address_ >> 8) & 0xFF);  // Starting Address (High Byte)
    packet.push_back(starting_address_ & 0xFF);  // Starting Address (Low Byte)
    packet.push_back((n_of_inputs_ >> 8) & 0xFF);  // Number of Inputs (High Byte)
    packet.push_back(n_of_inputs_ & 0xFF);  // Number of Inputs (Low Byte)
    return packet;
}

void ReadDiscreteInputsResponse::fromByte(const std::vector<uint8_t>& packet)
{
    if (packet.size() < 1) {
        throw std::runtime_error("Invalid packet size for Read Discrete Inputs response");
    }

    // 기능 코드 확인
    uint8_t functionCode = packet[0];
    if (functionCode != static_cast<uint8_t>(FunctionCode::READ_DISCRETE_INPUTS)) {
        NLOG(error) << "Invalid function code for Read Discrete Inputs response " << functionCode;
        throw std::runtime_error("Invalid function code for Read Discrete Inputs response");
    }

    uint8_t byteCount = packet[1];  // 첫 번째 바이트는 바이트 수
    if (packet.size() < 1 + byteCount) {
        throw std::runtime_error("Packet size does not match byte count for Read Discrete Inputs response");
    }

    input_status_.clear();
    for (size_t i = 0; i < byteCount; ++i) {
        uint8_t byte = packet[2 + i];
        for (int bit = 0; bit < 8; ++bit) {
            input_status_.push_back((byte >> bit) & 0x01);
        }
    }
}
