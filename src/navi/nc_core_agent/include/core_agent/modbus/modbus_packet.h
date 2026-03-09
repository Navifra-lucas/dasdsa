
#ifndef NAVIFRA_MODBUSPACKET_H
#define NAVIFRA_MODBUSPACKET_H

#include <core_agent/modbus/modbus_constants.h>

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace NaviFra {
class ModbusRequest {
public:
    ModbusRequest() {}

    virtual ~ModbusRequest() = default;
    virtual std::vector<uint8_t> toByte() const = 0;
};

class ModbusResponse {
public:
    virtual ~ModbusResponse() = default;

    // 응답 데이터를 파싱하는 순수 가상 함수
    virtual void fromByte(const std::vector<uint8_t>& packet) = 0;
};

class ReadCoilsRequest : public ModbusRequest {
public:
    ReadCoilsRequest(uint8_t slave_address, uint16_t startingAddress, uint16_t nOfCoils)
        : slave_address_(slave_address)
        , starting_address_(startingAddress)
        , n_of_coils_(nOfCoils)
    {
    }

    std::vector<uint8_t> toByte() const override;

private:
    uint8_t slave_address_;
    uint16_t starting_address_;
    uint16_t n_of_coils_;
};

class ReadCoilsResponse : public ModbusResponse {
public:
    const std::vector<bool>& getCoilStatus() const { return coilStatus; }
    void fromByte(const std::vector<uint8_t>& packet) override;

private:
    std::vector<bool> coilStatus;  // Coil 상태를 저장하는 벡터
};

class WriteSingleCoilRequest : public ModbusRequest {
public:
    WriteSingleCoilRequest(uint8_t slave_address, uint16_t address, bool value)
        : slave_address_(slave_address)
        , address_(address)
        , value_(value ? 0xFF00 : 0x0000)
    {
    }  // 0xFF00은 ON, 0x0000은 OFF를 나타냄

    std::vector<uint8_t> toByte() const override;

private:
    uint8_t slave_address_;
    uint16_t address_;
    uint16_t value_;
};

class WriteSingleCoilResponse : public ModbusResponse {
public:
    uint16_t getAddress() const { return address_; }
    bool getValue() const { return value_ == 0xFF00; }  // ON일 경우 true, OFF일 경우 false

    void fromByte(const std::vector<uint8_t>& packet) override;

private:
    uint16_t address_;  // Coil Address
    uint16_t value_;  // Coil Value
};

class WriteMultipleCoilsRequest : public ModbusRequest {
public:
    WriteMultipleCoilsRequest(uint8_t slave_address, uint16_t startingAddress, const std::vector<bool>& values)
        : slave_address_(slave_address)
        , starting_address_(startingAddress)
        , values_(values)
    {
    }

    std::vector<uint8_t> toByte() const override;

private:
    uint8_t slave_address_;
    uint16_t starting_address_;
    std::vector<bool> values_;
};

class WriteMultipleCoilsResponse : public ModbusResponse {
public:
    uint16_t getStartingAddress() const { return starting_address_; }
    uint16_t getQuantityOfCoils() const { return quantity_of_coils_; }

    void fromByte(const std::vector<uint8_t>& packet) override;

private:
    uint16_t starting_address_;  // 시작 주소
    uint16_t quantity_of_coils_;  // 코일 개수
};

class ReadHoldingRegistersRequest : public ModbusRequest {
public:
    ReadHoldingRegistersRequest(uint8_t slave_address, uint16_t startingAddress, uint16_t quantityOfRegisters)
        : slave_address_(slave_address)
        , starting_address_(startingAddress)
        , quantity_of_registers_(quantityOfRegisters)
    {
    }

    std::vector<uint8_t> toByte() const override;

private:
    uint8_t slave_address_;
    uint16_t starting_address_;
    uint16_t quantity_of_registers_;
};

class ReadHoldingRegistersResponse : public ModbusResponse {
public:
    const std::vector<uint16_t>& getRegisterValues() const { return register_values_; }

    void fromByte(const std::vector<uint8_t>& packet) override;

private:
    std::vector<uint16_t> register_values_;
};

class ReadInputRegistersRequest : public ModbusRequest {
public:
    ReadInputRegistersRequest(uint8_t slave_address, uint16_t startingAddress, uint16_t quantityOfRegisters)
        : slave_address_(slave_address)
        , starting_address_(startingAddress)
        , quantity_of_registers_(quantityOfRegisters)
    {
    }

    std::vector<uint8_t> toByte() const override;

private:
    uint8_t slave_address_;
    uint16_t starting_address_;
    uint16_t quantity_of_registers_;
};

class ReadInputRegistersResponse : public ModbusResponse {
public:
    const std::vector<uint16_t>& getRegisterValues() const { return register_values_; }

    void fromByte(const std::vector<uint8_t>& packet);

private:
    std::vector<uint16_t> register_values_;
};

class WriteSingleRegisterRequest : public ModbusRequest {
public:
    WriteSingleRegisterRequest(uint8_t slave_address, uint16_t address, uint16_t value)
        : slave_address_(slave_address)
        , address_(address)
        , value_(value)
    {
    }

    std::vector<uint8_t> toByte() const override;

private:
    uint8_t slave_address_;
    uint16_t address_;
    uint16_t value_;
};

class WriteSingleRegisterResponse : public ModbusResponse {
public:
    uint16_t getAddress() const { return address_; }
    uint16_t getValue() const { return value_; }

    void fromByte(const std::vector<uint8_t>& packet) override;

private:
    uint16_t address_;
    uint16_t value_;
};

class WriteMultipleRegistersRequest : public ModbusRequest {
public:
    WriteMultipleRegistersRequest(uint8_t slave_address, uint16_t startingAddress, const std::vector<uint16_t>& values)
        : slave_address_(slave_address)
        , starting_address_(startingAddress)
        , values_(values)
    {
    }

    std::vector<uint8_t> toByte() const override;

private:
    uint8_t slave_address_;
    uint16_t starting_address_;
    std::vector<uint16_t> values_;
};

class WriteMultipleRegistersResponse : public ModbusResponse {
public:
    uint16_t getStartingAddress() const { return starting_address_; }
    uint16_t getQuantityOfRegisters() const { return quantity_of_registers_; }

    void fromByte(const std::vector<uint8_t>& packet) override;

private:
    uint16_t starting_address_;
    uint16_t quantity_of_registers_;
};

class ReadDiscreteInputsRequest : public ModbusRequest {
public:
    ReadDiscreteInputsRequest(uint8_t slave_address, uint16_t startingAddress, uint16_t nOfInputs)
        : slave_address_(slave_address)
        , starting_address_(startingAddress)
        , n_of_inputs_(nOfInputs)
    {
    }

    std::vector<uint8_t> toByte() const override;

private:
    uint8_t slave_address_;
    uint16_t starting_address_;
    uint16_t n_of_inputs_;
};

class ReadDiscreteInputsResponse : public ModbusResponse {
public:
    const std::vector<bool>& getInputStatus() const { return input_status_; }

    void fromByte(const std::vector<uint8_t>& packet) override;

private:
    std::vector<bool> input_status_;
};

class ModbusPacket {
public:
    static uint16_t calculateCRC(const std::vector<uint8_t>& data)
    {
        uint16_t crc = 0xFFFF;
        for (uint8_t byte : data) {
            crc ^= byte;
            for (int i = 0; i < 8; ++i) {
                if (crc & 0x0001) {
                    crc >>= 1;
                    crc ^= 0xA001;
                }
                else {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }

    static std::vector<uint8_t> appendCRC(const std::vector<uint8_t>& packet)
    {
        std::vector<uint8_t> packet_with_crc = packet;
        uint16_t crc = calculateCRC(packet);
        packet_with_crc.push_back(crc & 0xFF);  // CRC Low Byte
        packet_with_crc.push_back((crc >> 8) & 0xFF);  // CRC High Byte
        return packet_with_crc;
    }

    static bool verifyCRC(const std::vector<uint8_t>& packet)
    {
        if (packet.size() < 2)
            return false;
        uint16_t received_crc = packet[packet.size() - 2] | (packet[packet.size() - 1] << 8);
        std::vector<uint8_t> data_without_crc(packet.begin(), packet.end() - 2);
        uint16_t calculated_crc = calculateCRC(data_without_crc);
        return received_crc == calculated_crc;
    }
};

}  // namespace NaviFra

#endif  // NAVIFRA_MODBUSPACKET_H
