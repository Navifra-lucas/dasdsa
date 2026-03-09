#ifndef NAVIFRA_MODBUS_CONTROLLER_H
#define NAVIFRA_MODBUS_CONTROLLER_H

#include <core_agent/modbus/modbus_packet.h>
#include <core_agent/modbus/port/rtu_port.h>
#include <core_agent/modbus/port/tcp_port.h>

#include <mutex>

namespace NaviFra {
class ModbusController {
public:
    enum class PortType
    {
        TCP,
        RTU
    };

    ModbusController(
        boost::asio::io_context& io_context, PortType portType, const std::string& hostOrDevice, const std::string& portOrBaudrate);

    ~ModbusController();

    std::vector<bool> ReadCoils(uint8_t slaveID, uint16_t startingAddress, uint16_t nOfCoils);

    void WriteCoil(uint8_t slaveID, uint16_t coilAddress, bool value);

    void WriteMultipleCoils(uint8_t slaveID, uint16_t startingAddress, const std::vector<bool>& values);

    std::vector<int16_t> ReadHoldingRegisters(uint8_t slaveID, uint16_t startingAddress, uint16_t nOfHoldingRegister);

    std::vector<int16_t> ReadInputRegisters(uint8_t slaveID, uint16_t startingAddress, uint16_t nOfInputRegisters);

    void WriteSingleRegister(uint8_t slaveID, uint16_t address, uint16_t value);

    void WriteMultipleRegisters(uint8_t slaveID, uint16_t startingAddress, const std::vector<uint16_t>& values);

    std::vector<bool> ReadDiscreteInputs(uint8_t slaveID, uint16_t startingAddress, uint16_t nOfInputs);

private:
    template <typename ResponseType>
    void sendRequest(const ModbusRequest& request, std::function<void(const ResponseType&)> onResponse)
    {
        std::vector<uint8_t> pdu = request.toByte();

        // PDU와 Unit ID를 포트에 전달하여 프레임 작성 및 전송
        port_->sendFrame(pdu, [this, onResponse]() { receiveResponse<ResponseType>(onResponse); });
    }

    template <typename ResponseType>
    void receiveResponse(std::function<void(const ResponseType&)> onResponse, int timeout_ms = 1000)
    {
        auto buffer = std::make_shared<std::vector<uint8_t>>(256);

        // 비동기 읽기 시작
        port_->asyncRead(buffer->size(), [this, buffer, onResponse](const std::vector<uint8_t>& response) {
            if (!response.empty()) {
                // 응답 데이터 파싱 및 콜백 호출
                ResponseType parsed_response;
                parsed_response.fromByte(response);
                onResponse(parsed_response);
            }
            else {
                std::cerr << "Error: Empty response received\n";
                onResponse(ResponseType());  // 에러 시 빈 응답 객체 전달
            }
        });
    }

    boost::asio::io_context& io_context_;
    std::unique_ptr<ModbusPort> port_;
    PortType port_type_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
    std::thread io_thread_;
    std::mutex request_mutex_;
};

}  // namespace NaviFra

#endif  // NAVIFRA_MODBUS_CONTROLLER_H
