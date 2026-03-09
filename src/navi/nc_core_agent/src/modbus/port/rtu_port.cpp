#include "core_agent/core_agent.h"

#include <core_agent/modbus/modbus_packet.h>
#include <core_agent/modbus/port/rtu_port.h>

using namespace NaviFra;

RTUPort::RTUPort(boost::asio::io_context& io_context, const std::string& device, unsigned int baud_rate)
    : io_context_(io_context)
    , serial_port_(io_context)
    , device_(device)
    , baud_rate_(baud_rate)
{
}

void RTUPort::connect()
{
    boost::system::error_code ec;
    serial_port_.open(device_, ec);
    if (ec) {
        throw ModbusException("Failed to open RTU port: " + ec.message());
    }
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
}

void RTUPort::disconnect()
{
    if (serial_port_.is_open()) {
        serial_port_.close();
    }
}

bool RTUPort::isOpen() const
{
    return serial_port_.is_open();
}

void RTUPort::sendFrame(const std::vector<uint8_t>& pdu, std::function<void()> onWriteComplete)
{
    // CRC 추가
    std::vector<uint8_t> frame = pdu;
    uint16_t crc = ModbusPacket::calculateCRC(pdu);
    frame.push_back(crc & 0xFF);  // CRC 하위 바이트
    frame.push_back((crc >> 8) & 0xFF);  // CRC 상위 바이트

    asyncWrite(frame, onWriteComplete);
}

void RTUPort::asyncRead(std::size_t length, std::function<void(const std::vector<uint8_t>&)> onReceive)
{
    auto buffer = std::make_shared<std::vector<uint8_t>>(256);  // 임시 버퍼
    auto read_data = std::make_shared<std::vector<uint8_t>>();  // 전체 패킷을 저장할 버퍼

    auto read_handler = std::make_shared<std::function<void(boost::system::error_code, std::size_t)>>();

    // read_handler 정의 (read_handler 자체도 캡처 목록에 추가)
    *read_handler = [this, buffer, read_data, onReceive, read_handler](
                        boost::system::error_code ec, std::size_t bytes_transferred) mutable {
        if (!ec) {
            // 새로 읽은 데이터를 read_data에 추가
            read_data->insert(read_data->end(), buffer->begin(), buffer->begin() + bytes_transferred);

            // 최소 길이 확인 (기본 PDU 헤더 + 데이터 최소 길이)
            if (read_data->size() >= 5) {
                // 전체 패킷 길이 예측
                std::size_t expected_length = calculateExpectedLengthFromPartial(*read_data);

                // 전체 패킷이 완성되었는지 확인
                if (read_data->size() >= expected_length) {
                    // CRC 검증
                    if (!ModbusPacket::verifyCRC(*read_data)) {
                        throw std::runtime_error("CRC validation failed for RTU packet");
                    }

                    // PDU 부분 추출 (CRC와 헤더 제외)
                    std::vector<uint8_t> pdu(read_data->begin() + 1, read_data->end() - 2);

                    // 완료된 패킷 처리 - PDU만 전달
                    onReceive(pdu);
                    return;  // 처리 완료 후 핸들러 종료
                }
            }

            // 패킷이 아직 완성되지 않으면 다시 읽기
            serial_port_.async_read_some(boost::asio::buffer(*buffer),
                                         *read_handler);  // 재귀적으로 read_handler 호출
        }
        else {
            throw std::runtime_error("Error during async read_some: " + ec.message());
        }
    };

    // 첫 번째 async_read_some 호출
    serial_port_.async_read_some(boost::asio::buffer(*buffer), *read_handler);
}

void RTUPort::asyncWrite(const std::vector<uint8_t>& data, std::function<void()> on_write)
{
    boost::asio::async_write(serial_port_, boost::asio::buffer(data), [on_write](boost::system::error_code ec, std::size_t) {
        if (ec) {
            throw ModbusException("RTU write failed: " + ec.message());
        }
        else {
            on_write();
        }
    });
}
std::size_t RTUPort::calculateExpectedLengthFromPartial(const std::vector<uint8_t>& data)
{
    if (data.size() < 2) {
        return 4;  // 주소(1) + 기능 코드(1) + CRC(2)
    }

    uint8_t functionCode = data[1];

    switch (functionCode) {
        case 0x01:  // Read Coils
        case 0x02:  // Read Discrete Inputs
        case 0x03:  // Read Holding Registers
        case 0x04:  // Read Input Registers
            if (data.size() >= 3) {
                uint8_t byteCount = data[2];
                return 3 + byteCount + 2;  // 주소(1) + 기능 코드(1) + 데이터 + CRC(2)
            }
            break;

        case 0x05:  // Write Single Coil
        case 0x06:  // Write Single Register
        case 0x0F:  // Write Multiple Coils
        case 0x10:  // Write Multiple Registers
            return 8;  // 주소(1) + 기능 코드(1) + 데이터(4) + CRC(2)

        default:
            throw std::runtime_error("Unknown function code");
    }

    return 256;  // 최대 패킷 길이로 설정 (필요시 조정)
}
